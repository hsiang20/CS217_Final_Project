// Copyright 2026 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __NMPSPEC__
#define __NMPSPEC__

#include "AxiSpec.h"
#include "GBSpec.h"

#include <nvhls_int.h>
#include <nvhls_types.h>
#include <nvhls_vector.h>


namespace spec {
  namespace NMP {
    // AC fixed-point types for NMP computations
    const int kNmpInputNumFrac = 4;
    typedef ac_fixed<kIntWordWidth, kIntWordWidth - kNmpInputNumFrac, true, AC_TRN, AC_WRAP>
        InputFixedType;
    typedef ac_fixed<32, 16, true, AC_RND, AC_SAT> FloatToFixedTmp;
    typedef ac_fixed<kAttentionWordWidth, kAttentionNumInt, true, AC_TRN, AC_WRAP>
        FixedType;
    typedef ac_fixed<kAttentionWordWidth, kAttentionNumInt, false, AC_TRN, AC_WRAP>
        UnsignedFixedType;
    typedef ac_fixed<
        kAttentionWordWidth + 8,
        kAttentionNumInt + 4,
        true,
        AC_TRN,
        AC_WRAP>
        AccumType;
    typedef ac_fixed<
        kAttentionWordWidth + 8,
        kAttentionNumInt + 4,
        false,
        AC_TRN,
        AC_WRAP>
        UnsignedAccumType;

    // Vector definition as Matchlib vectors of size kVectorSize
    typedef nvhls::nv_scvector<FixedType, kVectorSize> VectorType;
    typedef nvhls::nv_scvector<UnsignedFixedType, kVectorSize>
        UnsignedVectorType;
    typedef nvhls::nv_scvector<AccumType, kVectorSize> AccumVectorType;
    typedef nvhls::nv_scvector<UnsignedAccumType, kVectorSize>
        UnsignedAccumVectorType;

    // Inverse of vector size for averaging
    const UnsignedAccumType kInvVectorSize = 1.0f / kVectorSize;
    // Epsilon value to avoid division by zero in RMSNorm
    const UnsignedAccumType kEpsilon = 1e-4f;

    /**
     * @brief Configuration registers for the NMP block.
     *
     * Layout matches the AXI write/read payload used by the original
     * GBControlConfig but only keeps the fields consumed by NMP.
     */
    class NMPConfig : public nvhls_message {
      static const int write_width = 128;

    public:
      NVUINT1 is_valid;
      NVUINT3 mode;           // 0: RMSNorm, 1: Softmax
      NVUINT3 memory_index_1; // target large-buffer index
      NVUINT8 num_vector_1;
      NVUINT16 num_timestep_1;

      NVUINT8 vector_counter;
      NVUINT16 timestep_counter;

      template <unsigned int Size>
      void Marshall(Marshaller<Size>& m) {
        m & is_valid;
        m & mode;
        m & memory_index_1;
        m & num_vector_1;
        m & num_timestep_1;
        m & vector_counter;
        m & timestep_counter;
      }

      void Reset() {
        is_valid       = 0;
        mode           = 0;
        memory_index_1 = 0;
        num_vector_1   = 1;
        num_timestep_1 = 1;
        ResetCounter();
      }

      void ResetCounter() {
        vector_counter   = 0;
        timestep_counter = 0;
      }

      NVUINT8 GetVectorIndex() const { return vector_counter; }

      NVUINT16 GetTimestepIndex() const { return timestep_counter; }

      void UpdateVectorCounter(bool& is_end) {
        is_end = 0;
        if (vector_counter >= (num_vector_1 - 1)) {
          is_end         = 1;
          vector_counter = 0;
        } else {
          vector_counter += 1;
        }
      }

      void UpdateTimestepCounter(bool& is_end) {
        is_end = 0;
        if (timestep_counter >= (num_timestep_1 - 1)) {
          is_end           = 1;
          timestep_counter = 0;
        } else {
          timestep_counter += 1;
        }
      }

      void ConfigWrite(
          const NVUINT16 write_index, const NVUINTW(write_width)& write_data) {
        if (write_index == 0x01) {
          is_valid       = nvhls::get_slc<1>(write_data, 0);
          mode           = nvhls::get_slc<3>(write_data, 8);
          memory_index_1 = nvhls::get_slc<3>(write_data, 32);
          num_vector_1   = nvhls::get_slc<8>(write_data, 48);
          num_timestep_1 = nvhls::get_slc<16>(write_data, 64);
        }
      }

      void ConfigRead(
          const NVUINT16 read_index, NVUINTW(write_width)& read_data) const {
        read_data = 0;
        if (read_index == 0x01) {
          read_data.set_slc<1>(0, is_valid);
          read_data.set_slc<3>(8, mode);
          read_data.set_slc<3>(32, memory_index_1);
          read_data.set_slc<8>(48, num_vector_1);
          read_data.set_slc<16>(64, num_timestep_1);
        }
      }
    };


  } // namespace NMP

} // namespace spec

inline spec::NMP::FixedType ConvertFromNmpInputType(spec::NMP::InputFixedType in) {
  return in;
}

inline spec::NMP::InputFixedType ConvertToNmpOutputType(spec::NMP::FixedType in) {
  return in;
}

#endif
