/*
 * Copyright 2026 Stanford University
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __TRANSPOSE__
#define __TRANSPOSE__

#include <nvhls_module.h>
#include <systemc.h>

#include "AxiSpec.h"
#include "GBSpec.h"
#include "Spec.h"

// ============================================================
// TransposeConfig
//
// Packed into a 128-bit AXI data word at local_index 0x01:
//   bits [ 0]     : is_valid
//   bits [10:8]   : memory_index_src  (source memory manager)
//   bits [18:16]  : memory_index_dst  (destination memory manager)
//   bits [39:32]  : num_rows   (rows in the source matrix)
//   bits [47:40]  : num_cols   (VectorType columns per row in source)
//   bits [50:48]  : opcode     0 = naive (original), 1 = banked BRAM (fill/drain)
// ============================================================
class TransposeConfig {
  static const int write_width = 128;

 public:
  NVUINT1 is_valid;
  NVUINT3 memory_index_src;
  NVUINT3 memory_index_dst;
  NVUINT8 num_rows;
  NVUINT8 num_cols;
  NVUINT3 opcode;  // 0 = naive, 1 = banked BRAM (fill/drain)

  // Runtime counters – not part of the AXI config register
  NVUINT8 row_counter;
  NVUINT8 col_counter;

  void Reset() {
    is_valid         = 0;
    memory_index_src = 0;
    memory_index_dst = 0;
    num_rows         = 1;
    num_cols         = 1;
    opcode           = 0;
    row_counter      = 0;
    col_counter      = 0;
  }

  void ConfigWrite(const NVUINT16 idx, const NVUINTW(write_width) & data) {
    if (idx == 0x01) {
      is_valid         = nvhls::get_slc<1>(data, 0);
      memory_index_src = nvhls::get_slc<3>(data, 8);
      memory_index_dst = nvhls::get_slc<3>(data, 16);
      num_rows         = nvhls::get_slc<8>(data, 32);
      num_cols         = nvhls::get_slc<8>(data, 40);
      opcode           = nvhls::get_slc<3>(data, 48);
    }
  }

  void ConfigRead(const NVUINT16 idx, NVUINTW(write_width) & data) const {
    data = 0;
    if (idx == 0x01) {
      data.set_slc<1>(0, is_valid);
      data.set_slc<3>(8, memory_index_src);
      data.set_slc<3>(16, memory_index_dst);
      data.set_slc<8>(32, num_rows);
      data.set_slc<8>(40, num_cols);
      data.set_slc<3>(48, opcode);
    }
  }

  void ResetCounters() {
    row_counter = 0;
    col_counter = 0;
  }

  // Advance to the next (row, col) element (column-major inner loop).
  // Returns true when every element has been processed.
  bool Advance() {
    if (col_counter < num_cols - 1) {
      col_counter++;
      return false;
    } else {
      col_counter = 0;
      if (row_counter < num_rows - 1) {
        row_counter++;
        return false;
      } else {
        row_counter = 0;
        return true;
      }
    }
  }
};

// ============================================================
// Transpose Module
//
// Opcode 0 (naive): Read A[row][col], write A^T[col][row] element-by-element.
//   FSM: READ → WAIT_RSP → WRITE → NEXT  (4 states per element)
//
// Opcode 1 (banked BRAM, pipelined):
//   Uses kNumBramBanks local BRAM banks with diagonal banking:
//     A[r][c] → bank (r+c) % kNumBramBanks, address r.
//
//   Fill phase:  read from GBCore SRAM into local BRAMs.
//     FILL_READ → FILL_STORE  (2 states per element)
//
//   Drain phase: read from BRAMs in transposed order, write to dest.
//     DRAIN_FIRST (1 setup state) → DRAIN_PIPE (1 state per element)
//     DRAIN_PIPE pushes the write for the current element while
//     pre-reading the next element from BRAM — these are independent
//     resources so they execute in parallel within II=3.
//
//   Total: ~3 states/element vs naive's 4 → ~1.33x speedup.
//
// Matrix layout in GBCore SRAM (via DataReq addressing):
//   Element A[r][c]  →  (memory_index=src, timestep=r, vector=c)
//   Element A^T[c][r] → (memory_index=dst, timestep=c, vector=r)
//
// AXI address region: 0x6, local_index 0x01. Start: region 0x0, local_index 0x3.
// ============================================================

static const unsigned int kNumBramBanks = 32;  // max(R, C) must be ≤ this
static const unsigned int kBramBankDepth = 32;  // max(R, C) must be ≤ this

class Transpose : public match::Module {
  static const int kDebugLevel = 3;
  SC_HAS_PROCESS(Transpose);

 public:
  // AXI config interface
  Connections::In<spec::Axi::SubordinateToRVA::Write>  rva_in;
  Connections::Out<spec::Axi::SubordinateToRVA::Read>  rva_out;

  // Start / done handshake
  Connections::In<bool>  start;
  Connections::Out<bool> done;

  // GBCore large-buffer interface
  Connections::Out<spec::GB::Large::DataReq>    large_req;
  Connections::In<spec::GB::Large::DataRsp<1>>  large_rsp;

  // --------------------------------------------------------
  // FSM: naive path  (READ, WAIT_RSP, WRITE, NEXT, FIN)
  //      banked-BRAM (FILL_READ, FILL_STORE,
  //                   DRAIN_FIRST, DRAIN_PIPE)
  // --------------------------------------------------------
  enum FSM {
    IDLE,
    READ, WAIT_RSP, WRITE, NEXT, FIN,
    FILL_READ, FILL_STORE,
    DRAIN_FIRST, DRAIN_PIPE
  };
  FSM state;

  bool is_start;
  TransposeConfig config;

  bool w_axi_rsp;
  spec::Axi::SubordinateToRVA::Read rva_out_reg;
  spec::GB::Large::WordType         read_data;

  // Local BRAM for opcode-1 transpose.
  // Stored as flat NVUINTW(128) so Catapult maps to a single wide BRAM
  // instead of decomposing nv_scvector into 16 separate 8-bit sub-memories.
  static const unsigned int kBramSize = kNumBramBanks * kBramBankDepth;
  static const unsigned int kWordWidth = spec::kVectorSize * spec::kIntWordWidth;
  NVUINTW(kWordWidth) bram_flat[kBramSize];

  Transpose(sc_module_name nm)
      : match::Module(nm),
        rva_in("rva_in"),
        rva_out("rva_out"),
        start("start"),
        done("done"),
        large_req("large_req"),
        large_rsp("large_rsp") {
    SC_THREAD(Run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void Reset() {
    state    = IDLE;
    is_start = 0;
    config.Reset();
    rva_in.Reset();
    rva_out.Reset();
    start.Reset();
    done.Reset();
    large_req.Reset();
    large_rsp.Reset();
  }

  // Diagonal banking: A[r][c] → flat BRAM address.
  // bank = (r+c) % kNumBramBanks, flat addr = bank * kBramBankDepth + r.
  NVUINT16 BramAddr(NVUINT8 row, NVUINT8 col) const {
    NVUINT8 bank = (NVUINT8)((row + col) % kNumBramBanks);
    return (NVUINT16)(bank * kBramBankDepth + row);
  }

  // Pack nv_scvector<uint8,16> → flat NVUINTW(128) for BRAM storage.
  NVUINTW(kWordWidth) PackWord(const spec::GB::Large::WordType& vec) const {
    NVUINTW(kWordWidth) result = 0;
    #pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      result.set_slc(i * spec::kIntWordWidth, (NVUINTW(spec::kIntWordWidth))vec[i]);
    }
    return result;
  }

  // Unpack flat NVUINTW(128) → nv_scvector<uint8,16>.
  spec::GB::Large::WordType UnpackWord(const NVUINTW(kWordWidth)& val) const {
    spec::GB::Large::WordType result;
    #pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      result[i] = nvhls::get_slc<spec::kIntWordWidth>(val, i * spec::kIntWordWidth);
    }
    return result;
  }

  void Initialize() { w_axi_rsp = 0; }

  void DecodeAxi() {
    spec::Axi::SubordinateToRVA::Write rva_reg;
    if (rva_in.PopNB(rva_reg)) {
      NVUINT4  tmp         = nvhls::get_slc<4>(rva_reg.addr, 20);
      NVUINT16 local_index = nvhls::get_slc<16>(rva_reg.addr, 4);
      if (tmp == 0x6) {
        if (rva_reg.rw) {
          config.ConfigWrite(local_index, rva_reg.data);
        } else {
          config.ConfigRead(local_index, rva_out_reg.data);
          w_axi_rsp = 1;
        }
      }
    }
  }

  void CheckStart() {
    bool s;
    if (start.PopNB(s)) {
      is_start = config.is_valid && s;
      if (is_start) config.ResetCounters();
      CDCOUT(sc_time_stamp() << name() << " Transpose Start!" << endl,
             kDebugLevel);
    }
  }

  void RunFSM() {
    spec::GB::Large::DataReq req;
    switch (state) {

      case IDLE: break;

      case READ: {
        // Naive: read A[row_counter][col_counter] from src
        req.is_write       = 0;
        req.memory_index   = config.memory_index_src;
        req.vector_index   = config.col_counter;
        req.timestep_index = config.row_counter;
        req.write_data     = 0;
        large_req.Push(req);
        break;
      }

      case WAIT_RSP: {
        spec::GB::Large::DataRsp<1> rsp = large_rsp.Pop();
        read_data                       = rsp.read_vector[0];
        break;
      }

      case WRITE: {
        // Naive: write A^T[col_counter][row_counter] to dst
        req.is_write       = 1;
        req.memory_index   = config.memory_index_dst;
        req.vector_index   = config.row_counter;
        req.timestep_index = config.col_counter;
        req.write_data     = read_data;
        large_req.Push(req);
        break;
      }

      case NEXT: break;

      case FIN: {
        is_start = 0;
        done.Push(1);
        break;
      }

      // --- Banked-BRAM path: fill from src, then drain to dst ---

      case FILL_READ: {
        req.is_write       = 0;
        req.memory_index   = config.memory_index_src;
        req.vector_index   = config.col_counter;
        req.timestep_index = config.row_counter;
        req.write_data     = 0;
        large_req.Push(req);
        break;
      }

      case FILL_STORE: {
        spec::GB::Large::DataRsp<1> rsp = large_rsp.Pop();
        NVUINT16 addr = BramAddr(config.row_counter, config.col_counter);
        bram_flat[addr] = PackWord(rsp.read_vector[0]);
        break;
      }

      case DRAIN_FIRST: {
        NVUINT16 addr = BramAddr(config.row_counter, config.col_counter);
        read_data = UnpackWord(bram_flat[addr]);
        break;
      }

      case DRAIN_PIPE: {
        // Push SRAM write using data pre-read in previous iteration
        req.is_write       = 1;
        req.memory_index   = config.memory_index_dst;
        req.vector_index   = config.row_counter;
        req.timestep_index = config.col_counter;
        req.write_data     = read_data;
        large_req.Push(req);

        // Speculatively pre-read BRAM for the NEXT element (parallel
        // with Push above — independent resources, no data dependency)
        NVUINT8 next_col = config.col_counter;
        NVUINT8 next_row = config.row_counter;
        if (next_col < (NVUINT8)(config.num_cols - 1)) {
          next_col++;
        } else {
          next_col = 0;
          next_row++;
        }
        NVUINT16 next_addr = BramAddr(next_row, next_col);
        read_data = UnpackWord(bram_flat[next_addr]);
        break;
      }

      default: break;
    }
  }

  void UpdateFSM() {
    FSM next;
    switch (state) {
      case IDLE:
        next = is_start ? (config.opcode == 0 ? READ : FILL_READ) : IDLE;
        break;
      case READ:     next = WAIT_RSP; break;
      case WAIT_RSP: next = WRITE;   break;
      case WRITE:    next = NEXT;    break;
      case NEXT: {
        bool all_done = config.Advance();
        next          = all_done ? FIN : READ;
        break;
      }
      case FIN: next = IDLE; break;

      case FILL_READ: next = FILL_STORE; break;
      case FILL_STORE: {
        bool all_read = config.Advance();
        next = all_read ? DRAIN_FIRST : FILL_READ;
        break;
      }
      case DRAIN_FIRST: next = DRAIN_PIPE; break;
      case DRAIN_PIPE: {
        bool all_written = config.Advance();
        next = all_written ? FIN : DRAIN_PIPE;
        break;
      }

      default: next = IDLE; break;
    }
    state = next;
  }

  void Run() {
    Reset();
#pragma hls_pipeline_init_interval 3
    while (1) {
      Initialize();
      RunFSM();
      if (!is_start) {
        DecodeAxi();
        if (w_axi_rsp) rva_out.Push(rva_out_reg);
        CheckStart();
      }
      UpdateFSM();
      wait();
    }
  }
};

#endif  // __TRANSPOSE__
