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
// ============================================================
class TransposeConfig {
  static const int write_width = 128;

 public:
  NVUINT1 is_valid;
  NVUINT3 memory_index_src;
  NVUINT3 memory_index_dst;
  NVUINT8 num_rows;
  NVUINT8 num_cols;

  // Runtime counters – not part of the AXI config register
  NVUINT8 row_counter;
  NVUINT8 col_counter;

  void Reset() {
    is_valid         = 0;
    memory_index_src = 0;
    memory_index_dst = 0;
    num_rows         = 1;
    num_cols         = 1;
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
// Reads A[row][col] from the src memory manager and writes
// A^T[col][row] to the dst memory manager, iterating over
// every (row, col) pair for the configured matrix dimensions.
//
// Matrix layout in GBCore SRAM (via DataReq addressing):
//   Element A[r][c]  →  (memory_index=src, timestep=r, vector=c)
//   Element A^T[c][r] → (memory_index=dst, timestep=c, vector=r)
//
// GBCore must be pre-configured so that:
//   num_vector_large[src] == num_cols
//   num_vector_large[dst] == num_rows
//
// AXI address region decoded by GBModule:
//   0x6:  config read/write (local_index 0x01)
// Start signal: GBModule pushes it when region=0x0, local_index=0x3
// ============================================================
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
  // FSM
  // --------------------------------------------------------
  enum FSM { IDLE, READ, WAIT_RSP, WRITE, NEXT, FIN };
  FSM state;

  bool is_start;
  TransposeConfig config;

  bool w_axi_rsp;
  spec::Axi::SubordinateToRVA::Read rva_out_reg;
  spec::GB::Large::WordType         read_data;

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
        // Read A[row_counter][col_counter] from src
        req.is_write       = 0;
        req.memory_index   = config.memory_index_src;
        req.vector_index   = config.col_counter;
        req.timestep_index = config.row_counter;
        req.write_data     = 0;
        large_req.Push(req);
        break;
      }

      case WAIT_RSP: {
        // Collect the read response from GBCore
        spec::GB::Large::DataRsp<1> rsp = large_rsp.Pop();
        read_data                       = rsp.read_vector[0];
        break;
      }

      case WRITE: {
        // Write A^T[col_counter][row_counter] to dst
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

      default: break;
    }
  }

  void UpdateFSM() {
    FSM next;
    switch (state) {
      case IDLE:     next = is_start ? READ : IDLE; break;
      case READ:     next = WAIT_RSP;                break;
      case WAIT_RSP: next = WRITE;                   break;
      case WRITE:    next = NEXT;                    break;
      case NEXT: {
        bool all_done = config.Advance();
        next          = all_done ? FIN : READ;
        break;
      }
      case FIN:    next = IDLE; break;
      default:     next = IDLE; break;
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
