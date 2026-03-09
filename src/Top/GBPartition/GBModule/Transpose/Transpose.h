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
//   bits [10:8]   : memory_index_src  (K matrix / source memory)
//   bits [18:16]  : memory_index_dst  (result / destination memory)
//   bits [26:24]  : memory_index_q    (Q matrix memory, opcodes 2&3)
//   bits [39:32]  : num_rows   (N = rows of K)
//   bits [47:40]  : num_cols   (D = cols of K = cols of Q)
//   bits [50:48]  : opcode     0=naive, 1=banked BRAM, 2=unfused Q*K^T, 3=fused Q*K^T
//   bits [63:56]  : num_rows_q (M = rows of Q, opcodes 2&3)
// ============================================================
class TransposeConfig {
  static const int write_width = 128;

 public:
  NVUINT1 is_valid;
  NVUINT3 memory_index_src;
  NVUINT3 memory_index_dst;
  NVUINT3 memory_index_q;
  NVUINT8 num_rows;
  NVUINT8 num_cols;
  NVUINT3 opcode;
  NVUINT8 num_rows_q;

  NVUINT8 row_counter;
  NVUINT8 col_counter;

  void Reset() {
    is_valid         = 0;
    memory_index_src = 0;
    memory_index_dst = 0;
    memory_index_q   = 0;
    num_rows         = 1;
    num_cols         = 1;
    opcode           = 0;
    num_rows_q       = 1;
    row_counter      = 0;
    col_counter      = 0;
  }

  void ConfigWrite(const NVUINT16 idx, const NVUINTW(write_width) & data) {
    if (idx == 0x01) {
      is_valid         = nvhls::get_slc<1>(data, 0);
      memory_index_src = nvhls::get_slc<3>(data, 8);
      memory_index_dst = nvhls::get_slc<3>(data, 16);
      memory_index_q   = nvhls::get_slc<3>(data, 24);
      num_rows         = nvhls::get_slc<8>(data, 32);
      num_cols         = nvhls::get_slc<8>(data, 40);
      opcode           = nvhls::get_slc<3>(data, 48);
      num_rows_q       = nvhls::get_slc<8>(data, 56);
    }
  }

  void ConfigRead(const NVUINT16 idx, NVUINTW(write_width) & data) const {
    data = 0;
    if (idx == 0x01) {
      data.set_slc<1>(0, is_valid);
      data.set_slc<3>(8, memory_index_src);
      data.set_slc<3>(16, memory_index_dst);
      data.set_slc<3>(24, memory_index_q);
      data.set_slc<8>(32, num_rows);
      data.set_slc<8>(40, num_cols);
      data.set_slc<3>(48, opcode);
      data.set_slc<8>(56, num_rows_q);
    }
  }

  void ResetCounters() {
    row_counter = 0;
    col_counter = 0;
  }

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
//   FSM: READ -> WAIT_RSP -> WRITE -> NEXT  (4 states per element)
//
// Opcode 1 (banked BRAM, pipelined):
//   Fill phase:  FILL_READ -> FILL_STORE  (2 states per element)
//   Drain phase: DRAIN_FIRST (1 setup) -> DRAIN_PIPE (1 state per element)
//   Total: ~3 states/element vs naive's 4 -> ~1.33x speedup.
//
// Opcode 2 (unfused Q*K^T):
//   Phase 0: Naive transpose K(N*D) -> K^T(D*N) at temp memory 3
//   Phase 1: Matmul Q(M*D) * K^T(D*N) -> result(M*N) at dst
//     For each (i,j): dot product over k reading Q and K^T from SRAM
//
// Opcode 3 (fused Q*K^T):
//   Phase 0: Fill K(N*D) into local BRAM (same as opcode 1 fill)
//   Phase 1: Matmul reading Q from SRAM, K^T from BRAM (no SRAM for K^T)
//     For each row i of Q: read Q[i][k], MAC over all j from BRAM, write results
//
// Matrix layout in GBCore SRAM:
//   K[r][c]  at (memory_index=src, timestep=r, vector=c)
//   Q[i][k]  at (memory_index=q,   timestep=i, vector=k)
//   result[i][j] at (memory_index=dst, timestep=i, vector=j)
// ============================================================

static const unsigned int kNumBramBanks = 32;
static const unsigned int kBramBankDepth = 32;

class Transpose : public match::Module {
  static const int kDebugLevel = 3;
  SC_HAS_PROCESS(Transpose);

 public:
  Connections::In<spec::Axi::SubordinateToRVA::Write>  rva_in;
  Connections::Out<spec::Axi::SubordinateToRVA::Read>  rva_out;

  Connections::In<bool>  start;
  Connections::Out<bool> done;

  Connections::Out<spec::GB::Large::DataReq>    large_req;
  Connections::In<spec::GB::Large::DataRsp<1>>  large_rsp;

  // --------------------------------------------------------
  // FSM states
  // --------------------------------------------------------
  enum FSM {
    IDLE,
    READ, WAIT_RSP, WRITE, NEXT, FIN,
    FILL_READ, FILL_STORE,
    DRAIN_FIRST, DRAIN_PIPE,
    MM_READ_Q, MM_WAIT_Q,
    MM_READ_KT, MM_WAIT_KT,
    MM_MAC,
    MM_WRITE
  };
  FSM state;

  bool is_start;
  TransposeConfig config;

  bool w_axi_rsp;
  spec::Axi::SubordinateToRVA::Read rva_out_reg;
  spec::GB::Large::WordType         read_data;

  // Local BRAM for opcode 1/3
  static const unsigned int kBramSize = kNumBramBanks * kBramBankDepth;
  static const unsigned int kWordWidth = spec::kVectorSize * spec::kIntWordWidth;
  NVUINTW(kWordWidth) bram_flat[kBramSize];

  // Matmul state (opcodes 2 and 3)
  NVUINT8 inner_counter;                // k index for matmul inner dimension
  NVUINT1 phase;                        // 0 = transpose/fill, 1 = matmul
  NVUINT32 accum;                       // single accumulator for unfused dot product
  NVUINT32 accum_arr[kNumBramBanks];    // N accumulators for fused matmul
  spec::GB::Large::WordType q_val;      // cached Q vector from SRAM read

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
    inner_counter = 0;
    phase = 0;
    accum = 0;
  }

  NVUINT16 BramAddr(NVUINT8 row, NVUINT8 col) const {
    NVUINT8 bank = (NVUINT8)((row + col) % kNumBramBanks);
    return (NVUINT16)(bank * kBramBankDepth + row);
  }

  NVUINTW(kWordWidth) PackWord(const spec::GB::Large::WordType& vec) const {
    NVUINTW(kWordWidth) result = 0;
    #pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      result.set_slc(i * spec::kIntWordWidth, (NVUINTW(spec::kIntWordWidth))vec[i]);
    }
    return result;
  }

  spec::GB::Large::WordType UnpackWord(const NVUINTW(kWordWidth)& val) const {
    spec::GB::Large::WordType result;
    #pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      result[i] = nvhls::get_slc<spec::kIntWordWidth>(val, i * spec::kIntWordWidth);
    }
    return result;
  }

  spec::GB::Large::WordType MakeResultVec(NVUINT32 val) const {
    spec::GB::Large::WordType result;
    NVUINT8 lo = nvhls::get_slc<8>(val, 0);
    #pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      result[i] = lo;
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
      if (is_start) {
        config.ResetCounters();
        phase = 0;
        inner_counter = 0;
        accum = 0;
      }
      CDCOUT(sc_time_stamp() << name() << " Transpose Start!" << endl,
             kDebugLevel);
    }
  }

  void RunFSM() {
    spec::GB::Large::DataReq req;
    switch (state) {

      case IDLE: break;

      // ---- Naive transpose path (opcodes 0, 2 phase 0) ----

      case READ: {
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
        req.is_write = 1;
        if (config.opcode == 2 && phase == 0) {
          req.memory_index = 3;
        } else {
          req.memory_index = config.memory_index_dst;
        }
        req.vector_index   = config.row_counter;
        req.timestep_index = config.col_counter;
        req.write_data     = read_data;
        large_req.Push(req);
        break;
      }

      case NEXT: break;

      case FIN: {
        if (config.opcode == 2 && phase == 0) {
          phase = 1;
          config.ResetCounters();
          inner_counter = 0;
          accum = 0;
        } else {
          is_start = 0;
          done.Push(1);
        }
        break;
      }

      // ---- Banked-BRAM path: fill from src, then drain to dst ----

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
        req.is_write       = 1;
        req.memory_index   = config.memory_index_dst;
        req.vector_index   = config.row_counter;
        req.timestep_index = config.col_counter;
        req.write_data     = read_data;
        large_req.Push(req);

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

      // ---- Matmul states (opcodes 2 and 3) ----

      case MM_READ_Q: {
        req.is_write       = 0;
        req.memory_index   = config.memory_index_q;
        req.vector_index   = inner_counter;
        req.timestep_index = config.row_counter;
        req.write_data     = 0;
        large_req.Push(req);
        break;
      }

      case MM_WAIT_Q: {
        spec::GB::Large::DataRsp<1> rsp = large_rsp.Pop();
        q_val = rsp.read_vector[0];
        break;
      }

      case MM_READ_KT: {
        req.is_write       = 0;
        req.memory_index   = 3;
        req.vector_index   = config.col_counter;
        req.timestep_index = inner_counter;
        req.write_data     = 0;
        large_req.Push(req);
        break;
      }

      case MM_WAIT_KT: {
        spec::GB::Large::DataRsp<1> rsp = large_rsp.Pop();
        NVUINT8 kt_scalar = rsp.read_vector[0][0];
        NVUINT8 q_scalar  = q_val[0];
        accum = accum + (NVUINT32)((NVUINT16)(q_scalar) * (NVUINT16)(kt_scalar));
        break;
      }

      case MM_MAC: {
        NVUINT16 baddr = BramAddr(config.col_counter, inner_counter);
        spec::GB::Large::WordType k_vec = UnpackWord(bram_flat[baddr]);
        NVUINT8 k_scalar = k_vec[0];
        NVUINT8 q_scalar = q_val[0];
        NVUINT32 product = (NVUINT32)((NVUINT16)(q_scalar) * (NVUINT16)(k_scalar));
        accum_arr[config.col_counter] = accum_arr[config.col_counter] + product;
        break;
      }

      case MM_WRITE: {
        req.is_write       = 1;
        req.memory_index   = config.memory_index_dst;
        req.timestep_index = config.row_counter;
        req.vector_index   = config.col_counter;
        NVUINT32 acc_val = (config.opcode == 3) ?
            accum_arr[config.col_counter] : accum;
        req.write_data = MakeResultVec(acc_val);
        large_req.Push(req);
        break;
      }

      default: break;
    }
  }

  void UpdateFSM() {
    FSM next;
    switch (state) {
      case IDLE:
        if (is_start) {
          if (config.opcode == 0 || config.opcode == 2) {
            next = READ;
          } else {
            next = FILL_READ;
          }
        } else {
          next = IDLE;
        }
        break;

      case READ:     next = WAIT_RSP; break;
      case WAIT_RSP: next = WRITE;   break;
      case WRITE:    next = NEXT;    break;
      case NEXT: {
        bool all_done = config.Advance();
        next          = all_done ? FIN : READ;
        break;
      }
      case FIN:
        next = is_start ? MM_READ_Q : IDLE;
        break;

      case FILL_READ: next = FILL_STORE; break;
      case FILL_STORE: {
        bool all_read = config.Advance();
        if (all_read) {
          if (config.opcode == 1) {
            next = DRAIN_FIRST;
          } else {
            phase = 1;
            config.ResetCounters();
            inner_counter = 0;
            #pragma hls_unroll yes
            for (int i = 0; i < (int)kNumBramBanks; i++) accum_arr[i] = 0;
            next = MM_READ_Q;
          }
        } else {
          next = FILL_READ;
        }
        break;
      }
      case DRAIN_FIRST: next = DRAIN_PIPE; break;
      case DRAIN_PIPE: {
        bool all_written = config.Advance();
        next = all_written ? FIN : DRAIN_PIPE;
        break;
      }

      // ---- Matmul transitions ----

      case MM_READ_Q: next = MM_WAIT_Q; break;

      case MM_WAIT_Q:
        next = (config.opcode == 3) ? MM_MAC : MM_READ_KT;
        break;

      case MM_READ_KT: next = MM_WAIT_KT; break;

      case MM_WAIT_KT: {
        inner_counter++;
        if (inner_counter < config.num_cols) {
          next = MM_READ_Q;
        } else {
          inner_counter = 0;
          next = MM_WRITE;
        }
        break;
      }

      case MM_MAC: {
        if (config.col_counter < config.num_rows - 1) {
          config.col_counter++;
          next = MM_MAC;
        } else {
          config.col_counter = 0;
          inner_counter++;
          if (inner_counter < config.num_cols) {
            next = MM_READ_Q;
          } else {
            inner_counter = 0;
            next = MM_WRITE;
          }
        }
        break;
      }

      case MM_WRITE: {
        if (config.opcode == 2) {
          // Unfused: wrote one result, advance (i,j)
          if (config.col_counter < config.num_rows - 1) {
            config.col_counter++;
            accum = 0;
            next = MM_READ_Q;
          } else {
            config.col_counter = 0;
            if (config.row_counter < config.num_rows_q - 1) {
              config.row_counter++;
              accum = 0;
              next = MM_READ_Q;
            } else {
              next = FIN;
            }
          }
        } else {
          // Fused: writing N results for current row i
          if (config.col_counter < config.num_rows - 1) {
            config.col_counter++;
            next = MM_WRITE;
          } else {
            config.col_counter = 0;
            if (config.row_counter < config.num_rows_q - 1) {
              config.row_counter++;
              inner_counter = 0;
              #pragma hls_unroll yes
              for (int i = 0; i < (int)kNumBramBanks; i++) accum_arr[i] = 0;
              next = MM_READ_Q;
            } else {
              next = FIN;
            }
          }
        }
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
