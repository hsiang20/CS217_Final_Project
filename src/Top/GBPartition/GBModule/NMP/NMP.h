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

#ifndef __NMP__
#define __NMP__

#include <ac_math.h>
#include <nvhls_module.h>
#include <systemc.h>

#include "NMPSpec.h"

/**
 * @brief NMP module definition performing RMSNorm or Softmax.
 */
class NMP : public match::Module {
  static const int kDebugLevel = 3;
  SC_HAS_PROCESS(NMP);

public:
  // ===========================================================================
  // External Interfaces
  // ===========================================================================
  // AXI interface for configuration
  Connections::In<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Out<spec::Axi::SubordinateToRVA::Read> rva_out;

  // Start/done handshake
  Connections::In<bool> start;
  Connections::Out<bool> done;

  // GB large-buffer interfaces
  Connections::Out<spec::GB::Large::DataReq> large_req;
  Connections::In<spec::GB::Large::DataRsp<1>> large_rsp;

  // ===========================================================================
  // FSM and Control State
  // ===========================================================================
  enum FSM {
    IDLE,
    PRE,
    READ,
    RMS_SUMSQ,    // RMSNorm: compute sum of squares
    RMS_SQRT,     // RMSNorm: compute sqrt and reciprocal
    RMS_NORM,     // RMSNorm: apply normalization
    SOFTMAX_MAX,  // Softmax: find maximum value
    SOFTMAX_EXP,  // Softmax: compute exp(x - max)
    SOFTMAX_SUM,  // Softmax: compute reciprocal of sum
    SOFTMAX_NORM, // Softmax: normalize by reciprocal
    WRITE,
    NEXT,
    FIN
  };
  FSM state, next_state;

  /** Start latch once configuration and start pulse are present */
  bool is_start;
  /** Configuration registers and counters */
  spec::NMP::NMPConfig nmp_config;

  /** Pending AXI response flag and register */
  bool w_axi_rsp;
  /** Latched AXI read response */
  spec::Axi::SubordinateToRVA::Read rva_out_reg;
  /** Done pulse flag */
  bool w_done;

  /** Latched GB large-buffer response */
  spec::GB::Large::DataRsp<1> large_rsp_reg;
  /** Prepared GB large-buffer request */
  spec::GB::Large::DataReq large_req_reg;
  /** Outgoing vector payload after computation */
  spec::VectorType write_data;

  /** opcode mapping: 0 -> RMSNorm, 1 -> Softmax */
  NVUINT1 op_softmax;

  // ===========================================================================
  // Fixed-point computation state
  // ===========================================================================
  /** Input vector in fixed-point format */
  spec::NMP::FixedType input_fixed[spec::kVectorSize];
  /** Output vector in fixed-point format */
  spec::NMP::FixedType output_fixed[spec::kVectorSize];
  /** Exponential results for softmax */
  spec::NMP::UnsignedFixedType exp_values[spec::kVectorSize];
  /** Maximum value for stable softmax */
  spec::NMP::FixedType max_value;
  /** Sum of exponentials for softmax normalization */
  spec::NMP::UnsignedAccumType sum_exp;
  /** Reciprocal of sum for division */
  spec::NMP::AccumType sum_exp_reciprocal;
  /** Sum of squares for RMSNorm */
  spec::NMP::AccumType sum_sq;
  /** RMS reciprocal for normalization */
  spec::NMP::AccumType rms_reciprocal;

  // ===========================================================================
  // Constructor / Reset / Initialization
  // ===========================================================================

  /** Constructor */
  NMP(sc_module_name nm) :
      match::Module(nm),
      rva_in("rva_in"),
      rva_out("rva_out"),
      start("start"),
      done("done"),
      large_req("large_req"),
      large_rsp("large_rsp") {
    SC_THREAD(NMPRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  } // NMP

  /** Master reset */
  void Reset() {
    state     = IDLE;
    is_start  = 0;
    w_axi_rsp = 0;
    w_done    = 0;
    nmp_config.Reset();
    ResetPorts();
    ResetCompute();
  } // Reset

  /** Reset computation state */
  void ResetCompute() {
    max_value          = spec::kAttentionWordMin;
    sum_exp            = 0;
    sum_exp_reciprocal = 0;
    sum_sq             = 0;
    rms_reciprocal     = 0;
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      input_fixed[i]  = 0;
      output_fixed[i] = 0;
      exp_values[i]   = 0;
    }
  } // ResetCompute

  /** Reset handshake interfaces */
  void ResetPorts() {
    rva_in.Reset();
    rva_out.Reset();
    start.Reset();
    done.Reset();
    large_req.Reset();
    large_rsp.Reset();
  } // ResetPorts

  // ===========================================================================
  // AXI Interface Handling
  // ===========================================================================
  /** Pop and decode an AXI transaction - called from main thread */
  void DecodeAxiWrite(const spec::Axi::SubordinateToRVA::Write& rva_in_reg) {
    NVUINT4 tmp          = nvhls::get_slc<4>(rva_in_reg.addr, 20);
    NVUINT16 local_index = nvhls::get_slc<16>(rva_in_reg.addr, 4);
    if (tmp == 0xC) {
      nmp_config.ConfigWrite(local_index, rva_in_reg.data);
    }
  } // DecodeAxiWrite

  /** Decode AXI read transaction and prepare response */
  void DecodeAxiRead(const spec::Axi::SubordinateToRVA::Write& rva_in_reg) {
    NVUINT4 tmp          = nvhls::get_slc<4>(rva_in_reg.addr, 20);
    NVUINT16 local_index = nvhls::get_slc<16>(rva_in_reg.addr, 4);
    w_axi_rsp            = 1;
    if (tmp == 0xC) {
      nmp_config.ConfigRead(local_index, rva_out_reg.data);
    }
  } // DecodeAxiRead

  // ===========================================================================
  // GB Request Preparation
  // ===========================================================================
  /** Prepare GB large-buffer read request */
  void PrepareReadReq() {
    large_req_reg.is_write       = 0;
    large_req_reg.memory_index   = nmp_config.memory_index_1;
    large_req_reg.vector_index   = nmp_config.GetVectorIndex();
    large_req_reg.timestep_index = nmp_config.GetTimestepIndex();
    large_req.Push(large_req_reg);
  } // PrepareReadReq

  /** Prepare GB large-buffer write request */
  void PrepareWriteReq() {
    large_req_reg.is_write       = 1;
    large_req_reg.memory_index   = nmp_config.memory_index_1;
    large_req_reg.vector_index   = nmp_config.GetVectorIndex();
    large_req_reg.timestep_index = nmp_config.GetTimestepIndex();
    large_req_reg.write_data     = write_data;
    large_req.Push(large_req_reg);
  } // PrepareWriteReq

  // ===========================================================================
  // Data Conversion Functions
  // Note that the I/O data is in int8 format, but computation is done
  // in fixed-point format.
  // ===========================================================================

  /**
   * @brief Convert input int vector to fixed-point format for computation.
   */
  void ConvertInputToFixed() {
    spec::ScalarType inputTmp = 0;
    spec::NMP::InputFixedType in_fixed = 0;
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      inputTmp = large_rsp_reg.read_vector[0][i];
      NVINTW(spec::kIntWordWidth) signed_input = (NVINTW(spec::kIntWordWidth))inputTmp;
      in_fixed.set_slc(0, signed_input);
      input_fixed[i] = ConvertFromNmpInputType(in_fixed);
      //cout << "input tmp, in_fixed, input_fixed: " << inputTmp << " " << in_fixed << " " << input_fixed[i] << endl;
    }
  } // ConvertInputToFixed

  /**
   * @brief Convert fixed-point output back to int for write
   */
  void ConvertOutputToInt() {
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      spec::NMP::FixedType out_fixed = output_fixed[i];
      spec::NMP::InputFixedType out_tmp = ConvertToNmpOutputType(out_fixed);
      write_data[i] = nvhls::get_slc<spec::kIntWordWidth>(out_tmp, 0);
      //cout << "out_fixed, out_tmp, write_data: " << out_fixed << " " << out_tmp << " " << write_data[i] << endl;
    }
  } // ConvertOutputToInt


  /** RMSNorm Step 1 */
  void ComputeRMSSumSq() {

    sum_sq = 0;
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      spec::NMP::AccumType sq = input_fixed[i] * input_fixed[i];
      sum_sq += sq;
    }

  } // ComputeRMSSumSq

  /** RMSNorm Step 2 */
  void ComputeRMSSqrtRecip() {

    // mean = sum_sq / kVectorSize
    spec::NMP::UnsignedAccumType rms_sqrt;
    spec::NMP::UnsignedAccumType mean =
        sum_sq * spec::NMP::kInvVectorSize + spec::NMP::kEpsilon;
    ac_math::ac_sqrt_pwl(mean, rms_sqrt);

    // reciprocal: 1 / sqrt(mean + epsilon)
    ac_math::ac_reciprocal_pwl(rms_sqrt, rms_reciprocal);

  } // ComputeRMSSqrtRecip

  /** RMSNorm Step 3 */
  void ComputeRMSNormalize() {

#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      output_fixed[i] = input_fixed[i] * rms_reciprocal;
    }

  } // ComputeRMSNormalize

  /** Softmax Step 1 */
  void ComputeSoftmaxMax() {
    max_value = spec::kAttentionWordMin;

#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      if (input_fixed[i] > max_value) {
        max_value = input_fixed[i];
      }
    }

  } // ComputeSoftmaxMax

  /** Softmax Step 2 */
  void ComputeSoftmaxExp() {

    // Shifted input for numerical stability
    spec::NMP::FixedType shifted[spec::kVectorSize];

// Subtract max for numerical stability
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      shifted[i] = input_fixed[i] - max_value;
    }

    // Compute exponential using piecewise-linear approximation
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      exp_values[i] =
          ac_math::ac_exp_pwl<spec::NMP::UnsignedFixedType>(shifted[i]);
    }


  } // ComputeSoftmaxExp

  /** Softmax Step 3 */
  void ComputeSoftmaxSum() {
    // Initialize sum of exponentials
    sum_exp = 0;

    // Accumulate sum of exponential
#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      sum_exp += exp_values[i];
    }
    ac_math::ac_reciprocal_pwl(sum_exp, sum_exp_reciprocal);

  } // ComputeSoftmaxSum

  /** Softmax Step 4 */
  void ComputeSoftmaxNormalize() {

#pragma hls_unroll yes
    for (int i = 0; i < spec::kVectorSize; i++) {
      output_fixed[i] = exp_values[i] * sum_exp_reciprocal;
    }

  } // ComputeSoftmaxNormalize

  // ===========================================================================
  // Finite State Machine Functions
  // ===========================================================================

  // Run FSM operations for current state
  void RunFSM() {
    switch (state) {
      // Reset computation state when idle
      case IDLE: ResetCompute(); break;
      // Prepare read request
      case PRE: PrepareReadReq(); break;
      // Data reception handled in UpdateFSM
      case READ: break;

      // RMSNorm path
      case RMS_SUMSQ: ComputeRMSSumSq(); break;
      case RMS_SQRT: ComputeRMSSqrtRecip(); break;
      case RMS_NORM:
        ComputeRMSNormalize();
        ConvertOutputToInt();
        break;

      // Softmax path
      case SOFTMAX_MAX: ComputeSoftmaxMax(); break;
      case SOFTMAX_EXP: ComputeSoftmaxExp(); break;
      case SOFTMAX_SUM: ComputeSoftmaxSum(); break;
      case SOFTMAX_NORM:
        ComputeSoftmaxNormalize();
        ConvertOutputToInt();
        break;

      // Write results back to GBCore
      case WRITE: PrepareWriteReq(); break;
      // Update counters and check for completion in UpdateFSM
      case NEXT: break;
      // Finish operation and reset start latch
      case FIN:
        is_start = 0;
        w_done   = 1;
        break;

      // Default case (should not occur)
      default: break;
    }
  } // RunFSM

  // Update FSM state based on current state and inputs
  void UpdateFSM() {
    switch (state) {
      // Check start signal only in IDLE state
      case IDLE: {
        bool start_reg;
        if (start.PopNB(start_reg)) {
          is_start = nmp_config.is_valid && start_reg;
          CDCOUT(
              sc_time_stamp() << name() << " NMP Start !!!" << endl,
              kDebugLevel);
        }
        if (is_start) {
          nmp_config.ResetCounter();
          op_softmax = (nmp_config.mode == 1);
          next_state = PRE;
        } else {
          next_state = IDLE;
        }
        break;
      } // IDLE

      // Next state is always READ
      case PRE: next_state = READ; break;

      // Wait for data, then branch based on operation mode
      case READ: {
        spec::GB::Large::DataRsp<1> data_rsp;
        if (large_rsp.PopNB(data_rsp)) {
          large_rsp_reg = data_rsp;
          ConvertInputToFixed();
          next_state = op_softmax ? SOFTMAX_MAX : RMS_SUMSQ;
        } else {
          // Keep waiting for data
          next_state = READ;
        }
        break;
      } // READ

      // RMSNorm pipeline
      case RMS_SUMSQ: next_state = RMS_SQRT; break;
      case RMS_SQRT: next_state = RMS_NORM; break;
      case RMS_NORM: next_state = WRITE; break;

      // Softmax pipeline
      case SOFTMAX_MAX: next_state = SOFTMAX_EXP; break;
      case SOFTMAX_EXP: next_state = SOFTMAX_SUM; break;
      case SOFTMAX_SUM: next_state = SOFTMAX_NORM; break;
      case SOFTMAX_NORM: next_state = WRITE; break;

      case WRITE: next_state = NEXT; break;
      case NEXT: {
        // Wires for end conditions
        bool vec_end = 0, time_end = 0;
        // Update counters and check for end conditions
        nmp_config.UpdateVectorCounter(vec_end);
        if (vec_end) {
          nmp_config.UpdateTimestepCounter(time_end);
        }
        // Decide next state based on end conditions
        next_state = (vec_end && time_end) ? FIN : PRE;
        break;
      } // NEXT

      // Finish and return to IDLE
      case FIN: next_state = IDLE; break;
      // Default case (should not occur)
      default: next_state = IDLE; break;
    } // switch

    // Update state register
    state = next_state;
  } // UpdateFSM

  // ===========================================================================
  // Main Thread
  // ===========================================================================
  void NMPRun() {
    Reset();
#pragma hls_pipeline_init_interval 1
    while (1) {
      // Clear per-cycle signals
      w_axi_rsp = 0;
      w_done    = 0;

      // Decode AXI requests with highest priority
      spec::Axi::SubordinateToRVA::Write rva_in_reg;
      if (rva_in.PopNB(rva_in_reg)) {
        CDCOUT(
            sc_time_stamp() << name() << " NMP RVA Pop " << endl, kDebugLevel);
        if (rva_in_reg.rw) {
          DecodeAxiWrite(rva_in_reg);
        } else {
          DecodeAxiRead(rva_in_reg);
        }
      } else {
        // Only run FSM when no AXI request is pending
        RunFSM();
        UpdateFSM();
      }

      // Push AXI response if generated
      if (w_axi_rsp) {
        rva_out.Push(rva_out_reg);
      }
      // Push done signal if generated
      if (w_done)
        done.Push(1);
      wait();
    } // while
  } // NMPRun
}; // NMP

#endif
