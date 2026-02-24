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

#ifndef GBCORE_H
#define GBCORE_H

// SystemC and Matchlib includes
#include <ArbitratedScratchpadDP.h>
#include <nvhls_int.h>
#include <nvhls_module.h>
#include <nvhls_types.h>
#include <nvhls_vector.h>
#include <systemc.h>

// Project includes
#include "AxiSpec.h"
#include "GBSpec.h"


/**
 * @brief Global Buffer Core module definition of unified SRAM scratchpad
 * buffer.
 *
 * This module implements a multibanked SRAM scratchpad for the Global Buffer
 * (GB) with AXI configuration interface and streaming interfaces to external
 * submodules, with an ArbitratedScratchpadDP as the underlying memory to handle
 * concurrent read/write requests from multiple clients.
 *
 * For this simplified implementation of Lab 3, the only external submodule is
 * the NMP (Near Memory Processing) module.
 *
 * The operation of this module is as follows:
 * 1. AXI configuration writes set up base address information for SRAM
 * 2. GBCore polls interface ports for outstanding requests from submodules
 * 3. Upon receiving a request, GBCore maps logical addresses to physical SRAM
 *    addresses and issues read/write commands to the SRAM
 * 4. If there is a read request, the polling functions would also set a
 *    response mode flag to indicate that a read response needs to be sent at
 * the end of the cycle
 * 5. The run() function of SRAM is called to process read/write commands
 * 6. At the end of the cycle, if a read response is pending, GBCore collects
 *    the read data from SRAM and pushes it to the requesting submodule
 */
class GBCore : public match::Module {
  static const int kDebugLevel = 4;
  SC_HAS_PROCESS(GBCore);

  // Response mode for end-of-cycle output push, indicating which interface to
  // respond to
  enum RspMode {
    RSP_NONE     = 0,   // No response this cycle
    RSP_SRAM_CFG = 0x3, // AXI read of SC_SRAM_CONFIG
    RSP_ADDR_CFG = 0x4, // AXI read of address config registers
    RSP_AXI_SRAM = 0x5, // AXI direct SRAM read
    RSP_NMP      = 0x7,  // NMP streaming read response
    RSP_GBControl = 0x8
  };

  // Number of vectors per timestep for each memory region
  NVUINT8 num_vector_large[spec::GB::Large::kMaxNumManagers];
  // Base address offset in SRAM for each memory region
  NVUINT16 base_large[spec::GB::Large::kMaxNumManagers];
  // Response register for NMP - declared at class level for HLS synthesis
  spec::GB::Large::DataRsp<1> large_rsp_reg;

  // Per-cycle control state
  bool is_axi;      // Flag indicating AXI request is being processed this cycle
  RspMode rsp_mode; // Response mode selector for end-of-cycle push
  spec::Axi::SubordinateToRVA::Write rva_in_reg; // Latched AXI write request
  spec::Axi::SubordinateToRVA::Read rva_out_reg; // Prepared AXI read response
  spec::GB::Large::DataReq large_req_reg;        // Latched NMP request

  // ===========================================================================
  // SRAM and Interface Signals
  // ===========================================================================

  // Single port SRAM for unified large buffer -- see include/ files for
  // sizing definition details
  ArbitratedScratchpadDP<
      spec::GB::Large::kNumBanks,       // number of banks
      spec::GB::Large::kNumReadPorts,   // number of read ports
      spec::GB::Large::kNumWritePorts,  // number of write ports
      spec::GB::Large::kEntriesPerBank, // entries per bank
      spec::GB::Large::WordType,        // data type
      false,                            // enable store forwarding
      true>                             // single port mode
      large_mem;

  // Read addresses
  spec::GB::Large::Address large_read_addrs[spec::GB::Large::kNumReadPorts];
  // Read request valid flags - set to 1 to initiate a read on that port
  bool large_read_req_valid[spec::GB::Large::kNumReadPorts];
  // Write port addresses
  spec::GB::Large::Address large_write_addrs[spec::GB::Large::kNumWritePorts];
  // Write request valid flag - set to 1 to initiate a write
  bool large_write_req_valid[spec::GB::Large::kNumWritePorts];
  // Data to be written to SRAM
  spec::GB::Large::WordType large_write_data[spec::GB::Large::kNumWritePorts];
  // Read acknowledge outputs from SRAM
  bool large_read_ack[spec::GB::Large::kNumReadPorts];
  // Write acknowledge outputs from SRAM
  bool large_write_ack[spec::GB::Large::kNumWritePorts];
  // Read ready flags
  bool large_read_ready[spec::GB::Large::kNumReadPorts];
  // Read data outputs from SRAM
  spec::GB::Large::WordType large_port_read_out[spec::GB::Large::kNumReadPorts];
  // Read output valid flags - indicates data is available on that port
  bool large_port_read_out_valid[spec::GB::Large::kNumReadPorts];

public:
  // ===========================================================================
  // External Interfaces
  // ===========================================================================

  // AXI interface for configuration
  Connections::In<spec::Axi::SubordinateToRVA::Write> rva_in_large;
  Connections::Out<spec::Axi::SubordinateToRVA::Read> rva_out_large;

  // NMP streaming interface
  Connections::In<spec::GB::Large::DataReq> nmp_large_req;
  Connections::Out<spec::GB::Large::DataRsp<1>> nmp_large_rsp;

  // GB Control interface
  Connections::In<spec::GB::Large::DataReq> gbcontrol_large_req;
  Connections::Out<spec::GB::Large::DataRsp<1>> gbcontrol_large_rsp;

  // 32-bit SRAM configuration register
  sc_in<NVUINT32> SC_SRAM_CONFIG;

  // ===========================================================================
  // Constructor and Reset Functions
  // ===========================================================================
  GBCore(const sc_module_name& nm) :
      match::Module(nm),
      rva_in_large("rva_in_large"),
      rva_out_large("rva_out_large"),

      nmp_large_req("nmp_large_req"),
      nmp_large_rsp("nmp_large_rsp"),

      SC_SRAM_CONFIG("SC_SRAM_CONFIG") {
    SC_THREAD(GBCoreRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void Reset() {
    // Reset interface ports
    rva_in_large.Reset();
    rva_out_large.Reset();
    nmp_large_req.Reset();
    nmp_large_rsp.Reset();
    gbcontrol_large_req.Reset();
    gbcontrol_large_rsp.Reset();


    // Reset address mapping registers
#pragma hls_unroll yes
    for (int i = 0; i < spec::GB::Large::kMaxNumManagers; i++) {
      num_vector_large[i] = 1;
      base_large[i]       = 0;
    }
  }

  // Reset per-cycle control state and SRAM interface signals
  void Initialize() {
    is_axi   = 0;
    rsp_mode = RSP_NONE;

#pragma hls_unroll yes
    for (unsigned i = 0; i < spec::GB::Large::kNumReadPorts; i++) {
      large_read_addrs[i]     = 0;
      large_read_req_valid[i] = 0;
      large_read_ready[i]     = 0;
    }
    large_write_addrs[0]     = 0;
    large_write_req_valid[0] = 0;
    large_write_data[0]      = 0;
  }

  // ===========================================================================
  // AXI Interface Handling
  // ===========================================================================

  /**
   * Decode AXI write request and update internal registers or initiate SRAM
   * write.
   */
  void DecodeAxiWrite() {
    is_axi               = 1;
    NVUINT4 tmp          = nvhls::get_slc<4>(rva_in_reg.addr, 20);
    NVUINT16 local_index = nvhls::get_slc<16>(rva_in_reg.addr, 4);
    CDCOUT(
        sc_time_stamp() << " GBCore Large: " << name() << "RVA Write " << endl,
        kDebugLevel);

    switch (tmp) {
      case 0x4: {
        if (local_index == 0x01) {
#pragma hls_unroll yes
          for (int i = 0; i < spec::GB::Large::kMaxNumManagers; i++) {
            num_vector_large[i] = nvhls::get_slc<8>(rva_in_reg.data, 32 * i);
            base_large[i] = nvhls::get_slc<16>(rva_in_reg.data, 32 * i + 16);
          }
        }
        break;
      }
      case 0x5: {
        large_write_addrs[0]     = local_index;
        large_write_req_valid[0] = 1;
        large_write_data[0]      = rva_in_reg.data;
        //cout << "local index and data:" << local_index << " " << rva_in_reg.data << endl;
        break;
      }
      default: {
        break;
      }
    } // switch
  } // DecodeAxiWrite

  /**
   * Decode AXI read request and prepare response data, either from the
   * configuration registers or by initiating an SRAM read.
   */
  void DecodeAxiRead() {
    is_axi               = 1;
    NVUINT4 tmp          = nvhls::get_slc<4>(rva_in_reg.addr, 20);
    NVUINT16 local_index = nvhls::get_slc<16>(rva_in_reg.addr, 4);
    CDCOUT(
        sc_time_stamp() << " GBCore Large: " << name() << "RVA Read " << endl,
        kDebugLevel);
    // is_axi_rsp = 1;
    rva_out_reg.data = 0;
    switch (tmp) {
      case 0x3: {
        rva_out_reg.data = SC_SRAM_CONFIG.read();
        rsp_mode         = RSP_SRAM_CFG;
        break;
      }
      case 0x4: {
        if (local_index == 0x01) {
#pragma hls_unroll yes
          for (int i = 0; i < spec::GB::Large::kMaxNumManagers; i++) {
            rva_out_reg.data.set_slc<8>(32 * i, num_vector_large[i]);
            rva_out_reg.data.set_slc<16>(32 * i + 16, base_large[i]);
          }
        }
        rsp_mode = RSP_ADDR_CFG;
        break;
      }
      case 0x5: {
        large_read_addrs[0]     = local_index;
        large_read_req_valid[0] = 1;
        large_read_ready[0]     = 1;
        rsp_mode                = RSP_AXI_SRAM;
        break;
      }
      default: {
        break;
      }
    } // switch
  } // DecodeAxiRead

  // ===========================================================================
  // GBCore Thread Helpers
  // ===========================================================================

  /**
   * @brief Map NMP large buffer request to physical SRAM address and
   *        prepare read/write signals.
   *
   * This function is called after a valid request has been received from some
   * submodule (e.g., NMP). It decodes the logical address in the request and
   * computes the physical SRAM address based on the base and stride registers.
   * It then sets up the read or write signals to the SRAM accordingly.
   *
   * @tparam N Number of read ports to prepare
   * @param large_req_reg Registered request to serve
   */
  template <unsigned N>
  inline void SetLargeBuffer(const spec::GB::Large::DataReq large_req_reg) {
    NVUINT3 memory_index                 = large_req_reg.memory_index;
    NVUINT8 vector_index                 = large_req_reg.vector_index;
    NVUINT16 timestep_index              = large_req_reg.timestep_index;
    spec::GB::Large::WordType write_data = large_req_reg.write_data;

    NVUINT4 lower_timestep_index  = nvhls::get_slc<4>(timestep_index, 0);
    NVUINT12 upper_timestep_index = nvhls::get_slc<12>(timestep_index, 4);

    spec::GB::Large::Address base_addr =
        base_large[memory_index] + lower_timestep_index +
        (upper_timestep_index * num_vector_large[memory_index] + vector_index) *
            spec::GB::Large::kNumBanks;
    if (large_req_reg.is_write) {

      large_write_addrs[0]     = base_addr;
      large_write_req_valid[0] = 1;
      large_write_data[0]      = write_data;

    } else {

#pragma hls_unroll yes
      for (unsigned i = 0; i < N; i++) {
        large_read_addrs[i]     = base_addr + i;
        large_read_req_valid[i] = 1;
        large_read_ready[i]     = 1;
      }
    }
  } // SetLargeBuffer

  /**
   * Poll NMP large buffer request interface and prepare SRAM access. If a read
   * request is received, set response mode for NMP read response.
   */
  void PollNMPPort() {
    if (nmp_large_req.PopNB(large_req_reg)) {
      // Initiate SRAM access for buffer request
      SetLargeBuffer<1>(large_req_reg);
      // Set read response mode
      if (!large_req_reg.is_write) {
        rsp_mode = RSP_NMP;
      }
    }
    else if (gbcontrol_large_req.PopNB(large_req_reg)){
      SetLargeBuffer<1>(large_req_reg);
      if (!large_req_reg.is_write) {
        rsp_mode = RSP_GBControl;
      }
    }
  }

  /**
   * Push outputs based on response mode set during request decoding
   */
  void PushOutputs() {

    switch (rsp_mode) {
      // For configuration reads, directly push the prepared response
      // from DecodeAxiRead()
      case RSP_SRAM_CFG:
      case RSP_ADDR_CFG: {
        rva_out_large.Push(rva_out_reg);
        break;
      }
      // For direct SRAM read via AXI, collect read data and push response
      // at end of cycle
      case RSP_AXI_SRAM: {
        rva_out_reg.data = large_port_read_out[0].to_rawbits();
        rva_out_large.Push(rva_out_reg);
        break;
      }
      // For NMP read response, collect read data and push to NMP interface
      case RSP_NMP: {
        large_rsp_reg.read_vector[0] = large_port_read_out[0];
        nmp_large_rsp.Push(large_rsp_reg);
        break;
      }
      case RSP_GBControl: {
        large_rsp_reg.read_vector[0] = large_port_read_out[0];
        gbcontrol_large_rsp.Push(large_rsp_reg);
        break;
      }
      // Default: no response this cycle
      default: break;
    } // switch

  } // PushOutputs


  void GBCoreRun() {
    Reset();

#pragma hls_pipeline_init_interval 1
    while (1) {
      // Clear per-cycle signals
      Initialize();

      // Process AXI requests first
      if (rva_in_large.PopNB(rva_in_reg)) {
        if (rva_in_reg.rw)
          DecodeAxiWrite();
        else
          DecodeAxiRead();
      }

      // If no AXI request, poll the interface port(s) from submodules
      // Right now only the NMP module is connected
      // But in the actual FlexASR system, interface ports from other modules
      // will also be polled in some arbitrated manner to process their requests
      // to the SRAM.
      // These polling functions would set a response mode indicating which
      // module to push a read response to at the end of the cycle.
      else
        PollNMPPort();


      large_mem.run(
          large_read_addrs,
          large_read_req_valid,
          large_write_addrs,
          large_write_req_valid,
          large_write_data,
          large_read_ack,
          large_write_ack,
          large_read_ready,
          large_port_read_out,
          large_port_read_out_valid);

      // Push outputs based on response mode
      PushOutputs();

      // Wait for next cycle
      wait();
    }
  }
};
#endif
