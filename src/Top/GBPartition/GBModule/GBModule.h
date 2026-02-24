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

#ifndef __GBMODULE__
#define __GBMODULE__

// SystemC and Matchlib includes
#include <ArbitratedScratchpadDP.h>
#include <nvhls_int.h>
#include <nvhls_module.h>
#include <nvhls_types.h>
#include <nvhls_vector.h>
#include <systemc.h>

// Project includes
#include "AxiSpec.h"
#include "GBCore/GBCore.h"
#include "GBControl/GBControl.h"
#include "NMP/NMP.h"
#include "Spec.h"


/**
 * @brief Top-level Global Buffer module integrating GBCore and NMP.
 *
 * This is a simplified version for Lab 3 that only includes:
 * - GBCore: Unified SRAM scratchpad buffer with AXI interface
 * - NMP: Near Memory Processing unit for RMSNorm/Softmax
 *
 * AXI address regions handled:
 * - 0x3: SRAM configuration register (write only)
 * - 0x4: GBCore configuration
 * - 0x5: GBCore large buffer read/write
 * - 0xC: NMP configuration and control
 */
class GBModule : public match::Module {
  static const int kDebugLevel = 3;
  SC_HAS_PROCESS(GBModule);

public:
  // ===========================================================================
  // External Interfaces
  // ===========================================================================
  /** AXI write request input from host */
  Connections::In<spec::Axi::SubordinateToRVA::Write> rva_in;
  /** AXI read response output to host */
  Connections::Out<spec::Axi::SubordinateToRVA::Read> rva_out;

    //GBControl <-> PE
  Connections::In<spec::StreamType>   data_in;          
  Connections::Out<spec::StreamType>  data_out;
  Connections::Out<bool>              pe_start;
  Connections::In<bool>               pe_done; 

  Connections::Out<bool> gb_done;


  // ===========================================================================
  // Internal Channels
  // ===========================================================================
  /** AXI request channel to GBCore */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> gbcore_rva_in;
  /** AXI response channel from GBCore */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> gbcore_rva_out;
  /** AXI request channel to NMP */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> nmp_rva_in;
  /** AXI response channel from NMP */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> nmp_rva_out;
  /** AXI response channel to GBControl */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> gbcontrol_rva_in;
  /** AXI response channel from GBControl */
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> gbcontrol_rva_out;

  /** NMP to GBCore large buffer request channel */
  Connections::Combinational<spec::GB::Large::DataReq> nmp_large_req;
  /** GBCore to NMP large buffer response channel */
  Connections::Combinational<spec::GB::Large::DataRsp<1>> nmp_large_rsp;

  /** GBCore to GBControl large buffer request channel */
  Connections::Combinational<spec::GB::Large::DataReq> gbcontrol_large_req;
  /** GBControl to GBCore large buffer response channel */
  Connections::Combinational<spec::GB::Large::DataRsp<1>> gbcontrol_large_rsp;

  /** Global SRAM configuration register */
  sc_signal<NVUINT32> SC_SRAM_CONFIG;

  Connections::Combinational<bool> gbcontrol_start;
  Connections::Combinational<bool> nmp_start;

  Connections::Combinational<bool> gbcontrol_done;
  Connections::Combinational<bool> nmp_done;

  // ===========================================================================
  // Submodule Instances
  // ===========================================================================
  /** GBCore SRAM scratchpad controller */
  GBCore gbcore_inst;
  /** NMP Near Memory Processing unit */
  NMP nmp_inst;
  /** GBControl for data in and out */
  GBControl gbcontrol_inst;


  // ===========================================================================
  // Constructor
  // ===========================================================================
  GBModule(sc_module_name nm) :
      match::Module(nm),
      rva_in("rva_in"),
      rva_out("rva_out"),
      data_in("data_in"),
      data_out("data_out"),
      pe_start("pe_start"),
      pe_done("pe_done"),
      gb_done("gb_done"),
      gbcore_rva_in("gbcore_rva_in"),
      gbcore_rva_out("gbcore_rva_out"),
      gbcontrol_rva_in("gbcontrol_rva_in"),
      gbcontrol_rva_out("gbcontrol_rva_out"),
      nmp_rva_in("nmp_rva_in"),
      nmp_rva_out("nmp_rva_out"),
      nmp_large_req("nmp_large_req"),
      nmp_large_rsp("nmp_large_rsp"),
      gbcontrol_large_req("gbcontrol_large_req"),
      gbcontrol_large_rsp("gbcontrol_large_rsp"),
      SC_SRAM_CONFIG("SC_SRAM_CONFIG"),
      gbcore_inst("gbcore_inst"),
      nmp_inst("nmp_inst"),
      gbcontrol_inst("gbcontrol_inst") {
    SC_THREAD(RVAInRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(RVAOutRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(GBDoneRun);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    // GBCore port bindings
    gbcore_inst.clk(clk);
    gbcore_inst.rst(rst);
    gbcore_inst.rva_in_large(gbcore_rva_in);
    gbcore_inst.rva_out_large(gbcore_rva_out);
    gbcore_inst.nmp_large_req(nmp_large_req);
    gbcore_inst.nmp_large_rsp(nmp_large_rsp);
    gbcore_inst.gbcontrol_large_req(gbcontrol_large_req);
    gbcore_inst.gbcontrol_large_rsp(gbcontrol_large_rsp);
    gbcore_inst.SC_SRAM_CONFIG(SC_SRAM_CONFIG);

    // NMP port bindings
    nmp_inst.clk(clk);
    nmp_inst.rst(rst);
    nmp_inst.rva_in(nmp_rva_in);
    nmp_inst.rva_out(nmp_rva_out);
    nmp_inst.start(nmp_start);
    nmp_inst.done(nmp_done);
    nmp_inst.large_req(nmp_large_req);
    nmp_inst.large_rsp(nmp_large_rsp);

    // GBControl port bindings
    gbcontrol_inst.clk(clk);
    gbcontrol_inst.rst(rst);
    gbcontrol_inst.rva_in(gbcontrol_rva_in);
    gbcontrol_inst.rva_out(gbcontrol_rva_out);
    gbcontrol_inst.start(gbcontrol_start);
    gbcontrol_inst.done(gbcontrol_done);
    gbcontrol_inst.large_req(gbcontrol_large_req);
    gbcontrol_inst.large_rsp(gbcontrol_large_rsp);
    gbcontrol_inst.data_out(data_out);
    gbcontrol_inst.data_in(data_in);
    gbcontrol_inst.pe_start(pe_start);
    gbcontrol_inst.pe_done(pe_done);

  } // GBModule

  // ===========================================================================
  // Thread Functions
  // ===========================================================================

  /**
   * @brief Input routing thread - routes AXI requests to GBCore or NMP.
   */
  void RVAInRun() {
    rva_in.Reset();
    gbcore_rva_in.ResetWrite();
    nmp_rva_in.ResetWrite();
    gbcontrol_rva_in.ResetWrite();
    gbcontrol_start.ResetWrite();
    nmp_start.ResetWrite();
    SC_SRAM_CONFIG.write(0);

#pragma hls_pipeline_init_interval 1
    while (1) {
      spec::Axi::SubordinateToRVA::Write rva_in_reg;
      if (rva_in.PopNB(rva_in_reg)) {
        NVUINT4 tmp = nvhls::get_slc<4>(rva_in_reg.addr, 20);
        NVUINT16 local_index = nvhls::get_slc<16>(rva_in_reg.addr, 4);
        //cout << "tmp, local index, data, addr: " << tmp << " " << local_index << " " << rva_in_reg.data << " " << rva_in_reg.addr << endl;

        if (tmp == 0x3 && rva_in_reg.rw) {
          SC_SRAM_CONFIG.write(nvhls::get_slc<32>(rva_in_reg.data, 0));
        } else if (tmp == 0x3 || tmp == 0x4 || tmp == 0x5) {
          gbcore_rva_in.Push(rva_in_reg);
        } else if (tmp == 0xC) {
          nmp_rva_in.Push(rva_in_reg);
        } else if (tmp == 0x7) {
          gbcontrol_rva_in.Push(rva_in_reg);
        } else if (tmp == 0x0){
            // TODO #1:
            // 1. Decode the `local_index` from the AXI address to identify the target sub-module.
            // 2. Send a start signal to the corresponding module's start channel.
            //    - 0x1: GBControl (PE <-> GB communication)
            //    - 0x2: NMP
          /////////////// YOUR CODE STARTS HERE ///////////////
          if (local_index == 0x1) {
            gbcontrol_start.Push(1);
          } else if (local_index == 0x2) {
            nmp_start.Push(1);
          }
          /////////////// YOUR CODE ENDS HERE ///////////////

        }
      }
      wait();
    }
  } // RVAInRun

  /**
   * @brief Output multiplexer thread - polls responses from GBCore and NMP.
   */
  void RVAOutRun() {
    rva_out.Reset();
    gbcore_rva_out.ResetRead();
    nmp_rva_out.ResetRead();
    gbcontrol_rva_out.ResetRead();

#pragma hls_pipeline_init_interval 1
    while (1) {
      spec::Axi::SubordinateToRVA::Read rva_out_reg;
      bool is_valid = 0;
      if (gbcore_rva_out.PopNB(rva_out_reg)) {
        is_valid = 1;
      } else if (nmp_rva_out.PopNB(rva_out_reg)) {
        is_valid = 1;
      } else if (gbcontrol_rva_out.PopNB(rva_out_reg)) {
        is_valid = 1;
      }
      if (is_valid) {
        rva_out.Push(rva_out_reg);
      }
      wait();
    }
  } // RVAOutRun
  
   void GBDoneRun() {
    gb_done.Reset();
    gbcontrol_done.ResetRead();
    nmp_done.ResetRead();


    #pragma hls_pipeline_init_interval 1
    while(1) {

      bool is_done = 0, done_reg = 0;
      if (gbcontrol_done.PopNB(done_reg)) {
        is_done = 1;
      }
      else if (nmp_done.PopNB(done_reg)) {
        is_done = 1;
      }
      if (is_done == 1){
        gb_done.Push(1);       
      }

      wait();
    }
  }

}; // GBModule

#endif // __GBMODULE__
