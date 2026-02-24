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

// =============================================================================
// GBCore Unit Testbench
// =============================================================================
//
// This testbench validates the GBCore configuration and
// its streaming interface to the NMP module.
//
// Test Coverage:
// - AXI config write/readback for large buffer base/stride data.
// - Streaming write from NMP interface into large buffer SRAM.
// - Streaming read from NMP interface and data integrity check.
// =============================================================================

#include <mc_scverify.h>
#include <nvhls_connections.h>
#include <systemc.h>
#include <testbench/nvhls_rand.h>

#include <iostream>
#include <vector>

#include "AxiSpec.h"
#include "GBCore.h"
#include "GBSpec.h"
#include "Spec.h"
#include "helper.h"

#define NVHLS_VERIFY_BLOCKS (GBCore)
#include <nvhls_verify.h>
#ifdef COV_ENABLE
#pragma CTC SKIP
#endif

// =============================================================================
// Global State Variables
// =============================================================================

// Expected AXI config data for readback verification
NVUINTW(128) expected_cfg_data;
// Flag indicating config data has been written and should be checked
bool expected_cfg_valid = false;
// Flag indicating config readback has been received and verified
bool seen_cfg_read = false;

// Expected data for each SRAM bank after write operations
spec::VectorType expected_large_data[spec::GB::Large::kNumBanks];
// Flags indicating data has been written to each bank
bool expected_large_valid[spec::GB::Large::kNumBanks];
// Flags indicating read response has been received for each bank
bool seen_large_read[spec::GB::Large::kNumBanks];
// Counter for total number of successful read verifications
int reads_completed = 0;

/**
 * @brief Build 128-brm it AXI config data for GBCore large buffer.
 * @param num_vec Number of vectors per timestep (bits [7:0])
 * @param base Base address offset in SRAM (bits [31:16])
 * @return Packed 128-bit configuration word
 */
inline NVUINTW(128) make_gbcore_cfg_data(NVUINT8 num_vec, NVUINT16 base) {
  NVUINTW(128) data = 0;
  data.set_slc<8>(0, num_vec);
  data.set_slc<16>(16, base);
  return data;
}
  

// =============================================================================
// Source Module
// =============================================================================

SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::Out<spec::Axi::SubordinateToRVA::Write> rva_in_large;
  Connections::Out<spec::GB::Large::DataReq> nmp_large_req;
  Connections::Out<spec::GB::Large::DataReq> gbcontrol_large_req;


  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void run() {
    rva_in_large.Reset();
    nmp_large_req.Reset();
    gbcontrol_large_req.Reset();
    wait();

    spec::Axi::SubordinateToRVA::Write rva_write;
    rva_write.rw       = 1;
    expected_cfg_data  = make_gbcore_cfg_data(1, 0);
    expected_cfg_valid = true;
    rva_write.data     = expected_cfg_data;
    rva_write.addr     = set_bytes<3>("40_00_10");
    rva_in_large.Push(rva_write);
    wait(2);

    rva_write.rw   = 0;
    rva_write.data = 0;
    rva_write.addr = set_bytes<3>("40_00_10");
    rva_in_large.Push(rva_write);
    wait(4);

    // Write to all banks by varying timestep_index lower bits
    // Address = base + lower_timestep_index + (upper * num_vec + vec_idx) *
    // kNumBanks Bank = Address % kNumBanks, so lower_timestep_index directly
    // selects the bank
    for (unsigned int bank = 0; bank < spec::GB::Large::kNumBanks; bank++) {
      spec::GB::Large::DataReq write_req;
      write_req.Reset();
      write_req.is_write          = 1;
      write_req.memory_index      = 0;
      write_req.vector_index      = 0;
      write_req.timestep_index    = bank; // lower 4 bits select bank
      spec::VectorType write_data = 0;
      for (int i = 0; i < spec::kVectorSize; i++) {
        write_data[i] = bank * spec::kVectorSize + i;
      }
      write_req.write_data       = write_data;
      expected_large_data[bank]  = write_data;
      expected_large_valid[bank] = true;
      seen_large_read[bank]      = false;
      nmp_large_req.Push(write_req);
      wait(2);
    }

    // Read from all banks
    for (unsigned int bank = 0; bank < spec::GB::Large::kNumBanks; bank++) {
      spec::GB::Large::DataReq read_req;
      read_req.Reset();
      read_req.is_write       = 0;
      read_req.memory_index   = 0;
      read_req.vector_index   = 0;
      read_req.timestep_index = bank;
      nmp_large_req.Push(read_req);
      wait(2);
    }
    wait();
  }
};

// =============================================================================
// Dest Module
// =============================================================================

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  // AXI read response interface - receives config readback data
  Connections::In<spec::Axi::SubordinateToRVA::Read> rva_out_large;
  // NMP read response interface - receives SRAM read data
  Connections::In<spec::GB::Large::DataRsp<1>> nmp_large_rsp;
  Connections::In<spec::GB::Large::DataRsp<1>> gbcontrol_large_rsp;


  SC_CTOR(Dest) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void run() {
    rva_out_large.Reset();
    nmp_large_rsp.Reset();
    gbcontrol_large_rsp.Reset();
    wait();

    while (1) {
      spec::Axi::SubordinateToRVA::Read rva_out;
      if (rva_out_large.PopNB(rva_out)) {
        cout << hex << sc_time_stamp() << " RVA read data = " << rva_out.data
             << endl;
        if (expected_cfg_valid && !seen_cfg_read) {
          if (rva_out.data != expected_cfg_data) {
            SC_REPORT_ERROR("GBCore", "RVA config readback mismatch");
          } else {
            cout << sc_time_stamp() << " RVA config matched" << endl;
          }
          seen_cfg_read = true;
        }
      }

      spec::GB::Large::DataRsp<1> rsp;
      if (nmp_large_rsp.PopNB(rsp)) {
        // Find which bank this response matches by checking expected data
        int matched_bank = -1;
        for (unsigned int bank = 0; bank < spec::GB::Large::kNumBanks; bank++) {
          if (expected_large_valid[bank] && !seen_large_read[bank]) {
            bool match = true;
            for (int i = 0; i < spec::kVectorSize; i++) {
              if (rsp.read_vector[0][i] != expected_large_data[bank][i]) {
                match = false;
                break;
              }
            }
            if (match) {
              matched_bank = bank;
              break;
            }
          }
        }
        if (matched_bank >= 0) {
          cout << sc_time_stamp() << " Large buffer bank " << matched_bank
               << " read matched" << endl;
          seen_large_read[matched_bank] = true;
          reads_completed++;
        } else {
          SC_REPORT_ERROR("GBCore", "Large buffer read mismatch");
        }
      }
      wait();
    }
  }
};

// =============================================================================
// Testbench Top Module
// =============================================================================

SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);

  // Clock and reset signals
  sc_clock clk;
  sc_signal<bool> rst;
  // SRAM configuration register input to DUT
  sc_signal<NVUINT32> sc_sram_config;

  // AXI interface channels
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> rva_in_large;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> rva_out_large;
  // NMP streaming interface channels
  Connections::Combinational<spec::GB::Large::DataReq> nmp_large_req;
  Connections::Combinational<spec::GB::Large::DataRsp<1>> nmp_large_rsp;

  Connections::Combinational<spec::GB::Large::DataReq> gbcontrol_large_req;
  Connections::Combinational<spec::GB::Large::DataRsp<1>> gbcontrol_large_rsp;



  // Module instances
  NVHLS_DESIGN(GBCore) dut;
  Source source;
  Dest dest;

  testbench(sc_module_name name) :
      sc_module(name),
      clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
      rst("rst"),
      sc_sram_config("sc_sram_config"),
      dut("dut"),
      source("source"),
      dest("dest") {
    dut.clk(clk);
    dut.rst(rst);
    dut.rva_in_large(rva_in_large);
    dut.rva_out_large(rva_out_large);
    dut.nmp_large_req(nmp_large_req);
    dut.nmp_large_rsp(nmp_large_rsp);
    dut.gbcontrol_large_req(gbcontrol_large_req);
    dut.gbcontrol_large_rsp(gbcontrol_large_rsp);
    dut.SC_SRAM_CONFIG(sc_sram_config);

    source.clk(clk);
    source.rst(rst);
    source.rva_in_large(rva_in_large);
    source.nmp_large_req(nmp_large_req);
    source.gbcontrol_large_req(gbcontrol_large_req);


    dest.clk(clk);
    dest.rst(rst);
    dest.rva_out_large(rva_out_large);
    dest.nmp_large_rsp(nmp_large_rsp);
    dest.gbcontrol_large_rsp(gbcontrol_large_rsp);


    SC_THREAD(run);
  }

  void run() {
    sc_sram_config.write(0);
    wait(2, SC_NS);
    std::cout << "@" << sc_time_stamp() << " Asserting reset" << std::endl;
    rst.write(false);
    wait(2, SC_NS);
    rst.write(true);
    std::cout << "@" << sc_time_stamp() << " De-Asserting reset" << std::endl;
    wait(500, SC_NS); // Increase timeout for bank operations

    // Check that config readback was seen
    if (!seen_cfg_read) {
      SC_REPORT_ERROR("GBCore", "RVA config readback not observed");
    }
    // Check all banks were read
    for (unsigned int bank = 0; bank < spec::GB::Large::kNumBanks; bank++) {
      if (!seen_large_read[bank]) {
        std::cout << "Bank " << bank << " read response not observed"
                  << std::endl;
        SC_REPORT_ERROR("GBCore", "Large buffer read response not observed");
      }
    }
    std::cout << "@" << sc_time_stamp() << " All " << reads_completed
              << " bank reads completed" << std::endl;
    std::cout << "@" << sc_time_stamp() << " sc_stop" << std::endl;
    sc_stop();
  }
};

// =============================================================================
// Simulation Entry Point
// =============================================================================

int sc_main(int argc, char* argv[]) {
  // Initialize random seed for reproducible test patterns
  nvhls::set_random_seed();
  testbench tb("tb");

  // Configure error reporting to display but not abort
  sc_report_handler::set_actions(SC_ERROR, SC_DISPLAY);
  sc_start();

  // Return pass/fail based on error count
  bool rc = (sc_report_handler::get_count(SC_ERROR) > 0);
  if (rc)
    DCOUT("TESTBENCH FAIL" << endl);
  else
    DCOUT("TESTBENCH PASS" << endl);
  return rc;
}
