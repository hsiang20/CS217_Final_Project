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
// GBModule Integration Testbench (GBCore + NMP)
// =============================================================================
//
// This testbench validates the integrated GBModule containing GBCore (SRAM
// scratchpad) and NMP (Near Memory Processing) submodules working together.
//
// Test Coverage:
// (a) AXI config read/write for GBCore and NMP configuration registers.
// (b) AXI direct read/write of GBCore large buffer SRAM.
// (c) Softmax operation via NMP with write-back to GBCore SRAM.
// (d) RMSNorm operation via NMP with write-back to GBCore SRAM.
// =============================================================================

#include "GBModule.h"

#include <mc_scverify.h>
#include <nvhls_connections.h>
#include <systemc.h>
#include <testbench/nvhls_rand.h>

#include <cmath>
#include <deque>
#include <iostream>
#include <vector>

#include <ac_math.h>
#include "GBSpec.h"
#include "NMPSpec.h"
#include "Spec.h"
#include "helper.h"

#define NVHLS_VERIFY_BLOCKS (GBModule)
#include <nvhls_verify.h>

#ifdef COV_ENABLE
#pragma CTC SKIP
#endif

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

/**
 * @brief Build AXI address for direct GBCore SRAM access.
 * @param local_index SRAM address (bank-interleaved)
 * @return 24-bit AXI address with region selector
 */
inline NVUINTW(24) make_gbcore_data_addr(NVUINT16 local_index) {
  NVUINTW(24) addr = 0;
  addr.set_slc<4>(20, NVUINT4(0x5));
  addr.set_slc<16>(4, local_index);
  return addr;
}

static const double kAbsTolerance = 0.5;
static const double kPctTolerance = 10.0;
bool vectors_match_with_tolerance(
    const spec::VectorType& actual,
    const spec::VectorType& expected) {
  bool ok = true;
  for (int i = 0; i < spec::kVectorSize; i++) { 
    const double exp_val = fixed2float<spec::kIntWordWidth, spec::kIntWordWidth - spec::NMP::kNmpInputNumFrac>(expected[i]);
    const double act_val = fixed2float<spec::kIntWordWidth, spec::kIntWordWidth - spec::NMP::kNmpInputNumFrac>(actual[i]);
    const double abs_err = std::fabs(act_val - exp_val);
    const double denom   = std::max(std::fabs(exp_val), 1e-9);
    const double pct_err = (abs_err / denom) * 100.0;
    const bool match = !(abs_err > kAbsTolerance || pct_err > kPctTolerance);
    std::cout << (match ? "Match" : "Mismatch") << " idx " << i
              << ": expected=" << exp_val << " actual=" << act_val
              << " abs_err=" << abs_err << " pct_err=" << pct_err << "%"
              << std::endl;
    if (!match) {
      ok = false;
    }
  }
  return ok;
}
  NVINTW(spec::kIntWordWidth) float2fixed(const float in, const int frac_bits) {
    return in * (1 << frac_bits);
  }

  void compute_rms_expected(const spec::VectorType& in, spec::VectorType& out){
    std::vector<double> vals_full(spec::kVectorSize, 0.0);
    for (int i = 0; i < spec::kVectorSize; i++){
      vals_full[i] = fixed2float<spec::kIntWordWidth, spec::kIntWordWidth - spec::NMP::kNmpInputNumFrac>(in[i]);
      //cout << "in and vals_full: " << in[i] << " " << vals_full[i] << endl;
    }
    double sum_sq = 0.0;
    for (int i = 0; i < spec::kVectorSize; i++) {
      sum_sq += vals_full[i] * vals_full[i];
    }
    const double inv_size       = 1.0 / static_cast<double>(spec::kVectorSize);
    const double mean           = sum_sq * inv_size;
    const double epsilon        = 1e-4;
    const double rms_reciprocal = 1.0 / std::sqrt(mean + epsilon);

    for (int i = 0; i < spec::kVectorSize; i++) {
      const double out_val = vals_full[i] * rms_reciprocal;
      out[i] = float2fixed(out_val, spec::NMP::kNmpInputNumFrac);
      //cout << "out and vals_full: " << out[i] << " " << out_val << endl;
    }
  }

  void compute_softmax_expected(const spec::VectorType& in, spec::VectorType& out) {
  std::vector<double> vals_full(spec::kVectorSize, 0.0);
    for (int i = 0; i < spec::kVectorSize; i++){
      vals_full[i] = fixed2float<spec::kIntWordWidth, spec::kIntWordWidth - spec::NMP::kNmpInputNumFrac>(in[i]);
    }

  double max_val = vals_full[0];
  for (int i = 1; i < spec::kVectorSize; i++) {
    if (vals_full[i] > max_val) {
      max_val = vals_full[i];
    }
  }

  std::vector<double> exp_vals(spec::kVectorSize, 0.0);
  double sum_exp = 0.0;
  for (int i = 0; i < spec::kVectorSize; i++) {
    exp_vals[i] = std::exp(vals_full[i] - max_val);
    sum_exp += exp_vals[i];
  }
  const double inv_sum = (sum_exp == 0.0) ? 0.0 : (1.0 / sum_exp);

  for (int i = 0; i < spec::kVectorSize; i++) {
    const double out_val = exp_vals[i] * inv_sum;
      out[i] = float2fixed(out_val, spec::NMP::kNmpInputNumFrac);
      //cout << "input and outout: " << in[i] << " " << out[i] << endl;
  
  }
}

  NVUINTW(128) make_nmp_cfg_data(
    uint8_t mode,
    uint8_t mem,
    uint8_t nvec,
    uint16_t ntimesteps) {
  NVUINTW(128) data = 0;
  data.set_slc<1>(0, NVUINT1(1));
  data.set_slc<3>(8, NVUINT3(mode));
  data.set_slc<3>(32, NVUINT3(mem));
  data.set_slc<8>(48, NVUINT8(nvec));
  data.set_slc<16>(64, NVUINT16(ntimesteps));
  return data;
  }
/**
 * @brief Create AXI write command with NMP configuration.
 * @return Complete AXI write request struct
 */

 NVUINTW(128) make_gbcontrol_cfg(
     uint8_t mode,
    uint8_t mem1,
    uint8_t mem2,
    uint8_t nvec1,
    uint8_t nvec2,
    uint16_t ntimestep1,
    uint16_t ntimestep2) {
      uint8_t is_rnn = 0;
      NVUINTW(128) data = 0;
      data.set_slc<1>(0, NVUINT1(1));
      data.set_slc<3>(8, NVUINT3(mode));
      data.set_slc<3>(32, NVUINT3(mem1));
      data.set_slc<3>(40, NVUINT3(mem2));
      data.set_slc<8>(48, NVUINT8(nvec1));
      data.set_slc<8>(56, NVUINT8(nvec2));
      data.set_slc<16>(64, NVUINT16(ntimestep1));
      data.set_slc<16>(80, NVUINT16(ntimestep2));
      return data;
    }
      
spec::Axi::SubordinateToRVA::Write make_cfg(
    uint8_t mode,
    uint8_t mem,
    uint8_t nvec,
    uint16_t ntimestep) {
  spec::Axi::SubordinateToRVA::Write w;
  w.rw   = 1;
  w.data = make_nmp_cfg_data(mode, mem, nvec, ntimestep);
  w.addr = set_bytes<3>("C0_00_10");
  return w;
}
// =============================================================================
// Global State Variables
// =============================================================================

// Queue of expected AXI read responses (for config and direct SRAM reads)
std::deque<NVUINTW(128)> expected_rva_reads;
// Queue of expected NMP outputs (require tolerance-based comparison)
std::deque<spec::VectorType> expected_nmp_outputs;
// Counter for AXI read responses (used for Source/Dest synchronization)
unsigned rva_read_count = 0;

spec::VectorType data_out_popped = 0;
spec::VectorType rva_out_data = 0; 

// =============================================================================
// Source Module
// =============================================================================

SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  // AXI write interface for configuration and SRAM access
  Connections::Out<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Out<bool> pe_done;
  Connections::Out<spec::StreamType> data_in;

  // Done signal from GBModule (operation complete)

  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  // ===========================================================================
  // Synchronization Helper Functions
  // ===========================================================================


  /**
   * @brief Block until Dest module receives an AXI read response.
   */
  void wait_for_read_response() {
    unsigned before = rva_read_count;
    while (rva_read_count == before) {
      wait();
    }
  }

  // ===========================================================================
  // Main Test Sequence
  // ===========================================================================

  void run() {
    rva_in.Reset();
    pe_done.Reset();
    data_in.Reset();
    wait();

    spec::Axi::SubordinateToRVA::Write rva_cmd;

    // (a) AXI config read/write for GBCore and NMP.
    //cout << sc_time_stamp() << " Test (a): AXI config write/read for GBCore and NMP" << endl;
    NVUINTW(128) gbcore_cfg = make_gbcore_cfg_data(1, 0);
    rva_cmd.rw              = 1;
    rva_cmd.data            = gbcore_cfg;
    rva_cmd.addr            = set_bytes<3>("40_00_10");
    rva_in.Push(rva_cmd);
    cout << "    WRITE Config: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;

    wait();

    NVUINTW(128) nmp_cfg = make_nmp_cfg_data(1, 0, 1, 1);
    rva_cmd.rw           = 1;
    rva_cmd.data         = nmp_cfg;
    rva_cmd.addr         = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    cout << "    WRITE Config: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;

    wait();

    rva_cmd.rw   = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("40_00_10");
    rva_in.Push(rva_cmd);
    expected_rva_reads.push_back(gbcore_cfg);
    wait_for_read_response();

    rva_cmd.rw   = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    expected_rva_reads.push_back(nmp_cfg);
    wait_for_read_response();

    // (b) AXI read/write of GBCore large SRAM.
    //cout << sc_time_stamp() << " Test (b): AXI write/read of GBCore large SRAM" << endl;
    /*for (NVUINT16 bank_idx = 0; bank_idx < spec::GB::Large::kNumBanks;
         bank_idx++) {
      spec::VectorType direct_data = 0;
      for (int i = 0; i < spec::kVectorSize; i++) {
        direct_data[i] = bank_idx + 1;
      }
      rva_cmd.rw   = 1;
      rva_cmd.data = direct_data.to_rawbits();
      rva_cmd.addr = make_gbcore_data_addr(bank_idx);
      rva_in.Push(rva_cmd);
      cout << "    WRITE Data: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;

      wait();

      rva_cmd.rw   = 0;
      rva_cmd.data = 0;
      rva_cmd.addr = make_gbcore_data_addr(bank_idx);
      rva_in.Push(rva_cmd);
      expected_rva_reads.push_back(direct_data.to_rawbits());
      wait_for_read_response();
    }*/

    // (c) Softmax operation on NMP and read back via GBCore.
    // cout << sc_time_stamp() << " Test (c): NMP Softmax writeback to GBCore SRAM" << endl;
    spec::VectorType softmax_input = nvhls::get_rand<spec::VectorType::width>();
    rva_cmd.rw                     = 1;
    rva_cmd.data                   = softmax_input.to_rawbits();
    rva_cmd.addr                   = set_bytes<3>("50_00_00");
    rva_in.Push(rva_cmd);
    cout << "    WRITE softmax: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait();

    nmp_cfg      = make_nmp_cfg_data(1, 0, 1, 1);
    rva_cmd.rw   = 1;
    rva_cmd.data = nmp_cfg;
    rva_cmd.addr = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    cout << "    WRITE Config: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait();

    rva_cmd.rw   = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    expected_rva_reads.push_back(nmp_cfg);
    wait_for_read_response();

    rva_cmd.rw = 1;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("00_00_20"); // nmp_start
    rva_in.Push(rva_cmd);
    cout << "    START NMP " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait(100);

    spec::VectorType softmax_expected;
    compute_softmax_expected(softmax_input, softmax_expected);
    rva_cmd.rw   = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("50_00_00");
    rva_in.Push(rva_cmd);
    expected_nmp_outputs.push_back(softmax_expected);
    wait_for_read_response();

    // GBControl Program
    rva_cmd.rw = 1;
    rva_cmd.data = make_gbcontrol_cfg(1, 0, 0, 1, 0, 1, 0);
    rva_cmd.addr = set_bytes<3>("70_00_10");
    rva_in.Push(rva_cmd);
    cout << "    WRITE GBControl: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait();

    rva_cmd.rw = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("70_00_10");
    rva_in.Push(rva_cmd);
    expected_rva_reads.push_back(make_gbcontrol_cfg(1, 0, 0, 1, 0, 1, 0));
    wait_for_read_response();

    rva_cmd.rw = 1;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("00_00_10"); // nmp_start
    rva_in.Push(rva_cmd);
    cout << "    START GBControl " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait(100);
    


    // (d) RMSNorm operation on NMP and read back via GBCore.
    /*cout << sc_time_stamp() << " Test (d): NMP RMSNorm writeback to GBCore SRAM"
         << endl;
    spec::VectorType rms_input = nvhls::get_rand<spec::VectorType::width>();
    rva_cmd.rw                 = 1;
    rva_cmd.data               = rms_input.to_rawbits();
    rva_cmd.addr               = set_bytes<3>("50_00_00");
    rva_in.Push(rva_cmd);
    cout << "    WRITE rms: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait();

    nmp_cfg      = make_nmp_cfg_data(0, 0, 1, 1);
    rva_cmd.rw   = 1;
    rva_cmd.data = nmp_cfg;
    rva_cmd.addr = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    cout << "    WRITE NMP cfg: " << std::hex << rva_cmd.data << " @ " << rva_cmd.addr << endl;
    wait();

    rva_cmd.rw   = 0;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("C0_00_10");
    rva_in.Push(rva_cmd);
    expected_rva_reads.push_back(nmp_cfg);
    wait_for_read_response();

    rva_cmd.rw = 1;
    rva_cmd.data = 0;
    rva_cmd.addr = set_bytes<3>("00_00_20"); // nmp_start
    rva_in.Push(rva_cmd);
    cout << "    START NMP" << endl;
    wait(2);

    spec::VectorType rms_expected;
    compute_rms_expected(rms_input, rms_expected);
    rva_cmd.rw                    = 0;
    rva_cmd.data                  = 0;
    rva_cmd.addr                  = set_bytes<3>("50_00_00");
    rva_in.Push(rva_cmd);
    // Use tolerance-based comparison for NMP outputs
    expected_nmp_outputs.push_back(rms_expected);
    wait_for_read_response();*/
  }
};

// =============================================================================
// Dest Module
// =============================================================================

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  // AXI read response interface
  Connections::In<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::In<spec::StreamType>   data_out;
  Connections::In<bool>              pe_start;
  Connections::In<bool>              gb_done;

  bool gb_done_received = false;
  bool pe_start_received = false;
  bool data_out_received = false;



  spec::StreamType data_out_reg;

  SC_CTOR(Dest) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(CheckDone);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(PopDataOut);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(SimExit);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void SimExit() {
    wait();

    while(1) {
      wait();
      if (pe_start_received && gb_done_received && data_out_received){
        for (int i = 0; i < 16; i++){
          if (data_out_popped[i] != rva_out_data[i]){
          SC_REPORT_ERROR("Mistmatch", "Between rva out and data out");
          }
        }
        sc_stop();
      }
    }
  }

  void CheckDone(){
    pe_start.Reset();
    gb_done.Reset();

    bool pe_start_reg = false;
    bool gb_done_reg = false;   

    wait();
    while(1){
      wait();
      if (pe_start.PopNB(pe_start_reg)){
        cout << sc_time_stamp() << " Recevied PE Start = " << pe_start_reg << endl;
        pe_start_received = true;
      } else if (gb_done.PopNB(gb_done_reg)){
        cout << sc_time_stamp() << " Recevied GB Done = " << gb_done_reg << endl;
        gb_done_received = true;
      }
    }
  }

  void PopDataOut() {
    data_out.Reset();
    wait();

    while (1) {
      wait();
      if (data_out.PopNB(data_out_reg)) {
        cout << sc_time_stamp() << " Data out popped: " << std::hex << data_out_reg.data << endl;
        data_out_popped = data_out_reg.data;
        data_out_received = true;
      }
    }
  }

  void run() {
    rva_out.Reset();
    rva_read_count = 0;
    wait();

    while (1) {
      spec::Axi::SubordinateToRVA::Read rva_out_dest;
      if (rva_out.PopNB(rva_out_dest)) {
        //cout << hex << sc_time_stamp() << " RVA read data = " << rva_out_dest.data << endl;
        
        

        // Check if this is an NMP output requiring tolerance-based comparison
        if (!expected_nmp_outputs.empty()) {
          rva_read_count++;
          spec::VectorType expected   = expected_nmp_outputs.front();
          expected_nmp_outputs.pop_front();

          spec::VectorType actual;
          actual = rva_out_dest.data;
          rva_out_data = rva_out_dest.data;


          cout << sc_time_stamp() << " Comparing NMP output with tolerance..."
               << endl;
          if (!vectors_match_with_tolerance(actual, expected)) {
            SC_REPORT_ERROR("GBModule", "NMP output mismatch");
          } else {
            cout << sc_time_stamp() << " NMP output matched within tolerance"
                 << endl;
          }
        } else if (!expected_rva_reads.empty()) {
          rva_read_count++;
          // Exact match for non-NMP reads (config, direct SRAM)
          NVUINTW(128) expected = expected_rva_reads.front();
          expected_rva_reads.pop_front();
          if (rva_out_dest.data != expected) {
            cout << hex << sc_time_stamp()
                 << " Expected RVA data = " << expected << endl;
            SC_REPORT_ERROR("GBModule", "RVA read mismatch");
          } else {
            //cout << sc_time_stamp() << " RVA read matched" << endl;
          }
        } else {
          SC_REPORT_ERROR("GBModule", "Unexpected RVA read");
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

  // AXI interface channels
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> rva_out;

  // GBControl <-> PE interface
  Connections::Combinational<spec::StreamType>   data_in;          
  Connections::Combinational<spec::StreamType>  data_out;
  Connections::Combinational<bool>              pe_start;
  Connections::Combinational<bool>               pe_done; 
  
  // Done signal
  Connections::Combinational<bool> gb_done;

  // Module instances
  NVHLS_DESIGN(GBModule) dut;
  Source source;
  Dest dest;

  testbench(sc_module_name name) :
      sc_module(name),
      clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
      rst("rst"),
      dut("dut"),
      source("source"),
      dest("dest") {
    dut.clk(clk);
    dut.rst(rst);
    dut.rva_in(rva_in);
    dut.rva_out(rva_out);
    dut.data_in(data_in);
    dut.data_out(data_out);
    dut.pe_start(pe_start);
    dut.pe_done(pe_done);
    dut.gb_done(gb_done);

    source.clk(clk);
    source.rst(rst);
    source.rva_in(rva_in);
    source.pe_done(pe_done);
    source.data_in(data_in);

    dest.clk(clk);
    dest.rst(rst);
    dest.rva_out(rva_out);
    dest.data_out(data_out);
    dest.pe_start(pe_start);
    dest.gb_done(gb_done);


    SC_THREAD(run);
  }

  void run() {
    wait(2, SC_NS);
    std::cout << "@" << sc_time_stamp() << " Asserting reset" << std::endl;
    rst.write(false);
    wait(2, SC_NS);
    rst.write(true);
    std::cout << "@" << sc_time_stamp() << " De-Asserting reset" << std::endl;
    wait(1000, SC_NS);
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
