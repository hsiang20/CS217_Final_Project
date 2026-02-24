/*
 * All rights reserved - Stanford University. 
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License.  
 * You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "PEModule.h"
#include <systemc.h>
#include <mc_scverify.h>
#include <testbench/nvhls_rand.h>
#include <nvhls_connections.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <math.h> // testbench only
#include <iomanip>
#include <iostream>
#include <fstream>

#include "Spec.h"
#include "AxiSpec.h"
#include "helper.h"

#define NVHLS_VERIFY_BLOCKS (PEModule)
#include <nvhls_verify.h>

spec::ActVectorType golden_y;
sc_event sync_event;

SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::Out<spec::StreamType> input_port;
  Connections::Out<bool> start;
  Connections::Out<spec::Axi::SubordinateToRVA::Write> rva_in;

  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

void Tanh_ref(const spec::ActVectorType& in, spec::ActVectorType& out) {
    for (int i = 0; i < spec::kNumVectorLanes; i++) {
      float in_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(in[i]);
      float out_float = tanh(in_float);
      out[i] = float2fixed(out_float, spec::kActNumFrac);
    }
  }

  void Relu_ref(const spec::ActVectorType& in, spec::ActVectorType& out) {
    for (int i = 0; i < spec::kNumVectorLanes; i++) {
      float in_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(in[i]);
      float out_float = (in_float > 0) ? in_float : 0;
      out[i] = float2fixed(out_float, spec::kActNumFrac);
    }
  }

  NVINTW(spec::kActWordWidth) float2fixed(const float in, const int frac_bits) {
    return in * (1 << frac_bits);
  }

  void Silu_ref(const spec::ActVectorType& in, spec::ActVectorType& out) {
    for (int i = 0; i < spec::kNumVectorLanes; i++) {
      float in_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(in[i]);
      float out_float = in_float * sigmoid(in_float);
      out[i] = float2fixed(out_float, spec::kActNumFrac);
    }
  }

  void Gelu_ref(const spec::ActVectorType& in, spec::ActVectorType& out) {
    for (int i = 0; i < spec::kNumVectorLanes; i++) {
      float in_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(in[i]);
      float out_float = 0.5 * in_float * (1 + tanh(sqrt(2/M_PI) * (in_float + 0.044715 * pow(in_float, 3))));
      out[i] = float2fixed(out_float, spec::kActNumFrac);
    }
  }

  void run() {
    start.Reset();
    input_port.Reset();
    rva_in.Reset();
    wait();

    // Generate random data
    std::vector<std::vector<int>> W(spec::kNumVectorLanes, std::vector<int>(spec::kNumVectorLanes));
    std::vector<int> x(spec::kNumVectorLanes);

    for (int i = 0; i < spec::kNumVectorLanes; ++i) {
      for (int j = 0; j < spec::kNumVectorLanes; ++j) {
        W[i][j] = nvhls::get_rand<spec::kIntWordWidth>();
      }
      x[i] = nvhls::get_rand<spec::kIntWordWidth>();
    }

    // Golden model: y = tanh(W*x)
    std::vector<int> Wx = MatrixVectorMul(W, x);
    spec::ActVectorType ActUnitInput = 0;

    for (int i = 0; i < spec::kNumVectorLanes; ++i) {
        ActUnitInput[i] = int(Wx[i]/12.25);
      }

    Tanh_ref(ActUnitInput, golden_y);

    spec::Axi::SubordinateToRVA::Write rva_in_src;

    // 1. PEConfig: 
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<8>("00_00_01_01_00_00_00_01");
    rva_in_src.addr = set_bytes<3>("40_00_10");
    cout << "    WRITE PEConfig: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();

    // 2. PEManager 0:
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<8>("00_00_00_00_00_00_01_00");
    rva_in_src.addr = set_bytes<3>("40_00_20");
    cout << "    WRITE PEManager: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();

    // 3. Load Weights (W)
    for (int i = 0; i < spec::kNumVectorLanes; i++) { // For each row of W
      spec::VectorType weight_vec;
      for (int j = 0; j < spec::kNumVectorLanes; j++) {
        weight_vec[j] = W[i][j];
      }
      rva_in_src.rw = 1;
      rva_in_src.data = weight_vec.to_rawbits();
      rva_in_src.addr = 0x500000 + i * 16;
      cout << "    WRITE Weight: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
      rva_in.Push(rva_in_src);
      wait();
    }
    
    // 4. Load Input (x)
    spec::VectorType input_vec;
    for (int i = 0; i < spec::kNumVectorLanes; i++) {
        input_vec[i] = x[i];
    }
    rva_in_src.rw = 1;
    rva_in_src.data = input_vec.to_rawbits();
    rva_in_src.addr = 0x600000 + 0;
    cout << "    WRITE Input: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();

    // 6. ActUnit Config: 0x01
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<16>("00_00_00_00_00_00_00_00_00_00_00_01_03_02_00_01");
    rva_in_src.addr = set_bytes<3>("80_00_10");
    cout << "    WRITE ActUnit Config: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();

    // 7. ActUnit Instructions: INPE, TANH, OUTGB
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<16>("00_00_00_00_00_00_00_00_00_00_00_00_00_40_B0_30");
    rva_in_src.addr = set_bytes<3>("80_00_20");
    cout << "    WRITE ActUnit Instructions: " << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();

    // 8. Start
    rva_in_src.rw = 1;
    rva_in_src.data = 0;
    rva_in_src.addr = set_bytes<3>("00_00_00");
    cout << "    WRITE Start write:" << std::hex << rva_in_src.data << " @ " << rva_in_src.addr << endl;
    rva_in.Push(rva_in_src);
    wait();
  }
};

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::In<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::In<spec::StreamType> output_port;
  Connections::In<bool> done;

  spec::StreamType dut_output;
  spec::StreamType output_port_dest;
  spec::StreamType output_check;
  bool dut_output_popped = false;
  bool done_signal_received = false;

  SC_CTOR(Dest) {
    SC_THREAD(PopDone);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(PopRVAOut);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(PopOutport);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(OutputCompare);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void PopRVAOut() {
    rva_out.Reset();
    wait();
    while(1) {
        spec::Axi::SubordinateToRVA::Read rva_out_dest;
        if (rva_out.PopNB(rva_out_dest)) {
            cout << sc_time_stamp() << " Dest: Received AXI read response." << endl;
        }
        wait();
    }
  }

  void PopOutport() {
    output_port.Reset();
    wait();

    while (1) {
      if (output_port.PopNB(output_port_dest)) {
        cout << sc_time_stamp() << " Dest: Received output port data:" << output_port_dest.data << endl;
        output_check = output_port_dest;
        dut_output_popped = true;
      }
      wait();
    }
  }

  void PopDone() {
    done.Reset();
    wait();
    while (1) {
      bool done_dest;
      if (done.PopNB(done_dest)) {
        if (done_dest) {
          done_signal_received = true;
        }
      }
      wait();
    }
  }

  void OutputCompare() {
    double max_diff = 0.0;
    double diff = 0.0;
    double percent_diff = 0.0;
    double accumulated_percent_diff = 0.0;
    while (1) {
      wait();
      if (dut_output_popped && done_signal_received) {
        cout << "Starting comparison with golden model" << endl;

          bool passed = true;
          
          for (size_t i = 0; i < spec::kNumVectorLanes; ++i) {              
              float dut_output_float = fixed2float<spec::kIntWordWidth, spec::Act::kActOutNumFrac>(output_check.data[i]);
              float golden_y_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(golden_y[i]);
              
              diff = std::abs(dut_output_float - golden_y_float);
              percent_diff = diff * 100 / (std::abs(golden_y[i]));
              accumulated_percent_diff = accumulated_percent_diff + percent_diff;
              cout << " Dut output, golden output and percentage difference: " << dut_output_float << " " << golden_y_float << " " << percent_diff << "%" << endl;
              if (diff > max_diff) max_diff = diff;
              if (diff > 1e-1) { // Looser tolerance for basic test
                  cout << "MISMATCH at index " << i << ": DUT=" << dut_output_float << ", Golden=" << golden_y_float << endl;
                  passed = false;
              }
          }
          cout << "Max difference: " << max_diff << endl;
          //cout << "Percent difference: " << accumulated_percent_diff/dut_output.size() << endl;
      

          if (passed) {
            cout << endl;
            cout << "Max difference: " << max_diff << " is less than threshold: " << 1e-1 << endl;
            cout << "TESTBENCH PASSED" << endl;
          } else {
            cout << "TESTBENCH FAILED" << endl;
            SC_REPORT_ERROR("testbench", "TESTBENCH FAILED");
            sc_stop();
          }
          sc_stop();
      }
      wait();
    }
  }


};

SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);
  sc_clock clk;
  sc_signal<bool> rst;

  Connections::Combinational<spec::StreamType> input_port;
  Connections::Combinational<bool> start;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::Combinational<spec::StreamType> output_port;
  Connections::Combinational<bool> done;

  NVHLS_DESIGN(PEModule) dut;
  Source  source;
  Dest    dest;

  testbench(sc_module_name name)
  : sc_module(name),
    clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
    rst("rst"),
    dut("dut"),
    source("source"),
    dest("dest")
   {
     dut.clk(clk);
     dut.rst(rst);
     dut.input_port(input_port);
     dut.start(start);
     dut.rva_in(rva_in);
     dut.rva_out(rva_out);
     dut.output_port(output_port);
     dut.done(done);

     source.clk(clk);
     source.rst(rst);
     source.input_port(input_port);
     source.start(start);
     source.rva_in(rva_in);

     dest.clk(clk);
     dest.rst(rst);
     dest.rva_out(rva_out);
     dest.output_port(output_port);
     dest.done(done);

     SC_THREAD(run);
   }

  void run(){
    wait(2, SC_NS );
    std::cout << "@" << sc_time_stamp() <<" Asserting reset" << std::endl;
    rst.write(false);
    wait(10, SC_NS );
    rst.write(true);
    std::cout << "@" << sc_time_stamp() <<" De-Asserting reset" << std::endl;
    wait(5000, SC_NS );
    cout << "Error: Simulation timed out!" << endl;
    SC_REPORT_ERROR("testbench", "Simulation timeout");
    sc_stop();
  }
};

int sc_main(int argc, char *argv[]) {
  nvhls::set_random_seed();
  testbench tb("tb");
  sc_report_handler::set_actions(SC_ERROR, SC_DISPLAY);
  sc_start();
  bool rc = (sc_report_handler::get_count(SC_ERROR) > 0);
  if (rc)
    DCOUT("TESTBENCH FAIL" << endl);
  return rc;
}
