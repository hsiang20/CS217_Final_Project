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

#include <systemc.h>
#include <mc_scverify.h>
#include <testbench/nvhls_rand.h>
#include <nvhls_connections.h>
#include <map>
#include <vector>
#include <deque>
#include <utility>
#include <sstream>
#include <string>
#include <cstdlib>
#include <math.h> // testbench only
#include <queue>

#include "ActUnit.h"
#include "AxiSpec.h"

#include "helper.h"

#include <iostream>
#include <sstream>
#include <iomanip>


#define NVHLS_VERIFY_BLOCKS (ActUnit)
#include <nvhls_verify.h>

#ifdef COV_ENABLE
   #pragma CTC SKIP
#endif

SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;  
  Connections::Out<spec::ActVectorType> act_port;
  Connections::Out<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Out<spec::ActVectorType> expected_output;

  Connections::Out<bool> start;    

  bool start_src;
  spec::ActVectorType act_port_src;
  spec::ActVectorType test_in[16];
  spec::ActVectorType expected_out[16];
 
  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void Reset() {
    act_port.Reset();
    rva_in.Reset();
    start.Reset();
    expected_output.Reset();
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


  
  void run(){
    Reset();
    spec::Axi::SubordinateToRVA::Write  rva_in_src;

    for (int i = 0; i < 16; i++) {
      for (int j = 0; j < spec::kNumVectorLanes; j++) {
        test_in[i][j] = nvhls::get_rand<spec::kActWordWidth>();
      }
    }

    wait(); 

    //Axi Config 0x01
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<16>("00_00_00_00_00_00_00_00_00_00_01_01_0A_04_00_01"); 
    rva_in_src.addr = set_bytes<3>("80_00_10");  // last 4 bits never used 
    rva_in.Push(rva_in_src);
    wait(); 

    //inpe inst_reg[00] --> tanh actregs[00] --> and output_port --> inpe inst_reg[01] --> silu actregs[01] --> output_port --> EADD actregs[01] --> output_port --> EMUL actregs [01] --> output_port
    //gelu actregs[01] --> output_port --> relu actregs[01] --> output_port
    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<16>("00_00_00_00_00_00_44_C4_44_F4_44_E4_34_40_B0_30");
    rva_in_src.addr = set_bytes<3>("80_00_20");  // last 4 bits never used 
    rva_in.Push(rva_in_src);
    wait();

    rva_in_src.rw = 1;
    rva_in_src.data = set_bytes<16>("00_00_00_00_00_00_00_00_00_00_00_4C_1C_24_44_D4");
    rva_in_src.addr = set_bytes<3>("80_00_30");  // last 4 bits never used 
    rva_in.Push(rva_in_src);
    wait();

    //start signal Poped
    start_src = 1;
    start.Push(start_src);
    wait();

    // Tanh
    cout << "\nTest Tanh" << endl;
    act_port.Push(test_in[0]);
    Tanh_ref(test_in[0], expected_out[0]);
    expected_output.Push(expected_out[0]);
    wait(5);

    // Silu
    cout << "\nTest Silu" << endl;
    act_port.Push(test_in[1]);
    Silu_ref(test_in[1], expected_out[1]);
    expected_output.Push(expected_out[1]);
    wait(5);

    // Gelu
    cout << "\nTest Gelu" << endl;
    Gelu_ref(expected_out[1], expected_out[2]);
    expected_output.Push(expected_out[2]);
    wait(2);

    // Relu
    cout << "\nTest Relu" << endl;
    Relu_ref(expected_out[2], expected_out[3]);
    expected_output.Push(expected_out[3]);
    wait(2);

    wait(100);
   
  }// void run()

};

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::In<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::In<spec::StreamType> output_port; 
  Connections::In<spec::ActVectorType> expected_output;
  Connections::In<bool> done;

  spec::StreamType output_port_dest;
  spec::Axi::SubordinateToRVA::Read rva_out_dest;
  spec::ActVectorType expected_output_dest;

  int matches = 0;
  int mismatches = 0;

  SC_CTOR(Dest) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void Reset() {
    rva_out.Reset();
    output_port.Reset();
    expected_output.Reset();
    done.Reset();
  }
  
  void run() {
    Reset();
    wait();
    
    const float kTolerance = 10.0; 
    const float mseTolerance = 2.0;

    while (1) {
      if (rva_out.PopNB(rva_out_dest)) {
        cout << hex << sc_time_stamp() << " Dest rva data = " << rva_out_dest.data << endl;
      }
      if (output_port.PopNB(output_port_dest) && expected_output.PopNB(expected_output_dest)) {
        cout << hex << sc_time_stamp() << " output_port data = " << output_port_dest.data << endl;
        cout << hex << sc_time_stamp() << " expected_output data = " << expected_output_dest << endl;
        bool match = true;
        float total_percent_diff = 0.0;
        float total_diff = 0.0;
        for (int i = 0; i < spec::kNumVectorLanes; i++) {
          float actual_float = fixed2float<spec::kActWordWidth, spec::kActNumFrac>(expected_output_dest[i]);
          float pwl_float = fixed2float<spec::kIntWordWidth, spec::Act::kActOutNumFrac>(output_port_dest.data[i]);

          float diff = std::abs(actual_float - pwl_float);
          float percent_diff = 0.0;
          total_diff += diff*diff;

          if (abs(actual_float) < 1){
            percent_diff = (diff) * 100.0;
          } else {
            percent_diff = (diff / std::abs(actual_float)) * 100.0; 
          }
          
          total_percent_diff += percent_diff;
          
          /*cout << "Index " << i << ": PWL=" << pwl_float << ", Actual=" << actual_float 
               << ", Diff=" << diff << ", %Diff=" << percent_diff << "%" << endl;

          if (percent_diff > kTolerance) {
            match = false;
            cout << "MISMATCH detected with tolerance " << percent_diff << "%" << endl;
          }*/
        }
        float average_percent_diff = total_percent_diff / spec::kNumVectorLanes;
        if (average_percent_diff > kTolerance) {
          match = false;
        }
        float average_mse = 100 * total_diff / spec::kNumVectorLanes;
        if (average_mse > mseTolerance) {
          match = false;
        }
        
        cout << "\tAverage % Difference: " << average_percent_diff << "%" << endl;
        cout << "\tMSE %: " << (average_mse) << "%" << endl;
        if (match) {
          matches++;
        } else {
          mismatches++;
        }
      }
      bool done_val;
      if (done.PopNB(done_val)) {
        cout << "\n" << endl;
        cout << "SIMULATION DONE" << endl;
        cout << "Matches: " << matches << endl;
        cout << "Mismatches: " << mismatches << endl;
        if (mismatches > 0) {
          cout << "TEST FAILED" << endl;
        } else {
          cout << "TEST PASSED" << endl;
        }
        sc_stop();
      }
      wait();    
    } // while
  } //run

};


SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);
	sc_clock clk;
  sc_signal<bool> rst;

  Connections::Combinational<spec::ActVectorType> act_port;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::Combinational<spec::StreamType> output_port; 
  Connections::Combinational<spec::ActVectorType> expected_output;
  
  Connections::Combinational<bool> start;
  Connections::Combinational<bool> done;

  NVHLS_DESIGN(ActUnit) dut;
  //ActUnit dut;
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
	  dut.act_port(act_port);
		dut.rva_in(rva_in);
		dut.rva_out(rva_out);		
		dut.output_port(output_port);
    dut.start(start);
    dut.done(done);		
    
    source.clk(clk);
    source.rst(rst); 
	  source.act_port(act_port);
		source.rva_in(rva_in);
    source.start(start);
    source.expected_output(expected_output);
    		
		dest.clk(clk);
		dest.rst(rst);
		dest.rva_out(rva_out);
		dest.output_port(output_port);
    dest.expected_output(expected_output);
    dest.done(done);		
    
    SC_THREAD(run);
  }
  
  
  void run(){
	  wait(2, SC_NS );
    std::cout << "@" << sc_time_stamp() <<" Asserting reset" << std::endl;
    rst.write(false);
    wait(2, SC_NS );
    rst.write(true);
    std::cout << "@" << sc_time_stamp() <<" De-Asserting reset" << std::endl;
    wait(10000, SC_NS );
    std::cout << "@" << sc_time_stamp() <<" sc_stop" << std::endl;
    sc_stop();
  }
};





int sc_main(int argc, char *argv[]) {
  nvhls::set_random_seed();
  /*NVINT8 test = 14;
  cout << fixed2float<8, 3>(test) << endl;*/

  
  testbench tb("tb");
  
  sc_report_handler::set_actions(SC_ERROR, SC_DISPLAY);
  sc_start();



  bool rc = (sc_report_handler::get_count(SC_ERROR) > 0);
  if (rc)
    DCOUT("TESTBENCH FAIL" << endl);
  else
    DCOUT("TESTBENCH PASS" << endl);
  return rc;
}
