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
#include <ac_reset_signal_is.h>
#include "axi/axi4.h"
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

#include "axi/testbench/ManagerFromFile.h"
#include "Spec.h"
#include "AxiSpec.h"

#include "helper.h"
#include "PEPartition.h"

#include <iostream>
#include <sstream>
#include <iomanip>

#define NVHLS_VERIFY_BLOCKS (PEPartition)
#include <nvhls_verify.h>

#ifdef COV_ENABLE
   #pragma CTC SKIP
#endif

bool correct = true;
bool axiManagerDone = false;

SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;  
  Connections::Out<spec::StreamType>  input_port;
  Connections::Out<bool>              start;  
  
  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

  }
  
  void run(){
    input_port.Reset();
    start.Reset();

    wait(1000);
    while(1){
      wait();
    }
  } //run
};

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::In<bool>              done;
  Connections::In<spec::StreamType>  output_port;

  spec::StreamType output_port_dest;
  bool done_dest;

  bool output_port_popped = false;
  bool done_signal_received = false;


  SC_CTOR(Dest) {
    SC_THREAD(PopOutport);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(PopDone);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(SimStop); 
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }  

  void PopOutport() {
   output_port.Reset();
   wait();
 
   while (1) {
     if (output_port.PopNB(output_port_dest)) {
        cout << "Design output_port result: " << std::hex << output_port_dest.data << endl;
        output_port_popped = true;
     }
     wait(); 
   } // while
   
  } //PopOutputport

  void PopDone() {
   done.Reset();
   wait();
 
   while (1) {
     if (done.PopNB(done_dest)) {
        cout << "Design done result: " << done_dest << endl;  
        done_signal_received = true;
     }
     wait(); 
   } // while
   
  } //PopDone

  void SimStop() {
    wait ();
    while(1) {
      wait();
      if (output_port_popped && done_signal_received && axiManagerDone) {
        sc_stop(); 
      }
    }
  }

};



SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);
  ManagerFromFile<spec::Axi::axiCfg> manager;

  sc_clock clk;
  sc_signal<bool> rst;
  sc_signal<bool> manager_done;

  //typename axi::axi4<spec::Axi::axiCfg>::read::chan axi_read;
  //typename axi::axi4<spec::Axi::axiCfg>::write::chan axi_write;
  
  Connections::Combinational<spec::StreamType>  input_port;
  Connections::Combinational<spec::StreamType>  output_port;
  Connections::Combinational<bool>              done;  
  Connections::Combinational<bool>            start;


  NVHLS_DESIGN(PEPartition) dut;
  Source  source;
  Dest    dest;
  
  typename axi::axi4<spec::Axi::axiCfg>::read::template chan<> axi_read;
  typename axi::axi4<spec::Axi::axiCfg>::write::template chan<> axi_write;
  
  testbench(sc_module_name name)
  : sc_module(name),
     manager("manager", "./axi_commands_test.csv"),
     clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
     rst("rst"),
     dut("dut"),
     source("source"),
     dest("dest"),
     axi_read("axi_read"),
     axi_write("axi_write"){
     
    dut.clk(clk);
    dut.rst(rst);
    dut.if_axi_wr(axi_write);
    dut.if_axi_rd(axi_read);
    dut.input_port(input_port);
    dut.output_port(output_port);
    dut.done(done);
    dut.start(start);

    manager.clk(clk);
    manager.reset_bar(rst);
    manager.done(manager_done);
    manager.if_rd(axi_read);
    manager.if_wr(axi_write);

    source.clk(clk);
    source.rst(rst);
    source.input_port(input_port);
    source.start(start);

    dest.clk(clk);
    dest.rst(rst);
    dest.done(done);
    dest.output_port(output_port);
    				
    SC_THREAD(run);
  }
  
  
  void run(){
    wait(2, SC_NS );
    std::cout << "@" << sc_time_stamp() <<" Asserting reset" << std::endl;
    rst.write(false);
    wait(2, SC_NS );
    rst.write(true);
    std::cout << "@" << sc_time_stamp() <<" De-Asserting reset" << std::endl;

    while (1) {
      wait(1, SC_NS);
      //cout << "manager.waddr_q.read() = " << hex << manager.addr_pld.addr << endl;
      if (manager_done==1) {      
        std::cout << "@" << sc_time_stamp() <<" AXI Manager has finished issuing AXI commands" << std::endl;
        axiManagerDone = true;
        break;
      }
    }

    wait(160000, SC_NS ); 
    // If timeout happens, test is a fail
    cout << "Error: Simulation timed out! No output popped from DUT" << endl;
    SC_REPORT_ERROR("testbench", "Simulation timeout");
    sc_stop();
  }

};

int sc_main(int argc, char *argv[]) {
  nvhls::set_random_seed();
  NVINT8 test = 14;
  cout << fixed2float<8, 3>(test) << endl;  

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

#ifdef COV_ENABLE
   #pragma CTC ENDSKIP
#endif
