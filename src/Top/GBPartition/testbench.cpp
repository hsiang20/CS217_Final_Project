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
#include "GBPartition.h"

#include <iostream>
#include <sstream>
#include <iomanip>

#define NVHLS_VERIFY_BLOCKS (GBPartition)
#include <nvhls_verify.h>

#ifdef COV_ENABLE
   #pragma CTC SKIP
#endif

bool correct = true;
bool axiManagerDone = false;


SC_MODULE(Source) {
  sc_in<bool> clk;
  sc_in<bool> rst;  
  Connections::Out<spec::StreamType>  data_in;
  Connections::Out<bool>              pe_done;
  
  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

  }
  
  void run(){
    data_in.Reset();
    pe_done.Reset();
    // Just keep the stub alive; the ManagerFromFile drives all AXI commands
    while (1) wait();
  }
};

SC_MODULE(Dest) {
  sc_in<bool> clk;
  sc_in<bool> rst;
  Connections::In<bool>              pe_start;
  Connections::In<bool>              done;
  Connections::In<spec::StreamType>  data_out;

  bool done_PopDone = false;

  SC_CTOR(Dest) {
    SC_THREAD(DrainPorts);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(PopDone);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(SimStop);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  // Drain pe_start and data_out so they never block
  void DrainPorts() {
    pe_start.Reset();
    data_out.Reset();
    wait();
    while (1) {
      bool s; spec::StreamType d;
      pe_start.PopNB(s);
      data_out.PopNB(d);
      wait();
    }
  }

  void PopDone() {
    done.Reset();
    wait();
    while (1) {
      bool d;
      if (done.PopNB(d)) {
        cout << sc_time_stamp() << " gb_done received!" << endl;
        done_PopDone = true;
      }
      wait();
    }
  }

  void SimStop() {
    wait();
    while (1) {
      wait();
      if (done_PopDone && axiManagerDone) {
        sc_stop();
      }
    }
  }
};



SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);
  ManagerFromFile<spec::Axi::axiCfg> master;

  sc_clock clk;
  sc_signal<bool> rst;
  sc_signal<bool> master_done;

  //typename axi::axi4<spec::Axi::axiCfg>::read::chan axi_read;
  //typename axi::axi4<spec::Axi::axiCfg>::write::chan axi_write;
  
  Connections::Combinational<spec::StreamType>  data_in;
  Connections::Combinational<spec::StreamType>  data_out;
  Connections::Combinational<bool>              pe_done;  
  Connections::Combinational<bool>              done;  
  Connections::Combinational<bool>            pe_start;


  NVHLS_DESIGN(GBPartition) dut;
  Source  source;
  Dest    dest;

  typename axi::axi4<spec::Axi::axiCfg>::read::template chan<> axi_read;
  typename axi::axi4<spec::Axi::axiCfg>::write::template chan<> axi_write;

  
  testbench(sc_module_name name)
  : sc_module(name),
     master("master", "./axi_commands_test.csv"),
     clk("clk", 1.0, SC_NS, 0.5, 0, SC_NS, true),
     rst("rst"),
     dut("dut"),
     source("source"),
     dest("dest"),
     axi_read("axi_read"),
     axi_write("axi_write") {
     
    dut.clk(clk);
    dut.rst(rst);
    dut.if_axi_wr(axi_write);
    dut.if_axi_rd(axi_read);
    dut.data_in(data_in);
    dut.data_out(data_out);
    dut.pe_done(pe_done);
    dut.gb_done(done);
    dut.pe_start(pe_start);

    master.clk(clk);
    master.reset_bar(rst);
    master.done(master_done);
    master.if_rd(axi_read);
    master.if_wr(axi_write);
    
    source.clk(clk);
    source.rst(rst);
    source.data_in(data_in);
    source.pe_done(pe_done);

    dest.clk(clk);
    dest.rst(rst);
    dest.pe_start(pe_start);
    dest.done(done);
    dest.data_out(data_out);
    				
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
      if (master_done==1) {
        cout << "Manager has finished issuing AXI Writes" << endl;
        axiManagerDone = true;
        break;
      }
    }

    wait(100000, SC_NS );
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
