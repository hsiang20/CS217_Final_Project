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
// GBModule Transpose Testbench
// =============================================================================
//
// Verifies the Transpose hardware module operating through GBModule.
//
// Matrix layout (VectorType = 16 uint8 scalars per entry):
//
//   Source A  [3 rows × 2 cols]:
//     A[0][0] = all 0x01   A[0][1] = all 0x02
//     A[1][0] = all 0x03   A[1][1] = all 0x04
//     A[2][0] = all 0x05   A[2][1] = all 0x06
//
//   Expected A^T  [2 rows × 3 cols]:
//     A^T[0][0]=A[0][0]=0x01   A^T[0][1]=A[1][0]=0x03   A^T[0][2]=A[2][0]=0x05
//     A^T[1][0]=A[0][1]=0x02   A^T[1][1]=A[1][1]=0x04   A^T[1][2]=A[2][1]=0x06
//
// GBCore SRAM addressing (base_large[i], num_vector_large[i]):
//   Memory 0 (src): base=0,  num_vector=2  → src SRAM addresses 0,1,2,16,17,18
//   Memory 1 (dst): base=64, num_vector=3  → dst SRAM addresses 64,65,66,80,81,82
//
// Test steps:
//   (a) Write GBCore memory config.
//   (b) Write source matrix elements to SRAM via AXI direct writes.
//   (c) Write Transpose config.
//   (d) Start Transpose (region 0x0, local_index 0x3).
//   (e) Wait for gb_done.
//   (f) Read back transposed elements and verify.
// =============================================================================

#include "GBModule.h"

#include <mc_scverify.h>
#include <nvhls_connections.h>
#include <systemc.h>
#include <testbench/nvhls_rand.h>

#include <iostream>
#include <vector>

#include "GBSpec.h"
#include "Spec.h"
#include "helper.h"

#define NVHLS_VERIFY_BLOCKS (GBModule)
#include <nvhls_verify.h>

#ifdef COV_ENABLE
#pragma CTC SKIP
#endif

// ---------------------------------------------------------------------------
// Global synchronisation flag: set when gb_done is received.
// ---------------------------------------------------------------------------
bool g_gb_done = false;

// ---------------------------------------------------------------------------
// Helper: build a VectorType whose every element equals `val`.
// ---------------------------------------------------------------------------
static spec::VectorType make_uniform_vector(uint8_t val) {
  spec::VectorType v;
  for (int i = 0; i < spec::kVectorSize; i++) v[i] = val;
  return v;
}

// ---------------------------------------------------------------------------
// Helper: build the 24-bit AXI-RVA address for a GBCore SRAM direct access.
//   bits[23:20] = region = 0x5
//   bits[19:4]  = sram_addr (raw SRAM index)
// ---------------------------------------------------------------------------
static NVUINTW(24) sram_addr(NVUINT16 idx) {
  NVUINTW(24) a = 0;
  a.set_slc<4>(20, NVUINT4(0x5));
  a.set_slc<16>(4, idx);
  return a;
}

// ---------------------------------------------------------------------------
// Source: drives all RVA write commands, then signals when done.
// ---------------------------------------------------------------------------
SC_MODULE(Source) {
  sc_in<bool>  clk;
  sc_in<bool>  rst;
  // AXI-RVA write interface (config + data)
  Connections::Out<spec::Axi::SubordinateToRVA::Write> rva_in;
  // Stub ports that GBControl/PE interface needs (unused in this test)
  Connections::Out<spec::StreamType> data_in;
  Connections::Out<bool>             pe_done;

  SC_CTOR(Source) {
    SC_THREAD(run);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  // Helper: block until gb_done is received by Dest
  void wait_for_gb_done() {
    while (!g_gb_done) wait();
  }

  void push_rva_write(NVUINTW(24) addr, NVUINTW(128) data) {
    spec::Axi::SubordinateToRVA::Write cmd;
    cmd.rw   = 1;
    cmd.addr = addr;
    cmd.data = data;
    rva_in.Push(cmd);
    wait();
  }

  void push_rva_read(NVUINTW(24) addr) {
    spec::Axi::SubordinateToRVA::Write cmd;
    cmd.rw   = 0;
    cmd.addr = addr;
    cmd.data = 0;
    rva_in.Push(cmd);
  }

  void run() {
    rva_in.Reset();
    data_in.Reset();
    pe_done.Reset();
    wait();

    // -----------------------------------------------------------------------
    // (a) GBCore memory config (region 0x4, local_index 0x01)
    //   Memory 0: num_vector=2, base=0
    //   Memory 1: num_vector=3, base=64
    //   128-bit layout: bits[32*i+7:32*i] = num_vector[i]
    //                   bits[32*i+31:32*i+16] = base[i]
    // -----------------------------------------------------------------------
    NVUINTW(128) gbcore_cfg = 0;
    gbcore_cfg.set_slc<8>(0,  NVUINT8(2));   // num_vector[0] = 2
    gbcore_cfg.set_slc<16>(16, NVUINT16(0)); // base[0] = 0
    gbcore_cfg.set_slc<8>(32, NVUINT8(3));   // num_vector[1] = 3
    gbcore_cfg.set_slc<16>(48, NVUINT16(64));// base[1] = 64

    NVUINTW(24) cfg_addr = 0;
    cfg_addr.set_slc<4>(20, NVUINT4(0x4));
    cfg_addr.set_slc<16>(4, NVUINT16(0x01));
    push_rva_write(cfg_addr, gbcore_cfg);
    cout << sc_time_stamp() << " [Source] GBCore config written" << endl;

    // -----------------------------------------------------------------------
    // (b) Write source matrix A[row][col] to SRAM via direct AXI writes.
    //
    // Physical SRAM address formula (base=0, num_vec=2):
    //   addr = base + row%16 + (row/16 * num_vec + col) * 16
    //   A[0][0] → 0    A[0][1] → 16
    //   A[1][0] → 1    A[1][1] → 17
    //   A[2][0] → 2    A[2][1] → 18
    // -----------------------------------------------------------------------
    push_rva_write(sram_addr(0),  make_uniform_vector(0x01).to_rawbits()); // A[0][0]
    push_rva_write(sram_addr(16), make_uniform_vector(0x02).to_rawbits()); // A[0][1]
    push_rva_write(sram_addr(1),  make_uniform_vector(0x03).to_rawbits()); // A[1][0]
    push_rva_write(sram_addr(17), make_uniform_vector(0x04).to_rawbits()); // A[1][1]
    push_rva_write(sram_addr(2),  make_uniform_vector(0x05).to_rawbits()); // A[2][0]
    push_rva_write(sram_addr(18), make_uniform_vector(0x06).to_rawbits()); // A[2][1]
    cout << sc_time_stamp() << " [Source] Source matrix written to SRAM" << endl;

    // -----------------------------------------------------------------------
    // (c) Write Transpose config (region 0x6, local_index 0x01)
    //   bits[0]     = is_valid = 1
    //   bits[10:8]  = memory_index_src = 0
    //   bits[18:16] = memory_index_dst = 1
    //   bits[39:32] = num_rows = 3
    //   bits[47:40] = num_cols = 2
    // -----------------------------------------------------------------------
    NVUINTW(128) xp_cfg = 0;
    xp_cfg.set_slc<1>(0,  NVUINT1(1));  // is_valid
    xp_cfg.set_slc<3>(8,  NVUINT3(0));  // memory_index_src
    xp_cfg.set_slc<3>(16, NVUINT3(1));  // memory_index_dst
    xp_cfg.set_slc<8>(32, NVUINT8(3));  // num_rows
    xp_cfg.set_slc<8>(40, NVUINT8(2));  // num_cols

    NVUINTW(24) xp_cfg_addr = 0;
    xp_cfg_addr.set_slc<4>(20, NVUINT4(0x6));
    xp_cfg_addr.set_slc<16>(4, NVUINT16(0x01));
    push_rva_write(xp_cfg_addr, xp_cfg);
    cout << sc_time_stamp() << " [Source] Transpose config written" << endl;

    // -----------------------------------------------------------------------
    // (d) Start Transpose (region 0x0, local_index 0x3)
    // -----------------------------------------------------------------------
    NVUINTW(24) start_addr = 0;
    start_addr.set_slc<16>(4, NVUINT16(0x3));
    push_rva_write(start_addr, 0);
    cout << sc_time_stamp() << " [Source] Transpose start issued" << endl;

    // -----------------------------------------------------------------------
    // (e) Wait for gb_done before issuing verification reads
    // -----------------------------------------------------------------------
    wait_for_gb_done();
    cout << sc_time_stamp() << " [Source] gb_done received, reading back transposed data" << endl;

    // -----------------------------------------------------------------------
    // (f) Read back A^T from dst memory (memory 1, base=64, num_vec=3).
    //
    // Physical dst SRAM addresses:
    //   A^T[0][0]=A[0][0] written at (time=0,vec=0): 64+0+(0*3+0)*16 = 64
    //   A^T[0][1]=A[1][0] written at (time=0,vec=1): 64+0+(0*3+1)*16 = 80
    //   A^T[0][2]=A[2][0] written at (time=0,vec=2): 64+0+(0*3+2)*16 = 96
    //   A^T[1][0]=A[0][1] written at (time=1,vec=0): 64+1+(0*3+0)*16 = 65
    //   A^T[1][1]=A[1][1] written at (time=1,vec=1): 64+1+(0*3+1)*16 = 81
    //   A^T[1][2]=A[2][1] written at (time=1,vec=2): 64+1+(0*3+2)*16 = 97
    // -----------------------------------------------------------------------
    push_rva_read(sram_addr(64));
    push_rva_read(sram_addr(80));
    push_rva_read(sram_addr(96));
    push_rva_read(sram_addr(65));
    push_rva_read(sram_addr(81));
    push_rva_read(sram_addr(97));
  }
};

// ---------------------------------------------------------------------------
// Dest: drains AXI read responses and verifies the transposed matrix.
//       Also drains pe_start / data_out (unused in this test).
// ---------------------------------------------------------------------------
SC_MODULE(Dest) {
  sc_in<bool>  clk;
  sc_in<bool>  rst;
  Connections::In<spec::Axi::SubordinateToRVA::Read> rva_out;
  Connections::In<bool>              gb_done;
  // Unused PE interface ports (GBControl still exists; drain them)
  Connections::In<bool>              pe_start;
  Connections::In<spec::StreamType>  data_out;

  bool all_reads_ok = true;

  SC_CTOR(Dest) {
    SC_THREAD(RunReadVerify);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);

    SC_THREAD(RunDrain);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  // Expected values for the 6 read-back positions (same order as Source's reads)
  static const int kNumReads = 6;
  uint8_t expected_val[kNumReads] = {0x01, 0x03, 0x05, 0x02, 0x04, 0x06};

  void RunReadVerify() {
    rva_out.Reset();
    gb_done.Reset();
    wait();

    // First wait until Transpose signals gb_done
    while (1) {
      bool d;
      if (gb_done.PopNB(d)) {
        cout << sc_time_stamp() << " [Dest]   gb_done received" << endl;
        g_gb_done = true;
        break;
      }
      wait();
    }

    // Then collect and verify all 6 AXI read responses
    int read_idx = 0;
    while (read_idx < kNumReads) {
      spec::Axi::SubordinateToRVA::Read rsp;
      if (rva_out.PopNB(rsp)) {
        spec::VectorType actual;
        actual = rsp.data;
        spec::VectorType expected = make_uniform_vector(expected_val[read_idx]);

        bool match = (actual == expected);
        if (!match) {
          cout << sc_time_stamp() << " [Dest]   MISMATCH at read " << read_idx
               << ": expected all 0x" << std::hex << (int)expected_val[read_idx]
               << " got " << actual << endl;
          SC_REPORT_ERROR("Transpose", "read-back mismatch");
          all_reads_ok = false;
        } else {
          cout << sc_time_stamp() << " [Dest]   Read " << read_idx
               << " OK: all 0x" << std::hex << (int)expected_val[read_idx] << endl;
        }
        read_idx++;
      }
      wait();
    }

    cout << sc_time_stamp() << " [Dest]   All reads verified." << endl;
    sc_stop();
  }

  static spec::VectorType make_uniform_vector(uint8_t val) {
    spec::VectorType v;
    for (int i = 0; i < spec::kVectorSize; i++) v[i] = val;
    return v;
  }

  // Drain pe_start and data_out so they never block
  void RunDrain() {
    pe_start.Reset();
    data_out.Reset();
    wait();
    while (1) {
      bool         s;
      spec::StreamType d;
      pe_start.PopNB(s);
      data_out.PopNB(d);
      wait();
    }
  }
};

// ---------------------------------------------------------------------------
// Top-level testbench
// ---------------------------------------------------------------------------
SC_MODULE(testbench) {
  SC_HAS_PROCESS(testbench);

  sc_clock        clk;
  sc_signal<bool> rst;

  // AXI-RVA channels
  Connections::Combinational<spec::Axi::SubordinateToRVA::Write> rva_in;
  Connections::Combinational<spec::Axi::SubordinateToRVA::Read>  rva_out;

  // GBControl <-> PE stub channels
  Connections::Combinational<spec::StreamType> data_in;
  Connections::Combinational<spec::StreamType> data_out;
  Connections::Combinational<bool>             pe_start;
  Connections::Combinational<bool>             pe_done;

  // Done signal from GBModule
  Connections::Combinational<bool> gb_done;

  NVHLS_DESIGN(GBModule) dut;
  Source source;
  Dest   dest;

  testbench(sc_module_name name)
      : sc_module(name),
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
    source.data_in(data_in);
    source.pe_done(pe_done);

    dest.clk(clk);
    dest.rst(rst);
    dest.rva_out(rva_out);
    dest.gb_done(gb_done);
    dest.pe_start(pe_start);
    dest.data_out(data_out);

    SC_THREAD(run);
  }

  void run() {
    wait(2, SC_NS);
    cout << "@" << sc_time_stamp() << " Asserting reset" << endl;
    rst.write(false);
    wait(2, SC_NS);
    rst.write(true);
    cout << "@" << sc_time_stamp() << " De-asserting reset" << endl;

    // Generous timeout: 6 elements × ~5 cycles each + overhead
    wait(5000, SC_NS);
    cout << sc_time_stamp() << " ERROR: Simulation timed out!" << endl;
    SC_REPORT_ERROR("testbench", "Simulation timeout");
    sc_stop();
  }
};

int sc_main(int argc, char* argv[]) {
  nvhls::set_random_seed();

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
