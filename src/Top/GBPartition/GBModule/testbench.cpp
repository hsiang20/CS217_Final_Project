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
//   (a) Write GBCore memory config (3 managers: src=0, dst_naive=1, dst_diag=2).
//   (b) Write source matrix to SRAM.
//   (c) Run transpose opcode 0 (naive): config, start, wait gb_done, read back from dst 1, verify.
//   (d) Run transpose opcode 1 (diagonal): config, start, wait gb_done, read back from dst 2, verify.
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
    //   Memory 0: num_vector=2, base=0   (source)
    //   Memory 1: num_vector=3, base=64  (naive result)
    //   Memory 2: num_vector=3, base=128 (diagonal result)
    // -----------------------------------------------------------------------
    NVUINTW(128) gbcore_cfg = 0;
    gbcore_cfg.set_slc<8>(0,   NVUINT8(2));    // num_vector[0] = 2
    gbcore_cfg.set_slc<16>(16, NVUINT16(0));   // base[0] = 0
    gbcore_cfg.set_slc<8>(32,  NVUINT8(3));    // num_vector[1] = 3
    gbcore_cfg.set_slc<16>(48, NVUINT16(64));  // base[1] = 64
    gbcore_cfg.set_slc<8>(64,  NVUINT8(3));    // num_vector[2] = 3
    gbcore_cfg.set_slc<16>(80, NVUINT16(128)); // base[2] = 128

    NVUINTW(24) cfg_addr = 0;
    cfg_addr.set_slc<4>(20, NVUINT4(0x4));
    cfg_addr.set_slc<16>(4, NVUINT16(0x01));
    push_rva_write(cfg_addr, gbcore_cfg);
    cout << sc_time_stamp() << " [Source] GBCore config written (3 managers)" << endl;

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
    // (c) Test 1: Naive transpose (opcode 0) -> dst manager 1 (base 64)
    //   bits[50:48] = opcode = 0
    // -----------------------------------------------------------------------
    NVUINTW(128) xp_cfg = 0;
    xp_cfg.set_slc<1>(0,  NVUINT1(1));   // is_valid
    xp_cfg.set_slc<3>(8,  NVUINT3(0));   // memory_index_src
    xp_cfg.set_slc<3>(16, NVUINT3(1));   // memory_index_dst
    xp_cfg.set_slc<8>(32, NVUINT8(3));   // num_rows
    xp_cfg.set_slc<8>(40, NVUINT8(2));   // num_cols
    xp_cfg.set_slc<3>(48, NVUINT3(0));   // opcode = 0 (naive)

    NVUINTW(24) xp_cfg_addr = 0;
    xp_cfg_addr.set_slc<4>(20, NVUINT4(0x6));
    xp_cfg_addr.set_slc<16>(4, NVUINT16(0x01));
    push_rva_write(xp_cfg_addr, xp_cfg);
    cout << sc_time_stamp() << " [Source] Transpose config written (opcode=0 naive)" << endl;

    NVUINTW(24) start_addr = 0;
    start_addr.set_slc<16>(4, NVUINT16(0x3));
    push_rva_write(start_addr, 0);
    cout << sc_time_stamp() << " [Source] Transpose start (naive)" << endl;

    wait_for_gb_done();
    cout << sc_time_stamp() << " [Source] gb_done (naive), reading back from dst 1" << endl;

    push_rva_read(sram_addr(64));
    push_rva_read(sram_addr(80));
    push_rva_read(sram_addr(96));
    push_rva_read(sram_addr(65));
    push_rva_read(sram_addr(81));
    push_rva_read(sram_addr(97));

    // -----------------------------------------------------------------------
    // (d) Test 2: Diagonal transpose (opcode 1) -> dst manager 2 (base 128)
    // -----------------------------------------------------------------------
    g_gb_done = false;  // allow second wait_for_gb_done
    xp_cfg.set_slc<3>(16, NVUINT3(2));   // memory_index_dst = 2
    xp_cfg.set_slc<3>(48, NVUINT3(1));   // opcode = 1 (diagonal)
    push_rva_write(xp_cfg_addr, xp_cfg);
    cout << sc_time_stamp() << " [Source] Transpose config written (opcode=1 diagonal)" << endl;
    push_rva_write(start_addr, 0);
    cout << sc_time_stamp() << " [Source] Transpose start (diagonal)" << endl;

    wait_for_gb_done();
    cout << sc_time_stamp() << " [Source] gb_done (diagonal), reading back from dst 2" << endl;

    push_rva_read(sram_addr(128));
    push_rva_read(sram_addr(144));
    push_rva_read(sram_addr(160));
    push_rva_read(sram_addr(129));
    push_rva_read(sram_addr(145));
    push_rva_read(sram_addr(161));
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

  static const int kNumReadsPerTest = 6;
  static const int kNumTests        = 2;  // naive then diagonal
  uint8_t expected_val[kNumReadsPerTest] = {0x01, 0x03, 0x05, 0x02, 0x04, 0x06};

  void RunReadVerify() {
    rva_out.Reset();
    gb_done.Reset();
    wait();

    for (int test = 0; test < kNumTests; test++) {
      while (1) {
        bool d;
        if (gb_done.PopNB(d)) {
          cout << sc_time_stamp() << " [Dest]   gb_done received (test " << test << ")" << endl;
          g_gb_done = true;
          break;
        }
        wait();
      }

      for (int i = 0; i < kNumReadsPerTest; i++) {
        spec::Axi::SubordinateToRVA::Read rsp;
        while (!rva_out.PopNB(rsp)) wait();
        spec::VectorType actual = rsp.data;
        spec::VectorType expected = make_uniform_vector(expected_val[i]);
        int read_idx = test * kNumReadsPerTest + i;
        if (!(actual == expected)) {
          cout << sc_time_stamp() << " [Dest]   MISMATCH read " << read_idx
               << " (test " << test << "): expected 0x" << std::hex << (int)expected_val[i]
               << " got " << actual << endl;
          SC_REPORT_ERROR("Transpose", "read-back mismatch");
          all_reads_ok = false;
        } else {
          cout << sc_time_stamp() << " [Dest]   Read " << read_idx << " OK (0x"
               << std::hex << (int)expected_val[i] << ")" << endl;
        }
      }
      if (test == 0) g_gb_done = false;  // allow Source to proceed to second transpose
    }

    cout << sc_time_stamp() << " [Dest]   All " << (kNumReadsPerTest * kNumTests) << " reads verified (naive + diagonal)." << endl;
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

    // Timeout: 2 tests × (6 elements × ~5 cycles) + overhead
    wait(10000, SC_NS);
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
