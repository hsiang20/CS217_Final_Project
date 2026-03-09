`include "common_base_test.svh"
`include "design_top_defines.vh"

bit test_failed = 0;

module design_top_base_test();
import tb_type_defines_pkg::*;

  typedef struct {
    logic [31:0] addr;
    logic [127:0] data;
  } AxiWriteCommand;

  typedef struct {
    logic [31:0] addr;
    logic [127:0] data;
    logic [127:0] expected_read_data;
  } AxiReadCommand;

  task automatic ocl_wr32(input logic [ADDR_WIDTH_OCL - 1 : 0] addr, input logic [WIDTH_AXI - 1:0] data);
    tb.poke_ocl(.addr(addr), .data(data));
  endtask

  task automatic ocl_rd32(input logic [ADDR_WIDTH_OCL - 1 : 0] addr, output logic [WIDTH_AXI - 1:0] data);
    tb.peek_ocl(.addr(addr), .data(data));
  endtask

  // =========================================================================
  // AXI Write: split wide address+data across 32-bit OCL registers
  // =========================================================================
  task automatic top_write(input AxiWriteCommand write_command);
    logic [49:0] transfer_addr = {8'b0, write_command.addr, 10'b0};
    logic [144:0] transfer_data = {17'h1ffff, write_command.data};

    for (int i = 0; i < LOOP_TOP_AXI_AW; i++) begin
        logic [31:0] temp_addr;
        temp_addr = transfer_addr[i*32 +: 32];
        if (i == LOOP_TOP_AXI_AW - 1)
          temp_addr = {14'b0, transfer_addr[49:32]};
        ocl_wr32(ADDR_TOP_AXI_AW_START + i*4, temp_addr);
        #10ns;
    end
    #100ns;
    for (int i = 0; i < LOOP_TOP_AXI_W; i++) begin
        logic [31:0] temp_data;
        temp_data = transfer_data[i*32 +: 32];
        if (i == LOOP_TOP_AXI_W - 1)
          temp_data = {17'd0, transfer_data[144:128]};
        ocl_wr32(ADDR_TOP_AXI_W_START + i*4, temp_data);
        #10ns;
    end
  endtask

  // =========================================================================
  // AXI Read: send address, wait, read back data, compare with expected
  // =========================================================================
  task automatic top_read(input AxiReadCommand read_command);
    logic [49:0] transfer_addr = {8'b0, read_command.addr, 10'b0};
    logic [159:0] transfer_data;
    logic [31:0] poll_data;
    int poll_count;

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++) begin
        logic [31:0] temp_addr;
        temp_addr = transfer_addr[i*32 +: 32];
        if (i == LOOP_TOP_AXI_AR - 1)
          temp_addr = {18'd0, transfer_addr[49:32]};
        ocl_wr32(ADDR_TOP_AXI_AR_START + i*4, temp_addr);
        #10ns;
    end

    // Poll rd[0] until bridge has captured the AXI read response
    // (top_r_valid_q=0 returns 0xDEADBEEF)
    poll_data = 32'hDEADBEEF;
    poll_count = 0;
    while (poll_data == 32'hDEADBEEF && poll_count < 500) begin
        #20ns;
        ocl_rd32(ADDR_TOP_AXI_R_START, poll_data);
        poll_count++;
    end
    transfer_data[31:0] = poll_data;

    for (int i = 1; i < LOOP_TOP_AXI_R; i++) begin
        logic [31:0] temp_data;
        ocl_rd32(ADDR_TOP_AXI_R_START + i*4, temp_data);
        #10ns;
        transfer_data[i*32 +: 32] = temp_data;
    end
    read_command.data = transfer_data[137:10];

    if (read_command.data != read_command.expected_read_data) begin
      $error("MISMATCH addr=0x%h  got=0x%h  exp=0x%h",
             read_command.addr, read_command.data, read_command.expected_read_data);
      test_failed = 1'b1;
    end else begin
      $display("  PASS addr=0x%h", read_command.addr);
    end
  endtask

  // =========================================================================
  // Helpers
  // =========================================================================

  // SRAM flat address matching GBCore's SetLargeBuffer (general formula):
  //   flat = base_large[mem] + (ts % 16) + ((ts / 16) * num_vec + vec) * 16
  //   AXI  = 0x33500000 + flat * 0x10
  // With base_large = {0, 1024, 2048}, num_vec = 32
  function automatic logic [31:0] sram_addr(int mem, int vec, int ts);
    int flat;
    flat = mem * 1024 + (ts % 16) + ((ts / 16) * 32 + vec) * 16;
    return 32'h33500000 + flat[31:0] * 32'h10;
  endfunction

  // Replicate an 8-bit value across all 16 bytes of a 128-bit word
  function automatic logic [127:0] replicate8(logic [7:0] val);
    return {16{val}};
  endfunction

  // Build 128-bit TransposeConfig register value
  function automatic logic [127:0] transpose_cfg(
      int src_mem, int dst_mem, int rows, int cols, int opcode);
    logic [127:0] cfg = 128'b0;
    cfg[0]     = 1'b1;                  // is_valid
    cfg[10:8]  = src_mem[2:0];          // memory_index_src
    cfg[18:16] = dst_mem[2:0];          // memory_index_dst
    cfg[39:32] = rows[7:0];            // num_rows
    cfg[47:40] = cols[7:0];            // num_cols
    cfg[50:48] = opcode[2:0];          // opcode
    return cfg;
  endfunction

  // Convenience wrappers
  task automatic write_sram(input logic [31:0] addr, input logic [127:0] data);
    AxiWriteCommand cmd;
    cmd.addr = addr;
    cmd.data = data;
    top_write(cmd);
  endtask

  task automatic verify_sram(input logic [31:0] addr, input logic [127:0] expected);
    AxiReadCommand cmd;
    cmd.addr = addr;
    cmd.data = '0;
    cmd.expected_read_data = expected;
    top_read(cmd);
  endtask

  // =========================================================================
  // Run one transpose test: writes source, runs both opcodes, verifies results
  //
  //   Source matrix A (rows x cols) at memory_index = src_mem
  //     A[r][c] stored at SRAM(timestep=r, vector=c)
  //
  //   After transpose, A[r][c] appears at dst(timestep=c, vector=r)
  //
  //   Element value = (r*cols + c) % 255 + 1  (always in [1, 255], never zero)
  //
  //   overhead = fixed OCL bridge latency (from 1x1 baseline), subtracted
  //              to get pure transpose time.
  // =========================================================================
  task automatic run_transpose_test(
      input string label,
      input int rows, cols,
      input int src_mem, dst_naive, dst_opt,
      input int overhead,
      output logic [31:0] out_naive, out_opt);

    logic [127:0] cfg;
    logic [7:0]   val;
    logic [31:0]  naive_cycles, opt_cycles;
    int           wait_ns;

    $display("\n=========================================================");
    $display(" Test: %s  (%0d x %0d)  src=%0d  naive_dst=%0d  opt_dst=%0d",
             label, rows, cols, src_mem, dst_naive, dst_opt);
    $display("=========================================================");

    // Scale wait time: ~200ns per element, minimum 5000ns
    wait_ns = rows * cols * 200;
    if (wait_ns < 5000) wait_ns = 5000;

    // --- Write source matrix ---
    $display("  Writing source matrix ...");
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = (r * cols + c) % 255 + 1;
        write_sram(sram_addr(src_mem, c, r), replicate8(val));
      end
    end

    // --- Naive transpose (opcode 0) ---
    $display("  Running naive transpose (opcode 0) ...");
    cfg = transpose_cfg(src_mem, dst_naive, rows, cols, 0);
    write_sram(32'h33600010, cfg);
    ocl_wr32(ADDR_TOP_INTERRUPT, 32'h0);       // arm perf counter
    write_sram(32'h33000030, 128'h0);           // start transpose
    repeat (wait_ns) #1ns;
    ocl_rd32(ADDR_TOP_INTERRUPT, naive_cycles); // read latency

    $display("  Verifying naive result (mem=%0d) ...", dst_naive);
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = (r * cols + c) % 255 + 1;
        verify_sram(sram_addr(dst_naive, r, c), replicate8(val));
      end
    end

    // --- Optimized transpose (opcode 1) ---
    $display("  Running optimized transpose (opcode 1, banked BRAM) ...");
    cfg = transpose_cfg(src_mem, dst_opt, rows, cols, 1);
    write_sram(32'h33600010, cfg);
    ocl_wr32(ADDR_TOP_INTERRUPT, 32'h0);       // arm perf counter
    write_sram(32'h33000030, 128'h0);           // start transpose
    repeat (wait_ns) #1ns;
    ocl_rd32(ADDR_TOP_INTERRUPT, opt_cycles);   // read latency

    $display("  Verifying optimized result (mem=%0d) ...", dst_opt);
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = (r * cols + c) % 255 + 1;
        verify_sram(sram_addr(dst_opt, r, c), replicate8(val));
      end
    end

    // --- Performance comparison ---
    out_naive = naive_cycles;
    out_opt   = opt_cycles;
    begin
      int adj_naive, adj_opt;
      adj_naive = ($signed(naive_cycles) > overhead) ? ($signed(naive_cycles) - overhead) : 1;
      adj_opt   = ($signed(opt_cycles)   > overhead) ? ($signed(opt_cycles)   - overhead) : 1;
      $display("  >> %s (%0dx%0d):", label, rows, cols);
      $display("     Raw:      naive=%0d cyc, optimized=%0d cyc, speedup=%.2fx",
               naive_cycles, opt_cycles, real'(naive_cycles) / real'(opt_cycles));
      $display("     Adjusted: naive=%0d cyc, optimized=%0d cyc, speedup=%.2fx",
               adj_naive, adj_opt, real'(adj_naive) / real'(adj_opt));
    end
  endtask

  // =========================================================================
  // Main Test Sequence
  // =========================================================================
  initial begin
    logic [31:0] naive_out, opt_out;
    int perf_overhead;

    tb.power_up(.clk_recipe_a(ClockRecipe::A0),
                .clk_recipe_b(ClockRecipe::B0),
                .clk_recipe_c(ClockRecipe::C0));
    #500ns;

    // GBControl configuration (required before any GB operations)
    // base_large = {0, 1024, 2048}, num_vector_large = 32 for all regions
    // Each region holds up to 1024 entries -> supports matrices up to 32x32
    $display("\n===== GBControl config =====");
    write_sram(32'h33400010, 128'h00000000_08000020_04000020_00000020);

    // ---- Overhead calibration: 1x1 matrix ----
    run_transpose_test("1x1", 1, 1, 0, 1, 2, 0, naive_out, opt_out);
    perf_overhead = (naive_out + opt_out) / 2;
    $display("\n  ** Perf overhead estimate (from 1x1 avg): %0d cycles **\n", perf_overhead);

    // ---- Test suite: square and rectangular matrices ----
    run_transpose_test("3x2",    3,  2,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("4x4",    4,  4,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("4x8",    4,  8,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("8x4",    8,  4,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("8x8",    8,  8,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("16x8",  16,  8,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("8x16",   8, 16,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("16x16", 16, 16,  0, 1, 2, perf_overhead, naive_out, opt_out);
    run_transpose_test("32x32", 32, 32,  0, 1, 2, perf_overhead, naive_out, opt_out);

    #500ns;
    tb.power_down();

    if (!test_failed)
      $display("\n---- ALL TESTS PASSED ----");
    else
      $display("\n---- SOME TESTS FAILED ----");

    $finish;
  end
endmodule
