`include "common_base_test.svh"
`include "design_top_defines.vh"

bit matmul_test_failed = 0;

module design_top_matmul_test();
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
      matmul_test_failed = 1'b1;
    end else begin
      $display("  PASS addr=0x%h", read_command.addr);
    end
  endtask

  // =========================================================================
  // Helpers
  // =========================================================================

  function automatic logic [31:0] sram_addr(int mem, int vec, int ts);
    int flat;
    flat = mem * 1024 + (ts % 16) + ((ts / 16) * 32 + vec) * 16;
    return 32'h33500000 + flat[31:0] * 32'h10;
  endfunction

  function automatic logic [127:0] replicate8(logic [7:0] val);
    return {16{val}};
  endfunction

  // Build TransposeConfig for matmul: includes memory_index_q and num_rows_q
  function automatic logic [127:0] matmul_cfg(
      int src_mem, int dst_mem, int q_mem,
      int n_rows, int n_cols, int opcode, int m_rows);
    logic [127:0] cfg = 128'b0;
    cfg[0]     = 1'b1;
    cfg[10:8]  = src_mem[2:0];
    cfg[18:16] = dst_mem[2:0];
    cfg[26:24] = q_mem[2:0];
    cfg[39:32] = n_rows[7:0];
    cfg[47:40] = n_cols[7:0];
    cfg[50:48] = opcode[2:0];
    cfg[63:56] = m_rows[7:0];
    return cfg;
  endfunction

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
  // Compute expected Q*K^T result[i][j] = sum_k Q[i][k] * K[j][k]
  // Q value = (i*D+k)%255+1, K value = (j*D+k)%255+1
  // Returns lower 8 bits of dot product
  // =========================================================================
  function automatic logic [7:0] expected_matmul(int i, int j, int D);
    int sum;
    sum = 0;
    for (int k = 0; k < D; k++) begin
      int q_val, k_val;
      q_val = (i * D + k) % 255 + 1;
      k_val = (j * D + k) % 255 + 1;
      sum = sum + q_val * k_val;
    end
    return sum[7:0];
  endfunction

  // =========================================================================
  // Run one Q*K^T test: both unfused (opcode 2) and fused (opcode 3)
  //
  //   K is (N x D) at src_mem,  element = (r*D+c)%255+1
  //   Q is (M x D) at q_mem,    element = (r*D+c)%255+1
  //   result is (M x N) at dst_mem
  //
  //   overhead = fixed OCL bridge latency to subtract
  // =========================================================================
  task automatic run_matmul_test(
      input string label,
      input int M, N, D,
      input int src_mem, q_mem, dst_mem,
      input int overhead,
      output logic [31:0] out_unfused, out_fused);

    logic [127:0] cfg;
    logic [7:0]   val;
    logic [31:0]  unfused_cycles, fused_cycles;
    int           wait_ns;

    $display("\n=========================================================");
    $display(" Matmul Test: %s  Q(%0dx%0d) * K(%0dx%0d)^T = (%0dx%0d)",
             label, M, D, N, D, M, N);
    $display("   src(K)=%0d  q(Q)=%0d  dst(result)=%0d", src_mem, q_mem, dst_mem);
    $display("=========================================================");

    wait_ns = (N * D + M * N * D) * 100;
    if (wait_ns < 10000) wait_ns = 10000;

    // --- Write K matrix (N x D) at src_mem ---
    $display("  Writing K matrix (%0dx%0d) at mem %0d ...", N, D, src_mem);
    for (int r = 0; r < N; r++) begin
      for (int c = 0; c < D; c++) begin
        val = (r * D + c) % 255 + 1;
        write_sram(sram_addr(src_mem, c, r), replicate8(val));
      end
    end

    // --- Write Q matrix (M x D) at q_mem ---
    $display("  Writing Q matrix (%0dx%0d) at mem %0d ...", M, D, q_mem);
    for (int r = 0; r < M; r++) begin
      for (int c = 0; c < D; c++) begin
        val = (r * D + c) % 255 + 1;
        write_sram(sram_addr(q_mem, c, r), replicate8(val));
      end
    end

    // --- Unfused Q*K^T (opcode 2) ---
    $display("  Running UNFUSED Q*K^T (opcode 2) ...");
    cfg = matmul_cfg(src_mem, dst_mem, q_mem, N, D, 2, M);
    write_sram(32'h33600010, cfg);
    ocl_wr32(ADDR_TOP_INTERRUPT, 32'h0);
    write_sram(32'h33000030, 128'h0);
    repeat (wait_ns) #1ns;
    ocl_rd32(ADDR_TOP_INTERRUPT, unfused_cycles);

    $display("  Verifying unfused result ...");
    for (int i = 0; i < M; i++) begin
      for (int j = 0; j < N; j++) begin
        val = expected_matmul(i, j, D);
        verify_sram(sram_addr(dst_mem, j, i), replicate8(val));
      end
    end

    // --- Fused Q*K^T (opcode 3) ---
    $display("  Running FUSED Q*K^T (opcode 3) ...");
    cfg = matmul_cfg(src_mem, dst_mem, q_mem, N, D, 3, M);
    write_sram(32'h33600010, cfg);
    ocl_wr32(ADDR_TOP_INTERRUPT, 32'h0);
    write_sram(32'h33000030, 128'h0);
    repeat (wait_ns) #1ns;
    ocl_rd32(ADDR_TOP_INTERRUPT, fused_cycles);

    $display("  Verifying fused result ...");
    for (int i = 0; i < M; i++) begin
      for (int j = 0; j < N; j++) begin
        val = expected_matmul(i, j, D);
        verify_sram(sram_addr(dst_mem, j, i), replicate8(val));
      end
    end

    // --- Performance comparison ---
    out_unfused = unfused_cycles;
    out_fused   = fused_cycles;
    begin
      int adj_unfused, adj_fused;
      adj_unfused = ($signed(unfused_cycles) > overhead) ? ($signed(unfused_cycles) - overhead) : 1;
      adj_fused   = ($signed(fused_cycles)   > overhead) ? ($signed(fused_cycles)   - overhead) : 1;
      $display("  >> %s  Q(%0dx%0d)*K(%0dx%0d)^T:", label, M, D, N, D);
      $display("     Raw:      unfused=%0d cyc, fused=%0d cyc, speedup=%.2fx",
               unfused_cycles, fused_cycles, real'(unfused_cycles) / real'(fused_cycles));
      $display("     Adjusted: unfused=%0d cyc, fused=%0d cyc, speedup=%.2fx",
               adj_unfused, adj_fused, real'(adj_unfused) / real'(adj_fused));
    end
  endtask

  // =========================================================================
  // Main Test Sequence
  // =========================================================================
  initial begin
    logic [31:0] unfused_out, fused_out;
    int perf_overhead;

    tb.power_up(.clk_recipe_a(ClockRecipe::A0),
                .clk_recipe_b(ClockRecipe::B0),
                .clk_recipe_c(ClockRecipe::C0));
    #500ns;

    // GBControl: 4 memory regions, base={0,1024,2048,3072}, num_vec=32
    $display("\n===== GBControl config (4 regions) =====");
    write_sram(32'h33400010, 128'h0C000020_08000020_04000020_00000020);

    // Overhead calibration: smallest possible matmul (M=N=D=1)
    run_matmul_test("1x1x1", 1, 1, 1, 0, 1, 2, 0, unfused_out, fused_out);
    perf_overhead = (unfused_out + fused_out) / 2;
    $display("\n  ** Matmul perf overhead estimate (from 1x1x1 avg): %0d cycles **\n", perf_overhead);

    // Test suite: Q(MxD) * K(NxD)^T = result(MxN)
    //   Memory: K at 0, Q at 1, result at 2, K^T temp at 3 (unfused only)
    run_matmul_test("2x2x2",   2, 2, 2, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("4x4x4",   4, 4, 4, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("4x4x8",   4, 4, 8, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("8x8x4",   8, 8, 4, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("8x8x8",   8, 8, 8, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("4x8x4",   4, 8, 4, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("16x16x8", 16,16, 8, 0, 1, 2, perf_overhead, unfused_out, fused_out);
    run_matmul_test("16x16x16",16,16,16, 0, 1, 2, perf_overhead, unfused_out, fused_out);

    #500ns;
    tb.power_down();

    if (!matmul_test_failed)
      $display("\n---- ALL MATMUL TESTS PASSED ----");
    else
      $display("\n---- SOME MATMUL TESTS FAILED ----");

    $finish;
  end
endmodule
