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

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++) begin
        logic [31:0] temp_addr;
        temp_addr = transfer_addr[i*32 +: 32];
        if (i == LOOP_TOP_AXI_AR - 1)
          temp_addr = {18'd0, transfer_addr[49:32]};
        ocl_wr32(ADDR_TOP_AXI_AR_START + i*4, temp_addr);
        #10ns;
    end
    #100ns;
    for (int i = 0; i < LOOP_TOP_AXI_R; i++) begin
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

  // SRAM address: 0x33500000 + mem*0x400 + vec*0x100 + ts*0x10
  function automatic logic [31:0] sram_addr(int mem, int vec, int ts);
    return 32'h33500000 + mem[31:0] * 32'h400
                        + vec[31:0] * 32'h100
                        + ts[31:0]  * 32'h10;
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
  //     All 16 bytes of A[r][c] = (r*cols + c + 1)
  //
  //   After transpose, A[r][c] appears at dst(timestep=c, vector=r)
  // =========================================================================
  task automatic run_transpose_test(
      input string label,
      input int rows, cols,
      input int src_mem, dst_naive, dst_opt);

    logic [127:0] cfg;
    logic [7:0]   val;
    realtime t0, t1;

    $display("\n=========================================================");
    $display(" Test: %s  (%0d x %0d)  src=%0d  naive_dst=%0d  opt_dst=%0d",
             label, rows, cols, src_mem, dst_naive, dst_opt);
    $display("=========================================================");

    // --- Write source matrix ---
    $display("  Writing source matrix ...");
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = r * cols + c + 1;
        write_sram(sram_addr(src_mem, c, r), replicate8(val));
      end
    end

    // --- Naive transpose (opcode 0) ---
    $display("  Running naive transpose (opcode 0) ...");
    cfg = transpose_cfg(src_mem, dst_naive, rows, cols, 0);
    write_sram(32'h33600010, cfg);
    t0 = $realtime;
    write_sram(32'h33000030, 128'h0);
    #5000ns;
    t1 = $realtime;

    $display("  Verifying naive result (mem=%0d) ...", dst_naive);
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = r * cols + c + 1;
        verify_sram(sram_addr(dst_naive, r, c), replicate8(val));
      end
    end
    $display("  Naive wall time: %0t", t1 - t0);

    // --- Optimized transpose (opcode 1) ---
    $display("  Running optimized transpose (opcode 1, banked BRAM) ...");
    cfg = transpose_cfg(src_mem, dst_opt, rows, cols, 1);
    write_sram(32'h33600010, cfg);
    t0 = $realtime;
    write_sram(32'h33000030, 128'h0);
    #5000ns;
    t1 = $realtime;

    $display("  Verifying optimized result (mem=%0d) ...", dst_opt);
    for (int r = 0; r < rows; r++) begin
      for (int c = 0; c < cols; c++) begin
        val = r * cols + c + 1;
        verify_sram(sram_addr(dst_opt, r, c), replicate8(val));
      end
    end
    $display("  Optimized wall time: %0t", t1 - t0);
  endtask

  // =========================================================================
  // Main Test Sequence
  // =========================================================================
  initial begin
    logic [31:0] interrupt_cycles;

    tb.power_up(.clk_recipe_a(ClockRecipe::A0),
                .clk_recipe_b(ClockRecipe::B0),
                .clk_recipe_c(ClockRecipe::C0));
    #500ns;

    // GBControl configuration (required before any GB operations)
    $display("\n===== GBControl config =====");
    write_sram(32'h33400010, 128'h00000000008000030040000300000002);

    // ---- Test suite: multiple matrix sizes, each with opcode 0 and 1 ----
    //   Memory layout per test: src=0, naive_dst=1, opt_dst=2
    run_transpose_test("1x1",   1, 1,  /*src*/0, /*naive*/1, /*opt*/2);
    run_transpose_test("2x3",   2, 3,  /*src*/0, /*naive*/1, /*opt*/2);
    run_transpose_test("3x2",   3, 2,  /*src*/0, /*naive*/1, /*opt*/2);
    run_transpose_test("4x4",   4, 4,  /*src*/0, /*naive*/1, /*opt*/2);

    // Read interrupt counter
    ocl_rd32(ADDR_TOP_INTERRUPT, interrupt_cycles);
    $display("\nTotal interrupt cycles across all tests: %0d", interrupt_cycles);

    #500ns;
    tb.power_down();

    if (!test_failed)
      $display("\n---- ALL TESTS PASSED ----");
    else
      $display("\n---- SOME TESTS FAILED ----");

    $finish;
  end
endmodule
