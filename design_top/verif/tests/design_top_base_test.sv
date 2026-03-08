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

  task automatic top_write(input AxiWriteCommand write_command);
    logic [49:0] transfer_addr = {8'b0, write_command.addr, 10'b0};
    logic [144:0] transfer_data = {17'h1ffff, write_command.data};

    for (int i = 0; i < LOOP_TOP_AXI_AW; i++) begin
        logic [31:0] temp_addr;
        temp_addr = transfer_addr[i*32 +: 32];
        if (i == LOOP_TOP_AXI_AW - 1) begin
          temp_addr = {14'b0, transfer_addr[49:32]};
        end
        ocl_wr32(ADDR_TOP_AXI_AW_START + i*4, temp_addr);
        #10ns;
    end

    #100ns;

    for (int i = 0; i < LOOP_TOP_AXI_W; i++) begin
        logic [31:0] temp_data;
        temp_data = transfer_data[i*32 +: 32];
        if (i == LOOP_TOP_AXI_W - 1) begin
          temp_data = {17'd0, transfer_data[144:128]};
        end
        ocl_wr32(ADDR_TOP_AXI_W_START + i*4, temp_data);
        #10ns;
    end
  endtask

  task automatic top_read(AxiReadCommand read_command);
    logic [49:0] transfer_addr = {8'b0, read_command.addr, 10'b0};
    logic [159:0] transfer_data;

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++) begin
        logic [31:0] temp_addr;
        temp_addr = transfer_addr[i*32 +: 32];
        if (i == LOOP_TOP_AXI_AR - 1) begin
          temp_addr = {18'd0, transfer_addr[49:32]};
        end
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
      $error(" Read data vs expected data mismatch! addr=0x%h, Read=0x%h, Expected=0x%h",
             read_command.addr, read_command.data, read_command.expected_read_data);
      test_failed = 1'b1;
    end
    else begin
      $display("PASS: addr=0x%h data=0x%h", read_command.addr, read_command.data);
    end
  endtask

  // =========================================================================
  // Main Test: Naive transpose (opcode 0) then Optimized transpose (opcode 1)
  //
  // Source matrix A (3 rows x 2 cols) at memory_index=0:
  //   A[0][0]=0x01..01  A[0][1]=0x02..02
  //   A[1][0]=0x03..03  A[1][1]=0x04..04
  //   A[2][0]=0x05..05  A[2][1]=0x06..06
  //
  // Expected A^T (2 rows x 3 cols):
  //   A^T[0][0]=0x01  A^T[0][1]=0x03  A^T[0][2]=0x05
  //   A^T[1][0]=0x02  A^T[1][1]=0x04  A^T[1][2]=0x06
  //
  // Naive  writes A^T to memory_index=1 (base 0x33500400)
  // Banked writes A^T to memory_index=2 (base 0x33500800)
  // =========================================================================
  initial begin
    logic [31:0] interrupt_cycles_after_naive;
    logic [31:0] interrupt_cycles_after_opt;
    realtime t_start, t_end;

    tb.power_up(.clk_recipe_a(ClockRecipe::A0),
                .clk_recipe_b(ClockRecipe::B0),
                .clk_recipe_c(ClockRecipe::C0));
    #500ns;

    // =================================================================
    // Phase 0 — GBControl configuration
    // =================================================================
    $display("\n===== Phase 0: GBControl config =====");
    top_write('{32'h33400010, 128'h00000000008000030040000300000002});

    // =================================================================
    // Phase 1 — Write source matrix into GB SRAM (memory_index=0)
    // =================================================================
    $display("\n===== Phase 1: Write source matrix (3x2) =====");
    top_write('{32'h33500000, 128'h01010101010101010101010101010101}); // A[0][0]
    top_write('{32'h33500100, 128'h02020202020202020202020202020202}); // A[0][1]
    top_write('{32'h33500010, 128'h03030303030303030303030303030303}); // A[1][0]
    top_write('{32'h33500110, 128'h04040404040404040404040404040404}); // A[1][1]
    top_write('{32'h33500020, 128'h05050505050505050505050505050505}); // A[2][0]
    top_write('{32'h33500120, 128'h06060606060606060606060606060606}); // A[2][1]

    // =================================================================
    // Phase 2 — Naive transpose (opcode=0, src=0, dst=1, 3 rows, 2 cols)
    // =================================================================
    $display("\n===== Phase 2: Naive transpose (opcode 0) =====");
    // TransposeConfig: is_valid=1, src=0, dst=1, rows=3, cols=2, opcode=0
    top_write('{32'h33600010, 128'h00000000000000000000020300010001});
    // Trigger start
    t_start = $realtime;
    top_write('{32'h33000030, 128'h0});

    #5000ns;

    // Verify transposed result at memory_index=1
    $display("\n--- Verifying naive transpose result (memory_index=1) ---");
    top_read('{32'h33500400, '0, 128'h01010101010101010101010101010101}); // A^T[0][0]
    top_read('{32'h33500500, '0, 128'h03030303030303030303030303030303}); // A^T[0][1]
    top_read('{32'h33500600, '0, 128'h05050505050505050505050505050505}); // A^T[0][2]
    top_read('{32'h33500410, '0, 128'h02020202020202020202020202020202}); // A^T[1][0]
    top_read('{32'h33500510, '0, 128'h04040404040404040404040404040404}); // A^T[1][1]
    top_read('{32'h33500610, '0, 128'h06060606060606060606060606060606}); // A^T[1][2]
    t_end = $realtime;
    $display("Naive transpose wall time: %0t", t_end - t_start);

    ocl_rd32(ADDR_TOP_INTERRUPT, interrupt_cycles_after_naive);
    $display("Interrupt counter after naive: %0d", interrupt_cycles_after_naive);

    // =================================================================
    // Phase 3 — Optimized transpose (opcode=1, src=0, dst=2, 3 rows, 2 cols)
    // =================================================================
    $display("\n===== Phase 3: Optimized transpose (opcode 1, banked BRAM) =====");
    // TransposeConfig: is_valid=1, src=0, dst=2, rows=3, cols=2, opcode=1
    top_write('{32'h33600010, 128'h00000000000000000001020300020001});
    // Trigger start
    t_start = $realtime;
    top_write('{32'h33000030, 128'h0});

    #5000ns;

    // Verify transposed result at memory_index=2
    $display("\n--- Verifying optimized transpose result (memory_index=2) ---");
    top_read('{32'h33500800, '0, 128'h01010101010101010101010101010101}); // A^T[0][0]
    top_read('{32'h33500900, '0, 128'h03030303030303030303030303030303}); // A^T[0][1]
    top_read('{32'h33500A00, '0, 128'h05050505050505050505050505050505}); // A^T[0][2]
    top_read('{32'h33500810, '0, 128'h02020202020202020202020202020202}); // A^T[1][0]
    top_read('{32'h33500910, '0, 128'h04040404040404040404040404040404}); // A^T[1][1]
    top_read('{32'h33500A10, '0, 128'h06060606060606060606060606060606}); // A^T[1][2]
    t_end = $realtime;
    $display("Optimized transpose wall time: %0t", t_end - t_start);

    ocl_rd32(ADDR_TOP_INTERRUPT, interrupt_cycles_after_opt);
    $display("Interrupt counter after optimized: %0d", interrupt_cycles_after_opt);

    // =================================================================
    // Summary
    // =================================================================
    $display("\n===== Performance Summary =====");
    $display("Interrupt cycles (naive):     %0d", interrupt_cycles_after_naive);
    $display("Interrupt cycles (optimized): %0d (delta = %0d)",
             interrupt_cycles_after_opt,
             interrupt_cycles_after_opt - interrupt_cycles_after_naive);

    #500ns;
    tb.power_down();

    if (!test_failed)
      $display("\n---- TEST PASSED ----");
    else
      $display("\n---- TEST FAILED ----");

    $finish;
  end
endmodule
