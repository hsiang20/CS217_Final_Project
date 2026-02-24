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
    // Write address to AW channel
    // Strobe = 1ffff
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
        
   // Write data to W channel
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

    // Write address to AR channel
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

    // Read data from R channel
    for (int i = 0; i < LOOP_TOP_AXI_R; i++) begin
        logic [31:0] temp_data;
        ocl_rd32(ADDR_TOP_AXI_R_START + i*4, temp_data);
        #10ns;
        transfer_data[i*32 +: 32] = temp_data;
    end
    read_command.data = transfer_data[137:10];

    if (read_command.data != read_command.expected_read_data) begin
      $error(" Read data vs expected data mismatch! Read data = 0x%h, Expected data = 0x%h", read_command.data, read_command.expected_read_data);
      test_failed = 1'b1;
    end
    else begin
      $display("Read value matches the expected = 0x%h at 0x%h", read_command.data, read_command.addr);
    end
    

  endtask

  // =========================================================================
  // Main Test Sequence
  // =========================================================================
  initial begin
    logic [31:0] interrupt_cycles;
    // --- Command Arrays ---
    AxiWriteCommand write_commands[] = {
      '{32'h33500000, 128'hD4C04352A0A882BF584169B29EE3E635},
      '{32'h34500000, 128'hC74278675C7F32026BD421D788E1D68C},
      '{32'h34500010, 128'hD0319B6C21E7A2FDA174272E5EC23966},
      '{32'h34500020, 128'h5781547CB8E9DD33A331DDE2178B1B85},
      '{32'h34500030, 128'h4C8C4B8304BCB9614E92D9208D22BBEB},
      '{32'h34500040, 128'h67AD5414A9D9B7200DC7A487C5BFA479},
      '{32'h34500050, 128'h69AB46F7D70083F70292B32C4A09CF2D},
      '{32'h34500060, 128'h161F65749B261E3FD103A7F427208FCB},
      '{32'h34500070, 128'hAABCB5EFB2819ED0ED6A7A4AE1356000},
      '{32'h34500080, 128'hF6D7D9415D2D4CDAC59C9EC4A9695EE4},
      '{32'h34500090, 128'h17899ACBF78E9E7F7F1519733E0DFD81},
      '{32'h345000A0, 128'hFCFD96D169A343A4ACC647814E3AD635},
      '{32'h345000B0, 128'h5C0F148DE065D4848582459D58371BA5},
      '{32'h345000C0, 128'hCC77DBF7B30AE0F6A96684FC1E5515A8},
      '{32'h345000D0, 128'hC5AA491813F2CD9DC97FD011DF439320},
      '{32'h345000E0, 128'hB8E6F69A72129DD4520BE5B52C4EE908},
      '{32'h345000F0, 128'h38B305DEDFD4E551679295D0AE2D9CD6},
      '{32'h34400010, 128'h10100000001},
      '{32'h34400020, 128'h100},
      '{32'h34800010, 128'h103020001},
      '{32'h34800020, 128'h40B030},
      '{32'h33400010, 128'h1},
      '{32'h33700010, 128'h100010101010000000001},
      '{32'h33000010, 128'h0},
      '{32'h33400010, 128'h10001},
      '{32'h33500010, 128'h298E1EFC3652115C5D0340C6761D3767},
      '{32'h33C00010, 128'h10001000000000101},
      '{32'h33000020, 128'h0},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF}, 
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF},
      '{32'h345000F0, 128'hDEADBEEFDEADBEEFDEADBEEFDEADBEEF}
    };

    AxiReadCommand read_commands[] = {
      '{32'h33500010, '0, 128'h00000000000000010100000008000003},
      '{32'h34600000, '0, 128'hD4C04352A0A882BF584169B29EE3E635},
      '{32'h33500000, '0, 128'h10101010101010101010101010101010}
    };

    // Power up the testbench
    tb.power_up(.clk_recipe_a(ClockRecipe::A0),
                .clk_recipe_b(ClockRecipe::B0),
                .clk_recipe_c(ClockRecipe::C0));

    #500ns;

    $display("\n Starting AXI reads and writes...\n");

    // --- Execute Commands ---
    foreach (write_commands[i]) begin
      top_write(write_commands[i]);
    end

    foreach (read_commands[i]) begin
      top_read(read_commands[i]);
    end

    // Count Interrupt cycles and read the value
    ocl_rd32(ADDR_TOP_INTERRUPT, interrupt_cycles);
    $display("Interrupt cycles = %d", interrupt_cycles);
    if (interrupt_cycles <= 10) begin
      $error(" Interrupt cycles lesser than expected! Interrupt cycles = %d", interrupt_cycles);
      test_failed = 1'b1;
    end

    #500ns;
    tb.power_down();
    
    if (!test_failed)
      $display("---- TEST PASSED ----");
    else
      $display("---- TEST FAILED ----");
      
    $finish;
  end
endmodule