// ============================================================================
// Amazon FPGA Hardware Development Kit
//
// Copyright 2024 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Amazon Software License (the "License"). You may not use
// this file except in compliance with the License. A copy of the License is
// located at
//
//    http://aws.amazon.com/asl/
//
// or in the "license" file accompanying this file. This file is distributed on
// an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, express or
// implied. See the License for the specific language governing permissions and
// limitations under the License.
// ============================================================================

//====================================================================================
// Top level module file for design_top - Top module wrapper for AWS F2
//====================================================================================

`include "./concat_Top.v"
`include "./counter.v"

module design_top
  #(
     parameter EN_DDR = 0,
     parameter EN_HBM = 0
   )
   (
`include "cl_ports.vh"
   );

`include "design_top_defines.vh"
`include "cl_id_defines.vh"
  
  // ---------- AXI-Lite (master side of the reg-slice) to our bridge ----------
  logic [15:0]  axil_awaddr_m;
  logic         axil_awvalid_m, axil_awready_m;
  logic [31:0]  axil_wdata_m;
  logic [3:0]   axil_wstrb_m;
  logic         axil_wvalid_m, axil_wready_m;
  logic [1:0]   axil_bresp_m;
  logic         axil_bvalid_m, axil_bready_m;

  logic [15:0]  axil_araddr_m;
  logic         axil_arvalid_m, axil_arready_m;
  logic [31:0]  axil_rdata_m;
  logic [1:0]   axil_rresp_m;
  logic         axil_rvalid_m, axil_rready_m;

  // ---------- Top module AXI-like interface signals ----------
  logic interrupt;
  logic if_axi_rd_ar_vld, if_axi_rd_ar_rdy;
  logic [49:0] if_axi_rd_ar_dat;
  logic if_axi_rd_r_vld, if_axi_rd_r_rdy;
  logic [140:0] if_axi_rd_r_dat;
  logic if_axi_wr_aw_vld, if_axi_wr_aw_rdy;
  logic [49:0] if_axi_wr_aw_dat;
  logic if_axi_wr_w_vld, if_axi_wr_w_rdy;
  logic [144:0] if_axi_wr_w_dat;
  logic if_axi_wr_b_vld, if_axi_wr_b_rdy;
  logic [11:0] if_axi_wr_b_dat;

  //==============================
  // Top module instance
  //==============================
  // TODO #1:
  //    1. Instantiate the Top module (u_top) and connect its ports
  /////////////// YOUR CODE STARTS HERE ///////////////
  Top u_top (
        .clk              (clk_main_a0),
        .rst              (rst_main_n),
        .interrupt        (interrupt),
        .if_axi_rd_ar_vld (if_axi_rd_ar_vld),
        .if_axi_rd_ar_rdy (if_axi_rd_ar_rdy),
        .if_axi_rd_ar_dat (if_axi_rd_ar_dat),
        .if_axi_rd_r_vld  (if_axi_rd_r_vld),
        .if_axi_rd_r_rdy  (if_axi_rd_r_rdy),
        .if_axi_rd_r_dat  (if_axi_rd_r_dat),
        .if_axi_wr_aw_vld (if_axi_wr_aw_vld),
        .if_axi_wr_aw_rdy (if_axi_wr_aw_rdy),
        .if_axi_wr_aw_dat (if_axi_wr_aw_dat),
        .if_axi_wr_w_vld  (if_axi_wr_w_vld),
        .if_axi_wr_w_rdy  (if_axi_wr_w_rdy),
        .if_axi_wr_w_dat  (if_axi_wr_w_dat),
        .if_axi_wr_b_vld  (if_axi_wr_b_vld),
        .if_axi_wr_b_rdy  (if_axi_wr_b_rdy),
        .if_axi_wr_b_dat  (if_axi_wr_b_dat)
      );

  /////////////// YOUR CODE ENDS HERE ///////////////

  /*always_ff @(posedge clk_main_a0 or negedge rst_main_n) begin
    if (if_axi_rd_ar_vld & if_axi_rd_ar_rdy)
      $display("if_axi_rd_ar_vld = %b, if_axi_rd_ar_rdy = %b, if_axi_rd_ar_dat = %h", if_axi_rd_ar_vld, if_axi_rd_ar_rdy, if_axi_rd_ar_dat);
    if (if_axi_rd_r_vld & if_axi_rd_r_rdy)
      $display("if_axi_rd_r_vld = %b, if_axi_rd_r_rdy = %b, if_axi_rd_r_dat = %h", if_axi_rd_r_vld, if_axi_rd_r_rdy, if_axi_rd_r_dat);
    if (if_axi_wr_aw_vld & if_axi_wr_aw_rdy)
      $display("if_axi_wr_aw_vld = %b, if_axi_wr_aw_rdy = %b, if_axi_wr_aw_dat = %h", if_axi_wr_aw_vld, if_axi_wr_aw_rdy, if_axi_wr_aw_dat);
    if (if_axi_wr_w_vld & if_axi_wr_w_rdy)
      $display("if_axi_wr_w_vld = %b, if_axi_wr_w_rdy = %b, if_axi_wr_w_dat = %h", if_axi_wr_w_vld, if_axi_wr_w_rdy, if_axi_wr_w_dat);
    if (if_axi_wr_b_vld & if_axi_wr_b_rdy)
      $display("if_axi_wr_b_vld = %b, if_axi_wr_b_rdy = %b, if_axi_wr_b_dat = %h", if_axi_wr_b_vld, if_axi_wr_b_rdy, if_axi_wr_b_dat);
  end*/

  // Performance counter: measures clock cycles from arm to interrupt rising edge.
  // Arm by writing any value to ADDR_TOP_INTERRUPT via OCL.
  // Read the latency result from ADDR_TOP_INTERRUPT via OCL.
  logic        perf_start_pulse;
  logic        perf_running;
  logic [31:0] perf_counter;
  logic        interrupt_prev;

  always_ff @(posedge clk_main_a0 or negedge rst_main_n) begin
    if (!rst_main_n) begin
      perf_running   <= 1'b0;
      perf_counter   <= 32'd0;
      interrupt_prev <= 1'b0;
    end else begin
      interrupt_prev <= interrupt;
      if (perf_start_pulse) begin
        perf_counter <= 32'd0;
        perf_running <= 1'b1;
      end else if (interrupt && !interrupt_prev) begin
        perf_running <= 1'b0;
      end else if (perf_running) begin
        perf_counter <= perf_counter + 32'd1;
      end
    end
  end


  //=============================================================================
  // GLOBALS
  //=============================================================================
  always_comb
  begin
    cl_sh_flr_done    = 1'b1;
    cl_sh_status0     = 32'h0;
    cl_sh_status1     = 32'h0;
    cl_sh_status2     = 32'h0;
    cl_sh_id0         = `CL_SH_ID0;
    cl_sh_id1         = `CL_SH_ID1;
    cl_sh_status_vled = 16'h0;
    cl_sh_dma_wr_full = 1'b0;
    cl_sh_dma_rd_full = 1'b0;
  end

  //=============================================================================
  // OCL REGISTER SLICE INSTANCE
  //=============================================================================

  // OCL AXI-Lite Register Slice Connections
  logic [15:0] ocl_awaddr;
  logic        ocl_awvalid;
  logic        ocl_awready;
  logic [31:0] ocl_wdata;
  logic [3:0]  ocl_wstrb;
  logic        ocl_wvalid;
  logic        ocl_wready;
  logic [1:0]  ocl_bresp;
  logic        ocl_bvalid;
  logic        ocl_bready;
  logic [15:0] ocl_araddr;
  logic        ocl_arvalid;
  logic        ocl_arready;
  logic [31:0] ocl_rdata;
  logic [1:0]  ocl_rresp;
  logic        ocl_rvalid;
  logic        ocl_rready;

  // Internal master-side signals from reg-slice to our simple AXI-Lite slave
  logic [2:0]   axil_awprot_m;

  axi_register_slice_light AXIL_OCL_REG_SLC_BOT_SLR (
                             .aclk          (clk_main_a0),
                             .aresetn       (rst_main_n),

                             // Shell → CL (slave port of slice)
                             .s_axi_awaddr  (ocl_cl_awaddr),
                             .s_axi_awprot  ('0),
                             .s_axi_awvalid (ocl_cl_awvalid),
                             .s_axi_awready (cl_ocl_awready),
                             .s_axi_wdata   (ocl_cl_wdata),
                             .s_axi_wstrb   (ocl_cl_wstrb),
                             .s_axi_wvalid  (ocl_cl_wvalid),
                             .s_axi_wready  (cl_ocl_wready),
                             .s_axi_bresp   (cl_ocl_bresp),
                             .s_axi_bvalid  (cl_ocl_bvalid),
                             .s_axi_bready  (ocl_cl_bready),
                             .s_axi_araddr  (ocl_cl_araddr),
                             .s_axi_arvalid (ocl_cl_arvalid),
                             .s_axi_arready (cl_ocl_arready),
                             .s_axi_rdata   (cl_ocl_rdata),
                             .s_axi_rresp   (cl_ocl_rresp),
                             .s_axi_rvalid  (cl_ocl_rvalid),
                             .s_axi_rready  (ocl_cl_rready),

                             // CL-internal master side (wire these!)
                             .m_axi_awaddr  (axil_awaddr_m),
                             .m_axi_awprot  (axil_awprot_m),
                             .m_axi_awvalid (axil_awvalid_m),
                             .m_axi_awready (axil_awready_m),
                             .m_axi_wdata   (axil_wdata_m),
                             .m_axi_wstrb   (axil_wstrb_m),
                             .m_axi_wvalid  (axil_wvalid_m),
                             .m_axi_wready  (axil_wready_m),
                             .m_axi_bresp   (axil_bresp_m),
                             .m_axi_bvalid  (axil_bvalid_m),
                             .m_axi_bready  (axil_bready_m),

                             .m_axi_araddr  (axil_araddr_m),
                             .m_axi_arvalid (axil_arvalid_m),
                             .m_axi_arready (axil_arready_m),
                             .m_axi_rdata   (axil_rdata_m),
                             .m_axi_rresp   (axil_rresp_m),
                             .m_axi_rvalid  (axil_rvalid_m),
                             .m_axi_rready  (axil_rready_m)
                           );

  //=============================================================================
  // AXI-Lite to Top module bridge
  //=============================================================================
  // Simple AXI-Lite slave accepting one write/read at a time
  // Write channel bookkeeping
  logic        wr_aw_captured, wr_w_captured;
  logic [15:0] wr_addr_q;
  logic [31:0] wr_data_q;

  logic [11:0] if_axi_wr_b_dat_sig;

  // Default AXI responses
  assign axil_awprot_m = 3'b000;

  // Ready when not holding captured info
  logic axi_ready;
  assign axil_awready_m = ~wr_aw_captured && axi_ready;
  assign axil_wready_m  = ~wr_w_captured && axi_ready;

  always_ff @(posedge clk_main_a0 or negedge rst_main_n) begin
    if (!rst_main_n) begin
      wr_aw_captured <= 1'b0;
      wr_w_captured  <= 1'b0;
      wr_addr_q      <= '0;
      wr_data_q      <= '0;
      axil_bvalid_m  <= 1'b0;
      axil_bresp_m   <= 2'b00;

      if_axi_wr_aw_vld <= 1'b0;
      if_axi_wr_w_vld  <= 1'b0;
      if_axi_rd_ar_vld <= 1'b0;


      if_axi_wr_aw_dat <= '0;
      if_axi_wr_w_dat  <= '0;
      if_axi_rd_ar_dat <= '0;
      if_axi_wr_b_dat_sig <= '0;

      if_axi_wr_b_rdy <= 1'b1;
      axi_ready <= 1'b1;
      perf_start_pulse <= 1'b0;

    end else begin
      perf_start_pulse <= 1'b0;  // default: one-cycle pulse

      // Capture address/data
      if (axil_awvalid_m && axil_awready_m) begin
        wr_aw_captured <= 1'b1;
        wr_addr_q      <= axil_awaddr_m;
      end
      if (axil_wvalid_m && axil_wready_m) begin
        wr_w_captured <= 1'b1;
        wr_data_q     <= axil_wdata_m;
      end

      // Fire a write when both parts captured and no BRESP pending
      if (wr_aw_captured && wr_w_captured && !axil_bvalid_m) begin

        // Perf counter: arm on write to ADDR_TOP_INTERRUPT
        if (wr_addr_q == ADDR_TOP_INTERRUPT)
          perf_start_pulse <= 1'b1;

        // AXI Write Address Channel
        // TODO 2:
        //    1. Loop through the AXI Write address registers.
        //    2. Check if the write address matches the current register address.
        //    3. If it's the last register, assert if_axi_wr_b_rdy and axi_ready, but don't make it valid.
        //    4. Making if_axi_wr_aw_vld valid is the responsibility of the AXI Write data loop.
        /////////////// YOUR CODE STARTS HERE ///////////////
        for (int i = 0; i < LOOP_TOP_AXI_AW; i++) begin
          if (wr_addr_q == (ADDR_TOP_AXI_AW_START + i*4)) begin
            if (i == LOOP_TOP_AXI_AW - 1) begin
              if_axi_wr_aw_dat[49:32] <= wr_data_q[17:0];
              if_axi_wr_b_rdy <= 1'b1;
              axi_ready <= 1'b1;
            end else begin
              if_axi_wr_aw_dat[(i+1)*32-1 -: 32] <= wr_data_q;
            end
          end
        end
        /////////////// YOUR CODE ENDS HERE ///////////////

        // AXI Write Data Channel
        // TODO 3:
        //    1. Loop through the AXI Write data registers
        //    2. Check if the write address matches the current register address
        //    3. If it's the last register, assign the remaining data, and assert the valid signals
        //    4. De-assert axi_ready to wait for the handshake to complete
        //    5. Assign the data to the corresponding register
        /////////////// YOUR CODE STARTS HERE ///////////////
        for (int i = 0; i < LOOP_TOP_AXI_W; i++) begin
          if (wr_addr_q == (ADDR_TOP_AXI_W_START + i*4)) begin
            if (i == LOOP_TOP_AXI_W - 1) begin
              if_axi_wr_w_dat[144:128] <= wr_data_q[16:0];
              if_axi_wr_aw_vld <= 1'b1;
              if_axi_wr_w_vld <= 1'b1;
              axi_ready <= 1'b0;
            end else begin
              if_axi_wr_w_dat[(i+1)*32-1 -: 32] <= wr_data_q;
            end
          end
        end
        /////////////// YOUR CODE ENDS HERE ///////////////

        // AXI Read Address Channel
        for (int i = 0; i < LOOP_TOP_AXI_AR; i++) begin 
            if (wr_addr_q == (ADDR_TOP_AXI_AR_START + i*4)) begin
              if (i == LOOP_TOP_AXI_AR - 1) begin
                if_axi_rd_ar_dat[49:32] <= wr_data_q[17:0];
                if_axi_rd_ar_vld <= 1'b1;
                axi_ready <= 1'b0;
              end
              else
                if_axi_rd_ar_dat[(i+1)*32-1 -: 32] <= wr_data_q;
            end
        end

        wr_aw_captured <= 1'b0;
        wr_w_captured  <= 1'b0;
        axil_bvalid_m  <= 1'b1;
      end

      // Handle ready signals
      if (if_axi_wr_aw_vld && if_axi_wr_aw_rdy) begin
        if_axi_wr_aw_vld <= 1'b0;
      end

      if (if_axi_wr_w_vld && if_axi_wr_w_rdy) begin
        if_axi_wr_w_vld <= 1'b0;
      end

      if (if_axi_wr_b_vld && if_axi_wr_b_rdy) begin
        if_axi_wr_b_rdy <= 1'b0;
        axi_ready <= 1'b1;
        if_axi_wr_b_dat_sig <= if_axi_wr_b_dat;
      end else begin
        if_axi_wr_b_rdy <= 1'b1;
      end

      if (if_axi_rd_ar_vld && if_axi_rd_ar_rdy) begin
        if_axi_rd_ar_vld <= 1'b0;
        axi_ready <= 1'b1;
      end

      if (axil_bvalid_m && axil_bready_m) begin
        axil_bvalid_m <= 1'b0;
      end
    end
  end

  logic [WIDTH_TOP_AXI_R-1:0] top_r_dat_q;
  logic top_r_valid_q;
  logic [WIDTH_TOP_AXI_B-1:0] top_b_dat_q;

  // AXI-Lite read handshake: respond in-place
  always_ff @(posedge clk_main_a0 or negedge rst_main_n) begin
    if (!rst_main_n) begin
      axil_arready_m <= 1'b1;
      axil_rvalid_m  <= 1'b0;
      axil_rdata_m   <= 32'h0;
      axil_rresp_m   <= 2'b00;

      if_axi_rd_r_rdy <= 1'b1;

      top_r_dat_q <= '0;
      top_r_valid_q <= 1'b0;
      top_b_dat_q <= '0;

    end else begin
      // Capture data from Top module
      if (if_axi_rd_r_vld && if_axi_rd_r_rdy) begin
        top_r_dat_q <= if_axi_rd_r_dat;
        if_axi_rd_r_rdy <= 1'b0; 
        top_r_valid_q <= 1'b1;
      end else begin
        if_axi_rd_r_rdy <= 1'b1;
      end
      
      // Handle AXI-Lite read
      if (axil_arvalid_m && axil_arready_m) begin
        axil_arready_m <= 1'b0;
        axil_rvalid_m <= 1'b1;
        axil_rresp_m  <= 2'b00;

        if (top_r_valid_q) begin
        // TODO 4:
        //    1. Loop through the AXI Read data registers
        //    2. Check if the read address matches the current register address
        //    3. If it's the last register, de-assert top_r_valid_q and assert if_axi_rd_r_rdy
        //    4. Assign the corresponding data to axil_rdata_m
        /////////////// YOUR CODE STARTS HERE ///////////////
          for (int i = 0; i < LOOP_TOP_AXI_R; i++) begin
            if (axil_araddr_m == (ADDR_TOP_AXI_R_START + i*4)) begin
              if (i == LOOP_TOP_AXI_R - 1) begin
                axil_rdata_m <= {19'b0, top_r_dat_q[140:128]};
                top_r_valid_q <= 1'b0;
                if_axi_rd_r_rdy <= 1'b1;
              end else begin
                axil_rdata_m <= top_r_dat_q[(i+1)*32-1 -: 32];
              end
            end
          end
        /////////////// YOUR CODE ENDS HERE ///////////////

        end
        else begin
          if (axil_araddr_m == ADDR_TOP_INTERRUPT) begin
            axil_rdata_m <= perf_counter;
          end else begin
            axil_rdata_m <= 32'hDEADBEEF;
          end
        end
      end

      if (axil_rvalid_m && axil_rready_m) begin
        axil_rvalid_m  <= 1'b0;
        axil_arready_m <= 1'b1;
      end
    end
  end

  //=============================================================================
  // PCIM
  //=============================================================================

  // Cause Protocol Violations
  always_comb
  begin
    cl_sh_pcim_awaddr  = 'b0;
    cl_sh_pcim_awsize  = 'b0;
    cl_sh_pcim_awburst = 'b0;
    cl_sh_pcim_awvalid = 'b0;

    cl_sh_pcim_wdata   = 'b0;
    cl_sh_pcim_wstrb   = 'b0;
    cl_sh_pcim_wlast   = 'b0;
    cl_sh_pcim_wvalid  = 'b0;

    cl_sh_pcim_araddr  = 'b0;
    cl_sh_pcim_arsize  = 'b0;
    cl_sh_pcim_arburst = 'b0;
    cl_sh_pcim_arvalid = 'b0;
  end

  // Remaining CL Output Ports
  always_comb
  begin
    cl_sh_pcim_awid    = 'b0;
    cl_sh_pcim_awlen   = 'b0;
    cl_sh_pcim_awcache = 'b0;
    cl_sh_pcim_awlock  = 'b0;
    cl_sh_pcim_awprot  = 'b0;
    cl_sh_pcim_awqos   = 'b0;
    cl_sh_pcim_awuser  = 'b0;

    cl_sh_pcim_wid     = 'b0;
    cl_sh_pcim_wuser   = 'b0;

    cl_sh_pcim_arid    = 'b0;
    cl_sh_pcim_arlen   = 'b0;
    cl_sh_pcim_arcache = 'b0;
    cl_sh_pcim_arlock  = 'b0;
    cl_sh_pcim_arprot  = 'b0;
    cl_sh_pcim_arqos   = 'b0;
    cl_sh_pcim_aruser  = 'b0;

    cl_sh_pcim_rready  = 'b0;
  end

  //=============================================================================
  // PCIS
  //=============================================================================

  // Cause Protocol Violations
  always_comb
  begin
    cl_sh_dma_pcis_bresp   = 'b0;
    cl_sh_dma_pcis_rresp   = 'b0;
    cl_sh_dma_pcis_rvalid  = 'b0;
  end

  // Remaining CL Output Ports
  always_comb
  begin
    cl_sh_dma_pcis_awready = 'b0;

    cl_sh_dma_pcis_wready  = 'b0;

    cl_sh_dma_pcis_bid     = 'b0;
    cl_sh_dma_pcis_bvalid  = 'b0;

    cl_sh_dma_pcis_arready  = 'b0;

    cl_sh_dma_pcis_rid     = 'b0;
    cl_sh_dma_pcis_rdata   = 'b0;
    cl_sh_dma_pcis_rlast   = 'b0;
    cl_sh_dma_pcis_ruser   = 'b0;
  end

  //=============================================================================
  // SDA
  //=============================================================================

  // Cause Protocol Violations
  always_comb
  begin
    cl_sda_bresp   = 'b0;
    cl_sda_rresp   = 'b0;
    cl_sda_rvalid  = 'b0;
  end

  // Remaining CL Output Ports
  always_comb
  begin
    cl_sda_awready = 'b0;
    cl_sda_wready  = 'b0;

    cl_sda_bvalid = 'b0;

    cl_sda_arready = 'b0;

    cl_sda_rdata   = 'b0;
  end

  //=============================================================================
  // SH_DDR
  //=============================================================================

  sh_ddr
    #(
      .DDR_PRESENT (EN_DDR)
    )
    SH_DDR
    (
      .clk                       (clk_main_a0 ),
      .rst_n                     (            ),
      .stat_clk                  (clk_main_a0 ),
      .stat_rst_n                (            ),
      .CLK_DIMM_DP               (CLK_DIMM_DP ),
      .CLK_DIMM_DN               (CLK_DIMM_DN ),
      .M_ACT_N                   (M_ACT_N     ),
      .M_MA                      (M_MA        ),
      .M_BA                      (M_BA        ),
      .M_BG                      (M_BG        ),
      .M_CKE                     (M_CKE       ),
      .M_ODT                     (M_ODT       ),
      .M_CS_N                    (M_CS_N      ),
      .M_CLK_DN                  (M_CLK_DN    ),
      .M_CLK_DP                  (M_CLK_DP    ),
      .M_PAR                     (M_PAR       ),
      .M_DQ                      (M_DQ        ),
      .M_ECC                     (M_ECC       ),
      .M_DQS_DP                  (M_DQS_DP    ),
      .M_DQS_DN                  (M_DQS_DN    ),
      .cl_RST_DIMM_N             (RST_DIMM_N  ),
      .cl_sh_ddr_axi_awid        (            ),
      .cl_sh_ddr_axi_awaddr      (            ),
      .cl_sh_ddr_axi_awlen       (            ),
      .cl_sh_ddr_axi_awsize      (            ),
      .cl_sh_ddr_axi_awvalid     (            ),
      .cl_sh_ddr_axi_awburst     (            ),
      .cl_sh_ddr_axi_awuser      (            ),
      .cl_sh_ddr_axi_awready     (            ),
      .cl_sh_ddr_axi_wdata       (            ),
      .cl_sh_ddr_axi_wstrb       (            ),
      .cl_sh_ddr_axi_wlast       (            ),
      .cl_sh_ddr_axi_wvalid      (            ),
      .cl_sh_ddr_axi_wready      (            ),
      .cl_sh_ddr_axi_bid         (            ),
      .cl_sh_ddr_axi_bresp       (            ),
      .cl_sh_ddr_axi_bvalid      (            ),
      .cl_sh_ddr_axi_bready      (            ),
      .cl_sh_ddr_axi_arid        (            ),
      .cl_sh_ddr_axi_araddr      (            ),
      .cl_sh_ddr_axi_arlen       (            ),
      .cl_sh_ddr_axi_arsize      (            ),
      .cl_sh_ddr_axi_arvalid     (            ),
      .cl_sh_ddr_axi_arburst     (            ),
      .cl_sh_ddr_axi_aruser      (            ),
      .cl_sh_ddr_axi_arready     (            ),
      .cl_sh_ddr_axi_rid         (            ),
      .cl_sh_ddr_axi_rdata       (            ),
      .cl_sh_ddr_axi_rresp       (            ),
      .cl_sh_ddr_axi_rlast       (            ),
      .cl_sh_ddr_axi_rvalid      (            ),
      .cl_sh_ddr_axi_rready      (            ),
      .sh_ddr_stat_bus_addr      (            ),
      .sh_ddr_stat_bus_wdata     (            ),
      .sh_ddr_stat_bus_wr        (            ),
      .sh_ddr_stat_bus_rd        (            ),
      .sh_ddr_stat_bus_ack       (            ),
      .sh_ddr_stat_bus_rdata     (            ),
      .ddr_sh_stat_int           (            ),
      .sh_cl_ddr_is_ready        (            )
    );

  always_comb
  begin
    cl_sh_ddr_stat_ack   = 'b0;
    cl_sh_ddr_stat_rdata = 'b0;
    cl_sh_ddr_stat_int   = 'b0;
  end

  //=============================================================================
  // USER-DEFIEND INTERRUPTS
  //=============================================================================

  always_comb
  begin
    cl_sh_apppf_irq_req = 'b0;
  end

  //=============================================================================
  // VIRTUAL JTAG
  //=============================================================================

  always_comb
  begin
    tdo = 'b0;
  end

  //=============================================================================
  // HBM MONITOR IO
  //=============================================================================

  always_comb
  begin
    hbm_apb_paddr_1   = 'b0;
    hbm_apb_pprot_1   = 'b0;
    hbm_apb_psel_1    = 'b0;
    hbm_apb_penable_1 = 'b0;
    hbm_apb_pwrite_1  = 'b0;
    hbm_apb_pwdata_1  = 'b0;
    hbm_apb_pstrb_1   = 'b0;
    hbm_apb_pready_1  = 'b0;
    hbm_apb_prdata_1  = 'b0;
    hbm_apb_pslverr_1 = 'b0;

    hbm_apb_paddr_0   = 'b0;
    hbm_apb_pprot_0   = 'b0;
    hbm_apb_psel_0    = 'b0;
    hbm_apb_penable_0 = 'b0;
    hbm_apb_pwrite_0  = 'b0;
    hbm_apb_pwdata_0  = 'b0;
    hbm_apb_pstrb_0   = 'b0;
    hbm_apb_pready_0  = 'b0;
    hbm_apb_prdata_0  = 'b0;
    hbm_apb_pslverr_0 = 'b0;
  end

  //=============================================================================
  //
  //=============================================================================

  always_comb
  begin
    PCIE_EP_TXP    = 'b0;
    PCIE_EP_TXN    = 'b0;

    PCIE_RP_PERSTN = 'b0;
    PCIE_RP_TXP    = 'b0;
    PCIE_RP_TXN    = 'b0;
  end

endmodule // design_top