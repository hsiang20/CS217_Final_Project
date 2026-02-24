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
`ifndef design_top_DEFINES
`define design_top_DEFINES

        // Put module name of the CL design here. This is used to instantiate in top.sv
`define CL_NAME design_top

`define DDR_A_ABSENT
`define DDR_B_ABSENT


`endif

// ============================================================================
// Top Module Interface Parameters
// ============================================================================

// AXI Write Address Channel (Host to Top)
localparam int WIDTH_TOP_AXI_AW = 50;
localparam int LOOP_TOP_AXI_AW = (WIDTH_TOP_AXI_AW + 31) / 32; // 2 words
localparam int ADDR_TOP_AXI_AW_START = 16'h400;

// AXI Write Data Channel (Host to Top)
localparam int WIDTH_TOP_AXI_W = 145;
localparam int LOOP_TOP_AXI_W = (WIDTH_TOP_AXI_W + 31) / 32; // 5 words
localparam int ADDR_TOP_AXI_W_START = 16'h410;

// AXI Write Response Channel (Top to Host)
localparam int WIDTH_TOP_AXI_B = 12;
localparam int LOOP_TOP_AXI_B = (WIDTH_TOP_AXI_B + 31) / 32; // 1 word
localparam int ADDR_TOP_AXI_B_START = 16'h430;

// AXI Read Address Channel (Host to Top)
localparam int WIDTH_TOP_AXI_AR = 50;
localparam int LOOP_TOP_AXI_AR = (WIDTH_TOP_AXI_AR + 31) / 32; // 2 words
localparam int ADDR_TOP_AXI_AR_START = 16'h440;

// AXI Read Data Channel (Top to Host)
localparam int WIDTH_TOP_AXI_R = 141;
localparam int LOOP_TOP_AXI_R = (WIDTH_TOP_AXI_R + 31) / 32; // 5 words
localparam int ADDR_TOP_AXI_R_START = 16'h450;

// Interrupt (Top to Host)
localparam int ADDR_TOP_INTERRUPT = 16'h570; // Read

// FPGA OCL 
localparam int WIDTH_AXI = 32;
localparam int ADDR_WIDTH_OCL = 16;

