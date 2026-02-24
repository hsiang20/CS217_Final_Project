// Copyright 2026 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DESIGN_TOP_H
#define DESIGN_TOP_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Top Module Interface Parameters from design_top_defines.vh
// ============================================================================

// AXI Write Address Channel (Host to Top)
#define WIDTH_TOP_AXI_AW 50
#define LOOP_TOP_AXI_AW ((WIDTH_TOP_AXI_AW + 31) / 32) // 2 words
#define ADDR_TOP_AXI_AW_START 0x400

// AXI Write Data Channel (Host to Top)
#define WIDTH_TOP_AXI_W 145
#define LOOP_TOP_AXI_W ((WIDTH_TOP_AXI_W + 31) / 32) // 5 words
#define ADDR_TOP_AXI_W_START 0x410

// AXI Write Response Channel (Top to Host)
#define WIDTH_TOP_AXI_B 12
#define LOOP_TOP_AXI_B ((WIDTH_TOP_AXI_B + 31) / 32) // 1 word
#define ADDR_TOP_AXI_B_START 0x430

// AXI Read Address Channel (Host to Top)
#define WIDTH_TOP_AXI_AR 50
#define LOOP_TOP_AXI_AR ((WIDTH_TOP_AXI_AR + 31) / 32) // 2 words
#define ADDR_TOP_AXI_AR_START 0x440

// AXI Read Data Channel (Top to Host)
#define WIDTH_TOP_AXI_R 141
#define LOOP_TOP_AXI_R ((WIDTH_TOP_AXI_R + 31) / 32) // 5 words
#define ADDR_TOP_AXI_R_START 0x450

// Interrupt (Top to Host)
#define ADDR_TOP_INTERRUPT 0x570 // Read

// FPGA OCL
#define WIDTH_AXI 32
#define ADDR_WIDTH_OCL 16

// ============================================================================
// Data Structures
// ============================================================================

typedef struct {
    uint32_t addr;
    uint32_t data[4]; // 128 bits
} AxiWriteCommand;

typedef struct {
    uint32_t addr;
    uint32_t data[4]; // 128 bits
    uint32_t expected_read_data[4]; // 128 bits
} AxiReadCommand;


// ============================================================================
// Function Prototypes
// ============================================================================

// Low-level MMIO functions
int ocl_wr32(int bar_handle, uint16_t addr, uint32_t data);
int ocl_rd32(int bar_handle, uint16_t addr, uint32_t* data);

// Top-level AXI interface functions
int top_write(int bar_handle, const AxiWriteCommand* write_command);
int top_read(int bar_handle, AxiReadCommand* read_command);

#endif // DESIGN_TOP_H