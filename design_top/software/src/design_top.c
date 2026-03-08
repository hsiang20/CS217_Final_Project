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

// ============================================================================
// Transpose Test Application for design_top on AWS F2 FPGA
// ============================================================================
// Exercises both transpose opcodes (naive / banked-BRAM) at multiple matrix
// sizes: 1x1, 2x3, 3x2, 4x4.
//
// SRAM address encoding:
//   0x33500000 + memory_index*0x400 + vector_index*0x100 + timestep_index*0x10
//
// Matrix element A[r][c] is stored at (timestep=r, vector=c).
// After transpose, A[r][c] appears at dst (timestep=c, vector=r).
// ============================================================================

#include "design_top.h"
#include <fpga_mgmt.h>
#include <fpga_pci.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ============================================================================
// Low-level MMIO
// ============================================================================

int ocl_wr32(int bh, uint16_t addr, uint32_t data) {
  if (fpga_pci_poke(bh, addr, data)) {
    fprintf(stderr, "ERROR: MMIO write failed at 0x%04x\n", addr);
    return 1;
  }
  return 0;
}

int ocl_rd32(int bh, uint16_t addr, uint32_t* data) {
  if (fpga_pci_peek(bh, addr, data)) {
    fprintf(stderr, "ERROR: MMIO read failed at 0x%04x\n", addr);
    return 1;
  }
  return 0;
}

// ============================================================================
// AXI Write / Read through OCL bridge
// ============================================================================

int top_write(int bh, const AxiWriteCommand* cmd) {
    uint64_t addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t aw[LOOP_TOP_AXI_AW] = {0};
    aw[0] = addr_full & 0xFFFFFFFF;
    aw[1] = (addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AW; i++)
        if (ocl_wr32(bh, ADDR_TOP_AXI_AW_START + i*4, aw[i])) return 1;
    usleep(10);

    uint32_t w[LOOP_TOP_AXI_W] = {0};
    w[0] = cmd->data[0]; w[1] = cmd->data[1];
    w[2] = cmd->data[2]; w[3] = cmd->data[3];
    w[4] = 0x1FFFF;
    for (int i = 0; i < LOOP_TOP_AXI_W; i++)
        if (ocl_wr32(bh, ADDR_TOP_AXI_W_START + i*4, w[i])) return 1;
    return 0;
}

int top_read(int bh, AxiReadCommand* cmd) {
    uint64_t addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t ar[LOOP_TOP_AXI_AR] = {0};
    ar[0] = addr_full & 0xFFFFFFFF;
    ar[1] = (addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++)
        if (ocl_wr32(bh, ADDR_TOP_AXI_AR_START + i*4, ar[i])) return 1;
    usleep(10);

    uint32_t rd[LOOP_TOP_AXI_R] = {0};
    for (int i = 0; i < LOOP_TOP_AXI_R; i++)
        if (ocl_rd32(bh, ADDR_TOP_AXI_R_START + i*4, &rd[i])) return 1;

    cmd->data[0] = (rd[0] >> 10) | ((rd[1] & 0x3FF) << 22);
    cmd->data[1] = (rd[1] >> 10) | ((rd[2] & 0x3FF) << 22);
    cmd->data[2] = (rd[2] >> 10) | ((rd[3] & 0x3FF) << 22);
    cmd->data[3] = (rd[3] >> 10) | ((rd[4] & 0x3FF) << 22);

    if (memcmp(cmd->data, cmd->expected_read_data, sizeof(cmd->data)) != 0) {
        fprintf(stderr, "  MISMATCH addr=0x%X\n", cmd->addr);
        fprintf(stderr, "    got: 0x%08X_%08X_%08X_%08X\n",
                cmd->data[3], cmd->data[2], cmd->data[1], cmd->data[0]);
        fprintf(stderr, "    exp: 0x%08X_%08X_%08X_%08X\n",
                cmd->expected_read_data[3], cmd->expected_read_data[2],
                cmd->expected_read_data[1], cmd->expected_read_data[0]);
        return 1;
    }
    printf("  PASS addr=0x%X\n", cmd->addr);
    return 0;
}

// ============================================================================
// Helpers
// ============================================================================

static uint32_t sram_addr(int mem, int vec, int ts) {
    return 0x33500000 + mem * 0x400 + vec * 0x100 + ts * 0x10;
}

static void fill_vector(uint32_t data[4], uint8_t val) {
    uint32_t w = val | (val << 8) | (val << 16) | (val << 24);
    data[0] = data[1] = data[2] = data[3] = w;
}

static void make_transpose_cfg(uint32_t data[4],
                                int src, int dst, int rows, int cols, int opcode) {
    data[0] = 1 | ((src & 7) << 8) | ((dst & 7) << 16);
    data[1] = (rows & 0xFF) | ((cols & 0xFF) << 8) | ((opcode & 7) << 16);
    data[2] = 0;
    data[3] = 0;
}

// ============================================================================
// Run one transpose test (both opcodes) for a given matrix size
//
//   Source A[r][c] at (timestep=r, vector=c), value = r*cols+c+1
//   After transpose: A[r][c] at dst (timestep=c, vector=r)
// ============================================================================

static int run_transpose_test(int bh, const char* label,
                               int rows, int cols,
                               int src_mem, int dst_naive, int dst_opt) {
    int rc = 0;
    AxiWriteCommand wcmd;
    AxiReadCommand  rcmd;
    uint8_t val;

    printf("\n=========================================================\n");
    printf(" Test: %s  (%dx%d)  src=%d  naive_dst=%d  opt_dst=%d\n",
           label, rows, cols, src_mem, dst_naive, dst_opt);
    printf("=========================================================\n");

    // --- Write source matrix ---
    printf("  Writing source matrix ...\n");
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            val = r * cols + c + 1;
            wcmd.addr = sram_addr(src_mem, c, r);
            fill_vector(wcmd.data, val);
            if (top_write(bh, &wcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Naive transpose (opcode 0) ---
    printf("  Running naive transpose (opcode 0) ...\n");
    wcmd.addr = 0x33600010;
    make_transpose_cfg(wcmd.data, src_mem, dst_naive, rows, cols, 0);
    if (top_write(bh, &wcmd)) rc = 1;
    usleep(10);

    wcmd.addr = 0x33000030;
    memset(wcmd.data, 0, sizeof(wcmd.data));
    if (top_write(bh, &wcmd)) rc = 1;
    usleep(1000);

    printf("  Verifying naive result (mem=%d) ...\n", dst_naive);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            val = r * cols + c + 1;
            rcmd.addr = sram_addr(dst_naive, r, c);
            memset(rcmd.data, 0, sizeof(rcmd.data));
            fill_vector(rcmd.expected_read_data, val);
            if (top_read(bh, &rcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Optimized transpose (opcode 1) ---
    printf("  Running optimized transpose (opcode 1, banked BRAM) ...\n");
    wcmd.addr = 0x33600010;
    make_transpose_cfg(wcmd.data, src_mem, dst_opt, rows, cols, 1);
    if (top_write(bh, &wcmd)) rc = 1;
    usleep(10);

    wcmd.addr = 0x33000030;
    memset(wcmd.data, 0, sizeof(wcmd.data));
    if (top_write(bh, &wcmd)) rc = 1;
    usleep(1000);

    printf("  Verifying optimized result (mem=%d) ...\n", dst_opt);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            val = r * cols + c + 1;
            rcmd.addr = sram_addr(dst_opt, r, c);
            memset(rcmd.data, 0, sizeof(rcmd.data));
            fill_vector(rcmd.expected_read_data, val);
            if (top_read(bh, &rcmd)) rc = 1;
            usleep(10);
        }
    }

    return rc;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <slot_id>\n", argv[0]);
        return 1;
    }

    int slot_id    = atoi(argv[1]);
    int bar_handle = -1;
    int rc         = 0;

    if (fpga_mgmt_init() != 0) {
        fprintf(stderr, "Failed to initialize fpga_mgmt\n");
        return 1;
    }
    if (fpga_pci_attach(slot_id, FPGA_APP_PF, APP_PF_BAR0, 0, &bar_handle)) {
        fprintf(stderr, "fpga_pci_attach failed\n");
        return 1;
    }
    printf("---- System Initialization (bar_handle: %d) ----\n", bar_handle);

    // GBControl configuration (required before any GB operations)
    printf("\n===== GBControl config =====\n");
    AxiWriteCommand gb_cfg = {0x33400010, {0x00000002, 0x00400003, 0x00800003, 0x00000000}};
    if (top_write(bar_handle, &gb_cfg)) rc = 1;
    usleep(10);

    // ---- Test suite: multiple matrix sizes ----
    if (run_transpose_test(bar_handle, "1x1", 1, 1, 0, 1, 2)) rc = 1;
    if (run_transpose_test(bar_handle, "2x3", 2, 3, 0, 1, 2)) rc = 1;
    if (run_transpose_test(bar_handle, "3x2", 3, 2, 0, 1, 2)) rc = 1;
    if (run_transpose_test(bar_handle, "4x4", 4, 4, 0, 1, 2)) rc = 1;

    // Read interrupt counter
    uint32_t interrupt_cycles = 0;
    ocl_rd32(bar_handle, ADDR_TOP_INTERRUPT, &interrupt_cycles);
    printf("\nTotal interrupt cycles across all tests: %u\n", interrupt_cycles);

    printf("\n---- TEST %s ----\n", (rc == 0) ? "PASSED" : "FAILED");

    if (bar_handle != -1)
        fpga_pci_detach(bar_handle);
    return rc;
}
