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
// Matmul (Q * K^T) Test Application for design_top on AWS F2 FPGA
// ============================================================================
// Compares unfused (opcode 2) vs fused (opcode 3) Q*K^T computation.
//
// Unfused: naive transpose K -> K^T (SRAM), then matmul from SRAM
// Fused:   fill K into BRAM, then matmul reading K^T from BRAM directly
//
// Memory layout: K at mem 0, Q at mem 1, result at mem 2, K^T temp at mem 3
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
// Low-level MMIO (same as design_top.c)
// ============================================================================

static int mm_ocl_wr32(int bh, uint16_t addr, uint32_t data) {
  if (fpga_pci_poke(bh, addr, data)) {
    fprintf(stderr, "ERROR: MMIO write failed at 0x%04x\n", addr);
    return 1;
  }
  return 0;
}

static int mm_ocl_rd32(int bh, uint16_t addr, uint32_t* data) {
  if (fpga_pci_peek(bh, addr, data)) {
    fprintf(stderr, "ERROR: MMIO read failed at 0x%04x\n", addr);
    return 1;
  }
  return 0;
}

// ============================================================================
// AXI Write / Read through OCL bridge
// ============================================================================

static int mm_top_write(int bh, const AxiWriteCommand* cmd) {
    uint64_t addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t aw[LOOP_TOP_AXI_AW] = {0};
    aw[0] = addr_full & 0xFFFFFFFF;
    aw[1] = (addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AW; i++)
        if (mm_ocl_wr32(bh, ADDR_TOP_AXI_AW_START + i*4, aw[i])) return 1;
    usleep(10);

    uint32_t w[LOOP_TOP_AXI_W] = {0};
    w[0] = cmd->data[0]; w[1] = cmd->data[1];
    w[2] = cmd->data[2]; w[3] = cmd->data[3];
    w[4] = 0x1FFFF;
    for (int i = 0; i < LOOP_TOP_AXI_W; i++)
        if (mm_ocl_wr32(bh, ADDR_TOP_AXI_W_START + i*4, w[i])) return 1;
    return 0;
}

static int mm_top_read(int bh, AxiReadCommand* cmd) {
    uint64_t addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t ar[LOOP_TOP_AXI_AR] = {0};
    ar[0] = addr_full & 0xFFFFFFFF;
    ar[1] = (addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++)
        if (mm_ocl_wr32(bh, ADDR_TOP_AXI_AR_START + i*4, ar[i])) return 1;

    uint32_t rd[LOOP_TOP_AXI_R] = {0};
    int poll_count = 0;
    rd[0] = 0xDEADBEEF;
    while (rd[0] == 0xDEADBEEF && poll_count < 1000) {
        usleep(1);
        if (mm_ocl_rd32(bh, ADDR_TOP_AXI_R_START, &rd[0])) return 1;
        poll_count++;
    }
    if (rd[0] == 0xDEADBEEF) {
        fprintf(stderr, "ERROR: AXI read timeout for addr=0x%X\n", cmd->addr);
        return 1;
    }
    for (int i = 1; i < LOOP_TOP_AXI_R; i++)
        if (mm_ocl_rd32(bh, ADDR_TOP_AXI_R_START + i*4, &rd[i])) return 1;

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
    uint32_t flat = (uint32_t)(mem * 1024 + (ts & 0xF) + ((ts >> 4) * 32 + vec) * 16);
    return 0x33500000 + flat * 0x10;
}

static void fill_vector(uint32_t data[4], uint8_t val) {
    uint32_t w = val | (val << 8) | (val << 16) | (val << 24);
    data[0] = data[1] = data[2] = data[3] = w;
}

static void make_matmul_cfg(uint32_t data[4],
                             int src, int dst, int q_mem,
                             int n_rows, int n_cols, int opcode, int m_rows) {
    data[0] = 1 | ((src & 7) << 8) | ((dst & 7) << 16) | ((q_mem & 7) << 24);
    data[1] = (n_rows & 0xFF) | ((n_cols & 0xFF) << 8) | ((opcode & 7) << 16) | ((m_rows & 0xFF) << 24);
    data[2] = 0;
    data[3] = 0;
}

static uint8_t expected_matmul(int i, int j, int D) {
    uint32_t sum = 0;
    for (int k = 0; k < D; k++) {
        uint32_t q_val = (i * D + k) % 255 + 1;
        uint32_t k_val = (j * D + k) % 255 + 1;
        sum += q_val * k_val;
    }
    return (uint8_t)(sum & 0xFF);
}

// ============================================================================
// Run one matmul test (both unfused and fused) for given dimensions
// ============================================================================

static int run_matmul_test(int bh, const char* label,
                            int M, int N, int D,
                            int src_mem, int q_mem, int dst_mem,
                            int overhead,
                            uint32_t* out_unfused, uint32_t* out_fused) {
    int rc = 0;
    AxiWriteCommand wcmd;
    AxiReadCommand  rcmd;
    uint8_t val;
    uint32_t unfused_cycles, fused_cycles;

    printf("\n=========================================================\n");
    printf(" Matmul Test: %s  Q(%dx%d) * K(%dx%d)^T = (%dx%d)\n",
           label, M, D, N, D, M, N);
    printf("   src(K)=%d  q(Q)=%d  dst(result)=%d\n", src_mem, q_mem, dst_mem);
    printf("=========================================================\n");

    // --- Write K matrix (N x D) ---
    printf("  Writing K matrix (%dx%d) at mem %d ...\n", N, D, src_mem);
    for (int r = 0; r < N; r++) {
        for (int c = 0; c < D; c++) {
            val = (uint8_t)((r * D + c) % 255 + 1);
            wcmd.addr = sram_addr(src_mem, c, r);
            fill_vector(wcmd.data, val);
            if (mm_top_write(bh, &wcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Write Q matrix (M x D) ---
    printf("  Writing Q matrix (%dx%d) at mem %d ...\n", M, D, q_mem);
    for (int r = 0; r < M; r++) {
        for (int c = 0; c < D; c++) {
            val = (uint8_t)((r * D + c) % 255 + 1);
            wcmd.addr = sram_addr(q_mem, c, r);
            fill_vector(wcmd.data, val);
            if (mm_top_write(bh, &wcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Unfused Q*K^T (opcode 2) ---
    printf("  Running UNFUSED Q*K^T (opcode 2) ...\n");
    wcmd.addr = 0x33600010;
    make_matmul_cfg(wcmd.data, src_mem, dst_mem, q_mem, N, D, 2, M);
    if (mm_top_write(bh, &wcmd)) rc = 1;
    usleep(10);

    mm_ocl_wr32(bh, ADDR_TOP_INTERRUPT, 0);
    wcmd.addr = 0x33000030;
    memset(wcmd.data, 0, sizeof(wcmd.data));
    if (mm_top_write(bh, &wcmd)) rc = 1;
    usleep(1000 + (N * D + M * N * D) * 5);
    mm_ocl_rd32(bh, ADDR_TOP_INTERRUPT, &unfused_cycles);

    printf("  Verifying unfused result ...\n");
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            val = expected_matmul(i, j, D);
            rcmd.addr = sram_addr(dst_mem, j, i);
            memset(rcmd.data, 0, sizeof(rcmd.data));
            fill_vector(rcmd.expected_read_data, val);
            if (mm_top_read(bh, &rcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Fused Q*K^T (opcode 3) ---
    printf("  Running FUSED Q*K^T (opcode 3) ...\n");
    wcmd.addr = 0x33600010;
    make_matmul_cfg(wcmd.data, src_mem, dst_mem, q_mem, N, D, 3, M);
    if (mm_top_write(bh, &wcmd)) rc = 1;
    usleep(10);

    mm_ocl_wr32(bh, ADDR_TOP_INTERRUPT, 0);
    wcmd.addr = 0x33000030;
    memset(wcmd.data, 0, sizeof(wcmd.data));
    if (mm_top_write(bh, &wcmd)) rc = 1;
    usleep(1000 + (N * D + M * N * D) * 5);
    mm_ocl_rd32(bh, ADDR_TOP_INTERRUPT, &fused_cycles);

    printf("  Verifying fused result ...\n");
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < N; j++) {
            val = expected_matmul(i, j, D);
            rcmd.addr = sram_addr(dst_mem, j, i);
            memset(rcmd.data, 0, sizeof(rcmd.data));
            fill_vector(rcmd.expected_read_data, val);
            if (mm_top_read(bh, &rcmd)) rc = 1;
            usleep(10);
        }
    }

    // --- Performance comparison ---
    *out_unfused = unfused_cycles;
    *out_fused   = fused_cycles;
    int adj_unfused = ((int)unfused_cycles > overhead) ? ((int)unfused_cycles - overhead) : 1;
    int adj_fused   = ((int)fused_cycles   > overhead) ? ((int)fused_cycles   - overhead) : 1;
    printf("  >> %s  Q(%dx%d)*K(%dx%d)^T:\n", label, M, D, N, D);
    printf("     Raw:      unfused=%u cyc, fused=%u cyc, speedup=%.2fx\n",
           unfused_cycles, fused_cycles,
           fused_cycles > 0 ? (double)unfused_cycles / fused_cycles : 0.0);
    printf("     Adjusted: unfused=%d cyc, fused=%d cyc, speedup=%.2fx\n",
           adj_unfused, adj_fused,
           adj_fused > 0 ? (double)adj_unfused / adj_fused : 0.0);

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
    uint32_t unfused_out, fused_out;
    int perf_overhead;

    if (fpga_mgmt_init() != 0) {
        fprintf(stderr, "Failed to initialize fpga_mgmt\n");
        return 1;
    }
    if (fpga_pci_attach(slot_id, FPGA_APP_PF, APP_PF_BAR0, 0, &bar_handle)) {
        fprintf(stderr, "fpga_pci_attach failed\n");
        return 1;
    }
    printf("---- System Initialization (bar_handle: %d) ----\n", bar_handle);

    // GBControl: 4 memory regions (base={0,1024,2048,3072}, num_vec=32)
    printf("\n===== GBControl config (4 regions) =====\n");
    AxiWriteCommand gb_cfg = {0x33400010, {0x00000020, 0x04000020, 0x08000020, 0x0C000020}};
    if (mm_top_write(bar_handle, &gb_cfg)) rc = 1;
    usleep(10);

    // Overhead calibration
    if (run_matmul_test(bar_handle, "1x1x1", 1, 1, 1, 0, 1, 2, 0, &unfused_out, &fused_out)) rc = 1;
    perf_overhead = (int)(unfused_out + fused_out) / 2;
    printf("\n  ** Matmul perf overhead estimate (from 1x1x1 avg): %d cycles **\n", perf_overhead);

    // Test suite
    if (run_matmul_test(bar_handle, "2x2x2",   2, 2, 2, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "4x4x4",   4, 4, 4, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "4x4x8",   4, 4, 8, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "8x8x4",   8, 8, 4, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "8x8x8",   8, 8, 8, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "4x8x4",   4, 8, 4, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "16x16x8", 16,16, 8, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;
    if (run_matmul_test(bar_handle, "16x16x16",16,16,16, 0, 1, 2, perf_overhead, &unfused_out, &fused_out)) rc = 1;

    printf("\n---- TEST %s ----\n", (rc == 0) ? "PASSED" : "FAILED");

    if (bar_handle != -1)
        fpga_pci_detach(bar_handle);
    return rc;
}
