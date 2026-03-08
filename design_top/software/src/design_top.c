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
// Test Application for design_top on AWS F2 FPGA
// ============================================================================
// Tests the Transpose unit with both opcodes:
//   Opcode 0 — naive (element-by-element read+write)
//   Opcode 1 — optimized (banked BRAM fill/drain)
//
// Source matrix A (3 rows x 2 cols) at memory_index=0:
//   A[0][0]=0x01..01  A[0][1]=0x02..02
//   A[1][0]=0x03..03  A[1][1]=0x04..04
//   A[2][0]=0x05..05  A[2][1]=0x06..06
//
// Expected A^T (2 rows x 3 cols):
//   A^T[0][0]=01  A^T[0][1]=03  A^T[0][2]=05
//   A^T[1][0]=02  A^T[1][1]=04  A^T[1][2]=06
//
// Naive  result → memory_index=1 (SRAM base offset 0x400)
// Banked result → memory_index=2 (SRAM base offset 0x800)
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
// Low-level MMIO Functions
// ============================================================================

int ocl_wr32(int bar_handle, uint16_t addr, uint32_t data) {
  if (fpga_pci_poke(bar_handle, addr, data)) {
    fprintf(stderr, "ERROR: MMIO write failed at addr=0x%04x\n", addr);
    return 1;
  }
  return 0;
}

int ocl_rd32(int bar_handle, uint16_t addr, uint32_t* data) {
  if (fpga_pci_peek(bar_handle, addr, data)) {
    fprintf(stderr, "ERROR: MMIO read failed at addr=0x%04x\n", addr);
    return 1;
  }
  return 0;
}

// ============================================================================
// Top-level AXI Interface Functions
// ============================================================================

int top_write(int bar_handle, const AxiWriteCommand* cmd) {
    uint64_t transfer_addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t transfer_addr[LOOP_TOP_AXI_AW] = {0};

    transfer_addr[0] = transfer_addr_full & 0xFFFFFFFF;
    transfer_addr[1] = (transfer_addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AW; i++) {
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_AW_START + i * 4, transfer_addr[i]))
            return 1;
    }

    usleep(10);

    uint32_t transfer_data[LOOP_TOP_AXI_W] = {0};
    transfer_data[0] = cmd->data[0];
    transfer_data[1] = cmd->data[1];
    transfer_data[2] = cmd->data[2];
    transfer_data[3] = cmd->data[3];
    transfer_data[4] = 0x1FFFF; // Strobe

    for (int i = 0; i < LOOP_TOP_AXI_W; i++) {
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_W_START + i * 4, transfer_data[i]))
            return 1;
    }
    return 0;
}

int top_read(int bar_handle, AxiReadCommand* cmd) {
    uint64_t transfer_addr_full = ((uint64_t)cmd->addr << 10);
    uint32_t transfer_addr[LOOP_TOP_AXI_AR] = {0};

    transfer_addr[0] = transfer_addr_full & 0xFFFFFFFF;
    transfer_addr[1] = (transfer_addr_full >> 32) & 0x3FFFF;

    for (int i = 0; i < LOOP_TOP_AXI_AR; i++) {
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_AR_START + i * 4, transfer_addr[i]))
            return 1;
    }

    usleep(10);

    uint32_t transfer_data[LOOP_TOP_AXI_R] = {0};
    for (int i = 0; i < LOOP_TOP_AXI_R; i++) {
        if (ocl_rd32(bar_handle, ADDR_TOP_AXI_R_START + i * 4, &transfer_data[i]))
            return 1;
    }

    cmd->data[0] = (transfer_data[0] >> 10) | ((transfer_data[1] & 0x3FF) << 22);
    cmd->data[1] = (transfer_data[1] >> 10) | ((transfer_data[2] & 0x3FF) << 22);
    cmd->data[2] = (transfer_data[2] >> 10) | ((transfer_data[3] & 0x3FF) << 22);
    cmd->data[3] = (transfer_data[3] >> 10) | ((transfer_data[4] & 0x3FF) << 22);

    if (memcmp(cmd->data, cmd->expected_read_data, sizeof(cmd->data)) != 0) {
        fprintf(stderr, "\nMISMATCH at addr=0x%X\n", cmd->addr);
        fprintf(stderr, "  Read:     0x%08X_%08X_%08X_%08X\n",
                cmd->data[3], cmd->data[2], cmd->data[1], cmd->data[0]);
        fprintf(stderr, "  Expected: 0x%08X_%08X_%08X_%08X\n",
                cmd->expected_read_data[3], cmd->expected_read_data[2],
                cmd->expected_read_data[1], cmd->expected_read_data[0]);
        return 1;
    }
    printf("PASS: addr=0x%X data=0x%08X_%08X_%08X_%08X\n",
           cmd->addr, cmd->data[3], cmd->data[2], cmd->data[1], cmd->data[0]);
    return 0;
}

// ============================================================================
// Main Test Application
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

  // =========================================================================
  // Phase 0 — GBControl config
  // =========================================================================
  printf("\n===== Phase 0: GBControl config =====\n");
  AxiWriteCommand gb_cfg = {0x33400010, {0x00000002, 0x00400003, 0x00800003, 0x00000000}};
  if (top_write(bar_handle, &gb_cfg)) rc = 1;
  usleep(10);

  // =========================================================================
  // Phase 1 — Write source matrix (3 rows x 2 cols) into memory_index=0
  //   Address = 0x33500000 + vector_index*0x100 + timestep_index*0x10
  // =========================================================================
  printf("\n===== Phase 1: Write source matrix (3x2) =====\n");
  AxiWriteCommand src_data[] = {
      {0x33500000, {0x01010101, 0x01010101, 0x01010101, 0x01010101}}, // A[0][0]
      {0x33500100, {0x02020202, 0x02020202, 0x02020202, 0x02020202}}, // A[0][1]
      {0x33500010, {0x03030303, 0x03030303, 0x03030303, 0x03030303}}, // A[1][0]
      {0x33500110, {0x04040404, 0x04040404, 0x04040404, 0x04040404}}, // A[1][1]
      {0x33500020, {0x05050505, 0x05050505, 0x05050505, 0x05050505}}, // A[2][0]
      {0x33500120, {0x06060606, 0x06060606, 0x06060606, 0x06060606}}, // A[2][1]
  };
  for (int i = 0; i < 6; i++) {
      if (top_write(bar_handle, &src_data[i])) rc = 1;
      usleep(10);
  }

  // =========================================================================
  // Phase 2 — Naive transpose (opcode=0, src=0, dst=1, rows=3, cols=2)
  //   TransposeConfig bits: is_valid=1, src=0, dst=1, rows=3, cols=2, opcode=0
  //   128-bit = 0x00000000_00000000_00000203_00010001
  // =========================================================================
  printf("\n===== Phase 2: Naive transpose (opcode 0) =====\n");
  AxiWriteCommand naive_cfg = {0x33600010, {0x00010001, 0x00000203, 0x00000000, 0x00000000}};
  if (top_write(bar_handle, &naive_cfg)) rc = 1;
  usleep(10);

  AxiWriteCommand start_cmd = {0x33000030, {0, 0, 0, 0}};
  if (top_write(bar_handle, &start_cmd)) rc = 1;
  usleep(1000);

  printf("\n--- Verifying naive transpose (memory_index=1) ---\n");
  AxiReadCommand naive_reads[] = {
      {0x33500400, {0}, {0x01010101, 0x01010101, 0x01010101, 0x01010101}}, // A^T[0][0]
      {0x33500500, {0}, {0x03030303, 0x03030303, 0x03030303, 0x03030303}}, // A^T[0][1]
      {0x33500600, {0}, {0x05050505, 0x05050505, 0x05050505, 0x05050505}}, // A^T[0][2]
      {0x33500410, {0}, {0x02020202, 0x02020202, 0x02020202, 0x02020202}}, // A^T[1][0]
      {0x33500510, {0}, {0x04040404, 0x04040404, 0x04040404, 0x04040404}}, // A^T[1][1]
      {0x33500610, {0}, {0x06060606, 0x06060606, 0x06060606, 0x06060606}}, // A^T[1][2]
  };
  for (int i = 0; i < 6; i++) {
      if (top_read(bar_handle, &naive_reads[i])) rc = 1;
      usleep(10);
  }

  uint32_t interrupt_naive = 0;
  ocl_rd32(bar_handle, ADDR_TOP_INTERRUPT, &interrupt_naive);
  printf("Interrupt counter after naive: %u\n", interrupt_naive);

  // =========================================================================
  // Phase 3 — Optimized transpose (opcode=1, src=0, dst=2, rows=3, cols=2)
  //   TransposeConfig bits: is_valid=1, src=0, dst=2, rows=3, cols=2, opcode=1
  //   128-bit = 0x00000000_00000000_00010203_00020001
  // =========================================================================
  printf("\n===== Phase 3: Optimized transpose (opcode 1, banked BRAM) =====\n");
  AxiWriteCommand opt_cfg = {0x33600010, {0x00020001, 0x00010203, 0x00000000, 0x00000000}};
  if (top_write(bar_handle, &opt_cfg)) rc = 1;
  usleep(10);

  if (top_write(bar_handle, &start_cmd)) rc = 1;
  usleep(1000);

  printf("\n--- Verifying optimized transpose (memory_index=2) ---\n");
  AxiReadCommand opt_reads[] = {
      {0x33500800, {0}, {0x01010101, 0x01010101, 0x01010101, 0x01010101}}, // A^T[0][0]
      {0x33500900, {0}, {0x03030303, 0x03030303, 0x03030303, 0x03030303}}, // A^T[0][1]
      {0x33500A00, {0}, {0x05050505, 0x05050505, 0x05050505, 0x05050505}}, // A^T[0][2]
      {0x33500810, {0}, {0x02020202, 0x02020202, 0x02020202, 0x02020202}}, // A^T[1][0]
      {0x33500910, {0}, {0x04040404, 0x04040404, 0x04040404, 0x04040404}}, // A^T[1][1]
      {0x33500A10, {0}, {0x06060606, 0x06060606, 0x06060606, 0x06060606}}, // A^T[1][2]
  };
  for (int i = 0; i < 6; i++) {
      if (top_read(bar_handle, &opt_reads[i])) rc = 1;
      usleep(10);
  }

  uint32_t interrupt_opt = 0;
  ocl_rd32(bar_handle, ADDR_TOP_INTERRUPT, &interrupt_opt);
  printf("Interrupt counter after optimized: %u\n", interrupt_opt);

  // =========================================================================
  // Summary
  // =========================================================================
  printf("\n===== Performance Summary =====\n");
  printf("Interrupt cycles (naive):     %u\n", interrupt_naive);
  printf("Interrupt cycles (optimized): %u (delta = %u)\n",
         interrupt_opt, interrupt_opt - interrupt_naive);

  printf("\n---- TEST %s ----\n", (rc == 0) ? "PASSED" : "FAILED");

  if (bar_handle != -1) {
    fpga_pci_detach(bar_handle);
  }
  return rc;
}
