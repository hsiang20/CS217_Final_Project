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
// This application tests the design_top AXI interface on an AWS F2 instance.
// It mimics the SystemVerilog testbench (verif/tests/design_top_base_test.sv)
// to perform basic AXI write and read operations.
//
// Test steps:
//  (a) Initialize FPGA and PCI interface.
//  (b) Perform a series of AXI write operations.
//  (c) Perform a series of AXI read operations and verify the data.
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

// #define DEBUG

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

/**
 * @brief Send an AXI write command to the FPGA.
 *
 * Mimics the 'top_write' task in the SystemVerilog testbench.
 */
int top_write(int bar_handle, const AxiWriteCommand* write_command) {
    uint64_t transfer_addr_full = ((uint64_t)write_command->addr << 10);
    uint32_t transfer_addr[LOOP_TOP_AXI_AW] = {0};

    transfer_addr[0] = transfer_addr_full & 0xFFFFFFFF;
    transfer_addr[1] = (transfer_addr_full >> 32) & 0x3FFFF; // 18 bits

    // Write address to AW channel
    for (int i = 0; i < LOOP_TOP_AXI_AW; i++) {
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_AW_START + i * 4, transfer_addr[i])) {
            return 1;
        }
    }

    usleep(10); // Small delay

    // Write data to W channel
    uint32_t transfer_data[LOOP_TOP_AXI_W] = {0};
    transfer_data[0] = write_command->data[0];
    transfer_data[1] = write_command->data[1];
    transfer_data[2] = write_command->data[2];
    transfer_data[3] = write_command->data[3];
    transfer_data[4] = 0x1FFFF; // Strobe

    for (int i = 0; i < LOOP_TOP_AXI_W; i++) {
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_W_START + i * 4, transfer_data[i])) {
            return 1;
        }
    }
    return 0;
}


/**
 * @brief Send an AXI read command and retrieve data from the FPGA.
 *
 * Mimics the 'top_read' task in the SystemVerilog testbench.
 */
int top_read(int bar_handle, AxiReadCommand* read_command) {
    uint64_t transfer_addr_full = ((uint64_t)read_command->addr << 10);
    uint32_t transfer_addr[LOOP_TOP_AXI_AR] = {0};

    transfer_addr[0] = transfer_addr_full & 0xFFFFFFFF;
    transfer_addr[1] = (transfer_addr_full >> 32) & 0x3FFFF; // 18 bits

    // Write address to AR channel
    for (int i = 0; i < LOOP_TOP_AXI_AR; i++) {
      //data sent
        if (ocl_wr32(bar_handle, ADDR_TOP_AXI_AR_START + i * 4, transfer_addr[i])) {
            return 1;
        }
    }

    usleep(10); // Small delay

    // Read data from R channel
    uint32_t transfer_data[LOOP_TOP_AXI_R] = {0};
    for (int i = 0; i < LOOP_TOP_AXI_R; i++) {
        if (ocl_rd32(bar_handle, ADDR_TOP_AXI_R_START + i * 4, &transfer_data[i])) {
            return 1;
        }
    }

    // Unpack data (141 bits total, data is in bits 137:10)
    read_command->data[0] = (transfer_data[0] >> 10) | ((transfer_data[1] & 0x3FF) << 22);
    read_command->data[1] = (transfer_data[1] >> 10) | ((transfer_data[2] & 0x3FF) << 22);
    read_command->data[2] = (transfer_data[2] >> 10) | ((transfer_data[3] & 0x3FF) << 22);
    read_command->data[3] = (transfer_data[3] >> 10) | ((transfer_data[4] & 0x3FF) << 22);


    // Verify data
    if (memcmp(read_command->data, read_command->expected_read_data, sizeof(read_command->data)) != 0) {
        fprintf(stderr, "\nRead data vs expected data mismatch!\n");
        fprintf(stderr, "  Address: 0x%X\n", read_command->addr);
        fprintf(stderr, "  Read:      0x%08X_%08X_%08X_%08X\n", read_command->data[3], read_command->data[2], read_command->data[1], read_command->data[0]);
        fprintf(stderr, "  Expected:  0x%08X_%08X_%08X_%08X\n", read_command->expected_read_data[3], read_command->expected_read_data[2], read_command->expected_read_data[1], read_command->expected_read_data[0]);
        return 1; // Mismatch
    } else {
        printf("Read value matches the expected = 0x%08X_%08X_%08X_%08X at 0x%X\n", read_command->data[3], read_command->data[2], read_command->data[1], read_command->data[0], read_command->addr);
    }

    return 0; // Success
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

  // ========================================================================= 
  // 1. Initialization and Attachment
  // ========================================================================= 
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
  // Test Sequence
  // ========================================================================= 
  printf("\n---- Running AXI Write/Read Test ----\n");

  AxiWriteCommand write_commands[] = {
      {0x33500000, {0x9EE3E635, 0x584169B2, 0xA0A882BF, 0xD4C04352}},
      {0x34500000, {0x88E1D68C, 0x6BD421D7, 0x5C7F3202, 0xC7427867}},
      {0x34500010, {0x5EC23966, 0xA174272E, 0x21E7A2FD, 0xD0319B6C}},
      {0x34500020, {0x178B1B85, 0xA331DDE2, 0xB8E9DD33, 0x5781547C}},
      {0x34500030, {0x8D22BBEB, 0x4E92D920, 0x04BCB961, 0x4C8C4B83}},
      {0x34500040, {0xC5BFA479, 0x0DC7A487, 0xA9D9B720, 0x67AD5414}},
      {0x34500050, {0x4A09CF2D, 0x0292B32C, 0xD70083F7, 0x69AB46F7}},
      {0x34500060, {0x27208FCB, 0xD103A7F4, 0x9B261E3F, 0x161F6574}},
      {0x34500070, {0xE1356000, 0xED6A7A4A, 0xB2819ED0, 0xAABCB5EF}},
      {0x34500080, {0xA9695EE4, 0xC59C9EC4, 0x5D2D4CDA, 0xF6D7D941}},
      {0x34500090, {0x3E0DFD81, 0x7F151973, 0xF78E9E7F, 0x17899ACB}},
      {0x345000A0, {0x4E3AD635, 0xACC64781, 0x69A343A4, 0xFCFD96D1}},
      {0x345000B0, {0x58371BA5, 0x8582459D, 0xE065D484, 0x5C0F148D}},
      {0x345000C0, {0x1E5515A8, 0xA96684FC, 0xB30AE0F6, 0xCC77DBF7}},
      {0x345000D0, {0xDF439320, 0xC97FD011, 0x13F2CD9D, 0xC5AA4918}},
      {0x345000E0, {0x2C4EE908, 0x520BE5B5, 0x72129DD4, 0xB8E6F69A}},
      {0x345000F0, {0xAE2D9CD6, 0x679295D0, 0xDFD4E551, 0x38B305DE}},
      {0x34400010, {0x1, 0x101, 0x0, 0x0}},
      {0x34400020, {0x100, 0x0, 0x0, 0x0}},
      {0x34800010, {0x3020001, 0x1, 0x0, 0x0}},
      {0x34800020, {0x40B030, 0x0, 0x0, 0x0}},
      {0x33400010, {0x1, 0x0, 0x0, 0x0}},
      {0x33700010, {0x1, 0x1010100, 0x10001, 0x0}},
      {0x33000010, {0x0, 0x0, 0x0, 0x0}},
      {0x33400010, {0x10001, 0x0, 0x0, 0x0}},
      {0x33500010, {0x761D3767, 0x5D0340C6, 0x3652115C, 0x298E1EFC}},
      {0x33C00010, {0x101, 0x10000, 0x1, 0x0}},
      {0x33000020, {0x0, 0x0, 0x0, 0x0}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
      {0x345000F0, {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF}},
  };

  AxiReadCommand read_commands[] = {
      {0x33500010, {0}, {0x8000003, 0x1000000, 0x1, 0x0}},
      {0x33500000, {0}, {0x10101010, 0x10101010, 0x10101010, 0x10101010}},
      {0x34600000, {0}, {0x9EE3E635, 0x584169B2, 0xA0A882BF, 0xD4C04352}},
   };

  int num_write_commands = sizeof(write_commands) / sizeof(AxiWriteCommand);
  for (int i = 0; i < num_write_commands; i++) {
      if (top_write(bar_handle, &write_commands[i])) {
          rc = 1;
      }
      usleep(10);
  }

  int num_read_commands = sizeof(read_commands) / sizeof(AxiReadCommand);
  for (int i = 0; i < num_read_commands; i++) {
      if (top_read(bar_handle, &read_commands[i])) {
          rc = 1;
      }
      usleep(10);
  }

  // =========================================================================
  // Read Interrupt Cycles Counter
  // =========================================================================
  uint32_t interrupt_cycles = 0;
  printf("\n---- Reading Interrupt Cycles Counter ----\n");
  if (ocl_rd32(bar_handle, ADDR_TOP_INTERRUPT, &interrupt_cycles)) {
      rc = 1;
  }
  
  printf("Interrupt cycles = %u\n", interrupt_cycles);
  if (interrupt_cycles <= 10) {
      fprintf(stderr, "ERROR: Interrupt cycles lesser than expected! Interrupt cycles = %u\n", interrupt_cycles);
      rc = 1;
  }

  // ========================================================================= 
  // Test Complete
  // ========================================================================= 
  printf("\n---- TEST %s ----\n", (rc == 0) ? "PASSED" : "FAILED");

  if (bar_handle != -1) {
    fpga_pci_detach(bar_handle);
  }

  return rc;
}