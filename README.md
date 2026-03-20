# CS217 Final Project: Efficient Matrix Transpose and Fused QKᵀ for ML Workloads on FPGA

This project extends the Lab 4 NVDLA-derived accelerator with a hardware
**matrix transpose unit** and a **fused QKᵀ matmul** module, both implemented
in SystemC/HLS and verified end-to-end on the AWS F2 FPGA platform.

---

## Table of Contents

- [Key Concepts](#key-concepts)
- [File Structure](#file-structure)
- [Environment Setup](#environment-setup)
- [Step 1 — SystemC Functional Simulation](#step-1--systemc-functional-simulation)
- [Step 2 — HLS Synthesis (Catapult Ultra)](#step-2--hls-synthesis-catapult-ultra)
- [Step 3 — RTL Simulation (Vivado XSIM on AWS F2)](#step-3--rtl-simulation-vivado-xsim-on-aws-f2)
- [Step 4 — FPGA Build and Deployment](#step-4--fpga-build-and-deployment)
- [Opcodes Reference](#opcodes-reference)
- [AXI Address Map](#axi-address-map)

---

## Key Concepts

### The Problem

Matrix transpose is a zero-arithmetic-intensity operation (pure data movement)
that appears as an interstitial operator in Transformer attention blocks when
computing `S = QKᵀ`. On the Lab 4 accelerator, there was no hardware transpose
unit; any transposition would require an off-chip software round-trip. This
project adds a streaming, conflict-free hardware transpose directly inside the
`GBModule`.

### Design Points (four tiers)

| Design | Description | Cycles (16×16) |
|---|---|---|
| Lab 4 (system baseline) | No hardware transpose | off-chip round-trip |
| **Opcode 0** — Naive transpose | 4 FSM states/element, SRAM only | ~3569 adj. cycles |
| **Opcode 1** — Banked-BRAM transpose | 3 states/element, diagonal banking | ~2804 adj. cycles (**1.27×**) |
| **Opcode 2** — Unfused QKᵀ | Opcode-0 transpose + SRAM matmul | ~54368 adj. cycles (16×16×16) |
| **Opcode 3** — Fused QKᵀ | Fill K into BRAM, matmul reads BRAM | ~17122 adj. cycles (**3.18×**) |

### Diagonal BRAM Banking

The optimized transpose uses 32 BRAM banks (depth 32 each, 1024 elements
total). Element `A[r][c]` maps to bank `(r+c) % 32`, flat address
`bank×32 + r`. This guarantees that any row or column access hits distinct
banks, enabling conflict-free single-cycle reads with no runtime arbitration.

### GBCore Interface Bottleneck

The `GBCore` SRAM interface is single-port and serialised: one 128-bit element
(16 × uint8 scalars) per transaction at II=3 cycles. All four opcodes are
bottlenecked by this port. The fused opcode 3 bypasses it for inner-loop KᵀT
reads by reading from BRAM instead.

### Fusion Benefit

Opcode 3 eliminates:
1. The full Kᵀ write-back to SRAM (saves 2ND SRAM writes from opcode 2's naive transpose phase).
2. All M·N·D SRAM reads of Kᵀ during the matmul inner loop, replacing them
   with single-cycle BRAM reads.

Theoretical speedup approaches `4N/(2+N)` — about **3.56×** at N=16.

---

## File Structure

```
CS217_Final_Project/
├── src/                          # SystemC source code
│   ├── include/                  # Shared specs (GBSpec.h, AxiSpec.h, Spec.h, ...)
│   └── Top/
│       ├── Top.h                 # Top-level module
│       ├── testbench.cpp         # SystemC testbench (transpose opcodes 0 & 1)
│       ├── testbench_matmul.cpp  # SystemC testbench (matmul opcodes 2 & 3)
│       ├── axi_commands_test.csv         # AXI stimulus for transpose tests
│       ├── axi_commands_matmul_test.csv  # AXI stimulus for matmul tests
│       ├── GBPartition/
│       │   └── GBModule/
│       │       ├── GBCore/       # Unified SRAM scratchpad (1 MB, 16 banks)
│       │       ├── GBControl/    # PE↔GB data streaming
│       │       ├── NMP/          # RMSNorm / Softmax
│       │       └── Transpose/    # *** New: Transpose module (opcodes 0–3) ***
│       │           └── Transpose.h
│       └── PEPartition/
│           └── PEModule/
│               ├── PECore/       # MAC array (GEMM)
│               └── ActUnit/      # Post-accumulation activation
├── hls/                          # Catapult HLS scripts (mirrors src/ hierarchy)
├── design_top/
│   ├── design/
│   │   ├── design_top.sv         # RTL top wrapper + hardware performance counter
│   │   └── concat_Top.v          # Generated RTL from Catapult HLS
│   ├── verif/tests/
│   │   ├── design_top_base_test.sv    # RTL transpose tests (10 matrix sizes)
│   │   └── design_top_matmul_test.sv  # RTL matmul tests (9 dimension triples)
│   └── software/src/
│       ├── design_top.c           # FPGA runtime test (transpose)
│       └── design_top_matmul.c    # FPGA runtime test (matmul)
├── reports/                       # Generated logs (HLS, SystemC, AWS)
├── test.py                        # Automation script for sim/HLS flows
└── Makefile                       # Top-level build orchestration
```

---

## Environment Setup

```bash
# On the farm machines (required before any make/test.py commands)
source sourceme.sh

# Set required environment variables
export SRC_HOME=$(pwd)/src
export HLS_HOME=$(pwd)/hls
export AWS_HOME=$(pwd)/design_top
```

---

## Step 1 — SystemC Functional Simulation

Runs C++ simulation of the full accelerator hierarchy to verify functional
correctness of the transpose and matmul opcodes before HLS.

### Run all transpose tests (opcodes 0 and 1)

```bash
# Full automation (runs GBCore, GBModule, GBPartition, Top in order)
python3 test.py --action systemc_sim

# Or run the top-level testbench directly
make systemc_sim
# which is equivalent to:
cd src/Top && make
```

### Run matmul tests (opcodes 2 and 3)

```bash
make systemc_matmul
# which is equivalent to:
cd src/Top && make matmul
```

### Run individual module testbenches

```bash
# GBCore SRAM
cd src/Top/GBPartition/GBModule/GBCore && make

# GBModule (Transpose opcodes 0 & 1, 3×2 matrix)
cd src/Top/GBPartition/GBModule && make

# GBPartition
cd src/Top/GBPartition && make
```

Passing output ends with `TESTBENCH PASS`. Results are logged to
`reports/hls/systemc.log.txt`.

---

## Step 2 — HLS Synthesis (Catapult Ultra)

Converts the SystemC design to RTL using a bottom-up synthesis flow.
Requires Catapult Ultra to be available in your environment.

### Automated (recommended, ~2 hours)

```bash
python3 test.py --action rtl_sim
```

This runs HLS for all modules in dependency order and copies the generated
RTL to `design_top/design/concat_Top.v`.

### Manual bottom-up synthesis

```bash
# 1. Leaf modules (can be run in parallel)
cd hls/Top/PEPartition/PEModule/PECore   && make hls
cd hls/Top/PEPartition/PEModule/ActUnit  && make hls
cd hls/Top/GBPartition/GBModule/GBCore   && make hls
cd hls/Top/GBPartition/GBModule/NMP      && make hls
cd hls/Top/GBPartition/GBModule/GBControl && make hls

# 2. Mid-level modules
cd hls/Top/PEPartition/PEModule          && make hls
cd hls/Top/GBPartition/GBModule          && make hls   # includes Transpose

# 3. Partition level
cd hls/Top/PEPartition                   && make hls
cd hls/Top/GBPartition                   && make hls

# 4. Top level
cd hls/Top                               && make hls

# 5. Copy RTL to AWS design folder
make copy_rtl
```

### Verify II=3 target

After synthesis, check `reports/hls/Top.rpt` and confirm that the
`Transpose::Run()` and `GBControl::GBControlRun()` main loops achieve
`Pipeline initiation interval = 3`.

### Debug with waveforms

```bash
make hls_sim_debug
# Opens Verdi with the HLS simulation FSDBa
```

---

## Step 3 — RTL Simulation (Vivado XSIM on AWS F2)

Run cycle-accurate RTL simulation on AWS F2 after completing Step 2.

```bash
# SSH into AWS F2 instance, then:
cd ~/aws-fpga
source hdk_setup.sh
source sdk_setup.sh

cd [path-to-project]/design_top
source setup.sh

# Run transpose tests (opcodes 0 & 1, 10 matrix sizes)
# Uses design_top/verif/tests/design_top_base_test.sv
make hw_sim

# Run matmul tests (opcodes 2 & 3, 9 dimension triples)
# Uses design_top/verif/tests/design_top_matmul_test.sv
make hw_sim TEST=design_top_matmul_test
```

Logs are saved under `design_top/logs/` and copied to
`reports/aws/f2_hw_sim_*.log.txt`.

The hardware performance counter in `design_top/design/design_top.sv`
measures elapsed cycles from the OCL-write "arm" signal to the interrupt
rising edge. Subtract the `1×1` (or `1×1×1`) baseline to get adjusted
cycle counts.

---

## Step 4 — FPGA Build and Deployment

Build the full FPGA bitstream and run the C runtime tests on the programmed
device.

```bash
cd design_top

# Build bitstream (~2.5 hours)
make fpga_build

# Register with AWS and wait for AFI
make generate_afi
make check_afi_available   # poll until Status: available

# Program and test
make program_fpga

# Transpose runtime test (design_top/software/src/design_top.c)
make run_fpga_test

# Matmul runtime test (design_top/software/src/design_top_matmul.c)
make run_fpga_test_matmul
```

---

## Opcodes Reference

All four opcodes are configured via a single 128-bit register at AXI region
`0x6`, local index `0x01`. The Transpose module is started by writing to
AXI address `0x0 / 0x3`.

| Bits | Field | Description |
|---|---|---|
| 0 | `is_valid` | Enable bit |
| 10:8 | `memory_index_src` | K / source matrix location |
| 18:16 | `memory_index_dst` | Result / destination location |
| 26:24 | `memory_index_q` | Q location (opcodes 2, 3) |
| 39:32 | `num_rows` | N (rows of K) |
| 47:40 | `num_cols` | D (cols of K = cols of Q) |
| 50:48 | `opcode` | 0 = naive transpose, 1 = BRAM transpose, 2 = unfused QKᵀ, 3 = fused QKᵀ |
| 63:56 | `num_rows_q` | M (rows of Q, opcodes 2 and 3 only) |

---

## AXI Address Map

| Region (bits [23:20]) | Module | Notes |
|---|---|---|
| `0x3` | GBCore | SRAM configuration register |
| `0x4` | GBCore | Address config (base, num_vector per region) |
| `0x5` | GBCore | Direct SRAM read/write |
| `0x6` | Transpose | Configuration register (128-bit, index `0x01`) |
| `0x7` | GBControl | Streaming config |
| `0xC` | NMP | RMSNorm / Softmax config |
| `0x0` | Start trigger | `local_index=0x1` → GBControl, `0x2` → NMP, `0x3` → Transpose |

---

## Clean

```bash
# Clean all generated simulation, HLS, and FPGA build artifacts
python3 test.py --action clean
# or equivalently:
make clean
```
