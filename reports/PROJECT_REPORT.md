\documentclass{article}
\usepackage{graphicx} % Required for inserting images

\title{CS217 Project Checkpoint}
\author{Chia-Hsiang Chang}
\date{February 2026}

\begin{document}

\maketitle

# CS217 Final Project Progress Checkpoint

Yiming Tan (yimingt@stanford.edu)
Chia-Hsiang Chang (hsiangc@stanford.edu)

---

## 1. Overview

The **Transpose** module performs in-place matrix transpose on data stored in GBCore SRAM. Two execution modes are supported, selected by an **opcode** in the AXI configuration register:

- **Opcode 0 (naive):** Original element-by-element method.
- **Opcode 1 (diagonal):** Efficient method that processes by anti-diagonals using local BRAM for better memory locality.

Configuration is packed into a 128-bit AXI word at region `0x6`, local_index `0x01`, including `is_valid`, `memory_index_src`, `memory_index_dst`, `num_rows`, `num_cols`, and **opcode** (bits 50:48).

---

## 2. Original Method (Naive — Opcode 0)

### 2.1 Algorithm

- Traverse the source matrix in **column-major** order (inner loop over columns, outer over rows) using `row_counter` and `col_counter` in `TransposeConfig`.
- For each element:
  1. **READ:** Request `A[row_counter][col_counter]` from the source memory manager (SRAM).
  2. **WRITE:** Write the same value to `A^T[col_counter][row_counter]` in the destination memory manager.

### 2.2 Addressing

- **Source:** `(memory_index_src, timestep_index = row, vector_index = col)` → element `A[row][col]`.
- **Destination:** `(memory_index_dst, timestep_index = col, vector_index = row)` → element `A^T[col][row]`.

### 2.3 FSM States

`IDLE` → `READ` → `WAIT_RSP` → `WRITE` → `NEXT` → (if more elements) `READ`, else `FIN` → `IDLE`.

- **Advance()** updates `row_counter`/`col_counter` and returns true when all elements have been processed.

---

## 3. Diagonal Method (Efficient — Opcode 1)

### 3.1 Idea

- Process the matrix by **anti-diagonals**: indices `(i, j)` with `i + j = d` for `d = 0, 1, … , (R + C - 1)`.
- For each anti-diagonal:
  1. Read all elements on that diagonal from SRAM into a **local BRAM** (`diag_bram`, max length 32).
  2. Write from BRAM to the destination in transpose order.
- This improves locality by batching reads/writes along diagonals instead of scattering reads and writes across the matrix.

### 3.2 Diagonal Bounds

- **DiagBounds(d, i_lo, len):** For anti-diagonal index `d` and matrix dimensions `R×C`, computes the valid row range and length:
  - Row index `i` runs from `max(0, d - C + 1)` to `min(d, R - 1)`.
  - `i_lo` is the first row index; `len` is the number of elements (capped by `kMaxDiagLen = 32`).

### 3.3 State and Storage

- **BRAM:** `spec::GB::Large::WordType diag_bram[kMaxDiagLen]`.
- **Counters:** `diag_d` (current anti-diagonal), `diag_i_lo`, `diag_len`, `diag_in_idx` (fill BRAM), `diag_out_idx` (drain BRAM).

### 3.4 FSM (Diagonal Path)

- `IDLE` → (opcode 1) `DIAG_START` → `DIAG_READ` → `DIAG_READ_WAIT` → (when BRAM full) `DIAG_WRITE` → (when BRAM drained) `DIAG_NEXT` → (if `diag_d < R+C-1`) `DIAG_START`, else `FIN` → `IDLE`.
- **DIAG_READ:** Issue read for `A[i][d-i]` with `i = diag_i_lo + diag_in_idx`.
- **DIAG_READ_WAIT:** Store response in `diag_bram[diag_in_idx]`, increment `diag_in_idx`.
- **DIAG_WRITE:** Write `diag_bram[diag_out_idx]` to `A^T[col][row]` with `row = diag_i_lo + diag_out_idx`, `col = d - row`; increment `diag_out_idx`.

---

## 4. Test Design and Coverage

### 4.1 GBModule Testbench (`src/Top/GBPartition/GBModule/testbench.cpp`)

- **Purpose:** Verify the Transpose module through the full GBModule (GBCore SRAM, config, start/done).
- **Matrix:** 3×2 source matrix; expected 2×3 transpose.
  - Source: `A[0][0]=0x01`, `A[0][1]=0x02`, `A[1][0]=0x03`, `A[1][1]=0x04`, `A[2][0]=0x05`, `A[2][1]=0x06`.
  - Expected `A^T`: row0 `(0x01, 0x03, 0x05)`, row1 `(0x02, 0x04, 0x06)`.
- **Memory setup:**
  - GBCore config: three managers — src (base 0, num_vector 2), dst for naive (base 64, num_vector 3), dst for diagonal (base 128, num_vector 3).
  - Source matrix written via direct AXI writes to SRAM (region 0x5).
- **Test sequence:**
  1. Write GBCore memory config.
  2. Write source matrix to SRAM.
  3. **Test 1 (naive):** Write Transpose config (opcode 0, dst = manager 1), assert start, wait for `gb_done`, read back 6 locations from dst base 64, compare to expected.
  4. **Test 2 (diagonal):** Reuse source; write Transpose config (opcode 1, dst = manager 2), start, wait for `gb_done`, read back 6 locations from dst base 128, same expected values.
- **Coverage:** Both opcodes, config write, start/done handshake, full read-back verification (12 reads total: 6 per opcode). Pass/fail via `SC_REPORT_ERROR` on mismatch and `sc_stop()` after all checks.

### 4.2 Top-Level Testbench (`src/Top/testbench.cpp`)

- **Purpose:** Top-level integration using AXI commands from a CSV file.
- **Stimulus:** `ManagerFromFile` reads `./axi_commands_test.csv` and issues AXI writes/reads.
- **Flow:** CSV programs GBCore config, source matrix, Transpose config for **opcode 0** (naive), start, then read-back; then Transpose config for **opcode 1** (diagonal), start, read-back. Dest checks **interrupt**; testbench waits for `master_done` and one interrupt, then ends.
- **Pass criteria:** No `SC_ERROR` reports; simulation does not time out (5 s limit).

### 4.3 AXI Commands CSV (`src/Top/axi_commands_test.csv`)

- Encodes the same logical sequence as the GBModule test: GBCore config, source data (3×2), Transpose config at `0x33600010` (opcode in bits 50:48):
  - First run: `0x000000000000000000020300010001` → naive (opcode 0), dst 1.
  - Second run: `0x00000000000000000001020300020001` → diagonal (opcode 1), dst 2.
- Read-back addresses target the two destination regions (base 64 and base 128) to verify both transposes.

### 4.4 Test Automation (`test.py`)

- **SystemC simulation:** Runs `make` in:
  - `src/Top/GBPartition/GBModule/GBCore`
  - `src/Top/GBPartition/GBModule` (naive + diagonal test)
  - `src/Top/GBPartition`
  - `src/Top`
- Logs stdout/stderr to `reports/hls/systemc.log.txt`; pass/fail determined by presence of "TESTBENCH PASS" in stdout.
- **RTL simulation:** Same hierarchy under `hls/Top/...`; logs to `reports/hls/rtl_sim.log.txt`.

### 4.5 Coverage Summary

| Item | Coverage |
|------|----------|
| Transpose opcode 0 (naive) | GBModule testbench + Top CSV test |
| Transpose opcode 1 (diagonal) | GBModule testbench + Top CSV test |
| Config register (opcode, src/dst, rows/cols) | Both testbenches |
| Start/done handshake | GBModule (gb_done), Top (interrupt) |
| Read-back verification | 6 elements per opcode, same expected values for both |
| GBCore SRAM addressing | Direct AXI writes and config-driven accesses |
| Matrix size | 3×2 (and 2×3 transpose) in both tests |


\end{document}
