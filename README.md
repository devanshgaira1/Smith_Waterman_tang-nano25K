# Smith_Waterson_tang-nano25K
# Smith-Waterman on Sipeed Tang 25K FPGA

Implementing the Smith-Waterman local sequence alignment algorithm in Verilog on a Sipeed Tang 25K (Gowin GW5A-25) FPGA.

I originally came across [Carmine Pacilio's C++ HLS implementation](https://github.com/CarmineD8/Smith-Waterman-HLS) and used that as the starting point. 

---

## What is Smith-Waterman?

Smith-Waterman is a dynamic programming algorithm for **local sequence alignment** — finding the most similar region between two DNA/protein sequences, allowing for mismatches and gaps. It fills an (n × m) scoring matrix using this recurrence:

```
D[i][j] = max(
    0,
    D[i-1][j-1] + score(a[i], b[j]),   -- match/mismatch
    P[i][j],                             -- gap in sequence A
    Q[i][j]                              -- gap in sequence B
)

P[i][j] = max(D[i-1][j] + GAP_OPEN, P[i-1][j] + GAP_EXT)  -- affine gap
Q[i][j] = max(D[i][j-1] + GAP_OPEN, Q[i][j-1] + GAP_EXT)
```

For 1000×1000 sequences this is **1 million cell updates**, which is what makes it a good candidate for hardware acceleration.

---

## Architecture — FSM-Driven Scalar Wavefront

The design uses an **anti-diagonal wavefront** traversal. Cells on the same anti-diagonal have no data dependencies on each other, which is the key property that makes parallelism possible. However, this implementation is **scalar** — it computes one cell at a time, sequentially, cycling through:

```
IDLE → INIT → NEWDIAG → PREFETCH → LOAD → COMPUTE → ENDDIAG → FINISH
```

Each cell takes **3 clock cycles** (PREFETCH → LOAD → COMPUTE). At 50 MHz, this gives a theoretical maximum of ~16M cells/sec.

Three rotating BRAM banks store the P, Q, and D values for the previous two diagonals. This keeps memory usage at **O(n)** instead of the O(n²) that storing the full matrix would require.

```
Diagonal d:
  ┌─────────────────────────────────┐
  │  DB[0]  DB[1]  DB[2] ... DB[k]  │  ← database sequence (forward)
  │  T[k]   T[k-1] T[k-2] ... T[0]  │  ← target sequence (reverse indexed)
  └─────────────────────────────────┘
  Each cell reads from:
    - P/Da from diagonal d-1  (one_base offset)
    - Q/Db from diagonal d-2  (two_base offset)
  Writes result to current diagonal bank (cur_base offset)
```

### Scoring Parameters
```
MATCH     =  2
MISMATCH  = -1
GAP_OPEN  = -4   (total penalty for opening a gap)
GAP_EXT   = -1   (per-base extension penalty)
```

---

## How the Code Evolved

### Stage 1 — First Working Run (784ms)
Got the anti-diagonal FSM running correctly on short sequences. Verified against the original C++ golden reference. First  run came in at **784ms**.

The bottleneck was immediately obvious: UART at 115200 baud was spending more time transferring the sequences than computing the alignment.

### Stage 2 — UART Upgrade (591ms)
Bumped UART baud rate from 115200 to **921600**. Dropped runtime to **591ms** with zero architectural changes. Just removing the data transfer bottleneck.

This also required upgrading to official Gowin UART modules (uart_rx.v / uart_tx.v) with proper CLK_FRE/BAUD_RATE parameterisation.Both these stages involved short character sequences
### Stage 3 — Bug Hunting
Two critical bugs were found and fixed:

**Bug 1 — Diagonal indexing (NEWDIAG state):** The database and target cursors were being computed incrementally with a complex state machine that got wrong coordinates in the shrinking phase. Fixed by computing them directly from `num_diag` each iteration using combinational wires:
```verilog
next_db = max(0, num_diag - lenT + 1)
next_tc = num_diag - next_db
next_je = min(num_diag, lenD - 1)
next_dl = next_je - next_db + 1
```

**Bug 2 — t_idx reversal:** The target index formula was reading the sequence backwards:
```verilog
// Wrong — reversed the target
wire [10:0] t_idx = lenT - 11'd1 - (target_cursor[10:0] - j_cnt[10:0]);

// Correct — forward indexing
wire [10:0] t_idx = target_cursor[10:0] - j_cnt[10:0];
```

### Stage 4 — Benchmark Against Real Baselines
Compared against three CPU implementations. Results below.

---

## Benchmark Results

### Short sequences (8–21 bp), 10 pairs — 10/10 correct ✓

![Short sequence benchmark](https://github.com/devanshgaira1/Smith_Waterman_tang-nano25K/blob/main/Screenshot%202026-03-13%20175544.png)
![Short sequence benchmark](https://github.com/devanshgaira1/Smith_Waterman_tang-nano25K/blob/main/Screenshot%202026-03-13%20175618.png)
The slower case has a Lower baud rate
All 10 pairs score correctly. The FPGA is slower here due to UART overhead dominating compute time for such small sequences.

### 1000×1000 bp identical sequence — 1/1 correct ✓

![1000bp benchmark](https://github.com/devanshgaira1/Smith_Waterman_tang-nano25K/blob/main/Screenshot%202026-03-13%20175603.png)

```
CPU (Python)     : 1.145s   0.000875 GCUPS
FPGA (Tang 25K)  : 0.726s   0.001380 GCUPS
Speedup          : 1.6×
```

The FPGA is 1.6× faster than unoptimised Python for 1000bp sequences.

### Against optimised CPU baselines


```
CPU (Pure Python)    : 0.000801 GCUPS
CPU (BioPython)      : 0.115619 GCUPS   — 140× faster than Python
CPU (Parasail AVX2)  : 0.492335 GCUPS   — 615× faster than Python
FPGA (Tang 25K)      : 0.001380 GCUPS   — 1.6× faster than Python
```

The FPGA loses badly to BioPython and Parasail. This is expected. Parasail uses AVX2 SIMD which processes **32 cells per CPU instruction** at 3+ GHz. A scalar FSM at 50 MHz computing one cell per 3 cycles was never going to compete. Comparing only against unoptimised Python was pointed out as an unfair baseline — and it was.

---

## Why the FPGA Lost (and What Comes Next)

The current design is the simplest possible correct hardware implementation. The path to competitive performance is a **systolic array** — instead of one FSM looping over cells, you instantiate N parallel processing elements (PEs), each hardwired to its neighbour. Data flows through like a conveyor belt, processing an entire diagonal in a single clock cycle.

```
Current:  1 cell / 3 cycles → ~16M cells/sec at 50MHz
Systolic: 64 PEs / 1 cycle  → ~3.2B cells/sec at 50MHz (theoretical)
```

The Tang 25K has ~20,000 LUTs. A single SW cell uses ~50 LUTs, so roughly 300–400 parallel PEs would fit. That's the next thing to build.

---

## Files

| File | Description |
|------|-------------|
| `sw_core.v` | Anti-diagonal wavefront engine — the compute heart |
| `sw_top.v` | Top-level with UART FSM for Tang 25K |
| `uart_rx.v` | Official Gowin UART RX (CLK_FRE=50, BAUD=921600) |
| `uart_tx.v` | Official Gowin UART TX (CLK_FRE=50, BAUD=921600) |
| `tang25k.cst` | Physical constraints — pin assignments |
| `tang25k.sdc` | Timing constraint — 50 MHz clock |
| `benchmark.py` | Python benchmark script (runs CPU + FPGA, prints comparison) |
| `sequences.txt` | Test sequences — 10 short pairs + 1000bp TP53 pair |

---

## Pin Assignments (Tang 25K)

| Signal | Pin | Notes |
|--------|-----|-------|
| `clk` | E2 | 50 MHz onboard clock |
| `rst_n` | H8 | Active-low reset button |
| `uart_rx` | C3 | CH340 TXD → FPGA |
| `uart_tx` | B3 | CH340 RXD ← FPGA |

> **Note:** TX and RX are from the FPGA's perspective, not the CH340's. Getting these swapped causes the FPGA to echo 0xFA before every byte — a very specific and confusing symptom that took a while to track down.

---

## Running the Benchmark

```bash
pip install pyserial biopython parasail

python benchmark.py
# Select your COM port when prompted
# Flash bitstream in Gowin Programmer
# Type 'go' when ready
```

---

## Synthesising in Gowin IDE

1. New project → Device: **GW5A-LV25MG121NC1/I0**
2. Add all `.v` files, set top module to `sw_top`
3. Add `tang25k.cst` as physical constraints
4. Add `tang25k.sdc` via Project → Settings → Timing Constraints
5. Synthesise → Place & Route → Generate Bitstream → Program

---

## Credits

- **[Carmine Pacilio](https://github.com/CarmineD8)** — original C++ HLS implementation this was ported from
- **Pratham Jha** — explained the biological meaning of the scoring parameters
- **[Sipeed](https://sipeed.com)** — for making the Tang 25K

---

## What's Next

- Systolic array PE architecture
- Proper comparison against Parasail with parallel hardware

