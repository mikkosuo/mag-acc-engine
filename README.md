# mag_acc_engine

Magnitude-accumulate engine for non-coherent integration on A-Core. One job processes `LEN` words:

    X2[i] := (MODE ? X2[i] : 0) + (cordic_mag(X1[i]) >> SHIFT)        (mod 2^32)

`X1[i]` packs I in bits [15:0] and Q in bits [31:16] (int16 each). `X2[i]` is a uint32 accumulator. Run one job per PRI: `MODE = 0` (overwrite) on the first pulse of a CPI replaces the accumulator-clear pass, and `MODE = 1` (accumulate) is used for the rest.

## Files

| Path | Contents |
|---|---|
| `rtl/mag_acc_engine.v`, `rtl/mae_fifo.v` | Synthesizable Verilog-2005: AXI4-Lite configuration slave, two AXI4-Lite masters, and a pipelined CORDIC |
| `chisel/MagAccEngine.scala` | `BlackBox` with `HasBlackBoxResource`, plus a `Module` wrapper with plain AXI4-Lite bundles |
| `model/mag_acc_model.py` | Bit-accurate Python reference (scalar and NumPy), plus a datapath-width proof |
| `tb/` | cocotb testbench (`test_mag_acc_engine.py`, `axil.py`), pytest runner (`test_runner.py`), model checks (`test_model.py`) |
| `sw/mag_acc.h` | Bare-metal C driver, checked with `-Wall -Wextra -Wpedantic -Werror` for host and `rv32im` |

## Register map (32-bit, byte offsets)

| Offset | Name | Bits |
|---|---|---|
| 0x00 | CTRL | [0] START (self-clearing; ignored while BUSY), [1] MODE (0 overwrite, 1 accumulate), [2] IRQ_EN |
| 0x04 | STATUS | [0] BUSY (RO), [1] DONE (W1C), [2] ERR (W1C, set by any non-OKAY master response). DONE and ERR are also cleared when a job starts |
| 0x08 | SRC | X1 base address. Bits [1:0] are ignored |
| 0x0C | DST | X2 base address. Bits [1:0] are ignored |
| 0x10 | LEN | Number of elements. LEN = 0 completes immediately |
| 0x14 | SHIFT | [4:0] right shift applied to the magnitude |
| 0x18 | INFO | RO: [7:0] N_ITER, [15:8] GUARD, [31:16] version |
| 0x1C | CYCLES | RO: cycles the last job was BUSY |

`SRC`, `DST`, `LEN`, `SHIFT` and `MODE` are latched at START. Writing them while the engine is busy only affects the next job. The X1 and X2 ranges must not overlap. `irq = DONE & IRQ_EN` (level).

## Interfaces and throughput

There is one clock and an active-high synchronous reset (the Chisel convention). The engine has three ports:

- **`s_axil_*`** is the configuration slave.
- **`m_src_*`** is a read-only master that fetches X1, meant for the buffer bank (`dm_blocks_0`).
- **`m_dst_*`** is a read/write master. It fetches X2 (MODE = 1 only) and writes it back, and is meant for the accumulator bank (`dm_blocks_1`).

Each master keeps up to `FIFO_DEPTH` reads in flight and requires in-order responses, which AXI4-Lite guarantees.

A credit scheme bounds every FIFO, so `rready` and `bready` are tied high. The testbench checks that no FIFO is ever pushed while full, and drives each FIFO exactly to its depth.

| Memory | FIFO_DEPTH | Cycles / element (LEN = 1024) |
|---|---|---|
| Single-cycle on both ports | 32 | 1.018 (1042 cycles; the extra 18 are fill latency) |
| Single-cycle on both ports | 5 | 4.01 |
| Single-cycle on both ports | 1 | 12.0 |

Full rate needs `FIFO_DEPTH` to cover the round trip: read latency + N_ITER + 4 + write acceptance. If A-Core gives the accelerator only one master port, merge `m_src` and `m_dst` with a 2:1 AXI4-Lite arbiter (not included). Per element, that port then sees two reads and one write, so throughput falls to about one element per 2 cycles at best (reads and writes use separate AXI channels).

## CORDIC numerics

The CORDIC runs in vectoring mode. It first folds the input to x₀ = |I|, y₀ = Q, then iterates N_ITER times:

    d = sgn(y),  x ← x + d·(y >> k),  y ← y − d·(x >> k)

The datapath is W = 19 + GUARD bits wide, signed, and the result is rounded half-up when the guard bits are dropped. The output is ≈ K_N·|z| with K_N = 1.646760. The gain is not compensated, because CA-CFAR's test `z_cut > T·mean` is homogeneous of degree one and K cancels.

The defaults are N_ITER = 11 and GUARD = 6. Against K_N·|z| (10⁶ random points plus a full-scale ring), the maximum error is 0.59 LSB, the RMS error is 0.288 LSB (the uniform-rounding floor is 0.289), and the bias is +0.005 LSB. Running more iterations without more guard bits makes the result worse, because truncation error builds up: N = 16, G = 4 gives 1.06 LSB maximum. `model/mag_acc_model.py:check_widths` proves that no intermediate value overflows W.

The largest output is 76312 (I = Q = −32768), which is below 2¹⁷. With `SHIFT = 0` a uint32 accumulator therefore cannot overflow for M ≤ 56281 pulses.

## Consequences for the detection chain

- **Detector law.** The detector becomes linear-law: summing M Rayleigh variables is not Erlang. `find_T_q10` has to be re-derived for this statistic, which has no closed form; use the characteristic function numerically or Monte-Carlo with the bit-accurate model. The classic linear-vs-square-law detectability difference is small (≤ ~0.2 dB).
- **Software that stays.** Slow-time slice collection stays in software.
- **Reference model.** Add `cordic_mag_np` / `mag_accumulate_np` to `bit_accurate_functions` so the C and Python chains stay bit-exact.

## Software usage (one CPI)

```c
#define MAE_BASE 0x...           /* from the SoC address map */
#include "mag_acc.h"

for (uint32_t m = 0; m < M; m++) {
    const uint32_t *buf = wait_for_dma_buffer(m);      /* ping-pong IQ buffer */
    mae_start(buf, acc, N, 0, m > 0);                  /* overwrite on m == 0 */
    collect_slow_time_slices(buf, m);                  /* overlaps the engine */
    if (mae_wait() & MAE_ST_ERR) handle_bus_error();
}
```

`mae_wait` clears DONE, so a stale DONE can never satisfy the next wait. Give the engine to one core only, because no atomics are available.

## Verification

```
make lint            # verilator -Wall: clean
make model           # 9 reference-model checks
make test            # cocotb: 13 tests x {Icarus, Verilator} x 3 parameter sets
```

The parameter sets are `default` (11/6/32), `minimal` (8 iterations, GUARD = 0, FIFO_DEPTH = 1) and `odd` (16/4/5, a non-power-of-two FIFO). All 78 test runs pass on Icarus Verilog 12.0 and Verilator 5.48. Every data check compares against the bit-accurate model word for word. The tests also check:

- X1 and X2 are each read exactly once and in order, and every write lands exactly once and in order.
- The fence words around the destination range are untouched.
- AXI VALID and payload stay stable until the handshake, on all master channels.
- DONE never rises while responses are outstanding.

The tests cover:

- Register access, including byte strobes and AW/W skew.
- Random jobs in both modes.
- All 81 corner I/Q pairs × shifts {0, 1, 16, 17, 31}, with X2 values that force a uint32 wrap.
- An M = 16 CPI over ping-pong buffers.
- Random backpressure, latency and outstanding limits.
- A hard write stall that fills every FIFO to exactly its depth.
- LEN = 0 and LEN = 1.
- SLVERR on a read and on a late final write response.
- The IRQ line.
- START and register writes while busy.
- Unaligned base addresses.
- Throughput.

Mutation check: 12 hand-made RTL bugs were injected one at a time, and the suite catches every one. The rounding bug is caught by the `default` set; it is undetectable under `minimal` only because it is a no-op when GUARD = 0. The bugs were:

- a CORDIC sign error
- off-by-one in the source and result credits
- rounding dropped
- a wrong absolute value of −32768
- MODE ignored
- SHIFT ignored
- ARVALID dropped before its handshake
- a duplicate AW
- wrong write data
- DONE before the last B
- W1C clearing both status bits

cocotb 2.x needs Verilator ≥ 5.036, and Ubuntu 24.04's 5.020 fails to build the cocotb harness. Either build Verilator from source or use the PyPI wheel (`pip install verilator`, then put its `bin/` on PATH with `VERILATOR_ROOT` set), which is what was used here.

The Chisel wrapper was parse-checked only, because no Chisel/Maven access was available when this was written. A script confirmed that its 49 ports match the Verilog name for name, by direction and by width. Elaborate it in the A-Core tree. The emitted `mag_acc_engine.v` is the file the cocotb suite tests.
