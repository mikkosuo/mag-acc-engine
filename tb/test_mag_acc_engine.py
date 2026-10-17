"""cocotb tests for rtl/mag_acc_engine.v against model/mag_acc_model.py.

Every data check is exact: the RTL must reproduce the bit-accurate model word for word.
"""

from __future__ import annotations

import itertools
import os
import random
import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from mag_acc_model import mag_accumulate, pack_iq  # noqa: E402

from axil import AxilMaster, AxilMemSlave, SlaveTiming, SparseMemory  # noqa: E402

# register map
CTRL, STATUS, SRC, DST, LEN, SHIFT, INFO, CYCLES = range(0, 0x20, 4)
START, MODE_ACC, IRQ_EN = 1, 2, 4
BUSY, DONE, ERR = 1, 2, 4

FIFO_DEPTH = int(os.environ.get("MAE_FIFO_DEPTH", "32"))
N_ITER = int(os.environ.get("MAE_N_ITER", "11"))
GUARD = int(os.environ.get("MAE_GUARD", "6"))

BANK0 = 0x2000_0000          # X1 buffers (mirrors the A-Core dm_blocks_0 origin)
BANK1 = 0x2002_0000          # X2 accumulators (dm_blocks_1 origin)


class Env:
    def __init__(self, dut, src_t: SlaveTiming | None = None, dst_t: SlaveTiming | None = None,
                 seed: int = 1):
        self.dut = dut
        self.rng = random.Random(seed)
        self.mem = SparseMemory()
        self.cfg = AxilMaster(dut, dut.clock)
        self.src = AxilMemSlave(dut, dut.clock, "m_src", self.mem, src_t or SlaveTiming(),
                                has_write=False, rng=random.Random(seed + 1))
        self.dst = AxilMemSlave(dut, dut.clock, "m_dst", self.mem, dst_t or SlaveTiming(),
                                has_write=True, rng=random.Random(seed + 2))
        self.fifo_errors: list[str] = []
        self.fifo_max = [0, 0, 0]                 # peak occupancy of src, dst, res FIFOs

    async def start(self):
        cocotb.start_soon(Clock(self.dut.clock, 10, unit="ns").start())
        self.dut.reset.value = 1
        await ClockCycles(self.dut.clock, 3)
        await RisingEdge(self.dut.clock)
        self.dut.reset.value = 0
        cocotb.start_soon(self.src.run())
        cocotb.start_soon(self.dst.run())
        cocotb.start_soon(self._fifo_monitor())
        await ClockCycles(self.dut.clock, 2)

    async def _fifo_monitor(self):
        """No FIFO may ever be asked to take a word it cannot hold (credit scheme proof)."""
        fifos = [self.dut.u_src_fifo, self.dut.u_dst_fifo, self.dut.u_res_fifo]
        while True:
            await FallingEdge(self.dut.clock)
            await ReadOnly()
            for n, f in enumerate(fifos):
                cnt = int(f.count.value)
                self.fifo_max[n] = max(self.fifo_max[n], cnt)
                if cnt > FIFO_DEPTH:
                    self.fifo_errors.append(f"count {cnt} > depth")
                if int(f.push.value) and cnt == FIFO_DEPTH and not (int(f.pop.value) and cnt > 0):
                    self.fifo_errors.append("push into full FIFO")

    async def run_job(self, src, dst, n, shift, accumulate, irq_en=False, timeout=None):
        await self.cfg.write(SRC, src)
        await self.cfg.write(DST, dst)
        await self.cfg.write(LEN, n)
        await self.cfg.write(SHIFT, shift)
        await self.cfg.write(CTRL, START | (MODE_ACC if accumulate else 0) | (IRQ_EN if irq_en else 0))
        return await self.wait_done(timeout or (200 + 60 * n))

    async def wait_done(self, max_polls):
        for _ in range(max_polls):
            st = await self.cfg.read(STATUS)
            if st & DONE and not st & BUSY:
                # DONE may only rise after every read and write response has been taken
                assert self.src.pending == 0 and self.dst.pending == 0, \
                    f"DONE with responses outstanding (src {self.src.pending}, dst {self.dst.pending})"
                return st
        raise AssertionError("job did not finish")

    def check_protocol(self):
        errs = self.src.errors + self.dst.errors + self.fifo_errors
        assert not errs, "\n".join(errs[:10])


def rand_words(rng, n):
    return [rng.getrandbits(32) for _ in range(n)]


async def check_job(env: Env, n, shift, accumulate, x1=None, x2=None, src=BANK0, dst=BANK1):
    rng = env.rng
    x1 = x1 if x1 is not None else rand_words(rng, n)
    x2 = x2 if x2 is not None else rand_words(rng, n)
    env.mem.load(src, x1)
    env.mem.load(dst - 4, [0xDEADBEEF] + list(x2) + [0xCAFEF00D])     # fence words
    env.dst.write_log.clear()
    env.dst.read_log.clear()
    env.src.read_log.clear()
    await env.run_job(src, dst, n, shift, accumulate)
    exp = mag_accumulate(x1, x2, shift, accumulate, N_ITER, GUARD)
    got = env.mem.dump(dst, n)
    bad = [i for i in range(n) if got[i] != exp[i]]
    assert not bad, (f"{len(bad)} mismatches, first at {bad[0]}: x1=0x{x1[bad[0]]:08x} "
                     f"x2=0x{x2[bad[0]]:08x} got=0x{got[bad[0]]:08x} exp=0x{exp[bad[0]]:08x}")
    assert env.mem.read(dst - 4) == 0xDEADBEEF and env.mem.read(dst + 4 * n) == 0xCAFEF00D, \
        "write outside the destination range"
    assert env.dst.write_log == [dst + 4 * i for i in range(n)], "writes not exactly once, in order"
    assert env.src.read_log == [src + 4 * i for i in range(n)], "X1 not read exactly once, in order"
    exp_dst_reads = [dst + 4 * i for i in range(n)] if accumulate else []
    assert env.dst.read_log == exp_dst_reads, "X2 reads wrong (must be once each, in order, MODE=1 only)"
    env.check_protocol()
    return got


# ============================================================================= tests
@cocotb.test()
async def test_registers(dut):
    env = Env(dut)
    await env.start()
    c = env.cfg
    assert await c.read(CTRL) == 0
    assert await c.read(STATUS) == 0
    info = await c.read(INFO)
    assert info & 0xFF == N_ITER and (info >> 8) & 0xFF == GUARD and info >> 16 == 1, hex(info)
    for reg in (SRC, DST, LEN):
        for v in (0xFFFFFFFF, 0x12345678, 0):
            await c.write(reg, v)
            assert await c.read(reg) == v
    await c.write(SRC, 0xA1A2A3A4)
    await c.write(SRC, 0xB0C0D0E0, strb=0b1010)                 # bytes 1 and 3 only
    assert await c.read(SRC) == 0xB0A2D0A4
    await c.write(SHIFT, 0xFFFFFFFF)
    assert await c.read(SHIFT) == 31
    await c.write(CTRL, MODE_ACC | IRQ_EN)                       # no START
    assert await c.read(CTRL) == MODE_ACC | IRQ_EN
    assert await c.read(STATUS) == 0
    assert await c.read(0x40) == 0                              # unmapped reads as zero
    # AW and W skewed in both directions
    await c.write(DST, 0x11111111, aw_delay=3)
    assert await c.read(DST) == 0x11111111
    await c.write(DST, 0x22222222, w_delay=4, b_delay=2)
    assert await c.read(DST, r_delay=3) == 0x22222222


@cocotb.test()
async def test_overwrite_random(dut):
    env = Env(dut, seed=11)
    await env.start()
    for shift in (0, 3, 9):
        await check_job(env, 300, shift, accumulate=False)


@cocotb.test()
async def test_accumulate_random(dut):
    env = Env(dut, seed=12)
    await env.start()
    for shift in (0, 1, 5):
        await check_job(env, 300, shift, accumulate=True)


@cocotb.test()
async def test_edge_vectors(dut):
    """Every pairing of extreme I/Q values, every shift that matters, and X2 values that wrap."""
    env = Env(dut, seed=13)
    await env.start()
    corners = [-32768, -32767, -16384, -1, 0, 1, 16384, 32766, 32767]
    x1 = [pack_iq(i, q) for i, q in itertools.product(corners, corners)]
    n = len(x1)
    for shift in (0, 1, 16, 17, 31):
        x2 = [0xFFFFFFFF - env.rng.randrange(0, 1 << 17) for _ in range(n)]   # forces uint32 wrap
        await check_job(env, n, shift, accumulate=True, x1=x1, x2=x2)
    await check_job(env, n, 0, accumulate=False, x1=x1, x2=[0] * n)


@cocotb.test()
async def test_cpi_integration(dut):
    """The deployed use: M pulses, overwrite on the first, accumulate on the rest."""
    env = Env(dut, seed=14)
    await env.start()
    M, N, shift = 16, 257, 0
    acc = [0] * N
    for m in range(M):
        x1 = rand_words(env.rng, N)
        src = BANK0 + (m % 2) * 0x8000                      # ping-pong buffers
        env.mem.load(src, x1)
        await env.run_job(src, BANK1, N, shift, accumulate=(m > 0))
        acc = mag_accumulate(x1, acc, shift, m > 0, N_ITER, GUARD)
        assert env.mem.dump(BANK1, N) == acc, f"accumulator mismatch after pulse {m}"
    env.check_protocol()


@cocotb.test()
async def test_backpressure(dut):
    """Random ready, random latency, limited outstanding on both ports, both modes."""
    env = Env(dut, seed=100)
    await env.start()
    for p, lat, mo in [(0.5, (0, 4), 3), (0.25, (0, 12), 1 << 30), (0.8, (2, 2), 1)]:
        env.src.t = SlaveTiming(p_arready=p, r_lat=lat, max_outstanding=mo)
        env.dst.t = SlaveTiming(p_arready=p, p_awready=p, p_wready=1 - p / 2,
                                r_lat=lat, b_lat=(lat[0], 3 * lat[1]), max_outstanding=mo)
        await check_job(env, 200, env.rng.randrange(8), accumulate=True)
        await check_job(env, 200, env.rng.randrange(8), accumulate=False)
        await check_job(env, 1, 0, accumulate=True)


@cocotb.test()
async def test_len_zero_and_one(dut):
    env = Env(dut, seed=15)
    await env.start()
    env.mem.load(BANK1, [0x5A5A5A5A])
    st = await env.run_job(BANK0, BANK1, 0, 0, accumulate=True)
    assert st == DONE
    assert env.dst.write_log == [] and env.src.read_log == []
    assert env.mem.read(BANK1) == 0x5A5A5A5A
    assert await env.cfg.read(CYCLES) == 0
    await check_job(env, 1, 0, accumulate=True)
    await check_job(env, 1, 2, accumulate=False)


@cocotb.test()
async def test_error_response(dut):
    """SLVERR on one X1 read and one write: ERR is set, the job still completes."""
    n = 32
    src_t = SlaveTiming(err_read_addrs={BANK0 + 4 * 7})
    dst_t = SlaveTiming(err_write_addrs={BANK1 + 4 * (n - 1)}, b_lat=(0, 30))   # late error on the last B
    env = Env(dut, src_t, dst_t, seed=16)
    await env.start()
    env.mem.load(BANK0, rand_words(env.rng, n))
    st = await env.run_job(BANK0, BANK1, n, 0, accumulate=False)
    assert st == DONE | ERR, hex(st)
    assert len(env.dst.write_log) == n
    await env.cfg.write(STATUS, DONE)                           # W1C on DONE only
    assert await env.cfg.read(STATUS) == ERR
    await env.cfg.write(STATUS, ERR)                            # W1C on ERR only
    assert await env.cfg.read(STATUS) == 0
    st = await env.run_job(BANK0, BANK1, n, 0, accumulate=False)
    assert st == DONE | ERR
    await env.cfg.write(STATUS, ERR)
    assert await env.cfg.read(STATUS) == DONE
    # a clean job afterwards reports no error
    env.src.t.err_read_addrs.clear()
    env.dst.t.err_write_addrs.clear()
    await check_job(env, n, 0, accumulate=False)
    assert await env.cfg.read(STATUS) == DONE


@cocotb.test()
async def test_irq(dut):
    env = Env(dut, seed=17)
    await env.start()
    assert int(dut.irq.value) == 0
    env.mem.load(BANK0, rand_words(env.rng, 64))
    await env.cfg.write(SRC, BANK0)
    await env.cfg.write(DST, BANK1)
    await env.cfg.write(LEN, 64)
    await env.cfg.write(CTRL, START | IRQ_EN)
    for _ in range(2000):
        await RisingEdge(dut.clock)
        if int(dut.irq.value):
            break
    else:
        raise AssertionError("irq never asserted")
    st = await env.cfg.read(STATUS)
    assert st == DONE
    await env.cfg.write(STATUS, DONE)
    await RisingEdge(dut.clock)
    assert int(dut.irq.value) == 0
    # with IRQ_EN clear the line stays low
    await env.cfg.write(CTRL, START)
    await env.wait_done(2000)
    assert int(dut.irq.value) == 0


@cocotb.test()
async def test_start_while_busy(dut):
    """START and register writes during a job do not disturb it; they apply to the next job."""
    slow = SlaveTiming(p_arready=0.3, r_lat=(3, 6))
    env = Env(dut, slow, SlaveTiming(p_awready=0.5), seed=18)
    await env.start()
    n1, n2 = 120, 40
    x1a, x2a = rand_words(env.rng, n1), rand_words(env.rng, n1)
    x1b = rand_words(env.rng, n2)
    env.mem.load(BANK0, x1a)
    env.mem.load(BANK1, x2a)
    env.mem.load(BANK0 + 0x4000, x1b)
    env.mem.load(BANK1 + 0x4000, [0] * n2)
    await env.cfg.write(SRC, BANK0)
    await env.cfg.write(DST, BANK1)
    await env.cfg.write(LEN, n1)
    await env.cfg.write(SHIFT, 2)
    await env.cfg.write(CTRL, START | MODE_ACC)
    assert (await env.cfg.read(STATUS)) & BUSY
    await env.cfg.write(SRC, BANK0 + 0x4000)                   # retarget while busy
    await env.cfg.write(DST, BANK1 + 0x4000)
    await env.cfg.write(LEN, n2)
    await env.cfg.write(SHIFT, 0)
    await env.cfg.write(CTRL, START)                            # ignored: still busy
    assert (await env.cfg.read(STATUS)) & BUSY
    await env.wait_done(20000)
    assert env.mem.dump(BANK1, n1) == mag_accumulate(x1a, x2a, 2, True, N_ITER, GUARD)
    assert env.mem.dump(BANK1 + 0x4000, n2) == [0] * n2, "ignored START must not run a job"
    await env.cfg.write(CTRL, START)                            # now runs the retargeted job
    await env.wait_done(20000)
    assert env.mem.dump(BANK1 + 0x4000, n2) == mag_accumulate(x1b, None, 0, False, N_ITER, GUARD)
    env.check_protocol()


@cocotb.test()
async def test_unaligned_base_ignored(dut):
    env = Env(dut, seed=19)
    await env.start()
    n = 16
    x1, x2 = rand_words(env.rng, n), rand_words(env.rng, n)
    env.mem.load(BANK0, x1)
    env.mem.load(BANK1, x2)
    await env.run_job(BANK0 | 3, BANK1 | 2, n, 0, accumulate=True)
    assert env.mem.dump(BANK1, n) == mag_accumulate(x1, x2, 0, True, N_ITER, GUARD)
    env.check_protocol()


@cocotb.test()
async def test_throughput(dut):
    """Single-cycle memory on both ports: the engine should approach one element per cycle."""
    env = Env(dut, seed=20)
    await env.start()
    n = 1024
    for accumulate in (False, True):
        await check_job(env, n, 0, accumulate)
        cyc = await env.cfg.read(CYCLES)
        dut._log.info(f"mode={'acc' if accumulate else 'ovw'}: {n} elements in {cyc} cycles "
                      f"({cyc / n:.3f} cycles/element, FIFO_DEPTH={FIFO_DEPTH})")
        # Round trip for one element: AR -> R -> FIFO -> N_ITER+2 pipeline -> res FIFO -> AW/W -> B.
        if FIFO_DEPTH >= N_ITER + 8:
            assert cyc <= n + N_ITER + 16, f"{cyc} cycles for {n} elements"


@cocotb.test()
async def test_write_stall(dut):
    """Hold the write channel off until every FIFO is full, then release it.
    Exercises the credit limits at exactly FIFO_DEPTH in both modes."""
    env = Env(dut, seed=21)
    await env.start()
    n = 3 * FIFO_DEPTH + N_ITER + 7
    for accumulate in (True, False):
        env.dst.t = SlaveTiming(p_awready=0.0, p_wready=0.0)
        env.fifo_max = [0, 0, 0]
        x1, x2 = rand_words(env.rng, n), rand_words(env.rng, n)
        env.mem.load(BANK0, x1)
        env.mem.load(BANK1, x2)
        await env.cfg.write(SRC, BANK0)
        await env.cfg.write(DST, BANK1)
        await env.cfg.write(LEN, n)
        await env.cfg.write(CTRL, START | (MODE_ACC if accumulate else 0))
        await ClockCycles(dut.clock, 4 * FIFO_DEPTH + N_ITER + 50)
        assert env.fifo_max[0] == FIFO_DEPTH and env.fifo_max[2] == FIFO_DEPTH, env.fifo_max
        if accumulate:
            assert env.fifo_max[1] == FIFO_DEPTH, env.fifo_max
        env.dst.t = SlaveTiming(p_awready=0.6, p_wready=0.6)
        await env.wait_done(20000)
        assert env.mem.dump(BANK1, n) == mag_accumulate(x1, x2, 0, accumulate, N_ITER, GUARD)
    env.check_protocol()
