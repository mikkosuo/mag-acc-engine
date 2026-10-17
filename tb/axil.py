"""AXI4-Lite verification components for the magnitude-accumulate engine.

Timing discipline used by every component: drive right after a rising edge,
sample in the ReadOnly phase after the falling edge. Testbench drives only change
after rising edges and the DUT only changes on rising edges, so the values seen at
the falling edge are the values present at the next rising edge. A handshake seen
at a falling edge therefore completes at the following rising edge.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field

from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge

OKAY, SLVERR = 0, 2


async def _sample_point(clk):
    await FallingEdge(clk)
    await ReadOnly()


def _sig(dut, prefix, name):
    return getattr(dut, f"{prefix}_{name}")


# ----------------------------------------------------------------------------- master
class AxilMaster:
    """Drives the DUT's AXI4-Lite configuration slave (s_axil_*)."""

    def __init__(self, dut, clk, prefix: str = "s_axil"):
        self.clk = clk
        self.s = {n: _sig(dut, prefix, n) for n in (
            "awaddr", "awprot", "awvalid", "awready", "wdata", "wstrb", "wvalid", "wready",
            "bresp", "bvalid", "bready", "araddr", "arprot", "arvalid", "arready",
            "rdata", "rresp", "rvalid", "rready")}
        for n in ("awaddr", "awprot", "awvalid", "wdata", "wstrb", "wvalid",
                  "araddr", "arprot", "arvalid"):
            self.s[n].value = 0
        self.s["bready"].value = 0
        self.s["rready"].value = 0

    async def write(self, addr: int, data: int, strb: int = 0xF,
                    aw_delay: int = 0, w_delay: int = 0, b_delay: int = 0) -> int:
        """One write; AW and W can be offset by a number of cycles. Returns BRESP."""
        s = self.s
        await RisingEdge(self.clk)
        aw_left, w_left, cyc = True, True, 0
        while aw_left or w_left:
            if aw_left and cyc >= aw_delay:
                s["awaddr"].value, s["awvalid"].value = addr, 1
            if w_left and cyc >= w_delay:
                s["wdata"].value, s["wstrb"].value, s["wvalid"].value = data & 0xFFFFFFFF, strb, 1
            await _sample_point(self.clk)
            aw_hs = aw_left and int(s["awvalid"].value) == 1 and int(s["awready"].value) == 1
            w_hs = w_left and int(s["wvalid"].value) == 1 and int(s["wready"].value) == 1
            await RisingEdge(self.clk)
            if aw_hs:
                aw_left, s["awvalid"].value = False, 0
            if w_hs:
                w_left, s["wvalid"].value = False, 0
            cyc += 1
        for _ in range(b_delay):
            await RisingEdge(self.clk)
        s["bready"].value = 1
        while True:
            await _sample_point(self.clk)
            if int(s["bvalid"].value) == 1:
                resp = int(s["bresp"].value)
                await RisingEdge(self.clk)
                s["bready"].value = 0
                return resp
            await RisingEdge(self.clk)

    async def read(self, addr: int, r_delay: int = 0) -> int:
        s = self.s
        await RisingEdge(self.clk)
        s["araddr"].value, s["arvalid"].value = addr, 1
        while True:
            await _sample_point(self.clk)
            hs = int(s["arready"].value) == 1
            await RisingEdge(self.clk)
            if hs:
                s["arvalid"].value = 0
                break
        for _ in range(r_delay):
            await RisingEdge(self.clk)
        s["rready"].value = 1
        while True:
            await _sample_point(self.clk)
            if int(s["rvalid"].value) == 1:
                data, resp = int(s["rdata"].value), int(s["rresp"].value)
                await RisingEdge(self.clk)
                s["rready"].value = 0
                assert resp == OKAY, f"config read returned RRESP={resp}"
                return data
            await RisingEdge(self.clk)


# ----------------------------------------------------------------------------- memory
class SparseMemory:
    """Word-addressed memory shared by all slave ports. Unwritten words read as a
    deterministic pseudo-random pattern so stale-data bugs are visible."""

    def __init__(self):
        self.words: dict[int, int] = {}

    def read(self, addr: int) -> int:
        assert addr % 4 == 0, f"unaligned read 0x{addr:08x}"
        return self.words.get(addr, (addr * 2654435761) & 0xFFFFFFFF)

    def write(self, addr: int, data: int, strb: int = 0xF):
        assert addr % 4 == 0, f"unaligned write 0x{addr:08x}"
        old = self.read(addr)
        mask = sum(0xFF << (8 * b) for b in range(4) if strb >> b & 1)
        self.words[addr] = (old & ~mask | data & mask) & 0xFFFFFFFF

    def load(self, base: int, values):
        for n, v in enumerate(values):
            self.words[base + 4 * n] = int(v) & 0xFFFFFFFF

    def dump(self, base: int, n: int) -> list[int]:
        return [self.read(base + 4 * i) for i in range(n)]


@dataclass
class SlaveTiming:
    """Randomised slave behaviour. p_* = probability a ready is high in a cycle;
    lat = (min, max) response latency in cycles after the request handshake."""
    p_arready: float = 1.0
    p_awready: float = 1.0
    p_wready: float = 1.0
    r_lat: tuple[int, int] = (0, 0)
    b_lat: tuple[int, int] = (0, 0)
    max_outstanding: int = 1 << 30
    err_read_addrs: set = field(default_factory=set)
    err_write_addrs: set = field(default_factory=set)


class AxilMemSlave:
    """In-order AXI4-Lite slave port onto a SparseMemory.

    has_write=False models a read-only port (m_src_*). Records every read and write
    address and checks that the master keeps VALID high and the payload stable
    until the handshake (AXI4 A3.2.1)."""

    def __init__(self, dut, clk, prefix: str, mem: SparseMemory, timing: SlaveTiming,
                 has_write: bool, rng: random.Random):
        self.clk, self.mem, self.t, self.rng, self.prefix = clk, mem, timing, rng, prefix
        g = lambda n: _sig(dut, prefix, n)
        self.ar = (g("araddr"), g("arvalid"), g("arready"))
        self.r = (g("rdata"), g("rresp"), g("rvalid"), g("rready"))
        self.has_write = has_write
        if has_write:
            self.aw = (g("awaddr"), g("awvalid"), g("awready"))
            self.w = (g("wdata"), g("wstrb"), g("wvalid"), g("wready"))
            self.b = (g("bresp"), g("bvalid"), g("bready"))
        self._pending = 0              # requests accepted whose response handshake is not done
        self.read_log: list[int] = []
        self.write_log: list[int] = []
        self.errors: list[str] = []
        self.cycle = 0
        self._reset_outputs()

    def _reset_outputs(self):
        self.ar[2].value = 0
        self.r[0].value, self.r[1].value, self.r[2].value = 0, 0, 0
        if self.has_write:
            self.aw[2].value = 0
            self.w[3].value = 0
            self.b[0].value, self.b[1].value = 0, 0

    def _ready(self, p: float) -> int:
        return 1 if p >= 1.0 else int(self.rng.random() < p)

    def _lat(self, lo_hi) -> int:
        return self.rng.randint(*lo_hi)

    async def run(self):
        rq: deque = deque()          # (ready_cycle, data, resp) of accepted reads
        awq: deque = deque()         # accepted write addresses
        wq: deque = deque()          # accepted (data, strb)
        bq: deque = deque()          # (ready_cycle, resp)
        r_cur = b_cur = None         # response currently being presented
        prev = {}                    # previous-cycle (valid, ready, payload) per channel

        while True:
            await RisingEdge(self.clk)
            self.cycle += 1
            # ---- drive this cycle
            outstanding = len(rq) + (r_cur is not None)
            self.ar[2].value = self._ready(self.t.p_arready) if outstanding < self.t.max_outstanding else 0
            if r_cur is None and rq and rq[0][0] <= self.cycle:
                r_cur = rq.popleft()
            if r_cur is not None:
                self.r[0].value, self.r[1].value, self.r[2].value = r_cur[1], r_cur[2], 1
            else:
                self.r[2].value = 0
            if self.has_write:
                self.aw[2].value = self._ready(self.t.p_awready)
                self.w[3].value = self._ready(self.t.p_wready)
                if b_cur is None and bq and bq[0][0] <= self.cycle:
                    b_cur = bq.popleft()
                if b_cur is not None:
                    self.b[0].value, self.b[1].value = b_cur[1], 1
                else:
                    self.b[1].value = 0

            # ---- sample what will happen at the next rising edge
            await _sample_point(self.clk)
            now = self.cycle

            arv, arr = int(self.ar[1].value), int(self.ar[2].value)
            ar_pay = int(self.ar[0].value) if arv else None
            self._check_stable("AR", prev.get("ar"), arv, ar_pay)
            prev["ar"] = (arv, arr, ar_pay)
            if arv and arr:
                a = ar_pay
                self.read_log.append(a)
                resp = SLVERR if a in self.t.err_read_addrs else OKAY
                rq.append((now + 1 + self._lat(self.t.r_lat), self.mem.read(a), resp))
                self._pending += 1

            if r_cur is not None and int(self.r[3].value) == 1:
                r_cur = None
                self._pending -= 1

            if self.has_write:
                awv, awr = int(self.aw[1].value), int(self.aw[2].value)
                aw_pay = int(self.aw[0].value) if awv else None
                self._check_stable("AW", prev.get("aw"), awv, aw_pay)
                prev["aw"] = (awv, awr, aw_pay)
                if awv and awr:
                    awq.append(aw_pay)

                wv, wr = int(self.w[2].value), int(self.w[3].value)
                w_pay = (int(self.w[0].value), int(self.w[1].value)) if wv else None
                self._check_stable("W", prev.get("w"), wv, w_pay)
                prev["w"] = (wv, wr, w_pay)
                if wv and wr:
                    wq.append(w_pay)

                while awq and wq:
                    a = awq.popleft()
                    d, st = wq.popleft()
                    self.write_log.append(a)
                    bad = a in self.t.err_write_addrs
                    if not bad:
                        self.mem.write(a, d, st)
                    bq.append((now + 1 + self._lat(self.t.b_lat), SLVERR if bad else OKAY))
                    self._pending += 1

                if b_cur is not None and int(self.b[2].value) == 1:
                    b_cur = None
                    self._pending -= 1

    @property
    def pending(self) -> int:
        """Accepted requests (reads, or paired writes) still awaiting their response handshake.
        Unpaired AW or W beats are visible in the write log check instead."""
        return self._pending

    def _check_stable(self, ch, prev, valid, payload):
        if prev is None:
            return
        pv, pr, ppay = prev
        if pv and not pr:
            if not valid:
                self.errors.append(f"{self.prefix} {ch}: VALID dropped before handshake (cycle {self.cycle})")
            elif payload != ppay:
                self.errors.append(f"{self.prefix} {ch}: payload changed before handshake (cycle {self.cycle})")
