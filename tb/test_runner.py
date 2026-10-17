"""pytest entry point: builds the RTL and runs the cocotb tests on each simulator and
parameter set.  Usage:  cd tb && pytest -v test_runner.py   (add -k verilator etc.)"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest
from cocotb_tools.runner import get_runner

TB = Path(__file__).resolve().parent
RTL = TB.parent / "rtl"
SOURCES = [RTL / "mag_acc_engine.v", RTL / "mae_fifo.v"]

PARAM_SETS = {
    "default": dict(N_ITER=11, GUARD=6, FIFO_DEPTH=32),
    "minimal": dict(N_ITER=8, GUARD=0, FIFO_DEPTH=1),     # no guard bits, no outstanding reads
    "odd":     dict(N_ITER=16, GUARD=4, FIFO_DEPTH=5),    # non-power-of-two FIFO
}
SIMS = [s for s in ("icarus", "verilator") if shutil.which({"icarus": "iverilog"}.get(s, s))]


@pytest.mark.parametrize("params", list(PARAM_SETS), ids=list(PARAM_SETS))
@pytest.mark.parametrize("sim", SIMS)
def test_mag_acc_engine(sim, params):
    p = PARAM_SETS[params]
    build_dir = TB / "sim_build" / f"{sim}_{params}"
    runner = get_runner(sim)
    runner.build(
        sources=SOURCES,
        hdl_toplevel="mag_acc_engine",
        parameters=p,
        build_dir=build_dir,
        always=True,
        timescale=("1ns", "1ps"),
        build_args=["-Wno-fatal"] if sim == "verilator" else [],
    )
    runner.test(
        hdl_toplevel="mag_acc_engine",
        test_module="test_mag_acc_engine",
        test_dir=TB,
        build_dir=build_dir,
        results_xml=str(build_dir / "results.xml"),
        extra_env={f"MAE_{k}": str(v) for k, v in p.items()},
    )
    # runner.test does not raise on test failure; read the results file.
    xml = (build_dir / "results.xml").read_text()
    assert "<failure" not in xml and "<error" not in xml, f"cocotb failures, see {build_dir}"
