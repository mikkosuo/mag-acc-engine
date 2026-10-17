"""Self-checks of the Python reference model (no simulator needed)."""

import random
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "model"))
from mag_acc_model import (check_widths, cordic_gain, cordic_mag, cordic_mag_np,  # noqa: E402
                           mag_accumulate, mag_accumulate_np, pack_iq, unpack_iq)

CORNERS = [-32768, -32767, -1, 0, 1, 32767]


@pytest.mark.parametrize("n_iter,guard", [(11, 6), (8, 0), (16, 4)])
def test_widths_fit_rtl(n_iter, guard):
    check_widths(n_iter, guard)


@pytest.mark.parametrize("n_iter,guard", [(11, 6), (8, 0), (16, 4)])
def test_scalar_equals_vectorised(n_iter, guard):
    rng = random.Random(5)
    words = [rng.getrandbits(32) for _ in range(5000)] + [pack_iq(i, q) for i in CORNERS for q in CORNERS]
    vec = cordic_mag_np(np.array(words, dtype=np.uint64), n_iter, guard)
    assert [cordic_mag(w, n_iter, guard) for w in words] == vec.tolist()
    x2 = [rng.getrandbits(32) for _ in words]
    for shift, acc in ((0, True), (7, False), (31, True)):
        assert mag_accumulate(words, x2, shift, acc, n_iter, guard) == \
            mag_accumulate_np(words, x2, shift, acc, n_iter, guard).tolist()


def test_pack_roundtrip():
    for i in CORNERS:
        for q in CORNERS:
            assert unpack_iq(pack_iq(i, q)) == (i, q)


def test_default_accuracy():
    """Default N_ITER=11, GUARD=6: within 0.6 LSB of K_N*|z|, negligible bias."""
    rng = np.random.default_rng(7)
    i = rng.integers(-32768, 32768, 200_000)
    q = rng.integers(-32768, 32768, 200_000)
    words = ((q & 0xFFFF) << 16) | (i & 0xFFFF)
    err = cordic_mag_np(words) - cordic_gain(11) * np.hypot(i, q)
    assert np.abs(err).max() < 0.6
    assert abs(err.mean()) < 0.02
    assert cordic_mag(pack_iq(-32768, -32768)) == 76312       # largest possible output, < 2^17


def test_wraps_mod_2_32():
    assert mag_accumulate([pack_iq(3, 4)], [0xFFFFFFFF], 0, True, 11, 6) == [cordic_mag(pack_iq(3, 4)) - 1]
