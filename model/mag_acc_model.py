"""Bit-accurate reference model of the magnitude-accumulate engine (rtl/mag_acc_engine.v).

Per element i the engine computes

    X2[i] := (ACCUMULATE ? X2[i] : 0) + (cordic_mag(X1[i]) >> SHIFT)      (mod 2^32)

where X1[i] is one packed 32-bit word, I = bits[15:0] and Q = bits[31:16], both
two's-complement int16, and X2[i] is a uint32 accumulator word.

cordic_mag() is a vectoring-mode CORDIC with N_ITER iterations and GUARD extra
LSBs. Its output is approximately K_N * sqrt(I^2 + Q^2), where
K_N = prod_{k<N} sqrt(1 + 2^-2k) -> 1.646760... The gain is deliberately not
compensated, because a CA-CFAR test z_cut > T * mean(z_ref) is homogeneous of
degree one and the constant cancels.

The arithmetic below mirrors the RTL operation for operation: Python's >> on a
negative int is an arithmetic (floor) shift, exactly like Verilog's >>> on a signed
operand. No intermediate in the datapath overflows the RTL register widths, which
check_widths() asserts, so no masking is needed until the final uint32 add.
"""

from __future__ import annotations

import math

import numpy as np

N_ITER_DEFAULT = 11
GUARD_DEFAULT = 6
MAG_BITS = 17  # output width of cordic_mag; max value is < 2^17 for any int16 pair

U32 = 0xFFFFFFFF


def _i16(v: int) -> int:
    """Interpret the low 16 bits of v as a two's-complement int16."""
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def unpack_iq(word: int) -> tuple[int, int]:
    """Split a packed X1 word into (I, Q): I = bits[15:0], Q = bits[31:16]."""
    return _i16(word), _i16(word >> 16)


def pack_iq(i: int, q: int) -> int:
    """Pack int16 I and Q into one X1 word (inverse of unpack_iq)."""
    return ((q & 0xFFFF) << 16) | (i & 0xFFFF)


def cordic_gain(n_iter: int = N_ITER_DEFAULT) -> float:
    """K_N = prod_{k=0}^{N-1} sqrt(1 + 2^-2k)."""
    return math.prod(math.sqrt(1.0 + 2.0 ** (-2 * k)) for k in range(n_iter))


def cordic_mag(word: int, n_iter: int = N_ITER_DEFAULT, guard: int = GUARD_DEFAULT) -> int:
    """Bit-accurate CORDIC magnitude of one packed I/Q word. Returns an int in [0, 2^17)."""
    i, q = unpack_iq(word)
    x = abs(i) << guard          # fold into the right half-plane; |-32768| = 32768 is kept exactly
    y = q << guard
    for k in range(n_iter):
        if y >= 0:               # d_k = +1: rotate clockwise
            x, y = x + (y >> k), y - (x >> k)
        else:                    # d_k = -1: rotate counter-clockwise
            x, y = x - (y >> k), y + (x >> k)
    if guard > 0:
        return (x + (1 << (guard - 1))) >> guard   # round half up, drop the guard bits
    return x


def mag_accumulate(x1, x2, shift: int, accumulate: bool,
                   n_iter: int = N_ITER_DEFAULT, guard: int = GUARD_DEFAULT) -> list[int]:
    """Scalar reference for one engine job. x1, x2 are sequences of uint32 words.

    Returns the new X2 contents. In overwrite mode (accumulate=False) the old X2
    is ignored and may be None.
    """
    if not 0 <= shift <= 31:
        raise ValueError("shift is a 5-bit field")
    out = []
    for n, w in enumerate(x1):
        m = cordic_mag(int(w), n_iter, guard) >> shift
        acc = int(x2[n]) if accumulate else 0
        out.append((acc + m) & U32)
    return out


# --------------------------------------------------------------------------- vectorised
def cordic_mag_np(words: np.ndarray, n_iter: int = N_ITER_DEFAULT,
                  guard: int = GUARD_DEFAULT) -> np.ndarray:
    """Vectorised cordic_mag over an array of packed uint32 words (bit-identical)."""
    w = np.asarray(words, dtype=np.int64) & U32
    i = (w & 0xFFFF).astype(np.int64)
    q = ((w >> 16) & 0xFFFF).astype(np.int64)
    i = np.where(i >= 0x8000, i - 0x10000, i)
    q = np.where(q >= 0x8000, q - 0x10000, q)
    x = np.abs(i) << guard
    y = q << guard
    for k in range(n_iter):
        pos = y >= 0
        xs, ys = x >> k, y >> k                    # numpy >> on int64 is arithmetic, like Python
        x, y = np.where(pos, x + ys, x - ys), np.where(pos, y - xs, y + xs)
    if guard > 0:
        x = (x + (1 << (guard - 1))) >> guard
    return x


def mag_accumulate_np(x1, x2, shift: int, accumulate: bool,
                      n_iter: int = N_ITER_DEFAULT, guard: int = GUARD_DEFAULT) -> np.ndarray:
    """Vectorised mag_accumulate; returns uint32 array."""
    m = cordic_mag_np(x1, n_iter, guard) >> shift
    acc = (np.asarray(x2, dtype=np.int64) & U32) if accumulate else 0
    return ((acc + m) & U32).astype(np.uint32)


# --------------------------------------------------------------------------- width check
def internal_extremes(n_iter: int = N_ITER_DEFAULT, guard: int = GUARD_DEFAULT,
                      grid: int = 257) -> tuple[int, int]:
    """Max |x| and |y| over all CORDIC stages for a dense input grid incl. all corners."""
    vals = np.unique(np.concatenate([
        np.linspace(-32768, 32767, grid).astype(np.int64),
        np.array([-32768, -32767, -1, 0, 1, 32766, 32767], dtype=np.int64)]))
    I, Q = np.meshgrid(vals, vals)
    x = np.abs(I.ravel()) << guard
    y = Q.ravel() << guard
    xmax, ymax = int(np.abs(x).max()), int(np.abs(y).max())
    for k in range(n_iter):
        pos = y >= 0
        xs, ys = x >> k, y >> k
        x, y = np.where(pos, x + ys, x - ys), np.where(pos, y - xs, y + xs)
        xmax, ymax = max(xmax, int(np.abs(x).max())), max(ymax, int(np.abs(y).max()))
        assert (x >= 0).all(), "x must stay non-negative after the fold"
    return xmax, ymax


def check_widths(n_iter: int = N_ITER_DEFAULT, guard: int = GUARD_DEFAULT) -> int:
    """Assert the RTL's datapath width W = 19 + GUARD (signed) holds every intermediate."""
    w = 19 + guard
    xmax, ymax = internal_extremes(n_iter, guard)
    lim = 1 << (w - 1)
    assert xmax < lim and ymax < lim, (xmax, ymax, lim)
    assert ((xmax + (1 << max(guard - 1, 0))) >> guard) < (1 << MAG_BITS)
    return w
