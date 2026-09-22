"""
Golden reference model for the 16x14 INT8 weight-stationary systolic array.

Pure Python integers throughout -- no numpy, no floating point anywhere. That is
deliberate: "bit-exact against the golden model" is only a meaningful claim if
the golden model cannot itself round. Python ints are arbitrary precision, so
every value here is the mathematically exact one, and any disagreement with the
RTL is a real RTL bug rather than a modelling artefact.

Three layers, each checking the one below:

    gemm_tile()        what the array should compute
    packed_group()     what the DSP cascade actually computes, bit for bit,
                       including the 25-bit A-port truncation and the signed
                       borrow -- so the packing hazards are modelled, not
                       assumed away
    verify()           asserts the two agree over the full weight/activation
                       space

Run directly to execute the self-check:  python3 model/gemm_int8.py
"""

from __future__ import annotations

import random
import sys

# ---------------------------------------------------------------------------
# Parameters -- must track rtl/sa_pkg.sv. test_params_match() below enforces it.
# ---------------------------------------------------------------------------
ROWS = 16
COLS = 14
K_PACK = 17
DSP_A_W = 25
DSP_B_W = 18
DSP_P_W = 48
CASCADE_DEPTH = 4
WGT_MIN, WGT_MAX = -127, 127
ACT_MIN, ACT_MAX = -128, 127


# ---------------------------------------------------------------------------
# two's complement helpers
# ---------------------------------------------------------------------------
def to_signed(value: int, width: int) -> int:
    """Interpret the low `width` bits of `value` as a signed integer."""
    mask = (1 << width) - 1
    value &= mask
    if value & (1 << (width - 1)):
        value -= 1 << width
    return value


def fits(value: int, width: int) -> bool:
    return -(1 << (width - 1)) <= value < (1 << (width - 1))


# ---------------------------------------------------------------------------
# layer 1: what the array should compute
# ---------------------------------------------------------------------------
def gemm_tile(act: list[int], wgt: list[list[int]]) -> list[int]:
    """One activation vector against one weight tile.

    act : ROWS activations
    wgt : ROWS x COLS weights, wgt[i][j] stationary in PE(i, j)
    returns COLS partial sums.
    """
    assert len(act) == ROWS
    assert len(wgt) == ROWS and all(len(r) == COLS for r in wgt)
    return [sum(wgt[i][j] * act[i] for i in range(ROWS)) for j in range(COLS)]


# ---------------------------------------------------------------------------
# layer 2: what the DSP hardware actually does
# ---------------------------------------------------------------------------
def pack_weights(w1: int, w0: int) -> int:
    """Pack two INT8 weights into the 25-bit A port: A = w1 * 2^K + w0.

    Truncates to 25 bits exactly as the hardware does, so that passing
    w1 = -128 reproduces the overflow rather than hiding it.
    """
    return to_signed((w1 << K_PACK) + w0, DSP_A_W)


def dsp_mul(w1: int, w0: int, a: int) -> int:
    """One DSP48E1 packed multiply, truncated to the 48-bit P register."""
    a_port = pack_weights(w1, w0)
    b_port = to_signed(a, DSP_B_W)
    return to_signed(a_port * b_port, DSP_P_W)


def extract(p_packed: int) -> tuple[int, int]:
    """Recover (p0, p1) from a packed accumulator.

    The `+ borrow` term is the correction for a negative low field; without it
    p1 is one too small whenever p0 < 0. See ARCHITECTURE.md §3.4.
    """
    lo = to_signed(p_packed, K_PACK)
    borrow = 1 if lo < 0 else 0
    hi = to_signed(p_packed >> K_PACK, DSP_P_W - K_PACK) + borrow
    return lo, hi


def packed_group(w1s: list[int], w0s: list[int], acts: list[int]) -> tuple[int, int]:
    """One CASCADE_DEPTH-deep DSP cascade: accumulate packed, extract once."""
    acc = 0
    for w1, w0, a in zip(w1s, w0s, acts):
        acc = to_signed(acc + dsp_mul(w1, w0, a), DSP_P_W)
    return extract(acc)


def gemm_tile_hw(act: list[int], wgt: list[list[int]]) -> list[int]:
    """Full tile the way the hardware computes it: column pairs share a DSP,
    each column split into ROWS/CASCADE_DEPTH cascade groups, group results
    summed in the fabric."""
    out = [0] * COLS
    for jp in range(COLS // 2):
        j0, j1 = 2 * jp, 2 * jp + 1
        s0 = s1 = 0
        for grp in range(ROWS // CASCADE_DEPTH):
            rows = range(grp * CASCADE_DEPTH, (grp + 1) * CASCADE_DEPTH)
            g0, g1 = packed_group(
                [wgt[i][j1] for i in rows],
                [wgt[i][j0] for i in rows],
                [act[i] for i in rows],
            )
            s0 += g0
            s1 += g1
        out[j0], out[j1] = s0, s1
    return out


# ---------------------------------------------------------------------------
# layer 3: self-check
# ---------------------------------------------------------------------------
def rand_weights(rng: random.Random, extreme: float = 0.3) -> list[list[int]]:
    def w() -> int:
        if rng.random() < extreme:
            return rng.choice([WGT_MIN, WGT_MAX])
        return rng.randint(WGT_MIN, WGT_MAX)

    return [[w() for _ in range(COLS)] for _ in range(ROWS)]


def rand_acts(rng: random.Random, extreme: float = 0.3) -> list[int]:
    def a() -> int:
        if rng.random() < extreme:
            return rng.choice([ACT_MIN, ACT_MAX])
        return rng.randint(ACT_MIN, ACT_MAX)

    return [a() for _ in range(ROWS)]


def test_cascade_bound() -> None:
    """CASCADE_DEPTH is the largest depth whose sum still fits the low field."""
    prod_max = WGT_MAX * abs(ACT_MIN)  # 127 * 128 = 16256
    limit = (1 << (K_PACK - 1)) - 1  # 65535
    assert CASCADE_DEPTH * prod_max <= limit, "cascade too deep for the low field"
    assert (CASCADE_DEPTH + 1) * prod_max > limit, (
        "cascade could be deeper than CASCADE_DEPTH -- DSP accumulators are "
        "being wasted on fabric adds"
    )
    # The clamp on the LOW-field weight is load-bearing for exactly this bound:
    # allowing w0 = -128 gives 4 * 128 * 128 = 65536, one over the limit. This
    # is a different reason from the A-port constraint on w1, and it is why both
    # weights are clamped even though the A port only requires it of w1.
    assert CASCADE_DEPTH * 128 * 128 > limit, (
        "the [-127,127] clamp on w0 is what makes depth 4 fit; re-derive if it "
        "is ever relaxed"
    )
    print(f"  cascade bound      : depth {CASCADE_DEPTH} is maximal  "
          f"({CASCADE_DEPTH * prod_max} <= {limit} < {(CASCADE_DEPTH+1) * prod_max})")
    print(f"  w0 clamp is needed : 4*128*128 = {4*128*128} > {limit} by "
          f"{4*128*128 - limit}")


def test_a_port_overflow() -> None:
    """Pin down exactly where the 25-bit A port overflows.

    The boundary is sharper than "-128 is bad": A = w1*2^K + w0 exceeds the
    25-bit signed floor only when w1 = -128 AND w0 < 0, because -128*2^17 is
    exactly -2^24, the most negative representable value, so any negative w0
    pushes it over. w1 = -127 is safe for every w0, including -128.

    So the A port alone would permit w0 = -128. The clamp on w0 comes from a
    different constraint entirely -- see test_cascade_bound.
    """
    # the whole legal corner of the space is representable
    assert fits((WGT_MAX << K_PACK) + WGT_MAX, DSP_A_W)
    assert fits((WGT_MIN << K_PACK) + WGT_MIN, DSP_A_W)
    assert fits((WGT_MIN << K_PACK) + -128, DSP_A_W)  # w1=-127, w0=-128 is fine

    # w1 = -128 is the exact boundary: safe for w0 >= 0, overflows for w0 < 0
    assert fits((-128 << K_PACK) + 0, DSP_A_W)
    assert not fits((-128 << K_PACK) + -1, DSP_A_W)
    assert not fits((-128 << K_PACK) + -128, DSP_A_W)

    # and the overflow really corrupts the result rather than being benign
    p0, p1 = extract(dsp_mul(-128, -1, 1))
    assert (p1, p0) != (-128, -1), "expected w1=-128,w0=-1 to corrupt, but it did not"
    print(f"  A-port overflow    : w1=-128 safe iff w0>=0; w0=-1 corrupts "
          f"p1 -> {p1} (want -128)")


def test_borrow_correction() -> None:
    """Exhaustive over single products: the packed path must equal the plain
    product for every legal (w1, w0, a)."""
    n = 0
    for w1 in range(WGT_MIN, WGT_MAX + 1):
        for w0 in range(WGT_MIN, WGT_MAX + 1):
            for a in range(ACT_MIN, ACT_MAX + 1):
                p0, p1 = extract(dsp_mul(w1, w0, a))
                if p0 != w0 * a or p1 != w1 * a:
                    raise AssertionError(
                        f"packing mismatch at w1={w1} w0={w0} a={a}: "
                        f"got ({p1}, {p0}) want ({w1*a}, {w0*a})"
                    )
                n += 1
    print(f"  packing round-trip : {n:,} triples exhaustively verified")


def test_hw_matches_math(trials: int = 3000, seed: int = 0xA1E5) -> None:
    """The hardware decomposition must equal the plain GEMM."""
    rng = random.Random(seed)
    for _ in range(trials):
        wgt = rand_weights(rng)
        act = rand_acts(rng)
        ref = gemm_tile(act, wgt)
        got = gemm_tile_hw(act, wgt)
        if ref != got:
            raise AssertionError(f"tile mismatch\n  ref={ref}\n  got={got}")
    # and the all-extremes worst case, which random search will not hit
    for wv, av in ((WGT_MAX, ACT_MIN), (WGT_MIN, ACT_MIN),
                   (WGT_MAX, ACT_MAX), (WGT_MIN, ACT_MAX)):
        wgt = [[wv] * COLS for _ in range(ROWS)]
        act = [av] * ROWS
        assert gemm_tile(act, wgt) == gemm_tile_hw(act, wgt), (wv, av)
    peak = ROWS * WGT_MAX * abs(ACT_MIN)
    print(f"  hw == math         : {trials:,} random tiles + 4 extreme tiles")
    print(f"  column sum range   : +/-{peak} -> needs {peak.bit_length()+1} bits "
          f"(sa_pkg COLSUM_W = 20)")


def test_params_match_rtl() -> None:
    """Fail loudly if this file and sa_pkg.sv drift apart."""
    import pathlib
    import re

    pkg = pathlib.Path(__file__).resolve().parent.parent / "rtl" / "sa_pkg.sv"
    if not pkg.exists():
        print("  param sync         : SKIPPED (rtl/sa_pkg.sv not found)")
        return
    text = pkg.read_text()
    expect = {
        "ROWS": ROWS, "COLS": COLS, "K_PACK": K_PACK,
        "CASCADE_DEPTH": CASCADE_DEPTH, "WGT_MIN": WGT_MIN, "WGT_MAX": WGT_MAX,
        "DSP_A_W": DSP_A_W, "DSP_B_W": DSP_B_W, "DSP_P_W": DSP_P_W,
    }
    for name, want in expect.items():
        m = re.search(rf"parameter\s+int\s+{name}\s*=\s*(-?\d+)\s*;", text)
        if not m:
            raise AssertionError(f"{name} not found in sa_pkg.sv")
        got = int(m.group(1))
        if got != want:
            raise AssertionError(
                f"{name} drifted: model has {want}, sa_pkg.sv has {got}"
            )
    print(f"  param sync         : {len(expect)} parameters match sa_pkg.sv")


def verify() -> int:
    print("=" * 57)
    print(" golden model self-check")
    print("=" * 57)
    test_params_match_rtl()
    test_cascade_bound()
    test_a_port_overflow()
    test_hw_matches_math()
    test_borrow_correction()
    print("-" * 57)
    print(" PASS  all golden-model checks")
    print("=" * 57)
    return 0


if __name__ == "__main__":
    sys.exit(verify())
