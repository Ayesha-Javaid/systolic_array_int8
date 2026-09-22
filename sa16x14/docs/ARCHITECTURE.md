# Architecture — 16×14 Weight-Stationary INT8 Systolic Array

Target: Xilinx Artix-7 **XC7A100T-1CSG324C** (Arty A7-100T)
Clock target: 100 MHz
Arithmetic: INT8 × INT8 → INT32 accumulate

---

## 1. Top-level dataflow

The array is **weight-stationary (WS)**: each processing element (PE) holds one
weight for the duration of a GEMM tile. Activations stream in from the left and
propagate right; partial sums accumulate downward through each column.

```
                 act[0] →  PE(0,0) → PE(0,1) → … → PE(0,13)
                 act[1] →  PE(1,0) → PE(1,1) → … → PE(1,13)
                    ⋮          ↓         ↓             ↓
                act[15] →  PE(15,0)→ PE(15,1)→ … → PE(15,13)
                               ↓         ↓             ↓
                            psum[0]   psum[1]       psum[13]
```

Per PE, per cycle:

```
psum_out <= psum_in + w * act_in
act_out  <= act_in
```

With a 16-row array, each output column value is the dot product of a
16-element activation vector with a 16-element weight column. The array
computes a **16×14 tile of a GEMM** — i.e. `C[1×14] += A[1×16] · W[16×14]`
every cycle once the pipeline is full.

Peak throughput: **16 × 14 = 224 MACs/cycle**. At 100 MHz that is
**22.4 GMAC/s = 44.8 GOP/s**.

---

## 2. Why the array is 16 rows × 14 columns

The shape is set by the DSP budget, not by convenience.

- XC7A100T provides **240 DSP48E1 slices**.
- With two INT8 MACs packed per DSP (§3), 224 MACs needs **112 DSPs**.
- 14 columns divides evenly by 2, so every DSP is fully packed — no odd
  column left running at half utilisation.
- 16 rows matches the depth at which the packed accumulator analysis in §4
  works out cleanly (four cascade groups of four).

> **Open item for Ayesha (see ROADMAP §Day 0):** your résumé bullet says
> *"128 of 240 DSP slices"*. The arithmetic above lands on **112** for the
> array proper. 112 + 16 = 128, which would be one extra DSP per row — that
> matches a design where each row also has a dedicated DSP (e.g. bias add or
> a 15th accumulation stage). Before you publish this repo, decide which
> number is true for the design you actually built, and we will make the RTL
> and the résumé agree. The repo will report whatever the real post-synthesis
> utilisation report says.

---

## 3. Packing two INT8 MACs into one DSP48E1

### 3.1 The primitive

DSP48E1 computes `P[47:0] = (A ± D) × B + C`, where the multiplier is
**25 bits (A) × 18 bits (B)** signed.

### 3.2 The packing

Two weights `w0, w1` (signed INT8) share one activation `a` (signed INT8).
Pack both weights into the 25-bit A port at an offset of `k` bits:

```
A = w1 · 2^k + w0            (as a signed 25-bit integer)
B = a
P = A × B = (w1·a) · 2^k + (w0·a)
```

Both products land in `P` in separate bit-fields. Let `p0 = w0·a` and
`p1 = w1·a`.

### 3.3 Choosing k

`w1` occupies bits `[k+7 : k]` of A, so `k + 8 ≤ 25` ⟹ **k ≤ 17**.
We take **k = 17**, the maximum, because every bit of headroom goes to the
low field's accumulation margin (§4).

### 3.4 The signed-borrow correction

`P = p1·2^17 + p0`. Extraction is *not* a plain bit-slice, because a negative
`p0` borrows from the field above it:

```
p0 = signed(P[16:0])                     // exact: |p0| ≤ 16256 < 2^16
p1 = signed(P[47:17]) + P[16]            // +1 corrects the borrow
```

Proof of the correction: `signed(P[47:17]) = floor(P / 2^17)`. Substituting
`P = p1·2^17 + p0` gives `floor(p1 + p0/2^17)`, which equals `p1` when
`p0 ≥ 0` and `p1 − 1` when `p0 < 0`. `P[16]` is exactly the sign bit of the
17-bit low field, so adding it restores `p1` in both cases. ∎

### 3.5 The −128 exclusion

The packed A value must fit in 25 bits signed, range `[−16777216, 16777215]`.
The boundary is sharper than "−128 is bad", and it is worth being exact about
because the two halves of the pack are **not** symmetric:

```
A_max = 127·2^17 + 127      =  16646271   ✓
A     = −127·2^17 − 128     = −16646272   ✓   w1 = −127 is safe for every w0
A     = −128·2^17 + 0       = −16777216   ✓   exactly the 25-bit floor
A     = −128·2^17 − 1       = −16777217   ✗   any negative w0 tips it over
A     = −128·2^17 − 128     = −16777344   ✗
```

So the A port alone forbids only **`w1 = −128` together with `w0 < 0`**.
`w0 = −128` is perfectly representable as far as the A port is concerned.

**The low-field weight is nevertheless clamped too, for an unrelated reason.**
The depth-4 cascade bound in §4 needs `|w0 · a| ≤ 16256`; permitting
`w0 = −128` gives `4 × 128 × 128 = 65536`, which is one greater than the
17-bit field can hold. Two independent constraints, one on each half of the
pack, converge on the same rule:

> **all weights ∈ [−127, +127]**

This is not a compromise imposed by the hardware — it is exactly what
symmetric INT8 quantisation already does. PyTorch (`qint8`, symmetric) and
TFLite both clamp weights to `[−127, 127]` so that the zero-point is exactly
zero and negation is exact. The packing constraints and the quantisation
convention agree for free.

Activations are unconstrained over the full `[−128, 127]` INT8 range.
With the weight clamp, `|p| ≤ 127 × 128 = 16256`.

Both boundaries are asserted, not assumed: see `test_a_port_overflow` and
`test_cascade_bound` in `model/gemm_int8.py`.

---

## 4. Accumulation strategy: depth-4 DSP cascade groups

The tempting move is to accumulate in the packed domain all the way down a
column, using the DSP48E1 `PCIN/PCOUT` cascade, and extract once at the
bottom. **This does not work at full depth**, and the reason sets the
architecture:

Accumulating packed values sums each field independently:

```
Σ P = (Σ p1)·2^17 + (Σ p0)
```

The low field is only **17 bits** wide. Over a 16-deep column:

```
|Σ p0| ≤ 16 × 16256 = 260096  →  needs 20 bits.  Overflows into p1. ✗
```

The maximum cascade depth `N` that keeps the low field intact is:

```
N × 16256 ≤ 2^16 − 1 = 65535   ⟹   N ≤ 4.03   ⟹   N = 4
```

At `N = 4`: `4 × 16256 = 65024 ≤ 65535` ✓ (and `−65024 ≥ −65536` ✓).

Note this margin exists *only because* of the `[−127,127]` weight clamp from
§3.5 — with `w = −128` allowed, `4 × 16384 = 65536` overflows by one. The two
constraints resolve each other.

**Therefore each 16-deep column-pair is split into 4 cascade groups of 4
DSPs.** Within a group the DSP accumulator chain does the work for free; the
4 group results are extracted (§3.4) and summed in a small fabric adder tree.

```
rows  0– 3 → DSP cascade ─┐
rows  4– 7 → DSP cascade ─┤
rows  8–11 → DSP cascade ─┼→ extract ×4 → fabric adder tree → psum[2j], psum[2j+1]
rows 12–15 → DSP cascade ─┘
```

This keeps 3 of every 4 additions inside the DSP (zero fabric cost) while
guaranteeing bit-exactness against the integer golden model.

### 4.1 Accumulator widths

| Signal | Width | Justification |
|---|---|---|
| product `p` | 16b signed | `|p| ≤ 16256` |
| group sum (4 deep) | 18b signed | `|Σ| ≤ 65024` |
| column sum (16 deep) | 20b signed | `|Σ| ≤ 260096` |
| exported psum | 32b signed | headroom for multi-tile accumulation |

Multi-tile accumulation (K > 16) reuses the 32-bit export path, so K is
limited only by 32 bits: up to `2^31 / 16256 ≈ 132k` accumulated terms.

---

## 5. Input stagger network

PE(i,j) must see activation element `i` at the cycle when the partial sum
travelling down column `j` arrives. Because the psum takes one cycle per row,
the activation for row `i` must be delayed by **`i` cycles** relative to row 0.

Each row `i` therefore has a **shift-register delay chain of depth `i`**
(depth 0 for row 0, depth 15 for row 15). On Artix-7 these infer as **SRL16E**
primitives — one LUT per 16 bits of delay per bit of width — rather than
flip-flop chains, costing 8 SRLs per row instead of `8 × i` flip-flops.

Total stagger cost: `Σ i for i=0..15 = 120` byte-delays ≈ 8 SRL16E × 15 rows.

This is what eliminates inter-tile data hazards: without it, a new activation
tile entering the array would collide with the drain of the previous tile,
because the two travel through the array on different diagonals.

## 6. Output de-skew network

Symmetrically, column `j`'s result emerges `j` cycles after column 0's. The
egress path delays column `j` by **`13 − j`** cycles so that all 14 results of
a given activation vector present on the same cycle as one 14-wide beat to
the AXI4-Stream master.

---

## 7. Interfaces

| Interface | Direction | Width | Purpose |
|---|---|---|---|
| `s_axis_act` | slave | 128b (16 × INT8) | one activation vector per beat |
| `s_axis_wgt` | slave | 128b | weight tile load, 14 beats |
| `m_axis_psum` | master | 448b (14 × INT32) | one result vector per beat |
| `s_axi_ctrl` | AXI4-Lite | 32b | CSR: start, mode, status, tile count |

Packing activations 16-wide means one beat feeds the whole array edge per
cycle, so the array never starves at 100 MHz on a 128-bit stream.

---

## 8. Verification strategy

1. **Golden model** (`model/gemm_int8.py`) — plain integer Python, no
   floating point anywhere, so "bit-exact" is meaningful.
2. **Exhaustive** test of the packing cell over all `(w0, w1, a)` triples in
   `[−127,127] × [−127,127] × [−128,127]` = 8.3 M cases. Small enough to
   brute-force, and it is the one module where a single corner case (§3.4,
   §3.5) silently corrupts every result downstream.
3. **Directed** tests for cascade-group overflow boundaries (all-max, all-min).
4. **Randomised** full-array GEMM against the golden model.
5. **Formal** (SymbiYosys) properties on stagger alignment — a bounded proof
   that row `i` and column `j` meet at the right cycle, which is far stronger
   than sampling it with random vectors.

Every module is checked by a self-checking testbench that exits non-zero on
failure, so CI is meaningful rather than decorative.
