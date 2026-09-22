# Tooling notes

Things that cost real debugging time, written down so they cost it only once.

---

## 1. Verilator 5.020: element-wise writes may not propagate through a port

**Symptom.** Every result from the array was zero under Verilator, while Icarus
simulated the identical source correctly and every self-check passed there.
Probing showed the stationary weights arriving inside the instance as `0`
forever, even though the testbench had clearly written them.

**Cause.** A variable that is *only ever written element-by-element*, and is
connected to a submodule input port that feeds **combinational** logic, does
not get its dependency hooked up. The instance keeps reading the initial value.

It is not about packed vs unpacked arrays — both reproduce it — and it is not
about arrays at all. A flat vector written through a part-select
(`fw0[g*8 +: 8] = ...`) reproduces it too. The common factor is a *partial*
write to a variable that has never been assigned as a whole.

**Why the activations looked fine.** Activations reached the multiplier through
`always_ff`, which re-reads its inputs on every clock edge regardless of change
detection. Only the combinational path (`assign a_packed = pack_weights(...)`)
was affected. That asymmetry is what made the bug look like an arithmetic
problem rather than a scheduling one.

**Fix.** Assign the whole variable once, then write elements freely:

```systemverilog
w0 = '0;                       // <- this line is load-bearing
for (i = 0; i < N; i++) w0[i] = something;
```

**Repo convention.** Testbenches initialise every port-connected variable as a
whole before driving elements. `tb_dsp_pack_mac.sv` has a comment at the
initialisation site; do not "tidy" it into a loop.

**How it was isolated.** Four minimal cases, each one variable different:
scalar port vs array-element port; packed vs unpacked; part-select of a flat
vector; whole-variable write followed by partial writes. Only the last
propagated. Worth repeating that method — the bug was invisible in the full
testbench and obvious in twenty lines.

---

## 2. Verilator parses comments that begin with its own name

A comment whose first word is the tool's name is read as a lint pragma:

```
// Verilator 5.020 will not propagate ...
```

fails the build with `Unknown verilator comment`. Start such a sentence with
any other word.

---

## 3. `before` is a reserved word

`integer before;` is a syntax error under Icarus with `-g2012`: `before` is a
SystemVerilog keyword (randsequence production control). The error message
points at the *next* statement, not the declaration, which makes it look like
a problem with the code after it. Renamed to `n_before`.

---

## 4. Icarus does not accept everything the LRM allows

Two constructs had to go, both in testbench code:

- dynamic-array arguments to tasks (`task f(input int a[]);`) →
  `internal error: How can there be an unpacked range here?`
- block-local declarations with initialisers inside a `begin` after statements

Testbenches are therefore written in a plainer style than strictly necessary —
module-scope declarations, fixed-size arrays — so the same source runs under
Icarus 12, Verilator 5 and Vivado's simulator without per-tool `ifdef`s. This
is deliberate; please do not modernise it without checking all three.

---

## 5. Clock generation style

`always #5 clk = ~clk;` trips Verilator's `BLKSEQ` warning ("blocking
assignment in sequential logic process"). Use:

```systemverilog
initial forever #5 clk = ~clk;
```

which is not classified as a sequential process, and needs no warning
suppression.

---

## 6. Why the RTL infers DSPs rather than instantiating them

`dsp_pack_mac.sv` is written as plain inferable arithmetic with a `use_dsp`
attribute, not as a `DSP48E1` primitive instantiation. Instantiating the
primitive would tie the regression suite to Vivado's simulation libraries,
which means CI could not run in a container without a Vivado install — and a
CI that cannot run is a CI that does not catch anything.

The cost is that DSP inference has to be *verified* rather than assumed. That
is what `make synth` and the utilisation report are for; the numbers in the
README come from that report, not from the arithmetic in
`docs/ARCHITECTURE.md`.
