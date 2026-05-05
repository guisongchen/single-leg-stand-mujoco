# Review: Walking Milestone — Post-ADR Cross-Reference

**Documents reviewed:** `docs/walking_architecture.md`, `docs/adr/001-walking-gait-architecture.md`, `docs/review_walking_milestone_glm.md`  
**Review scope:** Theoretical and physical correctness after the GRF-based transition revision. Code not examined.

---

## Summary

The GRF-based transition revision (ADR 001) resolves 8 of the 11 issues identified in the original reviews. The architecture is now theoretically sound. Three concerns remain that warrant attention, and one geometric calculation needs correction in all documents.

---

## What the ADR Revision Solved

The original reviews identified 11 compounding issues. After the GRF-based revision:

| # | Original Issue | Resolution |
|---|---------------|------------|
| 1 | Min guard (0.05 s) outlasts CP transit time (~0.03 s) | GRF transitions have no timing window problem — GRF is monotonic during weight shift |
| 2 | Biased midpoint (70/30) outside next foot's CoP polygon | DS → WS is now time-driven (0.15 s); no spatial condition at this stage |
| 3 | CP-based safety abort logically impossible | Redefined: CP outside convex hull of **both** polygons by > 0.05 m |
| 4 | Missing τ_z torsional friction constraint | Added: `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` per foot |
| 5 | Initial pose unstable for 1.0 s | `init_duration` reduced to 0.1 s |
| 6 | step_width formula coordinate error | Absolute lateral positions instead of offset-from-support |
| 7 | Quintic velocity discontinuity at phase boundaries | Dropped — PD-only setpoint during single support |
| 8 | Contact wrench discontinuity at lift-off | GRF < 5 N precondition before lift-off transition |

CP-based geometry checks are retained only as a coarse safety abort (catastrophic instability). All routine phase transitions use GRF thresholds with hysteresis, which correctly accounts for swing dynamics, is monotonic during weight shift, and has no tight geometric margin.

---

## Geometric Correction: `d_min` Diagonal Distance

All four documents contain the same error in calculating the minimum CoM shift distance.

### The error

The current documents state:

```
d_min = √(0.17² + 0.15²) ≈ 0.23 m
```

and claim 0.17 m is the "forward component." Both the forward component and the resulting diagonal are wrong.

### Correct derivation

The CoP envelope per foot extends forward +0.12 m and backward −0.05 m from the foot center. With `step_length = 0.25 m` and `step_width = 0.20 m`, the closest edges between the two CoP envelopes are:

```
Forward gap  = step_length − forward_extent − backward_extent
             = 0.25 − 0.12 − 0.05
             = 0.08 m   (NOT 0.17 m)

Lateral gap  = step_width − 2 × lateral_extent
             = 0.20 − 2 × 0.025
             = 0.15 m
```

The CoP extents **shrink** the gap between envelopes; the previous calculation erroneously added them. The correct minimum diagonal is:

```
d_min = √(0.08² + 0.15²) = √(0.0064 + 0.0225) = √0.0289 ≈ 0.17 m
```

The outcome (0.17 m) coincidentally matches the originally quoted "forward component," but for the correct reason: it is the diagonal between closest CoP edges, not a single-axis distance.

### Corrected timing estimates

```
T_min_quintic = √(5.774 × 0.17 / 7.85) ≈ 0.35 s   (not 0.41 s)
T_min_bangbang = √(4 × 0.17 / 7.85) ≈ 0.29 s       (not 0.34 s)
```

A fixed double-support timer of 0.15 s would require ~1.7g of CoM acceleration — still physically impossible. Event-driven switching remains the correct design choice.

### Documents requiring correction

- `docs/walking_architecture.md` §"Minimum CoM shift during double support"
- `docs/adr/001-walking-gait-architecture.md` Decision #8
- `docs/review_walking_milestone_glm.md` Concern #4
- This document (corrected)

---

## Remaining Concerns

### 1. No explicit CoM velocity damping during single support

**Context:** During single support, the CoM target is a constant setpoint (support foot center), tracked with PD feedback only. The argument is that the GRF transition condition (80%·mg on support foot) guarantees the CoM is near the support foot center at phase entry, so PD convergence is sufficient.

**Problem:** PD on position error guarantees zero steady-state position error, but does **not** guarantee zero velocity at convergence. During the WEIGHT_SHIFT → SINGLE transition, the CoM has been moving toward the support foot — it may be near the center but still have residual lateral velocity from the shift. Residual CoM velocity of just 0.05 m/s produces a CP offset of 0.05/3.13 ≈ 1.6 cm, consuming ~65% of the ±2.5 cm lateral CoP margin during single support.

**Recommendation:** Add a CoM velocity target of zero during single support (e.g., a small velocity damping term in the CoM task), or use a CP-based CoM reference (`x_ref = x_support + k·ẋ_com`) that pulls the CoM ahead of the support foot proportional to forward velocity while damping lateral velocity.

### 2. Swing-leg angular momentum is acknowledged but not feedforward-compensated

**Context:** The ADR identifies swing-leg angular momentum as producing a ~2.8 cm CP shift. The mitigation is time-varying swing-xy weight (low during transit, high during descent).

**Problem:** Lowering the tracking weight only prevents the QP from **amplifying** the disturbance through servo action — it does not cancel the inherent reaction wrench on the trunk. The QP must react to the disturbance after it appears in the state feedback, introducing a delay of one control cycle (2 ms) plus PD convergence time.

**Recommendation:** Consider a centroidal momentum feedforward: pre-compute the swing leg's expected reaction wrench from its planned trajectory and offset the floating-base dynamics constraint in the QP. This is standard practice in whole-body control and would reduce the disturbance before it appears in state feedback.

### 3. Torsional friction capacity may limit yaw regulation

**Context:** The torsional friction constraint limits yaw torque to ~3.4 N·m per foot (6.8 N·m in double support). The pelvis yaw regulation task (weight 0.3) is a soft objective.

**Problem:** If torso yaw disturbance (from asymmetric foot placement, swing-leg reaction torques, etc.) exceeds the friction limit, the QP will hit the hard torsional constraint bound. The soft yaw task weight then becomes irrelevant — the constraint, not the objective, limits correction. Yaw drift will accumulate regardless of the heading correction logic.

**Recommendation:** Monitor `λ_τz` in the QP solution during testing. If it repeatedly saturates at the constraint bound, the robot cannot control yaw and the walking direction will drift. Mitigation options: (a) increase step width to enlarge the friction moment arm, (b) reduce single-support duration to limit yaw disturbance accumulation, or (c) accept drift and rely on heading-corrected footsteps (already implemented) as a feedforward compensation rather than feedback.

---

## Cross-Reference with GLM Review

The GLM review (`docs/review_walking_milestone_glm.md`) identified 7 concerns. Its recommendation of GRF-based transitions was adopted. Its concern #4 (`d_min` undercounted, 0.17 → 0.23 m) is itself incorrect — the correct diagonal is 0.17 m (see §Geometric Correction above). The GLM review's concerns #1 (tight CP window), #2 (swing momentum), #3 (contact wrench discontinuity), #5 (quintic velocity discontinuity), and #6 (70/30 bias clarification) are all addressed by the ADR revision. Concern #7 (expected walking speed) is documented.

---

## Overall Assessment

The architecture is theoretically sound for quasi-static bipedal walking given the G1's ±2.5 cm lateral CoP envelope. The GRF-based transition revision correctly resolves the fundamental mismatch between CP-based spatial conditions and the robot's tight geometric margins.

The three remaining concerns (CoM velocity damping, swing-leg feedforward, torsional saturation) are tunable implementation details rather than architectural flaws — they affect robustness and performance but do not threaten the fundamental viability of the approach.
