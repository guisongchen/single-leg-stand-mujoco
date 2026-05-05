# Review: Walking Milestone Architecture

**Documents reviewed:** `docs/walking_milestone_task.md`, `docs/adr/001-walking-gait-architecture.md`  
**Review scope:** Theoretical and physical correctness only (code not examined)

---

## Summary

8 pitfalls identified, ranked by severity. The first 3 are likely to cause phase-transition failure or indefinite hangs. The architecture is fundamentally sound but has a timing mismatch between minimum guards and dynamic CP entry windows, one logically impossible safety check, and a missing contact constraint.

---

## Findings

### #1 — CRITICAL: Minimum guard (0.05 s) may outlast the CP entry window

**Document:** milestone §"Transition rules", ADR Decision #2

The WEIGHT_SHIFT → SINGLE transition gates on `min_double_guard = 0.05 s` (25 timesteps) elapsed from *phase start*. The CP enters the ±1.5 cm lateral polygon window during the PD transient as the CoM accelerates toward the support foot. With typical convergence velocities, the CP traverses a 3 cm polygon in approximately 0.03–0.04 s.

If the CP enters at t = 0.03 s, the guard blocks the transition. By t = 0.05 s, the CP may have already overshot and exited the polygon. Recovery requires the CoM to reverse direction (PD overshoot reversal), which is unreliable.

**Recommendation:** Replace fixed-from-start guard with a **post-entry latch**: once CP enters the polygon AND lateral velocity is converging, arm the transition for 0.1 s. If CP exits during the latch window, reset and re-arm on re-entry. Alternatively, count consecutive in-polygon timesteps (e.g., 10 = 0.02 s) after first entry.

---

### #2 — HIGH: Biased double-support CoM target lies outside the next support foot's CoP polygon

**Document:** ADR Decision #1, milestone §3

With `step_length = 0.25 m` and `step_width = 0.20 m`, a 70/30 biased midpoint falls outside the next foot's polygon in both x and y:

- Right foot polygon: x ∈ [0.20, 0.37], y ∈ [-0.125, -0.075]
- 70/30 biased midpoint: x = 0.175, y = -0.04 — **x is 0.025 m behind, y is 0.035 m lateral of the polygon**

At steady state (v ≈ 0), CP ≈ CoM, so CP is outside. The DOUBLE_SUPPORT → WEIGHT_SHIFT transition therefore relies on the CP dynamically sweeping through the polygon during the PD transient, which can be missed between timesteps.

**Recommendation:** Make DOUBLE_SUPPORT → WEIGHT_SHIFT time-driven (0.15–0.20 s fixed). The real stability gate is WEIGHT_SHIFT → SINGLE, where the CoM has reached the support foot center. No spatial condition is needed at this stage — only confirmation that motion has started.

---

### #3 — HIGH: Safety abort mechanism is logically impossible

**Document:** milestone §2 "Safety abort"

> "If the capture point at the end of double-support lies outside the upcoming support foot's CoP polygon by more than 0.05 m despite the event condition, abort the step..."

The event condition for exiting DOUBLE_SUPPORT already requires CP inside the polygon (±0.015 m with the 0.01 m margin). If the event condition fires, CP is guaranteed to be within 0.015 m, which is well under the 0.05 m abort threshold. The abort can never trigger.

**Recommendation:** Either (a) remove the abort (the 5.0 s phase timeout already handles stuck transitions), or (b) redefine it as "CP is > 0.05 m beyond the **union** of both foot polygons during any two-foot phase."

---

### #4 — MEDIUM: Missing torsional friction constraint for 6-D contacts

**Document:** ADR Decision #4 (6-D contacts), milestone §1 "CoP envelope"

The QP's wrench cones impose friction pyramid (`|fx|, |fy| ≤ μ·fz`) and CoP bounds, but no constraint on **yaw torque** `τ_z`. For a 17 cm × 5 cm foot at μ = 0.8 and F_z ≈ 170 N, the torsional friction limit is:

```
τ_z_max ≈ μ · F_z · min(width, length) / 2 ≈ 0.8 × 170 × 0.025 ≈ 3.4 N·m
```

Without `|τ_z| ≤ γ·fz` in the QP, the optimizer may request physically impossible yaw torques. MuJoCo's contact solver will then produce constraint forces that diverge from the QP's expectation.

**Recommendation:** Add two linear inequalities per foot in `_build_wrench_cones()`:
```
|λ_τz| ≤ (μ · min(width, length) / 2) · λ_fz
```

---

### #5 — MEDIUM: Initial pose instability may prevent surviving BIPEDAL_INIT

**Document:** milestone §1 "Transition rules", CLAUDE.md "Key Findings"

CLAUDE.md confirms the initial reference pose (knee=0.15, ankle_pitch=-0.08 rad) puts the CoM ~5.7 cm forward of the foot midpoint, causing "immediate forward tip-over." The BIPEDAL_INIT phase holds for 1.0 s before transitioning — but the robot must survive that first second.

**Recommendation:** Reduce `init_duration` from 1.0 s to 0.1 s. The QP-WBC engages from the first timestep and actively stabilizes; a long settle period before the controller activates is counterproductive when the base pose is unstable. Alternatively, compute a statically stable bipedal pose with the CoM inside the bipedal support polygon (x ∈ [-0.05, +0.37]).

---

### #6 — LOW: step_width formula is ambiguous / incorrect

**Document:** milestone §2 "Footstep Planner" line 123

The spec says the swing foot target uses `±step_width/2` relative to the support foot. But both feet are already offset by `±step_width/2` from the pelvis midline. So `support_foot_y + step_width/2` puts the next foot at the midline, not at its proper lateral offset.

**Recommendation:** Compute foot positions independently of the support foot, using absolute lateral positions:
```python
left_foot_y  = +step_width / 2
right_foot_y = -step_width / 2
```
Target for a left swing: right_foot_y. Target for a right swing: left_foot_y. This avoids sign-tracking errors entirely.

---

### #7 — LOW: Swing foot drift during low-weight transit may exceed correction window

**Document:** ADR Decision #5, milestone §4

Low xy weight (0.1) during 60% of swing allows the foot to drift from its planned trajectory. During the remaining 40% (≈ 0.2 s for a 0.5 s swing), the foot must correct drift and decelerate into the target 0.02 m tolerance. If lateral drift exceeds ~2 cm, correction within 0.2 s requires > 1.0 m/s² lateral acceleration, which may couple reaction forces into the trunk.

**Recommendation:** Track the **full planned swing trajectory** (position, velocity from the SwingFootPlanner's quintic) as a soft task throughout the entire swing, at a moderate constant weight (0.5). The feedforward handles bulk acceleration; the QP only corrects deviations continuously, rather than ramping up late.

---

### #8 — LOW: Quintic may overshoot after WEIGHT_SHIFT convergence

**Document:** ADR Decision #6, milestone §3 "During LEFT_SINGLE / RIGHT_SINGLE"

If WEIGHT_SHIFT already converged the CoM to the support foot center, applying a quintic smoothstep from current position to the same target injects negligible useful feedforward but adds commanded acceleration that can cause overshoot.

**Recommendation:** At SINGLE phase entry, if `‖CoM_current − CoM_target‖ < 0.02 m`, skip the ComPlanner quintic and use PD-only setpoint hold. The quintic serves as a safety net when WEIGHT_SHIFT didn't fully converge, not as a mandatory trajectory.

---

## Recommended Fix Priority

| Priority | Issue | Action |
|----------|-------|--------|
| 1 | #1 — Min guard outlasts CP window | Post-entry latch or consecutive-timestep count |
| 2 | #2 — Biased midpoint outside polygon | Time-driven DS → WS transition |
| 3 | #3 — Impossible safety abort | Remove or redefine as union-of-polygons check |
| 4 | #4 — Missing τ_z constraint | Add linear inequality per foot |
| 5 | #5 — Unstable initial pose | Shorten INIT to 0.1 s |

Issues #6–#8 are documentation-level corrections or minor tuning improvements. They won't cause catastrophic failure but fixing them removes fragility and ambiguity.
