# Phase 2 Debug Status Report — 2026-05-04

## Current Status

The automatic state-machine loop `BIPEDAL → WEIGHT_SHIFT → SINGLE_LEG → BIPEDAL_RETURN → BIPEDAL` now achieves **physical touchdown** but crashes ~1.8 s later with **OSQP primal infeasible** (t ≈ 10.28 s).

## Solved Issues

1. **QP primal infeasible during hard two-foot constraint** — Fixed by splitting `BIPEDAL_RETURN` into pre-touchdown (support foot hard only) and post-touchdown (both feet hard) modes.
2. **`q_ref` fighting descent** — Fixed by restoring `q_ref = q_ref_bipedal.copy()` in `_setup_bipedal_return()` so the posture task helps retraction instead of holding the single-leg pose.
3. **Forced landing feedback causing oscillation** — Reverted `return_land_kp` position feedback; returned to velocity damping after touchdown.
4. **CoM target not reaching QP in BIPEDAL_RETURN** — `com_target` was reassigned inside the state block *after* `_compute_task_targets` had already consumed the old value (support-foot CoM). Moved the planner evaluation to the early state-based block so the QP sees the shifted target.

## Unsolved Issues

### 1. Swing foot drifts above planner target during `SINGLE_LEG`
- **Evidence**: With `swing_lift_height = 0.015`, actual swing foot reaches ~0.063 m.
- **Mechanism**: `w_swing` is not boosted during `SINGLE_LEG` (stays at config default 10.0). The weak soft objective cannot fully counteract upward momentum.
- **Status**: Mitigated by anticipatory CoM shift; foot still touches down but descent distance is larger than necessary.

### 2. Crash ~1.8 s after touchdown in `BIPEDAL_RETURN`
- **Evidence**: Touchdown at t=8.45 s (fz=137.7 N). OSQP primal infeasible at t=10.28 s.
- **Metrics at crash**: max_tau spikes >150 Nm, support slip >0.05 m, pelvis roll/pitch exceed fall thresholds.
- **Mechanism (hypothesis)**: After touchdown the controller switches to:
  - Both feet hard constraints (6-D each)
  - CoM continuing to shift toward midpoint
  - Posture task restored to bipedal reference
  These simultaneous hard constraints may conflict when the feet are not yet in the exact bipedal geometry. The swing foot may be at a different XY position than `q_ref_bipedal` expects, making the fixed-foot kinematics incompatible with the joint-space posture target.
- **Consequence**: Robot destabilizes, joints spike, QP reports infeasible.

## Next Move Plan

| Priority | Fix | Expected Effect |
|----------|-----|-----------------|
| **1** | **Extend soft-landing settle phase** — keep swing foot as soft impedance (not hard constraint) for longer after touchdown, or until `|swing_fz|` is sustained and foot XY is near the bipedal reference. | Prevents premature hard constraint that conflicts with posture. |
| **2** | **Add XY swing tracking during BIPEDAL_RETURN descent** — currently only Z is tracked. Guiding XY toward the initial foot placement removes the geometric mismatch when the hard constraint finally engages. | Reduces the jump between soft and hard foot constraints. |
| **3** | **Boost `w_swing` during `SINGLE_LEG`** (add `single_leg_w_swing ≈ 100–200`) so the foot tracks the planner and does not drift above ~0.05 m. | Reduces descent distance and landing impact. |
| **4** | **Investigate whether the restored `q_ref_bipedal` posture is kinematically compatible** with the current foot positions after single-leg. If feet have drifted, the bipedal reference may be infeasible. | Fix root cause of infeasibility. |
| **5** | Re-run `test_single_to_double.py` and verify all 9 checks pass. | Confirm the loop is stable end-to-end. |
