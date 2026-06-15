# Stage 1 Walking Debug Report

**Date:** 2026-05-05  
**Branch:** dev  
**Test:** `scripts/test_walking.py` (static alternating support, step_length = 0)

---

## Current Status

Stage 1 is **partially working**. The first full step cycle now completes cleanly:
`BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT`.

However, the robot collapses during the second weight shift (`WEIGHT_SHIFT_R`).
Step count: 1 in 10 s (needs ≥ 4).

---

## Solved Issues

| # | Issue | Fix | Verified |
|---|-------|-----|----------|
| 1 | QP "primal infeasible" during WEIGHT_SHIFT | Restored pelvis orientation task to weight-shift phases | Yes — failure moved from t=1.7 s to t=8 s |
| 2 | Trunk tilts -30° during weight shift | Added `ComPlanner` smooth CoM interpolation | Yes — pelvis stays within ±10° for 7 s |
| 3 | Weight shift too slow (5.4 s) | Added swing-foot preload (`a_lift`) during unload | Yes — weight shift now ~1.9 s |
| 4 | CoM target chases slipping foot | Fixed `self._single_support_com_target` at phase entry | Yes — CoM target no longer tracks current foot pos |
| 5 | Posture task fights leaned configuration | Snapshot `q_ref` at single-support entry | No measurable improvement |
| 6 | Transition gate measured com_err to ankle | Changed to foot CoP centre (3.5 cm forward of ankle), tightened to 0.03 m | Verified — gate fires at correct geometry |
| 7 | **Support foot slips 0.25 m during single support** | Removed torsional friction override in `_build_wrench_cones()`; boosted pelvis task (kp=200, w=50) | Yes — slip dropped from 0.25 m to ~0 |
| 8 | **`w_cam=200` blocked leg swing** (foot apex only 0.044 m, target 0.084 m) | Reduced `single_leg_w_cam: 200→20`; the CAM Jacobian was dominating the QP Hessian and fighting angular-momentum-generating joint motion | Yes — apex now 0.085–0.090 m |
| 9 | **Violent landing impulse** (870 N GRF spike, support foot bounces) | Removed Cartesian landing pull task (kp=300 from apex created −18 m/s² demand); replaced with full-duration quintic trajectory tracking | Yes — GRF_L stable ~330 N during descent |
| 10 | **xy damping task (w=500) blocked z-descent via Hessian coupling** | Removed xy damping task from landing branch in `_build_phase_tasks()` | Yes — z-descent unblocked |
| 11 | **Swing z weight too low** (w=50 → only 14 % of desired z-accel executed) | Increased `swing_weights.z: 50→200` | Yes — trajectory tracked accurately |
| 12 | **Premature weight-shift transition** (WEIGHT_SHIFT_R fired after ~0.5 s due to transient GRF spike) | Enforced `min_weight_shift_duration: 1.5 s` in `_check_weight_shift_to_single()` | Yes — weight shift now lasts ≥ 1.5 s |
| 13 | **Touchdown never fires** (foot hovers at ~38 mm, support at ~34 mm) | Changed hardcoded `z_ok < 0.003` in `_check_touchdown()` to read `self.touchdown_z_tolerance` (0.005 from config) | Yes — touchdown triggers at ~4.1 s, step_count increments |

---

## Unsolved Issues

| # | Issue | Evidence |
|---|-------|----------|
| 1 | **Catastrophic fall after first step** — during `WEIGHT_SHIFT_R` the trunk rolls to 177° and pitch reaches 87° | `max_roll_deg=177.2`, `max_pitch_deg=87.5` |
| 2 | **Massive support-foot slip (0.29 m)** — occurs after first touchdown or during second weight shift | `max_support_slip_m=0.287`; likely the support foot slides during `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` |
| 3 | **Only 1 step completed** — `RIGHT_SINGLE` is never reached | `total_steps=1`, unique states missing `RIGHT_SINGLE` |
| 4 | **Cycle time too long** — even if stable, current pacing would not fit 4 steps in 10 s | `t_weight_shift=2.5 s`, `swing_duration=1.5 s` |

---

## Key Findings

### QP Hessian coupling (root cause of multiple blocks)
A soft task with Jacobian J and weight w adds `w · Jᵀ J` to the Hessian. This
penalises **any** joint acceleration with a component along J — including the
accelerations needed by other tasks that share the same DOFs.

- `w_cam=200`, 3-row CAM Jacobian → blocks all swing-leg joint motion that produces
  angular momentum (i.e., all of it). Fix: `w_cam=20`.
- `w_xy=500` in landing branch → blocks joint motion needed for z-descent (xy and z
  Jacobian rows share leg DOFs). Fix: remove xy task during landing.

### Quintic trajectory vs post-trajectory pull
The `SwingTrajectoryPlanner` quintic profile naturally delivers zero terminal
velocity at target z, avoiding impact impulse. After `dt > swing_duration`, the
clamped planner returns constant `(target_pos, 0, 0)` → the task degrades to a
pure position pull with gain `swing_kp`. At `kp=50` this is too weak; at `kp=300`
it overshoots. Current config uses `swing_kp=120`.

### Touchdown check fragility
The original hardcoded `z_ok = |swing_z − support_z| < 3 mm` was too strict for
this model: the swing foot stabilises ~3.5–4 mm above the support foot body
centre because the swing-z PD pull after trajectory end is weak (−0.3 m/s²
commanded, ~14 % executed). Relaxing to the config tolerance (`0.005 m`)
allows touchdown to fire without risking premature contact.

### Minimum weight-shift duration is necessary
Without it, transient GRF spikes (e.g. support foot micro-bounce) can trigger
the 80 % fire threshold after < 1 s, sending the robot into single support
before the CoM has actually crossed the support polygon. This caused the
second-cycle collapse in earlier runs.

---

## Next Move Plan

1. **Diagnose the 0.29 m slip** — run a per-step logger during
   `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` to see whether the left or right foot
   slides first, and whether GRF drops to zero before or after slip begins.

2. **If slip is contact-related** (support foot loses contact during weight
   shift): increase `foot_kd` or add a small continuous-contact penalty in the
   QP inequality constraints.

3. **If slip is balance-related** (CoM shift is too aggressive for the return
   phase): reduce `t_weight_shift` or add a smaller bias for the second cycle.

4. **Once stable for 4 steps** → shorten `t_weight_shift` (2.5 s → 1.5 s) and
   `swing_duration` (1.5 s → 1.2 s) to fit the required 4 cycles in 10 s.

5. **Cleanup (after Stage 1 passes)**:
   - Delete `/tmp/diag_*.py`
   - Disable OSQP verbose diagnostics in `base_qp_wbc.py` (~line 307)
   - Remove unused config keys: `landing_kp`, `landing_kd`, `landing_kp_cart`,
     `landing_kd_cart`, `landing_cart_w`, `swing_lift_duration`
