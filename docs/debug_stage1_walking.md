# Stage 1 Walking Debug Report

**Date:** 2026-05-05  
**Branch:** feat/walking  
**Test:** `scripts/test_walking.py` (static alternating support, step_length = 0)

---

## Current Status

Stage 1 is still failing. The swing foot now lifts and tracks the quintic trajectory
correctly (apex ≈ 0.09 m, GRF_L stable ~330 N during swing), but landing is not yet
clean: `swing_kp=50` descends too slowly (~30 mm/s after trajectory ends), while
`swing_kp=300` overshoots and bounces both feet off the ground. Step count: 1 (needs 4).

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

---

## Unsolved Issues

| # | Issue | Evidence |
|---|-------|----------|
| 1 | **Landing too slow with `swing_kp=50`** — after trajectory ends (dt > 2.0 s) the clamped planner returns zero vel/accel; pull is only `kp*(0.034−z_foot)` ≈ −0.8 m/s², QP executes ~14% → ~30 mm/s descent | Foot still 14 mm above ground at t = 6.45 s |
| 2 | **Landing bounce with `swing_kp=300`** — overshoot sends impulse through kinematic chain; support foot loses contact (GRF_L → 0 at t = 5.85 s), both feet airborne by t = 6.1 s | `diag_ftz.py` output: LFz = 0.076, RFz = 0.190 at t = 5.85 s |
| 3 | **Full Stage 1 test not yet passed** — step_count = 1, needs ≥ 4 in 10 s | Blocked by issues 1 & 2 |
| 4 | **Cycle time too long** — current cycle ≈ 5 s/step; need ≤ 2.5 s/step for 4 steps in 10 s | Not yet investigated; `t_weight_shift=2.5 s`, `swing_duration=2.0 s` |

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
The `SwingTrajectoryPlanner` quintic profile (rise 0–0.8 s, hold 0.8–1.2 s, descent
1.2–2.0 s) naturally delivers zero terminal velocity at target z, avoiding impact
impulse. After `dt > swing_duration`, the clamped planner returns constant
`(target_pos, 0, 0)` → the task degrades to a pure position pull with gain `swing_kp`.
At kp=50 this is too weak; at kp=300 it overshoots.

### Touchdown check fragility
`z_ok = |swing_z − support_z| < 15 mm` checks **relative** height, not absolute
ground clearance. When the support foot drifts up a few mm, `z_ok` fires prematurely.
This caused a cascade: early touchdown → DOUBLE_SUPPORT with both feet airborne →
WEIGHT_SHIFT with no real contact → uncontrolled pitch build-up (5°→19° in 0.4 s).

---

## Next Move Plan

1. **Tune `swing_kp` to 100–150** and re-run `diag_ftz.py`.  
   Target: foot reaches z ≈ 0.034 m with GRF_R > 33 N, no bounce on support foot.

2. **If bounce persists**, cap commanded z-acceleration to ≤ 3 m/s² in the swing task
   regardless of `swing_kp` (velocity-limited landing).

3. Once clean touchdown confirmed → **run `scripts/test_walking.py`** for full 10 s eval.

4. If step_count < 4 → shorten `t_weight_shift` (2.5 s → 1.5 s) and `swing_duration`
   (2.0 s → 1.5 s) to fit 4 cycles in 10 s.

5. **Cleanup (after Stage 1 passes)**:
   - Delete `/tmp/diag_*.py`
   - Disable OSQP verbose diagnostics in `base_qp_wbc.py` (~line 307)
   - Remove unused config keys: `landing_kp`, `landing_kd`, `landing_kp_cart`,
     `landing_kd_cart`, `landing_cart_w`, `swing_lift_duration`
