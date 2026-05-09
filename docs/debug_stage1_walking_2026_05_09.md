# Stage 1 Walking Debug Report

**Date:** 2026-05-09  
**Branch:** feat/walking  
**Test:** `scripts/test_walking.py` (static alternating support, `step_length = 0`)  
**Controller:** `controllers/walking_controller.py` (QP-WBC, 5-phase FSM)  
**Config:** `configs/g1_config.yaml`  

---

## Current Status

**Overall: FAIL**

The robot completes the first single-support phase (`LEFT_SINGLE`) but collapses during the return weight shift (`WEIGHT_SHIFT_R`).  Only 1 step is completed in 10 s (needs >= 4).  The root cause is CoM drift during `LEFT_SINGLE` that places the robot outside the support-foot CoP envelope before double support can stabilise it.

### Test Metrics

| Metric | Result | Value |
|--------|--------|-------|
| no_fall | FAIL | max roll/pitch exceed 15 deg |
| left_foot_clearance | FAIL | 0 m (never reaches `RIGHT_SINGLE`) |
| right_foot_clearance | PASS | ~30 mm during `LEFT_SINGLE` |
| support_slip | FAIL | > 5 mm |
| min_steps | FAIL | 1 step (needs >= 4) |
| states_present | PASS | 5 unique states seen |
| grf_hysteresis | PASS | support GRF > 80 % mg at transition |

---

## Solved Issues (All Sessions)

### Architecture & QP Stability

| # | Issue | Fix | Evidence |
|---|-------|-----|----------|
| 1 | QP "primal infeasible" during `WEIGHT_SHIFT` | Restored pelvis orientation task to weight-shift phases | Failure moved from t=1.7 s to t=8 s |
| 2 | Trunk tilts -30 deg during weight shift | Added `ComPlanner` smooth CoM interpolation | Pelvis stays within +/-10 deg for 7 s |
| 3 | Violent landing impulse (870 N GRF spike) | Replaced Cartesian landing pull with full-duration quintic trajectory | GRF_L stable ~330 N during descent |
| 4 | XY damping task (w=500) blocked z-descent via Hessian coupling | Removed XY damping task from landing branch | Z-descent unblocked |
| 5 | Swing z weight too low (w=50 -> 14 % execution) | Increased `swing_weights.z: 50 -> 200` | Trajectory tracked accurately |
| 6 | `w_cam=200` blocked leg swing (apex 44 mm vs 84 mm target) | Reduced `single_leg_w_cam: 200 -> 20` | Apex now 85-90 mm |
| 7 | Post-trajectory landing stall | Added `swing_land_accel` constant downward push after `dt > swing_duration` | Foot settles to ground instead of hovering |
| 8 | OSQP verbose diagnostics cluttering output | Disabled verbose mode in `base_qp_wbc.py` | Clean console output |

### Weight-Shift Phase

| # | Issue | Fix | Evidence |
|---|-------|-----|----------|
| 9 | Weight shift too slow (5.4 s) | Added swing-foot preload (`a_lift`) during unload | Weight shift now ~1.9 s |
| 10 | Premature weight-shift transition (fired at ~0.5 s) | Enforced `min_weight_shift_duration: 1.5 s` | Shift lasts >= 1.5 s |
| 11 | Transition gate measured `com_err` to ankle joint | Changed to foot CoP centre (3.5 cm forward of ankle), tightened to 0.03 m | Gate fires at correct geometry |
| 12 | CoM target chases slipping foot during single support | Fixed `_single_support_com_target` at phase entry | CoM target no longer tracks current foot pos |

### Support Foot & Contact

| # | Issue | Fix | Evidence |
|---|-------|-----|----------|
| 13 | Support foot slips 0.25 m during single support | Removed torsional friction override in `_build_wrench_cones()`; boosted pelvis task (kp=200, kd=20, w=50) | Slip dropped to ~0 |
| 14 | Touchdown never fired (foot hovered at ~38 mm) | Changed hardcoded `z_ok < 0.003` to read `touchdown_z_tolerance` (0.005 m from config) | Touchdown triggers at ~4.1 s in earlier runs |
| 15 | Strong PD anchors on support foot over-constrained balance | Replaced PD anchor with pure velocity damping (`-foot_kd * vel`) | Support foot no longer fights balance corrections |
| 16 | Landing chase code coupled swing leg to body balance | Removed swing-foot XY target update during landing | Reduced reaction wrench feedback into body |

### Swing Foot Trajectory

| # | Issue | Fix | Evidence |
|---|-------|-----|----------|
| 17 | Swing XY tracking caused CoM drift and oscillation | Disabled XY tracking when `step_length = 0` (Z-only tracking) | CoM drift reduced but not eliminated |
| 18 | Touchdown forced to wait full `swing_duration` even after ground contact | Relaxed minimum to `min_single_duration` (0.5 s) | Earlier opportunity to enter double support |
| 19 | Double support anchor missing | Added `_double_support_left_anchor` / `_double_support_right_anchor` snapshot at entry | Feet held at touchdown positions |
| 20 | Step height and air time too large | Reduced `step_height: 0.05 -> 0.03`, `swing_duration: 1.5 -> 0.6`, `swing_lift_duration: 0.9 -> 0.36` | Air time reduced to ~0.5 s |

---

## Unsolved Issues

### Issue 1: CoM Overshoot During Single Support (CRITICAL)

**Symptom:** During `LEFT_SINGLE` the CoM y drifts from 0.091 to 0.155, overshooting the support foot CoP centre (y=0.119).  By the time touchdown fires, the CoM is already outside the left foot's CoP envelope.

**Evidence from `trace_states.py`:**

```
t=2.202 state=LEFT_SINGLE     left=(+0.019,+0.118,0.034) right=(-0.014,-0.123,0.037) com=(+0.052,+0.091,0.707)
t=2.502 state=LEFT_SINGLE     left=(+0.019,+0.118,0.034) right=(-0.005,-0.155,0.068) com=(+0.054,+0.103,0.707)
t=2.902 state=LEFT_SINGLE     left=(+0.019,+0.119,0.034) right=(+0.082,-0.121,0.035) com=(+0.054,+0.125,0.699)
t=3.002 state=LEFT_SINGLE     left=(+0.019,+0.119,0.036) right=(+0.081,-0.120,0.035) com=(+0.054,+0.137,0.694)
t=3.102 state=DOUBLE_SUPPORT  left=(+0.019,+0.119,0.035) right=(+0.081,-0.120,0.035) com=(+0.054,+0.155,0.688)
```

**Root-cause analysis:**

1. **Touchdown fires too late.**  The swing foot makes contact around t=2.9 s (`grf_R=49.4 N`) but the touchdown check requires `z_ok AND vz_ok` for 0.1 s (50 steps).  Touchdown does not fire until t=3.1 s, giving the CoM an extra ~0.2 s to drift left.

2. **No XY swing constraint.**  With `step_length=0` we disabled XY tracking to avoid coupling the swing leg to balance.  However, the swing foot drifts from x=-0.014 to x=+0.082 during the swing.  This pendulum-like motion generates reaction forces that pull the CoM laterally.

3. **Double support uses strong PD anchors.**  After touchdown, `DOUBLE_SUPPORT` fixes both feet with `kp=200, kd=20`.  When the CoM enters double support at y=0.155 (already outside the left foot envelope), the rigid foot anchors prevent the robot from shifting its base to recover balance.  The CoM continues to y=0.175, 0.201, 0.237...

4. **CoM target in double support biases toward next support.**  `double_support_com_bias=0.70` pushes the CoM toward the right foot during the brief double-support phase.  When the CoM is already too far left, this bias accelerates the fall rather than correcting it.

### Issue 2: Swing Foot Horizontal Drift

**Symptom:** During `LEFT_SINGLE` the right foot drifts from x=-0.014 to x=+0.082 even though `step_length=0` should produce no forward motion.

**Evidence:** See trace above.  The drift occurs primarily during descent (t=2.5-2.9 s).

**Hypothesis:** The `SwingTrajectoryPlanner` computes a linear XY interpolation from `start` to `target`.  Although `step_length=0`, `FootstepPlanner.plan_step()` may still return a target that differs from the start because the support foot has moved during weight shift.  Since XY tracking is disabled, the foot is not pulled to the target; instead, the free leg swings like a pendulum under gravity and body motion, and the initial preload (`a_lift`) during weight shift may have imparted horizontal velocity.

### Issue 3: GRF Touchdown Detection Gap

**Symptom:** The swing foot makes contact (`grf_R=49.4 N` at t=2.9 s, `69.0 N` at t=3.0 s) but the state machine remains in `LEFT_SINGLE` for another 0.1-0.2 s because the kinematic touchdown check has not yet satisfied the 0.1 s settling timer.

**Consequence:** Unmodelled contact forces push the robot while the controller is still configured for single support (one hard foot, one soft foot).  The QP does not account for the unexpected ground reaction on the "swing" foot, leading to incorrect torque distribution.

### Issue 4: Cycle Time Too Long for 4 Steps in 10 s

**Symptom:** Even if stable, the first cycle takes ~3.1 s.  At this pace, 4 steps would require ~12.4 s, exceeding the 10 s test duration.

**Contributing factors:**
- `t_weight_shift=2.5 s` (config) / actual ~1.9-2.2 s
- `swing_duration=0.6 s` + touchdown settling ~0.1 s
- `double_support_duration=0.5 s` (test script override)
- Recovery/settling time after touchdown

---

## Key Findings

### Pendulum Effect of Free Swing Leg
Disabling XY swing tracking eliminates the QP fight but allows the swing leg to act as a double pendulum.  During descent the leg straightens and swings forward, creating reaction forces that pull the CoM left.  The transition controller avoids this by tracking Z-only with a very short air time (~0.3 s rise, immediate descent).  The walking controller's quintic trajectory still has a 40 % rise, 20 % hold, 40 % descent profile that keeps the foot airborne for ~0.36 s at minimum.

### Touchdown Settling Timer is the Bottleneck
The 0.1 s `touchdown_timer` (requiring `z_ok AND vz_ok` for 50 consecutive steps) is the primary reason the robot stays in single support after ground contact.  During this window the CoM drifts an additional ~3-5 cm, which is enough to place it outside the support polygon.

### Double Support PD Anchors Prevent Recovery
Using `kp=200` anchors on both feet in double support prevents the foot from sliding to a new equilibrium when the CoM arrives off-centre.  The transition controller's approach of velocity-damping only allows the feet to find a natural stance.

### QP Hessian Coupling Still Matters
Even with XY tracking disabled, the swing Z task (J_swing[2:3], w=200) and the support foot hard constraint (J_support, equality) share leg-joint DOFs.  The QP must balance these simultaneously; if the swing foot is disturbed by contact, the support foot constraint forces the body to absorb the impulse, causing CoM drift.

---

## Next Plan

### Immediate fixes (expected to stabilise first cycle)

1. **Add GRF-based early touchdown.**  When `grf_swing > touchdown_threshold` (e.g. 10-20 N), bypass the kinematic settling timer and switch to `DOUBLE_SUPPORT` immediately.  This eliminates the 0.1-0.2 s drift window.

2. **Replace double-support PD anchors with velocity damping.**  Use the same `foot_kd` approach as `BIPEDAL_INIT` and the transition controller.  Allow the feet to slide slightly so the robot can find a natural stance after an imperfect landing.

3. **Reduce double-support duration.**  The test script overrides to 0.5 s; the config has 0.30 s.  Use 0.15-0.20 s to minimise time spent with rigid feet while the CoM is off-centre.

4. **Add CoM velocity damping during single support.**  Reduce `kd_com` or add a lateral velocity penalty to the CoM task to prevent overshoot:
   ```python
   com_accel_des = kp_com * (com_target - com_pos) - kd_com * com_vel
   ```
   Currently `kd_com=20` may be too low to damp the pendulum-like oscillation.

### Medium-term fixes (needed for 4 steps in 10 s)

5. **Shorten weight-shift duration.**  Reduce `t_weight_shift` from 2.5 s to 1.0-1.2 s.  The preload (`a_lift`) already accelerates unloading; the CoM planner can use a faster interpolation.

6. **Use `SwingFootPlanner` (Z-only) for Stage 1.**  `SwingTrajectoryPlanner` always interpolates XY, which is unnecessary when `step_length=0`.  `SwingFootPlanner` holds XY constant and only moves Z, eliminating the pendulum drift entirely.

7. **Reduce `step_height` further.**  From 0.03 m to 0.02 m or even 0.015 m.  Less clearance means less air time and less opportunity for drift.

8. **Consider a small continuous XY correction on the swing foot.**  Instead of full trajectory tracking, apply a weak spring (`kp=20, w=1`) that pulls the swing foot back to its lift-off XY position.  This prevents drift without strong coupling to balance.

### Validation protocol

After each change, run in this order:
1. `scripts/trace_states.py` -> verify no_fall for 10 s and check CoM drift magnitude
2. `scripts/trace_detailed.py` -> verify swing foot trajectory and touchdown timing
3. `scripts/test_walking.py` -> verify all 7 metrics pass

Do **not** tune parameters until the root cause (touchdown delay + double-support rigidity) is confirmed fixed by the trace scripts.
