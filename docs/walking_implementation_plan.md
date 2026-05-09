# Walking Milestone — Implementation Plan

## Branch
`feat/walking`

## Overview

Build a periodic walking gait for G1 on flat ground, reusing the existing QP-WBC infrastructure. The deliverable is a `WalkingController` that cycles through a 5-phase FSM with GRF-based transitions, using the architecture defined in `docs/walking_architecture.md` and `docs/adr/001-walking-gait-architecture.md`.

The plan is staged: infrastructure → static alternating support → in-place stepping → small forward steps → full 5 m walk.

---

## Reference Architecture Summary

**5-phase FSM:**
```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

**Key design decisions:**
- GRF-based transitions with 50% arm / 80% fire hysteresis (no CP geometry for routine transitions)
- 6-D contacts (position + orientation) during all double-foot phases
- Torsional friction constraint: `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` per foot
- CP used only as coarse safety abort (> 0.05 m outside both-foot convex hull)
- PD-only CoM setpoint during single support (no quintic smoothstep)
- Time-varying swing-xy weight: 0.1 during transit, 1.0 near touchdown
- Heading-corrected footsteps (rotated by pelvis yaw) with absolute lateral positions
- Pelvis orientation regulation task (roll, pitch, yaw) in the QP
- Lifted-foot GRF < 5 N precondition before entering single support
- 5.0 s phase timeout as graceful degradation fallback

**Expected gait:** quasi-static, stop-and-go, ~0.25 m/s (~20 s for 5 m).

---

## Stage 0: New Files and Infrastructure

### 0.1 `planners/footstep_planner.py`

**Responsibility:** Generate swing foot targets in world frame with heading correction.

**Algorithm:**
```python
def plan_step(support_foot_pos, pelvis_yaw, step_length, step_width, is_right_swing):
    # Absolute lateral positions (not offset from support foot)
    left_foot_y = +step_width / 2
    right_foot_y = -step_width / 2

    target_y = left_foot_y if is_right_swing else right_foot_y
    step_dir = np.array([cos(pelvis_yaw), sin(pelvis_yaw), 0.0])

    if is_right_swing:
        step_dir = np.array([cos(pelvis_yaw), sin(pelvis_yaw), 0.0])
    else:
        step_dir = np.array([cos(pelvis_yaw), sin(pelvis_yaw), 0.0])

    target_x = support_foot_pos[0] + step_dir[0] * step_length
    target_y = right_foot_y if is_right_swing else left_foot_y
    return np.array([target_x, target_y, ground_z])
```

**Config additions** (see full config at end of doc):
```yaml
walking:
  step_length: 0.25
  step_width: 0.20
  step_height: 0.05
  min_single_duration: 0.50
  init_duration: 0.1
  double_support_duration: 0.15
  phase_timeout: 5.0
  forward_direction: [1.0, 0.0]
  double_support_com_bias: 0.70
```

### 0.2 `controllers/walking_controller.py` (skeleton)

Inherit from `QPWBCController`. Override:

- `reset()`: initialise phase = `BIPEDAL_INIT`, timers, footstep schedule, GRF arming state
- `compute()`: call `_update_gait_phase()` → build `active_feet` / `com_target` / `swing_task` / `pelvis_orientation_target` → call `_solve_qp()`

Required helper methods (stub for now, implement in later stages):
- `_update_gait_phase()`: evaluate all 7 transition conditions, advance FSM
- `_compute_com_target()`: return setpoint based on current phase
- `_compute_swing_weights()`: return (xy_weight, z_weight) based on swing progress
- `_compute_pelvis_orientation_target()`: return target roll/pitch/yaw
- `_compute_footstep_target()`: heading-corrected target from footstep planner
- `_compute_grf(foot_name)`: sum vertical contact forces on foot geoms via `mj_contactForce()`
- `_check_weight_shift_to_single()`: GRF hysteresis logic (arm 50% → fire 80% with delay)
- `_check_touchdown()`: swing foot z + xy proximity + GRF > 10%·mg
- `_compute_cp()`: capture point for safety abort only

### 0.3 Config additions

Full walking section (synced with `docs/walking_architecture.md` §Configuration Parameters):
```yaml
walking:
  step_length: 0.25
  step_width: 0.20
  step_height: 0.05
  min_single_duration: 0.50
  init_duration: 0.1
  double_support_duration: 0.15
  phase_timeout: 5.0
  forward_direction: [1.0, 0.0]
  double_support_com_bias: 0.70

transitions:
  grf_arm_threshold: 0.50
  grf_fire_threshold: 0.80
  grf_arm_to_fire_delay: 0.05
  grf_touchdown_threshold: 0.10
  grf_liftoff_threshold: 5.0
  cp_abort_margin: 0.05

safety:
  touchdown_xy_tolerance: 0.02
  touchdown_z_tolerance: 0.005

swing_weights:
  xy_early: 0.10
  xy_late: 1.0
  z: 1.0

pelvis_orientation:
  roll_weight: 0.5
  pitch_weight: 0.5
  yaw_weight: 0.3
  target_yaw: 0.0
```

---

## Stage 1: Static Alternating Support (step_length = 0)

**Goal:** Verify the 5-phase FSM with GRF transitions can lift and place each foot without falling.

**Scope:** No forward motion. Feet lift/place in place.

### 1.1 Gait state machine

Implement the full 5-phase FSM with all 7 transition rules:

| Transition | Condition |
|------------|-----------|
| `BIPEDAL_INIT → WEIGHT_SHIFT_L` | Time: 0.1 s elapsed |
| `WEIGHT_SHIFT_L → LEFT_SINGLE` | GRF on left > 80%·mg AND 0.05 s since GRF > 50%·mg AND right GRF < 5 N |
| `LEFT_SINGLE → DOUBLE_SUPPORT` | Right foot z < touchdown_z AND xy within 0.02 m of target AND right GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` | Time: 0.15 s elapsed |
| `WEIGHT_SHIFT_R → RIGHT_SINGLE` | GRF on right > 80%·mg AND 0.05 s since GRF > 50%·mg AND left GRF < 5 N |
| `RIGHT_SINGLE → DOUBLE_SUPPORT` | Left foot z < touchdown_z AND xy within 0.02 m of target AND left GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_L` | Time: 0.15 s elapsed |

**GRF hysteresis implementation:**
```python
def _check_weight_shift_to_single(self, support_foot, lift_foot):
    grf_support = self._compute_grf(support_foot)
    grf_lift = self._compute_grf(lift_foot)
    mg = self.model.total_mass * 9.81

    # Arm when support foot GRF exceeds 50% mg
    if grf_support > self.grf_arm_threshold * mg:
        if not self._armed:
            self._arm_time = self.time
            self._armed = True

    # Fire when armed long enough AND above 80% AND lift foot near zero
    if self._armed and (self.time - self._arm_time > self.grf_arm_to_fire_delay):
        if grf_support > self.grf_fire_threshold * mg and grf_lift < self.grf_liftoff_threshold:
            self._armed = False
            return True
    return False
```

**Active feet and tasks per phase:**

| Phase | active_feet | CoM target | Swing task | Pelvis orient |
|-------|-------------|------------|------------|---------------|
| `BIPEDAL_INIT` | Both (6-D) | Midpoint | None | None |
| `WEIGHT_SHIFT_L` | Both (6-D) | Left foot center | None | Roll/pitch/yaw |
| `LEFT_SINGLE` | Left only (6-D) | Left foot center | Right foot z+xy | Roll/pitch/yaw |
| `DOUBLE_SUPPORT` | Both (6-D) | 70/30 biased toward next support | None | Roll/pitch/yaw |
| `WEIGHT_SHIFT_R` | Both (6-D) | Right foot center | None | Roll/pitch/yaw |
| `RIGHT_SINGLE` | Right only (6-D) | Right foot center | Left foot z+xy | Roll/pitch/yaw |

### 1.2 6-D contacts with torsional friction

All double-foot phases (INIT, WEIGHT_SHIFT, DOUBLE_SUPPORT) use 6-D contact constraints (position + orientation). Add to `_build_wrench_cones()`:
```python
# Torsional friction per foot
tau_z_max = (mu * min(foot_width, foot_length) / 2.0) * lambda_fz
A_cones.extend([...])  # |λ_τz| ≤ tau_z_max
```

### 1.3 Time-varying swing weights

```python
def _compute_swing_weights(self, phase_progress):
    if phase_progress < 0.6:
        xy_weight = self.swing_xy_early   # 0.1
    else:
        xy_weight = self.swing_xy_early + (self.swing_xy_late - self.swing_xy_early) * ((phase_progress - 0.6) / 0.4)
    return xy_weight, self.swing_z_weight  # (xy_weight, 1.0)
```

### 1.4 Test script

`scripts/test_walking.py` Stage 1 variant:
- Run for 10 seconds (4–6 alternating lift/place cycles)
- Step length = 0
- Log: phase, GRF per foot, foot heights, pelvis tilt, transition times
- Pass if: no falls, both feet alternate cleanly, GRF hysteresis fires reliably

---

## Stage 2: In-Place Stepping with Event-Driven Transitions (step_length = 0)

**Goal:** Verify event-driven transitions work without timers; confirm GRF measurements are reliable.

### 2.1 Touchdown detection (full condition)

Already implemented in Stage 1. Verify:
```python
def _check_touchdown(self, swing_foot_name, target_xy):
    """SINGLE → DOUBLE_SUPPORT: three-condition check."""
    # 1. Foot near ground
    z_ok = abs(self._foot_z(swing_foot_name) - self._support_foot_z()) < self.touchdown_z_tol
    # 2. Foot near target xy
    xy_ok = np.linalg.norm(self._foot_xy(swing_foot_name) - target_xy) < self.touchdown_xy_tol
    # 3. GRF confirms physical contact
    grf_ok = self._compute_grf(swing_foot_name) > self.grf_touchdown_threshold * mg
    return z_ok and xy_ok and grf_ok
```

### 2.2 Contact wrench continuity at lift-off

The `WEIGHT_SHIFT → SINGLE` transition already requires lifting foot GRF < 5 N. Verify this prevents the ~1 cm CP perturbation. Log GRF on the lifting foot at the transition instant — should be < 5 N.

### 2.3 Phase timeout fallback

Every phase has a 5.0 s timeout. If a transition condition is not met within the timeout, the controller degrades to bipedal stance (both feet 6-D, CoM centered) and the walk aborts. Implement:
```python
if self.phase_elapsed > self.phase_timeout:
    self._emergency_bipedal_stance()
    return  # Stop gait
```

### 2.4 Test script

- Run for 15 seconds
- Verify: all transitions fire on GRF conditions (not timers), lifting foot GRF < 5 N at lift-off, no timeouts

---

## Stage 3: Small Forward Steps (step_length = 0.10 m)

**Goal:** Introduce forward progression with heading correction and pelvis orientation regulation.

### 3.1 Footstep planner with heading correction

At the start of each `LEFT_SINGLE` / `RIGHT_SINGLE` phase:
```python
yaw = extract_yaw(pelvis_quat)
is_right_swing = (phase == LEFT_SINGLE)
target = plan_step(support_foot_pos, yaw, step_length, step_width, is_right_swing)
```

This rotates the step direction by the current pelvis yaw to prevent diagonal walking.

### 3.2 Pelvis orientation regulation task

Add a 6-D task to the QP during all phases except BIPEDAL_INIT:
```python
# Target: zero roll/pitch, nominal walking heading for yaw
pelvis_orientation_target = np.array([0.0, 0.0, target_yaw])
weights = np.array([roll_weight, pitch_weight, yaw_weight, 0.0, 0.0, 0.0])  # position dofs zero
```
Weight hierarchy: below CoM and swing-z, above swing-xy during transit.

### 3.3 CoM target strategy (PD-only, no quintic)

| Phase | CoM target |
|-------|------------|
| `BIPEDAL_INIT` | Midpoint of both feet |
| `WEIGHT_SHIFT_L/R` | Support foot center (full shift) |
| `LEFT_SINGLE` / `RIGHT_SINGLE` | Support foot center (PD-only setpoint) |
| `DOUBLE_SUPPORT` | 70/30 biased midpoint toward next support foot |

**No quintic smoothstep anywhere.** The GRF transition condition guarantees the CoM is near the support foot center at SINGLE entry. PD-only tracking avoids velocity discontinuities and unnecessary overshoot.

### 3.4 CoM velocity damping (optional hardening)

If residual lateral velocity destabilises single support, add a small velocity damping term to the CoM task:
```python
com_vel_target = np.zeros(3)  # zero velocity target during single support
com_vel_weight = 0.1  # small relative to position weight
```

This is documented as a concern in `docs/review_walking_milestone_ds.md` (§Remaining Concern #1).

### 3.5 Test script

- Run for 10 seconds, 3–5 steps
- Log: forward displacement, foot placement error, pelvis yaw, CoM tracking error, support slip, foot clearance, GRF at liftoff

Pass criteria:

| Check | Threshold | Rationale |
|-------|-----------|-----------|
| No fall | pelvis z > 0.5 m, |roll|,|pitch| < 15° | Hard safety |
| Pelvis stable | |roll|,|pitch| < 5° RMS during single support | Verifies orientation regulation |
| Forward displacement | ≥ 0.5 × step_length × num_steps | Verifies actual forward motion |
| Foot placement RMSE | < 0.05 m | Allowed 50% of step length for first forward steps |
| Yaw drift per step | < 2° | 2° on 0.10 m step = 3.5 mm lateral error |
| Support slip | < 0.005 m | 20% of lateral CoP margin (±0.025 m) |
| Foot clearance | > 0.02 m | Verifies actual lift, not slide |
| GRF at liftoff | support > 80% mg, swing < 5 N | Per architecture GRF hysteresis |

---

## Stage 4: Full 5-Meter Walk (step_length = 0.25 m)

**Goal:** Achieve the full milestone — walk 5 meters forward.

### 4.1 Parameter progression

| Parameter | Stage 3 | Stage 4 | Notes |
|-----------|---------|---------|-------|
| `step_length` | 0.10 m | 0.25 m | Increase by 0.05 m per test |
| `step_width` | 0.20 m | 0.20 m | Keep constant |
| `step_height` | 0.05 m | 0.05 m | Keep constant |
| `double_support_duration` | 0.15 s | 0.15 s | Keep constant (time-driven) |

### 4.2 CP safety abort (coarse tripwire only)

Before each step, check if CP is inside the convex hull of both foot polygons:
```python
if cp_distance_outside_both_hulls > cp_abort_margin:  # 0.05 m
    # Abort: set step_length = 0 for this step (in-place stepping)
    step_length = 0.0
    # Wait for CP to recover before resuming
```

This is NOT a transition condition — it's a last-resort safety tripwire. Routine transitions use GRF thresholds exclusively.

### 4.3 Termination

Stop when pelvis x ≥ 5.0 m. Log total time, step count, and average step period.

### 4.4 Test metrics (aligned with architecture §Evaluation Criteria)

| Metric | Target |
|--------|--------|
| Forward displacement | ≥ 5.0 m |
| No falls | Pelvis z > 0.5 m, |roll| < 15°, |pitch| < 15° |
| Yaw drift | Cumulative < 10° over 5 m |
| Foot clearance | Swing foot z > 0.03 m above support foot |
| Foot placement error | RMSE < 0.05 m |
| GRF ratio at lift-off | Support foot > 80%·mg |
| Lifted foot GRF < 5 N | At transition into single support |
| CP safety | Inside convex hull of both polygons (or abort fires) |
| Phase duration stability | Step time std dev < 0.15 s |
| No timeouts | No phase reaches 5.0 s |
| Control latency | QP solve < 0.002 s (99th percentile) |
| Torque limits | All joints < 150 N·m |

---

## Stage 5: Diagnostics and Hardening

### 5.1 `scripts/debug_walking.py`

Plots (multi-panel):
1. Gait phase timeline (colored bands)
2. Forward displacement vs time
3. Foot placement: planned vs actual (x and y)
4. Swing foot z height
5. GRF per foot over time (with 50%/80% thresholds)
6. CoM tracking error (x and y)
7. Pelvis orientation (roll, pitch, yaw)
8. Capture point vs convex hull of both foot polygons
9. Torque utilisation (max per joint per step)
10. QP solve time histogram

### 5.2 Optional hardening items

These address the remaining concerns from `docs/review_walking_milestone_ds.md`:

1. **CoM velocity damping** (Concern #1): Add zero-velocity target during single support to prevent residual lateral velocity from consuming CoP margin.
2. **Swing-leg centroidal momentum feedforward** (Concern #2): Pre-compute swing leg reaction wrench from planned trajectory and offset the floating-base constraint — reduces disturbance before state feedback.
3. **Torsional saturation monitoring** (Concern #3): Log `λ_τz` from QP solution; flag if repeatedly saturated at constraint bound (indicates yaw regulation is constraint-limited, not objective-driven).

### 5.3 Push recovery during walking

Extend test script:
- Apply impulses during `LEFT_SINGLE` and `RIGHT_SINGLE`
- Verify CP safety abort fires correctly (step reduced to in-place)
- Verify phase timeout fallback works if recovery fails

---

## File Change Summary

| File | Action | Lines (est.) |
|------|--------|--------------|
| `planners/footstep_planner.py` | New | ~50 |
| `controllers/walking_controller.py` | New | ~400 |
| `scripts/test_walking.py` | New (or major rework) | ~300 |
| `scripts/debug_walking.py` | New | ~400 |
| `configs/g1_config.yaml` | Add `walking:`, `transitions:`, `safety:`, `swing_weights:`, `pelvis_orientation:` | ~40 |

**No changes required to:** `env/g1_env.py`, `utils/kinematics.py`. The QP-WBC base class and wrench cone builder need the torsional friction constraint addition (handled in the walking controller override).

---

## Transition Logic Reference (all in one place)

```
GRF hysteresis:
  50% mg  → arm the transition (weight is starting to transfer)
  80% mg  → fire the transition (most weight is on support foot)
  0.05 s  → minimum delay between arm and fire (prevents chatter)
  < 5 N   → lifting foot GRF precondition (contact wrench continuity)

Touchdown (SINGLE → DOUBLE_SUPPORT):
  z        → within 0.005 m of support foot z
  xy       → within 0.02 m of target
  GRF      → > 10% mg (physical contact confirmed)

Time-driven:
  INIT → WEIGHT_SHIFT:    0.1 s  (initial settle + break symmetry)
  DOUBLE_SUPPORT → WEIGHT_SHIFT: 0.15 s (weight transfer hasn't started yet)

Timeout:  5.0 s per phase → emergency bipedal stance fallback
Safety:   CP > 0.05 m outside both-foot convex hull → step_length = 0
```

---

## Critical Assumptions and Fallbacks

1. **Assumption:** GRF measurements from `mj_contactForce()` are reliable enough for transition detection.
   - **Fallback:** Smooth GRF with 5-tap moving average; increase `grf_arm_to_fire_delay` to 0.1 s.

2. **Assumption:** Step length 0.25 m is within G1 kinematic reach.
   - **Fallback:** Reduce to 0.20 m or increase hip extension in reference pose.

3. **Assumption:** The QP-WBC can stabilise single support with the support foot's ±2.5 cm lateral CoP margin.
   - **Fallback:** Increase `min_single_duration` to give CoM more settle time; add CoM velocity damping.

4. **Assumption:** Torsional friction constraint (3.4 N·m per foot) is adequate for yaw regulation.
   - **Fallback:** Increase `yaw_weight` in pelvis orientation task; monitor `λ_τz` saturation.

5. **Assumption:** Time-varying swing-xy weight (0.1 early, 1.0 late) sufficiently decouples swing momentum.
   - **Fallback:** If trunk disturbance is excessive, add centroidal momentum feedforward (Stage 5 hardening #2).

---

## Estimated Effort

| Stage | Effort | Cumulative |
|-------|--------|------------|
| 0: Infrastructure | 3–4 h | 3–4 h |
| 1: Static alternating | 4–5 h | 7–9 h |
| 2: In-place stepping | 2–3 h | 9–12 h |
| 3: Small forward steps | 4–5 h | 13–17 h |
| 4: Full 5 m walk | 3–4 h | 16–21 h |
| 5: Diagnostics | 2–3 h | 18–24 h |

> Total: **3–4 working days** for a developer familiar with the existing codebase.
