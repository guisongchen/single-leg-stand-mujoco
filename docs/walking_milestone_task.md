# Milestone: Forward Walking on Flat Ground

## Objective

Extend the existing bipedal stance and single-leg balance capabilities into a **periodic walking gait**. The robot must walk 5 meters forward on flat ground, starting from a static bipedal stance, using alternating left/right support phases.

This milestone builds directly on the Phase 1–3 infrastructure (QP-WBC, CoM planners, swing foot trajectories, capture-point switching). No MPC is required; the architecture stays hierarchical with a gait scheduler on top.

---

## Physics Constraints

All phase timing and transition rules are derived from the physical limits of the G1 robot.

### Friction force limit

Maximum horizontal ground reaction force: `F_h_max = μ · m · g = 0.8 × 34.13 × 9.81 ≈ 268 N`

Maximum CoM linear acceleration (both feet or single foot — the limit is `μ·g` in either case):

```
a_com_max = μ · g = 7.85 m/s²
```

### CoP envelope per foot

The four corner-sphere geoms per foot form a rectangular CoP envelope in the foot's local frame:

| Direction | Offset from body origin | Bound |
|-----------|------------------------|-------|
| Forward (toe) | +12 cm | `cop_x_forward = 0.12 m` |
| Backward (heel) | –5 cm | `cop_x_back = 0.05 m` |
| Lateral (each side) | ±2.5 cm | `cop_y_half = 0.025 m` |

These are hard inequality constraints in the QP (`_build_wrench_cones`). A CoM offset beyond these bounds makes the QP infeasible.

### Capture-point transition margin

The tiny lateral CoP envelope (±2.5 cm) makes CP-based transitions fragile. With `cp_polygon_margin = 0.01 m`, the effective CP check window is roughly:
- Lateral: ±1.5 cm total (still tight, but leaves room for the CoM velocity to carry the CP through)
- Forward: 0.11 m forward, 0.04 m backward

A static-margin-only check is insufficient. The transition condition must also verify that the **CP is converging** — its lateral velocity component must be toward the polygon center (not drifting away). This prevents triggering on a momentary CP excursion where the CoM is still decelerating from the weight shift.

### Consequence: quasi-static gait

The 2.5 cm lateral CoP envelope per foot forces a **quasi-static** gait. The CoM must be nearly over the support foot center before the opposite foot can lift. The robot cannot lean or exploit angular momentum the way a full-sized humanoid foot (CoP envelope ~15 cm × 7 cm) would allow.

### Minimum CoM shift during double support

With `step_length = 0.25 m` and `step_width = 0.20 m`, the CoM must move from the trailing foot's CoP envelope to the leading foot's CoP envelope during double support. The shortest diagonal between the closest edges of the two footprints is approximately 0.17 m. Given `a_com_max = 7.85 m/s²`:

```
T_min_ds = √(5.774 × 0.17 / 7.85) ≈ 0.35 s   (quintic smoothstep)
T_min_ds ≈ 0.29 s   (bang-bang, theoretical minimum)
```

A fixed double-support timer of 0.1–0.2 s would require 24–98 m/s² of CoM acceleration (2.5–10g), which is physically impossible. **This is why the design uses event-driven switching instead of a fixed timer** — the double-support phase self-adjusts to however long physics requires.

---

## Technical Approach

### 1. Gait Scheduler (Event-Driven FSM)

A periodic gait cycles through five phases:

```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

The `WEIGHT_SHIFT` phases break the symmetry of double support by explicitly shifting the CoM toward the upcoming support foot. Without this, the CoM would sit at the midpoint and the capture point would never enter either foot's tiny CoP polygon (±2.5 cm lateral).

| Phase | Active feet | CoM target | Swing foot |
|-------|-------------|------------|------------|
| `BIPEDAL_INIT` | Both (6-D) | Centered between feet | None |
| `WEIGHT_SHIFT_L` | Both (6-D) | Left foot center (biased) | None (both grounded) |
| `LEFT_SINGLE` | Left only (6-D) | Left foot center | Right foot, forward target |
| `DOUBLE_SUPPORT` | Both (6-D) | Biased toward next support foot | None (both grounded) |
| `WEIGHT_SHIFT_R` | Both (6-D) | Right foot center (biased) | None (both grounded) |
| `RIGHT_SINGLE` | Right only (6-D) | Right foot center | Left foot, forward target |

Both-feet phases use **6-D contacts** (position + orientation). During double support, the robot must resist yaw rotation of the feet on the ground; 3-D position-only contacts provide no yaw constraint, and without it the feet drift in yaw over repeated steps.

**Phase transitions use hybrid switching: a minimum time guard plus a spatial event condition.** No transition relies on a fixed timer alone. Every transition has a generous timeout to prevent hanging.

#### Transition rules

| Transition | Minimum guard | Event condition |
|------------|--------------|-----------------|
| `INIT → WEIGHT_SHIFT_L` | 1.0 s | (time-driven only — initial settle, then break symmetry) |
| `WEIGHT_SHIFT_L → LEFT_SINGLE` | 0.05 s | CP inside **left** foot's CoP polygon (with margin) AND CP lateral velocity converging toward polygon center |
| `LEFT_SINGLE → DOUBLE_SUPPORT` | `min_single_duration` (0.5 s) | Swing foot z < touchdown threshold AND swing foot xy within tolerance AND CP inside support foot polygon |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` | 0.05 s | CP inside **right** foot's CoP polygon (with margin) AND CP lateral velocity converging toward polygon center |
| `WEIGHT_SHIFT_R → RIGHT_SINGLE` | 0.05 s | CP inside **right** foot's CoP polygon (with margin) AND CP lateral velocity converging toward polygon center |
| `RIGHT_SINGLE → DOUBLE_SUPPORT` | `min_single_duration` (0.5 s) | Swing foot z < touchdown threshold AND swing foot xy within tolerance AND CP inside support foot polygon |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_L` | 0.05 s | CP inside **left** foot's CoP polygon (with margin) AND CP lateral velocity converging toward polygon center |

After the initial `WEIGHT_SHIFT_L`, the gait alternates `SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT → SINGLE` indefinitely.

**Timeout for every phase:** 5.0 s. If a transition condition is not met within the timeout, the controller degrades to bipedal stance (both feet hard contacts, CoM centered) and the walk is aborted.

**Termination:** when total forward pelvis displacement ≥ 5.0 m.

#### Why this is robust

- `WEIGHT_SHIFT` phases explicitly drive the CoM toward the upcoming support foot. This breaks the midpoint deadlock where the CP oscillates near the center and never enters either foot's polygon. The CP enters the target polygon because the CoM target is already there.
- `DOUBLE_SUPPORT` phases use a biased midpoint target (70/30 toward next support foot) to start the weight transfer. The subsequent `WEIGHT_SHIFT` phase finishes the job with a full shift.
- Minimum time guards (0.05 s) prevent chatter from momentary CP excursions.
- The single-support phase uses a longer minimum guard (0.5 s) because the swing foot needs time to travel its arc; the guard also ensures the swing foot reaches its target before the next touchdown check fires.
- Timeout protection means the controller degrades gracefully rather than getting stuck in a phase indefinitely.

### 2. Footstep Planner

Simple fixed-schedule planner (no vision, no terrain adaptation):

```
step_length = 0.25 m
step_width  = 0.20 m  (pelvis width, keeps feet under hips)
```

For each step:
- Place the swing foot at `(support_foot_x + step_length, ±step_width/2, support_foot_z)`
- The sign alternates left/right (`+` for left swing, `−` for right swing)
- Foot placement target is computed in world frame at the moment the swing phase begins

**Heading correction:** The step direction is rotated to match the current pelvis yaw, not a fixed world-frame direction. Each step target is computed as:

```python
# Rotate step_length vector by current pelvis yaw
yaw = pelvis_orientation_quat.to_yaw()
target_x = support_foot_x + step_length * cos(yaw)
target_y = support_foot_y + step_length * sin(yaw)  # drift correction
```

This prevents the robot from walking diagonally when yaw drifts. The QP also receives a pelvis yaw regulation task (see §6) that drives the torso back toward the nominal heading.

**Safety abort:** If the capture point at the end of double-support lies outside the upcoming support foot's CoP polygon by more than 0.05 m despite the event condition, abort the step and request zero step length (in-place stepping) until stability is recovered.

### 3. CoM Target Strategy

The CoM reference is a **setpoint**, not a timed trajectory, during weight transfer. This lets the QP drive the CoM at whatever rate physics allows — no pre-tuned duration needed.

- **During `BIPEDAL_INIT`:** CoM target = static midpoint between the two feet. Constant. The QP's CoM feedback (PD) converges to it naturally.

- **During `WEIGHT_SHIFT_L` / `WEIGHT_SHIFT_R`:** CoM target = center of the upcoming support foot (left or right respectively). This is a full shift, not a biased midpoint. The QP drives the CoM using PD error, and the friction-cone hard constraint clips the achievable acceleration. The CP converges toward the support foot and the phase transition fires when the CP is inside the polygon with converging velocity. This phase replaces the old "double support with midpoint target" that could never trigger a transition.

- **During `DOUBLE_SUPPORT`:** CoM target = **biased midpoint**: 70% toward the next support foot, 30% toward the current support foot. This starts the weight shift so the CoM is already moving in the right direction when the subsequent `WEIGHT_SHIFT` phase begins. The bias prevents the CP from stalling at the neutral midpoint.

- **During `LEFT_SINGLE` / `RIGHT_SINGLE`:** CoM target = support foot center. A quintic smoothstep (`ComPlanner`) interpolates from the CoM position at phase entry to the target, with duration `min_single_duration`. This provides a feedforward acceleration reference during the first 0.5 s. After the smoothstep completes, the target holds at the support foot center and the QP tracks it with PD-only feedback for the remainder of the phase. This is acceptable because the CoM should already be near the support foot by this point.

**Rationale for setpoint-only weight shift:** A timed smoothstep during weight shift demands an acceleration profile scaled by `1/T²`. If T is chosen too short, the reference acceleration exceeds friction limits and the QP produces large tracking error. A constant setpoint with PD feedback avoids this: the acceleration demand is `kp·position_error`, which starts high but is naturally clipped by the friction cone constraints. The CoM converges at the rate physics allows.

### 4. Swing Foot Trajectory

Use the existing `SwingFootPlanner`:
- Lift height: 0.05 m (above ground irregularities)
- Rise duration: 40% of swing phase, descent: 40%, apex dwell: 20%
- **z tracking: soft objective** (tracked by a swing task in the QP, same as single-leg stand)
- **xy tracking: time-varying weight.** The swing foot's xy position is tracked with a low weight during the first 60% of the swing phase (lift and transit) to avoid coupling swing-leg reaction forces into the trunk. The weight ramps up during the final 40% (descent) so the foot decelerates into the target accurately. This balances two competing needs: low weight mid-swing to protect trunk stability, and high weight near touchdown to meet the 0.02 m placement tolerance.

Swing foot xy progress is also used for the transition condition (foot within 0.02 m of target). If tracking causes the foot to lag, the single-support phase extends until it catches up (up to the timeout).

### 5. Pelvis Orientation Regulation

Angular momentum of the trunk and swing leg directly affects the capture point during single support. The QP must include a pelvis orientation task that regulates:
- **Roll and pitch:** drive toward zero (upright torso). This indirectly constrains trunk angular momentum.
- **Yaw:** drive toward the nominal walking heading. This prevents accumulated yaw drift across steps.

The pelvis orientation task is a 6-D task (3 rotational DOFs) with moderate weight — below CoM and swing-z tracking, but above swing-xy during the transit phase. Without this task, the swing leg's reaction torques cause the trunk to yaw and pitch, which shifts the capture point laterally and destabilises the gait.

### 6. Controller Architecture

```
GaitScheduler
    └── evaluates spatial conditions (CP, CP converging, foot position, contact force)
        └── determines: active_feet, com_target, swing_task, pelvis_orientation_target
            └── calls: _solve_qp(...) with updated active_feet & tasks
```

The walking controller extends `QPWBCController` directly and manages:
- `_update_gait_phase()`: evaluate all transition conditions, advance FSM
- `_compute_com_target()`: return biased midpoint, full shift, or smoothstep reference depending on phase
- `_compute_swing_weights()`: return time-varying swing-xy weight (low early, high near touchdown)
- `_compute_pelvis_orientation_target()`: return nominal walking heading for yaw, zero for roll/pitch
- `_compute_footstep_target()`: rotate step vector by current pelvis yaw for heading correction
- `compute()`: assemble active feet, swing task, pelvis orientation task, call QP, return torques

---

## Deliverables

| File | Purpose | Status |
|------|---------|--------|
| `controllers/walking_controller.py` | Event-driven gait scheduler + walking logic | Requires update |
| `planners/footstep_planner.py` | Fixed-schedule foot placement generator | Exists |
| `scripts/test_walking.py` | End-to-end test: 5 m walk, metrics, pass/fail | Requires update (currently in-place only) |
| `scripts/debug_walking.py` | Phase timing, foot placement error, CP tracking | New |

---

## Configuration Parameters

```yaml
walking:
  step_length: 0.25             # forward distance per step (m)
  step_width: 0.20              # lateral distance between feet (m)
  step_height: 0.05             # swing foot clearance above ground (m)
  min_single_duration: 0.50     # minimum time in single-support before touchdown check (s)
  min_double_guard: 0.05        # minimum time in double-support/weight-shift to prevent chatter (s)
  init_duration: 1.0            # seconds in INIT before first weight shift (s)
  phase_timeout: 5.0            # maximum time in any phase before abort (s)
  forward_direction: [1.0, 0.0] # nominal walking direction (world x); actual direction rotated by pelvis yaw
  double_support_com_bias: 0.70 # fraction of weight shift toward next support foot during DOUBLE_SUPPORT

safety:
  cp_polygon_margin: 0.01       # inward margin from CoP edge for CP check (m) — reduced from 0.02 for small feet
  cp_velocity_converging: true   # require CP lateral velocity toward polygon center
  touchdown_xy_tolerance: 0.02  # swing foot must be this close to target xy (m)
  touchdown_z_tolerance: 0.005  # swing foot must be this close to support foot z (m)
  abort_cp_margin: 0.05         # if CP is this far outside the polygon, abort the step (m)

swing_weights:
  xy_early: 0.10               # swing xy weight during first 60% of swing (low — decouple from trunk)
  xy_late: 1.0                 # swing xy weight during last 40% of descent (high — placement accuracy)
  z: 1.0                       # swing z weight (constant — clearance priority)

pelvis_orientation:
  roll_weight: 0.5             # weight for roll regulation task
  pitch_weight: 0.5             # weight for pitch regulation task
  yaw_weight: 0.3              # weight for yaw regulation task (lower — allow slow turning)
  target_yaw: 0.0             # nominal heading (world x), updated per-step from pelvis state
```

---

## Evaluation Criteria (Pass/Fail)

| Metric | Target | Test Method |
|--------|--------|-------------|
| Forward displacement | ≥ 5.0 m | Pelvis x position at end |
| No falls | Pelvis z > 0.5 m, |roll| < 15°, |pitch| < 15° | Per-timestep check |
| Yaw drift | Cumulative yaw < 10° over 5 m | Per-step pelvis yaw log |
| Foot clearance | Swing foot z > 0.03 m above support foot during swing | Height log |
| Foot placement error | Actual vs planned foot xy < 0.05 m | RMSE at first timestep after touchdown detection |
| CP inside polygon at lift-off | CP in support foot CoP polygon when entering single support | Per-transition check |
| CP velocity converging at lift-off | CP lateral velocity toward polygon center when entering single support | Per-transition check |
| Phase duration stability | Step time std dev < 0.15 s | Phase duration log (expects variation from event-driven switching) |
| No timeouts | No phase reaches its 5.0 s timeout | Phase duration log |
| Control latency | QP solve < 0.002 s (99th percentile) | Per-step timing log |
| Torque limits | All joint torques < 150 N·m | Per-timestep check |

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Swing foot xy tracking destabilises trunk | Robot falls mid-gait | Time-varying weight: low during transit (0.1), high during descent (1.0); z-only swing task in the QP |
| CoM fails to reach support foot CoP during weight shift | Robot stuck, never transitions | Explicit WEIGHT_SHIFT phase drives CoM to support foot; timeout (5.0 s) triggers bipedal-stance fallback |
| CP transition deadlock at midpoint | Robot stuck in INITIAL/DOUBLE_SUPPORT | WEIGHT_SHIFT phase breaks symmetry; biased CoM target (70/30) starts transfer; CP convergence check prevents premature trigger |
| Event condition fires too early (CP chatter) | Robot enters single support before CoM is stable | Minimum time guard (0.05 s); CP convergence check requires lateral velocity toward polygon center |
| Single support minimum duration too short | Swing foot touches down late or at wrong location | `min_single_duration` is conservative (0.5 s); transition requires xy + z proximity |
| Torque saturation during dynamic weight shift | QP infeasible or large tracking error | Increase `wbc_tau_limit` to 150 N·m if needed; monitor `max_tau` in debug |
| Foot placement on previous step | Self-collision or tripping | Maintain `step_width = 0.20 m`; verify with MuJoCo contact pair exclusions |
| Phase timeout triggered by accumulated error | Walk aborted early | Tune `phase_timeout` and CP margins; log phase durations to detect drift |
| Yaw drift accumulates over steps | Robot walks diagonally, lateral fall | Pelvis yaw regulation task in QP; step direction rotated by current yaw; 6-D contacts during double support resist yaw |
| Swing xy weight too low near touchdown | Foot placement error exceeds 0.02 m tolerance | Weight ramps up during final 40% of swing; if tolerance still not met, phase extends until touchdown |
| Quintic ends before swing foot lands | CoM in PD-only regime after 0.5 s | Target holds at support foot center; PD is sufficient since CoM should be near target by then |
| CP margin too tight for dynamic gait | Transitions fail due to velocity-induced CP shifts | Margin reduced to 0.01 m (from 0.02); convergence check adds velocity criterion; WEIGHT_SHIFT ensures CoM is already near support foot |

---

## Why Not MPC (Yet)

Full nonlinear MPC with contact scheduling would:
- Require mixed-integer optimization or pre-specified gait timings
- Exceed the 2 ms control budget for a 10-step horizon
- Add model-mismatch risk (MuJoCo vs simplified MPC model)

The hierarchical approach (event-driven gait scheduler + single-step QP-WBC) is the standard architecture for humanoid walking (e.g., IHMC, MIT DRC). MPC can be added later as a **CoM preview layer** without touching the QP-WBC.

---

## Suggested Development Order

1. **Static alternating support** (no forward motion)
   - Lift right foot, hold, place, lift left foot, hold, place
   - Verify the event-driven FSM works without falling

2. **In-place stepping** (zero step length, only lift/place)
   - Tests contact scheduling and the double-support CP event condition

3. **Small forward steps** (step_length = 0.10 m)
   - 3–5 steps, verify CoM tracking and foot placement error

4. **Triggered step-length adjustment**
   - Implement the CP safety abort: if CP is far outside the upcoming foot polygon, set step_length = 0 for that step

5. **Full 5 m walk** (step_length = 0.25 m)
   - Verify event conditions fire reliably, no timeouts, all pass/fail criteria met
