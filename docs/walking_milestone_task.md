# Milestone: Forward Walking on Flat Ground

## Objective

Extend the existing bipedal stance and single-leg balance capabilities into a **periodic walking gait**. The robot must walk 5 meters forward on flat ground, starting from a static bipedal stance, using alternating left/right support phases.

This milestone builds directly on the Phase 1–3 infrastructure (QP-WBC, CoM planners, swing foot trajectories). No MPC is required; the architecture stays hierarchical with a gait scheduler on top.

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

These are hard inequality constraints in the QP (`_build_wrench_cones`). They restrict what torques the optimizer can command, **not** when gait transitions happen. Transition conditions use ground reaction force (GRF) thresholds instead of capture-point (CP) geometry, because the ±2.5 cm lateral envelope makes CP-based transitions unreliable (see §"Why GRF transitions instead of CP transitions" below).

### Why GRF transitions instead of CP transitions

The original design used CP-based transition conditions (CP inside support foot's CoP polygon with converging velocity). This approach has three compounding problems for a robot with ±2.5 cm lateral feet:

1. **Tight margin:** The effective CP window is only ±1.5 cm (after 0.01 m margin). This is 1.5% of the step width — comparable to MuJoCo's contact solver tolerance and numerical noise.

2. **Timing mismatch:** The CP traverses the 3 cm polygon in ~0.03–0.04 s during PD transient convergence, but the minimum time guard is 0.05 s. The transition can never fire: the guard blocks it while the CP is inside, and by the time the guard expires, the CP has overshot.

3. **Swing-leg angular momentum:** The swing leg (~5–7 kg) creates ~3 N·m·s of angular momentum about the support foot, shifting the CP by ~2.8 cm — nearly double the ±1.5 cm effective window.

GRF-based transitions avoid all three problems:
- GRF directly measures weight distribution — is the load actually on the foot?
- GRF is monotonically increasing during weight shift (50% → 80%), so there is no timing window problem
- GRF naturally includes all dynamic effects (swing momentum, contact impulses)
- GRF thresholds use hysteresis (50% arm → 80% fire) to prevent chatter

CP conditions are retained **only as safety aborts**: if CP is > 0.05 m outside the convex hull of both foot polygons at any time, the step is aborted.

### Consequence: quasi-static gait

The 2.5 cm lateral CoP envelope per foot forces a **quasi-static** gait. The CoM must be nearly over the support foot center before the opposite foot can lift. The robot cannot lean or exploit angular momentum the way a full-sized humanoid foot (CoP envelope ~15 cm × 7 cm) would allow.

Expected walking speed is approximately **0.25 m/s** (~20 s for 5 m), since forward CoM velocity goes to zero during each single-support phase and forward motion occurs only during weight-transfer phases.

### Minimum CoM shift during double support

With `step_length = 0.25 m` and `step_width = 0.20 m`, the CoM must move from the trailing foot's CoP envelope to the leading foot's CoP envelope during double support. The shortest **diagonal** between the closest edges of the two footprints (accounting for both lateral and forward offset) is approximately:

```
d_min = √(0.17² + 0.15²) ≈ 0.23 m
```

Not 0.17 m (which is only the forward component). Given `a_com_max = 7.85 m/s²`:

```
T_min_ds = √(5.774 × 0.23 / 7.85) ≈ 0.41 s   (quintic smoothstep)
T_min_ds ≈ √(4 × 0.23 / 7.85) ≈ 0.34 s   (bang-bang, theoretical minimum)
```

A fixed double-support timer of 0.1–0.2 s would require 17–50 m/s² of CoM acceleration (1.7–5.1g), which is physically impossible. **This is why the design uses event-driven switching instead of a fixed timer** — the double-support phase self-adjusts to however long physics requires.

### Contact wrench continuity at lift-off

When transitioning from double to single support, the QP removes one foot's contact constraints. If that foot has residual contact force (e.g., 10 N vertical) at lift-off, the sudden removal creates an acceleration step of ~0.3 m/s², shifting the CP by ~1 cm. To prevent this, transitions into single support require the lifting foot's GRF to be below a threshold (see transition rules below).

---

## Technical Approach

### 1. Gait Scheduler (Event-Driven FSM with GRF Transitions)

A periodic gait cycles through five phases:

```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

The `WEIGHT_SHIFT` phases break the symmetry of double support by explicitly shifting the CoM toward the upcoming support foot. Without this, the CoM would sit at the midpoint and the GRF would never reach the 80% threshold on one foot.

| Phase | Active feet | CoM target | Swing foot |
|-------|-------------|------------|------------|
| `BIPEDAL_INIT` | Both (6-D) | Centered between feet | None |
| `WEIGHT_SHIFT_L` | Both (6-D) | Left foot center | None (both grounded) |
| `LEFT_SINGLE` | Left only (6-D) | Left foot center | Right foot, forward target |
| `DOUBLE_SUPPORT` | Both (6-D) | Biased toward next support foot | None (both grounded) |
| `WEIGHT_SHIFT_R` | Both (6-D) | Right foot center | None (both grounded) |
| `RIGHT_SINGLE` | Right only (6-D) | Right foot center | Left foot, forward target |

Both-feet phases use **6-D contacts** (position + orientation). During double support, the robot must resist yaw rotation of the feet on the ground; 3-D position-only contacts provide no yaw constraint, and without it the feet drift in yaw over repeated steps.

**Phase transitions use GRF thresholds with hysteresis.** No transition relies on CP geometry alone. Every transition has a generous timeout to prevent hanging.

#### Transition rules

| Transition | Condition |
|------------|-----------|
| `INIT → WEIGHT_SHIFT_L` | Time-driven: 0.1 s elapsed (initial settle, then break symmetry) |
| `WEIGHT_SHIFT_L → LEFT_SINGLE` | GRF on **left** foot > 80%·mg AND 0.05 s since GRF first exceeded 50%·mg AND lifted foot GRF < 5 N |
| `LEFT_SINGLE → DOUBLE_SUPPORT` | Swing foot z < touchdown threshold AND swing foot xy within 0.02 m of target AND swing foot GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` | Time-driven: 0.15 s elapsed (weight transfer hasn't started yet; no spatial condition needed) |
| `WEIGHT_SHIFT_R → RIGHT_SINGLE` | GRF on **right** foot > 80%·mg AND 0.05 s since GRF first exceeded 50%·mg AND lifted foot GRF < 5 N |
| `RIGHT_SINGLE → DOUBLE_SUPPORT` | Swing foot z < touchdown threshold AND swing foot xy within 0.02 m of target AND swing foot GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_L` | Time-driven: 0.15 s elapsed |

After the initial `WEIGHT_SHIFT_L`, the gait alternates `SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT → SINGLE` indefinitely.

**Why GRF 80%/50% hysteresis for WEIGHT_SHIFT → SINGLE:**
- The 50% threshold arms the transition (weight is starting to transfer)
- The 80% threshold fires the transition (most weight is on the support foot)
- The 0.05 s delay since arming prevents chatter from momentary GRF spikes
- The lifting foot GRF < 5 N ensures contact wrench continuity

**Why DOUBLE_SUPPORT → WEIGHT_SHIFT is time-driven:**
- The biased midpoint (70/30) places the CoM outside either individual foot's CoP polygon. A CP-based condition would never fire because CP ≈ CoM at low velocity.
- The real stability gate is WEIGHT_SHIFT → SINGLE, where the CoM has reached the support foot center. At the DOUBLE_SUPPORT → WEIGHT_SHIFT transition, all that's needed is confirmation that motion has started, which a short timer provides.

**Safety abort:** If at any time the capture point lies > 0.05 m outside the convex hull of both foot polygons, abort the step and request zero step length (in-place stepping) until stability is recovered.

**Timeout for every phase:** 5.0 s. If a transition condition is not met within the timeout, the controller degrades to bipedal stance (both feet hard contacts, CoM centered) and the walk is aborted.

**Termination:** when total forward pelvis displacement ≥ 5.0 m.

#### Why this is robust

- `WEIGHT_SHIFT` phases explicitly drive the CoM toward the upcoming support foot. The GRF on that foot monotonically increases, making the 80% threshold easy to reach without tight geometric margins.
- `DOUBLE_SUPPORT` phases use a biased midpoint target (70/30 toward next support foot) to start the weight transfer. The subsequent `WEIGHT_SHIFT` phase finishes the job with a full shift.
- GRF hysteresis (50% arm, 80% fire) prevents chatter from momentary force spikes.
- Lifted foot GRF < 5 N precondition prevents contact wrench discontinuity at lift-off.
- The single-support phase uses a 0.5 s minimum duration and requires swing foot touchdown + xy proximity + GRF confirmation, ensuring physical contact before transitioning.
- Timeout protection means the controller degrades gracefully rather than getting stuck indefinitely.

### 2. Footstep Planner

Simple fixed-schedule planner (no vision, no terrain adaptation):

```
step_length = 0.25 m
step_width  = 0.20 m  (pelvis width, keeps feet under hips)
```

For each step:
- Left foot target y = +step_width / 2 (absolute position)
- Right foot target y = −step_width / 2 (absolute position)
- Swing foot target x = support_foot_x + step_length (for a right swing; subtract for left)
- Foot placement target is computed in world frame at the moment the swing phase begins

**Heading correction:** The step direction is rotated to match the current pelvis yaw, not a fixed world-frame direction. Each step target is computed as:

```python
yaw = pelvis_orientation_quat.to_yaw()
target_x = support_foot_x + step_length * cos(yaw)
target_y = foot_y  # absolute lateral position, not offset from support foot
```

This prevents the robot from walking diagonally when yaw drifts. The QP also receives a pelvis yaw regulation task (see §6) that drives the torso back toward the nominal heading.

### 3. CoM Target Strategy

The CoM reference is a **setpoint**, not a timed trajectory, during weight transfer. This lets the QP drive the CoM at whatever rate physics allows — no pre-tuned duration needed.

- **During `BIPEDAL_INIT`:** CoM target = static midpoint between the two feet. Constant. The QP's CoM feedback (PD) converges to it naturally.

- **During `WEIGHT_SHIFT_L` / `WEIGHT_SHIFT_R`:** CoM target = center of the upcoming support foot (left or right respectively). This is a full shift, not a biased midpoint. The QP drives the CoM using PD error, and the friction-cone hard constraint clips the achievable acceleration. The GRF on the support foot increases monotonically, and the phase transition fires when GRF exceeds 80% mg.

- **During `DOUBLE_SUPPORT`:** CoM target = **biased midpoint**: 70% toward the next support foot, 30% toward the current support foot. This starts the weight shift so the CoM is already moving in the right direction when the subsequent `WEIGHT_SHIFT` phase begins.

- **During `LEFT_SINGLE` / `RIGHT_SINGLE`:** CoM target = support foot center. **No quintic smoothstep.** PD feedback alone tracks the constant setpoint. Since the GRF transition condition guarantees the CoM is near the support foot center at phase entry, PD convergence is sufficient. This avoids velocity discontinuities at phase boundaries and unnecessary overshoot when the weight-shift phase has already converged the CoM to the target.

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
    └── evaluates GRF thresholds, foot position, contact state
        └── determines: active_feet, com_target, swing_task, pelvis_orientation_target
            └── calls: _solve_qp(...) with updated active_feet & tasks
```

The walking controller extends `QPWBCController` directly and manages:
- `_update_gait_phase()`: evaluate all transition conditions, advance FSM
- `_compute_com_target()`: return biased midpoint, full shift, or PD-only setpoint depending on phase
- `_compute_swing_weights()`: return time-varying swing-xy weight (low early, high near touchdown)
- `_compute_pelvis_orientation_target()`: return nominal walking heading for yaw, zero for roll/pitch
- `_compute_footstep_target()`: rotate step vector by current pelvis yaw for heading correction
- `compute()`: assemble active feet, swing task, pelvis orientation task, call QP, return torques

### 7. QP Wrench Cone Additions

The existing `_build_wrench_cones()` must include torsional friction constraints for 6-D contacts:

```
|λ_τz| ≤ (μ · min(foot_width, foot_length) / 2) · λ_fz
```

For a 5 cm × 17 cm foot at μ = 0.8 and F_z ≈ 170 N:

```
τ_z_max ≈ 0.8 × 170 × 0.025 ≈ 3.4 N·m
```

Without this constraint, the QP optimizer may request physically impossible yaw torques, and MuJoCo's contact solver will produce constraint forces that diverge from the QP's expectation.

---

## Deliverables

| File | Purpose | Status |
|------|---------|--------|
| `controllers/walking_controller.py` | Event-driven gait scheduler + walking logic | Requires update |
| `planners/footstep_planner.py` | Fixed-schedule foot placement generator | Exists |
| `scripts/test_walking.py` | End-to-end test: 5 m walk, metrics, pass/fail | Requires update (currently in-place only) |
| `scripts/debug_walking.py` | Phase timing, foot placement error, GRF tracking | New |

---

## Configuration Parameters

```yaml
walking:
  step_length: 0.25             # forward distance per step (m)
  step_width: 0.20              # lateral distance between feet (m)
  step_height: 0.05             # swing foot clearance above ground (m)
  min_single_duration: 0.50     # minimum time in single-support before touchdown check (s)
  init_duration: 0.1            # seconds in INIT before first weight shift (s)
  double_support_duration: 0.15 # fixed duration for DOUBLE_SUPPORT phase (s)
  phase_timeout: 5.0            # maximum time in any phase before abort (s)
  forward_direction: [1.0, 0.0] # nominal walking direction (world x); actual direction rotated by pelvis yaw
  double_support_com_bias: 0.70 # fraction of weight shift toward next support foot during DOUBLE_SUPPORT

transitions:
  grf_arm_threshold: 0.50       # fraction of mg to arm WEIGHT_SHIFT → SINGLE transition
  grf_fire_threshold: 0.80      # fraction of mg to fire WEIGHT_SHIFT → SINGLE transition
  grf_arm_to_fire_delay: 0.05   # minimum time between arm and fire (s)
  grf_touchdown_threshold: 0.10 # fraction of mg to confirm swing foot contact for SINGLE → DOUBLE_SUPPORT
  grf_liftoff_threshold: 5.0   # Newtons; lifting foot must be below this to enter SINGLE
  cp_abort_margin: 0.05         # abort step if CP is this far outside convex hull of both foot polygons (m)

safety:
  touchdown_xy_tolerance: 0.02  # swing foot must be this close to target xy (m)
  touchdown_z_tolerance: 0.005  # swing foot must be this close to support foot z (m)

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
| GRF ratio at lift-off | Support foot GRF > 80% mg when entering single support | Per-transition GRF log |
| Lifted foot near-zero GRF | Lifted foot GRF < 5 N when entering single support | Per-transition GRF log |
| CP safety | CP inside convex hull of both foot polygons at all times (or abort triggers correctly) | Per-timestep check |
| Phase duration stability | Step time std dev < 0.15 s | Phase duration log |
| No timeouts | No phase reaches its 5.0 s timeout | Phase duration log |
| Control latency | QP solve < 0.002 s (99th percentile) | Per-step timing log |
| Torque limits | All joint torques < 150 N·m | Per-timestep check |

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Swing foot xy tracking destabilises trunk | Robot falls mid-gait | Time-varying weight: low during transit (0.1), high during descent (1.0); z-only swing task in the QP |
| CoM fails to reach support foot during weight shift | Robot stuck, never transitions | GRF-based transition: if weight doesn't transfer, the 80% threshold never fires; timeout (5.0 s) triggers bipedal-stance fallback |
| GRF measurement noise causes false transitions | Robot enters single support prematurely | Hysteresis: arm at 50%·mg, fire at 80%·mg, with 0.05 s delay since arming |
| Contact wrench discontinuity at lift-off | ~1 cm CP perturbation at phase boundary | Lifted foot GRF must be < 5 N before entering single support |
| Single support minimum duration too short | Swing foot touches down late or at wrong location | `min_single_duration` is conservative (0.5 s); transition requires xy + z + GRF proximity |
| Torque saturation during dynamic weight shift | QP infeasible or large tracking error | Increase `wbc_tau_limit` to 150 N·m if needed; monitor `max_tau` in debug |
| Foot placement on previous step | Self-collision or tripping | Maintain `step_width = 0.20 m`; verify with MuJoCo contact pair exclusions |
| Phase timeout triggered by accumulated error | Walk aborted early | Log phase durations to detect drift; tune GRF thresholds |
| Yaw drift accumulates over steps | Robot walks diagonally, lateral fall | Pelvis yaw regulation task in QP; step direction rotated by current yaw; 6-D contacts during double support resist yaw |
| Swing xy weight too low near touchdown | Foot placement error exceeds 0.02 m tolerance | Weight ramps up during final 40% of swing; if tolerance still not met, phase extends until touchdown |
| CP outside both polygons during gait | Robot falls | CP safety abort: if CP > 0.05 m outside convex hull of both foot polygons, abort step and switch to in-place stepping |
| QP requests impossible yaw torques | Constraint forces diverge from QP expectation | Torsional friction constraint `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` per foot |

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
   - Verify the event-driven FSM with GRF transitions works without falling

2. **In-place stepping** (zero step length, only lift/place)
   - Tests contact scheduling and the GRF-based transition conditions

3. **Small forward steps** (step_length = 0.10 m)
   - 3–5 steps, verify CoM tracking and foot placement error

4. **Triggered step-length adjustment**
   - Implement the CP safety abort: if CP is > 0.05 m outside the convex hull of both foot polygons, set step_length = 0 for that step

5. **Full 5 m walk** (step_length = 0.25 m)
   - Verify GRF transitions fire reliably, no timeouts, all pass/fail criteria met