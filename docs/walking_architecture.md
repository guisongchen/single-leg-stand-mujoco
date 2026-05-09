# Walking Gait: Architecture and Specification

## Objective

Extend the existing bipedal stance and single-leg balance capabilities into a
**periodic walking gait**.  The robot must walk 5 metres forward on flat ground,
starting from a static bipedal stance, using alternating left/right support
phases.

This milestone builds directly on the Phase 1–3 infrastructure (QP-WBC, CoM
planners, swing foot trajectories).  No MPC is required; the architecture stays
hierarchical with a gait scheduler on top.

---

## Physics Constraints

All phase timing and transition rules are derived from the physical limits of
the G1 robot.

### Friction force limit

Maximum horizontal ground reaction force:
`F_h_max = μ · m · g = 0.8 × 34.13 × 9.81 ≈ 268 N`

Maximum CoM linear acceleration (both feet or single foot — the limit is `μ·g`
in either case):

```
a_com_max = μ · g = 7.85 m/s²
```

### CoP envelope per foot

The four corner-sphere geoms per foot form a rectangular CoP envelope in the
foot's local frame:

| Direction | Offset from body origin | Bound |
|-----------|------------------------|-------|
| Forward (toe) | +12 cm | `cop_x_forward = 0.12 m` |
| Backward (heel) | –5 cm | `cop_x_back = 0.05 m` |
| Lateral (each side) | ±2.5 cm | `cop_y_half = 0.025 m` |

These are hard inequality constraints in the QP (`_build_wrench_cones`).  They
restrict what torques the optimizer can command, **not** when gait transitions
happen.

### Foot geometry: 4 discrete sphere contacts

The G1 foot is NOT a flat plate.  Four 5 mm radius spheres at
(−5 cm, ±2.5 cm) and (+12 cm, ±3 cm) in the foot frame form a discrete
point-contact system.  See `docs/g1_foot_constrains.md` for the full analysis.
Key consequences:

- Contact transitions are sharper than a flat foot — partial lift-off of one
  sphere reduces the effective support polygon.
- Touchdown typically involves a single sphere making first contact, creating
  an unwanted moment impulse.
- The architecture's response is a **near-ground trajectory** (§4) that
  avoids contact during single support and a **zero-terminal-velocity
  descent** (§4b) during double support.

### Consequence: quasi-static gait

The 2.5 cm lateral CoP envelope per foot forces a **quasi-static** gait.  The
CoM must be nearly over the support foot centre before the opposite foot can
lift.

Expected walking speed is approximately **0.25 m/s** (~20 s for 5 m), since
forward CoM velocity effectively stops during each single-support phase and
forward motion occurs mainly during weight-transfer phases.

### Contact wrench continuity at lift-off

When transitioning from double to single support, the QP removes one foot's
contact constraints.  If that foot has residual contact force (e.g. 10 N
vertical) at lift-off, the sudden removal creates an acceleration step of
~0.3 m/s², shifting the CP by ~1 cm.  To prevent this, transitions into
single support require the lifting foot's GRF to be below a threshold (see
transition rules below).  Additionally, a GRF-based contact-loss fallback
automatically downgrades any hard-constrained foot whose GRF drops below the
liftoff threshold, preventing QP infeasibility.

---

## Technical Approach

### 1. Gait Scheduler (Event-Driven FSM)

A periodic gait cycles through five phases:

```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT →
WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

| Phase | Active feet | CoM target | Swing foot |
|-------|-------------|------------|------------|
| `BIPEDAL_INIT` | Both (6-D) | Centred between feet | None |
| `WEIGHT_SHIFT_L` | Both (6-D) | Left foot centre | None (both grounded) |
| `LEFT_SINGLE` | Left only (6-D) | **None** (CoM follows kinematics, §3) | Right foot, near-ground target |
| `DOUBLE_SUPPORT` | Both (6-D) | Biased toward next support foot | None (landing foot descends via quintic profile, §4b) |
| `WEIGHT_SHIFT_R` | Both (6-D) | Right foot centre | None (both grounded) |
| `RIGHT_SINGLE` | Right only (6-D) | **None** (CoM follows kinematics, §3) | Left foot, near-ground target |

Both-feet phases use **6-D contacts** (position + orientation).  During double
support the robot must resist yaw rotation of the feet on the ground; 3-D
position-only contacts provide no yaw constraint.

#### Transition rules (implemented)

| Transition | Condition |
|------------|-----------|
| `INIT → WEIGHT_SHIFT_L` | Time: 1.0 s elapsed (initial settle, break symmetry) |
| `WEIGHT_SHIFT_L → LEFT_SINGLE` | GRF on left > 80%·mg AND 0.05 s since > 50%·mg AND right GRF < 5 N |
| `LEFT_SINGLE → DOUBLE_SUPPORT` | **Time: swing_duration elapsed** (foot at 1 cm gap — no contact to detect) |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` | Time: 0.8 s elapsed (foot has completed quintic descent) |
| `WEIGHT_SHIFT_R → RIGHT_SINGLE` | GRF on right > 80%·mg AND 0.05 s since > 50%·mg AND left GRF < 5 N |
| `RIGHT_SINGLE → DOUBLE_SUPPORT` | Time: swing_duration elapsed |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_L` | Time: 0.8 s elapsed |

**Timeout for every phase:** 5.0 s.  If a transition condition is not met
within the timeout, the controller degrades to `BIPEDAL_INIT` (both feet
hard, CoM centred).

**Termination:** when total forward pelvis displacement ≥ 5.0 m.

#### GRF contact-loss fallback

During any multi-foot phase (`INIT`, `WEIGHT_SHIFT`, `DOUBLE_SUPPORT`), if a
foot's vertical GRF drops below the liftoff threshold (5 N), its hard
constraint is automatically downgraded to soft 3-D position tracking.  This
prevents QP infeasibility when a foot loses physical contact while the QP
still enforces a hard holonomic constraint on it.

### 2. Footstep Planner (CP-based)

The footstep planner uses the **capture point** to determine how far forward
to step, replacing the fixed `step_length` parameter.  The CP-CoP
relationship is the fundamental constraint connecting foot placement to CoM
dynamics.

**Algorithm:**

```python
def plan_step(support_foot_pos, pelvis_yaw, is_right_swing, cp):
    step_dir = [cos(yaw), sin(yaw), 0]
    # Progress of CP ahead of support foot along walking direction
    cp_progress = dot(cp[:2] − support_foot[:2], step_dir[:2])
    # Place ankle so the CoP centre (3.5 cm ahead of body origin)
    # sits under the CP.  Clamp: never step backward, ≤ nominal length.
    ankle_progress = clamp(cp_progress − 0.035, 0.0, step_length)
    target_x = support_foot_x + step_dir_x * ankle_progress
    target_y = right_foot_y if is_right_swing else left_foot_y  # absolute
    return [target_x, target_y, support_foot_z]
```

**Heading correction** rotates the step direction by the current pelvis yaw.

When no CP is available, falls back to a fixed-length step (nominal
`step_length` from config).

### 3. CoM Target Strategy

The CoM target is **phase-dependent** and reflects whether the CoM is
kinematically free (double-foot phases) or determined by foot positions
(single-support phases).

- **During `BIPEDAL_INIT`:** CoM target = static midpoint between feet.

- **During `WEIGHT_SHIFT_L/R`:** CoM target = centre of upcoming support
  foot, reached via a quintic `ComPlanner` trajectory (2.5 s).  The CoM
  task is active (weight 100) because both feet are hard-constrained and
  the floating base has 6 free DOF — the CoM position IS a choice.

- **During `DOUBLE_SUPPORT`:** CoM target = biased midpoint: 70% toward
  next support foot, 30% toward current.  Promotes passive weight transfer
  that presses the landing foot into the ground.

- **During `LEFT_SINGLE` / `RIGHT_SINGLE`: No CoM task.**  With one foot
  hard-constrained, the remaining kinematic DOFs are needed to reposition
  the swing leg.  The CoM position is **kinematically determined** by the
  two foot positions and there is no redundancy to control it independently.
  The QP's hard CoP inequality bounds keep the CoM inside the support
  polygon without an explicit tracking objective.  Adding a point-target
  CoM task during single support would compete with the swing-foot
  repositioning for the same finite kinematic budget and cause the QP to
  abandon the swing trajectory.

### 4. Swing Foot Trajectory (near-ground, timer-based)

The swing foot follows a quintic trajectory (`SwingTrajectoryPlanner`) that:

- Starts at the foot's current world position (including any elevation from
  the preceding weight-shift pre-load)
- Lifts by `step_height` (0.05 m) to clear the ground
- Descends to **1 cm above ground level** — not all the way

**Why the 1 cm gap:** A soft-constrained (finite-weight) descent to ground
level creates a violent impact bounce that destabilises the pelvis.  By
stopping the descent 1 cm above ground, the foot never makes contact during
single support.  No contact means no bounce.

**Transition:** Timer-based at `swing_duration` elapsed.  No GRF-based or
kinematic touchdown detection is needed because the foot is still 1 cm
above ground at the end of the swing.

**XY tracking:** Disabled during single support.  The foot's lateral and
forward repositioning is determined by the CP-based footstep planner at
the start of the swing phase; the quintic trajectory provides the
position reference.  Active XY tracking in the QP would couple swing-leg
momentum into the trunk and create unreconcilable task conflicts with the
pelvis orientation task.

### 4b. Foot Descent During Double Support (quintic profile)

The final ~1 cm descent happens **during `DOUBLE_SUPPORT`** under a hard
constraint with a quintic descent profile — no soft tracking, no impact
bounce.

At `DOUBLE_SUPPORT` entry, a quintic polynomial is fitted to the boundary
conditions:

```
z(0)    = z_start (current foot height, ~1 cm above ground)
vz(0)   = vz_start (current foot z-velocity)
az(0)   = 0
z(T)    = ground_z
vz(T)   = 0        ← zero terminal velocity eliminates impact
az(T)   = 0        ← zero terminal acceleration eliminates discontinuity
```

where T = `double_support_duration` (0.8 s).  The hard constraint for the
landing foot's Z is then:

```
J_foot[2] · qacc = a_des(t) + kp·(z_des(t) − z) + kd·(vz_des(t) − vz)
```

The PD gains (kp=200, kd=60) provide tracking stiffness, while the quintic
feed-forward terms guarantee near-zero velocity at touchdown.

### 5. Pelvis Orientation Regulation

Angular momentum of the trunk and swing leg directly affects the capture
point during single support.  The QP includes a pelvis orientation task
that regulates roll, pitch, and yaw toward upright.  Weight: 50
(TransitionController-level tuning), applied in all phases except
`BIPEDAL_INIT`.

### 6. Controller Architecture

```
GaitScheduler
    └── evaluates GRF thresholds, foot position, phase timers
        └── determines: active_feet, com_target, swing_task,
                        pelvis_orientation_target
            └── calls: _solve_qp(...) with updated active_feet & tasks
```

The walking controller extends `QPWBCController` and manages:
- `_update_gait_phase()`: evaluate all transition conditions, advance FSM
- `_compute_com_target()`: setpoint or none, depending on phase
- `_compute_swing_weights()`: time-varying swing-xy weight (unused in
  current architecture — XY tracking disabled)
- `_build_phase_tasks()`: assembles active feet, swing tasks, and soft
  objectives per phase
- `_setup_swing_phase()`: initialises CP-based footstep target and
  quintic swing trajectory
- `_enter_double_support()`: initialises quintic descent profile
- `_downgrade_foot_contacts()`: GRF-based fallback for multi-foot phases
- `compute()`: assembles active feet, swing task, pelvis orientation
  task, calls QP, recovers torques

### 7. QP Wrench Cone Additions

The existing `_build_wrench_cones()` includes torsional friction
constraints for 6-D contacts:

```
|λ_τz| ≤ (μ · min(foot_width, foot_length) / 2) · λ_fz
```

Torsional constraints are **skipped during single-support phases** to
avoid binding further against the narrow friction budget.

---

## Deliverables

| File | Purpose | Status |
|------|---------|--------|
| `controllers/walking_controller.py` | Gait scheduler + walking logic | Implemented (2 steps), tuning ongoing |
| `planners/footstep_planner.py` | CP-based foot placement generator | Implemented |
| `scripts/test_walking.py` | End-to-end test: metrics, pass/fail | Implemented |
| `scripts/debug_qp_consistency.py` | QP model verification diagnostics | Implemented |

---

## Configuration Parameters

```yaml
walking:
  step_length: 0.10             # nominal forward distance per step (m) — CP may adjust
  step_width: 0.20              # lateral distance between feet (m)
  step_height: 0.05             # swing foot clearance above ground (m)
  swing_duration: 0.60          # total seconds for lift + near-ground descent
  swing_lift_duration: 0.36     # seconds before foot descends toward near-ground
  min_single_duration: 0.50     # minimum seconds in single support
  init_duration: 1.0            # seconds in INIT before first weight shift
  double_support_duration: 0.80 # seconds for quintic foot descent
  phase_timeout: 5.0            # seconds before emergency bipedal fallback
  forward_direction: [1.0, 0.0] # world-frame walking direction (xy only)
  double_support_com_bias: 0.70 # fraction toward next support foot in DS

transitions:
  grf_arm_threshold: 0.50       # fraction of mg to arm weight-shift transition
  grf_fire_threshold: 0.80      # fraction of mg to fire weight-shift transition
  grf_arm_to_fire_delay: 0.05   # minimum seconds between arm and fire
  grf_touchdown_threshold: 0.10 # (unused in current timer-based touchdown)
  grf_liftoff_threshold: 5.0    # N; lifted foot must be below this to enter SINGLE
  cp_abort_margin: 0.05         # (unused — CP is now a planning input)

safety:
  touchdown_xy_tolerance: 0.02  # m from target xy (unused in timer-based mode)
  touchdown_z_tolerance: 0.005  # m from ground (unused in timer-based mode)

swing_weights:
  xy_early: 0.1                 # (unused — XY tracking disabled)
  xy_late: 1.0                  # (unused — XY tracking disabled)
  z: 200.0                      # swing-z QP weight (clearance priority)

pelvis_orientation:
  roll_weight: 0.5              # (overridden by transition.pelvis_weight = 50)
  pitch_weight: 0.5             # (overridden)
  yaw_weight: 0.3               # (overridden)
  target_yaw: 0.0               # desired pelvis yaw (rad)
```

---

## Evaluation Criteria (Pass/Fail)

| Metric | Target | Stage 3 status |
|--------|--------|----------------|
| Forward displacement | ≥ 5.0 m (Stage 4) / ≥ 0.05 m (Stage 3) | FAIL |
| No falls | Pelvis z > 0.5 m, |roll| < 15°, |pitch| < 15° | FAIL (falls during weight shift) |
| Pelvis stable during single | RMS |roll| < 5°, |pitch| < 5° | **PASS** (~4°/1°) |
| Yaw drift | Cumulative < 2° per step | FAIL |
| Foot clearance | > 0.02 m | FAIL |
| Foot placement error | RMSE < 0.05 m | FAIL |
| Support foot slip | < 0.005 m | FAIL |
| GRF ratio at lift-off | Support foot > 80%·mg | FAIL (second transition) |
| Lifted foot GRF at lift-off | < 5 N | FAIL |
| Min steps | ≥ 2 (Stage 3) | **PASS** (2 steps) |

---

## Risk Assessment

| Risk | Mitigation |
|------|------------|
| Swing foot impact bounce during soft descent | Near-ground trajectory (1 cm gap) — no contact during single support |
| Landing foot impact bounce during DS | Quintic descent profile with zero terminal velocity |
| CoM task competes with swing repositioning | No CoM task during single support; CoP inequality bounds provide passive stability |
| Hard foot constraint locks airborne foot after bounce → QP infeasible | GRF contact-loss fallback — downgrades to soft tracking in all multi-foot phases |
| QP λ mismatches MuJoCo spring forces during descent | Quintic profile guarantees low contact velocity; residual mismatch handled by PD tracking |
| Torque limits prevent foot repositioning | Pelvis orientation weight balanced against swing task |

---

## Why Not MPC (Yet)

Full nonlinear MPC with contact scheduling would:
- Require mixed-integer optimisation or pre-specified gait timings
- Exceed the 2 ms control budget for a 10-step horizon
- Add model-mismatch risk (MuJoCo vs simplified MPC model)

The hierarchical approach (CP-driven gait scheduler + single-step QP-WBC) is
the standard architecture for humanoid walking (e.g. IHMC, MIT DRC).  MPC can
be added later as a **CoM preview layer** without touching the QP-WBC.

---

## Suggested Development Order

1. **Static alternating support** (no forward motion) — *complete*
2. **In-place stepping** (zero step length) — *complete*
3. **Small forward steps** (step_length = 0.10 m, 2+ steps) — *2 steps achieved, tuning in progress*
4. **Full 5 m walk** (step_length from CP) — *not started*
5. **Diagnostics and hardening** — *not started*
