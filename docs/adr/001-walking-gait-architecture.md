# ADR 001: Walking Gait Architecture

## Status

Accepted — Revised (GRF-based transitions, torsional friction, contact ramp-down)

## Context

The G1 robot (23 DOF + 6 floating base, ~34.13 kg) must walk 5 meters forward on flat ground using alternating single/double support phases. The robot has extremely small feet (CoP envelope ±2.5 cm lateral, 12 cm forward, 5 cm backward), which forces a quasi-static gait where the CoM must be nearly over the support foot before the opposite foot can lift.

Previous iterations used a 4-phase FSM with CP-based transition conditions and a midpoint CoM target during double support. Cross-referenced reviews (`docs/review_walking_milestone.md` and `docs/review_walking_milestone_glm.md`) identified 11 compounding issues:

1. **CP timing mismatch** — The 0.05 s minimum guard exceeds the ~0.03 s CP transit time through the ±1.5 cm window, making transitions impossible to trigger reliably
2. **Biased midpoint outside target polygon** — The 70/30 CoM target during double support lies outside the next support foot's CoP polygon, so the CP-based event condition can never fire
3. **Swing-leg angular momentum** — ~2.8 cm CP shift from swing momentum alone exceeds the ±1.5 cm effective CP window
4. **Impossible safety abort** — CP abort condition (CP > 0.05 m outside polygon) can never trigger because the transition condition already requires CP inside ±0.015 m
5. **Missing τ_z constraint** — 6-D contacts need torsional friction limits in the QP
6. **Contact wrench discontinuity** — Instantaneous removal of lifting foot's contact constraints causes ~1 cm CP perturbation
7. **Unstable initial pose** — 1.0 s INIT holds robot in dynamically unstable pose (CoM 5.7 cm forward)
8. **Quintic velocity discontinuity** — Quintic smoothstep at SINGLE entry assumes zero initial velocity, causing overshoot or tracking error
9. **Diagonal shift distance underestimated** — 0.17 m should be 0.23 m (diagonal, not single-axis)
10. **step_width formula error** — Offset from support foot places next foot at midline instead of proper lateral position
11. **No stated walking speed** — Expected ~0.25 m/s, ~20 s for 5 m not documented

The core insight: **CP-based spatial transitions are the wrong abstraction for a robot with ±2.5 cm lateral feet**. The margin is too tight for dynamic effects, and the timing math doesn't work. The solution is to replace CP-based transitions with GRF (ground reaction force) thresholds.

## Decision

### 1. Five-phase FSM with explicit WEIGHT_SHIFT (unchanged)

**Decision:** Add `WEIGHT_SHIFT_L` and `WEIGHT_SHIFT_R` phases between `BIPEDAL_INIT`/`DOUBLE_SUPPORT` and the corresponding `SINGLE_SUPPORT` phases.

**Rationale:**
- Breaks the midpoint deadlock by explicitly driving the CoM to the upcoming support foot center
- GRF on the support foot monotonically increases, making the transition threshold easy to reach
- During `DOUBLE_SUPPORT`, the CoM target is a biased midpoint (70/30 toward the next support foot), starting the transfer

**FSM cycle:**
```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

**Phase targets:**

| Phase | CoM target | Active feet |
|-------|------------|-------------|
| `BIPEDAL_INIT` | Midpoint | Both (6-D) |
| `WEIGHT_SHIFT_L` | Left foot center | Both (6-D) |
| `LEFT_SINGLE` | Left foot center | Left only (6-D) |
| `DOUBLE_SUPPORT` | 70/30 biased midpoint | Both (6-D) |
| `WEIGHT_SHIFT_R` | Right foot center | Both (6-D) |
| `RIGHT_SINGLE` | Right foot center | Right only (6-D) |

**Location:** `controllers/walking_controller.py`

### 2. GRF-based transitions (replaces CP convergence check)

**Decision:** All transitions into single support use ground reaction force (GRF) thresholds with hysteresis, not capture-point geometry.

| Transition | Condition |
|------------|-----------|
| `INIT → WEIGHT_SHIFT_L` | Time: 0.1 s elapsed |
| `WEIGHT_SHIFT_L → LEFT_SINGLE` | GRF on left foot > 80%·mg AND 0.05 s since GRF > 50%·mg AND lifted foot GRF < 5 N |
| `LEFT_SINGLE → DOUBLE_SUPPORT` | Swing foot touchdown (z + xy) AND swing foot GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_R` | Time: 0.15 s elapsed |
| `WEIGHT_SHIFT_R → RIGHT_SINGLE` | GRF on right foot > 80%·mg AND 0.05 s since GRF > 50%·mg AND lifted foot GRF < 5 N |
| `RIGHT_SINGLE → DOUBLE_SUPPORT` | Swing foot touchdown (z + xy) AND swing foot GRF > 10%·mg |
| `DOUBLE_SUPPORT → WEIGHT_SHIFT_L` | Time: 0.15 s elapsed |

**Rationale:**
- **GRF directly measures weight distribution.** No geometric ambiguity, no margin calculation, no velocity convergence check.
- **GRF is monotonically increasing during weight shift.** The 50% arm → 80% fire hysteresis prevents chatter; there is no timing window problem.
- **GRF naturally includes all dynamic effects** (swing momentum, contact impulses). No analytical correction needed.
- **CP conditions are kept only as safety aborts.** If CP > 0.05 m outside the convex hull of both foot polygons at any time, abort the step. CP is useful for detecting catastrophic instability, not for triggering routine transitions.

**Why `DOUBLE_SUPPORT → WEIGHT_SHIFT` is time-driven (0.15 s):** The biased midpoint (70/30) places the CoM outside either individual foot's CoP polygon. A CP-based condition would never fire because CP ≈ CoM at low velocity. The real stability gate is WEIGHT_SHIFT → SINGLE, where the CoM has reached the support foot center. At the DS → WS transition, only confirmation that motion has started is needed, which a short timer provides.

**Why `INIT` is 0.1 s (not 1.0 s):** The initial reference pose is dynamically unstable (CoM 5.7 cm forward of foot midpoint). The QP-WBC engages from the first timestep; a long settle period before motion is counterproductive.

**Design:**
```python
def _check_weight_shift_to_single(self, support_foot, lift_foot):
    grf_support = self._compute_grf(support_foot)
    grf_lift = self._compute_grf(lift_foot)
    mg = self.model.total_mass * 9.81

    # Hysteresis: arm when GRF > 50% mg, fire when > 80% mg
    if grf_support > self.grf_arm_threshold * mg:
        if not self._armed:
            self._arm_time = time.time()
            self._armed = True

    if self._armed and (time.time() - self._arm_time > self.grf_arm_to_fire_delay):
        if grf_support > self.grf_fire_threshold * mg and grf_lift < self.grf_liftoff_threshold:
            return True
    return False
```

**Location:** `controllers/walking_controller.py`, `_update_gait_phase()`

### 3. Pelvis yaw regulation and heading-corrected footsteps (unchanged)

**Decision:** Add a pelvis orientation task (roll, pitch, yaw) to the QP, and rotate step targets by the current pelvis yaw.

**Rationale:**
- Without yaw feedback, 1° of drift per step accumulates to significant lateral deviation over 20 steps
- The pelvis orientation task indirectly constrains trunk angular momentum, stabilizing the capture point during swing
- Step direction rotated by pelvis yaw prevents walking diagonally

**Design:**
```python
# Step target computation — use absolute lateral positions
yaw = extract_yaw(pelvis_quat)
left_foot_y = +step_width / 2   # absolute, not offset from support foot
right_foot_y = -step_width / 2
target_x = support_foot_x + step_length * cos(yaw)
target_y = foot_y  # left_foot_y or right_foot_y, not support_foot_y + offset
```

**Location:** `controllers/walking_controller.py`, `_compute_pelvis_orientation_target()` and `_compute_footstep_target()`

### 4. 6-D contacts with torsional friction constraint (revised)

**Decision:** All double-support and weight-shift phases use 6-D contact constraints (position + orientation) with torsional friction limits in the QP wrench cones.

**Rationale:**
- 3-D position-only contacts provide no yaw constraint — the feet can rotate on the ground
- Over repeated steps, this yaw rotation accumulates and destabilizes the gait
- 6-D contacts resist yaw, roll, and pitch at both feet during weight transfer
- **Without a torsional friction constraint**, the QP optimizer can request physically impossible yaw torques (`τ_z` up to infinity with no bound). For a 5 cm × 17 cm foot at μ = 0.8 and F_z ≈ 170 N, the physical limit is `τ_z_max ≈ 3.4 N·m`.

**New wrench cone constraint:**
```
|λ_τz| ≤ (μ · min(foot_width, foot_length) / 2) · λ_fz
```

This adds two linear inequalities per foot in `_build_wrench_cones()`.

**Location:** `controllers/walking_controller.py`, `_build_wrench_cones()`

### 5. Time-varying swing xy weight (unchanged)

**Decision:** Swing foot xy tracking uses a low weight (0.1) during the first 60% of the swing phase and ramps up to high weight (1.0) during the final 40% (descent).

**Rationale:**
- Low weight mid-swing decouples swing-leg reaction forces from the trunk (~2.8 cm CP shift from swing angular momentum)
- High weight near touchdown ensures the 0.02 m placement tolerance is met
- The phase timeout extends naturally if the foot hasn't reached the target

**Design:**
```python
def _compute_swing_weights(self, phase_progress):
    if phase_progress < 0.6:
        xy_weight = 0.1
    else:
        xy_weight = 0.1 + (1.0 - 0.1) * ((phase_progress - 0.6) / 0.4)
    return xy_weight, 1.0  # (xy_weight, z_weight)
```

**Location:** `controllers/walking_controller.py`, `_compute_swing_weights()`

### 6. PD-only CoM tracking during single support (revised — quintic removed)

**Decision:** During `LEFT_SINGLE` and `RIGHT_SINGLE`, the CoM target is a constant setpoint (support foot center) tracked with PD feedback only. No quintic smoothstep.

**Rationale:**
- The GRF transition condition (80%·mg on support foot) guarantees the CoM is near the support foot center at phase entry. PD convergence from this point is sufficient.
- The quintic smoothstep assumed zero initial velocity, causing a velocity discontinuity when the CoM still has lateral velocity from the weight shift.
- If the weight shift fully converged the CoM to the target, the quintic injects unnecessary acceleration causing overshoot.
- PD-only tracking avoids both failure modes: no velocity discontinuity, no overshoot when converged.

**Location:** `controllers/walking_controller.py`, `_compute_com_target()`

### 7. Contact wrench ramp-down before lift-off (new)

**Decision:** The transition from `WEIGHT_SHIFT` to `SINGLE` requires the lifting foot's GRF to be below 5 N before the phase switch is allowed.

**Rationale:**
- Instantaneous removal of a foot's contact constraint while it still has residual force (e.g., 10 N vertical) creates an acceleration step of ~0.3 m/s², shifting the CP by ~1 cm
- This is significant at the ±1.5 cm effective CP margin scale
- The GRF < 5 N precondition ensures contact wrench continuity at lift-off

**Location:** `controllers/walking_controller.py`, `_check_weight_shift_to_single()`

### 8. Corrected minimum shift distance and expected walking speed (new)

**Decision:** Use the diagonal shift distance (0.23 m, not 0.17 m) for physics calculations, and document the expected walking speed (~0.25 m/s).

**Rationale:**
- The original 0.17 m was the forward component only. The actual minimum shift between CoP envelopes is the diagonal: `√(0.17² + 0.15²) ≈ 0.23 m`
- This yields minimum double-support times of 0.41 s (quintic) and 0.34 s (bang-bang), not 0.35 s and 0.29 s
- The gait is stop-and-go (CoM velocity → 0 during each single-support phase). Expected speed: `0.25 m / (0.5 + 0.4 + 0.15) ≈ 0.25 m/s`, total time for 5 m: ~20 s

**Location:** `docs/walking_milestone_task.md`, `configs/g1_config.yaml`

## Consequences

### Positive

- **Reliable transitions:** GRF thresholds are monotonically increasing during weight shift, eliminating the timing mismatch that made CP-based transitions impossible to trigger
- **No margin dependency:** Transitions don't depend on CP being inside a ±1.5 cm window. The QP's hard CoP constraints handle geometry; the gait scheduler only needs force measurements
- **Contact wrench continuity:** Lift-off precondition (GRF < 5 N) prevents impulse perturbations
- **Yaw stability:** Pelvis orientation task + 6-D contacts + heading-corrected footsteps eliminate accumulated yaw drift
- **Accurate foot placement:** Time-varying swing weight resolves the stability-vs-accuracy tradeoff without a fixed compromise
- **Graceful degradation:** Phase timeouts fall back to bipedal stance rather than indefinite hangs; CP safety abort catches catastrophic instability
- **No quintic overshoot:** PD-only CoM tracking during single support avoids velocity discontinuity and overshoot at phase boundaries

### Negative

- **GRF measurement dependency:** Transitions require reliable force/torque sensing or contact force estimation. MuJoCo provides `mj_contactForce`, but real hardware would need F/T sensors or contact force estimation from joint torques.
- **Longer gait cycle:** Two additional phases (WEIGHT_SHIFT per side) increase the total cycle time by ~0.4 s per step
- **More state machine complexity:** Five phases with mixed time-driven and GRF-driven transitions require careful implementation
- **6-D contacts constrain more DOFs:** During double support, the QP has 6 additional hard constraints per foot plus torsional friction limits, reducing the feasible solution space

## Alternatives Considered

### Alternative 1: Timer-based double support with timed CoM trajectory

Use a fixed-duration double-support phase with a quintic CoM trajectory from one foot to the other.

**Rejected:**
- Requires knowing the weight-shift duration in advance, which depends on friction, tracking error, and CoM position
- Minimum durations of 0.34–0.41 s leave no margin for error
- Event-driven switching adapts to actual robot state rather than assuming ideal timing

### Alternative 2: Capture-point transitions with post-entry latch (patch to original design)

Keep CP-based transitions but replace the fixed-from-start guard with a post-entry latch: once CP enters the polygon AND lateral velocity is converging, arm the transition for 0.1 s.

**Rejected:**
- Swing-leg angular momentum (~2.8 cm CP shift) can push the CP outside the ±1.5 cm window during single support, even after the latch fires
- CP-based checks require geometric margin analysis and velocity convergence verification; GRF thresholds are simpler and more robust
- The biased midpoint during double support places the CoM outside the target polygon, making the DS → WS CP condition impossible

### Alternative 3: Capture-point MPC preview controller

Add a linear inverted pendulum model MPC layer that plans CoM trajectories over a 1–2 second horizon.

**Rejected:**
- Exceeds the 2 ms control budget for reasonable horizons
- Adds model-mismatch risk (LIPM vs full MuJoCo dynamics)
- The hierarchical approach (event-driven scheduler + QP-WBC) is sufficient for flat-ground walking
- Can be added later as a CoM preview layer without changing the gait scheduler

## Implementation Notes

### Dependencies

No new Python packages. The walking controller reuses the existing QP-WBC infrastructure (`osqp`, `numpy`, `mujoco`).

### GRF computation

MuJoCo provides contact forces via `mj_contactForce()`. The GRF for each foot is computed by summing the vertical (z) component of all contact forces on that foot's geoms:

```python
def _compute_grf(self, foot_body_id):
    grf_z = 0.0
    for i in range(self.data.ncon):
        contact = self.data.contact[i]
        # Check if contact involves this foot's geom
        if self._contact_involves_foot(contact, foot_body_id):
            forces = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, forces)
            # Transform to world frame vertical component
            grf_z += self._world_vertical_force(contact, forces)
    return grf_z
```

### Testing Strategy

```bash
# Stage 1: Static alternating support (no forward motion)
python scripts/test_walking.py --step-length 0 --duration 10

# Stage 2: In-place stepping (test FSM transitions + GRF checks)
python scripts/test_walking.py --step-length 0 --duration 15

# Stage 3: Small forward steps
python scripts/test_walking.py --step-length 0.10 --duration 10

# Stage 4: Full 5 m walk
python scripts/test_walking.py --step-length 0.25 --target-distance 5.0

# Diagnostics
python scripts/debug_walking.py
```

### Key configuration parameters

```yaml
walking:
  step_length: 0.25
  step_width: 0.20
  step_height: 0.05
  min_single_duration: 0.50
  init_duration: 0.1             # reduced from 1.0 s (unstable initial pose)
  double_support_duration: 0.15  # replaces min_double_guard
  phase_timeout: 5.0
  forward_direction: [1.0, 0.0]
  double_support_com_bias: 0.70

transitions:
  grf_arm_threshold: 0.50        # arm when support foot GRF > 50% mg
  grf_fire_threshold: 0.80        # fire when support foot GRF > 80% mg
  grf_arm_to_fire_delay: 0.05     # minimum delay between arm and fire
  grf_touchdown_threshold: 0.10   # swing foot GRF > 10% mg confirms contact
  grf_liftoff_threshold: 5.0      # lifted foot GRF must be < 5 N
  cp_abort_margin: 0.05           # abort if CP > 0.05 m outside both polygons

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

## References

- `docs/walking_milestone_task.md` — Full milestone specification with physics constraints and evaluation criteria (updated for GRF transitions)
- `docs/review_walking_milestone.md` — Original review identifying 8 pitfalls
- `docs/review_walking_milestone_glm.md` — Cross-referenced review with integrated solution (GRF-based transitions)
- `configs/g1_config.yaml` — Robot parameters, initial pose, control gains