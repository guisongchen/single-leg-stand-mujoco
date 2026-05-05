# ADR 001: Walking Gait Architecture

## Status

Accepted

## Context

The G1 robot (23 DOF + 6 floating base, ~34.13 kg) must walk 5 meters forward on flat ground using alternating single/double support phases. The robot has extremely small feet (CoP envelope ±2.5 cm lateral, 12 cm forward, 5 cm backward), which forces a quasi-static gait where the CoM must be nearly over the support foot before the opposite foot can lift.

Previous iterations used a 4-phase FSM (`BIPEDAL_INIT → LEFT_SINGLE → DOUBLE_SUPPORT → RIGHT_SINGLE → repeat`) with a midpoint CoM target during double support. Eight theoretical pitfalls were identified in this design:

1. **Midpoint CoM deadlock** — the capture point never enters either foot's narrow polygon from the neutral midpoint
2. **CP transition margin too tight** — ±1 cm effective lateral clearance is insufficient for a dynamic gait
3. **No yaw correction** — pelvis yaw drifts over 20+ steps with no feedback
4. **INIT symmetry deadlock** — the gait never starts because the CoM sits at the midpoint after settling
5. **Swing xy weight tradeoff** — low weight avoids trunk coupling but sacrifices placement accuracy
6. **Unregulated angular momentum** — swing leg reaction torques shift the capture point
7. **Double-support yaw drift** — 3-D position-only contacts provide no yaw constraint
8. **Quintic ends before touchdown** — CoM enters PD-only regime while swing foot still in air

## Decision

### 1. Five-phase FSM with explicit WEIGHT_SHIFT

**Decision:** Add `WEIGHT_SHIFT_L` and `WEIGHT_SHIFT_R` phases between `BIPEDAL_INIT`/`DOUBLE_SUPPORT` and the corresponding `SINGLE_SUPPORT` phases.

**Rationale:**
- Breaks the midpoint deadlock by explicitly driving the CoM to the upcoming support foot center
- CP naturally enters the support foot's polygon because the CoM target is already there
- During `DOUBLE_SUPPORT`, the CoM target is a biased midpoint (70/30 toward the next support foot), starting the transfer so the CoM is already moving when `WEIGHT_SHIFT` begins

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

### 2. CP convergence check on transitions

**Decision:** All transitions into single support require the capture point to be inside the support foot's CoP polygon with a reduced margin (0.01 m) AND have lateral velocity converging toward the polygon center.

**Rationale:**
- The ±2.5 cm lateral CoP envelope leaves only ±1.5 cm effective CP window with 0.01 m margin — too tight for a position-only check
- A velocity criterion prevents triggering on a momentary CP excursion where the CoM is still decelerating
- The convergence check requires `v_cp_lateral` to point toward the polygon center, not away from it

**Design:**
```python
def _cp_converging_toward_polygon(cp, cp_vel, polygon_center, foot_local_y):
    lateral_offset = cp_local_y - polygon_center_y
    lateral_vel = cp_vel_local_y
    # CP must be moving toward polygon center in the lateral direction
    if lateral_offset > 0:
        return lateral_vel < 0  # offset positive, velocity must be negative
    else:
        return lateral_vel > 0
```

**Location:** `controllers/walking_controller.py`, `_update_gait_phase()`

### 3. Pelvis yaw regulation and heading-corrected footsteps

**Decision:** Add a pelvis orientation task (roll, pitch, yaw) to the QP, and rotate step targets by the current pelvis yaw.

**Rationale:**
- Without yaw feedback, 1° of drift per step accumulates to significant lateral deviation over 20 steps
- The pelvis orientation task indirectly constrains trunk angular momentum, stabilizing the capture point during swing
- Step direction rotated by pelvis yaw prevents walking diagonally

**Design:**
```python
# Step target computation
yaw = extract_yaw(pelvis_quat)
target_x = support_foot_x + step_length * cos(yaw)
target_y = support_foot_y + step_length * sin(yaw)

# QP task weights
pelvis_orientation:
  roll_weight: 0.5    # upright torso
  pitch_weight: 0.5    # upright torso
  yaw_weight: 0.3      # allow slow turning, resist drift
```

**Location:** `controllers/walking_controller.py`, `_compute_pelvis_orientation_target()` and `_compute_footstep_target()`

### 4. 6-D contacts for both-feet phases

**Decision:** All double-support and weight-shift phases use 6-D contact constraints (position + orientation), not 3-D position-only.

**Rationale:**
- 3-D position-only contacts provide no yaw constraint — the feet can rotate on the ground
- Over repeated steps, this yaw rotation accumulates and destabilizes the gait
- 6-D contacts resist yaw, roll, and pitch at both feet during weight transfer

**Location:** `controllers/walking_controller.py`, `_solve_qp()` active_feet construction

### 5. Time-varying swing xy weight

**Decision:** Swing foot xy tracking uses a low weight (0.1) during the first 60% of the swing phase and ramps up to high weight (1.0) during the final 40% (descent).

**Rationale:**
- Low weight mid-swing decouples swing-leg reaction forces from the trunk
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

### 6. CoM reference holds after quintic completion

**Decision:** After the `ComPlanner` quintic smoothstep completes (at `min_single_duration`), the CoM target holds at the support foot center and the QP tracks it with PD-only feedback.

**Rationale:**
- The quintic provides feedforward acceleration during the first 0.5 s when the CoM is still in transit
- After convergence, PD-only tracking is sufficient because the CoM is near the support foot center
- No special handling needed — the target is a constant setpoint after the smoothstep ends

**Location:** `controllers/walking_controller.py`, `_compute_com_target()`

## Consequences

### Positive

- **Reliable transitions:** WEIGHT_SHIFT phases guarantee the CP enters the target polygon instead of stalling at the midpoint
- **Yaw stability:** Pelvis orientation task + 6-D contacts + heading-corrected footsteps eliminate accumulated yaw drift
- **Accurate foot placement:** Time-varying swing weight resolves the stability-vs-accuracy tradeoff without a fixed compromise
- **Robust CP checks:** Velocity convergence criterion prevents premature or spurious transitions
- **Graceful degradation:** Phase timeouts fall back to bipedal stance rather than indefinite hangs

### Negative

- **Longer gait cycle:** Two additional phases (WEIGHT_SHIFT per side) increase the total cycle time by the weight-shift duration (~0.35 s per step)
- **More state machine complexity:** Five phases instead of four, with more transition conditions to debug
- **6-D contacts constrain more DOFs:** During double support, the QP has 6 additional hard constraints (3 orientation per foot), which reduces the feasible solution space and may cause infeasibility if the robot is significantly misaligned
- **CP margin is still tight:** Even at 0.01 m, the effective lateral window is only ±1.5 cm — this is near the limit of what the G1 foot geometry allows

## Alternatives Considered

### Alternative 1: Timer-based double support with timed CoM trajectory

Use a fixed-duration double-support phase with a quintic CoM trajectory from one foot to the other.

**Rejected:**
- Requires knowing the weight-shift duration in advance, which depends on friction, tracking error, and CoM position
- If timer is too short, the CoM doesn't reach the next polygon; if too long, the gait is sluggish
- The `a_com_max` calculation shows minimum durations of 0.29–0.35 s, leaving no margin for error
- Event-driven switching adapts to actual robot state rather than assuming ideal timing

### Alternative 2: Single double-support phase with setpoint midpoint (original design)

Keep the 4-phase FSM with midpoint CoM target during double support, relying purely on the QP to shift weight.

**Rejected:**
- CP never enters either foot's ±2.5 cm polygon from the midpoint with low velocity — the transitions deadlock
- No mechanism to break symmetry after INIT
- 3-D contacts allow yaw drift

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

### Testing Strategy

```bash
# Stage 1: Static alternating support (no forward motion)
python scripts/test_walking.py --step-length 0 --duration 10

# Stage 2: In-place stepping (test FSM transitions + CP checks)
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
  min_double_guard: 0.05
  init_duration: 1.0
  phase_timeout: 5.0
  forward_direction: [1.0, 0.0]
  double_support_com_bias: 0.70

safety:
  cp_polygon_margin: 0.01
  cp_velocity_converging: true
  touchdown_xy_tolerance: 0.02
  touchdown_z_tolerance: 0.005
  abort_cp_margin: 0.05

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

- `docs/walking_milestone_task.md` — Full milestone specification with physics constraints and evaluation criteria
- `docs/walking_implementation_plan.md` — Staged implementation plan (Stages 0–5)
- `docs/walking_controller_step_plan.md` — Step-by-step debugging and implementation guide
- `configs/g1_config.yaml` — Robot parameters, initial pose, control gains
- `controllers/bipedal_stance_controller.py` — Phase 1 QP-WBC controller (base class)