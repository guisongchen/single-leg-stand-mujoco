# Milestone: Forward Walking on Flat Ground

## Objective

Extend the existing bipedal stance and single-leg balance capabilities into a **periodic walking gait**. The robot must walk 5 meters forward on flat ground, starting from a static bipedal stance, using alternating left/right support phases.

This milestone builds directly on the Phase 1–3 infrastructure (QP-WBC, CoM planners, swing foot trajectories, capture-point switching). No MPC is required; the architecture stays hierarchical with a gait scheduler on top.

---

## Why This Milestone

The single-leg controller already contains all the physics needed for walking:
- **Support phase** = the existing `SINGLE_LEG` controller (CoM + CAM + pelvis regulation)
- **Weight transfer** = the existing `WEIGHT_SHIFT` controller (CoM move + foot unload)
- **Swing tracking** = the existing z-only swing planner (proven not to destabilize)

What is missing is the **temporal sequencing** and **spatial progression**: when to lift which foot, where to place it, and how to advance the CoM target forward step by step.

---

## Technical Approach

### 1. Gait Scheduler (Finite State Machine)

A periodic gait cycles through four phases:

```
BIPEDAL_INIT → LEFT_SINGLE → DOUBLE_SUPPORT → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

| Phase | Duration | Active feet | CoM target | Swing foot |
|-------|----------|-------------|------------|------------|
| `BIPEDAL_INIT` | 1.0 s | Both | Centered between feet | None |
| `LEFT_SINGLE` | 0.5–0.7 s | Left only | Over left foot center | Right foot, forward target |
| `DOUBLE_SUPPORT` | 0.1–0.2 s | Both | Mid-stride forward | None (both grounded) |
| `RIGHT_SINGLE` | 0.5–0.7 s | Right only | Over right foot center | Left foot, forward target |

**State machine rules:**
- Advance to next phase when:
  - Single-leg phase: swing foot has reached its target xy position (within 0.02 m) **and** capture point is inside the support foot polygon **and** minimum duration elapsed
  - Double-support phase: fixed timer expires
- Terminate when total forward displacement ≥ 5.0 m

### 2. Footstep Planner

Simple fixed-schedule planner (no vision, no terrain adaptation):

```
step_length = 0.25 m
step_width  = 0.20 m  (pelvis width, keeps feet under hips)
```

For each step:
- Place the swing foot at `(support_foot_x + step_length, ±step_width/2, 0)`
- The sign alternates left/right
- Foot placement target is computed in world frame before the swing phase begins

**Safety margin:** If the capture point at the end of double-support lies outside the *upcoming* support foot polygon by more than 0.05 m, abort the step and request zero step length (in-place stepping) until stability is recovered.

### 3. CoM Trajectory Generator

Use the existing `ComPlanner` (smoothstep) with dynamically moving targets:

- During `LEFT_SINGLE`: CoM target = left foot center
- During `RIGHT_SINGLE`: CoM target = right foot center
- During `DOUBLE_SUPPORT`: CoM target = midpoint between next support foot and current support foot

The smoothstep duration is set to the phase duration. This is not LIPM-optimal, but it is proven to work with the existing QP-WBC and is computationally free.

**Future upgrade:** Replace smoothstep with an LIPM preview controller that optimizes CoM velocity at the end of each step to minimise acceleration demands.

### 4. Swing Foot Trajectory

Reuse the existing `SwingFootPlanner`:
- Lift height: 0.05 m (higher than single-leg stand to clear ground irregularities)
- Rise duration: 40% of swing phase
- xy tracking: **soft objective only** (same lesson from single-leg: hard xy tracking couples swing leg to trunk and causes oscillation)
- z tracking: hard constraint or high-weight soft objective

### 5. Controller Architecture

```
GaitScheduler
    └── determines: active_feet, phase_duration, swing_target, com_target
        └── calls: TransitionController._solve_qp(...) with updated active_feet & tasks
```

The `TransitionController` base class already supports variable `active_feet`. The walking controller inherits from it and overrides:
- `reset()`: initialise foot placement schedule, gait phase index
- `compute()`: update phase, set active_feet / swing_task / com_target based on gait schedule

---

## Deliverables

| File | Purpose |
|------|---------|
| `controllers/walking_controller.py` | Gait scheduler + walking logic, inherits from `QPWBCController` |
| `planners/footstep_planner.py` | Fixed-schedule foot placement generator |
| `scripts/test_walking.py` | End-to-end test: 5 m walk, metrics, pass/fail |
| `scripts/debug_walking.py` | Phase timing, foot placement error, CP tracking |

---

## Evaluation Criteria (Pass/Fail)

| Metric | Target | Test Method |
|--------|--------|-------------|
| Forward displacement | ≥ 5.0 m | Pelvis x position at end |
| No falls | Pelvis z > 0.5 m, \|roll\| < 15°, \|pitch\| < 15° | Automatic check every step |
| Foot clearance | Swing foot z > 0.03 m during swing | Height log |
| Foot placement error | Actual vs planned foot position < 0.05 m | RMSE at touchdown |
| Gait periodicity | Step time std dev < 0.1 s | Phase duration log |
| Control latency | QP solve < 0.002 s | 99th percentile |

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| Swing foot xy tracking destabilises trunk | Robot falls mid-gait | Use **z-only** hard tracking, xy as low-weight soft objective (same as single-leg) |
| Double-support phase too short | QP infeasible (foot not fully loaded) | Minimum 0.15 s double-support; use CP check before allowing next single-leg phase |
| CoM smoothstep too slow | Robot runs out of step length, late touchdown | Reduce smoothstep duration or increase step length |
| Torque saturation during dynamic walking | QP infeasible | Increase `wbc_tau_limit` to 150 N·m if needed; monitor in `debug_walking.py` |
| Foot placement on previous step | Self-collision or tripping | Maintain step_width = 0.20 m; verify with MuJoCo contact pair exclusions |

---

## Why Not MPC (Yet)

Full nonlinear MPC with contact scheduling would:
- Require mixed-integer optimization or pre-specified gait timings
- Exceed the 2 ms control budget for a 10-step horizon
- Add model-mismatch risk (MuJoCo vs simplified MPC model)

The hierarchical approach (gait scheduler + single-step QP-WBC) is the standard architecture for humanoid walking (e.g., IHMC, MIT DRC). MPC can be added later as a **CoM preview layer** without touching the QP-WBC.

---

## Suggested Development Order

1. **Static alternating support** (no forward motion)
   - Lift right foot, hold, place, lift left foot, hold, place
   - Verify the periodic state machine works without falling
2. **In-place stepping** (zero step length, only lift/place)
   - Tests contact scheduling and double-support timing
3. **Small forward steps** (step_length = 0.10 m)
   - 3–5 steps, verify CoM tracking and foot placement
4. **Full 5 m walk** (step_length = 0.25 m)
   - Tune phase durations and smoothstep timing
