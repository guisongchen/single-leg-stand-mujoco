# Walking Milestone — Implementation Plan

## Branch
`feat/walking`

## Overview

Build a periodic walking gait for G1 on flat ground, reusing the existing QP-WBC infrastructure. The deliverable is a `WalkingController` that cycles through alternating single-support and double-support phases, advancing the CoM target and placing swing feet forward step by step.

The plan is staged: static alternating support → in-place stepping → small forward steps → full 5 m walk.

---

## Stage 0: New Files and Skeleton (no simulation yet)

### 0.1 `planners/footstep_planner.py`

**Responsibility:** Generate a sequence of foot placements in world frame.

**Algorithm:**
```python
def plan_step(support_foot_pos: np.ndarray, step_length: float, step_width: float, is_left: bool) -> np.ndarray:
    """Return swing foot target [x, y, z] in world frame."""
    direction = np.array([1.0, 0.0, 0.0])  # forward
    lateral = np.array([0.0, 1.0, 0.0]) if is_left else np.array([0.0, -1.0, 0.0])
    target = support_foot_pos + step_length * direction + (step_width / 2) * lateral
    target[2] = ground_z
    return target
```

**Config additions:**
```yaml
walking:
  step_length: 0.25
  step_width: 0.20
  step_height: 0.05
  single_support_duration: 0.60
  double_support_duration: 0.15
  init_duration: 1.0
```

### 0.2 `controllers/walking_controller.py` (skeleton)

Inherit from `QPWBCController` (same base as `TransitionController`).

Override:
- `reset()`: initialise gait phase index, footstep schedule, timers
- `compute()`: call `_update_gait_phase()`, then build `active_feet` / `swing_task` / `com_target`, then call `_solve_qp()`

---

## Stage 1: Static Alternating Support (no forward motion, step_length = 0)

**Goal:** Verify the periodic state machine can lift and place each foot without falling.

### 1.1 Gait state machine

States: `INIT → LEFT_SWING → DOUBLE → RIGHT_SWING → DOUBLE → (repeat)`

**Transition rules (hardcoded timers only for Stage 1):**
- `INIT` → `LEFT_SWING` after `init_duration`
- `LEFT_SWING` → `DOUBLE` after `single_support_duration`
- `DOUBLE` → `RIGHT_SWING` after `double_support_duration`
- `RIGHT_SWING` → `DOUBLE` after `single_support_duration`

**Active feet per state:**
| State | active_feet | swing_task |
|-------|-------------|------------|
| INIT | both feet | none |
| LEFT_SWING | left foot only | right foot z-track to lift height |
| DOUBLE | both feet | none |
| RIGHT_SWING | right foot only | left foot z-track to lift height |

**CoM target per state:**
| State | com_target |
|-------|------------|
| INIT | bipedal midpoint (existing `com_start`) |
| LEFT_SWING | left foot center |
| DOUBLE | midpoint of both feet |
| RIGHT_SWING | right foot center |

### 1.2 Swing foot trajectory

Reuse `SwingFootPlanner` with:
- `start` = current swing foot position at phase entry
- `lift_height = step_height`
- `rise_duration = 0.4 * single_support_duration`

Only track **z** (same lesson from single-leg). xy is left free.

### 1.3 Test script

`scripts/test_walking.py` Stage 1 variant:
- Run for 10 seconds (approx 7–8 alternating steps)
- Log: state, foot heights, pelvis tilt, support foot slip
- Pass if no falls and both feet alternate cleanly

---

## Stage 2: In-Place Stepping (step_length = 0)

**Goal:** Add touchdown detection so the state machine advances when the swing foot lands, not just on a timer.

### 2.1 Touchdown detection

```python
def _is_touchdown(self, foot_name: str) -> bool:
    """True if foot body has contact with ground and vertical velocity < 0.01 m/s."""
    fz = self._foot_fz(foot_name)
    vel = self._foot_vert_vel(foot_name)
    return fz > 20.0 and abs(vel) < 0.01
```

**State transition update:**
- `LEFT_SWING` → `DOUBLE` when:
  - `single_support_duration` elapsed **AND** right foot touchdown detected
  - OR `single_support_duration * 1.5` elapsed (safety timeout, force transition)

### 2.2 Why this matters

Timer-only transitions caused the single-leg tip-over in Phase 2 when the timeout fired while the swing foot was still mid-air. The same risk exists here.

---

## Stage 3: Small Forward Steps (step_length = 0.10 m)

**Goal:** Introduce forward progression while keeping dynamics mild.

### 3.1 Footstep planner integration

At the start of each `LEFT_SWING` / `RIGHT_SWING` phase:
1. Compute swing target using `FootstepPlanner.plan_step()`
2. Start a new `SwingFootPlanner` with that target
3. Update CoM target to the *support foot* center (which is now one step behind)

### 3.2 CoM forward shift

The CoM target must lead the support foot slightly so the robot is leaning forward into the next step. Use a **bias**:
```python
com_target = support_foot_center + np.array([0.5 * step_length, 0.0, 0.0])
```

This bias is critical: if the CoM stays directly over the support foot, the robot cannot initiate the next step (zero forward momentum).

### 3.3 Double-support CoM trajectory

During `DOUBLE_SUPPORT`, move the CoM target smoothly from the *previous* support foot to the *next* support foot. Use the existing `ComPlanner` with duration = `double_support_duration`.

---

## Stage 4: Full 5-Meter Walk (step_length = 0.25 m)

**Goal:** Achieve the full milestone.

### 4.1 Parameter tuning

| Parameter | Stage 3 value | Stage 4 target | Tuning rule |
|-----------|---------------|----------------|-------------|
| `step_length` | 0.10 m | 0.25 m | Increase by 0.05 m per test until fall |
| `single_support_duration` | 0.60 s | 0.50–0.60 s | Shorter = faster walking but harder balance |
| `double_support_duration` | 0.15 s | 0.10–0.15 s | Shorter = more dynamic, less stable |
| `wbc_tau_limit` | 100.0 | 100–150.0 | Monitor with `debug_walking.py` |

### 4.2 Capture-point gate for step initiation

Before allowing a state transition from `DOUBLE` → `SINGLE`:
```python
cp = self._compute_capture_point(com_pos, com_vel)
next_support_foot = "left" if current_state == "RIGHT_SWING" else "right"
if not self._is_cp_inside_support_foot(cp, next_support_foot):
    # Extend double-support until CP settles
    return  # do not advance phase
```

This is the same capture-point gate proven in Phase 2, now applied per step.

### 4.3 Termination

Stop the simulation when pelvis x-position ≥ 5.0 m. Log total time, number of steps, and average step period.

---

## Stage 5: Diagnostics and Hardening

### 5.1 `scripts/debug_walking.py`

Plots (8-panel, same style as `debug_single_leg.py`):
1. Gait phase timeline
2. Forward displacement vs time
3. Foot placement: planned vs actual
4. Swing foot height
5. CoM tracking error
6. Support foot slip per step
7. Torque utilisation (max per step)
8. Capture point vs support foot polygon

### 5.2 Push recovery during walking

Extend `scripts/test_push_recovery.py`:
- Apply impulses during `LEFT_SWING` and `RIGHT_SWING`
- Measure whether the robot completes the current step or aborts into an emergency double-support

---

## File Change Summary

| File | Action | Lines (est.) |
|------|--------|--------------|
| `planners/footstep_planner.py` | New | 40 |
| `controllers/walking_controller.py` | New | 250 |
| `scripts/test_walking.py` | New | 200 |
| `scripts/debug_walking.py` | New | 350 |
| `configs/g1_config.yaml` | Add `walking:` section | 10 |
| `controllers/transition_controller.py` | Extract shared helpers to base class (optional refactor) | 0 |

**No changes required to:** `base_qp_wbc.py`, `env/g1_env.py`, `planners/com_planner.py`, `planners/swing_foot_planner.py`, `utils/kinematics.py`.

---

## Critical Assumptions and Fallbacks

1. **Assumption:** The ground is perfectly flat and friction is uniform.
   - **Fallback:** Increase `mu` to 1.0 in config if foot slip is observed.

2. **Assumption:** Step length 0.25 m is within the kinematic reach of G1.
   - **Fallback:** Reduce to 0.20 m or increase hip extension in the reference pose.

3. **Assumption:** The single-leg controller (proven for 30 s static hold) can stabilise each step.
   - **Fallback:** If a step falls, increase `single_support_duration` to give the CoM more time to settle.

4. **Assumption:** Torque limits do not bind during dynamic walking.
   - **Fallback:** Increase `wbc_tau_limit` or reduce step_length (shorter steps = lower accelerations).

---

## Estimated Effort

| Stage | Effort | Cumulative |
|-------|--------|------------|
| 0: Skeleton | 2–3 h | 2–3 h |
| 1: Static alternating | 3–4 h | 5–7 h |
| 2: In-place stepping | 2–3 h | 7–10 h |
| 3: Small forward steps | 4–5 h | 11–15 h |
| 4: Full 5 m walk | 3–4 h | 14–19 h |
| 5: Diagnostics | 2–3 h | 16–22 h |

> Total: **2–3 working days** for a developer familiar with the existing codebase.
