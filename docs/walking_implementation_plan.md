# Walking Milestone — Implementation Plan

## Branch
`feat/walking`

## Overview

Build a periodic walking gait for G1 on flat ground, reusing the existing QP-WBC
infrastructure.  The deliverable is a `WalkingController` that cycles through a
5-phase FSM, using the architecture defined in `docs/walking_architecture.md`.

The plan is staged: infrastructure → static alternating support → in-place
stepping → small forward steps → full 5 m walk.

---

## Reference Architecture Summary

**5-phase FSM:**
```
BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT →
WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

**Key design decisions (implemented):**
- CP-based footstep planner (CP determines step length — not a fixed constant)
- No CoM task during single support (CoM follows kinematics; CoP inequalities guarantee stability)
- Near-ground swing trajectory (1 cm gap — no contact, no bounce)
- Timer-based transition from SINGLE to DOUBLE_SUPPORT
- Quintic foot-descent profile during DOUBLE_SUPPORT (zero terminal velocity)
- GRF hysteresis for WEIGHT_SHIFT → SINGLE transitions (50% arm / 80% fire)
- GRF contact-loss fallback in all multi-foot phases
- Hard foot constraints with zero acceleration in INIT/DS (no velocity damping)
- Torsional friction constraint: `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` per foot
- Pelvis orientation regulation task (roll, pitch, yaw) in the QP

**Expected gait:** quasi-static, stop-and-go, ~0.25 m/s (~20 s for 5 m).

---

## Stage 0: New Files and Infrastructure ✓ COMPLETE

### 0.1 `planners/footstep_planner.py` ✓

**Responsibility:** Generate swing foot targets in world frame with CP-based
forward adjustment and heading correction.

**Algorithm:**
```python
def plan_step(support_foot_pos, pelvis_yaw, is_right_swing, cp=None):
    step_dir = np.array([cos(pelvis_yaw), sin(pelvis_yaw), 0.0])
    if cp is not None:
        cp_progress = dot(cp[:2] - support_foot_pos[:2], step_dir[:2])
        ankle_progress = np.clip(cp_progress - 0.035, 0.0, self.step_length)
        target_x = support_foot_pos[0] + step_dir[0] * ankle_progress
    else:
        target_x = support_foot_pos[0] + step_dir[0] * self.step_length
    target_y = right_foot_y if is_right_swing else left_foot_y
    return np.array([target_x, target_y, support_foot_pos[2]])
```

### 0.2 `controllers/walking_controller.py` (skeleton) ✓

Inherits from `QPWBCController`.  Overrides:

- `reset()`: initialises phase = `BIPEDAL_INIT`, timers, footstep schedule,
  GRF arming state, descent profile cache
- `compute()`: call `_update_gait_phase()` → build `active_feet` / `com_target` /
  `swing_task` / `pelvis_orientation_target` → call `_solve_qp()`

Helper methods (all implemented):
- `_update_gait_phase()`: evaluate all 7 transition conditions, advance FSM
- `_compute_com_target()`: return setpoint based on current phase
- `_build_phase_tasks()`: active feet, swing tasks, extra tasks per phase
- `_setup_swing_phase()`: CP-based footstep target + quintic swing trajectory
- `_enter_double_support()`: quintic descent profile initialisation
- `_evaluate_descent()`: position/velocity/acceleration from descent profile
- `_check_weight_shift_to_single()`: GRF hysteresis logic
- `_downgrade_foot_contacts()`: GRF-based contact-loss fallback
- `_compute_cp()`: capture point
- `_build_pelvis_orientation_task()`: torso orientation regulation

### 0.3 Config additions ✓

Full walking section synced with `docs/walking_architecture.md` §Configuration
Parameters.

### 0.4 Additional diagnostics ✓

`scripts/debug_qp_consistency.py` — verifies QP model matches MuJoCo dynamics
(MJ consistency = 1.4e-12 verified).

---

## Stage 1: Static Alternating Support ✓ COMPLETE

**Goal:** Verify the 5-phase FSM with GRF transitions works structurally.

**Result:** FSM cycles through all 5 states correctly.  Single-support pelvis
stable with hover approach (RMS roll ~4°, pitch ~1°).  Issues with foot
descent during DOUBLE_SUPPORT identified and resolved in Stages 3–4.

### 1.1 Gait state machine ✓

Full 5-phase FSM with 7 transition rules implemented.

### 1.2 6-D contacts with torsional friction ✓

All double-foot phases use 6-D contact constraints.  Torsional friction
constraint added to `_build_wrench_cones()`.

### 1.3 GRF contact-loss fallback ✓

`_downgrade_foot_contacts()` monitors GRF on all hard-constrained feet during
multi-foot phases and downgrades to soft tracking if GRF < 5 N.

---

## Stage 2: In-Place Stepping with Event-Driven Transitions ✓ COMPLETE

**Goal:** Verify event-driven transitions work without timers.

**Result:** WEIGHT_SHIFT → SINGLE transitions fired reliably via GRF hysteresis.
SINGLE → DOUBLE_SUPPORT changed to timer-based (see Stage 3 findings below).

### 2.2 Contact wrench continuity ✓

GRF liftoff condition (< 5 N) verified before entering single support.

### 2.3 Phase timeout fallback ✓

Implemented 5.0 s timeout per phase → emergency bipedal stance.

---

## Stage 3: Small Forward Steps (step_length = 0.10 m) — IN PROGRESS

**Goal:** Introduce forward progression with CP-based foot placement and pelvis
orientation regulation.  Achieve 2+ steps.

**Status:** 2 steps completed (`min_steps: PASS`).  Pelvis stable during single
support (RMS roll ~4°, pitch ~1°).  Three remaining tuning issues identified.

### 3.1 CP-based footstep planner ✓

Capture point drives forward step length.  CP position relative to support foot
determines ankle placement.  Clamped to kinematic reach (0 to `step_length`).

### 3.2 No CoM task during single support ✓

CoM follows kinematics during single support.  CoP inequality constraints
provide passive stability.  Eliminates task conflict with swing repositioning.

### 3.3 Near-ground swing trajectory ✓

Swing trajectory descends to 1 cm above ground — no contact during single
support → no impact bounce.  Timer-based transition to DOUBLE_SUPPORT.

### 3.4 Quintic foot descent during DOUBLE_SUPPORT ✓

Landing foot descends from ~1 cm to ground via a quintic profile with zero
terminal velocity and acceleration.  Hard constraint with PD tracking.

### 3.5 Remaining tuning items

| Issue | Symptom | Planned fix |
|-------|---------|-------------|
| **Weight-shift timing** | Second GRF transition fails (support < 80% mg at liftoff) | Increase `min_weight_shift_duration` or double-support CoM bias |
| **Support foot slip** | Slip > 0.5 m during weight shift | Add soft XY anchor to support foot during WEIGHT_SHIFT |
| **Descent profile tracking error** | Foot z diverges from quintic profile; kp/kd tuning | Increase PD gains or tighten QP weight hierarchy |

---

## Stage 4: Full 5-Meter Walk (step_length from CP) — NOT STARTED

**Goal:** Achieve the full milestone — walk 5 meters forward using CP-driven
step lengths.

### 4.1 Parameter progression

| Parameter | Stage 3 | Stage 4 | Notes |
|-----------|---------|---------|-------|
| `step_length` | 0.10 m (nominal) | CP-driven up to 0.25 m | CP determines actual length |
| `step_width` | 0.20 m | 0.20 m | Keep constant |
| `step_height` | 0.05 m | 0.05 m | Keep constant |
| `double_support_duration` | 0.80 s | Tuned | May need increase for longer steps |

### 4.2 Termination

Stop when pelvis x ≥ 5.0 m.  Log total time, step count, average step period.

---

## Stage 5: Diagnostics and Hardening — NOT STARTED

### 5.1 `scripts/debug_walking.py`

Multi-panel plots: gait phase timeline, forward displacement, foot placement
vs planned, swing foot z, GRF per foot, CoM tracking error, pelvis orientation,
capture point, torque utilisation, QP solve time histogram.

### 5.2 Optional hardening items

1. **CoM velocity damping** during single support (zero-velocity target)
2. **Swing-leg centroidal momentum feedforward** to offset floating-base constraint
3. **Torsional saturation monitoring** — log `λ_τz` from QP

---

## File Change Summary

| File | Action | Status |
|------|--------|--------|
| `planners/footstep_planner.py` | Rewrite (CP-based) | ✓ |
| `controllers/walking_controller.py` | Major rewrite (~1200 lines) | ✓ (tuning ongoing) |
| `scripts/test_walking.py` | Updated for Stage 3 metrics | ✓ |
| `scripts/debug_qp_consistency.py` | New — QP model verification | ✓ |
| `configs/g1_config.yaml` | Walking section updated | ✓ |
| `docs/walking_architecture.md` | Updated with implemented architecture | ✓ |
| `env/g1_env.py` | No changes required | — |

---

## Transition Logic Reference

```
GRF hysteresis (WEIGHT_SHIFT → SINGLE):
  50% mg  → arm the transition
  80% mg  → fire the transition (after 0.05 s minimum delay since arm)
  < 5 N   → lifting foot GRF precondition

Timer-driven:
  INIT → WEIGHT_SHIFT:      1.0 s  (initial settle)
  SINGLE → DOUBLE_SUPPORT:  0.6 s  (swing completed, foot at 1 cm gap)
  DOUBLE_SUPPORT → WEIGHT_SHIFT: 0.8 s (quintic descent completed)

Timeout:  5.0 s per phase → emergency bipedal stance fallback
Safety:   GRF contact-loss fallback downgrades airborne feet to soft tracking
```
