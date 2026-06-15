# Walking Simulation Architecture (Top-Down)

## Level 1: Test Orchestration (`test_walking.py:main`)

Loads config → creates `G1Env` → creates `WalkingController` → calls `run_simulation()` → `assess()` → `plot()`

- `run_simulation`: steps env + controller for 10s, logs gait metrics
- `assess`: computes pass/fail from logs (roll/pitch, foot clearance, slip, steps, GRF hysteresis)
- `plot`: saves 3×2 diagnostic figure

## Level 2: Controller Entry (`WalkingController.compute`)

Called each tick (500 Hz). Seven steps:

```
FSM Update → Kinematics → CoM Target → Landing Chase → Task Targets → Phase Tasks → QP Solve → τ
```

## Level 3: Four-Layer Architecture

### Layer 1 — Gait FSM
- **`_update_gait_phase()`**: 5-state machine (BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → RIGHT_SINGLE → ...)
- **Transitions**: timer-driven (INIT, DOUBLE_SUPPORT), GRF-hysteresis + CoM proximity (WEIGHT_SHIFT→SINGLE), kinematic touchdown detector (SINGLE→DOUBLE)
- **Safety**: 5s timeout → `_emergency_bipedal_stance()`
- **Output**: `_phase`, `_support_foot_name`, `_swing_foot_name`, `_phase_start_time`

### Layer 2 — Motion Planning
- **FootstepPlanner**: geometric target `[support_x + step_length, ±step_width/2, ground_z]`. With `step_length=0`, target x = support x.
- **SwingTrajectoryPlanner**: quintic spline (40% rise → 20% hold → 40% descent), evaluated at `dt_phase`. Fixed at swing entry.
- **ComPlanner**: quintic CoM interpolation from current → support foot centre over `t_weight_shift`. Active only during WEIGHT_SHIFT.

All planners are **feedforward trajectory generators** — no prediction, no receding horizon, no CP-based adjustment.

### Layer 3 — Task Builder
Converts FSM state + planner output into QP ingredients:

| Phase | Active Feet (hard) | Soft Tasks | CoM target |
|-------|-------------------|------------|------------|
| BIPEDAL_INIT | Both, vel-damping | — | Midpoint |
| WEIGHT_SHIFT | Both, vel-damping + unload bias | Pelvis orientation | Quintic interpolated |
| SINGLE | Support foot only | Swing xy/z + pelvis orientation | Cached support foot centre |
| DOUBLE | Both, vel-damping | Pelvis orientation | Biased midpoint (70/30) |

Key details:
- **Swing weights**: xy ramps 0.1→1.0 during last 40% of swing; z constant at 1.0
- **Weight boost**: single-support overrides CoM/CAM weights to 200, posture to 1
- **CoM-z dropped** during single-support (avoids fight with swing-z task)
- **Landing xy chase**: `_swing_target[:2]` overwritten with current foot position during descent

### Layer 4 — QP Solver + Torque Recovery (`QPWBCController._solve_qp`)

- **Decision variables**: `[qacc (nv); λ_left (6); λ_right (6)]`
- **Hard equalities**: floating-base dynamics + active-foot acceleration constraints
- **Inequalities**: friction pyramid (`|fx|,|fy| ≤ μ·fz`), CoP bounds, torsional friction (skipped during SINGLE)
- **Cost**: weighted soft tasks (CoM, CAM, posture, swing, pelvis orientation)
- **Torque recovery**: `τ = (M·qacc + h − Σ J_i^T·λ_i)[6:]` — analytic, not via `mj_inverse`

## Data Flow (Strictly Downward)

```
  Gait FSM ───────────────────────────────────────────┐
      │                                                │
      ▼                                                │
  Motion Planners (footstep, swing, CoM) ──────────────┤
      │                                                │
      ▼                                                ▼
  Task Builder ───────────────→ active_feet, tasks, targets
      │                                                │
      ▼                                                ▼
  OSQP ──────────→ [qacc; λ_left; λ_right] ────────→ τ
```

Each layer only consumes output from the layer above. No layer reaches upward.
