# MuJoCo G1 Single-Leg Stand — Project Memory

## User Preferences
- Use `uv` for Python package management.
- **Never write long bash commands.** Write scripts to disk, run them, delete when done.
- QP-based Whole-Body Control (WBC) is preferred over Jacobian-transpose or pure PD.

## Technical Constraints
- Robot: Unitree G1, 23 DOF + 6 floating base (`nv=29`, `nu=23`, mass ~34.13 kg).
- Simulation dt: `0.002 s` (500 Hz).
- MuJoCo 3.x API differences:
  - `mj_rne(model, data, flg_acc, result)` — no separate `qacc` argument; uses `data.qacc` when `flg_acc=1`, or computes `C(q,qvel)` when `flg_acc=0`.
  - `mj_inverse(model, data)` — uses `data.qacc`, outputs to `data.qfrc_inverse`.

## Key Findings
- Initial reference pose (knee=0.15 rad, ankle_pitch=-0.08 rad) is **dynamically unstable**.
- CoM starts ~5.7 cm forward of the foot midpoint, causing immediate forward tip-over.
- Simple PD, Jacobian-transpose, and basic weighted-least-squares WBC all fail with the current pose.
- **Initial pose must be fixed before any controller can succeed.**

## Walking Controller — Current Architecture & Known Issues

### Architecture
- 5-phase FSM: BIPEDAL_INIT → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT → WEIGHT_SHIFT_R → (repeat)
- QP-WBC solves for `[qacc (nv); lambda_feet]` with OSQP
- Torque recovery: `tau = (M*qacc + h - sum(J_i^T * lambda_i))[6:]`
- Friction cone: pyramidal (`|fx|,|fy| <= mu*fz`) + CoP bounds + torsional friction
- CP-based footstep planner for step length adjustment

### Critical Finding: Contact Impedance Mismatch
- **Hard 6-DOF support foot constraint causes bounce-off instability in MuJoCo.**
- The QP assumes infinite contact stiffness (hard constraint), but MuJoCo uses soft spring-damper contacts (solref=[0.01,1.0]).
- During single support, the support foot drifts upward by 5-10mm due to qacc mismatch (qacc_actual ≠ qacc_des).
- This creates a positive feedback loop: foot lifts → contact weakens → more lift → total liftoff.
- Adding Z spring to hard constraint offset (`kp_z * (ground_z - foot_z)`) helps slightly but doesn't fix the root cause.
- **Solution that works: replace hard 6-DOF constraint with 3-DOF friction cone (xyz) + soft position tracking (weight 1000, kp 400, kd 40).** This keeps the foot on the ground by cooperating with MuJoCo's contact model instead of fighting it.

### Critical Finding: Yaw Drift During Single Support
- **~93° yaw drift per single-support phase (~0.6s).**
- Maximum yaw torque from friction: `tau_z_max = mu * min(W/2,L/2) * fz ≈ 0.8 * 0.025 * 335 ≈ 7 Nm`.
- This is insufficient to counteract swing leg angular momentum.
- Hard 6-DOF constraint with torsional friction also yaws ~70-90° (limited by the same friction cone).
- Removing torsional friction during single support made yaw worse (drift ~150°).
- Re-enabling torsional friction didn't help (still ~93°).
- Swing leg angular momentum is the primary driver of yaw drift.
- **Partial fix: settle period (0.2s) before swing start + CoM task during settle improve stability** (rms_roll 1.7° vs 8.9° without).

### Approach Comparison (tested solutions)
| Approach | Support Foot Vertical | Yaw Drift | Notes |
|----------|----------------------|------------|-------|
| Hard 6-DOF constraint | Bounces off (5-10mm) | ~70-90° | Original baseline |
| Hard 6-DOF + Z spring (kp=100-500) | Still bounces | ~70-100° | Z spring doesn't fix bounce root cause |
| Soft tracking (weight 1000) | Stays on ground! | ~150° | No friction cone → uncontrolled slip |
| 3-DOF friction + soft tracking | Stays on ground! | ~93° | Best vertical stability, yaw is the issue |
| Soft tracking (no hard feet) | N/A | ~152° | No friction at all, worst yaw |
| Torque-level Z PD correction | N/A | N/A | Causes OSQP infeasibility |

### What Hasn't Worked
- Using `data.qfrc_constraint` for torque recovery instead of `J^T*lambda` — includes swing foot and other contact forces, producing wrong torques during single support
- Increasing MuJoCo contact stiffness (solref 0.005) — doesn't help bounce
- Post-QP torque correction for support foot Z — creates OSQP infeasibility
- Partial Jacobian (5-DOF: XY+orientation) — `_build_wrench_cones` doesn't handle non-3/6 dimensions

### DOUBLE_SUPPORT: Landing Foot Z Descent
- Landing foot uses hard XY + orientation constraint, free Z (no Z hard constraint)
- A soft Z descent task (kp=80, kd=10, weight=500) pushes the landing foot toward ground
- This replaces the previous quintic descent profile approach

## Project Structure
```
configs/g1_config.yaml          # Robot params, initial pose, control gains
env/g1_env.py                   # MuJoCo environment wrapper
utils/kinematics.py             # CoM, body velocity, contact wrench, quat math
controllers/bipedal_stance_controller.py   # Phase 1 controller (QP-WBC target)
controllers/walking_controller.py          # Phase 3 controller (gait FSM + QP-WBC)
controllers/base_qp_wbc.py                  # QP construction, wrench cones, torque recovery
planners/footstep_planner.py               # CP-based foot placement
planners/swing_foot_planner.py             # Quintic swing trajectory
planners/com_planner.py                    # CoM interpolation during weight shift
scripts/test_walking.py                    # Walking evaluation script
scripts/test_bipedal_stance.py             # Phase 1 evaluation script
```

## Problem-Solving Flow (Debugging Methodology)

When a controller or simulation fails, follow this order. Do NOT jump to parameter tuning.

### 1. Verify the Initial Condition
- Is the starting pose actually a static equilibrium? Compute it analytically.
- Check `qfrc_bias` vs expected gravity. If they don't match expectations, the pose or model is wrong.
- Check contact forces after one `mj_step`. Are they what physics predicts?

### 2. Check Physics Solver Parameters
Before blaming the controller, verify MuJoCo's contact model:
- `geom_solref` and `geom_solimp` — soft contacts cause sinking and delayed force buildup
- `geom_friction` — especially rolling friction (`friction[2]`) for sphere foot geoms
- `model.opt.timestep` — too large causes contact instability
- Check `qfrc_constraint` vs `qfrc_bias`; if constraint forces don't balance gravity, contacts are the problem

### 3. Verify Actuator Mapping
- `model.actuator_trnid` and `model.actuator_gear` map `ctrl` indices to DOFs
- A sign error or index offset here makes every controller fail regardless of tuning
- Test: `ctrl[i] = 1` → check which `qfrc_actuator[j]` becomes non-zero

### 4. Distinguish Static vs Dynamic Stability
- A pose can be statically stable (positive contact forces, CoM inside support polygon) but dynamically unstable in simulation if the solver cannot resolve forces fast enough
- Do not assume "math says it's stable" means "MuJoCo will keep it stable"

### 5. Isolate with Diagnostic Scripts
Write small single-purpose scripts to `/scripts/debug_*.py`, run them, then delete:
- `debug_contact_params.py` — print `solref`, `solimp`, `friction`, contact forces
- `debug_qacc.py` — print actual `qacc` after first step with zero/known control
- `debug_static_torque.py` — compare `h_joints`, `J^T * f_contact`, and resulting torque
- `test_airborne.py` — verify gravity compensation in free space (eliminates contact variables)

### 6. Fix Physics Before Tuning Controllers
If the robot falls even with `qacc_des = 0` and exact inverse dynamics, the problem is physics, not control. Fix contacts, timestep, or initial pose first.

### 7. Only Then Implement the Controller
Once the bare physics hold (robot stays upright with static torques), add feedback:
- Prefer QP-based WBC over Jacobian-transpose for floating-base robots
- Use hard constraints for feet-fixed tasks, not soft weighted objectives
- **BUT: hard 6-DOF foot constraints cause bounce-off in MuJoCo** — use 3-DOF friction cone + soft tracking instead
- Full QP-WBC should include contact wrenches as explicit decision variables with friction-pyramid and CoP inequality constraints
- Recover joint torques analytically from the full dynamics equation, not via `mj_inverse`

## Code Organization

Test/evaluation scripts must separate these concerns into distinct functions. Never blend them in one loop.

| Stage | Function | Input | Output | Responsibility |
|-------|----------|-------|--------|----------------|
| 1. Run | `run_simulation(env, controller, duration)` | live env | `dict` of list-logs | Step loop, collect raw samples |
| 2. Crunch | `compute_metrics(env, controller, logs)` | log dict | `dict` of scalars | Convert lists to aggregate numbers |
| 3. Judge | `assess(metrics)` | metrics dict | `dict[str, bool]` | Compare against pass/fail thresholds |
| 4. Print | `report(metrics, checks)` | both dicts | stdout | Format results for human reading |
| 5. Save | `save_logs(logs, path)` | log dict, path | file | Persist raw data to disk |
| 6. Wire | `main()` | — | — | Config, setup, call 1→5 in order |

Rules:
- Each function has **one input, one output, one job**.
- Data flows in a single direction: `logs → metrics → checks → stdout`.
- Use plain `dict` as the data contract between stages — no classes needed for throwaway test data.
- The orchestration function (`main`) does no computation itself; it only wires stages together.
- Config is loaded **once** at the top of `main`; no component reads the config file independently.

## Implementation Notes
- All 23 actuators are `mjTRN_JOINT` (direct joint torque control).
- `_adjust_base_height()` in `g1_env.py` auto-sets pelvis z so lowest foot geom touches ground.
- Contact force calculation must iterate `data.contact` and use `mj_contactForce` with frame rotation.
- `mj_rne(model, data, 0, result)` computes `C(q,qvel)` including gravity; with `qvel=0` this is pure gravity bias.
- `mj_inverse(model, data)` computes `qfrc_inverse = M*qacc + C - qfrc_passive`; it does NOT account for `qfrc_constraint`. **The bipedal stance controller no longer uses `mj_inverse`** — torques are recovered analytically from the full dynamics equation with explicit contact wrenches.
- Full QP-WBC decision variables: `[qacc (nv); lambda_feet]` solved with OSQP.
- Hard equality constraints: floating-base dynamics + fixed-foot kinematics.
- Hard inequality constraints: linearised friction pyramid (`|fx|,|fy| <= mu*fz`) + CoP bounds (`|tx| <= (W/2)*fz`, `|ty| <= (L/2)*fz`).
- Torque recovery: `tau = (M*qacc + h - sum(J_i^T * lambda_i))[6:]` where `h = qfrc_bias - qfrc_passive`.
- **Using `data.qfrc_constraint` for torque recovery instead of `J^T*lambda` DOES NOT WORK** — it includes forces from swing foot and other contacts that the QP doesn't model, causing catastrophic instability.
- Foot geometry: 4 × 5 mm radius spheres at (−5 cm, ±2.5 cm) and (+12 cm, ±3 cm) in foot frame. Discrete contacts, not flat plate.