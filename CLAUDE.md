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

## Project Structure
```
configs/g1_config.yaml          # Robot params, initial pose, control gains
env/g1_env.py                   # MuJoCo environment wrapper
utils/kinematics.py             # CoM, body velocity, contact wrench, quat math
controllers/bipedal_stance_controller.py   # Phase 1 controller (QP-WBC target)
scripts/test_bipedal_stance.py  # Phase 1 evaluation script
scripts/test_env.py             # Phase 0 evaluation script
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
- Use `mj_inverse` to map desired `qacc` to joint torques

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
- `mj_inverse` computes `qfrc_inverse = M*qacc + C - qfrc_passive`; it does NOT account for `qfrc_constraint`. Contacts must be handled separately.
