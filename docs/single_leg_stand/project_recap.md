# Project Recap — MuJoCo G1 Single-Leg Stand

## Phase 0 — Environment & Baseline (commits `f5ae72f` → `e321a00`)

| Change | What was done | Simulation effect |
|--------|--------------|-------------------|
| Initial setup | Created MuJoCo wrapper (`G1Env`), config YAML, basic test scripts (`test_env.py`) | Robot loaded but fell immediately — no controller |
| `dt = 0.002 s` (`e321a00`) | Reduced timestep from `1/240 s` (~4.17 ms) to 2 ms | Contact forces became stable; earlier timesteps caused contact jitter and NaN falls |

## Phase 1 — Bipedal Stance Controller (commits `7b59922` → `d758e47`)

| Change | What was done | Simulation effect |
|--------|--------------|-------------------|
| KKT-based WBC (`7b59922`) | First QP-WBC via direct KKT solve | Unstable; solver conditioning issues with contact constraints |
| Full QP-WBC + OSQP (`d80bb00`) | Replaced KKT with OSQP; added explicit contact wrenches as decision variables; friction-pyramid + CoP inequalities; analytical torque recovery | Solver became robust, but robot still tipped forward immediately |
| Self-documenting names + CAM task (`c835253`) | Refactored variable names; added centroidal angular momentum task | No stability fix yet — code quality only |
| Built-in CAM (`be449a1`) | Switched to MuJoCo `subtree_angmom` | Discovered `subtree_angmom` is not populated in Python bindings — returned zeros |
| **Pose + CAM + gains fix** (`db46e07`) | **Critical fix trio:** (1) `hip_pitch = -0.1 rad` to shift CoM back over foot midpoint, (2) manual `J_cam @ qvel` CAM computation, (3) CAM gains reduced 10x | **Bipedal stance now stable.** Before: CoM was 5.7 cm forward of foot midpoint → immediate tip-over. After: 9/9 pass/fail checks pass; robot stands indefinitely |
| Diagnostics + pose sweep (`d758e47`) | Added CoM tracking error plots, pose sweep methodology, `initial_pose_sweep.png` | Confirmed pose sensitivity: only a narrow hip_pitch range (-0.08 to -0.15 rad) yields static equilibrium |

**Phase 1 result:** Stable infinite-duration bipedal stance.

## Phase 2 — Transition Controller: Bipedal → Single-Leg

### 2.1 Initial Implementation (`582e8c4`)

| Change | What was done | Simulation effect |
|--------|--------------|-------------------|
| State machine + planners | `BIPEDAL` → `WEIGHT_SHIFT` → `SINGLE_LEG` with smoothstep CoM trajectory and swing foot lift | Achieved ~4 s single-leg hold, then tipped. First proof that single-leg was possible at all |

### 2.2 Baseline & Sign Bug (`5d19ee1` → `236b4ed`)

| Change | What was done | Simulation effect |
|--------|--------------|-------------------|
| Weak baseline (`5d19ee1`) | Disabled swing tracking to isolate balance | Confirmed balance is possible without swing leg coupling, but hold time still short |
| **Pelvis sign fix** (`236b4ed`) | Fixed quaternion error in pelvis orientation task (`pelvis_accel_des = -kp*err - kd*omega`; was `+kp*err`) | **Critical bug fix.** Before: pelvis task amplified error instead of correcting it. After: pelvis stays near level |

### 2.3 Physics Fix — Damped Model (`d920dba`)

| Change | What was done | Simulation effect |
|--------|--------------|-------------------|
| Added joint damping + armature | Created `g1_23dof_damped.xml` with `damping="0.5" armature="0.01"` on all 23 actuated joints; backed up to `models/` | Before: zero joint damping → all velocity oscillations persisted indefinitely. After: **12 s single-leg hold passes cleanly** with max roll 5.25°, drift 1.3 mm |

### 2.4 Long-Horizon Debugging (`f38f01b`)

| Experiment | What was done | Simulation effect | Lesson learned |
|-----------|--------------|-------------------|----------------|
| Extend to 30 s | `t_single_leg: 8.0 → 30.0` | Tip at **t ≈ 18 s** | Slow-drift instability exposed on long horizons |
| Swing xy anchor | Added soft xy tracking on swing foot (`kp=20, weight=0.5`) | Tip at **t ≈ 14 s** (worse) | Swing-leg kinematic coupling feeds disturbance back into support foot |
| Stiffer pelvis | `pelvis_kp=80, kd=12` (was 30/5) | Tip at **t ≈ 10 s** (much worse) | Pelvis accel demands exceeded CoP-bounded ground torque budget |
| **CAM diagnostics** | Added CoP scatter + CAM/CoM-in-foot-frame plots to `debug_single_leg.py` | — | **Revealed CAM growing monotonically**; CoP was *not* saturated. True cause was weak angular momentum regulation |
| **CAM gain x10** | `base_qp_wbc.py:99-100`: multiplier `0.1 → 1.0` (`kp_cam ≈ 0.23 → 2.3`, `kd_cam ≈ 0.035 → 0.35`) | **Full 30 s hold passes.** Max roll 3.26°, pitch 0.40°, support drift 1.5 mm | Weak CAM gains allowed angular momentum to accumulate until CoM left support polygon |

## Summary of Causal Chain

The final 30 s hold required **three independent fixes**, each addressing a different timescale of instability:

1. **Static equilibrium** (`db46e07`): `hip_pitch = -0.1 rad` fixed the 5.7 cm CoM forward offset. Without this, no controller can succeed.
2. **Dynamic damping** (`d920dba`): Joint damping `0.5` + armature `0.01` dissipated velocity oscillations. Without this, the 12 s hold fails.
3. **Angular momentum regulation** (`f38f01b`): CAM gains x10 prevented slow rotational drift. Without this, the 30 s hold fails at ~18 s despite perfect short-horizon behavior.

The two reverted experiments (swing xy anchor, stiffer pelvis) both failed because they increased task demands without increasing the robot's physical control authority — the ground can only push back within the friction pyramid and CoP envelope.

## Current State

- **Branch:** `dev`
- **Working tree:** clean
- **Ahead of master:** 5 commits
- **Validated capability:** Stable 30 s single-leg stance on Unitree G1 in MuJoCo
