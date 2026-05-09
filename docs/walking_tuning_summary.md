# Walking Controller — Tuning Summary (Abandoned)

## Why We Stopped

Tuning MuJoCo-specific contact parameters (solref, friction, constraint types) is
not productive.  These parameters don't transfer to real hardware.  The core
controller architecture (QP-WBC + FSM) is sound; the remaining failures are
artifacts of MuJoCo's soft-contact solver fighting the QP's hard constraints.

The controller should be refined against real hardware or a stiff-contact
simulator (e.g. Drake, MuJoCo with `solref=0.001`), not by endlessly tweaking
empirical gains.

## What Was Tested

### Support-Foot Contact Strategies (Single Support)

| Approach | Vertical (Z) | Yaw | Notes |
|----------|--------------|------|-------|
| Hard 6-DOF constraint | Bounces 5-10 mm | ~70-90° | Original baseline |
| Hard 6-DOF + Z spring in accel_offset (kp 100-500) | Still bounces | ~70-100° | Z spring in QP offset = invisible to MuJoCo |
| Hard 6-DOF + Z spring kp=300 | Bounces | ~70° | Numerical instability at kp>500 |
| 3-DOF friction cone + soft position tracking (w=1000, kp=400, kd=40) | **Stays grounded** | ~150° | No orientation constraint → uncontrolled yaw |
| 3-DOF friction cone + soft pos + orientation tracking (w=200, kp=200) | Stays grounded | ~93° | Best vertical; yaw still driven by swing momentum |
| Soft tracking only (weight 1000, no friction cone) | Stays grounded | ~152° | No friction → lateral slip |
| Post-QP torque-level Z PD correction | N/A | N/A | OSQP infeasible (torques outside QP cone) |
| `data.qfrc_constraint` torque recovery (instead of J^T λ) | Catastrophic fall | N/A | Includes swing-foot forces → wrong torques |

### Swing-Start Strategies

| Strategy | rms_roll (single support) | Notes |
|----------|--------------------------|-------|
| Immediate swing (no settle) | ~8-9° | Swing momentum pulls support foot off |
| 0.2 s settle (CoM task active, swing weight ~0) | ~1.7° | Significant improvement |

### CoM Task During Single Support

| Strategy | Yaw drift | Notes |
|----------|-----------|-------|
| CoM task always on | ~68° | Competes with swing repositioning |
| CoM task off entirely | ~93° | No angular momentum regulation |
| CoM task during settle only | ~93° | Settle helps roll; yaw still drifts |

### DOUBLE_SUPPORT Landing Foot

| Strategy | Foot reaches ground? | Notes |
|----------|---------------------|-------|
| Hard Z constraint | No (λ mismatch) | QP assumes contact that doesn't exist |
| Free Z (no constraint) | Partial (2-3 cm gap) | No incentive to descend |
| Soft Z descent (kp=80, kd=10, w=500) | Partially | Better than free, still not reliable |

### Contact Stiffness (MuJoCo solref)

| solref | Support foot Z | Side effects |
|--------|----------------|-------------|
| [0.02, 1.0] (default) | Bounces off | Baseline |
| [0.01, 1.0] (stiffened) | Bounces off | No improvement |
| [0.005, 1.0] (very stiff) | Bounces off | MuJoCo warning about unstable QACC |

## Root Causes Identified

1. **Contact impedance mismatch**: The QP assumes infinitely rigid contacts
   (hard equality `J_foot * qacc = offset`), but MuJoCo models contacts as
   spring-damper pairs (`solref`). The resulting force mismatch causes the
   support foot to oscillate vertically and eventually lift off.

2. **Yaw torque saturation**: Maximum torsional friction from 4-sphere foot
   geometry is `τ_z ≈ μ · min(W/2, L/2) · fz ≈ 7 Nm`.  Swing-leg angular
   momentum exceeds this by a large margin, producing ~90° yaw per step.

3. **Double-support transition fragility**: After single support, the robot
   enters DOUBLE_SUPPORT with yaw rotation already accumulated.  The hard
   constraint on the landing foot (even partial) can't stabilise this.

## Current Code State

The following changes are in `controllers/walking_controller.py`:

- **Support foot during single support**: 3-DOF friction cone (XYZ) with soft
  position+orientation tracking.  This preserves vertical contact but cannot
  constrain yaw.
- **Settle period**: 0.2 s at start of single support with CoM task active and
  swing weights suppressed.
- **Soft Z descent during DOUBLE_SUPPORT**: kp=80, kd=10, weight=500 for
  landing foot.
- **Torsional friction**: Re-enabled during single support (doesn't fix yaw).

Config changes in `configs/g1_config.yaml`:

- `swing_settle_duration: 0.2`
- `support_foot_kp_z: 300.0` (used in hard-constraint fallback, currently
  inactive since we use soft tracking for single support)
- `ds_z_descent_kp: 80.0`, `ds_z_descent_kd: 10.0`, `ds_z_descent_weight: 500.0`

## Recommended Next Steps (Not Simulation Tuning)

1. **Reduce swing angular momentum**: Shorter steps, slower swing, or
   counter-rotating arm motions to cancel yaw torque.  This is a control
   strategy, not a sim parameter.

2. **Use a stiff-contact simulator** for development: Drake or Isaac Gym with
   rigid contacts.  The hard-constraint approach works in theory; it only fails
   because MuJoCo's contacts are soft.

3. **Test on real hardware**: The 3-DOF friction cone + soft tracking approach
   keeps the foot on the ground.  Real friction coefficients (~1.0 for rubber
   on concrete vs 0.8 in sim) and rigid ground may make the hard-constraint
   approach viable.

4. **Consider whole-body angular momentum regulation**: The QP currently has a
   CAM (centroidal angular momentum) task, but it targets zero.  A smarter
   target that pre-compensates for swing-leg angular momentum would reduce yaw
   drift without requiring more friction.