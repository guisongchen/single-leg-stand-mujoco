# Phase 1 Test Results — Iteration History

Date: 2025-05-03
Config: configs/g1_config.yaml (default)

---

## Run 1: Baseline (pre-fix)

Controller: Weighted-least-squares WBC (before fixes)
Physics: soft contacts (`solref[0]=0.02`, `friction[2]=0.0001`), no pre-sink

**Status: FAILED 7/9** (OSQP crash on step 0 prevented completion — NaN check only)

| Check | Target | Actual | Status |
|-------|--------|--------|--------|
| Simulation healthy (no NaN) | - | - | PASS |
| Solve time < budget | < 2000 us | ~180 us | PASS |
| CoM RMSE | < 0.02 m | **1.38 m** | FAIL |
| Left foot drift | < 0.005 m | **1.22 m** | FAIL |
| Right foot drift | < 0.005 m | **1.10 m** | FAIL |
| Pelvis roll | < 5 deg | **179.9 deg** | FAIL |
| Pelvis pitch | < 5 deg | **89.0 deg** | FAIL |
| Left foot fz | > 10 N | **-2894.6 N** | FAIL |
| Right foot fz | > 10 N | **-2293.0 N** | FAIL |

**Root causes identified:**
1. Contact force insufficient: `qfrc_constraint[2]` = 5.3 N vs gravity 334.8 N
2. Soft contacts: foot geoms `solref=[0.02, 1.0]` (~85 kN/m stiffness)
3. Tiny initial penetration: 12 um → ~0.23 N per contact point
4. Robot sinks ~4 mm before reaching equilibrium force, tipping forward
5. Sphere foot geoms with `friction[2]=0.0001` roll freely, ankle torque ineffective

---

## Run 2: Physics fixes + KKT WBC (current)

Controller: KKT direct-solve WBC (Equality-constrained QP)
Physics fixes applied:
- Stiffened contacts: `solref[0]` 0.02 → 0.002
- Rolling friction: `friction[2]` 0.0001 → 0.01
- Pre-sink base by 2 mm
- dt: 1/240s → 0.002s (500 Hz)
- OSQP replaced by KKT direct solve (OSQP 1.1.1 falsely reports "non convex" for PSD matrix)

**Status: FAILED 7/9** (completed all 5000 steps, robot falls catastrophically)

| Check | Target | Actual | Status |
|-------|--------|--------|--------|
| Simulation healthy (no NaN) | - | - | PASS |
| Solve time < budget | < 2000 us | ~273 us (p99: 470 us) | PASS |
| CoM RMSE | < 0.02 m | **3.27 m** | FAIL |
| Left foot drift | < 0.005 m | **3.27 m** | FAIL |
| Right foot drift | < 0.005 m | **3.40 m** | FAIL |
| Pelvis roll | < 5 deg | **180.0 deg** | FAIL |
| Pelvis pitch | < 5 deg | **88.8 deg** | FAIL |
| Left foot fz | > 10 N | **-6300 N** | FAIL |
| Right foot fz | > 10 N | **-10548 N** | FAIL |

MuJoCo warning at t=8.98s: "Nan, Inf or huge value in QACC at DOF 5. The simulation is unstable."

**Problems solved this iteration:**
- OSQP "non convex" bug → replaced with KKT direct solve (scipy.linalg.solve)
- Controller no longer crashes on step 0; runs full 10s

**Problems remaining:**
1. **Initial pose still unstable**: knee=0.15 rad, ankle_pitch=-0.08 rad puts CoM ~5.7 cm forward of foot midpoint (flagged in CLAUDE.md as root cause, never changed)
2. **Controller destabilizes rather than stabilizes**: hard 12-DOF foot constraint (`J_feet * qacc = 0`) may be infeasible or produce excessive torques when robot starts tipping
3. **CoM target is the current CoM (passive)**: no active CoM positioning — controller only tries to stop motion, not correct position
4. Possible mismatch: posture error computed on 23 actuated joints, WBC solves 29-DOF `qacc`, but `mj_inverse` expects full 29-DOF `qacc`

## Next Steps

1. **Fix initial pose** — solve for static equilibrium joint angles where CoM is centered over foot polygon
2. **Verify `mj_inverse` usage** — ensure `qacc_des[6:]` maps correctly to joint torques (debug with `qfrc_actuator` check)
3. **Consider soft foot constraint** — if hard constraint is infeasible, use weighted penalty instead
4. Re-run and validate 9/9 checks pass
