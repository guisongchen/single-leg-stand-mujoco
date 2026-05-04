# Phase 2 Transition Controller — Debug Status (2026-05-04)

## Original Problem

The transition controller (bipedal → single-leg) failed immediately after entering `SINGLE_LEG`. The initial 7-second diagnostic plot showed pelvis roll/pitch exploding to ±150°, support foot sliding 2.7 m, and the robot collapsing.

## What We Learned

### 1. WEIGHT_SHIFT actually works

A focused debug script (`scripts/debug_weight_shift.py`) overturned the initial hypothesis. During `WEIGHT_SHIFT`:

- CoM y tracks the smoothstep reference cleanly (peak error ≈ 22 mm).
- Weight transfers correctly: swing foot fz drops from 165 N → ~0 N; support foot rises to 335 N.
- State-machine trigger fires correctly at t ≈ 2.6 s.

The failure is **not** in weight shift — it is inside `SINGLE_LEG`.

### 2. Late-phase chatter in WEIGHT_SHIFT is real

Even though tracking succeeds, the last ~0.2 s of `WEIGHT_SHIFT` shows violent force chatter (±600 N spikes, negative swing fz, lateral force oscillations ±40 N). This comes from the QP fighting an infeasible bilateral constraint once `fz_swing → 0`.

**Fix applied:** During `WEIGHT_SHIFT`, the swing foot now receives an upward `accel_offset` that scales with unload fraction:

```python
unload_frac = clip(1.0 - swing_fz / 80.0, 0.0, 1.0)
a_lift = 1.0 * unload_frac
```

This prevents the QP from being asked to pin a foot that is no longer pushing on the ground. Chatter is reduced and the handoff stays clean.

### 3. SINGLE_LEG crashes within 250 ms

The `scripts/debug_single_leg.py` run shows:

- `SINGLE_LEG` entered at t = 2.602 s.
- By t = 2.85 s the robot has tipped (roll = 179°, pitch = 86°).
- Simulation goes NaN at t = 4.15 s.
- OSQP max-iterations error occurs at step 2759.

**Key evidence from the QP logs:**
- Constraint utilisation spikes to > 400 immediately after transition.
- QP support `fz` diverges to −40 000 N (impossible), meaning OSQP returns an unconverged/infeasible iterate.
- The problem is **QP infeasibility**, not just bad tuning.

### 4. Structural change to `_solve_qp`

`base_qp_wbc.py` was modified so that `active_feet` entries can have `m = 3` (linear-only) or `m = 6` (full wrench) Jacobians. The friction-pyramid / CoP cone builder adapts automatically:

- `m = 6` → 9 inequalities (`fz ≥ 0`, friction pyramid, CoP bounds).
- `m = 3` → 5 inequalities (`fz ≥ 0`, friction pyramid only).

In `SINGLE_LEG`, the support foot is now a **3D linear-only** hard constraint (`J_support[:3]`, no angular rows). This removes the impossible demand for ground torque once the robot starts to tip.

**Result:** The robot still tips. Removing the angular constraint means the foot is now a point contact; the robot behaves like an inverted pendulum with no ankle torque. The CoM / CAM / pelvis tasks are not strong enough to stabilise it.

## New Issues Created

1. **`scripts/debug_single_leg.py` indexing bug**
   The script assumes 6-D wrenches and tries to read `w[3]` (torque). With the new 3-D linear constraint, this raises `IndexError`. Needs a guard: only log torque components when `len(w) == 6`.

2. **SINGLE_LEG controller is fundamentally unstable**
   The combination of:
   - high-gain pelvis orientation task (`kp = 200, weight = 50`),
   - swing-foot tracking (`kp = 400`),
   - boosted CoM / CAM weights (`w_com = 200, w_cam = 200`),
   - collapsed posture weight (`w_posture = 0.01`),

   drives the QP infeasible within a few timesteps. The robot does not have enough control authority (or the right control strategy) to balance on one foot from the current bipedal pose.

3. **Initial pose may not be a single-leg equilibrium**
   The initial joint angles were tuned for bipedal stance. Once one foot lifts, the remaining leg geometry and mass distribution may place the CoM projection outside the effective support polygon for the given task set.

## Next Move Suggestions (in order)

1. **Fix the debug script**
   Patch `debug_single_leg.py` so it handles 3-D wrenches safely. Re-run to get clean diagnostics.

2. **Test whether a static single-leg equilibrium exists at all**
   Write a small script that holds the robot in single-leg stance with **zero desired acceleration** (`qacc_des = 0`) and the support foot as the only hard contact. If the robot still falls, the initial pose is wrong — fix the pose before tuning the controller.

3. **Reduce or remove the pelvis orientation task**
   The pelvis task is the heaviest consumer of the (now non-existent) ground torque budget. Try:
   - `pelvis_kp = 50` (down from 200)
   - `pelvis_weight = 5` (down from 50)
   - or comment the task out entirely and rely on CoM + CAM alone.

4. **Add angular damping as a soft task, not a hard constraint**
   Instead of demanding zero rotation via a hard equality, add a soft extra task on `J_pelvis_ang` with low weight that damps angular velocity (`accel_des = -kd * omega`). This lets the QP violate it gracefully when the ground cannot provide torque.

5. **Consider a pre-computed single-leg reference pose**
   Rather than lifting the swing leg while keeping all other joints at their bipedal reference, shift the hip/knee/ankle angles so the CoM projection is closer to the support foot center **before** the foot lifts. A different `q_ref` for `SINGLE_LEG` may be necessary.

6. **Add a ZMP / capture-point criterion**
   The current controller tracks CoM position but does not explicitly enforce that the ZMP stays inside the support polygon. A ZMP inequality constraint (or a capture-point target) would make the balance criterion explicit rather than implicit.
