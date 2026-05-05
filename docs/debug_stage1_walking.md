# Stage 1 Walking Debug Report

**Date:** 2026-05-05  
**Branch:** feat/walking  
**Test:** `scripts/test_walking.py` (static alternating support, step_length = 0)

---

## Current Status

Stage 0 infrastructure is complete. Stage 1 (static alternating support) is still failing.
The robot survives ~7 s but support foot slip during single support prevents touchdown,
so the gait never completes a single alternating cycle.

Latest test result (combined smooth CoM + swing-foot preload + fixed CoM target):
- Failure at t ≈ 8 s
- Support slip: 0.25 m during LEFT_SINGLE
- Step count: 0
- All metrics: FAIL

---

## Solved Issues

| # | Issue | Fix | Verified |
|---|-------|-----|----------|
| 1 | QP "primal infeasible" during WEIGHT_SHIFT | Restored pelvis orientation task to weight-shift phases | Yes — failure moved from t=1.7 s to t=8 s |
| 2 | Trunk tilts -30° during weight shift | Added `ComPlanner` smooth CoM interpolation | Yes — pelvis stays within ±10° for 7 s |
| 3 | Weight shift too slow (5.4 s) | Added swing-foot preload (`a_lift`) during unload | Yes — weight shift now ~1.9 s |
| 4 | CoM target chases slipping foot | Fixed `self._single_support_com_target` at phase entry | Yes — CoM target no longer tracks current foot pos |
| 5 | Posture task fights leaned configuration | Snapshot `q_ref` at single-support entry | No measurable improvement |
| 6 | Transition gate measured com_err to ankle | Changed to foot CoP centre (3.5 cm forward of ankle), tightened to 0.03 m | Verified — gate fires at correct geometry |

---

## Unsolved Issues

| # | Issue | Evidence |
|---|-------|----------|
| 1 | **Support foot slips 0.25–1.0 m during single support** | TransitionController: 0.0004 m / 30 s. WalkingController: 0.25 m / 6 s |
| 2 | Touchdown never fires | Slip breaks the 0.02 m xy tolerance; robot stays in LEFT_SINGLE for 5+ s |
| 3 | Strong pelvis task causes QP infeasibility | `|τz| ≤ 6.7 N·m` torsional limit likely binds against yaw torque demand |
| 4 | Step count = 0, all metrics fail | Robot never completes one full cycle |

---

## Key Findings from A/B Testing

### Smooth CoM interpolation (alone)
- Weight shift: 5.4 s (too slow)
- Trunk stable (±10°)
- Completes 1 step, then falls on next cycle

### Swing-foot preload (alone)
- Weight shift: 0.9 s (fast)
- Trunk unstable (-60° pitch)
- Falls immediately in single support

### Both together
- Weight shift: 1.9 s (good)
- Trunk stable for 7 s
- Support foot slip = 0.25 m (touchdown blocked)

---

## Root Cause Hypothesis

The WalkingController overrides `_build_wrench_cones()` to add a torsional friction
constraint:

```
|λ_τz| ≤ (μ · min(foot_width, foot_length) / 2) · λ_fz
        = 0.8 · 0.025 · λ_fz
        ≈ 0.02 · λ_fz   →   ~6.7 N·m at single support
```

The TransitionController uses the **base class** wrench cones, which have **no
torsional limit**.

In single support, the pelvis orientation task demands yaw torque to keep the trunk
upright. With the torsional limit, the optimizer cannot request enough yaw torque.
The foot yaws/rolls, the CoP shifts to the edge, and the foot slips. Without the
limit (TransitionController), the optimizer gets whatever yaw torque it needs from
the spherical contacts, and the foot stays planted.

**Supporting evidence:**
- Boosting pelvis task to TransitionController levels (kp=200, weight=50) causes
  immediate "primal infeasible" — the torsional limit is binding.
- The TransitionController holds single support for 30 s with 0.0004 m slip using
  the exact same `foot_kd` damping and fixed CoM target.

---

## Next Move Plan

1. **Disable the torsional friction override** temporarily and re-run the test.
   - If slip disappears → the override is the culprit. Fix: loosen the bound or
     apply it only during double support.
   - If slip persists → investigate making the swing touchdown target relative to
     the current support foot position (so slip doesn't break the xy tolerance).

2. If step 1 resolves slip, verify that alternating left/right single support works
   for 4+ steps without falls.

3. Only then proceed to Stage 2 (in-place stepping with event-driven transitions).
