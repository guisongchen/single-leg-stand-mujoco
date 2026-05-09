# Port Summary: feat/tapping -> feat/walking

**Date:** 2026-05-09  
**Source branch:** `feat/tapping`  
**Target branch:** `feat/walking` (fresh from `master`)  
**Commit:** `983cfbb`

---

## What Was Ported

Validated gait infrastructure from the Stage 1 in-place stepping branch (`feat/tapping`) was copied to the new forward-walking branch (`feat/walking`).

| Component | File(s) | Status |
|-----------|---------|--------|
| QP-WBC solver | `controllers/base_qp_wbc.py` | Ready — OSQP solver, wrench cones, torque recovery, `_quat_error` helper |
| Walking controller | `controllers/walking_controller.py` | Ready — 5-phase FSM, GRF hysteresis transitions, touchdown detection |
| Footstep planner | `planners/footstep_planner.py` | Ready — step target generation for arbitrary step length |
| Swing trajectory planner | `planners/swing_foot_planner.py` | Ready — quintic lift/hold/descent with corrected z_end descent |
| CoM planner | `planners/com_planner.py` | Ready — smooth CoM interpolation during weight shift |
| Kinematics utilities | `utils/kinematics.py` | Ready — added `compute_capture_point` helper |
| G1 model | `models/g1_23dof_clean.xml` + `scene_23dof_clean.xml` | Ready — clean 23-DOF model used for all walking development |
| Test framework | `scripts/test_walking.py` | Ready — metric logging, pass/fail thresholds, plotting |
| Config | `configs/g1_config.yaml` | Ready — `walking`, `transitions`, `safety`, `swing_weights`, `pelvis_orientation` sections added |
| Architecture docs | `docs/walking_architecture.md` | Ready — full gait architecture spec |
| Implementation plan | `docs/walking_implementation_plan.md` | Ready — staged roadmap (Stage 0 -> Stage 5) |
| ADR | `docs/adr/001-walking-gait-architecture.md` | Ready — architecture decision record |
| Foot geometry analysis | `docs/g1_foot_constrains.md` | Ready — with English review section |

---

## What Needs Changing for Stage 3 (Forward Walking)

The following Stage 1-specific adaptations should be revisited when `step_length > 0`:

1. **Re-enable XY swing tracking.**  In `_build_phase_tasks()` the swing-foot XY task is already gated behind `if self.step_length > 0.0:`.  Raising `step_length` from `0.0` to `0.10` will automatically re-enable it.

2. **Double-support foot constraints.**  Stage 1 replaced PD anchors with pure velocity damping to allow the feet to settle after imperfect landing.  Forward walking may need weak PD anchors restored (`kp ~ 100`, `kd ~ 10`) to prevent foot slip during the forward momentum phase.

3. **CoM-z task during single support.**  Stage 1 drops the CoM-z task during single support to avoid fighting the swing-foot descent.  With forward walking the pelvis moves forward, so the CoM-z task may need to be restored or softened rather than fully disabled.

4. **Parameter tuning.**  `step_length`, `step_height`, `swing_duration`, and `double_support_duration` will need re-tuning for the first forward steps.  The current values (`step_length = 0.25`, `step_height = 0.03`, `swing_duration = 0.6`) are from the Stage 1 experiments and may be too aggressive or conservative for forward motion.

5. **Capture-point safety checks.**  The `_compute_cp()` and `_is_cp_inside_combined_polygon()` methods exist on the controller but are currently used only as abort guards.  For forward walking they should be wired into the FSM as transition conditions or fallback triggers.

---

## Stage 1-Specific Files Left Behind

The following files were intentionally **not** ported because they are specific to the in-place stepping debug session:

- `docs/debug_stage1_walking_2026_05_09.md` — detailed Stage 1 debug report
- `docs/phase2_debug_status_2026_05_04.md` — old debug status
- `scripts/debug_feasibility.py` — temporary diagnostic script
- `scripts/test_single_to_double.py` — old transition test

They remain available on `feat/tapping` for reference.
