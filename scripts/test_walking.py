"""Walking gait test script.

Supports Stage 1 (step_length=0, in-place) and Stage 3+ (forward walking).
The test stage is inferred from the step_length in the config file.
"""
from __future__ import annotations

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.walking_controller import WalkingController
from utils.kinematics import euler_from_quat, compute_com_position

CONFIG_PATH = "configs/g1_config.yaml"
OUTPUT_PLOT = "outputs/test_walking.png"

# MuJoCo foot body standing height (set by _adjust_base_height)
_GROUND_Z = 0.034


def run_simulation(env: G1Env, controller: WalkingController, duration: float) -> dict:
    """Run walking controller and log gait metrics."""
    n_steps = int(duration / env.dt)

    log = {
        "t": [],
        "state": [],
        "left_foot_x": [], "left_foot_y": [], "left_foot_z": [],
        "right_foot_x": [], "right_foot_y": [], "right_foot_z": [],
        "pelvis_roll": [], "pelvis_pitch": [], "pelvis_yaw": [],
        "pelvis_z": [],
        "com_x": [], "com_y": [], "com_z": [],
        "support_slip": [],
        "step_count": [],
        "left_grf": [], "right_grf": [],
        "com_target_x": [], "com_target_y": [], "com_target_z": [],
        "swing_target_x": [], "swing_target_y": [], "swing_target_z": [],
        "foot_placement_err": [],
    }

    phase_start_pos = {
        "left": env.get_body_pos("left_foot")[:2].copy(),
        "right": env.get_body_pos("right_foot")[:2].copy(),
    }
    prev_state = None

    for _ in range(n_steps):
        try:
            ctrl = controller.compute()
        except RuntimeError as exc:
            print(f"controller failed at t={env.data.time:.3f}: {exc}")
            break

        env.step(ctrl)

        if not np.all(np.isfinite(env.data.qpos)):
            print(f"NaN detected at t={env.data.time:.3f}")
            break

        t_now = env.data.time
        state = controller.state

        left_pos = env.get_body_pos("left_foot")
        right_pos = env.get_body_pos("right_foot")

        if state != prev_state:
            phase_start_pos["left"] = left_pos[:2].copy()
            phase_start_pos["right"] = right_pos[:2].copy()
            prev_state = state

        quat = env.get_pelvis_quat()
        roll, pitch, yaw = euler_from_quat(*quat)
        com = compute_com_position(env.model, env.data)

        # Support slip: measure the non-swing foot
        if state == "LEFT_SINGLE":
            slip = np.linalg.norm(left_pos[:2] - phase_start_pos["left"])
        elif state == "RIGHT_SINGLE":
            slip = np.linalg.norm(right_pos[:2] - phase_start_pos["right"])
        elif state == "WEIGHT_SHIFT_L":
            slip = np.linalg.norm(left_pos[:2] - phase_start_pos["left"])
        elif state == "WEIGHT_SHIFT_R":
            slip = np.linalg.norm(right_pos[:2] - phase_start_pos["right"])
        else:
            slip = max(
                np.linalg.norm(left_pos[:2] - phase_start_pos["left"]),
                np.linalg.norm(right_pos[:2] - phase_start_pos["right"]),
            )

        log["t"].append(t_now)
        log["state"].append(state)
        log["left_foot_x"].append(left_pos[0])
        log["left_foot_y"].append(left_pos[1])
        log["left_foot_z"].append(left_pos[2])
        log["right_foot_x"].append(right_pos[0])
        log["right_foot_y"].append(right_pos[1])
        log["right_foot_z"].append(right_pos[2])
        log["pelvis_roll"].append(roll)
        log["pelvis_pitch"].append(pitch)
        log["pelvis_yaw"].append(yaw)
        log["pelvis_z"].append(env.data.qpos[2])
        log["com_x"].append(com[0])
        log["com_y"].append(com[1])
        log["com_z"].append(com[2])
        log["support_slip"].append(slip)
        log["step_count"].append(controller.step_count)
        log["left_grf"].append(controller._compute_grf("left_foot"))
        log["right_grf"].append(controller._compute_grf("right_foot"))

        # CoM target
        if controller.com_target is not None:
            log["com_target_x"].append(controller.com_target[0])
            log["com_target_y"].append(controller.com_target[1])
            log["com_target_z"].append(controller.com_target[2])
        else:
            log["com_target_x"].append(np.nan)
            log["com_target_y"].append(np.nan)
            log["com_target_z"].append(np.nan)

        # Swing target and foot placement error
        if controller._swing_target is not None:
            log["swing_target_x"].append(controller._swing_target[0])
            log["swing_target_y"].append(controller._swing_target[1])
            log["swing_target_z"].append(controller._swing_target[2])
            swing_name = controller._swing_foot_name
            if swing_name == "left":
                err = np.linalg.norm(left_pos[:2] - controller._swing_target[:2])
            elif swing_name == "right":
                err = np.linalg.norm(right_pos[:2] - controller._swing_target[:2])
            else:
                err = 0.0
            log["foot_placement_err"].append(err)
        else:
            log["swing_target_x"].append(np.nan)
            log["swing_target_y"].append(np.nan)
            log["swing_target_z"].append(np.nan)
            log["foot_placement_err"].append(np.nan)

    return log


def _common_metrics(log: dict) -> dict:
    """Extract common metrics used by both Stage 1 and Stage 3 assessors."""
    t = np.array(log["t"])
    states = np.array(log["state"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    left_grf = np.array(log["left_grf"])
    right_grf = np.array(log["right_grf"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    pelvis_z = np.array(log["pelvis_z"])

    max_roll = np.max(np.abs(roll)) if len(roll) > 0 else 0.0
    max_pitch = np.max(np.abs(pitch)) if len(pitch) > 0 else 0.0
    fell = np.min(pelvis_z) < 0.5 if len(pelvis_z) > 0 else True

    # Pelvis RMS during single-support phases
    mask_single = (states == "LEFT_SINGLE") | (states == "RIGHT_SINGLE")
    rms_roll = np.sqrt(np.mean(roll[mask_single] ** 2)) if mask_single.any() else 0.0
    rms_pitch = np.sqrt(np.mean(pitch[mask_single] ** 2)) if mask_single.any() else 0.0

    # Foot clearance
    left_swing_mask = states == "RIGHT_SINGLE"  # left foot swings during RIGHT_SINGLE
    right_swing_mask = states == "LEFT_SINGLE"  # right foot swings during LEFT_SINGLE
    left_clearance = (np.max(left_z[left_swing_mask]) - np.min(left_z)) if left_swing_mask.any() else 0.0
    right_clearance = (np.max(right_z[right_swing_mask]) - np.min(right_z)) if right_swing_mask.any() else 0.0
    min_clearance = min(left_clearance, right_clearance) if (left_clearance > 0 or right_clearance > 0) else 0.0

    # Transitions to single support (for GRF hysteresis checks)
    transitions_to_single = []
    n_left_single = 0
    n_right_single = 0
    for i in range(1, len(states)):
        if states[i] in ("LEFT_SINGLE", "RIGHT_SINGLE") and states[i - 1] != states[i]:
            transitions_to_single.append(i)
    # Count distinct entries (not total steps — a phase can be entered
    # multiple times through emergency restart cycles)
    for i in range(1, len(states)):
        if states[i] == "LEFT_SINGLE" and states[i - 1] != "LEFT_SINGLE":
            n_left_single += 1
        if states[i] == "RIGHT_SINGLE" and states[i - 1] != "RIGHT_SINGLE":
            n_right_single += 1

    mg_approx = 34.13 * 9.81
    grf_at_transitions = []
    lifted_grf_at_liftoff = []
    for idx in transitions_to_single:
        if states[idx] == "LEFT_SINGLE":
            grf_at_transitions.append(left_grf[idx])
            lifted_grf_at_liftoff.append(right_grf[idx])
        else:
            grf_at_transitions.append(right_grf[idx])
            lifted_grf_at_liftoff.append(left_grf[idx])

    grf_hysteresis_ok = (all(g > 0.70 * mg_approx for g in grf_at_transitions)
                         if grf_at_transitions else False)
    liftoff_grf_ok = (all(g < 5.0 for g in lifted_grf_at_liftoff)
                      if lifted_grf_at_liftoff else False)

    total_steps = log["step_count"][-1] if log["step_count"] else 0

    unique_states = list(set(states))
    expected = ["BIPEDAL_INIT", "WEIGHT_SHIFT_L", "LEFT_SINGLE",
                "DOUBLE_SUPPORT", "WEIGHT_SHIFT_R", "RIGHT_SINGLE"]
    states_present = len([s for s in expected if s in unique_states])

    return {
        "max_roll_deg": float(max_roll),
        "max_pitch_deg": float(max_pitch),
        "rms_roll_single_deg": float(rms_roll),
        "rms_pitch_single_deg": float(rms_pitch),
        "fell": fell,
        "min_clearance_m": float(min_clearance),
        "left_clearance_m": float(left_clearance),
        "right_clearance_m": float(right_clearance),
        "max_support_slip_m": float(np.max(slip)),
        "total_steps": int(total_steps),
        "duration_s": float(t[-1]) if len(t) > 0 else 0.0,
        "transitions_to_single": len(transitions_to_single),
        "n_left_single": n_left_single,
        "n_right_single": n_right_single,
        "both_single_entered": n_left_single >= 1 and n_right_single >= 1,
        "grf_hysteresis_ok": grf_hysteresis_ok,
        "liftoff_grf_ok": liftoff_grf_ok,
        "states_present": states_present,
        "min_grf_at_transition_N": float(min(grf_at_transitions)) if grf_at_transitions else 0.0,
    }


def assess_stage1(log: dict) -> tuple[dict, dict]:
    """Stage 1 pass/fail: in-place stepping."""
    m = _common_metrics(log)

    checks = {
        "no_fall": not m["fell"] and m["max_roll_deg"] < 15.0 and m["max_pitch_deg"] < 15.0,
        "pelvis_stable": m["rms_roll_single_deg"] < 5.0 and m["rms_pitch_single_deg"] < 5.0,
        "foot_clearance": m["min_clearance_m"] > 0.02,
        "support_slip": m["max_support_slip_m"] < 0.005,
        "min_steps": m["total_steps"] >= 4,
        "states_present": m["states_present"] >= 4,
        "grf_hysteresis": m["grf_hysteresis_ok"],
        "liftoff_grf_ok": m["liftoff_grf_ok"],
    }

    return m, checks


def assess_stage3(log: dict, step_length: float) -> tuple[dict, dict]:
    """Stage 3 pass/fail: small forward steps."""
    m = _common_metrics(log)

    com_x = np.array(log["com_x"])
    yaw = np.rad2deg(np.array(log["pelvis_yaw"]))
    placement_err = np.array(log["foot_placement_err"])
    states = np.array(log["state"])

    forward_displacement = com_x[-1] - com_x[0] if len(com_x) > 0 else 0.0
    expected_fwd = 0.5 * step_length * max(m["total_steps"], 1)

    # Foot placement RMSE at touchdown instants
    touchdown_mask = np.zeros(len(states), dtype=bool)
    for i in range(1, len(states)):
        if states[i] == "DOUBLE_SUPPORT" and states[i - 1] != "DOUBLE_SUPPORT":
            touchdown_mask[i] = True
    placement_errors = placement_err[touchdown_mask]
    placement_rmse = (np.sqrt(np.mean(placement_errors ** 2))
                      if len(placement_errors) > 0 else float("inf"))

    yaw_drift_total = yaw[-1] - yaw[0] if len(yaw) > 0 else 0.0
    yaw_drift_per_step = abs(yaw_drift_total) / max(m["total_steps"], 1)

    checks = {
        "no_fall": not m["fell"] and m["max_roll_deg"] < 15.0 and m["max_pitch_deg"] < 15.0,
        "pelvis_stable": m["rms_roll_single_deg"] < 5.0 and m["rms_pitch_single_deg"] < 5.0,
        "forward_displacement": forward_displacement >= expected_fwd,
        "foot_placement_rmse": placement_rmse < 0.05,
        "yaw_drift_per_step": yaw_drift_per_step < 2.0,
        "support_slip": m["max_support_slip_m"] < 0.005,
        "foot_clearance": m["min_clearance_m"] > 0.02,
        "grf_hysteresis": m["grf_hysteresis_ok"],
        "liftoff_grf_ok": m["liftoff_grf_ok"],
        "both_single_entered": m["both_single_entered"],
    }

    metrics = {**m,
        "forward_displacement_m": float(forward_displacement),
        "expected_min_fwd_m": float(expected_fwd),
        "foot_placement_rmse_m": float(placement_rmse),
        "yaw_drift_per_step_deg": float(yaw_drift_per_step),
        "yaw_drift_total_deg": float(yaw_drift_total),
        "step_length_m": float(step_length),
    }

    return metrics, checks


def plot(log: dict, metrics: dict, checks: dict, step_length: float) -> None:
    os.makedirs(os.path.dirname(OUTPUT_PLOT), exist_ok=True)

    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    left_x = np.array(log["left_foot_x"])
    right_x = np.array(log["right_foot_x"])
    left_y = np.array(log["left_foot_y"])
    right_y = np.array(log["right_foot_y"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    yaw = np.rad2deg(np.array(log["pelvis_yaw"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])
    left_grf = np.array(log["left_grf"])
    right_grf = np.array(log["right_grf"])
    com_x = np.array(log["com_x"])
    com_y = np.array(log["com_y"])
    target_x = np.array(log["swing_target_x"])
    target_y = np.array(log["swing_target_y"])
    pelvis_z = np.array(log["pelvis_z"])

    is_forward = step_length > 0.0
    mg = 34.13 * 9.81

    # ------------------------------------------------------------------ #
    # Build phase-colored background bands
    # ------------------------------------------------------------------ #
    phase_colors = {
        "BIPEDAL_INIT":    "#e8e8e8",
        "WEIGHT_SHIFT_L":  "#cce5ff",
        "LEFT_SINGLE":     "#ffe0b2",
        "DOUBLE_SUPPORT":  "#c8e6c9",
        "WEIGHT_SHIFT_R":  "#cce5ff",
        "RIGHT_SINGLE":    "#ffe0b2",
    }
    phase_names = {
        "BIPEDAL_INIT": "INIT", "WEIGHT_SHIFT_L": "W-SH-L",
        "LEFT_SINGLE": "L-SINGLE", "DOUBLE_SUPPORT": "DOUBLE",
        "WEIGHT_SHIFT_R": "W-SH-R", "RIGHT_SINGLE": "R-SINGLE",
    }
    phase_bands = []
    if len(states) > 0:
        band_start = t[0]
        band_phase = states[0]
        for i in range(1, len(states)):
            if states[i] != band_phase:
                phase_bands.append((band_start, t[i - 1], band_phase))
                band_start = t[i]
                band_phase = states[i]
        phase_bands.append((band_start, t[-1], band_phase))

    def _paint_bands(ax):
        for bs, be, ph in phase_bands:
            ax.axvspan(bs, be, alpha=0.12, color=phase_colors.get(ph, "#fff"))

    # ------------------------------------------------------------------ #
    # Figure layout: 4 rows x 2 cols
    # ------------------------------------------------------------------ #
    fig, axes = plt.subplots(4, 2, figsize=(18, 16))
    plt.subplots_adjust(hspace=0.35, wspace=0.30)

    # ---- [0,0] Gait phase timeline + phase bands as background --------
    ax = axes[0][0]
    state_map = {
        "BIPEDAL_INIT": 0, "WEIGHT_SHIFT_L": 1, "LEFT_SINGLE": 2,
        "DOUBLE_SUPPORT": 3, "WEIGHT_SHIFT_R": 4, "RIGHT_SINGLE": 5,
    }
    sv = [state_map.get(s, -1) for s in states]
    ax.step(t, sv, where="post", color="tab:blue", linewidth=1.5)
    ax.set_yticks([0, 1, 2, 3, 4, 5])
    ax.set_yticklabels(["INIT", "W-SH-L", "L-SINGLE",
                        "DOUBLE", "W-SH-R", "R-SINGLE"], fontsize=7)
    ax.set_title(f"Gait phase timeline  (step_len={step_length:.2f} m)", fontsize=10)
    _paint_bands(ax)
    ax.grid(True, alpha=0.25)
    ax.set_ylim(-0.5, 5.5)

    # ---- [0,1] Pelvis orientation -------------------------------------
    ax = axes[0][1]
    ax.plot(t, roll, label="roll", color="tab:red", linewidth=1)
    ax.plot(t, pitch, label="pitch", color="tab:blue", linewidth=1)
    ax.plot(t, yaw, label="yaw", color="tab:green", linewidth=1, alpha=0.7)
    ax.axhline(+15, color="r", ls=":", alpha=0.5, lw=0.8)
    ax.axhline(-15, color="r", ls=":", alpha=0.5, lw=0.8)
    ax.axhline(+5,  color="orange", ls="--", alpha=0.4, lw=0.8)
    ax.axhline(-5,  color="orange", ls="--", alpha=0.4, lw=0.8)
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis orientation  (|·|<5° desired, <15° required)", fontsize=10)
    ax.legend(fontsize=7, ncol=3, loc="upper right")
    _paint_bands(ax)
    ax.grid(True, alpha=0.25)

    # ---- [1,0] Foot heights (ground-relative) -------------------------
    ax = axes[1][0]
    ground_ref = _GROUND_Z
    ax.plot(t, left_z - ground_ref, label="left foot", color="tab:blue", lw=1)
    ax.plot(t, right_z - ground_ref, label="right foot", color="tab:orange", lw=1)
    ax.axhline(0.02, color="r", ls="--", alpha=0.4, lw=0.8, label="min clearance (0.02 m)")
    if is_forward:
        sw_tgt_z = np.array(log["swing_target_z"])
        mask = ~np.isnan(sw_tgt_z)
        if mask.any():
            ax.plot(t[mask], sw_tgt_z[mask] - ground_ref, "k:", lw=0.8, alpha=0.4,
                    label="swing target")
    ax.set_ylabel("Height (m)")
    ax.set_title("Foot heights (ground-relative)", fontsize=10)
    ax.legend(fontsize=7, ncol=2, loc="upper left")
    _paint_bands(ax)
    ax.grid(True, alpha=0.25)

    # ---- [1,1] GRF per foot -------------------------------------------
    ax = axes[1][1]
    ax.plot(t, left_grf, label="left GRF", color="tab:blue", lw=1)
    ax.plot(t, right_grf, label="right GRF", color="tab:orange", lw=1)
    ax.axhline(0.5 * mg, color="g", ls="--", alpha=0.5, lw=0.8, label="50% mg (arm)")
    ax.axhline(0.8 * mg, color="r", ls="--", alpha=0.5, lw=0.8, label="80% mg (fire)")
    ax.axhline(5.0, color="purple", ls=":", alpha=0.4, lw=0.8, label="lift-off (<5 N)")
    ax.set_ylabel("Vertical force (N)")
    ax.set_title("Ground reaction force per foot", fontsize=10)
    ax.legend(fontsize=7, ncol=2, loc="upper right")
    _paint_bands(ax)
    ax.grid(True, alpha=0.25)

    if is_forward:
        # ---- [2,0] Forward displacement --------------------------------
        ax = axes[2][0]
        ax.plot(t, com_x, label="CoM actual", color="tab:orange", lw=1.5)
        com_tgt_x = np.array(log["com_target_x"])
        mask_tgt = ~np.isnan(com_tgt_x)
        if mask_tgt.any():
            ax.plot(t[mask_tgt], com_tgt_x[mask_tgt], "k--", lw=0.8,
                    alpha=0.6, label="CoM target")
        ax.plot(t, left_x, label="left foot", color="tab:blue", lw=0.8, alpha=0.6)
        ax.plot(t, right_x, label="right foot", color="tab:red", lw=0.8, alpha=0.6)
        ax.set_ylabel("X world (m)")
        ax.set_title("Forward displacement", fontsize=10)
        ax.legend(fontsize=7, ncol=4)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)

        # ---- [2,1] Foot placement (top-down) ---------------------------
        ax = axes[2][1]
        ax.plot(left_x, left_y, "b.", markersize=1.5, alpha=0.5,
                label="left foot", rasterized=True)
        ax.plot(right_x, right_y, "r.", markersize=1.5, alpha=0.5,
                label="right foot", rasterized=True)
        mask_valid = ~np.isnan(target_x)
        if mask_valid.any():
            ax.scatter(target_x[mask_valid], target_y[mask_valid], marker="x",
                       c="green", s=50, alpha=0.9, linewidths=1.5, label="swing target")
        ax.set_xlabel("X world (m)")
        ax.set_ylabel("Y world (m)")
        ax.set_title("Foot placement  (top-down)", fontsize=10)
        ax.legend(fontsize=7)
        ax.axis("equal")
        ax.grid(True, alpha=0.25)

        # ---- [3,0] Support foot slip -----------------------------------
        ax = axes[3][0]
        ax.plot(t, slip, color="tab:red", lw=1)
        ax.axhline(0.005, color="r", ls="--", alpha=0.5, lw=0.8,
                   label="threshold (5 mm)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Slip (m)")
        ax.set_title("Support foot horizontal slip", fontsize=10)
        ax.legend(fontsize=7)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)

        # ---- [3,1] Foot placement error --------------------------------
        ax = axes[3][1]
        pe = np.array(log["foot_placement_err"])
        mask = ~np.isnan(pe) & (pe > 0)
        if mask.any():
            ax.plot(t[mask], pe[mask], "r.", markersize=4, alpha=0.6,
                    label="instantaneous")
        ax.axhline(0.05, color="r", ls="--", alpha=0.5, lw=0.8,
                   label="max allowed (0.05 m)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.set_title("Swing foot placement error", fontsize=10)
        ax.legend(fontsize=7)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)
    else:
        # Stage 1: CoM position + support slip
        ax = axes[2][0]
        ax.plot(t, com_x, label="CoM x", color="tab:orange", lw=1)
        ax.plot(t, com_y, label="CoM y", color="tab:purple", lw=1)
        ax.set_ylabel("Position (m)")
        ax.set_title("CoM world position", fontsize=10)
        ax.legend(fontsize=7)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)

        ax = axes[2][1]
        ax.plot(t, slip, color="tab:red", lw=1)
        ax.axhline(0.005, color="r", ls="--", alpha=0.5, lw=0.8,
                   label="threshold (5 mm)")
        ax.set_ylabel("Slip (m)")
        ax.set_title("Support foot horizontal slip", fontsize=10)
        ax.legend(fontsize=7)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)

        # Stage 1: step count + summary text
        ax = axes[3][0]
        ax.plot(t, log["step_count"], color="tab:green", lw=1.5)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Steps")
        ax.set_title("Cumulative step count", fontsize=10)
        _paint_bands(ax)
        ax.grid(True, alpha=0.25)

        ax = axes[3][1]
        ax.axis("off")
        overall = "PASS" if all(checks.values()) else "FAIL"
        lines = [f"Overall: {overall}", ""]
        for name, passed in checks.items():
            lines.append(f"  {name:25s} : {'PASS' if passed else 'FAIL'}")
        lines.append("")
        lines.append("--- Key Metrics ---")
        for k, v in metrics.items():
            if isinstance(v, float):
                lines.append(f"  {k:25s} = {v:.4f}")
            else:
                lines.append(f"  {k:25s} = {v}")
        ax.text(0.02, 0.98, "\n".join(lines), transform=ax.transAxes,
                fontfamily="monospace", fontsize=6.5, verticalalignment="top")

    # ---- Phase colour legend at bottom --------------------------------
    legend_handles = []
    for ph in ["BIPEDAL_INIT", "WEIGHT_SHIFT_L", "LEFT_SINGLE",
               "DOUBLE_SUPPORT", "WEIGHT_SHIFT_R", "RIGHT_SINGLE"]:
        legend_handles.append(
            plt.Rectangle((0, 0), 1, 1, fc=phase_colors[ph], alpha=0.35,
                          label=phase_names[ph])
        )
    fig.legend(handles=legend_handles, loc="lower center", ncol=6,
               fontsize=7, title="Phase bands (background)",
               title_fontsize=8)

    plt.tight_layout(rect=[0, 0.06, 1, 0.97])
    fig.savefig(OUTPUT_PLOT, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"figure saved to {OUTPUT_PLOT}")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    walking_cfg = cfg.get("walking", {})
    step_length = float(walking_cfg.get("step_length", 0.0))

    is_forward = step_length > 0.0
    stage_label = ("Stage 3: Small forward steps" if is_forward
                   else "Stage 1: Static alternating support")

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = WalkingController(env, cfg)
    controller.reset()

    duration = 20.0

    print("=" * 60)
    print(f"{stage_label}  (step_length = {step_length:.2f} m)")
    print("=" * 60)

    log = run_simulation(env, controller, duration)

    if is_forward:
        metrics, checks = assess_stage3(log, step_length)
    else:
        metrics, checks = assess_stage1(log)

    plot(log, metrics, checks, step_length)

    print(f"\n{'='*20} Metrics {'='*20}")
    for k, v in metrics.items():
        if isinstance(v, float):
            print(f"  {k:30s} = {v:.4f}")
        else:
            print(f"  {k:30s} = {v}")

    print(f"\n{'='*20} Checks {'='*20}")
    all_pass = all(checks.values())
    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")
    for name, passed in checks.items():
        print(f"  {name:30s} : {'PASS' if passed else 'FAIL'}")


if __name__ == "__main__":
    main()
