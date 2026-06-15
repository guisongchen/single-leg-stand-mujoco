"""Test: Single-leg -> Bipedal return.

Verifies the PHYSICAL transition from single support to double support.
The state machine may claim BIPEDAL_RETURN -> BIPEDAL, but this script
checks that the swing foot actually touched the ground and that the robot
remains stably bipedal afterward.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from utils.kinematics import compute_com_position, compute_contact_wrench, euler_from_quat

CONFIG_PATH = "configs/g1_config.yaml"

# Thresholds for physical verification (not state-machine names)
GROUND_Z_THRESH = 0.038          # foot must be below this to count as on ground
TOUCHDOWN_FZ_THRESH = 20.0       # sustained contact force (N)
TOUCHDOWN_MIN_STEPS = 10         # min consecutive steps above force thresh
STABLE_BIPEDAL_MIN_S = 3.0       # must hold stable bipedal stance this long
STABLE_ROLL_DEG = 5.0
STABLE_PITCH_DEG = 5.0
FALL_ROLL_DEG = 15.0
FALL_PITCH_DEG = 15.0


def run(env: G1Env, ctrl: TransitionController, duration: float) -> dict:
    """Run until physical double support is achieved and stable, or timeout/crash."""
    n_steps = int(duration / env.dt)

    log = {
        "t": [],
        "state": [],
        "left_foot_x": [],
        "left_foot_y": [],
        "left_foot_z": [],
        "right_foot_x": [],
        "right_foot_y": [],
        "right_foot_z": [],
        "pelvis_roll": [],
        "pelvis_pitch": [],
        "support_slip": [],
        "com_x": [],
        "com_y": [],
        "com_z": [],
        "com_target_x": [],
        "com_target_y": [],
        "left_foot_fz": [],
        "right_foot_fz": [],
        "max_tau": [],
        "touchdown_flag": [],
    }

    left_init = env.get_body_pos("left_foot")[:2].copy()
    right_init = env.get_body_pos("right_foot")[:2].copy()

    swing_name = ctrl.swing_foot_name  # "left" or "right"
    swing_body_name = f"{swing_name}_foot"

    # Physical event tracking
    touchdown_step_count = 0
    actual_touchdown_occurred = False
    bipedal_stable_start = None
    bipedal_stable_duration = 0.0

    for step_i in range(n_steps):
        t = env.data.time

        try:
            tau = ctrl.compute()
        except RuntimeError as exc:
            print(f"controller failed at t={t:.3f}: {exc}")
            break

        env.step(tau)

        if not np.all(np.isfinite(env.data.qpos)):
            print(f"NaN detected at t={t:.3f}")
            break

        left_pos = env.get_body_pos("left_foot")
        right_pos = env.get_body_pos("right_foot")
        swing_pos = env.get_body_pos(swing_body_name)
        swing_fz = compute_contact_wrench(
            env.model, env.data, env.cfg["robot"]["body_names"][f"{swing_name}_foot"]
        )[2]

        if ctrl.state == "SINGLE_LEG":
            support_name = ctrl.support_foot_name
            slip = np.linalg.norm(
                env.get_body_pos(f"{support_name}_foot")[:2]
                - (left_init if support_name == "left" else right_init)
            )
        else:
            slip = max(
                np.linalg.norm(left_pos[:2] - left_init),
                np.linalg.norm(right_pos[:2] - right_init),
            )

        roll, pitch, _ = euler_from_quat(*env.get_pelvis_quat())
        com_pos = compute_com_position(env.model, env.data)
        left_body = env.cfg["robot"]["body_names"]["left_foot"]
        right_body = env.cfg["robot"]["body_names"]["right_foot"]
        left_fz = compute_contact_wrench(env.model, env.data, left_body)[2]
        right_fz = compute_contact_wrench(env.model, env.data, right_body)[2]

        log["t"].append(t)
        log["state"].append(ctrl.state)
        log["left_foot_x"].append(left_pos[0])
        log["left_foot_y"].append(left_pos[1])
        log["left_foot_z"].append(left_pos[2])
        log["right_foot_x"].append(right_pos[0])
        log["right_foot_y"].append(right_pos[1])
        log["right_foot_z"].append(right_pos[2])
        log["pelvis_roll"].append(roll)
        log["pelvis_pitch"].append(pitch)
        log["support_slip"].append(slip)
        log["com_x"].append(com_pos[0])
        log["com_y"].append(com_pos[1])
        log["com_z"].append(com_pos[2])
        # Log the controller's CoM target so we can compare actual vs reference
        com_target = getattr(ctrl, "com_target", None)
        if com_target is not None:
            log["com_target_x"].append(com_target[0])
            log["com_target_y"].append(com_target[1])
        else:
            log["com_target_x"].append(np.nan)
            log["com_target_y"].append(np.nan)
        log["left_foot_fz"].append(left_fz)
        log["right_foot_fz"].append(right_fz)
        log["max_tau"].append(float(np.max(np.abs(tau))))
        log["touchdown_flag"].append(
            ctrl._return_touchdown if ctrl.state == "BIPEDAL_RETURN" else False
        )

        # --- Physical touchdown detection (not controller flag) ---
        if ctrl.state == "BIPEDAL_RETURN" and not actual_touchdown_occurred:
            if swing_fz > TOUCHDOWN_FZ_THRESH:
                touchdown_step_count += 1
                if touchdown_step_count >= TOUCHDOWN_MIN_STEPS:
                    actual_touchdown_occurred = True
                    print(f"  PHYSICAL TOUCHDOWN at t={t:.3f} (fz={swing_fz:.1f} N)")
            else:
                touchdown_step_count = 0

        # --- Stable bipedal timer ---
        if ctrl.state == "BIPEDAL":
            roll_deg = abs(np.rad2deg(roll))
            pitch_deg = abs(np.rad2deg(pitch))
            both_on_ground = left_pos[2] < GROUND_Z_THRESH and right_pos[2] < GROUND_Z_THRESH
            both_loaded = left_fz > 10.0 and right_fz > 10.0
            stable_now = (
                roll_deg < STABLE_ROLL_DEG
                and pitch_deg < STABLE_PITCH_DEG
                and both_on_ground
                and both_loaded
            )
            if stable_now:
                if bipedal_stable_start is None:
                    bipedal_stable_start = t
                bipedal_stable_duration = t - bipedal_stable_start
            else:
                bipedal_stable_start = None
                bipedal_stable_duration = 0.0

            if bipedal_stable_duration >= STABLE_BIPEDAL_MIN_S:
                print(f"  STABLE BIPEDAL held for {bipedal_stable_duration:.2f} s, ending test.")
                break

    log["actual_touchdown_occurred"] = actual_touchdown_occurred
    log["bipedal_stable_duration"] = bipedal_stable_duration
    log["support_foot_name"] = ctrl.support_foot_name
    log["swing_foot_name"] = ctrl.swing_foot_name
    return log


def assess(log: dict) -> tuple[dict, dict]:
    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])
    left_fz = np.array(log["left_foot_fz"])
    right_fz = np.array(log["right_foot_fz"])
    max_tau = np.array(log["max_tau"])

    com_x = np.array(log["com_x"])
    com_y = np.array(log["com_y"])
    left_x = np.array(log["left_foot_x"])
    left_y = np.array(log["left_foot_y"])
    right_x = np.array(log["right_foot_x"])
    right_y = np.array(log["right_foot_y"])

    return_mask = states == "BIPEDAL_RETURN"
    return_steps = int(return_mask.sum())
    return_duration = return_steps * 0.002

    final_left_z = left_z[-1] if len(left_z) > 0 else 0.0
    final_right_z = right_z[-1] if len(right_z) > 0 else 0.0
    final_mid_x = 0.5 * (left_x[-1] + right_x[-1]) if len(left_x) > 0 else 0.0
    final_mid_y = 0.5 * (left_y[-1] + right_y[-1]) if len(left_y) > 0 else 0.0
    com_err_final = np.sqrt((com_x[-1] - final_mid_x) ** 2 + (com_y[-1] - final_mid_y) ** 2)

    n_avg = min(10, len(left_fz))
    final_left_fz = float(np.mean(left_fz[-n_avg:])) if n_avg > 0 else 0.0
    final_right_fz = float(np.mean(right_fz[-n_avg:])) if n_avg > 0 else 0.0

    return_max_tau = float(np.max(max_tau[return_mask])) if return_mask.any() else 0.0

    # --- PHYSICAL checks (the only ones that matter) ---
    actual_touchdown = log.get("actual_touchdown_occurred", False)
    bipedal_stable_dur = log.get("bipedal_stable_duration", 0.0)

    checks = {
        "no_fall": np.max(np.abs(roll)) < FALL_ROLL_DEG and np.max(np.abs(pitch)) < FALL_PITCH_DEG,
        "swing_foot_actually_touched_down": actual_touchdown,
        "return_phase_exists": return_mask.any(),
        "return_stable": return_duration >= 3.0,
        "both_feet_on_ground": final_left_z < 0.06 and final_right_z < 0.06,
        "support_slip": np.max(slip) < 0.05,
        "com_near_midpoint": com_err_final < 0.05,
        "stable_bipedal_after_return": bipedal_stable_dur >= STABLE_BIPEDAL_MIN_S,
        "both_feet_loaded": final_left_fz > 20.0 and final_right_fz > 20.0,
        "torque_within_limit": return_max_tau < 150.0,
    }

    metrics = {
        "max_roll_deg": float(np.max(np.abs(roll))),
        "max_pitch_deg": float(np.max(np.abs(pitch))),
        "return_duration_s": float(return_duration),
        "final_left_z_m": float(final_left_z),
        "final_right_z_m": float(final_right_z),
        "max_slip_m": float(np.max(slip)),
        "com_err_from_mid_m": float(com_err_final),
        "bipedal_stable_dur_s": float(bipedal_stable_dur),
        "final_left_fz_N": float(final_left_fz),
        "final_right_fz_N": float(final_right_fz),
        "return_max_tau_Nm": float(return_max_tau),
    }

    return metrics, checks


def plot(log: dict, metrics: dict, checks: dict) -> None:
    import matplotlib.pyplot as plt

    os.makedirs("outputs", exist_ok=True)

    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    left_x = np.array(log["left_foot_x"])
    left_y = np.array(log["left_foot_y"])
    right_x = np.array(log["right_foot_x"])
    right_y = np.array(log["right_foot_y"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    com_x = np.array(log["com_x"])
    com_y = np.array(log["com_y"])
    com_target_x = np.array(log["com_target_x"])
    com_target_y = np.array(log["com_target_y"])
    left_fz = np.array(log["left_foot_fz"])
    right_fz = np.array(log["right_foot_fz"])
    max_tau = np.array(log["max_tau"])
    states = np.array(log["state"])
    touchdown = np.array(log["touchdown_flag"])

    support_name = log.get("support_foot_name", "left")
    swing_name = log.get("swing_foot_name", "right")

    # Map generic left/right data to support/swing semantics for clear labelling
    support_z = left_z if support_name == "left" else right_z
    swing_z = right_z if swing_name == "right" else left_z
    support_fz = left_fz if support_name == "left" else right_fz
    swing_fz = right_fz if swing_name == "right" else left_fz
    support_x = left_x if support_name == "left" else right_x
    swing_x = right_x if swing_name == "right" else left_x
    support_y = left_y if support_name == "left" else right_y
    swing_y = right_y if swing_name == "right" else left_y

    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for idx, val in enumerate(states):
        if val != prev:
            state_changes.append((t[idx], val))
            prev = val

    td_indices = np.where(touchdown)[0]
    td_time = t[td_indices[0]] if len(td_indices) > 0 else None

    def _mark(ax):
        for tc, _ in state_changes:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)
        if td_time is not None:
            ax.axvline(td_time, color="r", ls="--", alpha=0.5)

    # ------------------------------------------------------------------
    # Figure 1 — Kinematics
    # ------------------------------------------------------------------
    fig1, axes = plt.subplots(3, 2, figsize=(12, 12))

    # 1. State machine phase
    ax = axes[0, 0]
    state_map = {"INIT": 0, "BIPEDAL": 0, "WEIGHT_SHIFT": 1, "SINGLE_LEG": 2, "BIPEDAL_RETURN": 3}
    state_vals = [state_map.get(s, -1) for s in states]
    ax.plot(t, state_vals, drawstyle="steps-post", color="tab:blue")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["BIPEDAL", "WEIGHT_SHIFT", "SINGLE_LEG", "BIPEDAL_RETURN"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Phase")
    ax.set_title("State machine phase")
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 2. Foot heights — both feet share the same metric (m)
    ax = axes[0, 1]
    ax.plot(t, support_z, color="C0", label=f"support ({support_name})")
    ax.plot(t, swing_z, color="C1", label=f"swing ({swing_name})")
    ax.axhline(GROUND_Z_THRESH, color="gray", ls="--", alpha=0.3, label="ground")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)")
    ax.set_title("Foot heights")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 3. CoM X — actual vs reference (same metric: m)
    ax = axes[1, 0]
    ax.plot(t, com_x, color="C0", label="actual")
    ax.plot(t, com_target_x, color="C1", ls="--", alpha=0.7, label="reference")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("CoM X")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 4. CoM Y — actual vs reference (same metric: m)
    ax = axes[1, 1]
    ax.plot(t, com_y, color="C0", label="actual")
    ax.plot(t, com_target_y, color="C1", ls="--", alpha=0.7, label="reference")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("CoM Y")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 5. Pelvis orientation — roll and pitch share the same metric (deg)
    ax = axes[2, 0]
    ax.plot(t, roll, label="roll", color="C0")
    ax.plot(t, pitch, label="pitch", color="C1")
    ax.axhline(STABLE_ROLL_DEG, color="orange", ls="--", alpha=0.3, label="stable")
    ax.axhline(-STABLE_ROLL_DEG, color="orange", ls="--", alpha=0.3)
    ax.axhline(FALL_ROLL_DEG, color="r", ls="--", alpha=0.3, label="fall")
    ax.axhline(-FALL_ROLL_DEG, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis orientation")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 6. Support slip
    ax = axes[2, 1]
    ax.plot(t, slip, color="C0")
    ax.axhline(0.05, color="r", ls="--", alpha=0.3, label="thresh")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip (m)")
    ax.set_title("Support foot slip")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    plt.tight_layout()
    plt.savefig("outputs/test_single_to_double_kinematics.png", dpi=150)
    print("figure saved to outputs/test_single_to_double_kinematics.png")

    # ------------------------------------------------------------------
    # Figure 2 — Dynamics (4 panels, grouped by metric)
    # ------------------------------------------------------------------
    fig2, axes = plt.subplots(2, 2, figsize=(10, 8))

    # 1. Foot vertical forces — both feet share the same metric (N)
    ax = axes[0, 0]
    ax.plot(t, support_fz, color="C0", label=f"support ({support_name})")
    ax.plot(t, swing_fz, color="C1", label=f"swing ({swing_name})")
    ax.axhline(TOUCHDOWN_FZ_THRESH, color="r", ls="--", alpha=0.3, label="td thresh")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title("Foot vertical forces")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 2. Max joint torque
    ax = axes[0, 1]
    ax.plot(t, max_tau, color="C1")
    ax.axhline(150, color="r", ls="--", alpha=0.3, label="limit")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Max joint torque")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 3. Foot X positions — support and swing share the same metric (m)
    ax = axes[1, 0]
    ax.plot(t, support_x, color="C0", label=f"support ({support_name})")
    ax.plot(t, swing_x, color="C1", label=f"swing ({swing_name})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Foot X positions")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 4. Foot Y positions — support and swing share the same metric (m)
    ax = axes[1, 1]
    ax.plot(t, support_y, color="C0", label=f"support ({support_name})")
    ax.plot(t, swing_y, color="C1", label=f"swing ({swing_name})")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Foot Y positions")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    plt.tight_layout()
    plt.savefig("outputs/test_single_to_double_dynamics.png", dpi=150)
    print("figure saved to outputs/test_single_to_double_dynamics.png")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    cfg["transition"]["t_single_leg"] = 3.0
    cfg["transition"]["swing_lift_height"] = 0.015

    env = G1Env(CONFIG_PATH)
    env.reset()
    ctrl = TransitionController(env, cfg)
    ctrl.reset()

    duration = 40.0

    print("=" * 60)
    print("Test: Single-leg -> Bipedal return (PHYSICAL verification)")
    print("=" * 60)

    log = run(env, ctrl, duration)
    metrics, checks = assess(log)
    plot(log, metrics, checks)

    print("\n=== Summary ===")
    all_pass = all(checks.values())
    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")
    for name, passed in checks.items():
        print(f"  {name:35s} : {'PASS' if passed else 'FAIL'}")

    print(f"\n  actual_touchdown = {log.get('actual_touchdown_occurred', False)}")
    print(f"  bipedal_stable_duration = {log.get('bipedal_stable_duration', 0.0):.2f} s")


if __name__ == "__main__":
    main()
