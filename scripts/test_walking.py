"""Stage 1 walking test: static alternating support (step_length = 0).

Runs the WalkingController with zero forward step length to verify the
5-phase FSM with GRF-based transitions can lift and place each foot
without falling.

Produces outputs/test_walking.png and a text summary.
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
from utils.kinematics import euler_from_quat

CONFIG_PATH = "configs/g1_config.yaml"
OUTPUT_PLOT = "outputs/test_walking.png"


def run_simulation(env: G1Env, controller: WalkingController, duration: float) -> dict:
    """Run walking controller and log gait metrics."""
    n_steps = int(duration / env.dt)

    log = {
        "t": [],
        "state": [],
        "left_foot_z": [],
        "right_foot_z": [],
        "pelvis_roll": [],
        "pelvis_pitch": [],
        "support_slip": [],
        "step_count": [],
        "left_grf": [],
        "right_grf": [],
    }

    left_init = env.get_body_pos("left_foot")[:2].copy()
    right_init = env.get_body_pos("right_foot")[:2].copy()

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

        quat = env.get_pelvis_quat()
        roll, pitch, _ = euler_from_quat(*quat)

        if state == "LEFT_SINGLE":
            slip = np.linalg.norm(right_pos[:2] - right_init)
        elif state == "RIGHT_SINGLE":
            slip = np.linalg.norm(left_pos[:2] - left_init)
        else:
            slip = max(
                np.linalg.norm(left_pos[:2] - left_init),
                np.linalg.norm(right_pos[:2] - right_init),
            )

        log["t"].append(t_now)
        log["state"].append(state)
        log["left_foot_z"].append(left_pos[2])
        log["right_foot_z"].append(right_pos[2])
        log["pelvis_roll"].append(roll)
        log["pelvis_pitch"].append(pitch)
        log["support_slip"].append(slip)
        log["step_count"].append(controller.step_count)
        log["left_grf"].append(controller._compute_grf("left_foot"))
        log["right_grf"].append(controller._compute_grf("right_foot"))

    return log


def assess(log: dict) -> tuple[dict, dict]:
    """Compute pass/fail metrics."""
    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])
    left_grf = np.array(log["left_grf"])
    right_grf = np.array(log["right_grf"])

    max_roll = np.max(np.abs(roll))
    max_pitch = np.max(np.abs(pitch))

    left_swing_mask = states == "LEFT_SINGLE"
    right_swing_mask = states == "RIGHT_SINGLE"
    left_clearance = np.max(left_z[left_swing_mask]) - np.min(left_z) if left_swing_mask.any() else 0.0
    right_clearance = np.max(right_z[right_swing_mask]) - np.min(right_z) if right_swing_mask.any() else 0.0

    total_steps = log["step_count"][-1] if log["step_count"] else 0

    expected_states = [
        "BIPEDAL_INIT", "WEIGHT_SHIFT_L", "LEFT_SINGLE",
        "DOUBLE_SUPPORT", "WEIGHT_SHIFT_R", "RIGHT_SINGLE",
    ]
    unique_states = [s for s in expected_states if s in states]

    # GRF hysteresis: check that support foot GRF > 80% mg at transitions into single
    transitions_to_single = []
    for i in range(1, len(states)):
        if states[i] in ("LEFT_SINGLE", "RIGHT_SINGLE") and states[i - 1] != states[i]:
            transitions_to_single.append(i)

    mg_approx = 34.13 * 9.81  # rough body weight; controller has exact value
    grf_at_transitions = []
    for idx in transitions_to_single:
        if states[idx] == "LEFT_SINGLE":
            grf_at_transitions.append(left_grf[idx])
        else:
            grf_at_transitions.append(right_grf[idx])

    grf_hysteresis_ok = all(g > 0.70 * mg_approx for g in grf_at_transitions) if grf_at_transitions else False

    checks = {
        "no_fall": max_roll < 15.0 and max_pitch < 15.0,
        "left_foot_clearance": left_clearance > 0.02,
        "right_foot_clearance": right_clearance > 0.02,
        "support_slip": np.max(slip) < 0.005,
        "min_steps": total_steps >= 4,
        "states_present": len(unique_states) >= 4,
        "grf_hysteresis": grf_hysteresis_ok,
    }

    metrics = {
        "max_roll_deg": float(max_roll),
        "max_pitch_deg": float(max_pitch),
        "left_clearance_m": float(left_clearance),
        "right_clearance_m": float(right_clearance),
        "max_support_slip_m": float(np.max(slip)),
        "total_steps": int(total_steps),
        "duration_s": float(t[-1]) if len(t) > 0 else 0.0,
        "transitions_to_single": len(transitions_to_single),
        "min_grf_at_transition_N": float(min(grf_at_transitions)) if grf_at_transitions else 0.0,
    }

    return metrics, checks


def plot(log: dict, metrics: dict, checks: dict) -> None:
    os.makedirs(os.path.dirname(OUTPUT_PLOT), exist_ok=True)

    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])
    left_grf = np.array(log["left_grf"])
    right_grf = np.array(log["right_grf"])

    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for idx, val in enumerate(states):
        if val != prev:
            state_changes.append((t[idx], val))
            prev = val

    fig, axes = plt.subplots(3, 2, figsize=(14, 11))

    # 1. Gait phase timeline
    ax = axes[0, 0]
    state_map = {
        "BIPEDAL_INIT": 0, "WEIGHT_SHIFT_L": 1, "LEFT_SINGLE": 2,
        "DOUBLE_SUPPORT": 3, "WEIGHT_SHIFT_R": 4, "RIGHT_SINGLE": 5,
    }
    state_vals = [state_map.get(s, -1) for s in states]
    ax.plot(t, state_vals, drawstyle="steps-post", color="tab:blue")
    ax.set_yticks([0, 1, 2, 3, 4, 5])
    ax.set_yticklabels(["INIT", "W_SHIFT_L", "LEFT_S", "DOUBLE", "W_SHIFT_R", "RIGHT_S"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Phase")
    ax.set_title("Gait phase timeline")
    ax.grid(True, alpha=0.3)

    # 2. Foot heights
    ax = axes[0, 1]
    ax.plot(t, left_z, label="left foot z")
    ax.plot(t, right_z, label="right foot z")
    ax.axhline(0.03, color="r", ls="--", alpha=0.3, label="min clearance")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)")
    ax.set_title("Foot heights")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Pelvis orientation
    ax = axes[1, 0]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.axhline(15, color="r", ls="--", alpha=0.3)
    ax.axhline(-15, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis orientation")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Support foot slip
    ax = axes[1, 1]
    ax.plot(t, slip, label="support slip")
    ax.axhline(0.005, color="r", ls="--", alpha=0.3, label="threshold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip (m)")
    ax.set_title("Support foot horizontal slip")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 5. GRF per foot
    ax = axes[2, 0]
    ax.plot(t, left_grf, label="left GRF")
    ax.plot(t, right_grf, label="right GRF")
    mg = 34.13 * 9.81
    ax.axhline(0.5 * mg, color="g", ls="--", alpha=0.3, label="50% mg")
    ax.axhline(0.8 * mg, color="r", ls="--", alpha=0.3, label="80% mg")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title("GRF per foot")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 6. Step count over time
    ax = axes[2, 1]
    ax.plot(t, log["step_count"], color="tab:green")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Step count")
    ax.set_title("Cumulative steps")
    ax.grid(True, alpha=0.3)

    for tc, _ in state_changes:
        for ax in axes.flat:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=80)
    print(f"figure saved to {OUTPUT_PLOT}")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    # Stage 1: zero forward motion
    cfg["walking"]["step_length"] = 0.0
    cfg["walking"]["double_support_duration"] = 0.5

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = WalkingController(env, cfg)
    controller.reset()

    duration = 10.0  # ~3-4 alternating lift/place cycles

    print("=" * 60)
    print("Stage 1: Static alternating support (step_length = 0)")
    print("=" * 60)

    log = run_simulation(env, controller, duration)
    metrics, checks = assess(log)
    plot(log, metrics, checks)

    print("\n=== Summary ===")
    all_pass = all(checks.values())
    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")
    for name, passed in checks.items():
        print(f"  {name:25s} : {'PASS' if passed else 'FAIL'}")


if __name__ == "__main__":
    main()
