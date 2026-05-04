"""Stage 1 walking test: static alternating support (step_length = 0).

Runs the WalkingController with zero forward step length to verify the
periodic state machine can lift and place each foot without falling.

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
    }

    # Store initial support foot positions for slip measurement
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

        # Support slip: whichever foot is supposed to be on ground
        if state == "LEFT_SWING":
            slip = np.linalg.norm(right_pos[:2] - right_init)
        elif state == "RIGHT_SWING":
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

    return log


def assess(log: dict) -> dict:
    """Compute pass/fail metrics."""
    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])

    # Check for falls
    pelvis_z = np.array([0.0])  # we didn't log pelvis z directly
    max_roll = np.max(np.abs(roll))
    max_pitch = np.max(np.abs(pitch))

    # Foot clearance during swing
    left_swing_mask = states == "LEFT_SWING"
    right_swing_mask = states == "RIGHT_SWING"
    left_clearance = np.max(left_z[left_swing_mask]) - np.min(left_z) if left_swing_mask.any() else 0.0
    right_clearance = np.max(right_z[right_swing_mask]) - np.min(right_z) if right_swing_mask.any() else 0.0

    # Step count
    total_steps = log["step_count"][-1] if log["step_count"] else 0

    # State alternation
    unique_states = [s for s in ["INIT", "LEFT_SWING", "DOUBLE_SUPPORT", "RIGHT_SWING"] if s in states]

    checks = {
        "no_fall": max_roll < 15.0 and max_pitch < 15.0,
        "left_foot_clearance": left_clearance > 0.02,
        "right_foot_clearance": right_clearance > 0.02,
        "support_slip": np.max(slip) < 0.005,
        "min_steps": total_steps >= 4,
        "states_present": len(unique_states) >= 4,
    }

    metrics = {
        "max_roll_deg": float(max_roll),
        "max_pitch_deg": float(max_pitch),
        "left_clearance_m": float(left_clearance),
        "right_clearance_m": float(right_clearance),
        "max_support_slip_m": float(np.max(slip)),
        "total_steps": int(total_steps),
        "duration_s": float(t[-1]) if len(t) > 0 else 0.0,
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

    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for idx, val in enumerate(states):
        if val != prev:
            state_changes.append((t[idx], val))
            prev = val

    fig, axes = plt.subplots(3, 2, figsize=(14, 11))

    # 1. Gait phase timeline
    ax = axes[0, 0]
    state_map = {"INIT": 0, "LEFT_SWING": 1, "DOUBLE_SUPPORT": 2, "RIGHT_SWING": 3}
    state_vals = [state_map.get(s, -1) for s in states]
    ax.plot(t, state_vals, drawstyle="steps-post", color="tab:blue")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["INIT", "LEFT_SWING", "DOUBLE", "RIGHT_SWING"])
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

    # 5. Pass/fail summary
    ax = axes[2, 0]
    ax.axis("off")
    lines = ["=== Stage 1 Assessment ===", ""]
    for name, passed in checks.items():
        status = "PASS" if passed else "FAIL"
        lines.append(f"  {name:25s} : {status}")
    lines.append("")
    for name, val in metrics.items():
        lines.append(f"  {name:25s} : {val:.4f}" if isinstance(val, float) else f"  {name:25s} : {val}")
    ax.text(0.1, 0.5, "\n".join(lines), transform=ax.transAxes,
            fontfamily="monospace", fontsize=9, verticalalignment="center")

    # 6. Step count over time
    ax = axes[2, 1]
    ax.plot(t, log["step_count"], color="tab:green")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Step count")
    ax.set_title("Cumulative steps")
    ax.grid(True, alpha=0.3)

    for tc, label in state_changes:
        for ax in axes.flat[:4]:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    print(f"figure saved to {OUTPUT_PLOT}")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    # Stage 1: zero forward motion
    cfg["walking"]["step_length"] = 0.0
    cfg["walking"]["single_support_duration"] = 2.0
    cfg["walking"]["double_support_duration"] = 0.5

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = WalkingController(env, cfg)
    controller.reset()

    duration = 10.0  # ~3-4 alternating steps

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
