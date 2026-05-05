"""Push-recovery envelope test for single-leg stance.

Applies short-duration impulses to the pelvis during SINGLE_LEG and
records whether the robot recovers. Scans upward in impulse magnitude
to find the failure threshold.

Produces outputs/test_push_recovery.png and a text summary.
"""
from __future__ import annotations

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from utils.kinematics import compute_com_position, euler_from_quat

CONFIG_PATH = "configs/g1_config.yaml"
OUTPUT_PLOT = "outputs/test_push_recovery.png"

# --------------------------------------------------------------------------- #
# Test parameters
# --------------------------------------------------------------------------- #
DURATION_AFTER_PUSH = 2.0          # seconds to continue after impulse
IMPULSE_DURATION = 0.05            # seconds (50 ms shove)
MIN_SETTLE_TIME = 1.5              # seconds into SINGLE_LEG before pushing
DIRECTIONS = [                     # impulse directions in world XY (deg)
    0,      # +x (forward)
    90,     # +y (left)
]


def apply_impulse(env: G1Env, magnitude: float, angle_deg: float) -> None:
    """Set an external force on the pelvis for one step."""
    angle = np.deg2rad(angle_deg)
    force = np.array([
        magnitude * np.cos(angle),
        magnitude * np.sin(angle),
        0.0,
    ])
    # Apply at pelvis CoM (body origin)
    env.data.qfrc_applied[:] = 0.0
    # We need to map world-frame force to generalized force.
    # The pelvis is body 1 (floating base). Use mj_applyFT.
    import mujoco
    mujoco.mj_applyFT(
        env.model,
        env.data,
        force,
        np.zeros(3),          # no torque
        env.data.xipos[env._body_ids["pelvis"]],
        env._body_ids["pelvis"],
        env.data.qfrc_applied,
    )


def run_trial(
    env: G1Env,
    controller: TransitionController,
    magnitude: float,
    angle_deg: float,
) -> dict:
    """Run one push-recovery trial. Returns pass/fail + metrics."""
    env.reset()
    controller.reset()

    dt = env.dt
    n_impulse_steps = int(IMPULSE_DURATION / dt)
    total_single_leg_time = controller.t_single_leg + DURATION_AFTER_PUSH

    # Pre-compute total steps
    duration = (
        controller.t_bipedal
        + controller.t_weight_shift
        + total_single_leg_time
    )
    n_steps = int(duration / dt)

    pushed = False
    push_step = 0
    result = {
        "magnitude": magnitude,
        "angle_deg": angle_deg,
        "recovered": True,
        "push_time": None,
        "max_roll_after_push": 0.0,
        "max_pitch_after_push": 0.0,
        "min_pelvis_z": float("inf"),
        "failure_reason": None,
    }

    for step in range(n_steps):
        t = env.data.time
        state = controller.state

        # Decide when to push (random within valid window)
        if state == "SINGLE_LEG" and not pushed:
            phase_dt = controller.phase_elapsed
            if phase_dt >= MIN_SETTLE_TIME:
                # Apply impulse now
                pushed = True
                push_step = step
                result["push_time"] = t

        # Apply impulse force for n_impulse_steps
        if pushed and step - push_step < n_impulse_steps:
            apply_impulse(env, magnitude, angle_deg)
        else:
            env.data.qfrc_applied[:] = 0.0

        try:
            ctrl = controller.compute()
        except RuntimeError:
            result["recovered"] = False
            result["failure_reason"] = "controller_runtime_error"
            break

        env.step(ctrl)

        # ---- Failure checks ---------------------------------------------
        pelvis_z = env.get_body_pos("pelvis")[2]
        quat = env.get_pelvis_quat()
        roll, pitch, _ = euler_from_quat(*quat)

        if pushed:
            result["max_roll_after_push"] = max(
                result["max_roll_after_push"], abs(roll)
            )
            result["max_pitch_after_push"] = max(
                result["max_pitch_after_push"], abs(pitch)
            )
            result["min_pelvis_z"] = min(result["min_pelvis_z"], pelvis_z)

        if not np.all(np.isfinite(env.data.qpos)):
            result["recovered"] = False
            result["failure_reason"] = "nan"
            break

        if pelvis_z < 0.5:
            result["recovered"] = False
            result["failure_reason"] = "pelvis_too_low"
            break

        if abs(roll) > np.deg2rad(45) or abs(pitch) > np.deg2rad(45):
            result["recovered"] = False
            result["failure_reason"] = "excessive_tilt"
            break

        # Support foot lost contact?
        if state == "SINGLE_LEG":
            support_fz = controller._support_fz()
            if pushed and support_fz < 10.0:
                result["recovered"] = False
                result["failure_reason"] = "support_lost_contact"
                break

    env.data.qfrc_applied[:] = 0.0
    return result


def scan_threshold(
    env: G1Env,
    controller_cls,
    cfg: dict,
) -> list[dict]:
    """Scan impulse magnitudes upward until all directions fail."""
    results = []
    magnitude = 10.0
    while True:
        direction_results = []
        for angle in DIRECTIONS:
            # Re-create env + controller for clean state each trial
            env.reset()
            controller = controller_cls(env, cfg)
            controller.reset()
            trial = run_trial(env, controller, magnitude, angle)
            direction_results.append(trial)
            results.append(trial)

        n_pass = sum(1 for r in direction_results if r["recovered"])
        print(
            f"  {magnitude:5.1f} N  |  {n_pass}/{len(DIRECTIONS)} passed  "
            + f"({', '.join(r['failure_reason'] or 'ok' for r in direction_results)})"
        )

        if n_pass == 0:
            break
        magnitude += 20.0

        # Safety cap
        if magnitude > 300.0:
            print("  stopped at 300 N (safety cap)")
            break

    return results


def plot(results: list[dict]) -> None:
    os.makedirs(os.path.dirname(OUTPUT_PLOT), exist_ok=True)

    # Group by magnitude
    mags = sorted(set(r["magnitude"] for r in results))
    pass_rates = []
    for m in mags:
        subset = [r for r in results if r["magnitude"] == m]
        rate = sum(1 for r in subset if r["recovered"]) / len(subset)
        pass_rates.append(rate)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Pass rate vs magnitude
    ax = axes[0]
    ax.plot(mags, pass_rates, "o-", color="tab:blue")
    ax.axhline(0.5, color="r", ls="--", alpha=0.4, label="50 % threshold")
    ax.set_xlabel("Impulse magnitude (N)")
    ax.set_ylabel("Recovery pass rate")
    ax.set_title("Push-recovery envelope")
    ax.set_ylim(-0.05, 1.05)
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Max tilt after push (colored by recovery)
    ax = axes[1]
    for recovered, color, label in [(True, "tab:green", "recovered"), (False, "tab:red", "failed")]:
        subset = [r for r in results if r["recovered"] == recovered]
        ax.scatter(
            [np.rad2deg(r["max_roll_after_push"]) for r in subset],
            [np.rad2deg(r["max_pitch_after_push"]) for r in subset],
            c=color,
            s=60,
            alpha=0.6,
            label=label,
        )
    ax.set_xlabel("Max |roll| after push (deg)")
    ax.set_ylabel("Max |pitch| after push (deg)")
    ax.set_title("Post-push attitude excursion")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    print(f"figure saved to {OUTPUT_PLOT}")


def summarize(results: list[dict]) -> None:
    mags = sorted(set(r["magnitude"] for r in results))
    print("\n=== Push-recovery summary ===")
    for m in mags:
        subset = [r for r in results if r["magnitude"] == m]
        n_pass = sum(1 for r in subset if r["recovered"])
        avg_roll = np.mean([np.rad2deg(r["max_roll_after_push"]) for r in subset])
        avg_pitch = np.mean([np.rad2deg(r["max_pitch_after_push"]) for r in subset])
        print(
            f"  {m:5.1f} N : {n_pass}/{len(subset)} passed  "
            f"avg roll={avg_roll:4.1f}° pitch={avg_pitch:4.1f}°"
        )

    # Find threshold (first magnitude where pass rate drops below 50%)
    for m in mags:
        subset = [r for r in results if r["magnitude"] == m]
        rate = sum(1 for r in subset if r["recovered"]) / len(subset)
        if rate < 0.5:
            print(f"\n  Estimated recovery threshold: < {m:.1f} N")
            break
    else:
        print(f"\n  All tested magnitudes passed (threshold > {mags[-1]:.1f} N)")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    env = G1Env(CONFIG_PATH)

    print("=" * 60)
    print("Push-recovery envelope scan")
    print("=" * 60)
    print(f"Impulse duration: {IMPULSE_DURATION*1000:.0f} ms")
    print(f"Directions: {DIRECTIONS}°")
    print(f"Push window: >= {MIN_SETTLE_TIME:.1f} s into SINGLE_LEG")
    print()

    results = scan_threshold(env, TransitionController, cfg)
    summarize(results)
    plot(results)


if __name__ == "__main__":
    main()
