"""Diagnose why CoM never shifts during the WEIGHT_SHIFT phase.

Runs the TransitionController only through BIPEDAL + WEIGHT_SHIFT and logs
the signals that distinguish the candidate failure modes (planner, QP
tracking, contact-force redistribution, unload trigger). Produces a single
figure at outputs/debug_weight_shift.png.
"""
from __future__ import annotations

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from utils.kinematics import compute_com_position, compute_com_velocity, compute_contact_wrench


CONFIG_PATH = "configs/g1_config.yaml"
OUTPUT_PATH = "outputs/debug_weight_shift.png"
TIME_CAP = 4.5  # stop if state machine doesn't advance


def run(env: G1Env, controller: TransitionController) -> dict:
    dt = env.dt
    n_steps = int(TIME_CAP / dt)
    body_left = env.cfg["robot"]["body_names"]["left_foot"]
    body_right = env.cfg["robot"]["body_names"]["right_foot"]

    log = {
        "t": [],
        "state": [],
        "phase_dt": [],
        "com_actual": [],
        "com_ref": [],
        "com_vel_actual": [],
        "com_vel_ref": [],
        "force_left": [],
        "force_right": [],
        "support_target_y": controller.com_target_single[1],
        "support_pos_y": controller.com_start[1],  # placeholder, overwritten below
    }

    # Read the support foot's actual y once; the controller uses this as
    # the inward-shifted CoM target.
    support_pos = env.get_body_pos(f"{controller.support_foot_name}_foot")
    log["support_pos_y"] = float(support_pos[1])

    for _ in range(n_steps):
        try:
            ctrl = controller.compute()
        except RuntimeError as exc:
            print(f"controller failed: {exc}")
            break

        env.step(ctrl)

        if not np.all(np.isfinite(env.data.qpos)):
            print("NaN detected, stopping.")
            break

        t_now = env.data.time
        state = controller.state
        phase_dt = controller.phase_elapsed

        com_actual = compute_com_position(env.model, env.data)
        com_vel_actual = compute_com_velocity(env.model, env.data)

        if state == "BIPEDAL":
            com_ref = controller.com_start.copy()
            com_vel_ref = np.zeros(3)
        elif state == "WEIGHT_SHIFT":
            com_ref, com_vel_ref, _ = controller.com_planner.evaluate(phase_dt)
        else:  # SINGLE_LEG (we stop here)
            com_ref = controller.com_target_single.copy()
            com_vel_ref = np.zeros(3)

        force_left = compute_contact_wrench(env.model, env.data, body_left)
        force_right = compute_contact_wrench(env.model, env.data, body_right)

        log["t"].append(t_now)
        log["state"].append(state)
        log["phase_dt"].append(phase_dt)
        log["com_actual"].append(com_actual)
        log["com_ref"].append(com_ref)
        log["com_vel_actual"].append(com_vel_actual)
        log["com_vel_ref"].append(com_vel_ref)
        log["force_left"].append(force_left)
        log["force_right"].append(force_right)

        if state == "SINGLE_LEG":
            print(f"transitioned to SINGLE_LEG at t={t_now:.3f} s -- stopping log.")
            break

    return log


def plot(log: dict, controller: TransitionController, body_weight_n: float) -> None:
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)

    t = np.array(log["t"])
    com_actual = np.array(log["com_actual"])
    com_ref = np.array(log["com_ref"])
    com_vel_actual = np.array(log["com_vel_actual"])
    com_vel_ref = np.array(log["com_vel_ref"])
    force_left = np.array(log["force_left"])
    force_right = np.array(log["force_right"])
    states = np.array(log["state"])

    swing_is_left = controller.swing_foot_name == "left"
    fz_swing = force_left[:, 2] if swing_is_left else force_right[:, 2]
    fz_support = force_right[:, 2] if swing_is_left else force_left[:, 2]

    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for index, value in enumerate(states):
        if value != prev:
            state_changes.append((t[index], value))
            prev = value

    fig, axes = plt.subplots(3, 2, figsize=(14, 11))

    # 1. CoM y: ref vs actual (the smoking gun)
    ax = axes[0, 0]
    ax.plot(t, com_ref[:, 1], label="CoM y reference", lw=2)
    ax.plot(t, com_actual[:, 1], label="CoM y actual", lw=2)
    ax.axhline(log["support_pos_y"], color="k", ls=":", alpha=0.6, label="support foot y")
    ax.axhline(controller.com_target_single[1], color="r", ls=":", alpha=0.6, label="CoM target (final)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("y (m)")
    ax.set_title("CoM y: tracking the planner reference")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. CoM x: ref vs actual (sanity check; should stay near zero)
    ax = axes[0, 1]
    ax.plot(t, com_ref[:, 0], label="CoM x reference", lw=2)
    ax.plot(t, com_actual[:, 0], label="CoM x actual", lw=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("x (m)")
    ax.set_title("CoM x: tracking (should be flat)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Per-foot vertical contact force
    ax = axes[1, 0]
    ax.plot(t, force_left[:, 2], label="left foot fz")
    ax.plot(t, force_right[:, 2], label="right foot fz")
    ax.axhline(body_weight_n, color="k", ls=":", alpha=0.5, label=f"body weight = {body_weight_n:.0f} N")
    ax.axhline(body_weight_n / 2, color="grey", ls=":", alpha=0.5, label="half body weight")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("fz (N)")
    ax.set_title("Vertical contact force on each foot")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Swing foot vertical force vs unload threshold
    ax = axes[1, 1]
    ax.plot(t, fz_swing, label=f"swing ({controller.swing_foot_name}) fz")
    ax.axhline(controller.swing_unload_force, color="r", ls="--", alpha=0.6,
               label=f"unload threshold = {controller.swing_unload_force:.0f} N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("fz (N)")
    ax.set_title("Swing foot fz vs unload trigger")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 5. CoM y velocity: ref vs actual (does QP move CoM as the planner asks?)
    ax = axes[2, 0]
    ax.plot(t, com_vel_ref[:, 1], label="CoM vy reference", lw=2)
    ax.plot(t, com_vel_actual[:, 1], label="CoM vy actual", lw=2)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("vy (m/s)")
    ax.set_title("CoM y velocity: ref vs actual")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 6. Lateral contact force on each foot (which way is the QP pushing?)
    ax = axes[2, 1]
    ax.plot(t, force_left[:, 1], label="left foot fy")
    ax.plot(t, force_right[:, 1], label="right foot fy")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("fy (N)")
    ax.set_title("Lateral (y) contact force on each foot")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Mark state transitions on every panel
    for ax in axes.flat:
        for tc, label in state_changes:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)
            ax.text(tc, ax.get_ylim()[1], label, color="g",
                    fontsize=7, rotation=90, va="top", ha="right")

    plt.tight_layout()
    plt.savefig(OUTPUT_PATH, dpi=150)
    print(f"figure saved to {OUTPUT_PATH}")


def summarize(log: dict, controller: TransitionController) -> None:
    if not log["t"]:
        print("(no samples logged)")
        return

    t = np.array(log["t"])
    com_actual = np.array(log["com_actual"])
    com_ref = np.array(log["com_ref"])
    states = np.array(log["state"])

    weight_mask = states == "WEIGHT_SHIFT"
    if weight_mask.any():
        com_y_ref_end = com_ref[weight_mask, 1][-1]
        com_y_actual_end = com_actual[weight_mask, 1][-1]
        peak_err = np.max(np.abs(com_actual[weight_mask, 1] - com_ref[weight_mask, 1]))
        print(f"WEIGHT_SHIFT lasted from t={t[weight_mask][0]:.3f} s "
              f"to t={t[weight_mask][-1]:.3f} s ({weight_mask.sum()} samples).")
        print(f"  Final CoM y ref:    {com_y_ref_end:+.4f} m")
        print(f"  Final CoM y actual: {com_y_actual_end:+.4f} m")
        print(f"  Peak |y tracking error|: {peak_err:.4f} m")
    else:
        print("WEIGHT_SHIFT phase never entered.")
    print(f"final state at end of log: {states[-1]}")


def main() -> None:
    with open(CONFIG_PATH, "r") as cfg_file:
        cfg = yaml.safe_load(cfg_file)

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = TransitionController(env, cfg)
    controller.reset()

    body_weight_n = float(np.sum(env.model.body_mass) * abs(env.model.opt.gravity[2]))

    print("=" * 60)
    print("Weight-shift debug: BIPEDAL + WEIGHT_SHIFT only")
    print("=" * 60)
    print(f"support foot: {controller.support_foot_name}")
    print(f"CoM start:    {controller.com_start}")
    print(f"CoM target:   {controller.com_target_single}")
    print(f"body weight:  {body_weight_n:.2f} N")
    print()

    log = run(env, controller)
    summarize(log, controller)
    plot(log, controller, body_weight_n)


if __name__ == "__main__":
    main()
