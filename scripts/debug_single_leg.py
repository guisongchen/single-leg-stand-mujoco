"""Diagnose the SINGLE_LEG phase of the transition controller.

Logs QP wrench decisions, support-foot slip, swing-foot tracking,
pelvis orientation, and CoM behaviour through the full state machine.
Produces outputs/debug_single_leg.png.
"""
from __future__ import annotations

import os
import sys
import types

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    compute_contact_wrench,
    euler_from_quat,
)

CONFIG_PATH = "configs/g1_config.yaml"
OUTPUT_PATH = "outputs/debug_single_leg.png"


def run(env: G1Env, controller: TransitionController) -> dict:
    duration = (
        controller.t_bipedal
        + controller.t_weight_shift
        + controller.t_single_leg
        + 0.5
    )
    n_steps = int(duration / env.dt)

    body_left = env.cfg["robot"]["body_names"]["left_foot"]
    body_right = env.cfg["robot"]["body_names"]["right_foot"]

    log = {
        "t": [],
        "state": [],
        "com_actual": [],
        "com_ref": [],
        "com_vel_actual": [],
        "pelvis_roll": [],
        "pelvis_pitch": [],
        "support_pos": [],
        "support_vel": [],
        "swing_pos": [],
        "swing_ref": [],
        "swing_vel": [],
        "force_left": [],
        "force_right": [],
        "qp_active_names": [],
        "qp_wrenches": [],
        "cam": [],
    }

    # Monkey-patch _solve_qp to capture the QP's wrench decisions.
    original_solve_qp = controller._solve_qp

    def logged_solve_qp(self, *args, **kwargs):
        qacc, wrenches, tau = original_solve_qp(*args, **kwargs)
        log["qp_active_names"].append([f["name"] for f in kwargs.get("active_feet", [])])
        log["qp_wrenches"].append([w.copy() for w in wrenches])
        return qacc, wrenches, tau

    controller._solve_qp = types.MethodType(logged_solve_qp, controller)

    try:
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
            elif state == "WEIGHT_SHIFT":
                com_ref, _, _ = controller.com_planner.evaluate(phase_dt)
            else:
                com_ref = controller.com_target_single.copy()

            quat = env.get_pelvis_quat()
            roll, pitch, _ = euler_from_quat(*quat)

            support_pos = env.get_body_pos(f"{controller.support_foot_name}_foot")
            support_vel = np.zeros(6)
            J_support = np.zeros((6, env.model.nv))
            mujoco.mj_jacBody(
                env.model, env.data, J_support[:3], J_support[3:], controller._support_bid
            )
            support_vel = J_support @ env.data.qvel

            swing_pos = env.get_body_pos(f"{controller.swing_foot_name}_foot")
            swing_vel = np.zeros(3)
            J_swing = np.zeros((6, env.model.nv))
            mujoco.mj_jacBody(
                env.model, env.data, J_swing[:3], J_swing[3:], controller._swing_bid
            )
            swing_vel = J_swing[:3] @ env.data.qvel

            if state == "SINGLE_LEG" and controller.swing_foot_planner is not None:
                swing_ref, swing_ref_vel, _ = controller.swing_foot_planner.evaluate(phase_dt)
            else:
                swing_ref = swing_pos.copy()
                swing_ref_vel = np.zeros(3)

            cam, _, _ = controller._compute_cam(env.model, env.data, com_actual)

            log["t"].append(t_now)
            log["state"].append(state)
            log["com_actual"].append(com_actual)
            log["com_ref"].append(com_ref)
            log["com_vel_actual"].append(com_vel_actual)
            log["pelvis_roll"].append(roll)
            log["pelvis_pitch"].append(pitch)
            log["support_pos"].append(support_pos)
            log["support_vel"].append(support_vel)
            log["swing_pos"].append(swing_pos)
            log["swing_ref"].append(swing_ref)
            log["swing_vel"].append(swing_vel)
            log["force_left"].append(compute_contact_wrench(env.model, env.data, body_left))
            log["force_right"].append(compute_contact_wrench(env.model, env.data, body_right))
            log["cam"].append(cam.copy())
    finally:
        controller._solve_qp = original_solve_qp

    return log


def plot(log: dict, controller: TransitionController) -> None:
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)

    t = np.array(log["t"])
    com_actual = np.array(log["com_actual"])
    com_ref = np.array(log["com_ref"])
    support_pos = np.array(log["support_pos"])
    support_vel = np.array(log["support_vel"])
    swing_pos = np.array(log["swing_pos"])
    swing_ref = np.array(log["swing_ref"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    states = np.array(log["state"])

    force_left = np.array(log["force_left"])
    force_right = np.array(log["force_right"])
    swing_is_left = controller.swing_foot_name == "left"
    mc_support_fz = force_right[:, 2] if swing_is_left else force_left[:, 2]
    mc_support_fx = force_right[:, 0] if swing_is_left else force_left[:, 0]
    mc_support_fy = force_right[:, 1] if swing_is_left else force_left[:, 1]

    # Extract QP support wrench by name mapping
    qp_support_fz = []
    qp_support_fx = []
    qp_support_fy = []
    qp_support_tx = []
    qp_support_ty = []
    for names, wrenches in zip(log["qp_active_names"], log["qp_wrenches"]):
        support_name = f"{controller.support_foot_name}_foot"
        if support_name in names:
            idx = names.index(support_name)
            w = wrenches[idx]
            qp_support_fx.append(w[0])
            qp_support_fy.append(w[1])
            qp_support_fz.append(w[2])
            # Linear-only (m=3) wrenches have no torque components.
            qp_support_tx.append(w[3] if len(w) >= 6 else np.nan)
            qp_support_ty.append(w[4] if len(w) >= 6 else np.nan)
        else:
            qp_support_fx.append(np.nan)
            qp_support_fy.append(np.nan)
            qp_support_fz.append(np.nan)
            qp_support_tx.append(np.nan)
            qp_support_ty.append(np.nan)

    qp_support_fz = np.array(qp_support_fz)
    qp_support_fx = np.array(qp_support_fx)
    qp_support_fy = np.array(qp_support_fy)
    qp_support_tx = np.array(qp_support_tx)
    qp_support_ty = np.array(qp_support_ty)

    mu = controller.mu
    cop_y_half = controller.foot_cop_y_half
    cop_x_back = controller.foot_cop_x_back
    cop_x_forward = controller.foot_cop_x_forward

    # Friction & CoP utilisation (guard div-by-zero). The CoP envelope is
    # asymmetric in x, so ty has different active bounds for positive
    # (heel-side) vs negative (toe-side) torques.
    with np.errstate(divide="ignore", invalid="ignore"):
        fx_util = np.abs(qp_support_fx) / (mu * qp_support_fz)
        fy_util = np.abs(qp_support_fy) / (mu * qp_support_fz)
        tx_util = np.abs(qp_support_tx) / (cop_y_half * qp_support_fz)
        ty_util = np.where(
            qp_support_ty >= 0.0,
            qp_support_ty / (cop_x_back * qp_support_fz),
            -qp_support_ty / (cop_x_forward * qp_support_fz),
        )

    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for idx, val in enumerate(states):
        if val != prev:
            state_changes.append((t[idx], val))
            prev = val

    cam = np.array(log["cam"])
    cam_mag = np.linalg.norm(cam, axis=1)
    com_in_foot = com_actual[:, :2] - support_pos[:, :2]

    # CoP location inside the support rectangle (foot-frame approximation):
    # tx = cop_y * fz, ty = -cop_x * fz, so cop_x = -ty/fz, cop_y = tx/fz.
    with np.errstate(divide="ignore", invalid="ignore"):
        cop_x = -qp_support_ty / qp_support_fz
        cop_y = qp_support_tx / qp_support_fz

    fig, axes = plt.subplots(4, 2, figsize=(14, 14))

    # 1. CoM tracking error in XY
    ax = axes[0, 0]
    ax.plot(t, com_actual[:, 0] - com_ref[:, 0], label="x error")
    ax.plot(t, com_actual[:, 1] - com_ref[:, 1], label="y error")
    ax.axhline(0, color="k", ls=":", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("CoM tracking error (actual - reference)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 2. Support foot horizontal slip velocity
    ax = axes[0, 1]
    ax.plot(t, np.linalg.norm(support_vel[:, :2], axis=1), label="|v_xy|")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (m/s)")
    ax.set_title("Support foot horizontal slip velocity")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 3. Swing foot tracking error
    ax = axes[1, 0]
    ax.plot(t, swing_pos[:, 0] - swing_ref[:, 0], label="x error")
    ax.plot(t, swing_pos[:, 1] - swing_ref[:, 1], label="y error")
    ax.plot(t, swing_pos[:, 2] - swing_ref[:, 2], label="z error")
    ax.axhline(0, color="k", ls=":", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.set_title("Swing foot position tracking error")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 4. Pelvis orientation
    ax = axes[1, 1]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.axhline(15, color="r", ls="--", alpha=0.3)
    ax.axhline(-15, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis orientation")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 5. Support fz: QP decision vs MuJoCo reality
    ax = axes[2, 0]
    ax.plot(t, qp_support_fz, label="QP support fz")
    ax.plot(t, mc_support_fz, label="MuJoCo support fz", ls="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("fz (N)")
    ax.set_title("Support foot vertical force: QP vs MuJoCo")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # 6. Friction & CoP constraint utilisation
    ax = axes[2, 1]
    ax.plot(t, fx_util, label="|fx| / (mu*fz)")
    ax.plot(t, fy_util, label="|fy| / (mu*fz)")
    ax.plot(t, tx_util, label="|tx| / (cop_y_half*fz)")
    ax.plot(t, ty_util, label="ty utilisation (asym)")
    ax.axhline(1.0, color="r", ls="--", alpha=0.4, label="bound = 1.0")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Utilisation ratio")
    ax.set_title("QP support-foot inequality constraint utilisation")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # 7. CoP path inside the support rectangle (foot-frame, world-aligned)
    ax = axes[3, 0]
    rect_x = [-cop_x_back, cop_x_forward, cop_x_forward, -cop_x_back, -cop_x_back]
    rect_y = [-cop_y_half, -cop_y_half, cop_y_half, cop_y_half, -cop_y_half]
    ax.plot(rect_x, rect_y, "k-", lw=1.5, alpha=0.6)
    single_mask_arr = states == "SINGLE_LEG"
    # Drop divide-by-zero outliers (post-collapse fz ~ 0) and points
    # outside the rectangle (those mean the QP gave up or the foot left
    # the ground -- not interesting for the slow-drift question).
    margin = 0.02
    in_rect = (
        (cop_x > -cop_x_back - margin)
        & (cop_x < cop_x_forward + margin)
        & (cop_y > -cop_y_half - margin)
        & (cop_y < cop_y_half + margin)
    )
    valid = (
        np.isfinite(cop_x)
        & np.isfinite(cop_y)
        & single_mask_arr
        & (qp_support_fz > 100.0)
        & in_rect
    )
    if valid.any():
        sc = ax.scatter(
            cop_x[valid], cop_y[valid], c=t[valid], cmap="viridis", s=4,
        )
        plt.colorbar(sc, ax=ax, label="time (s)")
    ax.set_xlim(-cop_x_back - margin, cop_x_forward + margin)
    ax.set_ylim(-cop_y_half - margin, cop_y_half + margin)
    ax.set_xlabel("CoP x (foot frame, m)")
    ax.set_ylabel("CoP y (foot frame, m)")
    ax.set_title("QP CoP path inside support rectangle (SINGLE_LEG only)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    # 8. CAM magnitude and CoM-in-foot-frame xy (the slow-drift diagnostics)
    ax = axes[3, 1]
    ax_twin = ax.twinx()
    ax.plot(t, com_in_foot[:, 0], label="CoM x in foot frame", color="tab:blue")
    ax.plot(t, com_in_foot[:, 1], label="CoM y in foot frame", color="tab:orange")
    ax.axhline(-cop_x_back, color="tab:blue", ls=":", alpha=0.4)
    ax.axhline(cop_x_forward, color="tab:blue", ls=":", alpha=0.4)
    ax.axhline(-cop_y_half, color="tab:orange", ls=":", alpha=0.4)
    ax.axhline(cop_y_half, color="tab:orange", ls=":", alpha=0.4)
    ax_twin.plot(t, cam_mag, label="|CAM|", color="tab:red", ls="--", alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CoM offset (m)")
    ax_twin.set_ylabel("|CAM| (kg m^2/s)", color="tab:red")
    ax_twin.tick_params(axis="y", labelcolor="tab:red")
    ax.set_title("CoM in foot frame & centroidal angular momentum")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax_twin.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7, loc="upper left")
    ax.grid(True, alpha=0.3)

    for i, ax in enumerate(axes.flat):
        if i == 6:  # axes[3, 0] is the CoP scatter, x-axis is not time
            continue
        for tc, label in state_changes:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)
            ax.text(tc, ax.get_ylim()[1], label, color="g",
                    fontsize=7, rotation=90, va="top", ha="right")

    plt.tight_layout()
    plt.savefig(OUTPUT_PATH, dpi=150)
    print(f"figure saved to {OUTPUT_PATH}")


def summarize(log: dict, controller: TransitionController) -> None:
    t = np.array(log["t"])
    states = np.array(log["state"])
    com = np.array(log["com_actual"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    support_pos = np.array(log["support_pos"])

    single_mask = states == "SINGLE_LEG"
    ws_mask = states == "WEIGHT_SHIFT"
    print(f"BIPEDAL:   {t[states == 'BIPEDAL'][-1] if (states == 'BIPEDAL').any() else 0:.3f} s")
    if ws_mask.any():
        print(f"WEIGHT_SHIFT: {t[ws_mask][0]:.3f} s -> {t[ws_mask][-1]:.3f} s")
    if single_mask.any():
        print(f"SINGLE_LEG:   {t[single_mask][0]:.3f} s -> {t[single_mask][-1]:.3f} s")
        print(f"  max |roll|  = {np.max(np.abs(roll[single_mask])):.2f} deg")
        print(f"  max |pitch| = {np.max(np.abs(pitch[single_mask])):.2f} deg")
        print(f"  support drift (xy) = {np.max(np.linalg.norm(support_pos[single_mask, :2] - support_pos[single_mask][0, :2], axis=1)):.4f} m")
        print(f"  final CoM err from support = {np.linalg.norm(com[-1, :2] - support_pos[-1, :2]):.4f} m")
    else:
        print("SINGLE_LEG phase never entered.")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = TransitionController(env, cfg)
    controller.reset()

    print("=" * 60)
    print("Single-leg phase debug: full run with QP wrench logging")
    print("=" * 60)
    print(f"support foot: {controller.support_foot_name}")
    print()

    log = run(env, controller)
    summarize(log, controller)
    plot(log, controller)


if __name__ == "__main__":
    main()
