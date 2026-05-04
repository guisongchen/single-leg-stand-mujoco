"""Step-by-step test: Single-leg -> Bipedal return.

Uses TransitionController to reach SINGLE_LEG, then adds a manual
BIPEDAL_RETURN phase that lowers the swing foot and shifts the CoM
back to the midpoint.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import yaml
import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from planners.com_planner import ComPlanner
from planners.swing_foot_planner import SwingFootPlanner
from utils.kinematics import compute_com_position, compute_contact_wrench, euler_from_quat

CONFIG_PATH = "configs/g1_config.yaml"


def run(env: G1Env, ctrl: TransitionController, duration: float) -> dict:
    """Run transition controller with a manual bipedal-return tacked on."""
    n_steps = int(duration / env.dt)
    dt = env.dt

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
        "left_foot_fz": [],
        "right_foot_fz": [],
        "max_tau": [],
        "touchdown": [],
    }

    left_init = env.get_body_pos("left_foot")[:2].copy()
    right_init = env.get_body_pos("right_foot")[:2].copy()

    # Reference positions for when we enter bipedal-return
    com_start = ctrl.com_start.copy()
    swing_foot_start = None
    com_planner_return = None
    swing_planner_return = None
    support_foot_name = ctrl.support_foot_name
    swing_foot_name = ctrl.swing_foot_name
    return_start_time = None
    touchdown = False  # once true, stays true for the rest of return

    # We will force a state override once SINGLE_LEG is reached
    override_state = None
    override_q_ref = None

    for i in range(n_steps):
        t = env.data.time

        # ---- Detect when SINGLE_LEG is reached and trigger return ----
        if ctrl.state == "SINGLE_LEG" and override_state is None:
            override_state = "BIPEDAL_RETURN"
            return_start_time = t
            override_q_ref = env.get_actuated_qpos().copy()

            # CoM target: back to midpoint of both feet
            left_pos = env.get_body_pos("left_foot")
            right_pos = env.get_body_pos("right_foot")
            com_mid = 0.5 * (left_pos + right_pos)
            com_mid[2] = com_start[2]
            com_now = compute_com_position(env.model, env.data)
            com_planner_return = ComPlanner(com_now, com_mid, 2.0)

            # Swing foot target: back to ground
            swing_pos = env.get_body_pos(f"{swing_foot_name}_foot")
            swing_foot_start = swing_pos.copy()
            swing_planner_return = SwingFootPlanner(
                swing_foot_start,
                lift_height=0.0,  # no lift; we want to land
                rise_duration=1.0,
            )

        # ---- Compute control ---------------------------------------
        if override_state is None:
            try:
                tau = ctrl.compute()
            except RuntimeError as exc:
                print(f"controller failed at t={t:.3f}: {exc}")
                break
        else:
            tau, touchdown = _compute_return(
                env, ctrl, override_q_ref, com_planner_return,
                swing_planner_return, support_foot_name, swing_foot_name,
                return_start_time, t, touchdown,
            )

        env.step(tau)

        if not np.all(np.isfinite(env.data.qpos)):
            print(f"NaN detected at t={t:.3f}")
            break

        left_pos = env.get_body_pos("left_foot")
        right_pos = env.get_body_pos("right_foot")

        # Determine slip
        if override_state is None:
            if ctrl.state == "SINGLE_LEG":
                slip = np.linalg.norm(
                    env.get_body_pos(f"{support_foot_name}_foot")[:2]
                    - (left_init if support_foot_name == "left" else right_init)
                )
            else:
                slip = max(
                    np.linalg.norm(left_pos[:2] - left_init),
                    np.linalg.norm(right_pos[:2] - right_init),
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
        log["state"].append(ctrl.state if override_state is None else override_state)
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
        log["left_foot_fz"].append(left_fz)
        log["right_foot_fz"].append(right_fz)
        log["max_tau"].append(float(np.max(np.abs(tau))))
        log["touchdown"].append(touchdown if override_state == "BIPEDAL_RETURN" else False)

        # End simulation if return is complete and stable
        if override_state == "BIPEDAL_RETURN" and t - return_start_time >= 7.0:
            break

    return log


def _compute_return(
    env, ctrl, q_ref, com_planner, swing_planner,
    support_name, swing_name, t_start, t_now, touchdown,
):
    """Manual BIPEDAL_RETURN compute."""
    model = env.model
    data = env.data
    nv = model.nv
    dt_return = t_now - t_start

    q = env.get_actuated_qpos()
    dq = env.get_actuated_qvel()

    mujoco.mj_fullM(model, ctrl._M, data.qM)
    bias_force = data.qfrc_bias - data.qfrc_passive

    J_com = np.zeros((3, nv))
    mujoco.mj_jacSubtreeCom(model, data, J_com, 0)

    support_bid = env._body_ids[f"{support_name}_foot"]
    swing_bid = env._body_ids[f"{swing_name}_foot"]
    J_support = np.zeros((6, nv))
    J_swing = np.zeros((6, nv))
    mujoco.mj_jacBody(model, data, J_support[:3], J_support[3:], support_bid)
    mujoco.mj_jacBody(model, data, J_swing[:3], J_swing[3:], swing_bid)

    com_pos = compute_com_position(model, data)
    com_target, _, _ = com_planner.evaluate(dt_return)

    com_accel_des, cam_rate_des, joint_accel_des, J_cam = ctrl._compute_task_targets(
        model, data, com_pos, com_target, q_ref, q, dq,
    )

    # Detect touchdown: swing foot is near ground and loaded.
    # Hysteresis: once true, stays true for the rest of the phase.
    swing_body_name = env.cfg["robot"]["body_names"][f"{swing_name}_foot"]
    swing_fz = compute_contact_wrench(model, data, swing_body_name)[2]
    swing_pos_cur = env.get_body_pos(f"{swing_name}_foot")
    if not touchdown:
        touchdown = swing_fz > 20.0 and swing_pos_cur[2] < 0.04

    foot_kd = ctrl.cfg["transition"]["foot_kd"]
    # Boost foot damping after touchdown to kill landing oscillation
    foot_kd_effective = foot_kd * 2.0 if touchdown else foot_kd
    support_vel = J_support @ data.qvel
    swing_vel = J_swing @ data.qvel

    active_feet = [
        {
            "jacobian": J_support,
            "name": f"{support_name}_foot",
            "accel_offset": -foot_kd_effective * support_vel,
        },
        {
            "jacobian": J_swing,
            "name": f"{swing_name}_foot",
            "accel_offset": -foot_kd_effective * swing_vel,
        },
    ]

    swing_pos, swing_vel_traj, swing_accel = swing_planner.evaluate(dt_return)
    current_swing_vel = J_swing[:3] @ data.qvel
    swing_kp = ctrl.cfg["transition"]["swing_kp"]
    swing_kd = ctrl.cfg["transition"]["swing_kd"]
    swing_accel_des_z = (
        swing_accel[2]
        + swing_kp * (swing_pos[2] - swing_pos_cur[2])
        + swing_kd * (swing_vel_traj[2] - current_swing_vel[2])
    )
    swing_task = {
        "jacobian": J_swing[2:3],
        "accel_des": np.array([swing_accel_des_z]),
    }

    # Pelvis orientation
    pelvis_quat_des = np.array([1.0, 0.0, 0.0, 0.0])
    pelvis_quat_cur = env.get_pelvis_quat()
    pelvis_ang_err = ctrl._quat_error(pelvis_quat_des, pelvis_quat_cur)
    J_pelvis_lin = np.zeros((3, nv))
    J_pelvis_ang = np.zeros((3, nv))
    mujoco.mj_jacBody(model, data, J_pelvis_lin, J_pelvis_ang, ctrl._pelvis_bid)
    pelvis_omega = J_pelvis_ang @ data.qvel
    pelvis_accel_des = (
        -ctrl.cfg["transition"]["pelvis_kp"] * pelvis_ang_err
        - ctrl.cfg["transition"]["pelvis_kd"] * pelvis_omega
    )
    extra_tasks = [
        (ctrl.cfg["transition"]["pelvis_weight"], J_pelvis_ang, pelvis_accel_des),
    ]

    # Use single-leg weights for the entire return phase; the CoM is still
    # shifting back to midpoint even after the foot touches down.
    old_w_com = ctrl.w_com
    old_w_cam = ctrl.w_cam
    old_w_posture = ctrl.w_posture
    ctrl.w_com = ctrl.cfg["transition"]["single_leg_w_com"]
    ctrl.w_cam = ctrl.cfg["transition"]["single_leg_w_cam"]
    ctrl.w_posture = ctrl.cfg["transition"]["single_leg_w_posture"]

    qacc_des, wrenches, tau = ctrl._solve_qp(
        model=model, data=data, J_com=J_com, J_cam=J_cam,
        com_accel_des=com_accel_des, cam_rate_des=cam_rate_des,
        joint_accel_des=joint_accel_des, active_feet=active_feet,
        bias_force=bias_force, swing_task=swing_task, extra_tasks=extra_tasks,
    )

    ctrl.w_com = old_w_com
    ctrl.w_cam = old_w_cam
    ctrl.w_posture = old_w_posture

    return tau, touchdown


def assess(log: dict) -> tuple[dict, dict]:
    t = np.array(log["t"])
    left_z = np.array(log["left_foot_z"])
    right_z = np.array(log["right_foot_z"])
    roll = np.rad2deg(np.array(log["pelvis_roll"]))
    pitch = np.rad2deg(np.array(log["pelvis_pitch"]))
    slip = np.array(log["support_slip"])
    states = np.array(log["state"])

    return_mask = states == "BIPEDAL_RETURN"
    return_steps = int(return_mask.sum())
    return_duration = return_steps * 0.002

    # At end of return, both feet should be near ground
    final_left_z = left_z[-1] if len(left_z) > 0 else 0.0
    final_right_z = right_z[-1] if len(right_z) > 0 else 0.0

    # ---- New metrics ------------------------------------------------
    com_x = np.array(log["com_x"])
    com_y = np.array(log["com_y"])
    com_z = np.array(log["com_z"])
    left_fz = np.array(log["left_foot_fz"])
    right_fz = np.array(log["right_foot_fz"])
    max_tau = np.array(log["max_tau"])

    left_x = np.array(log["left_foot_x"])
    left_y = np.array(log["left_foot_y"])
    right_x = np.array(log["right_foot_x"])
    right_y = np.array(log["right_foot_y"])

    # CoM final error from midpoint of feet
    final_mid_x = 0.5 * (left_x[-1] + right_x[-1]) if len(left_x) > 0 else 0.0
    final_mid_y = 0.5 * (left_y[-1] + right_y[-1]) if len(left_y) > 0 else 0.0
    com_err_final = np.sqrt((com_x[-1] - final_mid_x)**2 + (com_y[-1] - final_mid_y)**2)

    # Post-return oscillation: max roll/pitch in last 1.0 s of return
    t_arr = np.array(log["t"])
    if return_mask.any():
        last_second_mask = return_mask & (t_arr >= t_arr[-1] - 1.0)
        post_return_max_roll = np.max(np.abs(roll[last_second_mask])) if last_second_mask.any() else 0.0
        post_return_max_pitch = np.max(np.abs(pitch[last_second_mask])) if last_second_mask.any() else 0.0
    else:
        post_return_max_roll = 0.0
        post_return_max_pitch = 0.0

    # Contact forces at end (last 10 samples averaged to reduce noise)
    n_avg = min(10, len(left_fz))
    final_left_fz = float(np.mean(left_fz[-n_avg:])) if n_avg > 0 else 0.0
    final_right_fz = float(np.mean(right_fz[-n_avg:])) if n_avg > 0 else 0.0

    # Max torque during return
    return_max_tau = float(np.max(max_tau[return_mask])) if return_mask.any() else 0.0

    checks = {
        "no_fall": np.max(np.abs(roll)) < 15.0 and np.max(np.abs(pitch)) < 15.0,
        "return_phase_exists": return_mask.any(),
        "return_stable": return_duration >= 3.0,
        "both_feet_on_ground": final_left_z < 0.06 and final_right_z < 0.06,
        "support_slip": np.max(slip) < 0.05,
        "com_near_midpoint": com_err_final < 0.05,
        "post_return_settled": post_return_max_roll < 5.0 and post_return_max_pitch < 5.0,
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
        "post_return_roll_deg": float(post_return_max_roll),
        "post_return_pitch_deg": float(post_return_max_pitch),
        "final_left_fz_N": float(final_left_fz),
        "final_right_fz_N": float(final_right_fz),
        "return_max_tau_Nm": float(return_max_tau),
    }

    return metrics, checks


def plot(log: dict, metrics: dict, checks: dict) -> None:
    import matplotlib.pyplot as plt

    OUTPUT_PLOT = "outputs/test_single_to_double.png"
    os.makedirs(os.path.dirname(OUTPUT_PLOT), exist_ok=True)

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
    com_z = np.array(log["com_z"])
    left_fz = np.array(log["left_foot_fz"])
    right_fz = np.array(log["right_foot_fz"])
    max_tau = np.array(log["max_tau"])
    states = np.array(log["state"])
    touchdown = np.array(log["touchdown"])

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

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # 1. Phase timeline
    ax = axes[0, 0]
    state_map = {"INIT": 0, "BIPEDAL": 0, "WEIGHT_SHIFT": 1, "SINGLE_LEG": 2, "BIPEDAL_RETURN": 3}
    state_vals = [state_map.get(s, -1) for s in states]
    ax.plot(t, state_vals, drawstyle="steps-post", color="tab:blue")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_yticklabels(["BIPEDAL", "WEIGHT_SHIFT", "SINGLE_LEG", "BIPEDAL_RETURN"])
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Phase")
    ax.set_title("Gait phase timeline")
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 2. Foot heights + contact forces
    ax = axes[0, 1]
    ax.plot(t, left_z, color="C0", label="left z")
    ax.plot(t, right_z, color="C1", label="right z")
    ax.axhline(0.04, color="gray", ls="--", alpha=0.3, label="td thresh")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)", color="C0")
    ax.tick_params(axis="y", labelcolor="C0")
    ax.set_title("Foot heights & contact forces")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, left_fz, color="C0", ls="--", alpha=0.5, label="left fz")
    ax2.plot(t, right_fz, color="C1", ls="--", alpha=0.5, label="right fz")
    ax2.set_ylabel("Force (N)", color="gray")
    ax2.tick_params(axis="y", labelcolor="gray")
    l1, lb1 = ax.get_legend_handles_labels()
    l2, lb2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=7, loc="upper right")
    _mark(ax)

    # 3. CoM error from foot midpoint
    ax = axes[1, 0]
    mid_x = 0.5 * (left_x + right_x)
    mid_y = 0.5 * (left_y + right_y)
    ax.plot(t, com_x - mid_x, label="com_x err", color="C0")
    ax.plot(t, com_y - mid_y, label="com_y err", color="C1")
    ax.axhline(0, color="k", ls="-", alpha=0.2)
    ax.axhline(0.05, color="r", ls="--", alpha=0.3, label="thresh")
    ax.axhline(-0.05, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CoM error (m)")
    ax.set_title("CoM error from foot midpoint")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 4. Pelvis orientation with thresholds
    ax = axes[1, 1]
    ax.plot(t, roll, label="roll", color="C0")
    ax.plot(t, pitch, label="pitch", color="C1")
    ax.axhline(5, color="orange", ls="--", alpha=0.3, label="settle")
    ax.axhline(-5, color="orange", ls="--", alpha=0.3)
    ax.axhline(15, color="r", ls="--", alpha=0.3, label="fall")
    ax.axhline(-15, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis orientation")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    # 5. Support slip + max torque
    ax = axes[2, 0]
    ax.plot(t, slip, color="C0", label="slip")
    ax.axhline(0.05, color="r", ls="--", alpha=0.3, label="slip thresh")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Slip (m)", color="C0")
    ax.tick_params(axis="y", labelcolor="C0")
    ax.set_title("Support slip & max torque")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    ax2.plot(t, max_tau, color="C1", label="max tau")
    ax2.axhline(150, color="r", ls="--", alpha=0.3)
    ax2.set_ylabel("Torque (Nm)", color="C1")
    ax2.tick_params(axis="y", labelcolor="C1")
    l1, lb1 = ax.get_legend_handles_labels()
    l2, lb2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, lb1 + lb2, fontsize=7, loc="upper right")
    _mark(ax)

    # 6. Foot horizontal positions
    ax = axes[2, 1]
    ax.plot(t, left_x, color="C0", label="left x")
    ax.plot(t, left_y, color="C0", ls="--", alpha=0.7, label="left y")
    ax.plot(t, right_x, color="C1", label="right x")
    ax.plot(t, right_y, color="C1", ls="--", alpha=0.7, label="right y")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Foot horizontal positions")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    _mark(ax)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    print(f"figure saved to {OUTPUT_PLOT}")


def main() -> None:
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    env = G1Env(CONFIG_PATH)
    env.reset()
    ctrl = TransitionController(env, cfg)
    ctrl.reset()

    # Give enough time for the normal transition to reach SINGLE_LEG
    # then let the manual return phase run.
    duration = 40.0

    print("=" * 60)
    print("Step test: Single-leg -> Bipedal return")
    print("=" * 60)

    log = run(env, ctrl, duration)
    metrics, checks = assess(log)
    plot(log, metrics, checks)

    print("\n=== Summary ===")
    all_pass = all(checks.values())
    print(f"Overall: {'PASS' if all_pass else 'FAIL'}")
    for name, passed in checks.items():
        print(f"  {name:25s} : {'PASS' if passed else 'FAIL'}")


if __name__ == "__main__":
    main()
