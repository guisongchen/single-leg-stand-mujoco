import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import yaml
import matplotlib.pyplot as plt

from env.g1_env import G1Env
from utils.kinematics import compute_com_position


def evaluate_pose(env: G1Env, knee: float, ankle_pitch: float, hip_pitch: float = 0.0):
    """Set joints, run mj_forward, return CoM and foot positions."""
    qpos = env.get_qpos().copy()
    # Map joint names to qpos addresses
    names = env.cfg["robot"]["joint_names"]
    for side in ("left", "right"):
        knee_name = f"{side}_knee_joint"
        ankle_name = f"{side}_ankle_pitch_joint"
        hip_name = f"{side}_hip_pitch_joint"
        for n, adr in zip(names, env._qposadr):
            if n == knee_name:
                qpos[adr] = knee
            elif n == ankle_name:
                qpos[adr] = ankle_pitch
            elif n == hip_name:
                qpos[adr] = hip_pitch
    env.set_state(qpos, np.zeros(env.nv))

    com = compute_com_position(env.model, env.data)
    left_foot = env.get_body_pos("left_foot")
    right_foot = env.get_body_pos("right_foot")
    return com, left_foot, right_foot


def sweep_2d(env: G1Env, hip_pitch: float = 0.0):
    """Sweep knee and ankle for a fixed hip pitch."""
    knee_vals = np.linspace(0.0, 0.6, 25)
    ankle_vals = np.linspace(-0.3, 0.1, 25)

    com_x_rel = np.zeros((len(ankle_vals), len(knee_vals)))
    com_z = np.zeros((len(ankle_vals), len(knee_vals)))

    for i, ankle in enumerate(ankle_vals):
        for j, knee in enumerate(knee_vals):
            com, lf, rf = evaluate_pose(env, knee, ankle, hip_pitch)
            foot_mid_x = (lf[0] + rf[0]) / 2.0
            com_x_rel[i, j] = com[0] - foot_mid_x
            com_z[i, j] = com[2]

    return knee_vals, ankle_vals, com_x_rel, com_z


def sweep_poses():
    cfg_path = "configs/g1_config.yaml"
    env = G1Env(cfg_path)
    env.reset()

    # Try multiple hip pitches
    hip_vals = [0.0, -0.1, -0.2, -0.3]
    fig, axes = plt.subplots(len(hip_vals), 2, figsize=(12, 4 * len(hip_vals)))
    if len(hip_vals) == 1:
        axes = axes.reshape(1, -1)

    found_poses = []

    for row, hip in enumerate(hip_vals):
        knee_vals, ankle_vals, com_x_rel, com_z = sweep_2d(env, hip)

        im0 = axes[row, 0].imshow(
            com_x_rel * 100,  # cm
            origin="lower",
            aspect="auto",
            extent=[np.rad2deg(knee_vals[0]), np.rad2deg(knee_vals[-1]),
                    np.rad2deg(ankle_vals[0]), np.rad2deg(ankle_vals[-1])],
            cmap="RdBu_r",
            vmin=-10, vmax=10,
        )
        axes[row, 0].set_xlabel("Knee angle (deg)")
        axes[row, 0].set_ylabel("Ankle pitch (deg)")
        axes[row, 0].set_title(f"CoM x offset (cm) | hip={hip:.2f} rad ({np.rad2deg(hip):.1f}°)")
        fig.colorbar(im0, ax=axes[row, 0])
        cs = axes[row, 0].contour(
            np.rad2deg(knee_vals), np.rad2deg(ankle_vals), com_x_rel * 100,
            levels=[0], colors="black", linewidths=2
        )
        axes[row, 0].clabel(cs, inline=True, fmt="zero")

        im1 = axes[row, 1].imshow(
            com_z,
            origin="lower",
            aspect="auto",
            extent=[np.rad2deg(knee_vals[0]), np.rad2deg(knee_vals[-1]),
                    np.rad2deg(ankle_vals[0]), np.rad2deg(ankle_vals[-1])],
            cmap="viridis",
        )
        axes[row, 1].set_xlabel("Knee angle (deg)")
        axes[row, 1].set_ylabel("Ankle pitch (deg)")
        axes[row, 1].set_title(f"CoM height (m) | hip={hip:.2f} rad")
        fig.colorbar(im1, ax=axes[row, 1])

        # Collect near-zero poses
        for i, ankle in enumerate(ankle_vals):
            for j, knee in enumerate(knee_vals):
                if abs(com_x_rel[i, j]) < 0.01 and com_z[i, j] > 0.65:
                    found_poses.append({
                        "hip": hip, "knee": knee, "ankle": ankle,
                        "com_z": com_z[i, j],
                        "offset_cm": com_x_rel[i, j] * 100,
                    })

    plt.tight_layout()
    out_path = "outputs/initial_pose_sweep.png"
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    plt.savefig(out_path, dpi=150)
    print(f"Saved pose sweep to {out_path}")

    print("\n--- Poses with CoM near foot midpoint (|offset| < 1 cm, z > 0.65 m) ---")
    if not found_poses:
        print("  No poses found in swept range.")
    else:
        for p in found_poses:
            print(f"  hip={p['hip']:.3f} ({np.rad2deg(p['hip']):.1f}°), "
                  f"knee={p['knee']:.3f} ({np.rad2deg(p['knee']):.1f}°), "
                  f"ankle={p['ankle']:.3f} ({np.rad2deg(p['ankle']):.1f}°), "
                  f"com_z={p['com_z']:.3f} m, offset={p['offset_cm']:.2f} cm")

    # Current pose
    com_cur, lf_cur, rf_cur = evaluate_pose(env, 0.15, -0.08, 0.0)
    print(f"\n--- Current pose (hip=0.0, knee=0.15, ankle=-0.08) ---")
    print(f"  CoM = [{com_cur[0]:.4f}, {com_cur[1]:.4f}, {com_cur[2]:.4f}]")
    print(f"  left_foot  = [{lf_cur[0]:.4f}, {lf_cur[1]:.4f}, {lf_cur[2]:.4f}]")
    print(f"  right_foot = [{rf_cur[0]:.4f}, {rf_cur[1]:.4f}, {rf_cur[2]:.4f}]")
    print(f"  CoM x offset from foot midpoint = {(com_cur[0] - (lf_cur[0]+rf_cur[0])/2)*100:.2f} cm")


if __name__ == "__main__":
    sweep_poses()
