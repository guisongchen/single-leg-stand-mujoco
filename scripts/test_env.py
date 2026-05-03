import time
import sys
import os

import numpy as np
import yaml
import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    compute_contact_wrench,
    euler_from_quat,
)


def main():
    env = G1Env("configs/g1_config.yaml")
    with open("configs/g1_config.yaml", "r") as f:
        cfg = yaml.safe_load(f)

    env.reset()

    # Simple PD controller to hold initial posture
    kp = cfg["control"]["kp"]
    kd = cfg["control"]["kd"]
    q_target = env.get_actuated_qpos()

    duration = 5.0  # seconds
    n_steps = int(duration / env.dt)

    # Logging buffers
    com_pos_log = []
    com_vel_log = []
    left_foot_pos_log = []
    right_foot_pos_log = []
    left_foot_force_log = []
    right_foot_force_log = []
    pelvis_roll_log = []
    pelvis_pitch_log = []
    solve_times = []

    print("=" * 60)
    print("Phase 0 Test: Environment + Basic PD Posture Hold")
    print("=" * 60)
    print(f"Sim dt: {env.dt:.6f} s | Duration: {duration:.1f} s | Steps: {n_steps}")
    print()

    # Measure initial foot-ground contact quality (before simulation perturbs state)
    min_geom_z = float("inf")
    for foot_name in ("left_foot", "right_foot"):
        bid = env._body_ids[foot_name]
        for gid in range(env.model.ngeom):
            if env.model.geom_bodyid[gid] == bid:
                gtype = env.model.geom_type[gid]
                pos_local = np.array(env.model.geom_pos[gid])
                if gtype == mujoco.mjtGeom.mjGEOM_SPHERE.value:
                    pos_local[2] -= env.model.geom_size[gid][0]
                elif gtype == mujoco.mjtGeom.mjGEOM_MESH.value:
                    mesh_id = env.model.geom_dataid[gid]
                    verts = env.model.mesh_vert[
                        env.model.mesh_vertadr[mesh_id] : env.model.mesh_vertadr[mesh_id] + env.model.mesh_vertnum[mesh_id]
                    ]
                    scale = env.model.geom_size[gid]
                    pos_local[2] += (verts * scale)[:, 2].min()
                elif gtype == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value:
                    pos_local[2] -= env.model.geom_size[gid][2]
                elif gtype == mujoco.mjtGeom.mjGEOM_BOX.value:
                    pos_local[2] -= env.model.geom_size[gid][2]
                else:
                    pos_local[2] -= env.model.geom_size[gid][0]
                xpos = env.data.xpos[bid]
                xmat = env.data.xmat[bid].reshape(3, 3)
                pos_world = xpos + xmat @ pos_local
                min_geom_z = min(min_geom_z, pos_world[2])

    nan_detected = False
    for step in range(n_steps):
        t0 = time.perf_counter()

        q = env.get_actuated_qpos()
        dq = env.get_actuated_qvel()
        ctrl = kp * (q_target - q) - kd * dq

        env.step(ctrl)

        # Check simulation health
        if not np.all(np.isfinite(env.data.qpos)):
            nan_detected = True
            print(f"WARNING: NaN/Inf detected at step {step}, t={step*env.dt:.3f}s")
            break

        t1 = time.perf_counter()
        solve_times.append(t1 - t0)

        # Logging
        com_pos_log.append(compute_com_position(env.model, env.data))
        com_vel_log.append(compute_com_velocity(env.model, env.data))
        left_foot_pos_log.append(env.get_body_pos("left_foot"))
        right_foot_pos_log.append(env.get_body_pos("right_foot"))
        left_foot_force_log.append(compute_contact_wrench(env.model, env.data, env.cfg["robot"]["body_names"]["left_foot"]))
        right_foot_force_log.append(compute_contact_wrench(env.model, env.data, env.cfg["robot"]["body_names"]["right_foot"]))
        quat = env.get_pelvis_quat()
        roll, pitch, yaw = euler_from_quat(quat[0], quat[1], quat[2], quat[3])
        pelvis_roll_log.append(roll)
        pelvis_pitch_log.append(pitch)

    # Convert to arrays
    com_pos_log = np.array(com_pos_log)
    com_vel_log = np.array(com_vel_log)
    left_foot_pos_log = np.array(left_foot_pos_log)
    right_foot_pos_log = np.array(right_foot_pos_log)
    left_foot_force_log = np.array(left_foot_force_log)
    right_foot_force_log = np.array(right_foot_force_log)
    solve_times = np.array(solve_times)

    # ---- Metrics ----
    com_mean_z = com_pos_log[:, 2].mean() if len(com_pos_log) > 0 else 0.0
    solve_p99 = np.percentile(solve_times, 99) if len(solve_times) > 0 else 0.0

    initial_left_z = left_foot_pos_log[0, 2] if len(left_foot_pos_log) > 0 else -1
    initial_right_z = right_foot_pos_log[0, 2] if len(right_foot_pos_log) > 0 else -1

    # ---- Print Results ----
    print("Results:")
    print("-" * 60)
    print(f"Simulated steps:        {len(solve_times)}/{n_steps}")
    print(f"NaN detected:           {nan_detected}")
    print(f"Min foot geom z:        {min_geom_z:.4f} m")
    print(f"Initial left foot z:    {initial_left_z:.4f} m")
    print(f"Initial right foot z:   {initial_right_z:.4f} m")
    print(f"CoM height (mean):      {com_mean_z:.4f} m")
    print(f"Initial contact left:   {left_foot_force_log[0, 2]:.1f} N" if len(left_foot_force_log) > 0 else "N/A")
    print(f"Initial contact right:  {right_foot_force_log[0, 2]:.1f} N" if len(right_foot_force_log) > 0 else "N/A")
    print(f"Solve time (mean):      {solve_times.mean()*1e6:.2f} us")
    print(f"Solve time (p99):       {solve_p99*1e6:.2f} us")
    print(f"Budget (dt={env.dt*1000:.1f} ms):     {env.dt*1e6:.2f} us")
    print("-" * 60)

    # ---- Pass / Fail ----
    checks = {
        "Model loaded": True,
        "Simulation healthy (no NaN)": not nan_detected,
        "Initial foot contact near ground": abs(min_geom_z) < 0.005,
        "CoM height reasonable (0.6-1.2m)": 0.6 <= com_mean_z <= 1.2,
        "Solve time < budget": solve_p99 < env.dt,
    }

    passed = 0
    for name, ok in checks.items():
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        print(f"  [{status}] {name}")

    print("-" * 60)
    print(f"Summary: {passed}/{len(checks)} checks passed")
    if passed == len(checks):
        print("Phase 0 exit criteria SATISFIED.")
    else:
        print("Phase 0 exit criteria NOT satisfied — review failures above.")

    # Save log for inspection
    np.savez(
        "logs/phase0_test.npz",
        com_pos=com_pos_log,
        com_vel=com_vel_log,
        left_foot_pos=left_foot_pos_log,
        right_foot_pos=right_foot_pos_log,
        left_foot_force=left_foot_force_log,
        right_foot_force=right_foot_force_log,
        pelvis_roll=np.array(pelvis_roll_log),
        pelvis_pitch=np.array(pelvis_pitch_log),
        solve_times=solve_times,
    )
    print("Log saved to logs/phase0_test.npz")


if __name__ == "__main__":
    main()
