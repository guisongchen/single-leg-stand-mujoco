import time
import sys
import os

import numpy as np
import yaml
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.bipedal_stance_controller import BipedalStanceController
from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    compute_contact_wrench,
    euler_from_quat,
)


# ------------------------------------------------------------------ #
# 1. Simulation loop
# ------------------------------------------------------------------ #

def run_simulation(env: G1Env, controller: BipedalStanceController,
                   duration: float = 10.0) -> dict:
    n_steps = int(duration / env.dt)
    logs = _make_log_dict()

    for step in range(n_steps):
        t0 = time.perf_counter()

        try:
            ctrl = controller.compute()
        except RuntimeError as e:
            print(f"\nController failed at step {step}: {e}")
            logs["failed_step"] = step
            break

        env.step(ctrl)

        if not np.all(np.isfinite(env.data.qpos)):
            logs["nan_step"] = step
            break

        compute_time = time.perf_counter() - t0
        _append_sample(env, logs, compute_time)

    return logs


def _make_log_dict() -> dict:
    return {
        "com_pos": [],
        "com_vel": [],
        "left_foot_pos": [],
        "right_foot_pos": [],
        "left_foot_fz": [],
        "right_foot_fz": [],
        "pelvis_roll": [],
        "pelvis_pitch": [],
        "solve_times": [],
        "nan_step": None,
        "failed_step": None,
    }


def _append_sample(env: G1Env, logs: dict, compute_time: float) -> None:
    logs["com_pos"].append(compute_com_position(env.model, env.data))
    logs["com_vel"].append(compute_com_velocity(env.model, env.data))
    logs["left_foot_pos"].append(env.get_body_pos("left_foot"))
    logs["right_foot_pos"].append(env.get_body_pos("right_foot"))
    logs["left_foot_fz"].append(
        compute_contact_wrench(env.model, env.data,
                               env.cfg["robot"]["body_names"]["left_foot"])
    )
    logs["right_foot_fz"].append(
        compute_contact_wrench(env.model, env.data,
                               env.cfg["robot"]["body_names"]["right_foot"])
    )
    quat = env.get_pelvis_quat()
    roll, pitch, _ = euler_from_quat(*quat)
    logs["pelvis_roll"].append(roll)
    logs["pelvis_pitch"].append(pitch)
    logs["solve_times"].append(compute_time)


# ------------------------------------------------------------------ #
# 2. Metrics
# ------------------------------------------------------------------ #

def compute_metrics(env: G1Env, controller: BipedalStanceController,
                    logs: dict) -> dict:
    com = np.array(logs["com_pos"])
    lf = np.array(logs["left_foot_pos"])
    rf = np.array(logs["right_foot_pos"])
    lfz = np.array(logs["left_foot_fz"])
    rfz = np.array(logs["right_foot_fz"])
    solves = np.array(logs["solve_times"])

    com_target_xy = controller.com_target[:2]

    return {
        "n_steps": len(solves),
        "nan_detected": logs["nan_step"] is not None,
        "failed_step": logs.get("failed_step"),
        "com_rmse": float(np.sqrt(np.mean(np.sum((com[:, :2] - com_target_xy) ** 2, axis=1)))),
        "com_mean_z": float(com[:, 2].mean()),
        "left_drift": float(np.sqrt(np.mean(np.sum((lf - lf[0]) ** 2, axis=1)))),
        "right_drift": float(np.sqrt(np.mean(np.sum((rf - rf[0]) ** 2, axis=1)))),
        "max_roll_deg": float(np.rad2deg(np.max(np.abs(logs["pelvis_roll"])))),
        "max_pitch_deg": float(np.rad2deg(np.max(np.abs(logs["pelvis_pitch"])))),
        "min_left_fz": float(lfz[:, 2].min()),
        "min_right_fz": float(rfz[:, 2].min()),
        "solve_mean_us": float(solves.mean() * 1e6),
        "solve_p99_us": float(np.percentile(solves, 99) * 1e6),
        "budget_us": env.dt * 1e6,
    }


# ------------------------------------------------------------------ #
# 3. Pass / Fail assessment
# ------------------------------------------------------------------ #

def assess(m: dict) -> dict:
    return {
        "Simulation healthy (no NaN)": not m["nan_detected"],
        "CoM RMSE < 0.02 m":             m["com_rmse"] < 0.02,
        "Left foot drift < 0.005 m":     m["left_drift"] < 0.005,
        "Right foot drift < 0.005 m":    m["right_drift"] < 0.005,
        "Pelvis roll < 5 deg":           m["max_roll_deg"] < 5.0,
        "Pelvis pitch < 5 deg":          m["max_pitch_deg"] < 5.0,
        "Left foot fz > 10 N":           m["min_left_fz"] > 10.0,
        "Right foot fz > 10 N":          m["min_right_fz"] > 10.0,
        "Solve time < budget":           m["solve_p99_us"] < m["budget_us"],
    }


# ------------------------------------------------------------------ #
# 4. Reporting
# ------------------------------------------------------------------ #

def report(m: dict, checks: dict) -> None:
    print("Results:")
    print("-" * 60)
    print(f"Simulated steps:        {m['n_steps']}")
    print(f"NaN detected:           {m['nan_detected']}")
    print(f"Controller failed at:   {m['failed_step']}")
    print(f"CoM RMSE (horizontal):  {m['com_rmse']:.4f} m")
    print(f"CoM height (mean):      {m['com_mean_z']:.4f} m")
    print(f"Left foot drift RMSE:   {m['left_drift']:.4f} m")
    print(f"Right foot drift RMSE:  {m['right_drift']:.4f} m")
    print(f"Max pelvis roll:        {m['max_roll_deg']:.2f} deg")
    print(f"Max pelvis pitch:       {m['max_pitch_deg']:.2f} deg")
    print(f"Min left foot fz:       {m['min_left_fz']:.1f} N")
    print(f"Min right foot fz:      {m['min_right_fz']:.1f} N")
    print(f"Solve time (mean):      {m['solve_mean_us']:.2f} us")
    print(f"Solve time (p99):       {m['solve_p99_us']:.2f} us")
    print(f"Budget (dt=2.0 ms):     {m['budget_us']:.2f} us")
    print("-" * 60)

    passed = 0
    for name, ok in checks.items():
        status = "PASS" if ok else "FAIL"
        if ok:
            passed += 1
        print(f"  [{status}] {name}")

    print("-" * 60)
    print(f"Summary: {passed}/{len(checks)} checks passed")
    if passed == len(checks):
        print("Phase 1 exit criteria SATISFIED.")
    else:
        print("Phase 1 exit criteria NOT satisfied.")


# ------------------------------------------------------------------ #
# 5. Persistence
# ------------------------------------------------------------------ #

def save_logs(logs: dict, path: str = "logs/bipedal_stance_test.npz") -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(
        path,
        com_pos=logs["com_pos"],
        com_vel=logs["com_vel"],
        left_foot_pos=logs["left_foot_pos"],
        right_foot_pos=logs["right_foot_pos"],
        left_foot_force=logs["left_foot_fz"],
        right_foot_force=logs["right_foot_fz"],
        pelvis_roll=logs["pelvis_roll"],
        pelvis_pitch=logs["pelvis_pitch"],
        solve_times=logs["solve_times"],
    )
    print(f"Log saved to {path}")


# ------------------------------------------------------------------ #
# 6. Plotting
# ------------------------------------------------------------------ #

def plot_results(logs: dict, metrics: dict, out_dir: str = "outputs") -> None:
    """Generate diagnostic figures from test logs."""
    os.makedirs(out_dir, exist_ok=True)
    dt = 0.002
    t = np.arange(len(logs["solve_times"])) * dt

    com = np.array(logs["com_pos"])
    lf = np.array(logs["left_foot_pos"])
    rf = np.array(logs["right_foot_pos"])
    lfz = np.array(logs["left_foot_fz"])
    rfz = np.array(logs["right_foot_fz"])
    roll = np.rad2deg(np.array(logs["pelvis_roll"]))
    pitch = np.rad2deg(np.array(logs["pelvis_pitch"]))
    solves = np.array(logs["solve_times"]) * 1e6

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # 1. CoM trajectory
    ax = axes[0, 0]
    ax.plot(t, com[:, 0], label="x")
    ax.plot(t, com[:, 1], label="y")
    ax.plot(t, com[:, 2], label="z")
    ax.axhline(metrics.get("com_mean_z", 0), color="k", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CoM position (m)")
    ax.set_title("CoM Trajectory")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Foot drift
    ax = axes[0, 1]
    ax.plot(t, np.linalg.norm(lf - lf[0], axis=1), label="left")
    ax.plot(t, np.linalg.norm(rf - rf[0], axis=1), label="right")
    ax.axhline(0.005, color="r", ls="--", alpha=0.3, label="limit=5 mm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Foot drift (m)")
    ax.set_title("Foot Position Drift")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Contact forces (vertical)
    ax = axes[1, 0]
    ax.plot(t, lfz[:, 2], label="left fz")
    ax.plot(t, rfz[:, 2], label="right fz")
    ax.axhline(10, color="r", ls="--", alpha=0.3, label="limit=10 N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Vertical force (N)")
    ax.set_title("Contact Forces (MuJoCo)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Pelvis orientation
    ax = axes[1, 1]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.axhline(5, color="r", ls="--", alpha=0.3)
    ax.axhline(-5, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis Orientation")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. Solve time
    ax = axes[2, 0]
    ax.plot(t, solves)
    ax.axhline(2000, color="r", ls="--", alpha=0.3, label="budget=2000 us")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Solve time (us)")
    ax.set_title("OSQP Solve Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. CoM horizontal error
    ax = axes[2, 1]
    com_err = np.linalg.norm(com[:, :2] - com[0, :2], axis=1)
    ax.plot(t, com_err * 1000)  # mm
    ax.axhline(20, color="r", ls="--", alpha=0.3, label="limit=20 mm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (mm)")
    ax.set_title("CoM Horizontal Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = os.path.join(out_dir, "bipedal_stance_diagnostics.png")
    plt.savefig(path, dpi=150)
    print(f"Figure saved to {path}")


# ------------------------------------------------------------------ #
# 7. Orchestration
# ------------------------------------------------------------------ #

CONFIG_PATH = "configs/g1_config.yaml"


def main():
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = BipedalStanceController(env, cfg)
    controller.reset()

    print("=" * 60)
    print("Phase 1 Test: Bipedal Stance Controller")
    print("=" * 60)
    print(f"Sim dt: {env.dt:.6f} s | Duration: 10.0 s | Steps: {int(10.0 / env.dt)}")
    print(f"CoM target: {controller.com_target}")
    print()

    logs = run_simulation(env, controller, duration=10.0)
    metrics = compute_metrics(env, controller, logs)
    checks = assess(metrics)
    report(metrics, checks)
    save_logs(logs)
    plot_results(logs, metrics)


if __name__ == "__main__":
    main()
