import time
import sys
import os

import numpy as np
import yaml
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from env.g1_env import G1Env
from controllers.transition_controller import TransitionController
from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    compute_contact_wrench,
    euler_from_quat,
)


# ------------------------------------------------------------------ #
# 1. Simulation loop
# ------------------------------------------------------------------ #

def run_simulation(env: G1Env, controller: TransitionController,
                   duration: float = 6.0) -> dict:
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
        _append_sample(env, controller, logs, compute_time)

    return logs


def _make_log_dict() -> dict:
    return {
        "com_pos": [],
        "com_vel": [],
        "support_foot_pos": [],
        "swing_foot_pos": [],
        "support_foot_fz": [],
        "pelvis_roll": [],
        "pelvis_pitch": [],
        "pelvis_z": [],
        "state": [],
        "solve_times": [],
        "nan_step": None,
        "failed_step": None,
    }


def _append_sample(env: G1Env, controller: TransitionController,
                   logs: dict, compute_time: float) -> None:
    logs["com_pos"].append(compute_com_position(env.model, env.data))
    logs["com_vel"].append(compute_com_velocity(env.model, env.data))
    logs["support_foot_pos"].append(
        env.get_body_pos(f"{controller.support_foot_name}_foot")
    )
    logs["swing_foot_pos"].append(
        env.get_body_pos(f"{controller.swing_foot_name}_foot")
    )
    logs["support_foot_fz"].append(
        compute_contact_wrench(
            env.model, env.data,
            env.cfg["robot"]["body_names"][f"{controller.support_foot_name}_foot"]
        )
    )
    quat = env.get_pelvis_quat()
    roll, pitch, _ = euler_from_quat(*quat)
    logs["pelvis_roll"].append(roll)
    logs["pelvis_pitch"].append(pitch)
    logs["pelvis_z"].append(env.get_pelvis_pos()[2])
    logs["state"].append(controller.state)
    logs["solve_times"].append(compute_time)


# ------------------------------------------------------------------ #
# 2. Metrics
# ------------------------------------------------------------------ #

def compute_metrics(env: G1Env, controller: TransitionController,
                    logs: dict) -> dict:
    com = np.array(logs["com_pos"])
    support = np.array(logs["support_foot_pos"])
    swing = np.array(logs["swing_foot_pos"])
    sfz = np.array(logs["support_foot_fz"])
    solves = np.array(logs["solve_times"])
    states = np.array(logs["state"])
    pelvis_z = np.array(logs["pelvis_z"])

    single_mask = states == "SINGLE_LEG"
    single_steps = int(single_mask.sum())
    single_leg_hold_time = single_steps * env.dt

    com_final = com[-1] if len(com) > 0 else np.zeros(3)
    support_final = support[-1] if len(support) > 0 else np.zeros(3)
    com_err_final = np.linalg.norm(com_final[:2] - support_final[:2])

    swing_height_single = swing[single_mask][:, 2] if single_steps > 0 else np.array([])

    return {
        "n_steps": len(solves),
        "nan_detected": logs["nan_step"] is not None,
        "failed_step": logs.get("failed_step"),
        "min_base_height": float(pelvis_z.min()),
        "max_roll_deg": float(np.rad2deg(np.max(np.abs(logs["pelvis_roll"])))),
        "max_pitch_deg": float(np.rad2deg(np.max(np.abs(logs["pelvis_pitch"])))),
        "support_drift": float(np.sqrt(np.mean(np.sum((support - support[0]) ** 2, axis=1)))),
        "max_swing_height": float(swing[:, 2].max()),
        "min_swing_height_single": float(swing_height_single.min()) if len(swing_height_single) > 0 else 0.0,
        "single_leg_hold_time": single_leg_hold_time,
        "com_final_error_from_support": float(com_err_final),
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
        "Base height > 0.5 m": m["min_base_height"] > 0.5,
        "Pelvis roll < 15 deg": m["max_roll_deg"] < 15.0,
        "Pelvis pitch < 15 deg": m["max_pitch_deg"] < 15.0,
        "Support foot drift < 0.005 m": m["support_drift"] < 0.005,
        "Swing foot > 0.02 m during single leg": m["min_swing_height_single"] > 0.02,
        "Single-leg hold >= 3.0 s": m["single_leg_hold_time"] >= 3.0,
        "CoM near support foot < 0.05 m": m["com_final_error_from_support"] < 0.05,
        "Solve time < budget": m["solve_p99_us"] < m["budget_us"],
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
    print(f"Min base height:        {m['min_base_height']:.4f} m")
    print(f"Max pelvis roll:        {m['max_roll_deg']:.2f} deg")
    print(f"Max pelvis pitch:       {m['max_pitch_deg']:.2f} deg")
    print(f"Support foot drift:     {m['support_drift']:.4f} m")
    print(f"Max swing foot height:  {m['max_swing_height']:.4f} m")
    print(f"Min swing height (single): {m['min_swing_height_single']:.4f} m")
    print(f"Single-leg hold time:   {m['single_leg_hold_time']:.2f} s")
    print(f"CoM final error:        {m['com_final_error_from_support']:.4f} m")
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
        print("Phase 2 exit criteria SATISFIED.")
    else:
        print("Phase 2 exit criteria NOT satisfied.")


# ------------------------------------------------------------------ #
# 5. Persistence
# ------------------------------------------------------------------ #

def save_logs(logs: dict, path: str = "logs/transition_test.npz") -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(
        path,
        com_pos=logs["com_pos"],
        com_vel=logs["com_vel"],
        support_foot_pos=logs["support_foot_pos"],
        swing_foot_pos=logs["swing_foot_pos"],
        support_foot_force=logs["support_foot_fz"],
        pelvis_roll=logs["pelvis_roll"],
        pelvis_pitch=logs["pelvis_pitch"],
        pelvis_z=logs["pelvis_z"],
        state=logs["state"],
        solve_times=logs["solve_times"],
    )
    print(f"Log saved to {path}")


# ------------------------------------------------------------------ #
# 6. Plotting
# ------------------------------------------------------------------ #

def plot_results(
    logs: dict,
    metrics: dict,
    com_target_single: np.ndarray | None = None,
    out_dir: str = "outputs",
) -> None:
    """Generate diagnostic figures from transition test logs."""
    os.makedirs(out_dir, exist_ok=True)
    dt = 0.002
    t = np.arange(len(logs["solve_times"])) * dt

    com = np.array(logs["com_pos"])
    support = np.array(logs["support_foot_pos"])
    swing = np.array(logs["swing_foot_pos"])
    sfz = np.array(logs["support_foot_fz"])
    roll = np.rad2deg(np.array(logs["pelvis_roll"]))
    pitch = np.rad2deg(np.array(logs["pelvis_pitch"]))
    pelvis_z = np.array(logs["pelvis_z"])
    states = np.array(logs["state"])
    solves = np.array(logs["solve_times"]) * 1e6

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # 1. CoM horizontal trajectory vs support foot
    ax = axes[0, 0]
    ax.plot(t, com[:, 0], label="CoM x")
    ax.plot(t, com[:, 1], label="CoM y")
    ax.plot(t, support[:, 0], ls="--", label="support x")
    ax.plot(t, support[:, 1], ls="--", label="support y")
    if com_target_single is not None:
        ax.axhline(com_target_single[0], color="k", ls=":", alpha=0.3)
        ax.axhline(com_target_single[1], color="k", ls=":", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("CoM vs Support Foot (horizontal)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. Swing foot height
    ax = axes[0, 1]
    ax.plot(t, swing[:, 2], label="swing z")
    ax.axhline(0.02, color="r", ls="--", alpha=0.3, label="limit=2 cm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)")
    ax.set_title("Swing Foot Height")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. Support foot vertical force
    ax = axes[1, 0]
    ax.plot(t, sfz[:, 2], label="support fz")
    ax.axhline(10, color="r", ls="--", alpha=0.3, label="limit=10 N")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Vertical force (N)")
    ax.set_title("Support Foot Contact Force")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. Pelvis orientation
    ax = axes[1, 1]
    ax.plot(t, roll, label="roll")
    ax.plot(t, pitch, label="pitch")
    ax.axhline(15, color="r", ls="--", alpha=0.3)
    ax.axhline(-15, color="r", ls="--", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Pelvis Orientation")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. Base height
    ax = axes[2, 0]
    ax.plot(t, pelvis_z)
    ax.axhline(0.5, color="r", ls="--", alpha=0.3, label="limit=0.5 m")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)")
    ax.set_title("Pelvis Height")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. Solve time
    ax = axes[2, 1]
    ax.plot(t, solves)
    ax.axhline(2000, color="r", ls="--", alpha=0.3, label="budget=2000 us")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Solve time (us)")
    ax.set_title("OSQP Solve Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Add vertical lines for state transitions
    state_changes = []
    prev = states[0] if len(states) > 0 else None
    for i, s in enumerate(states):
        if s != prev:
            state_changes.append((t[i], s))
            prev = s
    for ax in axes.flat:
        for tc, s in state_changes:
            ax.axvline(tc, color="g", ls="--", alpha=0.4)
            ax.text(tc, ax.get_ylim()[1], s, color="g", fontsize=7, rotation=90, va="top")

    plt.tight_layout()
    path = os.path.join(out_dir, "transition_diagnostics.png")
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
    controller = TransitionController(env, cfg)
    controller.reset()

    duration = (
        controller.t_bipedal
        + controller.t_weight_shift
        + controller.t_single_leg
        + 0.5  # small buffer
    )

    print("=" * 60)
    print("Phase 2 Test: Transition Controller (Bipedal -> Single-Leg)")
    print("=" * 60)
    print(f"Sim dt: {env.dt:.6f} s | Duration: {duration:.1f} s")
    print(f"Support foot: {controller.support_foot_name}")
    print(f"CoM target (single): {controller.com_target_single}")
    print()

    logs = run_simulation(env, controller, duration=duration)
    metrics = compute_metrics(env, controller, logs)
    checks = assess(metrics)
    report(metrics, checks)
    save_logs(logs)
    plot_results(logs, metrics, com_target_single=controller.com_target_single)


if __name__ == "__main__":
    main()
