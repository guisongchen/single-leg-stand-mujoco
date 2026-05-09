"""Diagnostic v2: track QP residuals + CoM/GRF to pinpoint transition failure.
"""

import os, sys
import numpy as np
import mujoco
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.walking_controller import WalkingController
from utils.kinematics import compute_com_position

CONFIG_PATH = "configs/g1_config.yaml"


class InstrumentedWalkingController(WalkingController):
    def __init__(self, env, config):
        super().__init__(env, config)
        self._diag_log: list[dict] = []

    def _solve_qp(self, model, data, J_com, J_cam, com_accel_des, cam_rate_des,
                  joint_accel_des, active_feet, bias_force, swing_task=None,
                  extra_tasks=None):
        nv = model.nv
        M_snap = self._M.copy()
        h_snap = bias_force.copy()
        active_feet_snap = [dict(f) for f in active_feet]

        try:
            qacc_des, wrenches, tau = super()._solve_qp(
                model=model, data=data, J_com=J_com, J_cam=J_cam,
                com_accel_des=com_accel_des, cam_rate_des=cam_rate_des,
                joint_accel_des=joint_accel_des, active_feet=active_feet,
                bias_force=bias_force, swing_task=swing_task,
                extra_tasks=extra_tasks)
        except RuntimeError:
            self._record_diag(data.time, M_snap, h_snap, active_feet_snap, None, None)
            raise

        self._record_diag(data.time, M_snap, h_snap, active_feet_snap, qacc_des, wrenches)
        return qacc_des, wrenches, tau

    def _record_diag(self, t, M, h, active_feet, qacc_qp, wrenches):
        nv = M.shape[0]
        data = self.env.data
        model = self.env.model

        qp_residual = np.full(6, np.nan)
        if qacc_qp is not None and wrenches is not None:
            contact_term = np.zeros(6)
            for foot, w in zip(active_feet, wrenches):
                jac = foot["jacobian"]
                contact_term += jac[:, :6].T @ w
            qp_residual = M[:6] @ qacc_qp + h[:6] - contact_term

        mj_consistency = M[:6] @ data.qacc + h[:6] - data.qfrc_constraint[:6]

        lambda_dims = [f["jacobian"].shape[0] for f in active_feet]
        n_lambda = sum(lambda_dims)
        cond_val = np.nan
        if n_lambda > 0:
            constr_fb = np.zeros((6, nv + n_lambda))
            constr_fb[:, :nv] = M[:6, :]
            col = nv
            for foot in active_feet:
                m = foot["jacobian"].shape[0]
                constr_fb[:, col:col + m] = -foot["jacobian"][:, :6].T
                col += m
            cond_val = np.linalg.cond(constr_fb)

        com_pos = compute_com_position(model, data)
        com_target = self.com_target if self.com_target is not None else np.full(3, np.nan)

        left_grf = self._compute_grf("left_foot")
        right_grf = self._compute_grf("right_foot")

        self._diag_log.append({
            "t": float(t),
            "phase": self._phase,
            "qp_residual_max": float(np.nanmax(np.abs(qp_residual))),
            "mj_consistency_max": float(np.nanmax(np.abs(mj_consistency))),
            "cond_constr_fb": float(cond_val),
            "active_feet": [f["name"] for f in active_feet],
            "com_x": float(com_pos[0]),
            "com_y": float(com_pos[1]),
            "com_target_x": float(com_target[0]) if not np.isnan(com_target[0]) else np.nan,
            "com_target_y": float(com_target[1]) if not np.isnan(com_target[1]) else np.nan,
            "com_err": float(np.linalg.norm(com_pos[:2] - com_target[:2])) if not np.isnan(com_target[0]) else np.nan,
            "left_grf": float(left_grf),
            "right_grf": float(right_grf),
            "grf_ratio_left": float(left_grf / (left_grf + right_grf + 1e-6)),
            "grf_ratio_right": float(right_grf / (left_grf + right_grf + 1e-6)),
        })


def main():
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    step_length = float(cfg.get("walking", {}).get("step_length", 0.0))
    duration = 10.0

    env = G1Env(CONFIG_PATH)
    env.reset()
    ctrl = InstrumentedWalkingController(env, cfg)
    ctrl.reset()

    n_steps = int(duration / env.dt)

    print(f"Instrumented run: {n_steps} steps, step_length={step_length:.2f}m")

    try:
        for i in range(n_steps):
            tau = ctrl.compute()
            env.step(tau)
            if not np.all(np.isfinite(env.data.qpos)):
                print(f"NaN detected at t={env.data.time:.3f}")
                break
    except RuntimeError as e:
        print(f"\nController failed at t={env.data.time:.3f}: {e}")

    log = ctrl._diag_log
    if not log:
        print("No data logged.")
        return

    t_arr = np.array([r["t"] for r in log])
    phases = np.array([r["phase"] for r in log])
    qp_res = np.array([r["qp_residual_max"] for r in log])
    mj_con = np.array([r["mj_consistency_max"] for r in log])
    conds = np.array([r["cond_constr_fb"] for r in log])
    com_x = np.array([r["com_x"] for r in log])
    com_y = np.array([r["com_y"] for r in log])
    tgt_x = np.array([r["com_target_x"] for r in log])
    tgt_y = np.array([r["com_target_y"] for r in log])
    com_err = np.array([r["com_err"] for r in log])
    left_grf = np.array([r["left_grf"] for r in log])
    right_grf = np.array([r["right_grf"] for r in log])
    grf_ratio_l = np.array([r["grf_ratio_left"] for r in log])
    grf_ratio_r = np.array([r["grf_ratio_right"] for r in log])

    # ---- Summary ----
    valid = ~np.isnan(qp_res)
    print(f"\n--- Diagnostic Summary ({np.sum(valid)} valid solves / {len(log)} total) ---")
    if np.any(valid):
        print(f"  QP residual (valid):  mean={np.nanmean(qp_res):.4e}  max={np.nanmax(qp_res):.4e}")
    print(f"  MJ consistency:       mean={np.nanmean(mj_con):.4e}  max={np.nanmax(mj_con):.4e}")
    print(f"  max cond(constr_fb):  {np.nanmax(conds):.1f}")

    bad = np.nan_to_num(qp_res, nan=0.0) > 1e-3
    if np.any(bad):
        idx = np.argmax(bad)
        print(f"  First QP residual > 1e-3 at t={t_arr[idx]:.4f}s, phase={phases[idx]}")

    # Transition status before failure
    if len(log) > 2:
        last = log[-2] if not np.isnan(log[-1]["qp_residual_max"]) else log[-3]
        print(f"\n--- Last valid state (t={last['t']:.3f}s) ---")
        print(f"  phase          : {last['phase']}")
        print(f"  com_err        : {last['com_err']:.4f} m")
        print(f"  left_grf       : {last['left_grf']:.1f} N ({last['grf_ratio_left']*100:.0f}%)")
        print(f"  right_grf      : {last['right_grf']:.1f} N ({last['grf_ratio_right']*100:.0f}%)")
        print(f"  active_feet    : {last['active_feet']}")

    # ---- Plots ----
    phase_colors = {
        "BIPEDAL_INIT": "#e8e8e8", "WEIGHT_SHIFT_L": "#cce5ff",
        "LEFT_SINGLE": "#ffe0b2", "DOUBLE_SUPPORT": "#c8e6c9",
        "WEIGHT_SHIFT_R": "#cce5ff", "RIGHT_SINGLE": "#ffe0b2",
    }

    fig, axes = plt.subplots(4, 1, figsize=(16, 14), sharex=True)

    def paint_bands(ax):
        if len(phases) == 0:
            return
        bs = t_arr[0]
        bp = phases[0]
        for i in range(1, len(phases)):
            if phases[i] != bp:
                ax.axvspan(bs, t_arr[i - 1], alpha=0.08, color=phase_colors.get(bp, "#fff"))
                bs = t_arr[i]
                bp = phases[i]
        ax.axvspan(bs, t_arr[-1], alpha=0.08, color=phase_colors.get(bp, "#fff"))

    # Panel 1: QP residual
    ax = axes[0]
    y = np.maximum(np.nan_to_num(qp_res, nan=1e-16), 1e-16)
    ax.semilogy(t_arr, y, 'b-', lw=0.8, label="|QP constraint residual|")
    ax.axhline(1e-3, color='orange', ls='--', lw=0.8, label="1e-3")
    ax.set_ylabel("Residual (log)")
    ax.set_title("QP Floating-Base Constraint Residual")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    paint_bands(ax)

    # Panel 2: CoM tracking
    ax = axes[1]
    ax.plot(t_arr, com_x, 'b-', lw=0.8, label="CoM x")
    ax.plot(t_arr, com_y, 'r-', lw=0.8, label="CoM y")
    mask_tgt = ~np.isnan(tgt_x)
    if mask_tgt.any():
        ax.plot(t_arr[mask_tgt], tgt_x[mask_tgt], 'b--', lw=0.6, alpha=0.6, label="target x")
        ax.plot(t_arr[mask_tgt], tgt_y[mask_tgt], 'r--', lw=0.6, alpha=0.6, label="target y")
    ax.set_ylabel("Position (m)")
    ax.set_title("CoM Position vs Target")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    paint_bands(ax)

    # Panel 3: GRF per foot
    ax = axes[2]
    mg = 34.13 * 9.81
    ax.plot(t_arr, left_grf, 'b-', lw=0.8, label="left GRF")
    ax.plot(t_arr, right_grf, 'r-', lw=0.8, label="right GRF")
    ax.axhline(0.5 * mg, color='g', ls='--', lw=0.8, label="50% mg (arm)")
    ax.axhline(0.8 * mg, color='orange', ls='--', lw=0.8, label="80% mg (fire)")
    ax.axhline(5, color='purple', ls=':', lw=0.8, label="5N liftoff")
    ax.set_ylabel("Force (N)")
    ax.set_title("Ground Reaction Force per Foot")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    paint_bands(ax)

    # Panel 4: GRF ratio
    ax = axes[3]
    ax.plot(t_arr, grf_ratio_l * 100, 'b-', lw=0.8, label="left %")
    ax.plot(t_arr, grf_ratio_r * 100, 'r-', lw=0.8, label="right %")
    ax.axhline(80, color='orange', ls='--', lw=0.8, label="80% fire")
    ax.axhline(50, color='g', ls='--', lw=0.8, label="50% arm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("% of total GRF")
    ax.set_title("GRF Distribution")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    paint_bands(ax)

    plt.tight_layout()
    os.makedirs("outputs", exist_ok=True)
    path = "outputs/debug_qp_consistency_v2.png"
    plt.savefig(path, dpi=120)
    print(f"\nSaved: {path}")


if __name__ == "__main__":
    main()
