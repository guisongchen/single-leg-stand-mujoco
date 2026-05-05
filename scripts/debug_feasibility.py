"""Debug script: check QP feasibility metrics before OSQP solve.

Patches WalkingController._solve_qp to log condition number, rank,
and NaN checks of the equality constraint matrix.  Stops on the
first failure or after a fixed step limit.
"""
from __future__ import annotations

import os
import sys
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from env.g1_env import G1Env
from controllers.walking_controller import WalkingController

CONFIG_PATH = "configs/g1_config.yaml"


def patched_solve_qp(self, model, data, J_com, J_cam, com_accel_des,
                     cam_rate_des, joint_accel_des, active_feet, bias_force,
                     swing_task=None, extra_tasks=None):
    """Wrapped _solve_qp that logs feasibility metrics before solving."""
    import scipy.sparse as sp
    import osqp
    import mujoco

    nv = model.nv
    n_feet = len(active_feet)
    lambda_dims = [foot["jacobian"].shape[0] for foot in active_feet]
    n_lambda = sum(lambda_dims)
    nx = nv + n_lambda
    step = int(data.time / model.opt.timestep)

    # ---- Build objective (copy-paste from base) ------------------------
    tasks = [
        (np.sqrt(self.w_com), J_com, com_accel_des),
        (np.sqrt(self.w_cam), J_cam, cam_rate_des),
        (np.sqrt(self.w_posture), np.eye(nv)[6:], joint_accel_des),
    ]
    if swing_task is not None:
        tasks.append((np.sqrt(self.w_swing), swing_task["jacobian"], swing_task["accel_des"]))
    if extra_tasks is not None:
        for weight, jac, target in extra_tasks:
            tasks.append((np.sqrt(weight), jac, target))

    J_tasks_weighted = np.vstack([w * J for w, J, _ in tasks])
    task_targets_weighted = np.hstack([w * target for w, _, target in tasks])

    hessian_qacc = J_tasks_weighted.T @ J_tasks_weighted + self.reg * np.eye(nv)
    gradient_qacc = -J_tasks_weighted.T @ task_targets_weighted

    if n_lambda > 0:
        qp_hessian = sp.block_diag((hessian_qacc, self.reg_lambda * np.eye(n_lambda)), format="csc")
        qp_lin_term = np.hstack([gradient_qacc, np.zeros(n_lambda)])
    else:
        qp_hessian = sp.csc_matrix(hessian_qacc)
        qp_lin_term = gradient_qacc

    # ---- Build equality constraints ------------------------------------
    eq_blocks = []
    rhs_blocks = []

    mujoco.mj_fullM(model, self._M, data.qM)
    constr_fb = np.zeros((6, nx))
    constr_fb[:, :nv] = self._M[:6, :]
    for i, foot in enumerate(active_feet):
        jac = foot["jacobian"]
        m = jac.shape[0]
        start = nv + sum(lambda_dims[:i])
        constr_fb[:, start:start + m] = -jac[:, :6].T
    eq_blocks.append(constr_fb)
    rhs_blocks.append(-bias_force[:6])

    for foot in active_feet:
        jac = foot["jacobian"]
        m = jac.shape[0]
        constr_foot = np.zeros((m, nx))
        constr_foot[:, :nv] = jac
        eq_blocks.append(constr_foot)
        rhs_blocks.append(foot.get("accel_offset", np.zeros(m))[:m])

    constr_eq = np.vstack(eq_blocks)
    lb_eq = np.hstack(rhs_blocks)
    ub_eq = lb_eq.copy()

    # ---- Feasibility metrics -------------------------------------------
    cond_eq = np.linalg.cond(constr_eq)
    rank_eq = np.linalg.matrix_rank(constr_eq, tol=1e-8)
    n_rows_eq = constr_eq.shape[0]
    full_rank = rank_eq == n_rows_eq
    min_svd = np.linalg.svd(constr_eq, compute_uv=False)[-1]

    has_nan = not (np.isfinite(constr_eq).all() and np.isfinite(lb_eq).all())

    # Log every 50 steps and on suspicious conditions
    if step % 50 == 0 or cond_eq > 1e12 or not full_rank or has_nan:
        print(
            f"step={step:4d}  phase={self._phase:18s}  "
            f"rows={n_rows_eq:2d}  rank={rank_eq:2d}  cond={cond_eq:.2e}  "
            f"min_svd={min_svd:.2e}  nan={has_nan}  "
            f"feet={[f['name'] for f in active_feet]}"
        )

    # ---- Build inequalities (copy-paste) -------------------------------
    if n_lambda > 0:
        constr_ineq, lb_ineq, ub_ineq = self._build_wrench_cones(nv, active_feet)
    else:
        n_ineq = 0
        constr_ineq = np.zeros((0, nx))
        lb_ineq = np.zeros(0)
        ub_ineq = np.zeros(0)

    if self.tau_limit is not None:
        nu = model.nv - 6
        tau_min = np.full(nu, -self.tau_limit)
        tau_max = np.full(nu, +self.tau_limit)
        A_tau = np.zeros((nu, nx))
        A_tau[:, :nv] = self._M[6:, :]
        col = nv
        for foot in active_feet:
            jac = foot["jacobian"]
            m = jac.shape[0]
            A_tau[:, col:col + m] = -jac[:, 6:].T
            col += m
        b_tau = bias_force[6:]
        tau_lb = tau_min - b_tau
        tau_ub = tau_max - b_tau
        if constr_ineq.shape[0] > 0:
            constr_ineq = np.vstack([constr_ineq, A_tau])
            lb_ineq = np.hstack([lb_ineq, tau_lb])
            ub_ineq = np.hstack([ub_ineq, tau_ub])
        else:
            constr_ineq = A_tau
            lb_ineq = tau_lb
            ub_ineq = tau_ub

    if constr_eq.shape[0] > 0 and constr_ineq.shape[0] > 0:
        constr_qp = sp.csc_matrix(np.vstack([constr_eq, constr_ineq]))
        lb_qp = np.hstack([lb_eq, lb_ineq])
        ub_qp = np.hstack([ub_eq, ub_ineq])
    elif constr_eq.shape[0] > 0:
        constr_qp = sp.csc_matrix(constr_eq)
        lb_qp = lb_eq
        ub_qp = ub_eq
    else:
        constr_qp = sp.csc_matrix(constr_ineq)
        lb_qp = lb_ineq
        ub_qp = ub_ineq

    # ---- Solve with OSQP -----------------------------------------------
    m = osqp.OSQP()
    m.setup(
        P=qp_hessian, q=qp_lin_term, A=constr_qp, l=lb_qp, u=ub_qp,
        verbose=False, eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, polish=True,
    )
    if hasattr(self, "_x_prev") and self._x_prev.shape[0] == nx:
        m.warm_start(x=self._x_prev)
    res = m.solve()

    if res.info.status_val not in (1, 2):
        print(f"\n!!! OSQP failure at step {step}: {res.info.status}")
        print(f"    phase={self._phase}  cond={cond_eq:.2e}  rank={rank_eq}/{n_rows_eq}")
        print(f"    min_svd={min_svd:.2e}  nan={has_nan}")
        print(f"    active_feet={[f['name'] for f in active_feet]}")
        for foot in active_feet:
            jac = foot["jacobian"]
            print(f"    {foot['name']} jac shape={jac.shape}  finite={np.isfinite(jac).all()}")
        print(f"    com_target={self.com_target}")
        print(f"    pelvis_quat={self.env.get_pelvis_quat()}")
        raise RuntimeError(f"OSQP failed at step {step}: {res.info.status}")

    x_opt = res.x
    self._x_prev = x_opt.copy()
    qacc_des = x_opt[:nv]
    wrenches = []
    start = nv
    for m_dim in lambda_dims:
        wrenches.append(x_opt[start:start + m_dim])
        start += m_dim
    tau_full = self._M @ qacc_des + bias_force
    for foot, wrench in zip(active_feet, wrenches):
        tau_full -= foot["jacobian"].T @ wrench
    return qacc_des, wrenches, tau_full[6:]


def main():
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    cfg["walking"]["step_length"] = 0.0
    cfg["walking"]["double_support_duration"] = 0.5

    env = G1Env(CONFIG_PATH)
    env.reset()
    controller = WalkingController(env, cfg)
    controller.reset()

    # Patch the solver
    WalkingController._solve_qp = patched_solve_qp

    n_steps = int(10.0 / env.dt)
    for i in range(n_steps):
        try:
            ctrl = controller.compute()
        except RuntimeError as exc:
            print(f"\nSimulation stopped at t={env.data.time:.3f}s: {exc}")
            break
        env.step(ctrl)
        if not np.all(np.isfinite(env.data.qpos)):
            print(f"NaN in qpos at t={env.data.time:.3f}s")
            break

    print("\nDone.")


if __name__ == "__main__":
    main()
