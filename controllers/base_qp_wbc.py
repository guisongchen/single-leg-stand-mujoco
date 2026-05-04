import numpy as np
import scipy.sparse as sp
import osqp
import mujoco

from env import G1Env

from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
)


class QPWBCController:
    """
    Base class for QP-based Whole-Body Controllers.

    Provides shared infrastructure:
      - Centroidal angular momentum Jacobian
      - QP construction and OSQP solve with variable number of contacts
      - Friction-pyramid + CoP inequality generation
      - Analytical torque recovery
      - Diagnostic printing

    Subclasses override ``reset()`` and ``compute()`` to define task targets
    and choose which feet are active.
    """

    def __init__(self, env: G1Env, config: dict):
        self.env = env
        self.cfg = config

        c = config.get("control", {})
        self.kp_joint = float(c.get("stance_kp_joint", 100.0))
        self.kd_joint = float(c.get("stance_kd_joint", 8.0))
        self.kp_com = float(c.get("stance_kp_com", 120.0))
        self.kd_com = float(c.get("stance_kd_com", 20.0))
        self.kp_pelvis = float(c.get("stance_kp_pelvis", 80.0))
        self.kd_pelvis = float(c.get("stance_kd_pelvis", 12.0))

        self.w_com = float(c.get("wbc_w_com", 100.0))
        self.w_cam = float(c.get("wbc_w_pelvis", 50.0))
        self.w_posture = float(c.get("wbc_w_posture", 1.0))
        self.w_swing = float(c.get("wbc_w_swing", 0.1))
        self.reg = float(c.get("wbc_reg", 0.01))
        self.reg_lambda = float(c.get("wbc_reg_lambda", 1e-6))

        self.mu = float(config.get("simulation", {}).get("friction", 0.8))
        # Foot CoP envelope in foot frame, derived from the 4 corner spheres in
        # the G1 XML (positions: x in [-0.05, +0.12], y in [-0.025, +0.025]).
        # The rectangle is asymmetric in x (heel only 5 cm back, toe 12 cm
        # forward of the ankle joint), so a symmetric "half length" bound
        # would either forbid valid CoPs near the toe or admit impossible
        # ones behind the heel.
        self.foot_cop_x_back = 0.05      # CoP may extend 5 cm behind ankle
        self.foot_cop_x_forward = 0.12   # CoP may extend 12 cm forward of ankle
        self.foot_cop_y_half = 0.025     # CoP envelope half-width
        self.max_contact_force = float(c.get("wbc_max_contact_force", 5000.0))
        self.tau_limit = c.get("wbc_tau_limit", None)
        if self.tau_limit is not None:
            self.tau_limit = float(self.tau_limit)

        self._total_mass = float(env.model.body_subtreemass[0])

        self.q_ref: np.ndarray | None = None
        self.com_target: np.ndarray | None = None

        self._pelvis_bid = env._body_ids["pelvis"]
        self._left_bid = env._body_ids["left_foot"]
        self._right_bid = env._body_ids["right_foot"]

        nv = env.model.nv
        self._M = np.zeros((nv, nv))
        self._J_body_lin = np.zeros((3, nv))
        self._J_body_ang = np.zeros((3, nv))

    def _compute_cam(self, model, data, com_pos):
        """Return centroidal angular momentum and its Jacobian."""
        J_cam = self._compute_centroidal_angmom_jacobian(model, data, com_pos)
        cam = J_cam @ data.qvel
        cam_rate = J_cam @ data.qacc
        return cam, J_cam, cam_rate

    def _compute_task_targets(
        self,
        model,
        data,
        com_pos,
        com_target,
        q_ref,
        q,
        dq,
    ):
        """Compute desired task accelerations (CoM, CAM, posture)."""
        nv = model.nv
        com_vel = compute_com_velocity(model, data)
        com_accel_des = (
            self.kp_com * (com_target - com_pos)
            + self.kd_com * (-com_vel)
        )

        kp_cam = self.kp_pelvis / self._total_mass * 1.0
        kd_cam = self.kd_pelvis / self._total_mass * 1.0
        cam, J_cam, cam_rate = self._compute_cam(model, data, com_pos)
        cam_rate_des = (
            kp_cam * (-cam)
            + kd_cam * (-cam_rate)
        )

        joint_accel_des = (
            self.kp_joint * (q_ref - q)
            + self.kd_joint * (-dq)
        )

        return com_accel_des, cam_rate_des, joint_accel_des, J_cam

    def _solve_osqp(self, P, q, A, l, u, nx, step):
        """Solve QP with OSQP; retry with relaxed tolerances on failure."""
        m = osqp.OSQP()
        m.setup(
            P=P, q=q, A=A, l=l, u=u,
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
            polish=True,
        )
        if hasattr(self, "_x_prev") and self._x_prev.shape[0] == nx:
            m.warm_start(x=self._x_prev)
        res = m.solve()

        if res.info.status_val not in (1, 2):
            m.setup(
                P=P, q=q, A=A, l=l, u=u,
                verbose=False,
                eps_abs=1e-3,
                eps_rel=1e-3,
                max_iter=10000,
                polish=False,
            )
            if hasattr(self, "_x_prev") and self._x_prev.shape[0] == nx:
                m.warm_start(x=self._x_prev)
            res = m.solve()
        return res

    def _solve_qp(
        self,
        model,
        data,
        J_com,
        J_cam,
        com_accel_des,
        cam_rate_des,
        joint_accel_des,
        active_feet,          # list of {"jacobian": J, "name": str}
        bias_force,
        swing_task=None,      # optional {"jacobian": J, "accel_des": a}
        extra_tasks=None,     # optional list of (weight, J, accel_des)
    ):
        """
        Build and solve the QP for a variable number of contact feet.

        Parameters
        ----------
        active_feet : list[dict]
            Each dict has keys ``jacobian`` (m x nv, m=3 or 6) and ``name``.
        swing_task : dict | None
            Optional soft objective for a swing foot:
            ``{"jacobian": J, "accel_des": a_des}``.

        Returns
        -------
        qacc_des : np.ndarray
        wrenches : list[np.ndarray]
            One m-D wrench per active foot, in the same order as ``active_feet``.
        tau : np.ndarray
            Actuator torques (nu,).
        """
        nv = model.nv
        n_feet = len(active_feet)
        lambda_dims = [foot["jacobian"].shape[0] for foot in active_feet]
        n_lambda = sum(lambda_dims)
        nx = nv + n_lambda

        # ---- Objective -------------------------------------------------
        tasks = [
            (np.sqrt(self.w_com), J_com, com_accel_des),
            (np.sqrt(self.w_cam), J_cam, cam_rate_des),
            (np.sqrt(self.w_posture), np.eye(nv)[6:], joint_accel_des),
        ]
        if swing_task is not None:
            tasks.append(
                (np.sqrt(self.w_swing), swing_task["jacobian"], swing_task["accel_des"])
            )
        if extra_tasks is not None:
            for weight, jac, target in extra_tasks:
                tasks.append((np.sqrt(weight), jac, target))

        J_tasks_weighted = np.vstack([w * J for w, J, _ in tasks])
        task_targets_weighted = np.hstack([
            w * target for w, _, target in tasks
        ])

        hessian_qacc = J_tasks_weighted.T @ J_tasks_weighted + self.reg * np.eye(nv)
        gradient_qacc = -J_tasks_weighted.T @ task_targets_weighted

        if n_lambda > 0:
            qp_hessian = sp.block_diag((hessian_qacc, self.reg_lambda * np.eye(n_lambda)), format="csc")
            qp_lin_term = np.hstack([gradient_qacc, np.zeros(n_lambda)])
        else:
            qp_hessian = sp.csc_matrix(hessian_qacc)
            qp_lin_term = gradient_qacc

        # ---- Equality constraints --------------------------------------
        eq_blocks = []
        rhs_blocks = []

        # Floating-base dynamics
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

        # Fixed-foot kinematics (with optional velocity damping / offset)
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

        # ---- Inequality constraints ------------------------------------
        if n_lambda > 0:
            constr_ineq, lb_ineq, ub_ineq = self._build_wrench_cones(nv, active_feet)
        else:
            n_ineq = 0
            constr_ineq = np.zeros((0, nx))
            lb_ineq = np.zeros(0)
            ub_ineq = np.zeros(0)

        # ---- Torque limits ---------------------------------------------
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

        # ---- Stack for OSQP --------------------------------------------
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

        # ---- Solve -----------------------------------------------------
        step = int(data.time / model.opt.timestep)
        res = self._solve_osqp(
            qp_hessian,
            qp_lin_term,
            constr_qp,
            lb_qp,
            ub_qp,
            nx,
            step,
        )

        # # ---- Diagnostics -----------------------------------------------
        # if res.info.iter >= 1000 or res.info.status_val not in (1, 2):
        #     self._print_osqp_diagnostics(
        #         step=step,
        #         res=res,
        #         com_pos=compute_com_position(model, data),
        #         constr_eq=constr_eq,
        #         active_feet=active_feet,
        #         nx=nx,
        #         nv=nv,
        #     )

        if res.info.status_val not in (1, 2):
            raise RuntimeError(f"OSQP failed at step {step}: {res.info.status}")

        x_opt = res.x
        self._x_prev = x_opt.copy()

        qacc_des = x_opt[:nv]
        wrenches = []
        start = nv
        for m in lambda_dims:
            wrenches.append(x_opt[start:start + m])
            start += m

        # ---- Torque recovery -------------------------------------------
        tau_full = self._M @ qacc_des + bias_force
        for foot, wrench in zip(active_feet, wrenches):
            tau_full -= foot["jacobian"].T @ wrench

        return qacc_des, wrenches, tau_full[6:]

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _compute_centroidal_angmom_jacobian(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        com_pos: np.ndarray,
    ) -> np.ndarray:
        nv = model.nv
        J_L = np.zeros((3, nv))

        for bid in range(1, model.nbody):
            m = model.body_mass[bid]
            if m <= 0.0:
                continue

            R = data.ximat[bid, :9].reshape(3, 3)
            I_body = np.diag(model.body_inertia[bid, :])
            I_world = R @ I_body @ R.T

            com_offset_body = model.body_ipos[bid, :]
            com_offset_world = R @ com_offset_body

            c_i = data.xipos[bid, :]
            r_rel = c_i - com_pos

            self._J_body_lin.fill(0.0)
            self._J_body_ang.fill(0.0)
            mujoco.mj_jacBody(model, data, self._J_body_lin, self._J_body_ang, bid)

            J_lin_com = self._J_body_lin - self._skew(com_offset_world) @ self._J_body_ang
            J_L += I_world @ self._J_body_ang + m * self._skew(r_rel) @ J_lin_com

        return J_L

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])

    @staticmethod
    def _quat_error(q_des: np.ndarray, q_cur: np.ndarray) -> np.ndarray:
        qw, qx, qy, qz = q_cur
        dot = qw * q_des[0] + qx * q_des[1] + qy * q_des[2] + qz * q_des[3]
        if dot < 0.0:
            qw, qx, qy, qz = -qw, -qx, -qy, -qz
        w0, x0, y0, z0 = q_des
        w_err = w0 * qw + x0 * qx + y0 * qy + z0 * qz
        x_err = w0 * qx - x0 * qw - y0 * qz + z0 * qy
        y_err = w0 * qy + x0 * qz - y0 * qw - z0 * qx
        z_err = w0 * qz - x0 * qy + y0 * qx - z0 * qw
        w_err = np.clip(w_err, -1.0, 1.0)
        scale = 2.0 if w_err >= 0 else -2.0
        return np.array([scale * x_err, scale * y_err, scale * z_err])

    def _build_wrench_cones(self, nv: int, active_feet: list):
        mu = self.mu
        cop_y_half = self.foot_cop_y_half
        cop_x_back = self.foot_cop_x_back
        cop_x_forward = self.foot_cop_x_forward

        lambda_dims = [foot["jacobian"].shape[0] for foot in active_feet]
        n_ineq = 0
        for m in lambda_dims:
            n_ineq += 5 if m == 3 else 9

        nx = nv + sum(lambda_dims)

        A = np.zeros((n_ineq, nx))
        l = np.full(n_ineq, -np.inf)
        u = np.zeros(n_ineq)

        row = 0
        lam_start = nv
        for foot in active_feet:
            m = foot["jacobian"].shape[0]
            if m == 3:
                fx_i = lam_start + 0
                fy_i = lam_start + 1
                fz_i = lam_start + 2

                A[row, fz_i] = 1.0
                l[row] = 0.0
                u[row] = self.max_contact_force
                row += 1

                A[row, fx_i] = 1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fx_i] = -1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fy_i] = 1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fy_i] = -1.0
                A[row, fz_i] = -mu
                row += 1

            elif m == 6:
                fx_i = lam_start + 0
                fy_i = lam_start + 1
                fz_i = lam_start + 2
                tx_i = lam_start + 3
                ty_i = lam_start + 4

                A[row, fz_i] = 1.0
                l[row] = 0.0
                u[row] = self.max_contact_force
                row += 1

                A[row, fx_i] = 1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fx_i] = -1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fy_i] = 1.0
                A[row, fz_i] = -mu
                row += 1

                A[row, fy_i] = -1.0
                A[row, fz_i] = -mu
                row += 1

                # CoP envelope (foot frame). World-frame torques about the
                # foot body origin satisfy tx = cop_y * fz, ty = -cop_x * fz,
                # so y bounds become symmetric on tx and x bounds become
                # asymmetric on ty.
                A[row, tx_i] = 1.0
                A[row, fz_i] = -cop_y_half
                row += 1

                A[row, tx_i] = -1.0
                A[row, fz_i] = -cop_y_half
                row += 1

                # ty <= cop_x_back * fz   (CoP at -cop_x_back: heel)
                A[row, ty_i] = 1.0
                A[row, fz_i] = -cop_x_back
                row += 1

                # ty >= -cop_x_forward * fz   (CoP at +cop_x_forward: toe)
                A[row, ty_i] = -1.0
                A[row, fz_i] = -cop_x_forward
                row += 1

            lam_start += m

        return A, l, u

    def _print_osqp_diagnostics(
        self,
        step: int,
        res,
        com_pos: np.ndarray,
        constr_eq: np.ndarray,
        active_feet: list,
        nx: int,
        nv: int,
    ) -> None:
        data = self.env.data
        model = self.env.model

        print(f"\n=== OSQP diagnostic at step {step} ===")
        print(f"  status      : {res.info.status} (val={res.info.status_val})")
        print(f"  iterations  : {res.info.iter}")
        print(f"  ncon        : {data.ncon}")
        print(f"  active_feet : {[f['name'] for f in active_feet]}")

        if self.com_target is not None:
            print(f"  com_err     : {np.linalg.norm(self.com_target - com_pos):.4f} m")
        print(f"  cond(constr_eq) : {np.linalg.cond(constr_eq):.2e}")

        if res.x is not None:
            start = nv
            for foot in active_feet:
                m = foot["jacobian"].shape[0]
                w = res.x[start:start + m]
                start += m
                print(f"  {foot['name']:6s} fz={w[2]:7.2f}  |fx|/fz={abs(w[0])/(w[2]+1e-6):.3f}")

        from utils.kinematics import compute_contact_wrench
        for foot in active_feet:
            name = foot["name"]
            body_name = self.env.cfg["robot"]["body_names"].get(name)
            if body_name:
                f_mc = compute_contact_wrench(model, data, body_name)
                print(f"  MC {name:6s} fz={f_mc[2]:7.2f}")

        print("=" * 50)
