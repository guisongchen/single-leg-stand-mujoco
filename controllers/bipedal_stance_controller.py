import numpy as np
import scipy.sparse as sp
import osqp
import mujoco

from env import G1Env

from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    euler_from_quat,
)


class BipedalStanceController:
    """
    Full QP-based Whole-Body Controller for bipedal standing.

    Decision variables:
        x = [qacc (nv); wrench_left (6); wrench_right (6)]

    Hard equality constraints:
        - Floating-base dynamics (no actuators → contact wrenches must balance)
        - Both feet remain fixed (zero spatial acceleration)

    Hard inequality constraints:
        - Friction pyramid: |fx|, |fy| <= mu * fz
        - CoP bounds: |tx| <= (W/2)*fz, |ty| <= (L/2)*fz
        - Unilateral contact: fz >= 0

    Objective (weighted least-squares on qacc only):
        - CoM linear acceleration tracks horizontal target
        - Centroidal angular momentum rate drives L toward zero
        - Joint posture (lowest priority)

    Torques are recovered analytically from the full dynamics equation,
    eliminating the need for ``mj_inverse``.
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
        # wbc_w_pelvis is repurposed as centroidal angular momentum weight
        self.w_cam = float(c.get("wbc_w_pelvis", 50.0))
        self.w_posture = float(c.get("wbc_w_posture", 1.0))
        self.reg = float(c.get("wbc_reg", 0.01))
        self.reg_lambda = float(c.get("wbc_reg_lambda", 1e-6))

        self.mu = float(config.get("simulation", {}).get("friction", 0.8))
        # Foot support polygon half-size (m) from contact-sphere positions in XML
        self.foot_half_length = 0.085   # x: -0.05 … 0.12
        self.foot_half_width = 0.030    # y: -0.03 … 0.03

        # Total mass for scaling centroidal-angular-momentum gains
        self._total_mass = float(env.model.body_subtreemass[0])

        self.q_ref: np.ndarray | None = None
        self.com_target: np.ndarray | None = None
        self.pelvis_quat_ref: np.ndarray | None = None

        self._pelvis_bid = env._body_ids["pelvis"]
        self._left_bid = env._body_ids["left_foot"]
        self._right_bid = env._body_ids["right_foot"]

        nv = env.model.nv
        n_lambda = 12          # 2 feet × 6D wrench
        self._nx = nv + n_lambda
        self._x_prev = np.zeros(self._nx)

        # Pre-allocate reusable buffers
        self._M = np.zeros((nv, nv))
        self._J_body_lin = np.zeros((3, nv))
        self._J_body_ang = np.zeros((3, nv))

    def reset(self) -> None:
        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_target = compute_com_position(self.env.model, self.env.data)
        self.pelvis_quat_ref = self.env.get_pelvis_quat().copy()
        self._x_prev = np.zeros(self._nx)

    def compute(self) -> np.ndarray:
        model = self.env.model
        data = self.env.data
        nv = model.nv

        q = self.env.get_actuated_qpos()
        dq = self.env.get_actuated_qvel()

        # ---- Mass matrix & bias forces --------------------------------
        mujoco.mj_fullM(model, self._M, data.qM)
        bias_force = data.qfrc_bias - data.qfrc_passive

        # ---- Jacobians -------------------------------------------------
        J_com = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(model, data, J_com, 0)

        J_left = np.zeros((6, nv))
        J_right = np.zeros((6, nv))
        mujoco.mj_jacBody(model, data, J_left[:3], J_left[3:], self._left_bid)
        mujoco.mj_jacBody(model, data, J_right[:3], J_right[3:], self._right_bid)

        # ---- Centroidal angular momentum -------------------------------
        com_pos = compute_com_position(model, data)
        cam, J_cam = self._compute_centroidal_angular_momentum(model, data, com_pos)
        # Approximate current CAM rate from joint accelerations
        cam_rate = J_cam @ data.qacc

        # ---- Desired task accelerations (PD + feedforward) -------------
        # For standing, feedforward references are zero.  Explicit form
        # makes it trivial to swap in a trajectory planner later.
        com_vel = compute_com_velocity(model, data)
        com_accel_ref = np.zeros(3)
        com_vel_ref = np.zeros(3)
        com_accel_des = (
            com_accel_ref
            + self.kp_com * (self.com_target - com_pos)
            + self.kd_com * (com_vel_ref - com_vel)
        )

        # Scale pelvis gains by total mass so they behave similarly on
        # angular momentum (kg·m²/s) as they did on angular acceleration (rad/s²).
        kp_cam = self.kp_pelvis / self._total_mass
        kd_cam = self.kd_pelvis / self._total_mass
        cam_rate_ref = np.zeros(3)
        cam_ref = np.zeros(3)
        cam_rate_des = (
            cam_rate_ref
            + kp_cam * (cam_ref - cam)
            + kd_cam * (cam_rate_ref - cam_rate)
        )

        joint_accel_ref = np.zeros(nv - 6)
        joint_vel_ref = np.zeros(nv - 6)
        joint_accel_des = (
            joint_accel_ref
            + self.kp_joint * (self.q_ref - q)
            + self.kd_joint * (joint_vel_ref - dq)
        )

        # ---- Build objective (only penalise qacc) ----------------------
        J_tasks_weighted = np.vstack([
            np.sqrt(self.w_com) * J_com,
            np.sqrt(self.w_cam) * J_cam,
            np.sqrt(self.w_posture) * np.eye(nv)[6:],
        ])
        task_targets_weighted = np.hstack([
            np.sqrt(self.w_com) * com_accel_des,
            np.sqrt(self.w_cam) * cam_rate_des,
            np.sqrt(self.w_posture) * joint_accel_des,
        ])

        hessian_qacc = J_tasks_weighted.T @ J_tasks_weighted + self.reg * np.eye(nv)
        gradient_qacc = -J_tasks_weighted.T @ task_targets_weighted

        # Extend to full decision vector [qacc; wrench_left; wrench_right]
        qp_hessian = sp.block_diag((hessian_qacc, self.reg_lambda * np.eye(12)), format="csc")
        qp_lin_term = np.hstack([gradient_qacc, np.zeros(12)])

        # ---- Equality constraints --------------------------------------
        # 1. Floating-base dynamics (6 eq)
        #    M_fb*qacc - J_left[:,:6].T*wrench_l - J_right[:,:6].T*wrench_r = -bias_fb
        constr_fb = np.zeros((6, self._nx))
        constr_fb[:, :nv] = self._M[:6, :]
        constr_fb[:, nv:nv + 6] = -J_left[:, :6].T
        constr_fb[:, nv + 6:] = -J_right[:, :6].T
        rhs_fb = -bias_force[:6]

        # 2. Left foot fixed (6 eq)   J_left * qacc = 0
        constr_lfoot = np.zeros((6, self._nx))
        constr_lfoot[:, :nv] = J_left
        rhs_lfoot = np.zeros(6)

        # 3. Right foot fixed (6 eq)  J_right * qacc = 0
        constr_rfoot = np.zeros((6, self._nx))
        constr_rfoot[:, :nv] = J_right
        rhs_rfoot = np.zeros(6)

        constr_eq = np.vstack([constr_fb, constr_lfoot, constr_rfoot])
        lb_eq = np.hstack([rhs_fb, rhs_lfoot, rhs_rfoot])
        ub_eq = lb_eq.copy()

        # ---- Inequality constraints (friction pyramid + CoP) -----------
        constr_ineq, lb_ineq, ub_ineq = self._build_wrench_cones(nv)

        # ---- Stack for OSQP --------------------------------------------
        constr_qp = sp.csc_matrix(np.vstack([constr_eq, constr_ineq]))
        lb_qp = np.hstack([lb_eq, lb_ineq])
        ub_qp = np.hstack([ub_eq, ub_ineq])

        # ---- Solve -----------------------------------------------------
        m = osqp.OSQP()
        m.setup(
            P=qp_hessian,
            q=qp_lin_term,
            A=constr_qp,
            l=lb_qp,
            u=ub_qp,
            verbose=False,
            eps_abs=1e-5,
            eps_rel=1e-5,
            max_iter=4000,
            polish=True,
        )
        m.warm_start(x=self._x_prev)
        res = m.solve()

        if res.info.status_val not in (1, 2):  # 1=solved, 2=solved inaccurate
            raise RuntimeError(f"OSQP failed: {res.info.status}")

        x_opt = res.x
        self._x_prev = x_opt.copy()

        qacc_des = x_opt[:nv]
        wrench_lfoot = x_opt[nv:nv + 6]
        wrench_rfoot = x_opt[nv + 6:]

        # ---- Analytical torque from full dynamics ----------------------
        # tau = (M*qacc + bias_force - J_left^T*wrench_l - J_right^T*wrench_r)[6:]
        tau_full = (
            self._M @ qacc_des
            + bias_force
            - J_left.T @ wrench_lfoot
            - J_right.T @ wrench_rfoot
        )
        return tau_full[6:]

    def _compute_centroidal_angular_momentum(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        com_pos: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Compute centroidal angular momentum and its Jacobian.

        L = sum_i [ I_i_world * omega_i + m_i * (c_i - com) × v_i ]

        J_L = sum_i [ I_i_world * J_ang_i + m_i * skew(c_i - com) * J_lin_i ]

        where J_lin_i is the Jacobian of body i's CoM.
        """
        nv = model.nv
        L = np.zeros(3)
        J_L = np.zeros((3, nv))

        for bid in range(1, model.nbody):
            m = model.body_mass[bid]
            if m <= 0.0:
                continue

            # Body orientation (world from body)
            R = data.ximat[bid, :9].reshape(3, 3)

            # Body inertia in world frame
            I_body = np.diag(model.body_inertia[bid, :])
            I_world = R @ I_body @ R.T

            # CoM offset in body frame → world frame
            com_offset_body = model.body_ipos[bid, :]
            com_offset_world = R @ com_offset_body

            # Body CoM position
            c_i = data.xipos[bid, :]
            r_rel = c_i - com_pos

            # Jacobians of body origin
            self._J_body_lin.fill(0.0)
            self._J_body_ang.fill(0.0)
            mujoco.mj_jacBody(model, data, self._J_body_lin, self._J_body_ang, bid)

            # Adjust linear Jacobian to body CoM:
            #   v_com = v_origin + omega × offset
            #   J_lin_com = J_lin_origin - skew(offset) @ J_ang
            J_lin_com = self._J_body_lin - self._skew(com_offset_world) @ self._J_body_ang

            # Accumulate
            L += I_world @ (self._J_body_ang @ data.qvel) + m * np.cross(r_rel, J_lin_com @ data.qvel)
            J_L += I_world @ self._J_body_ang + m * self._skew(r_rel) @ J_lin_com

        return L, J_L

    @staticmethod
    def _skew(v: np.ndarray) -> np.ndarray:
        """Skew-symmetric matrix of a 3-vector."""
        return np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])

    def _build_wrench_cones(self, nv: int):
        """
        Build friction-pyramid + CoP inequalities for both feet.

        Per foot (wrench = [fx, fy, fz, tx, ty, tz]):
            fz >= 0
            |fx| <= mu*fz      →  fx - mu*fz <= 0,  -fx - mu*fz <= 0
            |fy| <= mu*fz      →  fy - mu*fz <= 0,  -fy - mu*fz <= 0
            |tx| <= (W/2)*fz   →  tx - Hw*fz <= 0,  -tx - Hw*fz <= 0
            |ty| <= (L/2)*fz   →  ty - Hl*fz <= 0,  -ty - Hl*fz <= 0
        """
        mu = self.mu
        Hw = self.foot_half_width
        Hl = self.foot_half_length
        n_ineq_per_foot = 9
        n_ineq = 2 * n_ineq_per_foot

        A = np.zeros((n_ineq, self._nx))
        l = np.full(n_ineq, -np.inf)
        u = np.zeros(n_ineq)

        for i, lam_start in enumerate((nv, nv + 6)):
            fx_i = lam_start + 0
            fy_i = lam_start + 1
            fz_i = lam_start + 2
            tx_i = lam_start + 3
            ty_i = lam_start + 4

            row = i * n_ineq_per_foot

            # fz >= 0
            A[row, fz_i] = 1.0
            l[row] = 0.0
            u[row] = np.inf
            row += 1

            # fx - mu*fz <= 0
            A[row, fx_i] = 1.0
            A[row, fz_i] = -mu
            row += 1

            # -fx - mu*fz <= 0
            A[row, fx_i] = -1.0
            A[row, fz_i] = -mu
            row += 1

            # fy - mu*fz <= 0
            A[row, fy_i] = 1.0
            A[row, fz_i] = -mu
            row += 1

            # -fy - mu*fz <= 0
            A[row, fy_i] = -1.0
            A[row, fz_i] = -mu
            row += 1

            # tx - Hw*fz <= 0
            A[row, tx_i] = 1.0
            A[row, fz_i] = -Hw
            row += 1

            # -tx - Hw*fz <= 0
            A[row, tx_i] = -1.0
            A[row, fz_i] = -Hw
            row += 1

            # ty - Hl*fz <= 0
            A[row, ty_i] = 1.0
            A[row, fz_i] = -Hl
            row += 1

            # -ty - Hl*fz <= 0
            A[row, ty_i] = -1.0
            A[row, fz_i] = -Hl
            row += 1

        return A, l, u

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
