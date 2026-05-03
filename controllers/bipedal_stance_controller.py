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
        x = [qacc (nv); lambda_left (6); lambda_right (6)]

    Hard equality constraints:
        - Floating-base dynamics (no actuators → contact wrenches must balance)
        - Both feet remain fixed (zero spatial acceleration)

    Hard inequality constraints:
        - Friction pyramid: |fx|, |fy| <= mu * fz
        - CoP bounds: |tx| <= (W/2)*fz, |ty| <= (L/2)*fz
        - Unilateral contact: fz >= 0

    Objective (weighted least-squares on qacc only):
        - CoM tracks a horizontal target
        - Pelvis stays upright
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
        self.w_pelvis = float(c.get("wbc_w_pelvis", 50.0))
        self.w_posture = float(c.get("wbc_w_posture", 1.0))
        self.reg = float(c.get("wbc_reg", 0.01))
        self.reg_lambda = float(c.get("wbc_reg_lambda", 1e-6))

        self.mu = float(config.get("simulation", {}).get("friction", 0.8))
        # Foot support polygon half-size (m) from contact-sphere positions in XML
        self.foot_half_length = 0.085   # x: -0.05 … 0.12
        self.foot_half_width = 0.030    # y: -0.03 … 0.03

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
        h = data.qfrc_bias - data.qfrc_passive

        # ---- Jacobians -------------------------------------------------
        J_com = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(model, data, J_com, 0)

        J_pelvis = np.zeros((6, nv))
        mujoco.mj_jacBody(model, data, J_pelvis[:3], J_pelvis[3:], self._pelvis_bid)

        J_left = np.zeros((6, nv))
        J_right = np.zeros((6, nv))
        mujoco.mj_jacBody(model, data, J_left[:3], J_left[3:], self._left_bid)
        mujoco.mj_jacBody(model, data, J_right[:3], J_right[3:], self._right_bid)

        # ---- Desired task accelerations --------------------------------
        com_pos = compute_com_position(model, data)
        com_vel = compute_com_velocity(model, data)
        a_com = self.kp_com * (self.com_target - com_pos) - self.kd_com * com_vel

        quat_err = self._quat_error(self.pelvis_quat_ref, self.env.get_pelvis_quat())
        pelvis_omega = (J_pelvis @ data.qvel)[3:]
        alpha_pelvis = -self.kp_pelvis * quat_err - self.kd_pelvis * pelvis_omega

        a_posture = self.kp_joint * (self.q_ref - q) - self.kd_joint * dq

        # ---- Build objective (only penalise qacc) ----------------------
        A_obj = np.vstack([
            np.sqrt(self.w_com) * J_com,
            np.sqrt(self.w_pelvis) * J_pelvis[3:],
            np.sqrt(self.w_posture) * np.eye(nv)[6:],
        ])
        b_obj = np.hstack([
            np.sqrt(self.w_com) * a_com,
            np.sqrt(self.w_pelvis) * alpha_pelvis,
            np.sqrt(self.w_posture) * a_posture,
        ])

        P_qacc = A_obj.T @ A_obj + self.reg * np.eye(nv)
        q_qacc = -A_obj.T @ b_obj

        # Extend to full decision vector [qacc; lambda_left; lambda_right]
        P = sp.block_diag((P_qacc, self.reg_lambda * np.eye(12)), format="csc")
        q_vec = np.hstack([q_qacc, np.zeros(12)])

        # ---- Equality constraints --------------------------------------
        # 1. Floating-base dynamics (6 eq)
        #    M_fb*qacc - J_left[:,:6].T*lambda_l - J_right[:,:6].T*lambda_r = -h_fb
        A_dyn = np.zeros((6, self._nx))
        A_dyn[:, :nv] = self._M[:6, :]
        A_dyn[:, nv:nv + 6] = -J_left[:, :6].T
        A_dyn[:, nv + 6:] = -J_right[:, :6].T
        b_dyn = -h[:6]

        # 2. Left foot fixed (6 eq)   J_left * qacc = 0
        A_lf = np.zeros((6, self._nx))
        A_lf[:, :nv] = J_left
        b_lf = np.zeros(6)

        # 3. Right foot fixed (6 eq)  J_right * qacc = 0
        A_rf = np.zeros((6, self._nx))
        A_rf[:, :nv] = J_right
        b_rf = np.zeros(6)

        A_eq = np.vstack([A_dyn, A_lf, A_rf])
        l_eq = np.hstack([b_dyn, b_lf, b_rf])
        u_eq = l_eq.copy()

        # ---- Inequality constraints (friction pyramid + CoP) -----------
        A_ineq, l_ineq, u_ineq = self._build_wrench_cones(nv)

        # ---- Stack for OSQP --------------------------------------------
        A_osqp = sp.csc_matrix(np.vstack([A_eq, A_ineq]))
        l_osqp = np.hstack([l_eq, l_ineq])
        u_osqp = np.hstack([u_eq, u_ineq])

        # ---- Solve -----------------------------------------------------
        m = osqp.OSQP()
        m.setup(
            P=P,
            q=q_vec,
            A=A_osqp,
            l=l_osqp,
            u=u_osqp,
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
        lambda_l = x_opt[nv:nv + 6]
        lambda_r = x_opt[nv + 6:]

        # ---- Analytical torque from full dynamics ----------------------
        # tau = (M*qacc + h - J_left^T*lambda_l - J_right^T*lambda_r)[6:]
        tau_full = (
            self._M @ qacc_des
            + h
            - J_left.T @ lambda_l
            - J_right.T @ lambda_r
        )
        return tau_full[6:]

    def _build_wrench_cones(self, nv: int):
        """
        Build friction-pyramid + CoP inequalities for both feet.

        Per foot (lambda = [fx, fy, fz, tx, ty, tz]):
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
