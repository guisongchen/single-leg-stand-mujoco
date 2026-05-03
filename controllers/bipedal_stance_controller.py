import numpy as np
from scipy.linalg import solve
import mujoco

from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    euler_from_quat,
)


class BipedalStanceController:
    """
    QP-based Whole-Body Controller for bipedal standing.

    Hard constraints:
        - Both feet remain fixed (zero spatial acceleration)

    Objective (weighted least-squares):
        - CoM tracks a horizontal target
        - Pelvis stays upright
        - Joint posture (lowest priority)

    ``mj_inverse`` maps the desired ``qacc`` to exact joint torques.
    """

    def __init__(self, env, config: dict):
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

        self.q_ref: np.ndarray | None = None
        self.com_target: np.ndarray | None = None
        self.pelvis_quat_ref: np.ndarray | None = None

        self._pelvis_bid = env._body_ids["pelvis"]
        self._left_bid = env._body_ids["left_foot"]
        self._right_bid = env._body_ids["right_foot"]

    def reset(self) -> None:
        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_target = compute_com_position(self.env.model, self.env.data)
        self.pelvis_quat_ref = self.env.get_pelvis_quat().copy()

    def compute(self) -> np.ndarray:
        model = self.env.model
        data = self.env.data
        nv = model.nv

        q = self.env.get_actuated_qpos()
        dq = self.env.get_actuated_qvel()

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

        # ---- Build QP --------------------------------------------------
        # Objective: minimise ||W*(A*qacc - b)||^2 + reg*||qacc||^2
        # Constraints: J_feet * qacc = 0  (hard)

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

        P = A_obj.T @ A_obj + self.reg * np.eye(nv)
        q_vec = -A_obj.T @ b_obj

        C_eq = np.vstack([J_left, J_right])
        nc = C_eq.shape[0]

        # KKT system for equality-constrained QP:
        #   [P   C_eq^T] [x]   [-q_vec]
        #   [C_eq   0  ] [λ] = [  0  ]
        KKT = np.zeros((nv + nc, nv + nc))
        KKT[:nv, :nv] = P
        KKT[:nv, nv:] = C_eq.T
        KKT[nv:, :nv] = C_eq

        rhs = np.zeros(nv + nc)
        rhs[:nv] = -q_vec

        sol = solve(KKT, rhs, assume_a='sym')
        qacc_des = sol[:nv]

        # ---- Inverse dynamics ------------------------------------------
        qacc_true = data.qacc.copy()
        data.qacc[:] = qacc_des
        mujoco.mj_inverse(model, data)
        ctrl = data.qfrc_inverse[6:].copy()
        data.qacc[:] = qacc_true
        return ctrl

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
