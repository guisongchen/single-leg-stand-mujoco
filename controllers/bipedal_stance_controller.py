import numpy as np
import mujoco

from env import G1Env
from controllers.base_qp_wbc import QPWBCController
from utils.kinematics import compute_com_position


class BipedalStanceController(QPWBCController):
    """
    Full QP-based Whole-Body Controller for bipedal standing.

    Inherits shared QP infrastructure from :class:`QPWBCController`.
    Both feet are hard contact constraints; no swing task is used.
    """

    def __init__(self, env: G1Env, config: dict):
        super().__init__(env, config)
        self.pelvis_quat_ref = None

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

        mujoco.mj_fullM(model, self._M, data.qM)
        bias_force = data.qfrc_bias - data.qfrc_passive

        J_com = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(model, data, J_com, 0)

        J_left = np.zeros((6, nv))
        J_right = np.zeros((6, nv))
        mujoco.mj_jacBody(model, data, J_left[:3], J_left[3:], self._left_bid)
        mujoco.mj_jacBody(model, data, J_right[:3], J_right[3:], self._right_bid)

        com_pos = compute_com_position(model, data)

        com_accel_des, cam_rate_des, joint_accel_des, J_cam = self._compute_task_targets(
            model, data, com_pos, self.com_target, self.q_ref, q, dq
        )

        active_feet = [
            {"jacobian": J_left, "name": "left_foot"},
            {"jacobian": J_right, "name": "right_foot"},
        ]

        qacc_des, wrenches, tau = self._solve_qp(
            model=model,
            data=data,
            J_com=J_com,
            J_cam=J_cam,
            com_accel_des=com_accel_des,
            cam_rate_des=cam_rate_des,
            joint_accel_des=joint_accel_des,
            active_feet=active_feet,
            bias_force=bias_force,
        )

        return tau
