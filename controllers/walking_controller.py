"""Periodic walking controller for G1 on flat ground.

Builds on QPWBCController and cycles through alternating single-support
and double-support phases, advancing the CoM target and placing swing
feet forward step by step.
"""
import numpy as np
import mujoco

from env import G1Env
from controllers.base_qp_wbc import QPWBCController
from planners.com_planner import ComPlanner
from planners.swing_foot_planner import SwingFootPlanner
from planners.footstep_planner import FootstepPlanner
from utils.kinematics import compute_com_position, compute_com_velocity


class WalkingController(QPWBCController):
    """
    Gait scheduler + QP-WBC for periodic forward walking.

    States
    ------
    INIT :
        Both feet fixed, CoM held at bipedal midpoint.  Builds initial
        static stability before the first step.
    LEFT_SWING :
        Right foot is support; left foot lifts and swings forward.
    DOUBLE_SUPPORT :
        Both feet on ground, brief weight transfer between steps.
    RIGHT_SWING :
        Left foot is support; right foot lifts and swings forward.

    The state machine cycles LEFT_SWING → DOUBLE → RIGHT_SWING → DOUBLE
    indefinitely until the target forward displacement is reached.
    """

    def __init__(self, env: G1Env, config: dict):
        super().__init__(env, config)

        wcfg = config.get("walking", {})
        self.step_length = float(wcfg.get("step_length", 0.25))
        self.step_width = float(wcfg.get("step_width", 0.20))
        self.step_height = float(wcfg.get("step_height", 0.05))
        self.single_support_duration = float(wcfg.get("single_support_duration", 0.60))
        self.double_support_duration = float(wcfg.get("double_support_duration", 0.15))
        self.init_duration = float(wcfg.get("init_duration", 1.0))
        self.forward_direction = np.array(
            wcfg.get("forward_direction", [1.0, 0.0, 0.0])
        )
        self.forward_direction /= np.linalg.norm(self.forward_direction)

        # Bodies
        self._left_bid = env._body_ids["left_foot"]
        self._right_bid = env._body_ids["right_foot"]
        self._pelvis_bid = env._body_ids["pelvis"]

        # Planners (initialised in reset)
        self.footstep_planner: FootstepPlanner | None = None
        self.swing_planner: SwingFootPlanner | None = None
        self.com_planner: ComPlanner | None = None

        # Gait state
        self._state = "INIT"
        self._phase_start_time = 0.0
        self._step_count = 0
        self._total_displacement = 0.0

        # Cached foot positions at phase entry
        self._swing_foot_start: np.ndarray | None = None
        self._swing_target: np.ndarray | None = None
        self._support_foot_name: str | None = None
        self._swing_foot_name: str | None = None
        self._next_swing_is_left: bool = True

    def reset(self) -> None:
        model = self.env.model
        data = self.env.data

        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_start = compute_com_position(model, data)

        # Ground z from initial foot position
        left_pos = self.env.get_body_pos("left_foot")
        right_pos = self.env.get_body_pos("right_foot")
        self._ground_z = float(left_pos[2])

        # Planners
        self.footstep_planner = FootstepPlanner(
            self.step_length,
            self.step_width,
            self.forward_direction,
        )

        # Gait state
        self._state = "INIT"
        self._phase_start_time = 0.0
        self._step_count = 0
        self._total_displacement = 0.0
        self._next_swing_is_left = True

        self.swing_planner = None
        self.com_planner = None
        self._swing_foot_start = None
        self._swing_target = None
        self._support_foot_name = None
        self._swing_foot_name = None

    # ------------------------------------------------------------------ #
    # Gait state machine
    # ------------------------------------------------------------------ #

    def _update_gait_phase(self) -> None:
        """Advance the gait state machine based on timers."""
        t = self.env.data.time
        dt_phase = t - self._phase_start_time

        if self._state == "INIT" and dt_phase >= self.init_duration:
            self._enter_double_support(t)

        elif self._state == "LEFT_SWING" and dt_phase >= self.single_support_duration:
            self._enter_double_support(t)

        elif self._state == "DOUBLE_SUPPORT" and dt_phase >= self.double_support_duration:
            if self._next_swing_is_left:
                self._enter_left_swing(t)
            else:
                self._enter_right_swing(t)

        elif self._state == "RIGHT_SWING" and dt_phase >= self.single_support_duration:
            self._enter_double_support(t)

    def _enter_left_swing(self, t: float) -> None:
        self._state = "LEFT_SWING"
        self._phase_start_time = t
        self._support_foot_name = "right"
        self._swing_foot_name = "left"
        self._next_swing_is_left = False
        self._setup_swing_phase()

    def _enter_right_swing(self, t: float) -> None:
        self._state = "RIGHT_SWING"
        self._phase_start_time = t
        self._support_foot_name = "left"
        self._swing_foot_name = "right"
        self._next_swing_is_left = True
        self._setup_swing_phase()

    def _enter_double_support(self, t: float) -> None:
        self._state = "DOUBLE_SUPPORT"
        self._phase_start_time = t
        self._support_foot_name = None
        self._swing_foot_name = None
        self.swing_planner = None
        self._swing_foot_start = None
        self._swing_target = None
        self._step_count += 1

    def _setup_swing_phase(self) -> None:
        """Initialise swing foot planner and CoM planner for the new step."""
        support_pos = self.env.get_body_pos(f"{self._support_foot_name}_foot")
        swing_pos = self.env.get_body_pos(f"{self._swing_foot_name}_foot")

        is_left_swing = self._swing_foot_name == "left"
        self._swing_target = self.footstep_planner.plan_step(support_pos, is_left_swing)
        self._swing_foot_start = swing_pos.copy()

        self.swing_planner = SwingFootPlanner(
            self._swing_foot_start,
            self.step_height,
            0.4 * self.single_support_duration,  # rise_duration = 40% of swing
        )

        # CoM target: over the support foot, biased slightly forward
        com_target = np.array([
            support_pos[0] + 0.5 * self.step_length,
            support_pos[1],
            self.com_start[2],
        ])
        com_now = compute_com_position(self.env.model, self.env.data)
        self.com_planner = ComPlanner(com_now, com_target, self.single_support_duration)

    # ------------------------------------------------------------------ #
    # Main compute loop
    # ------------------------------------------------------------------ #

    def compute(self) -> np.ndarray:
        model = self.env.model
        data = self.env.data
        nv = model.nv
        t = data.time
        dt_phase = t - self._phase_start_time

        # ---- Update gait phase -----------------------------------------
        self._update_gait_phase()
        dt_phase = t - self._phase_start_time  # refresh after possible transition

        # ---- Common kinematics & dynamics ------------------------------
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

        # ---- CoM target ------------------------------------------------
        if self._state == "INIT":
            com_target = self.com_start
        elif self._state == "DOUBLE_SUPPORT":
            # Interpolate between previous and next support foot
            left_pos = self.env.get_body_pos("left_foot")
            right_pos = self.env.get_body_pos("right_foot")
            com_target = 0.5 * (left_pos + right_pos)
            com_target[2] = self.com_start[2]
        elif self.com_planner is not None:
            com_target, _, _ = self.com_planner.evaluate(dt_phase)
        else:
            com_target = com_pos.copy()

        # ---- Task targets ----------------------------------------------
        com_accel_des, cam_rate_des, joint_accel_des, J_cam = self._compute_task_targets(
            model, data, com_pos, com_target, self.q_ref, q, dq
        )

        # ---- Active feet & swing task ----------------------------------
        swing_task = None
        extra_tasks = None

        if self._state == "INIT":
            active_feet = [
                {"jacobian": J_left, "name": "left_foot"},
                {"jacobian": J_right, "name": "right_foot"},
            ]

        elif self._state == "DOUBLE_SUPPORT":
            # 6-D constraints with velocity damping on both feet.
            foot_kd = self.cfg["transition"]["foot_kd"]
            active_feet = [
                {
                    "jacobian": J_left,
                    "name": "left_foot",
                    "accel_offset": -foot_kd * (J_left @ data.qvel),
                },
                {
                    "jacobian": J_right,
                    "name": "right_foot",
                    "accel_offset": -foot_kd * (J_right @ data.qvel),
                },
            ]

        elif self._state in ("LEFT_SWING", "RIGHT_SWING"):
            # Support foot: hard contact with velocity damping
            if self._support_foot_name == "left":
                J_support = J_left
                J_swing = J_right
            else:
                J_support = J_right
                J_swing = J_left

            support_vel = J_support @ data.qvel
            support_accel_offset = -self.cfg["transition"]["foot_kd"] * support_vel
            active_feet = [
                {
                    "jacobian": J_support,
                    "name": f"{self._support_foot_name}_foot",
                    "accel_offset": support_accel_offset,
                },
            ]

            # Swing foot: z-only tracking
            if self.swing_planner is not None:
                swing_pos, swing_vel, swing_accel = self.swing_planner.evaluate(dt_phase)
                current_swing_pos = self.env.get_body_pos(f"{self._swing_foot_name}_foot")
                current_swing_vel = J_swing[:3] @ data.qvel

                swing_accel_des_z = (
                    swing_accel[2]
                    + self.cfg["transition"]["swing_kp"] * (swing_pos[2] - current_swing_pos[2])
                    + self.cfg["transition"]["swing_kd"] * (swing_vel[2] - current_swing_vel[2])
                )
                swing_task = {
                    "jacobian": J_swing[2:3],
                    "accel_des": np.array([swing_accel_des_z]),
                }

            # Pelvis orientation task
            pelvis_quat_des = np.array([1.0, 0.0, 0.0, 0.0])
            pelvis_quat_cur = self.env.get_pelvis_quat()
            pelvis_ang_err = self._quat_error(pelvis_quat_des, pelvis_quat_cur)

            J_pelvis_lin = np.zeros((3, nv))
            J_pelvis_ang = np.zeros((3, nv))
            mujoco.mj_jacBody(model, data, J_pelvis_lin, J_pelvis_ang, self._pelvis_bid)
            pelvis_omega = J_pelvis_ang @ data.qvel

            pelvis_accel_des = (
                -self.cfg["transition"]["pelvis_kp"] * pelvis_ang_err
                - self.cfg["transition"]["pelvis_kd"] * pelvis_omega
            )

            extra_tasks = [
                (self.cfg["transition"]["pelvis_weight"], J_pelvis_ang, pelvis_accel_des),
            ]

        # ---- Temporarily boost single-leg task weights -----------------
        if self._state in ("LEFT_SWING", "RIGHT_SWING"):
            old_w_com = self.w_com
            old_w_cam = self.w_cam
            old_w_posture = self.w_posture
            self.w_com = self.cfg["transition"]["single_leg_w_com"]
            self.w_cam = self.cfg["transition"]["single_leg_w_cam"]
            self.w_posture = self.cfg["transition"]["single_leg_w_posture"]
        else:
            old_w_com = old_w_cam = old_w_posture = None

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
            swing_task=swing_task,
            extra_tasks=extra_tasks,
        )

        # Restore weights
        if self._state in ("LEFT_SWING", "RIGHT_SWING"):
            self.w_com = old_w_com
            self.w_cam = old_w_cam
            self.w_posture = old_w_posture

        return tau

    @property
    def state(self) -> str:
        return self._state

    @property
    def phase_elapsed(self) -> float:
        return self.env.data.time - self._phase_start_time

    @property
    def step_count(self) -> int:
        return self._step_count
