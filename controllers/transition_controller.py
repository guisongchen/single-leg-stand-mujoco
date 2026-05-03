import numpy as np
import mujoco

from env import G1Env
from controllers.base_qp_wbc import QPWBCController
from planners.com_planner import ComPlanner
from planners.swing_foot_planner import SwingFootPlanner
from utils.kinematics import compute_com_position, compute_contact_wrench


class TransitionController(QPWBCController):
    """
    State-machine controller for bipedal -> single-leg transition.

    States
    ------
    BIPEDAL :
        Both feet fixed. CoM held at the initial (bipedal) position.
    WEIGHT_SHIFT :
        Both feet remain fixed. CoM is smoothly moved from the bipedal
        midpoint to a point directly above the support foot. The state
        machine only advances to SINGLE_LEG once the swing foot is
        effectively unloaded and the CoM has settled.
    SINGLE_LEG :
        Only the support foot is a hard contact constraint. The swing foot
        is tracked as a soft objective (lifted and held at constant height).
        The posture reference is updated to the current pose at the moment
        of transition so the controller does not fight the natural lean.
    """

    def __init__(self, env: G1Env, config: dict):
        super().__init__(env, config)

        tcfg = config.get("transition", {})
        self.support_foot_name = tcfg.get("support_foot", "left")
        if self.support_foot_name not in ("left", "right"):
            raise ValueError(
                f"support_foot must be 'left' or 'right', got {self.support_foot_name}"
            )
        self.swing_foot_name = "right" if self.support_foot_name == "left" else "left"

        self.t_bipedal = float(tcfg.get("t_bipedal", 1.0))
        self.t_weight_shift = float(tcfg.get("t_weight_shift", 2.5))
        self.t_single_leg = float(tcfg.get("t_single_leg", 3.0))

        self.swing_lift_height = float(tcfg.get("swing_lift_height", 0.03))
        self.swing_rise_duration = float(tcfg.get("swing_rise_duration", 1.0))

        self.swing_kp = float(tcfg.get("swing_kp", 400.0))
        self.swing_kd = float(tcfg.get("swing_kd", 40.0))

        self.swing_unload_force = float(tcfg.get("swing_unload_force", 30.0))
        self.com_settle_tolerance = float(tcfg.get("com_settle_tolerance", 0.03))

        self._support_bid = env._body_ids[f"{self.support_foot_name}_foot"]
        self._swing_bid = env._body_ids[f"{self.swing_foot_name}_foot"]

        self._state = "BIPEDAL"
        self._phase_start_time = 0.0

        self.com_start = None
        self.com_target_single = None
        self.com_planner = None
        self.swing_foot_planner = None
        self.swing_foot_start = None

    def reset(self) -> None:
        model = self.env.model
        data = self.env.data

        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_start = compute_com_position(model, data)

        support_pos = self.env.get_body_pos(f"{self.support_foot_name}_foot")
        self.swing_foot_start = self.env.get_body_pos(f"{self.swing_foot_name}_foot")

        # CoM target for single-leg: over support foot, same height
        self.com_target_single = np.array([
            support_pos[0],
            support_pos[1],
            self.com_start[2],
        ])

        self.com_planner = ComPlanner(
            self.com_start,
            self.com_target_single,
            self.t_weight_shift,
        )

        self.swing_foot_planner = SwingFootPlanner(
            self.swing_foot_start,
            self.swing_lift_height,
            self.swing_rise_duration,
        )

        self._state = "BIPEDAL"
        self._phase_start_time = 0.0

    def _swing_fz(self) -> float:
        """MuJoCo contact force on the swing foot (vertical, world frame)."""
        body_name = self.env.cfg["robot"]["body_names"][f"{self.swing_foot_name}_foot"]
        force = compute_contact_wrench(self.env.model, self.env.data, body_name)
        return force[2]

    def compute(self) -> np.ndarray:
        model = self.env.model
        data = self.env.data
        nv = model.nv
        t = data.time
        dt_phase = t - self._phase_start_time

        # ---- State transitions -----------------------------------------
        if self._state == "BIPEDAL" and dt_phase >= self.t_bipedal:
            self._state = "WEIGHT_SHIFT"
            self._phase_start_time = t
            dt_phase = 0.0

        elif self._state == "WEIGHT_SHIFT":
            com_now = compute_com_position(model, data)
            com_err = np.linalg.norm(com_now[:2] - self.com_target_single[:2])
            swing_fz = self._swing_fz()
            # Switch as soon as the swing foot is unloaded and the CoM is
            # reasonably close to the single-leg target.  Do NOT wait for
            # the full timer — keeping both feet fixed while the CoM shifts
            # all the way to the support foot pushes the robot against its
            # kinematic limits and makes the QP infeasible.
            ready = (
                swing_fz < self.swing_unload_force
                and com_err < self.com_settle_tolerance
            )
            # Safety cap: if we somehow never meet the criteria, force-switch
            # after a long timeout so the test does not hang.
            if ready or dt_phase >= 5.0:
                self._state = "SINGLE_LEG"
                self._phase_start_time = t
                # Capture the current lean pose so the posture task
                # does not pull the robot back toward the bipedal reference.
                self.q_ref = self.env.get_actuated_qpos().copy()
                dt_phase = 0.0

        # ---- Common kinematics & dynamics ------------------------------
        q = self.env.get_actuated_qpos()
        dq = self.env.get_actuated_qvel()

        mujoco.mj_fullM(model, self._M, data.qM)
        bias_force = data.qfrc_bias - data.qfrc_passive

        J_com = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(model, data, J_com, 0)

        J_support = np.zeros((6, nv))
        J_swing = np.zeros((6, nv))
        mujoco.mj_jacBody(
            model, data, J_support[:3], J_support[3:], self._support_bid
        )
        mujoco.mj_jacBody(
            model, data, J_swing[:3], J_swing[3:], self._swing_bid
        )

        com_pos = compute_com_position(model, data)

        # ---- CoM target ------------------------------------------------
        if self._state == "BIPEDAL":
            com_target = self.com_start
        elif self._state == "WEIGHT_SHIFT":
            com_target, _, _ = self.com_planner.evaluate(dt_phase)
        else:  # SINGLE_LEG
            com_target = self.com_target_single

        # ---- Task targets ----------------------------------------------
        com_accel_des, cam_rate_des, joint_accel_des, J_cam = self._compute_task_targets(
            model, data, com_pos, com_target, self.q_ref, q, dq
        )

        # ---- Active feet & optional swing task -------------------------
        if self._state in ("BIPEDAL", "WEIGHT_SHIFT"):
            active_feet = [
                {"jacobian": J_support, "name": f"{self.support_foot_name}_foot"},
                {"jacobian": J_swing, "name": f"{self.swing_foot_name}_foot"},
            ]
            swing_task = None
        else:  # SINGLE_LEG
            active_feet = [
                {"jacobian": J_support, "name": f"{self.support_foot_name}_foot"},
            ]

            swing_pos, swing_vel, swing_accel = self.swing_foot_planner.evaluate(
                dt_phase
            )
            current_swing_pos = self.env.get_body_pos(f"{self.swing_foot_name}_foot")
            current_swing_vel = J_swing[:3] @ data.qvel

            swing_accel_des = (
                swing_accel
                + self.swing_kp * (swing_pos - current_swing_pos)
                + self.swing_kd * (swing_vel - current_swing_vel)
            )

            swing_task = {
                "jacobian": J_swing[:3],
                "accel_des": swing_accel_des,
            }

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
        )

        return tau

    @property
    def state(self) -> str:
        return self._state

    @property
    def phase_elapsed(self) -> float:
        return self.env.data.time - self._phase_start_time
