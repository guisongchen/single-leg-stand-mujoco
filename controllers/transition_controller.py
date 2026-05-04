import numpy as np
import mujoco

from env import G1Env
from controllers.base_qp_wbc import QPWBCController
from planners.com_planner import ComPlanner
from planners.swing_foot_planner import SwingFootPlanner
from utils.kinematics import compute_com_position, compute_com_velocity, compute_contact_wrench, euler_from_quat


class TransitionController(QPWBCController):
    """
    State-machine controller for bipedal -> single-leg transition.

    States
    ------
    BIPEDAL :
        Both feet fixed. CoM held at the initial (bipedal) position.
    WEIGHT_SHIFT :
        Both feet remain fixed. CoM is smoothly moved from the bipedal
        midpoint toward the support foot.  The state machine advances to
        SINGLE_LEG once the swing foot is effectively unloaded.
    SINGLE_LEG :
        Only the support foot is a hard contact constraint with velocity
        damping to prevent slip.  The swing foot is tracked as a soft
        objective.  A pelvis-orientation task keeps the torso upright.
        Posture weight is reduced so the robot can adopt the natural lean.
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

        self.swing_unload_force = float(tcfg.get("swing_unload_force", 20.0))
        self.com_settle_tolerance = float(tcfg.get("com_settle_tolerance", 0.05))

        # Single-leg tuning
        self.foot_kd = float(tcfg.get("foot_kd", 40.0))
        self.support_fallback_kp = float(tcfg.get("support_fallback_kp", 400.0))
        self.support_fallback_kd = float(tcfg.get("support_fallback_kd", 40.0))
        self.support_fallback_weight = float(tcfg.get("support_fallback_weight", 500.0))
        self.pelvis_kp = float(tcfg.get("pelvis_kp", 200.0))
        self.pelvis_kd = float(tcfg.get("pelvis_kd", 20.0))
        self.pelvis_weight = float(tcfg.get("pelvis_weight", 50.0))
        self.single_leg_w_com = float(tcfg.get("single_leg_w_com", 200.0))
        self.single_leg_w_cam = float(tcfg.get("single_leg_w_cam", 200.0))
        self.single_leg_w_posture = float(tcfg.get("single_leg_w_posture", 0.01))

        self._support_bid = env._body_ids[f"{self.support_foot_name}_foot"]
        self._swing_bid = env._body_ids[f"{self.swing_foot_name}_foot"]
        self._pelvis_bid = env._body_ids["pelvis"]

        self._state = "BIPEDAL"
        self._phase_start_time = 0.0

        self.com_start = None
        self.com_target_single = None
        self.com_planner = None
        self.swing_foot_planner = None
        self.swing_foot_start = None

        # Capture-point switching logic
        self.cp_margin = float(tcfg.get("cp_margin", 0.02))
        self.weight_shift_timeout = float(tcfg.get("weight_shift_timeout", 15.0))

    def reset(self) -> None:
        model = self.env.model
        data = self.env.data

        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_start = compute_com_position(model, data)

        support_pos = self.env.get_body_pos(f"{self.support_foot_name}_foot")
        self._support_foot_ground_z = support_pos[2]
        self.swing_foot_start = self.env.get_body_pos(f"{self.swing_foot_name}_foot")

        # CoM target for single-leg: the geometric centre of the support
        # foot's contact rectangle, expressed in world frame. The 4 corner
        # spheres span x in [-cop_x_back, +cop_x_forward] in foot frame, so
        # the rectangle centre is offset (cop_x_forward - cop_x_back)/2
        # forward of the ankle joint origin. Targeting the centre maximises
        # CoP margin in every direction; it also makes the QP feasible
        # without any inward bias.
        foot_center_x_local = 0.5 * (self.foot_cop_x_forward - self.foot_cop_x_back)
        support_R = data.xmat[self._support_bid].reshape(3, 3)
        foot_center_world = support_pos + support_R @ np.array(
            [foot_center_x_local, 0.0, 0.0]
        )
        self.com_target_single = np.array([
            foot_center_world[0],
            foot_center_world[1],
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

    def _compute_capture_point(self, com_pos: np.ndarray, com_vel: np.ndarray) -> np.ndarray:
        """Instantaneous capture point in world XY plane.

        CP = com_xy + com_vxy / omega, where omega = sqrt(g / com_z).
        If the CP lies inside the support foot polygon, the robot can
        theoretically recover to rest without taking a step.
        """
        g = abs(self.env.model.opt.gravity[2])
        z_com = max(com_pos[2], 0.1)  # guard against div-by-zero
        omega = np.sqrt(g / z_com)
        return com_pos[:2] + com_vel[:2] / omega

    def _is_cp_inside_support(self, cp: np.ndarray) -> bool:
        """Check whether the capture point is inside the support foot rectangle."""
        support_pos = self.env.get_body_pos(f"{self.support_foot_name}_foot")
        support_R = self.env.data.xmat[self._support_bid].reshape(3, 3)
        cp_local = support_R.T @ np.append(cp - support_pos[:2], 0.0)
        margin = self.cp_margin
        return (
            -self.foot_cop_x_back + margin <= cp_local[0] <= self.foot_cop_x_forward - margin
            and -self.foot_cop_y_half + margin <= cp_local[1] <= self.foot_cop_y_half - margin
        )

    def _is_cp_inside_combined_polygon(self, cp: np.ndarray) -> bool:
        """Check whether CP is inside the combined support polygon of both feet.

        During WEIGHT_SHIFT both feet are still on the ground; the relevant
        stability region is the convex hull of both contact rectangles.
        """
        left_pos = self.env.get_body_pos("left_foot")
        right_pos = self.env.get_body_pos("right_foot")
        left_R = self.env.data.xmat[self._left_bid].reshape(3, 3)
        right_R = self.env.data.xmat[self._right_bid].reshape(3, 3)

        # Transform CP into each foot frame
        cp_left = left_R.T @ np.append(cp - left_pos[:2], 0.0)
        cp_right = right_R.T @ np.append(cp - right_pos[:2], 0.0)

        def _inside_foot(cp_local):
            return (
                -self.foot_cop_x_back <= cp_local[0] <= self.foot_cop_x_forward
                and -self.foot_cop_y_half <= cp_local[1] <= self.foot_cop_y_half
            )

        return _inside_foot(cp_left) or _inside_foot(cp_right)

    def _swing_fz(self) -> float:
        """MuJoCo contact force on the swing foot (vertical, world frame)."""
        body_name = self.env.cfg["robot"]["body_names"][f"{self.swing_foot_name}_foot"]
        force = compute_contact_wrench(self.env.model, self.env.data, body_name)
        return force[2]

    def _support_fz(self) -> float:
        """MuJoCo contact force on the support foot (vertical, world frame)."""
        body_name = self.env.cfg["robot"]["body_names"][f"{self.support_foot_name}_foot"]
        force = compute_contact_wrench(self.env.model, self.env.data, body_name)
        return force[2]

    def _swing_wrench_from_mujoco(self) -> np.ndarray:
        """6-D contact wrench on the swing foot (world frame, body origin)."""
        model = self.env.model
        data = self.env.data
        bid = self._swing_bid
        cfrc = data.cfrc_ext[bid].copy()
        force = cfrc[:3]
        tau_com = cfrc[3:]
        R = data.ximat[bid, :9].reshape(3, 3)
        r_com_body = R @ model.body_ipos[bid, :]
        tau_origin = tau_com + np.cross(r_com_body, force)
        return np.hstack([force, tau_origin])

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
            com_vel = compute_com_velocity(model, data)
            com_err = np.linalg.norm(com_now[:2] - self.com_target_single[:2])
            swing_fz = self._swing_fz()

            # Capture-point safety gate: CP must be inside the *combined*
            # support polygon of both feet.  During weight shift the robot
            # is still on two feet; checking against the support foot alone
            # would block the natural handoff because the CP often sits near
            # the edge of the support foot while the swing foot is still
            # contributing.
            cp = self._compute_capture_point(com_now, com_vel)
            cp_safe = self._is_cp_inside_combined_polygon(cp)

            ready = (
                swing_fz < self.swing_unload_force
                and com_err < self.com_settle_tolerance
                and cp_safe
            )
            if ready or dt_phase >= self.weight_shift_timeout:
                self._state = "SINGLE_LEG"
                self._phase_start_time = t
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
        swing_task = None
        extra_tasks = None

        if self._state == "BIPEDAL":
            active_feet = [
                {"jacobian": J_support, "name": f"{self.support_foot_name}_foot"},
                {"jacobian": J_swing, "name": f"{self.swing_foot_name}_foot"},
            ]
        elif self._state == "WEIGHT_SHIFT":
            # Both feet stay in the QP, but the swing foot gets a gentle
            # upward acceleration offset as it unloads.  This prevents the
            # QP from fighting an infeasible bilateral constraint once
            # fz -> 0, which was the source of the late-phase chatter.
            swing_fz_now = self._swing_fz()
            unload_frac = np.clip(1.0 - swing_fz_now / 80.0, 0.0, 1.0)
            a_lift = 1.0 * unload_frac
            active_feet = [
                {"jacobian": J_support, "name": f"{self.support_foot_name}_foot"},
                {
                    "jacobian": J_swing,
                    "name": f"{self.swing_foot_name}_foot",
                    "accel_offset": np.array([0.0, 0.0, a_lift, 0.0, 0.0, 0.0]),
                },
            ]
        else:  # SINGLE_LEG
            support_contact_fz = self._support_fz()

            if support_contact_fz < 5.0:
                # Support foot lost contact -- degrade to soft tracking
                support_body_pos = self.env.get_body_pos(
                    f"{self.support_foot_name}_foot"
                )
                target_pos = np.array([
                    support_body_pos[0],
                    support_body_pos[1],
                    self._support_foot_ground_z,
                ])
                support_vel = J_support[:3] @ data.qvel
                support_accel_des = (
                    self.support_fallback_kp * (target_pos - support_body_pos)
                    + self.support_fallback_kd * (-support_vel)
                )
                active_feet = []
                support_fallback_task = {
                    "jacobian": J_support[:3],
                    "accel_des": support_accel_des,
                }
            else:
                # Full 6-D support contact: the foot's 4 corner spheres can
                # produce ground reaction torques up to the CoP envelope
                # encoded in base_qp_wbc._build_wrench_cones. Velocity
                # damping is applied to all 6 components to suppress slip
                # and rotation drift.
                support_vel = J_support @ data.qvel
                support_accel_offset = -self.foot_kd * support_vel
                active_feet = [
                    {
                        "jacobian": J_support,
                        "name": f"{self.support_foot_name}_foot",
                        "accel_offset": support_accel_offset,
                    },
                ]
                support_fallback_task = None

            # Swing foot tracking: vertical only.  Tracking the full xy
            # position couples the swing leg to whatever the body has to
            # do for balance -- the QP fights the natural lateral drift of
            # the foot, the reaction wrenches feed back into the body, and
            # a slow oscillation builds up over a few seconds.  All we
            # really want is for the foot to lift clear of the ground; let
            # the rest of the IK be a free parameter.
            swing_pos, swing_vel, swing_accel = self.swing_foot_planner.evaluate(
                dt_phase
            )
            current_swing_pos = self.env.get_body_pos(f"{self.swing_foot_name}_foot")
            current_swing_vel = J_swing[:3] @ data.qvel

            swing_accel_des_z = (
                swing_accel[2]
                + self.swing_kp * (swing_pos[2] - current_swing_pos[2])
                + self.swing_kd * (swing_vel[2] - current_swing_vel[2])
            )

            swing_task = {
                "jacobian": J_swing[2:3],
                "accel_des": np.array([swing_accel_des_z]),
            }

            # Pelvis orientation task (keep torso upright)
            pelvis_quat_des = np.array([1.0, 0.0, 0.0, 0.0])
            pelvis_quat_cur = self.env.get_pelvis_quat()
            pelvis_ang_err = self._quat_error(pelvis_quat_des, pelvis_quat_cur)

            J_pelvis_lin = np.zeros((3, nv))
            J_pelvis_ang = np.zeros((3, nv))
            mujoco.mj_jacBody(
                model, data, J_pelvis_lin, J_pelvis_ang, self._pelvis_bid
            )
            pelvis_omega = J_pelvis_ang @ data.qvel

            pelvis_accel_des = (
                -self.pelvis_kp * pelvis_ang_err
                - self.pelvis_kd * pelvis_omega
            )

            extra_tasks = [
                (self.pelvis_weight, J_pelvis_ang, pelvis_accel_des),
            ]
            if support_fallback_task is not None:
                extra_tasks.append(
                    (
                        self.support_fallback_weight,
                        support_fallback_task["jacobian"],
                        support_fallback_task["accel_des"],
                    )
                )

        # ---- Temporarily boost single-leg task weights -----------------
        if self._state == "SINGLE_LEG":
            old_w_com = self.w_com
            old_w_cam = self.w_cam
            old_w_posture = self.w_posture
            self.w_com = self.single_leg_w_com
            self.w_cam = self.single_leg_w_cam
            self.w_posture = self.single_leg_w_posture
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
        if self._state == "SINGLE_LEG":
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
