"""Periodic walking controller for G1 on flat ground.

Builds on QPWBCController and cycles through a 5-phase FSM with
GRF-based transitions, following the architecture in
``docs/walking_architecture.md`` and ``docs/adr/001-walking-gait-architecture.md``.

Phases
------
BIPEDAL_INIT      → WEIGHT_SHIFT_L  (timer: 0.1 s)
WEIGHT_SHIFT_L    → LEFT_SINGLE     (GRF hysteresis: 50 % arm / 80 % fire)
LEFT_SINGLE       → DOUBLE_SUPPORT  (touchdown: z + xy + GRF)
DOUBLE_SUPPORT    → WEIGHT_SHIFT_R  (timer: 0.15 s)
WEIGHT_SHIFT_R    → RIGHT_SINGLE    (GRF hysteresis)
RIGHT_SINGLE      → DOUBLE_SUPPORT  (touchdown)
DOUBLE_SUPPORT    → WEIGHT_SHIFT_L  (timer: 0.15 s)  … repeat
"""
from __future__ import annotations

import numpy as np
import mujoco

from env import G1Env
from controllers.base_qp_wbc import QPWBCController
from planners.footstep_planner import FootstepPlanner
from planners.swing_foot_planner import SwingTrajectoryPlanner, SwingFootPlanner
from planners.com_planner import ComPlanner
from utils.kinematics import (
    compute_com_position,
    compute_com_velocity,
    compute_capture_point,
    euler_from_quat,
)

# --------------------------------------------------------------------------- #
# Phase constants
# --------------------------------------------------------------------------- #
BIPEDAL_INIT = "BIPEDAL_INIT"
WEIGHT_SHIFT_L = "WEIGHT_SHIFT_L"
LEFT_SINGLE = "LEFT_SINGLE"
DOUBLE_SUPPORT = "DOUBLE_SUPPORT"
WEIGHT_SHIFT_R = "WEIGHT_SHIFT_R"
RIGHT_SINGLE = "RIGHT_SINGLE"


class WalkingController(QPWBCController):
    """
    Gait scheduler + QP-WBC for periodic forward walking.

    Parameters
    ----------
    env : G1Env
    config : dict
        Must contain ``walking``, ``transitions``, ``safety``,
        ``swing_weights``, and ``pelvis_orientation`` sections.
    """

    def __init__(self, env: G1Env, config: dict):
        super().__init__(env, config)

        # ---- walking parameters ------------------------------------------
        wcfg = config.get("walking", {})
        self.step_length = float(wcfg.get("step_length", 0.25))
        self.step_width = float(wcfg.get("step_width", 0.20))
        self.step_height = float(wcfg.get("step_height", 0.05))
        self.min_single_duration = float(wcfg.get("min_single_duration", 0.50))
        self.init_duration = float(wcfg.get("init_duration", 0.1))
        self.double_support_duration = float(wcfg.get("double_support_duration", 0.15))
        self.phase_timeout = float(wcfg.get("phase_timeout", 5.0))
        self.double_support_com_bias = float(wcfg.get("double_support_com_bias", 0.70))

        fwd = wcfg.get("forward_direction", [1.0, 0.0])
        self.forward_direction = np.array([fwd[0], fwd[1], 0.0])
        self.forward_direction /= np.linalg.norm(self.forward_direction)

        # ---- transition thresholds ---------------------------------------
        tcfg = config.get("transitions", {})
        self.grf_arm_threshold = float(tcfg.get("grf_arm_threshold", 0.50))
        self.grf_fire_threshold = float(tcfg.get("grf_fire_threshold", 0.80))
        self.grf_arm_to_fire_delay = float(tcfg.get("grf_arm_to_fire_delay", 0.05))
        self.grf_touchdown_threshold = float(tcfg.get("grf_touchdown_threshold", 0.10))
        self.grf_liftoff_threshold = float(tcfg.get("grf_liftoff_threshold", 5.0))
        self.cp_abort_margin = float(tcfg.get("cp_abort_margin", 0.05))

        # ---- safety tolerances -------------------------------------------
        scfg = config.get("safety", {})
        self.touchdown_xy_tolerance = float(scfg.get("touchdown_xy_tolerance", 0.02))
        self.touchdown_z_tolerance = float(scfg.get("touchdown_z_tolerance", 0.005))

        # ---- swing weights -----------------------------------------------
        swcfg = config.get("swing_weights", {})
        self.swing_xy_early = float(swcfg.get("xy_early", 0.10))
        self.swing_xy_late = float(swcfg.get("xy_late", 1.0))
        self.swing_z_weight = float(swcfg.get("z", 1.0))

        # ---- pelvis orientation ------------------------------------------
        pcfg = config.get("pelvis_orientation", {})
        self.pelvis_roll_weight = float(pcfg.get("roll_weight", 0.5))
        self.pelvis_pitch_weight = float(pcfg.get("pitch_weight", 0.5))
        self.pelvis_yaw_weight = float(pcfg.get("yaw_weight", 0.3))
        self.pelvis_target_yaw = float(pcfg.get("target_yaw", 0.0))

        # ---- body IDs ----------------------------------------------------
        self._left_bid = env._body_ids["left_foot"]
        self._right_bid = env._body_ids["right_foot"]
        self._pelvis_bid = env._body_ids["pelvis"]

        # ---- cached state (initialised in reset) -------------------------
        self.footstep_planner: FootstepPlanner | None = None
        self.swing_planner: SwingTrajectoryPlanner | None = None

        self._phase: str = BIPEDAL_INIT
        self._phase_start_time: float = 0.0
        self._step_count: int = 0
        self._total_displacement: float = 0.0

        self._swing_foot_start: np.ndarray | None = None
        self._swing_target: np.ndarray | None = None
        self._support_foot_name: str | None = None
        self._swing_foot_name: str | None = None
        self._next_support_name: str | None = None

        # GRF hysteresis state
        self._grf_armed: bool = False
        self._grf_arm_time: float = 0.0

        # Smooth CoM interpolation during weight shift
        self._com_planner: ComPlanner | None = None

        # Fixed CoM target during single support (set at entry, not updated)
        self._single_support_com_target: np.ndarray | None = None

        # Foot geometry for torsional friction (metres)
        self._foot_width = 2.0 * self.foot_cop_y_half
        self._foot_length = self.foot_cop_x_back + self.foot_cop_x_forward

    # ------------------------------------------------------------------ #
    # Reset
    # ------------------------------------------------------------------ #

    def reset(self) -> None:
        model = self.env.model
        data = self.env.data

        self.q_ref = self.env.get_actuated_qpos().copy()
        self.com_start = compute_com_position(model, data)

        left_pos = self.env.get_body_pos("left_foot")
        right_pos = self.env.get_body_pos("right_foot")
        self._ground_z = float(left_pos[2])

        self.footstep_planner = FootstepPlanner(
            self.step_length,
            self.step_width,
        )

        self._phase = BIPEDAL_INIT
        self._phase_start_time = 0.0
        self._step_count = 0
        self._total_displacement = 0.0

        self.swing_planner = None
        self._swing_foot_start = None
        self._swing_target = None
        self._support_foot_name = None
        self._swing_foot_name = None
        self._next_support_name = "left"  # first weight shift is to left

        self._grf_armed = False
        self._grf_arm_time = 0.0

    # ------------------------------------------------------------------ #
    # Main compute loop
    # ------------------------------------------------------------------ #

    def compute(self) -> np.ndarray:
        model = self.env.model
        data = self.env.data
        nv = model.nv
        t = data.time

        # ---- Update gait phase -----------------------------------------
        self._update_gait_phase()
        dt_phase = t - self._phase_start_time

        # ---- Phase timeout fallback -------------------------------------
        if dt_phase > self.phase_timeout:
            self._emergency_bipedal_stance()
            dt_phase = t - self._phase_start_time

        # ---- Common kinematics & dynamics -------------------------------
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

        # ---- CoM target -------------------------------------------------
        com_target = self._compute_com_target(com_pos)
        self.com_target = com_target

        # ---- Task targets -----------------------------------------------
        com_accel_des, cam_rate_des, joint_accel_des, J_cam = self._compute_task_targets(
            model, data, com_pos, com_target, self.q_ref, q, dq
        )

        # ---- Active feet & swing task -----------------------------------
        active_feet, swing_tasks, extra_tasks = self._build_phase_tasks(
            dt_phase, J_left, J_right
        )

        # ---- Temporarily boost single-support task weights -------------
        old_weights = None
        if self._phase in (LEFT_SINGLE, RIGHT_SINGLE):
            old_weights = {
                "w_com": self.w_com,
                "w_cam": self.w_cam,
                "w_posture": self.w_posture,
            }
            self.w_com = self.cfg.get("transition", {}).get("single_leg_w_com", 200.0)
            self.w_cam = self.cfg.get("transition", {}).get("single_leg_w_cam", 200.0)
            self.w_posture = self.cfg.get("transition", {}).get("single_leg_w_posture", 0.01)

        # ---- QP solve ---------------------------------------------------
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
            swing_task=None,  # we fold swing into extra_tasks
            extra_tasks=(extra_tasks + swing_tasks) if (extra_tasks or swing_tasks) else None,
        )

        # Restore weights
        if old_weights is not None:
            self.w_com = old_weights["w_com"]
            self.w_cam = old_weights["w_cam"]
            self.w_posture = old_weights["w_posture"]

        return tau

    # ------------------------------------------------------------------ #
    # Gait state machine
    # ------------------------------------------------------------------ #

    def _update_gait_phase(self) -> None:
        """Evaluate all transition conditions and advance the FSM."""
        t = self.env.data.time
        dt_phase = t - self._phase_start_time

        if self._phase == BIPEDAL_INIT and dt_phase >= self.init_duration:
            self._enter_weight_shift(t, "left")

        elif self._phase == WEIGHT_SHIFT_L:
            if self._check_weight_shift_to_single("left", "right"):
                self._enter_single_support(t, "left", "right")

        elif self._phase == LEFT_SINGLE:
            if self._check_touchdown("right", self._swing_target):
                self._enter_double_support(t)

        elif self._phase == DOUBLE_SUPPORT and dt_phase >= self.double_support_duration:
            if self._next_support_name == "left":
                self._enter_weight_shift(t, "left")
            else:
                self._enter_weight_shift(t, "right")

        elif self._phase == WEIGHT_SHIFT_R:
            if self._check_weight_shift_to_single("right", "left"):
                self._enter_single_support(t, "right", "left")

        elif self._phase == RIGHT_SINGLE:
            if self._check_touchdown("left", self._swing_target):
                self._enter_double_support(t)

    def _enter_weight_shift(self, t: float, support_foot: str) -> None:
        """Enter a weight-shift phase toward the given support foot."""
        self._phase = WEIGHT_SHIFT_L if support_foot == "left" else WEIGHT_SHIFT_R
        self._phase_start_time = t
        self._support_foot_name = support_foot
        self._swing_foot_name = "right" if support_foot == "left" else "left"
        self._next_support_name = "right" if support_foot == "left" else "left"
        self._grf_armed = False
        self._grf_arm_time = 0.0
        self._single_support_com_target = None

        # Smooth CoM interpolation from current position to support foot centre
        com_now = compute_com_position(self.env.model, self.env.data)
        foot_pos = self.env.get_body_pos(f"{support_foot}_foot")
        foot_bid = self._left_bid if support_foot == "left" else self._right_bid
        foot_target = self._foot_center_world(foot_pos, foot_bid)
        foot_target[2] = self.com_start[2]
        shift_duration = self.cfg.get("transition", {}).get("t_weight_shift", 2.5)
        self._com_planner = ComPlanner(com_now, foot_target, shift_duration)

    def _enter_single_support(self, t: float, support_foot: str, swing_foot: str) -> None:
        """Enter single support with the given support foot."""
        self._phase = LEFT_SINGLE if support_foot == "left" else RIGHT_SINGLE
        self._phase_start_time = t
        self._support_foot_name = support_foot
        self._swing_foot_name = swing_foot
        self._grf_armed = False
        self._grf_arm_time = 0.0
        self._com_planner = None

        # Fix the CoM target at the support foot CoP centre so the controller
        # does not chase a slipping foot.
        foot_pos = self.env.get_body_pos(f"{support_foot}_foot")
        foot_bid = self._left_bid if support_foot == "left" else self._right_bid
        com_target = self._foot_center_world(foot_pos, foot_bid)
        com_target[2] = self.com_start[2]
        self._single_support_com_target = com_target

        # Snapshot the current actuated pose as the reference posture for
        # single support.  Using the initial upright q_ref fights the leaned
        # configuration that the robot adopts during weight shift.
        self.q_ref = self.env.get_actuated_qpos().copy()

        self._setup_swing_phase()

    def _enter_double_support(self, t: float) -> None:
        """Enter double support after a swing touchdown."""
        self._phase = DOUBLE_SUPPORT
        self._phase_start_time = t
        self._support_foot_name = None
        self._swing_foot_name = None
        self.swing_planner = None
        self._swing_foot_start = None
        self._swing_target = None
        self._single_support_com_target = None
        self._step_count += 1

    def _emergency_bipedal_stance(self) -> None:
        """Graceful degradation: both feet fixed, CoM centred."""
        self._phase = BIPEDAL_INIT
        self._phase_start_time = self.env.data.time
        self._support_foot_name = None
        self._swing_foot_name = None
        self.swing_planner = None
        self._swing_foot_start = None
        self._swing_target = None

    # ------------------------------------------------------------------ #
    # Task builders per phase
    # ------------------------------------------------------------------ #

    def _build_phase_tasks(
        self,
        dt_phase: float,
        J_left: np.ndarray,
        J_right: np.ndarray,
    ) -> tuple[list[dict], list[tuple[float, np.ndarray, np.ndarray]], list[tuple[float, np.ndarray, np.ndarray]] | None]:
        """
        Return (active_feet, swing_tasks, extra_tasks) for the current phase.

        active_feet : list[dict]
            Each dict has ``jacobian``, ``name``, and optional ``accel_offset``.
        swing_tasks : list[tuple[float, np.ndarray, np.ndarray]]
            List of (weight, Jacobian, accel_des) for swing foot tracking.
        extra_tasks : list[tuple[float, np.ndarray, np.ndarray]] | None
            Pelvis orientation and other soft objectives.
        """
        data = self.env.data
        active_feet: list[dict] = []
        swing_tasks: list[tuple[float, np.ndarray, np.ndarray]] = []
        extra_tasks: list[tuple[float, np.ndarray, np.ndarray]] = []

        if self._phase == BIPEDAL_INIT:
            # Velocity damping prevents foot slip that was causing
            # ill-conditioned Jacobians after returning from single-leg.
            foot_kd = self.cfg.get("transition", {}).get("foot_kd", 40.0)
            left_vel = J_left @ data.qvel
            right_vel = J_right @ data.qvel
            active_feet = [
                {"jacobian": J_left, "name": "left_foot", "accel_offset": -foot_kd * left_vel},
                {"jacobian": J_right, "name": "right_foot", "accel_offset": -foot_kd * right_vel},
            ]

        elif self._phase in (WEIGHT_SHIFT_L, WEIGHT_SHIFT_R):
            # Both feet stay in the QP, but the swing foot gets a gentle
            # upward acceleration offset as it unloads.  The support foot
            # is a pure hard constraint (no damping) so weight can transfer
            # naturally.  This matches the TransitionController exactly.
            swing_fz_now = self._compute_grf(f"{self._swing_foot_name}_foot")
            unload_frac = np.clip(1.0 - swing_fz_now / 80.0, 0.0, 1.0)
            a_lift = 1.0 * unload_frac

            if self._swing_foot_name == "left":
                active_feet = [
                    {"jacobian": J_left, "name": "left_foot", "accel_offset": np.array([0.0, 0.0, a_lift, 0.0, 0.0, 0.0])},
                    {"jacobian": J_right, "name": "right_foot"},
                ]
            else:
                active_feet = [
                    {"jacobian": J_left, "name": "left_foot"},
                    {"jacobian": J_right, "name": "right_foot", "accel_offset": np.array([0.0, 0.0, a_lift, 0.0, 0.0, 0.0])},
                ]
            # Pelvis orientation task stabilises the trunk while weight
            # transfers; without it the robot rotates freely and the fixed-foot
            # constraints become infeasible.
            extra_tasks = self._build_pelvis_orientation_task()

        elif self._phase in (LEFT_SINGLE, RIGHT_SINGLE):
            # Support foot hard, swing foot soft tracking
            if self._support_foot_name == "left":
                J_support = J_left
                J_swing = J_right
            else:
                J_support = J_right
                J_swing = J_left

            support_vel = J_support @ data.qvel
            active_feet = [
                {
                    "jacobian": J_support,
                    "name": f"{self._support_foot_name}_foot",
                    "accel_offset": -self.cfg.get("transition", {}).get("foot_kd", 40.0) * support_vel,
                },
            ]

            # Swing foot trajectory
            if self.swing_planner is not None:
                swing_pos_traj, swing_vel_traj, swing_accel_traj = self.swing_planner.evaluate(dt_phase)
                current_swing_pos = self.env.get_body_pos(f"{self._swing_foot_name}_foot")
                current_swing_vel = J_swing[:3] @ data.qvel

                swing_kp = self.cfg.get("transition", {}).get("swing_kp", 50.0)
                swing_kd = self.cfg.get("transition", {}).get("swing_kd", 15.0)

                swing_accel_des = (
                    swing_accel_traj
                    + swing_kp * (swing_pos_traj - current_swing_pos)
                    + swing_kd * (swing_vel_traj - current_swing_vel)
                )

                # Swing tracking: z-only for static alternating support;
                # xy is free to move with the body to avoid trunk disturbance.
                phase_progress = min(dt_phase / self.min_single_duration, 1.0)
                _, z_weight = self._compute_swing_weights(phase_progress)

                swing_tasks.append((z_weight, J_swing[2:3], swing_accel_des[2:3]))

            extra_tasks = self._build_pelvis_orientation_task()

        elif self._phase == DOUBLE_SUPPORT:
            # Both feet 6-D with velocity damping
            foot_kd = self.cfg.get("transition", {}).get("foot_kd", 40.0)
            left_vel = J_left @ data.qvel
            right_vel = J_right @ data.qvel
            active_feet = [
                {"jacobian": J_left, "name": "left_foot", "accel_offset": -foot_kd * left_vel},
                {"jacobian": J_right, "name": "right_foot", "accel_offset": -foot_kd * right_vel},
            ]
            extra_tasks = self._build_pelvis_orientation_task()

        return active_feet, swing_tasks, extra_tasks if extra_tasks else None

    def _build_pelvis_orientation_task(self) -> list[tuple[float, np.ndarray, np.ndarray]]:
        """Build pelvis roll/pitch/yaw regulation task for the QP.

        Uses the TransitionController's tuning (kp=200, kd=20, weight=50)
        which keeps pelvis pitch within ±3.5° during single support.  The
        previous walking tuning (kp=30, kd=5, weight=0.5) allowed -30° pitch
        that drove the CoP to the foot edge and caused 0.25 m support slip.
        """
        model = self.env.model
        data = self.env.data
        nv = model.nv

        pelvis_quat_des = np.array([1.0, 0.0, 0.0, 0.0])
        pelvis_quat_cur = self.env.get_pelvis_quat()
        pelvis_ang_err = self._quat_error(pelvis_quat_des, pelvis_quat_cur)

        # Override yaw error to track target yaw instead of zero
        _, _, yaw_cur = euler_from_quat(*pelvis_quat_cur)
        yaw_err = self._angle_diff(self.pelvis_target_yaw, yaw_cur)
        pelvis_ang_err[2] = yaw_err

        J_pelvis_lin = np.zeros((3, nv))
        J_pelvis_ang = np.zeros((3, nv))
        mujoco.mj_jacBody(model, data, J_pelvis_lin, J_pelvis_ang, self._pelvis_bid)
        pelvis_omega = J_pelvis_ang @ data.qvel

        # Use TransitionController-level gains for stability
        pelvis_kp = self.cfg.get("transition", {}).get("pelvis_kp", 200.0)
        pelvis_kd = self.cfg.get("transition", {}).get("pelvis_kd", 20.0)
        pelvis_weight = self.cfg.get("transition", {}).get("pelvis_weight", 50.0)

        pelvis_accel_des = (
            -pelvis_kp * pelvis_ang_err
            - pelvis_kd * pelvis_omega
        )

        # Single unified 3-D task with high weight (like TransitionController)
        return [(pelvis_weight, J_pelvis_ang, pelvis_accel_des)]

    # ------------------------------------------------------------------ #
    # Setpoint helpers
    # ------------------------------------------------------------------ #

    def _compute_com_target(self, com_pos: np.ndarray) -> np.ndarray:
        """Return CoM setpoint based on the current phase."""
        left_pos = self.env.get_body_pos("left_foot")
        right_pos = self.env.get_body_pos("right_foot")
        midpoint = 0.5 * (left_pos + right_pos)
        midpoint[2] = self.com_start[2]

        if self._phase == BIPEDAL_INIT:
            return midpoint

        if self._phase in (WEIGHT_SHIFT_L, WEIGHT_SHIFT_R):
            if self._com_planner is not None:
                t = self.env.data.time - self._phase_start_time
                target, _, _ = self._com_planner.evaluate(t)
                return target
            # Fallback to step-change if planner missing
            if self._phase == WEIGHT_SHIFT_L:
                target = self._foot_center_world(left_pos, self._left_bid)
            else:
                target = self._foot_center_world(right_pos, self._right_bid)
            target[2] = self.com_start[2]
            return target

        if self._phase in (LEFT_SINGLE, RIGHT_SINGLE):
            if self._single_support_com_target is not None:
                return self._single_support_com_target
            # Fallback to dynamic target if cached target missing
            if self._phase == LEFT_SINGLE:
                target = self._foot_center_world(left_pos, self._left_bid)
            else:
                target = self._foot_center_world(right_pos, self._right_bid)
            target[2] = self.com_start[2]
            return target

        if self._phase == DOUBLE_SUPPORT:
            # Bias toward the next support foot
            if self._next_support_name == "left":
                biased = self.double_support_com_bias * left_pos + (1.0 - self.double_support_com_bias) * right_pos
            else:
                biased = self.double_support_com_bias * right_pos + (1.0 - self.double_support_com_bias) * left_pos
            biased[2] = self.com_start[2]
            return biased

        return midpoint

    def _foot_center_world(self, foot_pos: np.ndarray, foot_bid: int) -> np.ndarray:
        """Return the geometric centre of the foot CoP rectangle in world frame."""
        foot_R = self.env.data.xmat[foot_bid].reshape(3, 3)
        foot_center_x_local = 0.5 * (self.foot_cop_x_forward - self.foot_cop_x_back)
        center_world = foot_pos + foot_R @ np.array([foot_center_x_local, 0.0, 0.0])
        return center_world

    def _compute_swing_weights(self, phase_progress: float) -> tuple[float, float]:
        """Return (xy_weight, z_weight) based on swing progress."""
        if phase_progress < 0.6:
            xy_weight = self.swing_xy_early
        else:
            xy_weight = self.swing_xy_early + (self.swing_xy_late - self.swing_xy_early) * ((phase_progress - 0.6) / 0.4)
        return xy_weight, self.swing_z_weight

    def _setup_swing_phase(self) -> None:
        """Initialise swing foot planner for the new step.

        Uses the old lift-only SwingFootPlanner (no descent) to match the
        TransitionController, which holds the foot clear of the ground for the
        entire single-support phase.  The WalkingController's touchdown check
        handles the descent timing instead of a fixed trajectory.
        """
        support_pos = self.env.get_body_pos(f"{self._support_foot_name}_foot")
        swing_pos = self.env.get_body_pos(f"{self._swing_foot_name}_foot")

        pelvis_yaw = euler_from_quat(*self.env.get_pelvis_quat())[2]
        is_right_swing = self._swing_foot_name == "right"
        self._swing_target = self.footstep_planner.plan_step(support_pos, pelvis_yaw, is_right_swing)
        self._swing_foot_start = swing_pos.copy()

        # Use old lift-only planner; keep the foot clear until touchdown check
        # fires, rather than descending on a fixed schedule.
        self.swing_planner = SwingFootPlanner(
            self._swing_foot_start,
            self.cfg.get("transition", {}).get("swing_lift_height", 0.03),
            self.cfg.get("transition", {}).get("swing_rise_duration", 1.0),
        )

    # ------------------------------------------------------------------ #
    # Transition condition helpers
    # ------------------------------------------------------------------ #

    def _compute_grf(self, foot_name: str) -> float:
        """Vertical contact force (N) on the named foot."""
        force = self.env.get_contact_forces(foot_name)
        return float(force[2])

    def _check_weight_shift_to_single(self, support_foot: str, lift_foot: str) -> bool:
        """GRF hysteresis: arm at 50 %, fire at 80 % after delay.

        The fire condition requires BOTH force and geometry:
        - Support foot carries > 80 % of body weight
        - Lifted foot is effectively unloaded (< 5 N)
        - CoM is within 0.03 m of the support foot CoP centre (not the ankle
          joint).  The CoP centre is 3.5 cm forward of the ankle; measuring
          com_err to the ankle was causing transitions to fire while the CoM
          was still ~7 cm outside the CoP envelope.
        """
        grf_support = self._compute_grf(f"{support_foot}_foot")
        grf_lift = self._compute_grf(f"{lift_foot}_foot")
        mg = self._total_mass * 9.81

        if grf_support > self.grf_arm_threshold * mg:
            if not self._grf_armed:
                self._grf_armed = True
                self._grf_arm_time = self.env.data.time

        if self._grf_armed and (self.env.data.time - self._grf_arm_time > self.grf_arm_to_fire_delay):
            if grf_support > self.grf_fire_threshold * mg and grf_lift < self.grf_liftoff_threshold:
                com_pos = compute_com_position(self.env.model, self.env.data)
                foot_pos = self.env.get_body_pos(f"{support_foot}_foot")
                foot_bid = self._left_bid if support_foot == "left" else self._right_bid
                foot_center = self._foot_center_world(foot_pos, foot_bid)
                com_err = np.linalg.norm(com_pos[:2] - foot_center[:2])
                if com_err < 0.03:
                    self._grf_armed = False
                    return True

        return False

    def _check_touchdown(self, swing_foot_name: str, target_xy: np.ndarray | None) -> bool:
        """
        SINGLE → DOUBLE_SUPPORT: three-condition check.

        1. Foot near ground (z).
        2. Foot near target xy.
        3. GRF confirms physical contact.
        """
        if target_xy is None:
            return False

        swing_pos = self.env.get_body_pos(f"{swing_foot_name}_foot")
        support_pos = self.env.get_body_pos(f"{self._support_foot_name}_foot")

        z_ok = abs(swing_pos[2] - support_pos[2]) < self.touchdown_z_tolerance
        xy_ok = np.linalg.norm(swing_pos[:2] - target_xy[:2]) < self.touchdown_xy_tolerance
        grf_ok = self._compute_grf(f"{swing_foot_name}_foot") > self.grf_touchdown_threshold * self._total_mass * 9.81

        return z_ok and xy_ok and grf_ok

    def _compute_cp(self) -> np.ndarray:
        """Instantaneous capture point in world XY plane."""
        com_pos = compute_com_position(self.env.model, self.env.data)
        com_vel = compute_com_velocity(self.env.model, self.env.data)
        g = abs(self.env.model.opt.gravity[2])
        return compute_capture_point(com_pos, com_vel, g)

    def _is_cp_inside_combined_polygon(self, cp: np.ndarray) -> bool:
        """Check whether CP is inside the convex hull of both foot rectangles."""
        left_pos = self.env.get_body_pos("left_foot")
        right_pos = self.env.get_body_pos("right_foot")
        left_R = self.env.data.xmat[self._left_bid].reshape(3, 3)
        right_R = self.env.data.xmat[self._right_bid].reshape(3, 3)

        cp_left = left_R.T @ np.append(cp - left_pos[:2], 0.0)
        cp_right = right_R.T @ np.append(cp - right_pos[:2], 0.0)

        def _inside_foot(cp_local: np.ndarray) -> bool:
            return (
                -self.foot_cop_x_back <= cp_local[0] <= self.foot_cop_x_forward
                and -self.foot_cop_y_half <= cp_local[1] <= self.foot_cop_y_half
            )

        return _inside_foot(cp_left) or _inside_foot(cp_right)

    # ------------------------------------------------------------------ #
    # Wrench cone override (torsional friction)
    # ------------------------------------------------------------------ #

    def _build_wrench_cones(self, nv: int, active_feet: list):
        """Extend base wrench cones with torsional friction for 6-D contacts."""
        A, l, u = super()._build_wrench_cones(nv, active_feet)

        # Count how many extra rows we need for torsional friction
        lambda_dims = [foot["jacobian"].shape[0] for foot in active_feet]
        n_torsion = sum(1 for m in lambda_dims if m == 6)
        if n_torsion == 0:
            return A, l, u

        n_old = A.shape[0]
        n_new = n_old + 2 * n_torsion
        nx = nv + sum(lambda_dims)

        A_new = np.zeros((n_new, nx))
        l_new = np.full(n_new, -np.inf)
        u_new = np.zeros(n_new)

        A_new[:n_old, :] = A
        l_new[:n_old] = l
        u_new[:n_old] = u

        row = n_old
        lam_start = nv
        for foot in active_feet:
            m = foot["jacobian"].shape[0]
            if m == 6:
                fz_i = lam_start + 2
                tz_i = lam_start + 5
                tau_z_max = (self.mu * min(self._foot_width, self._foot_length) / 2.0)

                # tau_z <= tau_z_max * fz
                A_new[row, tz_i] = 1.0
                A_new[row, fz_i] = -tau_z_max
                row += 1

                # -tau_z <= tau_z_max * fz
                A_new[row, tz_i] = -1.0
                A_new[row, fz_i] = -tau_z_max
                row += 1

            lam_start += m

        return A_new, l_new, u_new

    # ------------------------------------------------------------------ #
    # Static helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _angle_diff(target: float, current: float) -> float:
        """Signed shortest angle difference from current to target."""
        diff = target - current
        while diff > np.pi:
            diff -= 2.0 * np.pi
        while diff < -np.pi:
            diff += 2.0 * np.pi
        return diff

    # ------------------------------------------------------------------ #
    # Properties
    # ------------------------------------------------------------------ #

    @property
    def state(self) -> str:
        return self._phase

    @property
    def phase_elapsed(self) -> float:
        return self.env.data.time - self._phase_start_time

    @property
    def step_count(self) -> int:
        return self._step_count
