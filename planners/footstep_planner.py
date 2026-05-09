"""Fixed-schedule foot placement planner for periodic walking.

Generates swing-foot target positions in world frame,
given a support foot position, pelvis heading, step length, and step width.
"""
import numpy as np


class FootstepPlanner:
    """
    Simple fixed-schedule footstep planner with heading correction.

    Parameters
    ----------
    step_length : float
        Forward distance per step (m).
    step_width : float
        Lateral distance between feet (m).
    """

    def __init__(
        self,
        step_length: float,
        step_width: float,
    ):
        self.step_length = float(step_length)
        self.step_width = float(step_width)

    def plan_step(
        self,
        support_foot_pos: np.ndarray,
        pelvis_yaw: float,
        is_right_swing: bool,
    ) -> np.ndarray:
        """
        Compute swing foot target position in world frame with heading correction.

        Uses absolute lateral positions (not offset from support foot):
        left_foot_y = +step_width / 2, right_foot_y = -step_width / 2.

        The step direction is rotated by pelvis_yaw to prevent diagonal walking.

        Parameters
        ----------
        support_foot_pos : np.ndarray
            Current support foot position [x, y, z] in world frame.
        pelvis_yaw : float
            Current pelvis yaw angle (rad) in world frame.
        is_right_swing : bool
            True if the right foot is the swing foot (i.e. left foot is support).

        Returns
        -------
        target : np.ndarray
            Swing foot target [x, y, z] in world frame.  z is set to ground_z
            from support_foot_pos.
        """
        left_foot_y = self.step_width / 2.0
        right_foot_y = -self.step_width / 2.0

        step_dir = np.array([np.cos(pelvis_yaw), np.sin(pelvis_yaw), 0.0])

        target_x = support_foot_pos[0] + step_dir[0] * self.step_length
        target_y = right_foot_y if is_right_swing else left_foot_y

        return np.array([target_x, target_y, support_foot_pos[2]])
