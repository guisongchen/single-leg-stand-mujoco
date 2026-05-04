"""Fixed-schedule foot placement planner for periodic walking.

Generates a sequence of swing-foot target positions in world frame,
given a support foot position, step length, and step width.
"""
import numpy as np


class FootstepPlanner:
    """
    Simple fixed-schedule footstep planner.

    Parameters
    ----------
    step_length : float
        Forward distance per step (m).
    step_width : float
        Lateral distance between feet (m).
    forward_direction : np.ndarray
        Unit vector in world frame indicating walking direction.
    """

    def __init__(
        self,
        step_length: float,
        step_width: float,
        forward_direction: np.ndarray | None = None,
    ):
        self.step_length = float(step_length)
        self.step_width = float(step_width)
        self.forward_direction = (
            np.array(forward_direction)
            if forward_direction is not None
            else np.array([1.0, 0.0, 0.0])
        )
        self.forward_direction = self.forward_direction / np.linalg.norm(
            self.forward_direction
        )

    def plan_step(
        self,
        support_foot_pos: np.ndarray,
        is_left_swing: bool,
    ) -> np.ndarray:
        """
        Compute swing foot target position in world frame.

        The target is placed ``step_length`` forward of the support foot,
        offset laterally by ``step_width / 2`` in the direction of the
        swing foot.

        Parameters
        ----------
        support_foot_pos : np.ndarray
            Current support foot position [x, y, z] in world frame.
        is_left_swing : bool
            True if the left foot is the swing foot (i.e. right foot is
            the current support foot).

        Returns
        -------
        target : np.ndarray
            Swing foot target [x, y, z] in world frame.  z is preserved
            from ``support_foot_pos``.
        """
        lateral = np.cross(self.forward_direction, np.array([0.0, 0.0, 1.0]))
        lateral = lateral / np.linalg.norm(lateral)

        sign = 1.0 if is_left_swing else -1.0
        target = (
            support_foot_pos
            + self.step_length * self.forward_direction
            + sign * (self.step_width / 2.0) * lateral
        )
        target[2] = support_foot_pos[2]
        return target
