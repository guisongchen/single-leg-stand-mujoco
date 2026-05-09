"""CP-based foot placement planner.

Connects footstep targets to CoM dynamics: the next support foot is
placed so its CoP envelope covers the capture point, guaranteeing
that the LIPM dynamics during single support converge to stability
rather than diverging into a fall.

Reference
---------
The CoM during single support follows the LIPM:
    ẍ = ω² (x − x_cop)

The instantaneous capture point is:
    x_cp = x + ẋ / ω   where ω = √(g / z)

If x_cp lies inside the support foot's CoP envelope at the moment
of weight transfer, the CoM can be brought to rest without stepping.
"""
import numpy as np


class FootstepPlanner:
    """CP-driven footstep planner with heading correction.

    Parameters
    ----------
    step_length : float
        Nominal forward distance per step (m).
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
        cp: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compute swing foot target with CP-based forward adjustment.

        The forward placement is chosen so that the capture point falls
        inside the new support foot's centre-of-pressure envelope, which
        extends from −5 cm (heel) to +12 cm (toe) about the ankle joint
        (foot body origin).  The ankle is placed approximately under the
        CP, clipped to the kinematic reach envelope.

        Parameters
        ----------
        support_foot_pos : np.ndarray
            Current support foot world position [x, y, z].
        pelvis_yaw : float
            Current pelvis yaw (rad).
        is_right_swing : bool
            True if the *right* foot is swinging (left is support).
        cp : np.ndarray or None
            Capture point in world XY plane.  When None a pure heading-
            corrected nominal step is returned.

        Returns
        -------
        target : np.ndarray
            Swing foot target [x, y, z] (world frame).
        """
        step_dir = np.array([np.cos(pelvis_yaw), np.sin(pelvis_yaw), 0.0])
        left_foot_y = self.step_width / 2.0
        right_foot_y = -self.step_width / 2.0

        if cp is not None:
            # How far the CP has progressed ahead of the support foot
            # along the walking direction.
            cp_progress = float(
                np.dot(cp[:2] - support_foot_pos[:2], step_dir[:2])
            )
            # The foot's CoP centre lies ~3.5 cm ahead of the ankle
            # (body origin).  Placing the ankle just behind the CP
            # puts the CP inside the forward CoP region, which extends
            # to +12 cm from the ankle.
            #   x_ankle = cp_progress − 0.035  (CoP centre under CP)
            #   x_ankle ≥ cp_progress − 0.155  (toe can still reach CP)
            # Clamp: never step backward, never exceed nominal length.
            ankle_progress = np.clip(cp_progress - 0.035, 0.0, self.step_length)
            target_x = support_foot_pos[0] + step_dir[0] * ankle_progress
        else:
            target_x = (support_foot_pos[0]
                        + step_dir[0] * self.step_length)

        target_y = right_foot_y if is_right_swing else left_foot_y

        return np.array([target_x, target_y, support_foot_pos[2]])
