import numpy as np


class SwingFootPlanner:
    """
    Vertical lift-and-hold trajectory for a swing foot.

    The horizontal position is held at the start position; only z changes.
    This minimizes lateral disturbance during the transition.

    Parameters
    ----------
    start_pos : np.ndarray
        Foot position at lift-off (3,).
    lift_height : float
        Desired clearance above the ground (m).
    rise_duration : float
        Time to reach full lift_height.
    """

    def __init__(
        self,
        start_pos: np.ndarray,
        lift_height: float,
        rise_duration: float,
    ):
        self.start = start_pos.copy()
        self.lift_height = lift_height
        self.rise_duration = max(rise_duration, 1e-6)

    def evaluate(self, t: float):
        """
        Evaluate trajectory at time t.

        Returns
        -------
        pos : np.ndarray
        vel : np.ndarray
        accel : np.ndarray
        """
        s = np.clip(t / self.rise_duration, 0.0, 1.0)

        # Smooth bell curve for vertical lift: sin(pi * s) for s in [0,1]
        # After s=1, hold at lift_height.
        if s < 1.0:
            lift = self.lift_height * np.sin(np.pi * 0.5 * s)
            vel_z = self.lift_height * np.pi * 0.5 / self.rise_duration * np.cos(np.pi * 0.5 * s)
            accel_z = -self.lift_height * (np.pi * 0.5 / self.rise_duration) ** 2 * np.sin(np.pi * 0.5 * s)
        else:
            lift = self.lift_height
            vel_z = 0.0
            accel_z = 0.0

        pos = np.array([self.start[0], self.start[1], self.start[2] + lift])
        vel = np.array([0.0, 0.0, vel_z])
        accel = np.array([0.0, 0.0, accel_z])

        return pos, vel, accel
