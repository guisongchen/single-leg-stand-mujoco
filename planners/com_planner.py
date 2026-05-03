import numpy as np


class ComPlanner:
    """
    Smooth CoM trajectory using a cubic interpolation (smoothstep).

    Parameters
    ----------
    start_pos : np.ndarray
        Initial CoM position (3,).
    target_pos : np.ndarray
        Target CoM position (3,).
    duration : float
        Total transition time in seconds.
    """

    def __init__(self, start_pos: np.ndarray, target_pos: np.ndarray, duration: float):
        self.start = start_pos.copy()
        self.target = target_pos.copy()
        self.duration = max(duration, 1e-6)

    def evaluate(self, t: float):
        """
        Evaluate trajectory at time t.

        Returns
        -------
        pos : np.ndarray
        vel : np.ndarray
        accel : np.ndarray
        """
        s = np.clip(t / self.duration, 0.0, 1.0)
        s2 = s * s
        s3 = s2 * s

        coeff = 3.0 * s2 - 2.0 * s3
        dcoeff_ds = 6.0 * s - 6.0 * s2
        d2coeff_ds2 = 6.0 - 12.0 * s

        delta = self.target - self.start
        pos = self.start + coeff * delta
        vel = dcoeff_ds / self.duration * delta
        accel = d2coeff_ds2 / (self.duration ** 2) * delta

        return pos, vel, accel
