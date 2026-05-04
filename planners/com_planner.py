import numpy as np


class ComPlanner:
    """
    Smooth CoM trajectory using a quintic smoothstep (10s^3 - 15s^4 + 6s^5).

    The quintic profile is C^2 across the phase boundary: position, velocity,
    and acceleration all hit zero at s=0 and s=1. The cubic alternative
    (3s^2 - 2s^3) leaves a 6/dur^2 jump in acceleration when entering or
    leaving the phase, which the QP must absorb as a step in the desired
    CoM acceleration -- the source of the lateral force chatter we saw at
    the WEIGHT_SHIFT -> SINGLE_LEG handoff.

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
        s4 = s3 * s
        s5 = s4 * s

        coeff = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
        dcoeff_ds = 30.0 * s2 - 60.0 * s3 + 30.0 * s4
        d2coeff_ds2 = 60.0 * s - 180.0 * s2 + 120.0 * s3

        delta = self.target - self.start
        pos = self.start + coeff * delta
        vel = dcoeff_ds / self.duration * delta
        accel = d2coeff_ds2 / (self.duration ** 2) * delta

        return pos, vel, accel
