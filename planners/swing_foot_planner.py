import numpy as np


class SwingFootPlanner:
    """
    Vertical lift-and-hold trajectory for a swing foot.

    The horizontal position is held at the start position; only z changes.
    The vertical profile is a quintic smoothstep z(s) = H (10s^3 - 15s^4 + 6s^5),
    which is C^2 at both endpoints: position, velocity, and acceleration all
    hit their target values with zero derivative across the boundary.

    The previous sin(pi/2 * s) profile started lift-off at peak vertical
    velocity (a step in dz/dt at s=0) and ended with a nonzero acceleration
    step at s=1 -- both of these caused the QP to receive impulse-like swing
    targets at exactly the moments the support foot was being asked to take
    over balance.

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
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s

        coeff = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
        dcoeff_ds = 30.0 * s2 - 60.0 * s3 + 30.0 * s4
        d2coeff_ds2 = 60.0 * s - 180.0 * s2 + 120.0 * s3

        lift = self.lift_height * coeff
        vel_z = self.lift_height * dcoeff_ds / self.rise_duration
        accel_z = self.lift_height * d2coeff_ds2 / (self.rise_duration ** 2)

        pos = np.array([self.start[0], self.start[1], self.start[2] + lift])
        vel = np.array([0.0, 0.0, vel_z])
        accel = np.array([0.0, 0.0, accel_z])

        return pos, vel, accel
