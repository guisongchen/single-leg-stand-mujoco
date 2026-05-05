import numpy as np


class SwingTrajectoryPlanner:
    """
    Full swing foot trajectory with lift, apex dwell, and descent.

    The vertical profile uses quintic smoothstep for both rise and descent:
    z(s) = H * (10s^3 - 15s^4 + 6s^5).

    The horizontal profile linearly interpolates from start to target xy
    over the full swing duration.

    Parameters
    ----------
    start_pos : np.ndarray
        Foot position at lift-off [x, y, z].
    target_pos : np.ndarray
        Foot target position at touchdown [x, y, z].
    lift_height : float
        Desired clearance above the start z.
    swing_duration : float
        Total swing phase duration.
    """

    def __init__(
        self,
        start_pos: np.ndarray,
        target_pos: np.ndarray,
        lift_height: float,
        swing_duration: float,
    ):
        self.start = start_pos.copy()
        self.target = target_pos.copy()
        self.lift_height = float(lift_height)
        self.swing_duration = max(float(swing_duration), 1e-6)

        self._rise_frac = 0.40
        self._hold_frac = 0.20
        self._descent_frac = 0.40

    def evaluate(self, t: float):
        """
        Evaluate trajectory at time t.

        Returns
        -------
        pos : np.ndarray
        vel : np.ndarray
        accel : np.ndarray
        """
        s = np.clip(t / self.swing_duration, 0.0, 1.0)

        # Horizontal: linear interpolation over full duration
        xy = self.start[:2] + s * (self.target[:2] - self.start[:2])
        xy_vel = (self.target[:2] - self.start[:2]) / self.swing_duration
        xy_accel = np.zeros(2)

        # Vertical: lift-hold-descent
        z0 = self.start[2]
        z_apex = z0 + self.lift_height
        z_end = self.target[2]

        if s < self._rise_frac:
            # Rise phase: 0 -> 1 over rise_frac
            s_rise = s / self._rise_frac
            z, dz, d2z = self._quintic_profile(s_rise)
            z_pos = z0 + self.lift_height * z
            z_vel = self.lift_height * dz / (self._rise_frac * self.swing_duration)
            z_accel = self.lift_height * d2z / ((self._rise_frac * self.swing_duration) ** 2)

        elif s < self._rise_frac + self._hold_frac:
            # Hold phase
            z_pos = z_apex
            z_vel = 0.0
            z_accel = 0.0

        else:
            # Descent phase: 1 -> 0 over descent_frac
            s_desc = (s - self._rise_frac - self._hold_frac) / self._descent_frac
            s_desc = np.clip(s_desc, 0.0, 1.0)
            z, dz, d2z = self._quintic_profile(s_desc)
            # quintic goes 0->1, we want 1->0
            z_pos = z_apex - self.lift_height * z
            z_vel = -self.lift_height * dz / (self._descent_frac * self.swing_duration)
            z_accel = -self.lift_height * d2z / ((self._descent_frac * self.swing_duration) ** 2)

        pos = np.array([xy[0], xy[1], z_pos])
        vel = np.array([xy_vel[0], xy_vel[1], z_vel])
        accel = np.array([xy_accel[0], xy_accel[1], z_accel])

        return pos, vel, accel

    @staticmethod
    def _quintic_profile(s: float):
        """Return (coeff, dcoeff_ds, d2coeff_ds2) for 10s^3 - 15s^4 + 6s^5."""
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s
        coeff = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
        dcoeff_ds = 30.0 * s2 - 60.0 * s3 + 30.0 * s4
        d2coeff_ds2 = 60.0 * s - 180.0 * s2 + 120.0 * s3
        return coeff, dcoeff_ds, d2coeff_ds2


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
