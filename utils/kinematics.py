import numpy as np
import mujoco


def compute_com_position(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Compute the total CoM position in world frame."""
    return np.array(data.subtree_com[0])


def compute_com_velocity(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Compute the total CoM velocity in world frame using the Jacobian method."""
    com_pos = compute_com_position(model, data)
    jac_com = np.zeros((3, model.nv))
    mujoco.mj_jacSubtreeCom(model, data, jac_com, 0)
    com_vel = jac_com @ data.qvel
    return com_vel


def compute_body_position(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    """Get body position in world frame."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return np.array(data.xpos[bid])


def compute_body_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> np.ndarray:
    """Get body linear velocity in world frame."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, bid)
    return jacp @ data.qvel


def compute_contact_wrench(
    model: mujoco.MjModel, data: mujoco.MjData, body_name: str
) -> np.ndarray:
    """
    Compute total contact wrench [fx, fy, fz] on a body.
    This iterates over active contacts and sums forces where the body participates.
    """
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    # Find all geoms belonging to this body
    body_geoms = []
    for gid in range(model.ngeom):
        if model.geom_bodyid[gid] == bid:
            body_geoms.append(gid)

    total_force = np.zeros(3)
    for i in range(data.ncon):
        con = data.contact[i]
        if con.geom1 in body_geoms or con.geom2 in body_geoms:
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, force)
            # contact force is expressed in contact frame; rotate to world
            frame = con.frame.reshape(3, 3)
            world_force = frame @ force[:3]
            # Determine which body receives the force (the one we asked for)
            if con.geom1 in body_geoms:
                total_force += world_force
            else:
                total_force -= world_force
    return total_force


def quat_to_rotation_matrix(qw, qx, qy, qz) -> np.ndarray:
    """Convert scalar-first quaternion to 3x3 rotation matrix."""
    q = np.array([qw, qx, qy, qz])
    q = q / np.linalg.norm(q)
    qw, qx, qy, qz = q
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
    ])
    return R


def euler_from_quat(qw, qx, qy, qz) -> tuple:
    """Return roll, pitch, yaw (rad) from scalar-first quaternion."""
    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def compute_capture_point(com_pos: np.ndarray, com_vel: np.ndarray, gravity_z: float) -> np.ndarray:
    """Compute the instantaneous capture point in the world XY plane.

    CP = com_xy + com_vxy / omega, where omega = sqrt(g / com_z).

    Parameters
    ----------
    com_pos : np.ndarray
        CoM position [x, y, z] in world frame.
    com_vel : np.ndarray
        CoM velocity [vx, vy, vz] in world frame.
    gravity_z : float
        Magnitude of gravitational acceleration (positive scalar).

    Returns
    -------
    cp : np.ndarray
        Capture point [x, y] in world frame.
    """
    z_com = max(com_pos[2], 0.1)
    omega = np.sqrt(gravity_z / z_com)
    return com_pos[:2] + com_vel[:2] / omega
