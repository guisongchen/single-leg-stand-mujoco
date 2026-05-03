import os
from typing import Optional, Dict, Any

import numpy as np
import yaml
import mujoco


class G1Env:
    """MuJoCo simulation environment for Unitree G1 humanoid."""

    def __init__(self, config_path: str = "configs/g1_config.yaml") -> None:
        with open(config_path, "r") as f:
            self.cfg: Dict[str, Any] = yaml.safe_load(f)

        self.model_path = self.cfg["simulation"]["model_path"]
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model not found: {self.model_path}")

        self.dt: float = self.cfg["simulation"]["dt"]
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.model.opt.timestep = self.dt
        self.model.opt.gravity[:] = self.cfg["simulation"]["gravity"]

        # Stiffen foot contacts and increase rolling friction to prevent sinking/rolling
        for foot_name in ("left_foot", "right_foot"):
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                     self.cfg["robot"]["body_names"][foot_name])
            for gid in range(self.model.ngeom):
                if self.model.geom_bodyid[gid] == bid:
                    self.model.geom_solref[gid][0] = 0.002
                    self.model.geom_friction[gid][2] = 0.01

        self.data = mujoco.MjData(self.model)

        # IDs for quick lookup
        self._body_ids = {
            k: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, v)
            for k, v in self.cfg["robot"]["body_names"].items()
        }

        self._joint_names = self.cfg["robot"]["joint_names"]
        self._joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self._joint_names
        ]
        # qpos addresses for actuated joints (floating base takes first 7)
        self._qposadr = np.array(
            [self.model.jnt_qposadr[jid] for jid in self._joint_ids], dtype=np.int32
        )
        # qvel addresses for actuated joints (floating base takes first 6)
        self._qveladr = np.array(
            [self.model.jnt_dofadr[jid] for jid in self._joint_ids], dtype=np.int32
        )

        self._initial_qpos = self._build_initial_qpos()

    # ------------------------------------------------------------------ #
    # State access
    # ------------------------------------------------------------------ #

    @property
    def nq(self) -> int:
        return self.model.nq

    @property
    def nv(self) -> int:
        return self.model.nv

    @property
    def nu(self) -> int:
        return self.model.nu

    def get_time(self) -> float:
        return float(self.data.time)

    def get_qpos(self) -> np.ndarray:
        return np.array(self.data.qpos)

    def get_qvel(self) -> np.ndarray:
        return np.array(self.data.qvel)

    def get_actuated_qpos(self) -> np.ndarray:
        return self.data.qpos[self._qposadr].copy()

    def get_actuated_qvel(self) -> np.ndarray:
        return self.data.qvel[self._qveladr].copy()

    def get_pelvis_pos(self) -> np.ndarray:
        return self.data.xpos[self._body_ids["pelvis"]].copy()

    def get_pelvis_quat(self) -> np.ndarray:
        return self.data.xquat[self._body_ids["pelvis"]].copy()

    def get_body_pos(self, name: str) -> np.ndarray:
        bid = self._body_ids.get(name)
        if bid is None:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        return self.data.xpos[bid].copy()

    def get_body_quat(self, name: str) -> np.ndarray:
        bid = self._body_ids.get(name)
        if bid is None:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        return self.data.xquat[bid].copy()

    # ------------------------------------------------------------------ #
    # Contact / force queries
    # ------------------------------------------------------------------ #

    def get_contact_forces(self, body_name: str) -> np.ndarray:
        """Return total contact force [fx, fy, fz] on a body (world frame)."""
        bid = self._body_ids.get(body_name)
        if bid is None:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        # Collect geom IDs belonging to this body
        body_geoms = [
            gid for gid in range(self.model.ngeom) if self.model.geom_bodyid[gid] == bid
        ]
        total_force = np.zeros(3)
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            if con.geom1 in body_geoms or con.geom2 in body_geoms:
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, force)
                frame = con.frame.reshape(3, 3)
                world_force = frame @ force[:3]
                if con.geom1 in body_geoms:
                    total_force += world_force
                else:
                    total_force -= world_force
        return total_force

    def get_contact_status(self) -> Dict[str, bool]:
        """Check whether left_foot and right_foot are in contact with the ground."""
        status = {"left_foot": False, "right_foot": False}
        for i in range(self.data.ncon):
            con = self.data.contact[i]
            b1 = self.model.geom_bodyid[con.geom1]
            b2 = self.model.geom_bodyid[con.geom2]
            for name in ("left_foot", "right_foot"):
                bid = self._body_ids[name]
                if b1 == bid or b2 == bid:
                    status[name] = True
        return status

    # ------------------------------------------------------------------ #
    # Control & stepping
    # ------------------------------------------------------------------ #

    def step(self, ctrl: Optional[np.ndarray] = None) -> None:
        if ctrl is not None:
            self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:] = self._initial_qpos
        mujoco.mj_forward(self.model, self.data)
        # Auto-adjust base height so feet touch the ground
        self._adjust_base_height()
        mujoco.mj_forward(self.model, self.data)

    def set_state(self, qpos: np.ndarray, qvel: Optional[np.ndarray] = None) -> None:
        self.data.qpos[:] = qpos
        if qvel is not None:
            self.data.qvel[:] = qvel
        else:
            self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------ #
    # Rendering helpers
    # ------------------------------------------------------------------ #

    def render(
        self,
        renderer: Optional[mujoco.Renderer] = None,
        camera: Optional[str] = None,
        width: int = 1280,
        height: int = 720,
    ) -> Optional[np.ndarray]:
        if renderer is None:
            renderer = mujoco.Renderer(self.model, height, width)
        if camera is not None:
            renderer.update_scene(self.data, camera=camera)
        else:
            renderer.update_scene(self.data)
        return renderer.render()

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #

    def _build_initial_qpos(self) -> np.ndarray:
        qpos = np.zeros(self.model.nq)
        base = self.cfg["robot"]["initial_base_pos"]
        qpos[:7] = base
        init_joints = self.cfg["robot"]["initial_qpos"]
        for i, name in enumerate(self._joint_names):
            qpos[self._qposadr[i]] = init_joints.get(name, 0.0)
        return qpos

    def _adjust_base_height(self) -> None:
        """Shift pelvis z so the lowest foot geom touches the ground (z=0)."""
        min_z = float("inf")
        for foot_name in ("left_foot", "right_foot"):
            bid = self._body_ids[foot_name]
            for gid in range(self.model.ngeom):
                if self.model.geom_bodyid[gid] == bid:
                    gtype = self.model.geom_type[gid]
                    pos_local = np.array(self.model.geom_pos[gid])
                    if gtype == mujoco.mjtGeom.mjGEOM_SPHERE.value:
                        pos_local[2] -= self.model.geom_size[gid][0]
                    elif gtype == mujoco.mjtGeom.mjGEOM_MESH.value:
                        mesh_id = self.model.geom_dataid[gid]
                        verts = self.model.mesh_vert[
                            self.model.mesh_vertadr[mesh_id] :
                            self.model.mesh_vertadr[mesh_id] + self.model.mesh_vertnum[mesh_id]
                        ]
                        scale = self.model.geom_size[gid]
                        scaled_min_z = (verts * scale)[:, 2].min()
                        pos_local[2] += scaled_min_z
                    elif gtype == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value:
                        pos_local[2] -= self.model.geom_size[gid][2]
                    elif gtype == mujoco.mjtGeom.mjGEOM_BOX.value:
                        pos_local[2] -= self.model.geom_size[gid][2]
                    else:
                        # fallback: treat size[0] as radius-like
                        pos_local[2] -= self.model.geom_size[gid][0]
                    xpos = self.data.xpos[bid]
                    xmat = self.data.xmat[bid].reshape(3, 3)
                    pos_world = xpos + xmat @ pos_local
                    min_z = min(min_z, pos_world[2])
        if min_z != 0.0:
            self.data.qpos[2] -= min_z
        # Pre-sink by 2 mm so initial penetration gives immediate contact force
        self.data.qpos[2] -= 0.002
