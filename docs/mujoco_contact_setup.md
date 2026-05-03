# MuJoCo Contact Setup for G1 Single-Leg Stand

## 1. Ankle Joint Structure

The Unitree G1 robot has **no ankle yaw joint**. Each ankle has only:

- `ankle_pitch_joint` — axis `[0, 1, 0]` (rotates about Y)
- `ankle_roll_joint` — axis `[1, 0, 0]` (rotates about X)

This is a deliberate hardware design choice. Ankle yaw is omitted because:
- It adds mass at the foot (bad for swing-leg dynamics)
- Foot yaw can be controlled via the **hip yaw joint** instead
- A yaw actuator at the ankle would have very limited useful range

## 2. Which Link Represents the Foot?

Use the **roll link** (`left_ankle_roll_link` / `right_ankle_roll_link`) as the foot frame.

Kinematic chain:

```
shin → ankle_pitch_link → ankle_roll_link → [ground contact]
```

The roll link is the **distal** (last) link in the leg chain and carries the collision geometry. The pitch link is just an intermediate body ~1.75 cm above the actual foot.

| Task | Correct Link |
|------|-------------|
| Foot position/orientation control | `ankle_roll_link` |
| Contact wrench computation | `ankle_roll_link` |
| Jacobian for foot tasks | `ankle_roll_link` |
| Support polygon | `ankle_roll_link` |

## 3. Foot Collision Geometry in XML

Each `ankle_roll_link` body contains **5 geoms**:

```xml
<body name="left_ankle_roll_link" pos="0 0 -0.017558">
  <!-- 1. Visual mesh (NO collision) -->
  <geom type="mesh" contype="0" conaffinity="0" group="1"
        mesh="left_ankle_roll_link"/>

  <!-- 2-5. Collision spheres (4 contact points) -->
  <geom size="0.005" pos="-0.05  0.025 -0.03"/>  <!-- 5mm sphere, back-left  -->
  <geom size="0.005" pos="-0.05 -0.025 -0.03"/>  <!-- 5mm sphere, back-right -->
  <geom size="0.005" pos=" 0.12  0.03  -0.03"/>  <!-- 5mm sphere, front-left  -->
  <geom size="0.005" pos=" 0.12 -0.03  -0.03"/>  <!-- 5mm sphere, front-right -->
</body>
```

**Important:** The 4 sphere geoms do **not** specify `contype` or `conaffinity`. In MuJoCo XML, omitted attributes inherit **defaults** (`contype="1"`, `conaffinity="1"`), they do **not** become 0.

The first geom explicitly sets `contype="0" conaffinity="0"` to disable collision on the visual mesh.

## 4. MuJoCo Contact Parameter Defaults

For geoms that don't specify these attributes, MuJoCo uses:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `solref` | `[0.02, 1.0]` | Time constant = 0.02s, damping ratio = 1.0 |
| `solimp` | `[0.9, 0.95, 0.001, 0.5, 2.0]` | Contact penetration curve |
| `friction[0]` (slide) | `1.0` | Sliding / tangential friction |
| `friction[1]` (spin) | `0.005` | Torsional friction about contact normal |
| `friction[2]` (roll) | `0.0001` | Rolling friction about tangent axes |

## 5. Why the Original Code Values Were Wrong

Original code in `env/g1_env.py`:

```python
self.model.geom_solref[gid][0] = 0.002   # time constant
self.model.geom_friction[gid][2] = 0.01  # rolling friction
```

### Problem 1: `solref[0] = 0.002`

- Simulation timestep is `dt = 0.002 s`
- MuJoCo stability guideline: `solref[0] >= 2 * timestep`
- Setting `solref[0] = 0.002` equals the timestep → at the numerical stability limit
- Can cause contact jitter and solver divergence for a 34 kg humanoid

### Problem 2: `friction[2] = 0.01`

- Foot contacts are 5 mm spheres
- Rolling friction of `0.01` is negligible (default is `0.0001`)
- Spheres roll almost freely like ball bearings, shifting contact points and destabilizing balance

### Problem 3: Modifying all geoms on the body

- The loop modified the visual mesh geom too (which has `contype=0`)
- Visual mesh doesn't participate in collision, so changing its contact params is harmless but misleading

## 6. Corrected Values

Current code in `env/g1_env.py`:

```python
foot_friction = self.cfg["simulation"].get("friction", 0.8)
for foot_name in ("left_foot", "right_foot"):
    bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                             self.cfg["robot"]["body_names"][foot_name])
    for gid in range(self.model.ngeom):
        if self.model.geom_bodyid[gid] == bid and self.model.geom_contype[gid] > 0:
            self.model.geom_solref[gid][:] = [0.01, 1.0]
            self.model.geom_friction[gid][:] = [foot_friction, 0.005, 0.1]
```

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `solref` | `[0.01, 1.0]` | 5× the timestep; stiffer than default but numerically stable |
| `friction[0]` (slide) | `0.8` (from config) | Explicitly set instead of silently using default `1.0` |
| `friction[1]` (spin) | `0.005` | Default is fine |
| `friction[2]` (roll) | `0.1` | 1000× default; enough to prevent 5mm sphere feet from rolling freely |

## 7. Key Rules for XML Attribute Defaults

- **`contype` / `conaffinity`**: Default is `1` if omitted
  - Explicitly set to `0` only for visual meshes
  - Collision geoms typically don't need these attributes at all
- **`solref` / `solimp`**: Inherit from `<default>` class or global defaults
  - For stiff contacts, ensure `solref[0] >= 2 * timestep`
- **`friction`**: Default is `[1.0, 0.005, 0.0001]`
  - For sphere/capsule feet, always bump rolling friction (`friction[2]`) significantly

## 8. Model vs Data Constants

`geom_contype`, `geom_conaffinity`, `geom_solref`, and `geom_friction` are part of **`mjModel`** (model definition), not `mjData` (simulation state). They are constant during simulation. However, MuJoCo allows modifying some `mjModel` fields at runtime before or between steps, as the code does above.

---

## 9. Full QP-WBC Formulation

The controller in `controllers/bipedal_stance_controller.py` uses a **full QP-based Whole-Body Control** formulation with explicit contact wrenches.

### 9.1 Decision Variables

```
x = [qacc (nv=29); lambda_left (6); lambda_right (6)]   → 41 variables
```

`lambda` is a **spatial wrench** `[fx, fy, fz, tx, ty, tz]` in **world frame** at each foot.

### 9.2 Objective (penalises only qacc)

```
minimise  ||√w_com    · (J_com·qacc      - a_com)||²
        + ||√w_pelvis · (J_pelvis[3:]·qacc - α_pelvis)||²
        + ||√w_posture· (qacc_joints     - a_posture)||²
        + reg·||qacc||² + reg_lambda·||lambda||²
```

The contact wrenches have **zero cost** — they are determined entirely by the constraints.

### 9.3 Equality Constraints (hard)

**1. Floating-base dynamics — 6 equations**

The floating base has **no actuators**, so contact wrenches must exactly balance inertia and gravity:

```
M[0:6,:]·qacc + h[0:6] = (J_left^T·lambda_left + J_right^T·lambda_right)[0:6]
```

where `h = qfrc_bias - qfrc_passive`.

**2. Foot-fixed kinematics — 12 equations**

```
J_left  · qacc = 0
J_right · qacc = 0
```

Both feet are constrained to zero spatial acceleration.

### 9.4 Inequality Constraints (hard)

Per foot, a **linearised friction pyramid + CoP bounds** (9 inequalities):

| # | Constraint | Form |
|---|-----------|------|
| 1 | `fz >= 0` | Unilateral contact |
| 2 | `fx - mu·fz <= 0` | Friction pyramid (front) |
| 3 | `-fx - mu·fz <= 0` | Friction pyramid (back) |
| 4 | `fy - mu·fz <= 0` | Friction pyramid (left) |
| 5 | `-fy - mu·fz <= 0` | Friction pyramid (right) |
| 6 | `tx - (W/2)·fz <= 0` | CoP roll bound |
| 7 | `-tx - (W/2)·fz <= 0` | CoP roll bound |
| 8 | `ty - (L/2)·fz <= 0` | CoP pitch bound |
| 9 | `-ty - (L/2)·fz <= 0` | CoP pitch bound |

Parameters:
- `mu = 0.8` (from config)
- `W/2 = 0.03 m` (foot half-width)
- `L/2 = 0.085 m` (foot half-length)

### 9.5 Torque Recovery

After solving the QP, joint torques are recovered **analytically** from the full dynamics equation (no `mj_inverse` needed):

```python
tau = (M·qacc + h - J_left^T·lambda_left - J_right^T·lambda_right)[6:]
```

This is exact because the QP already enforced consistency between `qacc`, `lambda`, and the floating-base dynamics.

### 9.6 Comparison: Simplified vs Full QP-WBC

| Aspect | Simplified (old) | Full (current) |
|--------|-----------------|----------------|
| Variables | `qacc` (29) | `qacc + lambda` (41) |
| Contact forces | Implicit (via `mj_inverse`) | Explicit decision variables |
| Dynamics constraint | None | Floating-base dynamics enforced |
| Friction cone | None | Linearised pyramid + CoP bounds |
| Torque computation | `mj_inverse` | Analytical from dynamics |
| Foot drift (10 s) | ~7 mm | ~7 mm |
| Solve time | ~400 μs | ~840 μs |

The full formulation is ~2× slower but is the correct foundation for adding:
- Single-leg stance (set one foot's wrench to zero)
- Walking (time-varying foot constraints)
- Push recovery (add CoM capture point tasks)
