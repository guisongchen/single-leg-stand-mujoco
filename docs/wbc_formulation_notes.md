# WBC Formulation Notes

This document collects design decisions and conceptual comparisons made during the development of the QP-based Whole-Body Controller for the Unitree G1 humanoid.

---

## 1. Task Choice: Pelvis Angular Acceleration vs. Centroidal Angular Momentum (CAM)

### Pelvis angular acceleration
- Task Jacobian: `J_pelvis[3:,:]` (angular part of the pelvis body Jacobian).
- Targets pelvis orientation directly via PD on angular acceleration.
- **Pros:** Stable for bipedal standing; intuitive; easy to debug.
- **Cons:** Only controls the torso, not the whole-body rotational momentum. A robot can have a perfectly upright pelvis while its limbs are rotating in ways that destabilize the CoM.

### Centroidal angular momentum
- Defined as the total angular momentum of all bodies about the system CoM.
- Task Jacobian: `J_cam` (must be constructed manually in MuJoCo; see below).
- **Pros:** Captures whole-body rotational dynamics. Zero CAM means the robot is statically balanced in rotation. Preferred for dynamic maneuvers (walking, running, push recovery).
- **Cons:** More sensitive to gain tuning. If gains are too high, the QP can become infeasible or oscillate because limb inertias couple back into the pelvis.

### Current choice
The controller uses **CAM** as the second task (after CoM linear acceleration) because the end goal is single-leg stance and dynamic balancing, where whole-body momentum matters. The pelvis orientation task is available as a fallback if CAM proves too aggressive for a given pose.

---

## 2. Computing CAM in MuJoCo

### Value: `data.subtree_angmom[0]` is not auto-populated
MuJoCo stores subtree angular momentum in `data.subtree_angmom`, but **it is not computed during `mj_step`** in MuJoCo 3.x Python bindings. It remains zero unless explicitly computed:

```python
mujoco.mj_subtreeVel(model, data)   # must be called explicitly
cam = data.subtree_angmom[0]        # now valid
```

Because forgetting this call produces a silent zero (which disables the CAM P-term and destabilizes the controller), we avoid it.

### Current choice: compute from the Jacobian
Since `J_cam` is already built for the QP objective, the CAM value is obtained from its definition:

```python
cam = J_cam @ data.qvel   # 3-vector, kg·m²/s
```

This is mathematically identical to `subtree_angmom[0]` (difference < 3% in practice) and costs only a 3×29 dot product. It removes a hidden dependency on `mj_subtreeVel`.

### Jacobian: must be built manually
MuJoCo does **not** expose a function for the CAM Jacobian (`J_cam`). It is constructed body-by-body:

```
J_cam = Σ_i [ I_i_world * J_ang_i + m_i * skew(c_i - com) * J_lin_i ]
```

where `J_lin_i` is the Jacobian of body i's CoM, not its origin. The shift from origin to CoM is:

```
J_lin_com = J_lin_origin - skew(offset_world) @ J_ang
```

A manual loop over all bodies is required because there is no `mj_jacSubtreeAngmom` in the MuJoCo API.

---

## 3. Why Contact Wrenches are Explicit Decision Variables

### Alternative: bake contact into the Hessian
One could treat contact as soft objectives:

```
min  ||M*qacc + h - J^T*wrench||²  +  w_foot*||J_foot*qacc||²  +  tasks...
```

This makes dynamics and fixed feet weighted penalties rather than hard constraints.

### Why hard constraints are preferred
1. **Physical correctness.** A 34 kg humanoid cannot violate Newton's laws. Hard equality constraints enforce exact dynamics and zero foot acceleration.
2. **Direct force feasibility.** With explicit wrenches, we can write inequalities on the actual contact forces (`|fx| <= mu*fz`, CoP bounds). If forces are implicit, the solver cannot prevent physically impossible reactions (e.g., `fz < 0`).
3. **Single-leg and walking readiness.** Setting one wrench to zero (single-leg stance) is trivial with explicit variables. With implicit forces, it requires a full reformulation.
4. **Exact torque recovery.** Torques are recovered analytically from the full dynamics equation using the same `qacc` and `wrench` that the QP produced. No `mj_inverse` is needed.

---

## 4. Reduced Task-Space WBC vs. Full QP-WBC

### Reduced Task-Space WBC ("3-Jacobian method")
Common in embedded systems and simple bipedal controllers.

- **Decision variables:** `qacc` only (`nv = 29`).
- **Tasks (objective):** CoM acceleration, angular momentum rate, joint posture.
- **Contact:** `J_contact * qacc = 0` (feet fixed, hard or soft constraint).
- **Dynamics:** Implicit. After solving for `qacc`, contact forces are recovered via `mj_inverse`.
- **Force constraints:** None. Friction and CoP are not enforced inside the QP.

### Full QP-WBC (current implementation)
Chosen as the foundation for dynamic humanoid control.

- **Decision variables:** `[qacc (29); wrench_left (6); wrench_right (6)]` → 41 variables.
- **Tasks (objective):** Same as above, but only `qacc` is penalized; wrenches have zero cost.
- **Contact:** `J_left * qacc = 0`, `J_right * qacc = 0` (hard equality).
- **Dynamics:** Explicit hard equality: `M[:6,:]*qacc = (J_left^T*wrench_l + J_right^T*wrench_r)[:6] - h[:6]`.
- **Force constraints:** Friction pyramid + CoP bounds as hard inequalities on `wrench_left` and `wrench_right`.
- **Torque recovery:** Analytical: `tau = (M*qacc + h - J_left^T*wrench_l - J_right^T*wrench_r)[6:]`.

### Key difference
Both methods use `J * qacc = 0`. The difference is **what happens to the contact forces after `qacc` is decided**:

- **Reduced:** Forces are an afterthought computed by `mj_inverse`. If they violate physics, the controller has no mechanism to fix it.
- **Full:** Forces are co-optimized with `qacc` and directly bounded. The solver can trade off motion and force distribution while respecting friction and CoP limits.

### When to use which

| Scenario | Choice |
|----------|--------|
| Flat ground, always two feet, high friction, fast loop (< 1 ms) | Reduced task-space |
| Uneven terrain, slipping risk, single-leg stance, walking | **Full QP-WBC** |
| Need to log, constrain, or visualize ground reaction forces | **Full QP-WBC** |

For this project, the full formulation is used because the end goal is single-leg stance, where force feasibility is not optional.

---

## 5. Notation Reference

| Symbol | Meaning |
|--------|---------|
| `nv` | Number of velocity DOFs (29 for G1) |
| `nu` | Number of actuators (23 for G1) |
| `M` | Mass matrix |
| `h` / `bias_force` | Coriolis, gravity, and passive forces (`qfrc_bias - qfrc_passive`) |
| `J_com` | CoM position Jacobian (3 x nv) |
| `J_cam` / `J_L` | Centroidal angular momentum Jacobian (3 x nv) |
| `J_left`, `J_right` | Spatial Jacobian of each foot (6 x nv) |
| `wrench_left`, `wrench_right` | Contact wrench `[fx, fy, fz, tx, ty, tz]` in world frame |
| `mu` | Friction coefficient (0.8) |
| `W/2`, `L/2` | Foot half-width (0.03 m) and half-length (0.085 m) |
