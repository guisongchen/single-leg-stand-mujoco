# Initial Pose Sweep Methodology

## Why hip, knee, and ankle?

These three joints form the **sagittal-plane leg chain** — the only joints that meaningfully move the CoM forward or backward relative to the feet when the robot stands symmetrically.

| Joint | Controls | Effect on CoM |
|-------|----------|---------------|
| **Hip pitch** | Thigh angle | Moves the **entire upper body** (torso, arms, head) forward or backward. The torso is ~60% of total mass, so this is the dominant lever. |
| **Knee pitch** | Shin angle | Bends the leg. A bent knee lowers the CoM and shifts it slightly backward because the shin swings forward. |
| **Ankle pitch** | Foot angle | Tilts the foot. A more negative ankle pitch (toes up) shifts the contact point backward under the shin, effectively moving the foot forward relative to the CoM. |

All other joints are irrelevant for fore-aft CoM placement in a symmetric standing pose:

- **Hip roll / ankle roll** — Rotate about X (side-to-side). At zero, they only affect lateral balance.
- **Hip yaw** — Rotates about Z (twist). Does not move CoM in the sagittal plane.
- **Arm joints** — Light compared to the torso; their default pose keeps their CoM near the torso.

## Why sweep hip as rows and use knee/ankle as plot axes?

**Hip pitch is the "coarse" control.** A small change (-0.1 rad) shifts the CoM by several centimeters because it reorients the heavy torso. It is so powerful that each row represents a different **posture family**.

**Knee and ankle are the "fine" controls within each family.** Once you fix a hip lean angle, you still have one degree of freedom left: you can trade knee bend against ankle tilt while keeping the foot on the ground.

| Example | Description |
|---------|-------------|
| Deep knee bend + flat ankle | Crouch |
| Straight knee + toes-down ankle | Tall stance |
| Medium knee + slight toes-up | Balanced middle ground |

These two are naturally coupled, so they form a 2D grid that shows the full landscape of possible stances at a given hip angle.

## Why not a full 3D cube?

The relationship is approximately separable:

```
CoM_x ≈ f(hip) + g(knee, ankle)
```

- `f(hip)` is large and independent.
- `g(knee, ankle)` is smaller and coupled.

By fixing hip as discrete rows, a 3D search becomes a series of 2D maps that are easy to visualize and interpret.

## How to read the sweep figure

The figure is a grid of heatmaps (4 rows x 2 columns).

**Rows** — One per hip pitch value:
- Row 1: hip = 0.0 rad (0deg)
- Row 2: hip = -0.1 rad (-5.7deg)
- Row 3: hip = -0.2 rad (-11.5deg)
- Row 4: hip = -0.3 rad (-17.2deg)

**Columns:**
- **Left:** CoM x-offset from foot midpoint. **Red** = CoM forward of feet. **Blue** = CoM behind feet. **White/near-zero** = CoM centered.
- **Right:** CoM height in meters. Taller (green/yellow) = more extended legs. Shorter (purple) = more crouched.

**Axes (both columns):**
- **X-axis:** Knee angle in degrees (left = straight, right = deeply bent).
- **Y-axis:** Ankle pitch in degrees (top = toes up, bottom = toes down).

**The black "zero" line** is the contour where CoM offset = 0. Any pose on this line places the CoM exactly over the foot midpoint.

| Row | Observation |
|-----|-------------|
| hip = 0.0deg | Entirely red. No zero line in range. The CoM is always forward. |
| hip = -5.7deg | Zero line appears at knee ~8-10deg, ankle ~-5deg to -10deg. |
| hip = -11.5deg | Zero line shifts to knee ~18-20deg. More knee bend needed. |
| hip = -17.2deg | Zero line shifts to knee ~28-30deg. Very deep crouch required. |

## Intuition in human terms

Imagine standing yourself:

1. **Lean forward at the hip** (hip pitch ~ 0). Your chest goes over your toes. You will fall forward unless you do something extreme with your knees or ankles — but the figure shows there is no comfortable solution.
2. **Lean back at the hip** (hip pitch ~ -0.1). Your chest moves behind your hips. Now your natural instinct is to **bend the knees slightly** and **tilt the ankles** to keep the feet flat under you.
3. **Lean back too far** (hip pitch ~ -0.3). You must crouch deeply (knee ~ 30deg) to avoid sitting down.

The black zero-contour line traces exactly that intuition: **"For each amount of backward lean, here is the knee/ankle combination that keeps you balanced."**

## Selected pose

From Row 2 (hip = -0.1 rad), a point just to the right of the zero line:

| Joint | Value | Rationale |
|-------|-------|-----------|
| Hip pitch | -0.1 rad (-5.7deg) | Leans torso back to center heavy upper body over feet. |
| Knee pitch | 0.15 rad (8.6deg) | Moderate bend, near the zero contour. |
| Ankle pitch | -0.08 rad (-4.6deg) | Keeps foot nearly flat so all 8 contact spheres touch ground evenly. |

Result: CoM x-offset ~0.4 cm, well inside the support polygon.
