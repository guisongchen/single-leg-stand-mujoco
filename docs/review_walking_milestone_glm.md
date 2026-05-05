# Review: Walking Milestone — Theory & Physics

**Document reviewed:** `docs/walking_milestone_task.md`  
**Reviewer:** GLM-5.1 (model-generated review)  
**Scope:** Theoretical correctness and physical reasonability only. No code implementation concerns.

---

## Verdict

The physics framework is sound. Friction limits, CoP constraints, CP-based transitions, and event-driven timing are all correctly formulated. The main risks cluster around an extremely tight lateral stability margin interacting with unquantified dynamic effects. Three specific physics issues — swing-leg angular momentum, contact wrench discontinuity, and the margin budget — compound each other and need explicit treatment before the design is considered complete.

---

## Sound Physics

### Friction limit correctly derived

`a_com_max = μ·g = 7.85 m/s²` is correct. Horizontal CoM acceleration is limited by friction regardless of contact count because the total normal force is always `mg`.

### Minimum double-support time correctly computed

The quintic factor (5.774) and resulting 0.35 s minimum are correct. The bang-bang minimum (0.29 s) is also correct. A 0.1–0.2 s fixed timer requiring 2.5–10g is physically impossible, which justifies the event-driven design.

### Event-driven FSM is the right architecture

Timer-driven transitions with tight CoP constraints are infeasible; event-driven switching lets the phase self-adjust to physics. This is consistent with established humanoid walking controllers (IHMC, MIT DRC).

### CP convergence check is correctly motivated

Without it, a momentary CP excursion while the CoM is still decelerating would trigger premature lift-off. The lateral velocity convergence criterion is physically necessary.

### 6-D contacts in double support are correct

3-D point contacts cannot resist yaw. Without orientation constraints, feet drift in yaw over repeated steps. This is a standard requirement for bipedal double support.

### Setpoint-based weight shift avoids trajectory acceleration scaling

A timed smoothstep during weight shift demands acceleration scaled by `1/T²`. If T is too short, the reference acceleration exceeds friction limits. A constant setpoint with PD feedback lets the QP drive the CoM at whatever rate physics allows, naturally clipped by friction.

---

## Concerns

### 1. Extremely tight effective CP window — only 1.5% of step width

The CoP envelope is ±2.5 cm per foot, the CP margin removes 1 cm, leaving an effective window of ±1.5 cm. The CoM must move 10 cm laterally per step (half the 20 cm step width), so the robot must traverse 10 cm laterally with a ±0.15 cm tolerance at the point where it commits to single support.

Any lateral disturbance — swing leg reaction, ground contact impulse, numerical drift — that pushes the CP 1.5 cm off center during single support violates the constraint. Over the ~20+ steps required for 5 m, error accumulation is a significant risk.

** Severity:** High. This margin is comparable to MuJoCo's contact solver tolerance and numerical noise.

### 2. Swing-leg angular momentum is unquantified and threatens the CP margin

The document acknowledges swing-leg reaction forces and deprioritizes swing-xy tracking (weight 0.1) during transit, but provides no analysis of how much angular momentum the swing leg creates.

Rough estimate:
- Swing leg mass: ~5–7 kg
- Swing velocity: ~0.5 m/s lateral relative to support foot (from swing dynamics)
- Angular momentum about support foot: `L ≈ m × v × d ≈ 5 × 0.5 × 1.2 ≈ 3 N·m·s`
- CP shift: `Δx_cp ≈ L/(m·ω₀) ≈ 3/(34.13 × 3.13) ≈ 0.028 m`

This ~2.8 cm CP shift from swing-leg momentum alone nearly exhausts the entire ±1.5 cm effective CP window. The pelvis orientation task (weight 0.3–0.5) only regulates trunk orientation — it does not compensate for full-body angular momentum.

**Recommendation:** Add a quantitative estimate of swing-leg angular momentum and its expected CP impact. Consider adding an angular momentum regulation task to the QP or increasing the CP margin to account for this effect.

### 3. Contact wrench discontinuity at lift-off is unaddressed

When transitioning from double to single support, the QP removes one foot's contact constraints instantaneously. If that foot has residual contact force (e.g., 10 N vertical) at lift-off, the sudden removal creates an acceleration step of `~0.3 m/s²`, shifting the CP by roughly `0.01 m` — comparable to the 1.5 cm effective margin.

**Recommendation:** Add a transition precondition that the lifting foot's normal contact force must be near zero (e.g., < 5 N) before allowing the phase switch. Alternatively, include a smooth ramp-down of the lift-off foot's contact wrench in the QP over a few timesteps.

### 4. Minimum double-support shift distance undercounts the diagonal

The document states a minimum shift of 0.17 m, which appears to be a single-component distance. The full center-to-center distance between feet is:

```
√(0.25² + 0.20²) ≈ 0.32 m
```

Accounting for CoP envelopes (closest edges):

```
√((0.17)² + (0.15)²) ≈ 0.23 m
```

not 0.17 m. The corrected minimum double-support times are:

```
T_quintic = √(5.774 × 0.23 / 7.85) ≈ 0.41 s
T_bangbang = √(4 × 0.23 / 7.85) ≈ 0.34 s
```

This does not invalidate the approach (event-driven switching still works), but the time estimates in the document should be revised.

### 5. Quintic CoM smoothstep has velocity discontinuity at phase boundaries

The quintic smoothstep during single support assumes zero initial velocity. At the transition from `WEIGHT_SHIFT` to `SINGLE`, the CoM may still have lateral velocity as it converges to the support foot. The quintic prescription would demand instant deceleration to zero lateral velocity, which either violates friction limits or creates a tracking error transient.

**Recommendation:** Either match the quintic's initial velocity to the CoM's estimated velocity at phase entry, or drop the timed smoothstep entirely and rely on PD feedback alone (since the target is just the support foot center, PD is sufficient once the CoM is near it — which is precisely what the transition condition guarantees).

### 6. The 70/30 CoM bias during DOUBLE_SUPPORT — clarification needed

The 70/30 bias places the CoM at a point outside either individual foot's CoP envelope, but inside the combined support polygon. This is physically correct — during double support, both feet share the load and the combined CoP can be anywhere in the convex hull.

However, the document should explicitly note that the ±2.5 cm individual-foot CoP constraint only applies during single support. A reader might infer that the CoM must be within one foot's CoP envelope even during double support, which is not the case.

### 7. No forward momentum management strategy is specified

The CoM target during single support is the static support foot center. This means forward CoM velocity must go to zero during each single-support phase. Forward motion happens only during weight-transfer phases. This produces a stop-and-go gait with estimated speed:

```
v_avg ≈ step_length / (min_single_duration + T_shift + T_double_support)
      ≈ 0.25 / (0.50 + ~0.40 + ~0.05)
      ≈ 0.25 m/s
```

Time to complete 5 m: ~20 s. This is acceptable for a quasi-static milestone, but the document should state this expected speed explicitly so that the 5 m target can be assessed for feasibility within a reasonable simulation time.

---

## Cross-Reference with Existing Review

Cross-referenced with `docs/review_walking_milestone.md` (8 findings). Below: how the two reviews reinforce each other, where they diverge, and what emerges from their combination.

### Compounding Issues

**My #1 (tight ±1.5 cm CP window) × Existing #1 (min guard outlasts CP window):**

These two findings compound into a worse problem than either review states alone. My review identified that the effective CP window is only ±1.5 cm. The existing review identified that the CP traverses this 3 cm window in approximately 0.03–0.04 s during PD transient convergence, but the minimum time guard is 0.05 s. The combination means:

- CP enters the polygon at t ≈ 0.03 s
- The 0.05 s guard blocks the transition until t = 0.05 s
- By t = 0.05 s, the CP has already overshot and exited the polygon (transit time < guard duration)
- Recovery requires CP to reverse direction via PD overshoot, which is unreliable at best

**This makes the WEIGHT_SHIFT → SINGLE transition effectively impossible to trigger reliably.** The existing review's recommendation (post-entry latch or consecutive-timestep count) is essential, but my finding about swing-leg angular momentum (2.8 cm CP shift) means that even if the timing is fixed, the CP may not stay in the polygon long enough to be latched. The combined severity is higher than either review alone suggests.

**My #5 (quintic velocity discontinuity) × Existing #8 (quintic overshoot after convergence):**

Both reviews flag the quintic smoothstep during single support from different angles. I note that the quintic assumes zero initial velocity at phase entry, creating a velocity discontinuity when the CoM still has lateral velocity from the weight shift. The existing review notes that if the weight shift already converged the CoM to the target, the quintic injects unnecessary acceleration causing overshoot. These are the same failure mode: the quintic clashes with whatever state the CoM is actually in at phase entry. The solution from both reviews converges — skip the quintic when the CoM is near the target, and if used, match the initial velocity to the actual state.

**My #6 (70/30 bias clarification) × Existing #2 (biased midpoint is concretely outside polygon):**

I flagged the 70/30 bias as "needs clarification" about whether individual-foot CoP constraints apply during double support. The existing review goes further and proves the biased midpoint is **concretely outside** the next support foot's polygon by 0.025 m forward and 0.035 m lateral. This means the `DOUBLE_SUPPORT → WEIGHT_SHIFT` transition's CP-based event condition cannot fire, because the CoM (and therefore CP ≈ CoM at low velocity) is already outside the target polygon. The existing review's recommendation (time-driven DS → WS transition) is correct and necessary.

### Issues the Existing Review Caught That Mine Missed

| Existing Finding | Why It Matters | My Assessment |
|---|---|---|
| **#1 CRITICAL: Min guard outlasts CP window** | The 0.05 s guard is longer than the ~0.03–0.04 s CP transit time. Transitions can never fire. | I identified the tight margin but failed to analyze the timing. This is the single most critical issue. |
| **#3: Impossible safety abort** | The abort condition (CP > 0.05 m outside polygon) can never trigger because the event condition already requires CP inside ±0.015 m. Logical contradiction. | I did not catch this logical impossibility. The abort should be removed or redefined. |
| **#4: Missing τ_z constraint** | 6-D contacts need torsional friction limits (`|τ_z| ≤ γ·fz`). Without this, the QP can request physically impossible yaw torques. | I noted 6-D contacts are correct in principle but missed that the wrench cone implementation is incomplete. |
| **#5: Initial pose instability** | The 1.0 s BIPEDAL_INIT holds the robot in a dynamically unstable pose. With CoM 5.7 cm forward of the foot midpoint, the robot may tip over before the first transition. | I did not consider the initial condition problem, which CLAUDE.md already documented. |
| **#6: step_width formula bug** | `support_foot_y + step_width/2` places the next foot at the midline, not at the proper lateral offset. Coordinate sign error. | I did not check the footstep planner's coordinate math. |

### Issues My Review Caught That the Existing Review Missed

| My Finding | Why It Matters for the Existing Review |
|---|---|
| **#2: Swing-leg angular momentum (~2.8 cm CP shift)** | The existing review mentions swing foot drift at low weight but does not quantify the angular momentum impact on CP. The 2.8 cm shift exceeds the ±1.5 cm effective window, meaning swing momentum alone can push the CP outside the polygon during single support — even after the timing guard and convergence checks pass. This makes the CP-margin problem worse than the existing review assesses. |
| **#3: Contact wrench discontinuity at lift-off (~1 cm)** | Not in the existing review. A 10 N residual contact force dropped instantaneously produces a ~1 cm CP perturbation, which is significant at the ±1.5 cm margin. This acts on the same scale as the min-guard timing problem. |
| **#4: Diagonal shift distance (0.17 → 0.23 m)** | The existing review does not challenge the 0.17 m figure. The correct diagonal is ~0.23 m, making the minimum double-support time 0.41 s (quintic) not 0.35 s. This makes weight shift slower than the document assumes, but doesn't affect the event-driven approach. |
| **#7: Walking speed (~0.25 m/s, ~20 s for 5 m)** | The existing review does not estimate the expected walking speed. Since the CoM stops during single support, the gait is inherently stop-and-go. Stating the expected speed explicitly allows verification that 5 m is achievable within a reasonable simulation time. |

### Combined Priority

Merging both reviews and compounding effects:

| Priority | Issue | Source | Compounds With |
|---|---|---|---|
| 1 | Min guard (0.05 s) > CP transit time (0.03 s): transitions cannot fire reliably | Existing #1 | My #1 (tight margin), My #2 (swing momentum) |
| 2 | Biased midpoint outside next foot polygon: DS → WS event condition impossible | Existing #2 | My #6 (ambiguity about per-foot vs combined constraints) |
| 3 | Swing-leg angular momentum shifts CP by ~2.8 cm, exceeding ±1.5 cm window | My #2 | Existing #1 (narrow time window makes this worse) |
| 4 | Safety abort is logically impossible (contradicts its own precondition) | Existing #3 | — |
| 5 | Missing τ_z torsional friction constraint in QP wrench cones | Existing #4 | — |
| 6 | Contact wrench discontinuity at lift-off | My #3 | Existing #1, My #1 |
| 7 | Initial pose unstable for 1.0 s BIPEDAL_INIT | Existing #5 | — |
| 8 | Quintic velocity mismatch / overshoot at SINGLE phase entry | My #5 + Existing #8 | — |
| 9 | Minimum shift distance underestimated (0.17 → 0.23 m) | My #4 | — |
| 10 | step_width coordinate sign error | Existing #6 | — |
| 11 | No stated walking speed (~0.25 m/s) | My #7 | — |

The top 3 priorities form a dependency chain: the CP window is so narrow (My #1) that the min guard timing problem (Existing #1) makes transitions unreliable, and even if timing is fixed, swing momentum (My #2) can push the CP out of the window during single support. All three must be addressed together.

---

## Recommended Solution: GRF-Based Transitions

The fundamental problem is that **CP-based spatial transitions are the wrong abstraction for a robot with ±2.5 cm lateral feet**. The margin is too tight for dynamic effects (swing momentum shifts CP by ~2.8 cm, exceeding the ±1.5 cm window), the min guard timing doesn't work (0.05 s > 0.03 s CP transit time), and the biased midpoint places the CoM outside the target polygon during double support. Patching individual issues won't fix the architectural mismatch.

The integrated solution replaces CP-based transitions with **ground reaction force (GRF) thresholds** and addresses all 11 issues in the priority table:

### Transition logic: GRF instead of CP

| Transition | Old (broken) | New |
|---|---|---|
| INIT → WEIGHT_SHIFT | Time (1.0 s) | Time (**0.1 s**) — also fixes unstable initial pose |
| WEIGHT_SHIFT → SINGLE | CP inside polygon + converging | **GRF on support foot > 80% mg** + 0.05 s since GRF > 50% mg |
| SINGLE → DOUBLE_SUPPORT | Swing foot touchdown + CP inside polygon | Swing foot z < threshold + xy tolerance + **GRF on swing foot > 10% mg** |
| DOUBLE_SUPPORT → WEIGHT_SHIFT | CP inside next polygon + converging | **Time-driven: 0.15 s** (no spatial condition — weight hasn't shifted yet) |

Why GRF works and CP doesn't:
- GRF directly measures weight distribution — is the load actually on the foot?
- GRF is robust to swing-leg momentum (the force measurement includes all dynamic effects)
- GRF transitions are monotonically increasing during weight shift (50% → 80%), preventing chatter
- CP constraints stay in the QP as hard inequality constraints where they belong — restricting what torques the optimizer can command, not when gait transitions happen

### Other fixes

| Fix | Addresses |
|---|---|
| Add τ_z constraint: `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` | Issue #5 |
| GRF < 5 N precondition before lift-off | Issue #6 (contact wrench discontinuity) |
| Drop quintic during single support — PD-only setpoint | Issue #8 |
| Correct minimum shift distance to 0.23 m (diagonal) | Issue #9 |
| Use absolute lateral positions for footstep targets | Issue #10 |
| Redefine safety abort: CP outside convex hull of both polygons > 0.05 m | Issue #4 |
| Document expected walking speed (~0.25 m/s, ~20 s for 5 m) | Issue #11 |
| CP conditions kept only as safety abort — not for routine transitions | Issues #1, #2, #3 |

---

## Summary of Recommendations

| # | Issue | Action | Status |
|---|-------|--------|--------|
| 1 | **[CRITICAL] Min guard outlasts CP transit time** | **Replaced by GRF-based transitions — no CP timing problem** | Resolved |
| 2 | **[CRITICAL] Biased midpoint outside target polygon** | **DS → WS is now time-driven; no CP check at this stage** | Resolved |
| 3 | Effective CP window (±1.5 cm) + swing momentum (~2.8 cm) | CP constraints stay in QP as hard constraints; transitions use GRF which naturally accounts for momentum | Resolved |
| 4 | Impossible safety abort | Redefined: CP outside convex hull of both polygons > 0.05 m | Resolved |
| 5 | Missing τ_z torsional friction constraint | Add `|λ_τz| ≤ (μ·min(w,L)/2)·λ_fz` per foot | Pending |
| 6 | Contact wrench discontinuity at lift-off | GRF < 5 N precondition for lift-off transition | Resolved |
| 7 | Unstable initial pose for 1.0 s INIT | Reduce init_duration to 0.1 s | Pending |
| 8 | Quintic velocity discontinuity at phase boundaries | Drop quintic — PD-only setpoint during single support | Pending |
| 9 | Minimum shift distance underestimated (0.17 → 0.23 m) | Correct diagonal calculation; revise time estimates | Pending |
| 10 | step_width formula coordinate error | Use absolute lateral positions | Pending |
| 11 | No stated walking speed expectation | Add estimated speed (~0.25 m/s, ~20 s) | Pending |