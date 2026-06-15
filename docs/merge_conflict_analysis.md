# `feat/walking` 合并冲突分析报告

> 状态：当前存在一次未完成的 merge，`MERGE_HEAD` 指向远程 `origin/feat/walking` 的 `2ff8150`。  
> 本报告只给出选择建议，不实际修改代码。

---

## 1. 当前 merge 是什么情况

- **本地 HEAD**：`b1626f6` 这条线（`230b489` / `73d25a8` / `7c55b16` / `646aa36` / `b2d7b25`）。
- **被合并方（theirs）**：`2ff8150` —— `feat(walking): extensive tuning investigation`。
- **冲突文件**：7 个，全是 walking 核心文件：
  - `configs/g1_config.yaml`
  - `controllers/walking_controller.py`
  - `planners/footstep_planner.py`
  - `planners/swing_foot_planner.py`
  - `scripts/test_walking.py`
  - `docs/walking_architecture.md`
  - `docs/walking_implementation_plan.md`

两条线从 `a73fd2e`（merge dev）之后开始分叉：
- 本地继续修时间参数、触地检测、weight-shift XY anchor、文档。
- 远程在 `983cfbb` "port from feat/tapping" 的基础上做了 Stage 3 小步前进、CP 落脚点、3-DOF 摩擦锥 + 软位置跟踪、settle period、五次下降曲线等 extensive tuning。

**核心判断**：远程 `2ff8150` 包含了项目目前最有希望的行走控制方案（与 `CLAUDE.md` 中记录的已知最佳实践一致），因此合并时应**优先采用远程版本**，只在少数明显更稳妥的本地修复上做融合。

---

## 2. 总体合并策略

| 策略 | 说明 |
|------|------|
| **默认选 theirs（`2ff8150`）** | 对控制器主体、规划器、配置、测试脚本，远程版本代表了 tuning investigation 的结果，更完整。 |
| **保留 HEAD 的少数修复** | 仅当本地修复解决了远程版本中仍未处理的 bug 或明显缺陷时才融合。 |
| **文档直接选 theirs 或重写** | 两份 docs 冲突非常严重，建议直接采用远程版本，合并后再根据最终代码更新。 |
| **合并后必须跑回归测试** | 单腿站立 30 秒、原地踏步、小步前进三个基线。 |

---

## 3. 逐文件分析

### 3.1 `configs/g1_config.yaml`（5 处冲突）

**建议：整体选 theirs，但对个别参数可微调。**

| 冲突 | HEAD | theirs | 建议 | 理由 |
|------|------|--------|------|------|
| `swing_kd` / `swing_land_accel` | 15.0 / 0.0 | 30.0 / 10.0 | **theirs** | tuning 后认为更大阻尼和落地加速度可减少足端悬空。 |
| `single_leg_w_cam` / 新增 `landing_w_posture` / `single_leg_kd_com` | 20.0 | 200.0 + 新增 | **theirs** | 远程增强了躯干角动量控制与 CoM 阻尼；但 `single_leg_w_cam=200` 可能限制摆腿，需测试。 |
| `walking:` 时序参数 | step_length=0.25, swing_duration=1.5, DS=0.15, init=0.1, timeout=5.0 | step_length=0.10, swing_duration=0.6, DS=2.0, init=1.0, timeout=10.0 + `swing_settle_duration=0.2` | **theirs** | Stage 3 调参结果；但 `double_support_duration=2.0` 偏长，可考虑改为 0.5~1.0 s 折中。 |
| `grf_touchdown_threshold` | 0.025 | 0.10 | **theirs** | 0.10 更鲁棒，避免过早触地。 |
| `swing_weights` xy | 1.0 / 1.0 | 10.0 / 20.0 | **theirs** | 远程显著提高了摆动足 xy 跟踪权重。 |

**注意**：如果合并后发现摆动腿运动受限或 QP 求解变慢，再把 `single_leg_w_cam`、`swing_weights` 往回调。

---

### 3.2 `controllers/walking_controller.py`（25 处冲突）

这是冲突最集中的文件，按功能块给出建议：

#### A. 初始化参数（lines 67, 136, 152 等）

- theirs 新增了 `swing_settle_duration`、`swing_z_weight` 等成员。
- **建议：theirs**。settle period 是 tuning 的关键发现之一（rms_roll 从 8.9° 降到 1.7°）。

#### B. Landing 阶段是否追逐摆动足 xy（lines 255-269）

- HEAD：落地时把 `_swing_target[:2]` 不断更新为当前摆动足位置，保证 touchdown xy 检查通过。
- theirs：明确不追逐 xy，认为追逐会耦合摆腿与平衡，导致不稳定。
- **建议：theirs**。这与 `TransitionController` 中的观察一致；落地检测应只依赖 z 和 vz/GRF。

#### C. 单支撑期 CoM 任务策略（lines 319-340）

- HEAD：整个单支撑期只保留 2-D CoM（去掉 z），防止 CoM-z 任务与摆腿下降冲突。
- theirs：settle period 内保留完整 3-D CoM，settle 后完全移除 CoM 任务。
- **建议：theirs**。settle period 内需要 CoM 稳定来抑制偏航/晃动；settle 后让 swing 自由 reposition。

#### D. WEIGHT_SHIFT 阶段的足端约束（lines 639-693）

- HEAD：简单 6-DOF 速度阻尼 + lift bias。
- theirs：增加了摆动足 XY anchor（`_swing_foot_anchor`）和支撑足 Z 锚定。
- **建议：theirs**，但需确认 `_swing_foot_anchor` 在 `_setup_swing_phase` 或状态切换时已被正确初始化；若未设置，theirs 的 `if swing_anchor is not None` 会安全跳过，不影响功能。

#### E. 单支撑期支撑足建模（lines 700-777）

- HEAD：硬 6-DOF 约束 + 接触丢失时降级为软跟踪。
- theirs：**3-DOF 摩擦锥（XYZ）+ 硬方向约束 + 软位置跟踪**（kp=400, kd=40, weight=1000）。
- **建议：theirs**。这与 `CLAUDE.md` 记录的“有效方案”完全一致，解决了硬 6-DOF 导致的弹跳问题。

#### F. DOUBLE_SUPPORT 阶段（lines 783-904）

- HEAD：双足都硬 6-DOF。
- theirs：支撑足 6-DOF，落地足硬 XY + 方向 + 软 Z 下降任务。
- **建议：theirs**。软 Z 下降是落地足处理的最佳实践，避免硬 Z 约束在足未触地时产生虚假接触力。

#### G. 多足阶段接触丢失降级（lines 544-584）

- HEAD：无此函数。
- theirs：新增 `_downgrade_foot_contacts`，当某足 GRF < 5 N 时把硬约束替换为软位置跟踪。
- **建议：theirs**。这是 weight-shift 阶段防止硬约束导致 QP 不可行的保险机制。

#### H. 摆动足规划（lines 1016-1067）

- HEAD：固定步长，目标 z=ground_z。
- theirs：CP 驱动步长，目标 z=ground_z + 0.005 m（near-ground），在 DOUBLE_SUPPORT 再完成最终下降。
- **建议：theirs**。near-ground 策略消除了单支撑期意外触地反弹。

#### I. Touchdown 检测（lines 1136-1180）

- HEAD：用 `z_ok + vz_ok`，要求持续 0.1 s。
- theirs：用 `z_ok + grf_ok`（GRF > 10% mg），要求持续 0.03 s。
- **建议：融合**。采用 theirs 的 `z + GRF` 思路（更可靠），但把持续 0.03 s 改为 0.05~0.1 s，避免单帧噪声误触发。同时保留 theirs 的 `min_single_duration` 最小时间检查（比 HEAD 的 `swing_duration` 更合理）。

#### J. 扭转摩擦约束（lines 1244-1256）

- HEAD：单支撑期**跳过**扭转摩擦，认为方向任务需要 yaw torque，扭转摩擦会限制它。
- theirs：单支撑期**保留**扭转摩擦，认为它是防止偏航漂移的关键。
- **建议：theirs**。`CLAUDE.md` 明确记录：移除扭转摩擦后偏航漂移从 ~93° 恶化到 ~150°，所以必须保留。

---

### 3.3 `planners/footstep_planner.py`（6 处冲突）

- HEAD：固定步长 + 航向校正。
- theirs：CP-based 步长调整 + 航向校正。
- **建议：theirs**。但注意：theirs 的 `plan_step` 签名新增了 `cp: np.ndarray | None = None`，调用方（`walking_controller.py`）已经传入 `cp`，兼容。

---

### 3.4 `planners/swing_foot_planner.py`（1 处冲突）

- HEAD：下降阶段固定从 `z_apex` 降到 0。
- theirs：支持 `z_end` 参数，下降阶段从 `z_apex` 降到 `z_end`。
- **建议：theirs**。`z_end` 参数让 `walking_controller` 可以设置 near-ground target（+0.005 m）。

---

### 3.5 `scripts/test_walking.py`（13 处冲突）

- HEAD：仅支持 Stage 1，日志项较少，评估较简单。
- theirs：支持 Stage 1 / Stage 3，日志项丰富（foot xyz、pelvis yaw、CoM、swing target、foot placement error 等），图表更全面，并有 `assess_stage1` / `assess_stage3`。
- **建议：theirs**。这是更好的诊断和评估基线。注意 theirs 在 `main` 里不再强制把 `step_length` 设为 0，而是从 config 读取，因此测试前确保 config 里的 `step_length` 是你想测的阶段。

**需要注意的兼容点**：
- theirs 使用了 `compute_com_position`（已在 import 中加入）。
- 如果 `WalkingController` 没有暴露 `com_target` 或某些日志需要的属性，合并后运行脚本时可能会报错，需要再修。

---

### 3.6 `docs/walking_architecture.md` 与 `docs/walking_implementation_plan.md`

- 两份文档冲突块多、篇幅大，且内容与代码实现高度耦合。
- **建议：直接选 theirs**，或干脆 `git checkout --theirs docs/walking_architecture.md docs/walking_implementation_plan.md`。
- 合并完成、代码稳定后，再根据实际实现重写/更新这两份文档。不要在这个阶段花大量时间手工合并文档。

---

## 4. 推荐操作步骤

如果你决定按上述建议执行，可以按以下顺序操作：

```bash
# 1. 确认当前确实在 merge 中
git status

# 2. 对文档直接采用远程版本
git checkout --theirs docs/walking_architecture.md docs/walking_implementation_plan.md

# 3. 对规划器、测试脚本直接采用远程版本
git checkout --theirs planners/footstep_planner.py planners/swing_foot_planner.py scripts/test_walking.py

# 4. 配置文件：整体采用远程版本，后续再微调
git checkout --theirs configs/g1_config.yaml

# 5. 控制器：这是唯一需要手工处理的文件
# 打开 controllers/walking_controller.py，按本报告 3.2 节的建议逐项选择：
#   - 默认 theirs
#   - touchdown 检测可融合为 z + GRF，持续时间 0.05~0.1 s
#   - 若发现 double_support_duration=2.0 太长，在 config 中改为 0.5~1.0 s

# 6. 标记解决并提交
git add -A
git commit -m "Merge origin/feat/walking into local feat/walking"

# 7. 运行回归测试
python scripts/test_bipedal_stance.py
python scripts/test_walking.py   # step_length=0 的 Stage 1
# 再把 config step_length 改为 0.10 跑 Stage 3
```

---

## 5. 合并后的预期与风险

### 5.1 预期

- 采用远程 `2ff8150` 后，你将得到一个相对完整的 Stage 3 小步前进基线：
  - 3-DOF 摩擦锥 + 软位置跟踪 → 支撑足垂直稳定。
  - settle period → 单支撑期 torso roll/pitch 稳定。
  - CP 驱动步长 → 前进步长自适应。
  - near-ground swing + 软 Z 下降 → 减少触地反弹。

### 5.2 仍存在的风险

- **偏航漂移**：`2ff8150` 本身未解决该问题，合并后单支撑期仍可能有 ~93°/步的 yaw drift。
- **第二次 GRF 过渡失败**：远程版本中 weight-shift → single 的 GRF 条件可能仍会在第二次过渡时失败，需要进一步调试。
- **参数敏感性**：`single_leg_w_cam=200`、`swing_weights` 变大后可能导致 QP 求解变慢或摆腿僵硬。
- **代码兼容性**：`test_walking.py` 假设 `WalkingController` 暴露某些属性，合并后若属性缺失会报错。

### 5.3 合并后第一优先级

1. 跑通 `scripts/test_walking.py`（Stage 1）。
2. 确认单腿站立/原地踏步不回归。
3. 观察偏航漂移和 GRF 过渡，决定是否需要引入角动量前馈或修改足底几何。

---

## 6. 总结

这次冲突的本质是 `feat/walking` 在本地和远程两条线上并行迭代，各自对同一套控制器做了不同方向的改进。远程 `2ff8150` 已经走到了项目目前最先进的 tuning 状态，因此**合并时应以 theirs 为主，只在 touchdown 持续时间等少数点上做本地融合**。

最省事的处理方式是：

```bash
git checkout --theirs docs/walking_architecture.md docs/walking_implementation_plan.md

git checkout --theirs planners/footstep_planner.py planners/swing_foot_planner.py scripts/test_walking.py configs/g1_config.yaml

# 仅手工处理 controllers/walking_controller.py， touchdown 处做轻微融合
```

完成合并后，立刻跑回归测试验证控制器是否还能站立和踏步。
