# G1 单腿站立与行走控制项目复盘报告

> 分支：`feat/walking`  
> 日期：2026-06-15  
> 记录范围：Phase 1（双足站立）→ Phase 3（单腿站立）→ Walking Stage 0~3（周期性行走）

---

## 1. 项目概述

### 1.1 目标

为 Unitree G1 人形机器人在 MuJoCo 仿真中实现稳定的**单腿站立**与**周期性行走**控制。项目以 QP-based Whole-Body Control（WBC）为核心，希望通过物理一致的动力学优化直接生成关节力矩，避免简单的 Jacobian-transpose 或纯 PD 控制。

### 1.2 机器人模型

| 参数 | 数值 |
|------|------|
| 机器人 | Unitree G1 |
| 自由度 | 23 关节 + 6 浮动基座，`nv = 29` |
| 驱动数 | 23（`mjTRN_JOINT`，直接力矩控制） |
| 质量 | 约 34.13 kg |
| 仿真步长 | 0.002 s（500 Hz） |
| 足底几何 | 4 个半径 5 mm 球体，位于 (−5 cm, ±2.5 cm) 与 (+12 cm, ±3 cm) |
| 摩擦系数 | 0.8 |

### 1.3 项目起点

初始参考姿态（`knee=0.15 rad`，`ankle_pitch=-0.08 rad`）在仿真中会**立即向前倾倒**。根本原因不是控制器参数，而是初始 CoM 位于双脚中点前约 **5.7 cm**，姿态本身不是静态平衡。这一发现奠定了整个项目的调试基调：**先验证物理与初始条件，再调控制器**。

---

## 2. 技术架构

### 2.1 核心控制范式：QP-WBC

`controllers/base_qp_wbc.py` 中实现的 `QPWBCController` 是整条控制管线的根基。

- **决策变量**：`[qacc (nv); lambda_feet]`，即广义加速度与足端接触力。
- **求解器**：OSQP。
- **等式约束**：
  - 浮动基座动力学：`M[0:6] @ qacc + h[0:6] = sum(J_i[0:6]^T @ lambda_i)`。
  - 支撑足运动学约束（可选硬约束或软跟踪）。
- **不等式约束**：
  - 线性化摩擦锥：`|fx|, |fy| <= mu * fz`。
  - CoP 边界：`|tx| <= (W/2) * fz`，`|ty| <= (L/2) * fz`。
  - 扭转摩擦（仅在双支撑期使用）：`|tau_z| <= mu * min(W/2, L/2) * fz`。
- **力矩恢复**：

  ```
  tau = (M @ qacc + h - sum(J_i^T @ lambda_i))[6:]
  ```

  其中 `h = qfrc_bias - qfrc_passive`。这里有一个关键经验教训：**绝不能使用 `data.qfrc_constraint` 代替 `J^T * lambda`**，因为它会混入摆动足等未建模接触力，导致灾难性不稳定。

### 2.2 行走状态机

`controllers/walking_controller.py` 实现了 5 相 FSM：

```
BIPEDAL_INIT
  → WEIGHT_SHIFT_L → LEFT_SINGLE → DOUBLE_SUPPORT
  → WEIGHT_SHIFT_R → RIGHT_SINGLE → DOUBLE_SUPPORT → (repeat)
```

| 相位 | 支撑足 | CoM 目标 | 摆动足 | 切换条件 |
|------|--------|----------|--------|----------|
| BIPEDAL_INIT | 双足 6-D | 双足中点 | 无 | 0.1 s 定时器 |
| WEIGHT_SHIFT_L/R | 双足 6-D | 支撑足中心 | 无 | GRF 滞后：50% 武装 → 80% 触发，抬足侧 < 5 N |
| LEFT_SINGLE / RIGHT_SINGLE | 单足 3-D 摩擦锥 + 软位置跟踪 | 支撑足中心 | z + xy 跟踪 | 定时器 / 触地检测 |
| DOUBLE_SUPPORT | 双足 6-D | 70/30 偏置中点 | 无 | 0.15 s 定时器 |

### 2.3 规划器

| 规划器 | 文件 | 作用 |
|--------|------|------|
| FootstepPlanner | `planners/footstep_planner.py` | 基于 Capture Point（CP）调整步长与航向，输出下一步落足点 |
| SwingFootPlanner | `planners/swing_foot_planner.py` | 五次多项式生成摆动足 lift-hold-descent 轨迹，终端速度与加速度设为零 |
| ComPlanner | `planners/com_planner.py` | 五次多项式平滑转移 CoM，用于 WEIGHT_SHIFT 阶段 |

### 2.4 环境与工具

- `env/g1_env.py`：MuJoCo 环境封装，负责加载模型、自动调整基座高度、步进仿真。
- `utils/kinematics.py`：CoM、体速度、接触力、四元数/欧拉角计算。
- `scripts/test_walking.py`：端到端行走测试与评估。
- `scripts/debug_qp_consistency.py`：QP 模型与 MuJoCo 动力学一致性诊断。

---

## 3. 迭代历程与里程碑

项目大致按以下阶段推进：

| 阶段 | 核心模块 | 成果 |
|------|----------|------|
| Phase 1 | `controllers/bipedal_stance_controller.py` + `configs/g1_config.yaml` | 修正初始姿态，实现稳定双足站立 |
| Phase 2 | `controllers/transition_controller.py` | 过渡控制器：双足 → 单腿 → 双足 |
| Phase 3 | `controllers/bipedal_stance_controller.py`（单腿模式） | 稳定单腿站立 30 秒 |
| Walking Stage 0 | `controllers/walking_controller.py` + `planners/footstep_planner.py` | 行走基础设施：FSM + FootstepPlanner 骨架 |
| Walking Stage 1 | `controllers/walking_controller.py` | 静态交替支撑（step_length=0） |
| Walking Stage 2 | `controllers/walking_controller.py` + GRF 状态机 | 原地踏步（GRF 事件驱动过渡） |
| Walking Stage 3 | `controllers/walking_controller.py` + `planners/swing_foot_planner.py` | 小步前进（step_length=0.10 m，完成 2 步） |
| Walking Tuning | `controllers/walking_controller.py` + `configs/g1_config.yaml` | 调参研究，记录于 `docs/walking_tuning_summary.md` |

### 3.1 已实现的能力

1. **稳定双足站立**：修正初始姿态后，QP-WBC 可长期保持双足站立。
2. **稳定单腿站立 30 秒**：在支撑足固定约束下， torso 姿态稳定。
3. **行走基础设施**：FSM、落脚点规划、摆动轨迹、CoM 转移全部就位。
4. **原地踏步**：GRF 事件驱动的相位切换可运行。
5. **小步前进 2 步**：在 Stage 3 目标下完成 2 步，但未达成 5 米前进目标。

---

## 4. 关键问题与解决方案

### 4.1 问题一：初始姿态不稳定

- **现象**：仿真开始后机器人立即向前倾倒。
- **根因**：初始 CoM 位于双脚中点前约 5.7 cm，参考姿态不是静态平衡。
- **解决**：调整初始关节角（如 hip 后倾补偿），并自动调整基座高度使足底刚好触地。
- **教训**：**任何控制器都无法拯救一个物理上不平衡的初始条件**。调试时必须先检查 `qfrc_bias`、接触力与静态力矩是否自洽。

### 4.2 问题二：硬 6-DOF 接触约束导致支撑足弹跳

- **现象**：单支撑期支撑足向上漂移 5-10 mm，最终离地。
- **根因**：QP 将支撑足视为无限刚性约束，但 MuJoCo 使用软弹簧-阻尼接触（`solref=[0.01, 1.0]`）。实际 `qacc_actual` 与 `qacc_des` 不匹配，产生正反馈：足抬起 → 接触力减小 → 进一步抬起。
- **尝试过的方案**：

  | 方案 | 支撑足垂直稳定性 | 偏航漂移 | 结论 |
  |------|------------------|----------|------|
  | 硬 6-DOF 约束 | 弹跳 5-10 mm | 70-90° | 基线，失败 |
  | 硬 6-DOF + Z 弹簧（kp=100-500） | 仍弹跳 | 70-100° | 不解决根本问题 |
  | 增加 MuJoCo 刚度（solref=0.005） | 不稳定 | - | 产生 QACC 警告 |
  | 纯软跟踪（weight=1000，无摩擦锥） | 稳定 | ~150° | 无摩擦导致滑移 |
  | **3-DOF 摩擦锥 + 软位置跟踪** | **稳定** | ~93° | **最佳折中** |

- **最终采用方案**：单支撑期不再硬约束支撑足 6-DOF，而是：
  - 3-DOF 摩擦锥约束（XYZ 方向力）。
  - 软位置跟踪任务：`weight=1000, kp=400, kd=40`，目标高度为地面。
  - 这样既保留了摩擦约束力，又与 MuJoCo 软接触模型“合作”而非对抗。

### 4.3 问题三：单支撑期偏航漂移

- **现象**：每步单支撑期累积约 **93°** 偏航漂移。
- **根因**：足端扭转摩擦预算不足。最大扭转力矩估算：

  ```
  tau_z_max = mu * min(W/2, L/2) * fz
            ≈ 0.8 * 0.025 * 335
            ≈ 7 Nm
  ```

  而摆动腿的角动量所需力矩远超此值。
- **尝试**：
  - 移除扭转摩擦：漂移恶化至 ~150°。
  - 重新启用扭转摩擦：仍 ~93°。
  - 增加 0.2 s settle period + CoM 任务： torso roll/pitch 稳定性显著改善（rms_roll 从 8.9° 降至 1.7°），但偏航仍不受控。
- **当前状态**：未根本解决。偏航漂移是项目从 Stage 3 继续前进的最大瓶颈。

### 4.4 问题四：双支撑期落地足下降

- **现象**：摆动足落地后无法可靠触地，或触地后反弹。
- **演进**：
  - 硬 Z 约束：不可行（QP 假设接触不存在，强约束会冲突）。
  - 自由 Z：无下降动力，落地慢。
  - 软 Z 下降任务：`kp=80, kd=10, weight=500`，部分有效。
  - 五次下降曲线：terminal velocity/acceleration 为零，避免触地冲击。
- **教训**：落地阶段需要显式的 Z 向任务，而不是依赖被动接触。

### 4.5 问题五：GRF 过渡 vs CP 过渡

- **现象**：基于 Capture Point 的过渡在 ±2.5 cm 侧向足宽下不可靠。
- **根因**：摆动腿角动量使 CP 偏移约 2.8 cm，已经接近支撑多边形边界，导致时序不匹配。
- **解决**：改用 **GRF 滞后过渡**：
  - 当支撑足 GRF 上升至 50% 体重时“武装”。
  - 当抬足侧 GRF 下降至 80% 体重以下且 < 5 N 时触发抬足。
  - CP 仅作为安全中止条件。

### 4.6 问题六：力矩恢复公式

- **错误做法**：使用 `data.qfrc_constraint` 作为接触力，代入力矩恢复。
- **错误原因**：`qfrc_constraint` 包含所有接触（包括摆动足短暂触地、环境碰撞等），而 QP 只建模了支撑足。
- **正确做法**：严格使用 QP 输出的 `lambda_i` 与对应 Jacobian 计算 `sum(J_i^T @ lambda_i)`。

---

## 5. 当前状态与风险

### 5.1 最严重的当前风险：实现路径尚未收敛

当前 walking 控制器基线已统一到一个版本，但以下几组设计决策仍待验证或回退：

| 决策点 | 当前选择 | 备选方案 | 影响 |
|--------|----------|----------|------|
| `swing_duration` / `double_support_duration` / `phase_timeout` | 当前 walking_controller.py 激进值 | 更保守的默认值 | 时序过短/过长会导致 GRF 过渡失败或支撑足滑移 |
| `_check_touchdown` | GRF 检测 | `z + vz` 运动学检测，或两者融合 | 接触不稳定时 GRF 检测可能误判 |
| 单支撑期 CoM 任务 | 稳定期后移除 CoM 任务 | 全程 2-D CoM 跟踪 | 影响 torso 稳定性与偏航漂移 |
| `swing_weights.xy` | 10 / 20 | 1 ~ 5 | 摆动足 XY 跟踪过强会拉拽支撑足 |
| `single_leg_w_cam` | 200 | 50 ~ 100 | 角动量任务权重过高可能加剧求解负担 |

**在把这些设计决策收敛到可稳定运行的组合前，Stage 3 行走无法可靠通过。** 建议先以 Stage 1（原地踏步）为基线，逐个参数回退并验证。

### 5.2 未解决的技术问题

| 问题 | 状态 | 说明 |
|------|------|------|
| 偏航漂移 | 未解决 | 每步 ~93°，扭转摩擦预算不足 |
| 支撑足滑移 | 未解决 | 重心转移期间滑移 > 0.5 m |
| 第二次 GRF 过渡失败 | 未解决 | 支撑足 < 80% mg 时提前抬足 |
| 五次下降跟踪误差 | 部分解决 | 足部 Z 偏离规划轨迹 |
| QP 求解时间监控 | 未持续 | 99 百分位需 < 0.002 s |
| 关节力矩饱和 | 未验证 | 长期行走下是否 < 150 N·m |
| 角动量前馈 | 未实现 | Stage 5 可选加强项 |
| CoM 速度阻尼 | 未实现 | 单支撑期零速度目标 |

### 5.3 评估指标与 Stage 3 目标

| 指标 | 目标 | 当前状态 |
|------|------|----------|
| 前进位移 | ≥ 0.05 m（Stage 3）/ ≥ 5.0 m（Stage 4） | 未达成 |
| 无跌倒（pelvis_z > 0.5 m，\|roll\| < 15°，\|pitch\| < 15°） | 通过 | 重心转移期间跌倒 |
| 单支撑期 torso 稳定（RMS roll/pitch < 5°） | 通过 | **通过**（~4°/1°） |
| 偏航漂移（每步累积 < 2°） | 通过 | 未通过 |
| 足间隙（> 0.02 m） | 通过 | 未通过 |
| 足 placement 误差（RMSE < 0.05 m） | 通过 | 未通过 |
| 支撑足滑移（< 0.005 m） | 通过 | 未通过 |
| 抬足侧 GRF（< 5 N） | 通过 | 未通过（第二次过渡） |
| 最小步数（≥ 2） | 通过 | **通过** |

---

## 6. 经验教训

### 6.1 调试方法论

项目逐步形成了“先物理，后控制”的调试顺序，记录在 `CLAUDE.md` 中：

1. **验证初始条件**：姿态是否静态平衡？`qfrc_bias` 与重力是否一致？
2. **检查物理求解器参数**：`solref`、`solimp`、`friction`、`timestep`。
3. **验证驱动映射**：`actuator_trnid`、`actuator_gear` 是否正确。
4. **区分静态稳定与动态稳定**：数学上稳定不等于 MuJoCo 能稳定。
5. **用诊断脚本隔离问题**：`debug_contact_params.py`、`debug_qacc.py`、`debug_static_torque.py`、`test_airborne.py`。
6. **先修物理，再调控制器**。
7. **最后实现控制器**。

### 6.2 仿真与现实的鸿沟

- **MuJoCo 软接触 ≠ 刚性接触**。QP 中的硬约束在仿真中可能表现为“对抗”，导致弹跳。
- **离散球体足底**的扭转摩擦预算极低，这是造成偏航失控的结构性原因，不是参数问题。
- 项目组已决定**停止继续针对 MuJoCo 的调参**，因为过度拟合仿真参数难以迁移到真实硬件。

### 6.3 QP-WBC 设计原则

- 将接触力作为显式决策变量，并用摩擦锥/CoP 约束，是正确方向。
- 力矩恢复必须基于 QP 内的 `lambda`，不能基于 `qfrc_constraint`。
- 硬约束用于“脚固定”任务时要谨慎；与软仿真接触配合时，软跟踪往往更鲁棒。

### 6.4 代码组织

测试脚本遵循了“Run → Crunch → Judge → Print → Save → Wire”的六阶段分离原则：

- `run_simulation`：只跑循环、收集日志。
- `compute_metrics`：列表 → 标量。
- `assess`：与阈值比较。
- `report`：格式化输出。
- `save_logs`：持久化原始数据。
- `main`：编排。

这一结构在后期诊断中非常有价值，建议继续保留。

---

## 7. 后续建议

### 7.1 短期（必须先做）

1. **收敛关键设计决策**：
   - 统一 `walking_controller.py` 中的 touchdown 检测与 CoM 任务策略。
   - 统一 `configs/g1_config.yaml` 中的时序参数。
   - 跑通 `scripts/test_walking.py`。

2. **建立基线测试**：
   - 单腿站立 30 秒必须通过。
   - 原地踏步 10 步必须通过。
   - 每次改动前跑这两个测试，防止回归。

### 7.2 中期（技术突破）

1. **偏航控制**：
   - 显式建模摆动腿角动量，并在 QP 中加入角动量约束或前馈。
   - 考虑在摆动期加入 torso yaw 反馈，或在双支撑期加入 yaw 修正。
   - 评估将足底球体改为接触面积更大的几何体（如 box/capsule）是否能提供更大扭转摩擦预算。

2. **GRF 过渡鲁棒性**：
   - 引入 GRF 平滑滤波，避免噪声导致提前抬足。
   - 结合 CP 与 GRF 的混合条件，提高对扰动的容忍度。

3. **双支撑期稳定性**：
   - 在双支撑期加入 torso yaw/roll/pitch 的主动修正任务。
   - 优化落地足下降轨迹，使触地冲击最小化。

### 7.3 长期（平台迁移）

- 考虑迁移到 **Drake** 等刚性接触仿真器，验证硬约束 WBC 在刚性接触下的表现。
- 在真实 G1 硬件上测试前，先在更真实的接触模型中验证算法，避免 MuJoCo 软接触带来的伪影。
- 如果硬件允许，使用足底力/力矩传感器反馈，替代仿真中的理想接触模型。

---

## 8. 附录

### 8.1 关键文件清单

| 路径 | 说明 |
|------|------|
| `configs/g1_config.yaml` | 机器人参数与控制增益配置 |
| `env/g1_env.py` | MuJoCo 环境封装 |
| `utils/kinematics.py` | 运动学、动力学、接触力计算 |
| `controllers/base_qp_wbc.py` | QP-WBC 基类 |
| `controllers/bipedal_stance_controller.py` | 双足站立控制器 |
| `controllers/transition_controller.py` | 单腿切换过渡控制器 |
| `controllers/walking_controller.py` | 行走 FSM + QP-WBC |
| `planners/footstep_planner.py` | CP 驱动落脚点规划 |
| `planners/swing_foot_planner.py` | 五次多项式摆动轨迹 |
| `planners/com_planner.py` | CoM 平滑过渡规划 |
| `scripts/test_walking.py` | 行走端到端测试 |
| `scripts/test_bipedal_stance.py` | 双足站立测试 |
| `scripts/debug_qp_consistency.py` | QP 一致性诊断 |
| `docs/walking_tuning_summary.md` | 调参经验总结 |
| `docs/g1_foot_constrains.md` | 足底离散接触几何分析 |
| `docs/adr/001-walking-gait-architecture.md` | 架构决策记录 |

### 8.2 常用参数

| 参数 | 值 | 位置 |
|------|-----|------|
| `wbc_w_com` | 100.0 | `configs/g1_config.yaml` |
| `wbc_w_pelvis` | 50.0 | `configs/g1_config.yaml` |
| `wbc_w_posture` | 1.0 | `configs/g1_config.yaml` |
| `wbc_reg` | 0.01 | `configs/g1_config.yaml` |
| `mu` | 0.8 | `configs/g1_config.yaml` |
| `foot_cop_x_back` | 0.05 m | `configs/g1_config.yaml` |
| `foot_cop_x_forward` | 0.12 m | `configs/g1_config.yaml` |
| `foot_cop_y_half` | 0.025 m | `configs/g1_config.yaml` |
| 软位置跟踪 kp/kd/weight | 400 / 40 / 1000 | `walking_controller.py` |
| 落地足下降 kp/kd/weight | 80 / 10 / 500 | `walking_controller.py` |

### 8.3 力矩恢复公式

```python
h = data.qfrc_bias - data.qfrc_passive
tau = (M @ qacc + h - sum(J_i.T @ lambda_i))[6:]
```

### 8.4 扭转摩擦上限估算

```python
tau_z_max = mu * min(W/2, L/2) * fz
          ≈ 0.8 * 0.025 * 335
          ≈ 7 Nm
```

---

## 9. 最新验证（2026-06-15）

### 9.1 当前代码基线

当前 `feat/walking` 分支采用统一的 walking 控制器实现，关键文件版本如下：

- `configs/g1_config.yaml`：时序参数、增益、足底几何配置
- `controllers/walking_controller.py`：5 相 FSM、 touchdown 检测、CoM 任务策略
- `planners/footstep_planner.py`：CP 驱动落脚点规划
- `planners/swing_foot_planner.py`：五次多项式摆动轨迹
- `scripts/test_walking.py`：端到端行走测试

以下是在该基线上运行的回归测试结果。

### 9.2 回归测试

#### 双足站立

脚本：`scripts/single_leg_stand/test_bipedal_stance.py`

| 检查项 | 结果 |
|--------|------|
| 仿真健康（无 NaN） | PASS |
| CoM RMSE < 0.02 m | PASS |
| 左右足漂移 < 0.005 m | PASS |
| 躯干 roll/pitch < 5° | PASS |
| 足底接触力 > 10 N | PASS |
| **OSQP 求解时间 < 2 ms** | **FAIL** |

- Solve time mean：2292 µs
- Solve time p99：2742 µs
- Budget：2000 µs

控制器逻辑保持正常，但 QP 求解速度已超出 500 Hz 实时预算，后续若上硬件需要优化求解时间或降采样。

#### 行走测试（Stage 3，step_length = 0.10 m）

脚本：`scripts/test_walking.py`

| 指标 | 数值 | 阈值 | 结果 |
|------|------|------|------|
| fell | True | False | FAIL |
| max_roll_deg | 178.78° | < 15° | FAIL |
| max_support_slip_m | 0.9788 m | < 0.005 m | FAIL |
| min_clearance_m | 0.0000 m | > 0.02 m | FAIL |
| yaw_drift_per_step_deg | 17.77° | < 2° | FAIL |
| total_steps | 1 | ≥ 2 | FAIL |
| forward_displacement_m | 0.5546 m | ≥ 0.05 m | PASS |
| rms_roll_single_deg | 2.17° | < 5° | PASS |
| rms_pitch_single_deg | 0.96° | < 5° | PASS |

关键异常：

```text
WARNING: Nan, Inf or huge value in QACC at DOF 0. The simulation is unstable. Time = 7.7020.
```

### 9.3 对当前状态的评估

当前 walking 控制器基线**双足站立仍可工作，但 Stage 3 行走仍未跑通**。这说明：

1. **当前 Stage 3 参数集本身不是一个可直接工作的解**，它与 `docs/walking_tuning_summary.md` 中“偏航漂移、支撑足滑移未解决”的结论一致。
2. **失败模式比单纯偏航漂移更严重**：支撑足滑移近 1 米、躯干翻转 178°、足间隙为 0，意味着支撑足约束与摆动足轨迹在当前参数/初始条件下失配。
3. **参数与模型/初始条件之间存在未对齐**：`double_support_duration=2.0 s`、`swing_weights.xy=10/20`、`single_leg_w_cam=200` 等调参值，可能与当前 `g1_env.py` 和初始姿态不完全兼容。

### 9.4 更新后的建议

1. **先把 Stage 1（原地踏步）跑通**：将 `walking.step_length` 改为 `0.0`，确认 5 相 FSM 能不跌倒地交替抬脚，再回退到前进参数。
2. **回退激进参数**：
   - `double_support_duration` 从 `2.0` 降到 `0.3~0.5 s`；
   - `swing_weights.xy` 从 `10/20` 降到 `1~5`；
   - `single_leg_w_cam` 从 `200` 降到 `50~100`。
3. **检查初始姿态与 `g1_env.py` 的兼容性**：确认当前 config 中的初始角度与 `_adjust_base_height` 配合后，CoM 仍在支撑多边形内。
4. **融合 touchdown 检测**：当前实现使用 GRF 检测，但接触不稳定时可恢复为 `z + vz` 运动学检测，或三者融合。
5. **最终若仍无法收敛**，应认真考虑迁移到刚性接触仿真器（Drake）或真实硬件，避免在 MuJoCo 软接触上继续收益递减的调参。

---

## 10. 结语

本项目在 MuJoCo 中成功搭建了 G1 人形机器人从双足站立到周期性行走控制的完整框架，验证了 QP-WBC 在浮动基座机器人上的可行性，也暴露了几个核心问题：

1. **仿真接触模型与 QP 刚性假设的冲突**已通过 3-DOF 摩擦锥 + 软位置跟踪得到缓解。
2. **离散球体足底的扭转摩擦预算不足**导致的偏航漂移仍是前进的最大障碍。
3. **当前 walking 控制器基线无法稳定完成 Stage 3 行走**，需要进一步回退/融合调试。

后续工作应从把 Stage 1（原地踏步）跑通开始，逐步恢复前进参数，同时监控 QP 求解时间，最终向更真实的仿真器或真实硬件迁移。
