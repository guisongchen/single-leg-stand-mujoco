# 机器人运动控制 —— 项目经历（简历版）

> **项目**：Unitree G1 人形机器人单腿站立与周期性行走控制
> **角色**：控制算法开发 / 仿真验证
> **平台**：MuJoCo 物理仿真
> **机器人**：Unitree G1（23 关节 + 6 浮动基座，29 DOF，约 34 kg）
> **周期**：持续迭代（ walking 控制器从 Stage 0 推进至 Stage 3）

---

## 一、一句话总结

基于 **QP-based Whole-Body Control（WBC）** 框架，在 MuJoCo 中为人形机器人 G1 实现了从双足站立、单腿站立到周期性行走的完整运动控制管线，覆盖动力学建模、摩擦锥约束、摆动足轨迹规划、状态机设计与仿真调参。

---

## 二、核心技术栈

| 类别 | 技术/工具 |
|------|----------|
| 物理仿真 | MuJoCo 3.x |
| 优化求解 | OSQP（二次规划） |
| 控制方法 | QP-WBC、摩擦锥/CoP 约束、力矩恢复、状态机（FSM） |
| 规划算法 | Capture Point（CP）落脚点规划、五次多项式摆动轨迹、CoM 转移规划 |
| 语言/工具 | Python、`uv`、NumPy/SciPy、YAML 配置化、Git |
| 机器人 | Unitree G1 23-DOF + 浮动基座 |

---

## 三、核心职责

1. **全身控制器（WBC）实现**
   - 构建 QP-WBC，决策变量为 `[qacc; lambda_feet]`（广义加速度 + 足端接触力）。
   - 加入浮动基座动力学等式约束、线性化摩擦锥与 CoP 边界不等式约束。
   - 实现解析力矩恢复：`tau = (M*qacc + h - ΣJ_i^T λ_i)[6:]`，避免使用 `qfrc_constraint` 导致的未建模接触污染。

2. **行走状态机设计**
   - 设计 5 相 FSM：`BIPEDAL_INIT → WEIGHT_SHIFT → SINGLE_SUPPORT → DOUBLE_SUPPORT`。
   - 实现 GRF 事件驱动的相位切换与 CP 安全门限。
   - 处理单支撑/双支撑期的约束切换（6-DOF 硬约束 ↔ 3-DOF 摩擦锥 + 软位置跟踪）。

3. **规划器开发**
   - 实现基于 Capture Point 的 footsteps 调整。
   - 实现五次多项式摆动足轨迹（lift-hold-descent），终端速度/加速度为零。
   - 实现 CoM 平滑转移规划器用于 weight-shift 阶段。

4. **仿真验证与诊断**
   - 编写端到端测试脚本：`test_walking.py`、`test_single_to_double.py`、`debug_feasibility.py`。
   - 建立诊断流程：先验证初始静态平衡，再检查接触模型，最后调控制器。
   - 监控 QP 求解时间、支撑足滑移、足间隙、躯干姿态等关键指标。

---

## 四、技术亮点与量化成果

| 成果 | 指标/说明 |
|------|----------|
| 双足站立稳定性 | 可长期保持；CoM RMSE < 0.02 m；足底接触力 > 10 N |
| 单腿站立 | 稳定保持 **30 秒**；torso RMS roll/pitch < 5° |
| 行走阶段 | 完成 Stage 1（静态交替支撑）→ Stage 3（小步前进，step_length = 0.10 m） |
| 求解实时性 | OSQP 平均求解时间约 2292 µs，p99 约 2742 µs（识别出超出 2 ms 实时预算的风险） |
| 摩擦锥建模 | 单支撑期采用 3-DOF 摩擦锥 + 软位置跟踪，解决硬 6-DOF 约束导致的支撑足弹跳问题 |
| 关键瓶颈定位 | 量化离散球体足底扭转摩擦上限仅约 **7 Nm**，识别出偏航漂移 (~93°/步) 的结构性根因 |

---

## 五、解决过的关键问题

### 1. 初始姿态不稳定（物理先于控制）
- **现象**：仿真启动后机器人立即前倾跌倒。
- **根因**：初始参考姿态 CoM 位于双脚中点前约 **5.7 cm**，不是静态平衡。
- **解决**：修正初始关节角并自动调整基座高度，使足底刚好触地。
- ** takeaway**：形成“先验证物理与初始条件，再调控制器”的调试方法论。

### 2. 硬接触约束与 MuJoCo 软接触的冲突
- **现象**：单支撑期支撑足向上漂移 5–10 mm，最终离地。
- **根因**：QP 假设无限刚性接触，而 MuJoCo 使用弹簧-阻尼接触（`solref=[0.01, 1.0]`）。
- **解决**：单支撑期改用 **3-DOF 摩擦锥 + 软位置跟踪**（weight=1000, kp=400, kd=40），与仿真接触模型协作而非对抗。

### 3. 单支撑期偏航漂移
- **现象**：每步单支撑期累积约 **93°** 偏航。
- **根因**：离散球体足底的扭转摩擦预算不足（估算仅 ~7 Nm），无法抵消摆动腿角动量。
- **成果**：通过定量分析识别出这是结构性瓶颈，而非单纯参数问题，为后续设计角动量前馈/ torso yaw 反馈提供方向。

### 4. 双支撑期落地足控制
- **现象**：摆动足落地不可靠或反弹。
- **解决**：落地足采用硬 XY + 姿态约束、自由 Z，并加入软 Z 下降任务（kp=80, kd=10, weight=500），避免硬 Z 约束与未建立接触的冲突。

---

## 六、可迁移能力

- **模型基础动力学**：熟练处理浮动基座机器人动力学、接触力建模、摩擦锥与 CoP 约束。
- **QP-based WBC**：能够从头构建和调试全身控制器，理解等式/不等式约束、正则化、力矩恢复等关键环节。
- **状态机与步态规划**：具备人形机器人行走 FSM、落脚点规划、摆动足轨迹设计的实战经验。
- **仿真调试能力**：善于用诊断脚本隔离问题，区分物理问题、参数问题与控制器问题。
- **代码组织**：遵循“Run → Crunch → Judge → Print → Save → Wire”的测试脚本分离原则。

---

## 七、简历描述建议（可直接复制使用）

### 中文版本

> **Unitree G1 人形机器人运动控制**
> - 基于 MuJoCo 搭建 G1（29 DOF）仿真环境，实现 QP-based Whole-Body Control 控制器，求解 `[qacc; λ_feet]` 并恢复关节力矩。
> - 设计 5 相行走状态机，实现双足站立、单腿站立（30 s）及小步前进（step_length=0.10 m）的步态控制。
> - 引入摩擦锥/CoP 约束、CP 落脚点规划、五次多项式摆动轨迹与 GRF 事件驱动切换，构建完整 walking 管线。
> - 定位并解决硬接触约束导致的支撑足弹跳问题，单支撑期改用 3-DOF 摩擦锥 + 软位置跟踪，提升稳定性。
> - 量化分析离散球体足底扭转摩擦上限（约 7 Nm），识别偏航漂移的结构性瓶颈，指导后续角动量控制设计。

### English Version

> **Unitree G1 Humanoid Robot Motion Control**
> - Built a MuJoCo simulation pipeline for Unitree G1 (29 DOF) and implemented a QP-based Whole-Body Control framework, optimizing over `[qacc; contact_forces]` and recovering joint torques analytically.
> - Designed a 5-phase walking state machine achieving stable bipedal stance, 30-second single-leg balance, and small-step walking (step length 0.10 m).
> - Integrated friction-pyramid / CoP constraints, Capture-Point footstep planning, quintic swing-foot trajectories, and GRF-triggered phase transitions.
> - Diagnosed and resolved support-foot bouncing caused by rigid 6-DOF contact constraints against MuJoCo's soft contacts; switched single-support to 3-DOF friction cone + soft position tracking.
> - Quantified torsional friction budget (~7 Nm) of discrete spherical foot contacts, identifying yaw drift as a structural limitation and informing future angular-momentum control design.

---

## 八、相关文件索引

| 文件 | 说明 |
|------|------|
| `controllers/base_qp_wbc.py` | QP-WBC 基类 |
| `controllers/walking_controller.py` | 行走 FSM + QP-WBC |
| `controllers/transition_controller.py` | 双足↔单腿过渡控制器 |
| `planners/footstep_planner.py` | CP 驱动落脚点规划 |
| `planners/swing_foot_planner.py` | 五次多项式摆动轨迹 |
| `planners/com_planner.py` | CoM 转移规划 |
| `scripts/test_walking.py` | 行走端到端测试 |
| `scripts/test_single_to_double.py` | 单腿→双足过渡测试 |
| `docs/project_retrospective.md` | 完整项目复盘 |
| `CLAUDE.md` | 项目技术约束与调试方法论 |
