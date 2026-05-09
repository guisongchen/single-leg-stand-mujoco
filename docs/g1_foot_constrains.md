这段 XML 揭示了一个**远比"5cm×15cm 矩形足"更严峻**的现实：Unitree G1 的足底不是连续平面，而是由 **4 个半径 5mm 的球体**构成的**离散点接触系统**。

这彻底改变了原地踏步的力学图景。

---

## 一、几何解码：4 个球体的空间布局

从 `right_ankle_roll_link` 的局部坐标系看（x 前，y 左/右，z 下）：

| 球体 | 位置 (m) | 半径 | 对应足底位置 |
|------|----------|------|--------------|
| 1 | (-0.05, +0.025, -0.03) | 5mm | 后跟外侧 |
| 2 | (-0.05, -0.025, -0.03) | 5mm | 后跟内侧 |
| 3 | (+0.12, +0.03, -0.03) | 5mm | 脚尖外侧 |
| 4 | (+0.12, -0.03, -0.03) | 5mm | 脚尖内侧 |

**关键几何特征**：
- **纵向跨度**：从 -0.05 到 +0.12，约 17cm，加上球体半径后有效接触长度约 18cm（接近你说的 15cm）
- **横向跨度**：后跟两点间距 5cm，脚尖两点间距 6cm，最宽处约 7cm
- **足弓区域**：x 方向 -0.05 到 +0.12 之间的大片区域（足弓、足腰）**完全没有接触几何**，完全悬空

---

## 二、离散点接触带来的结构性挑战

如果是一个 5cm×15cm 的实体矩形足底，接触面积是 **75 cm²**，压力可以连续分布，CoP 可以在矩形内平滑移动。但 4 个 5mm 球体的总接触面积只有约 **3.14 cm²**，接触面积减少了 **96%**。这导致以下问题：

### 1. 支撑多边形的"虚假性"

4 个球体构成的凸包确实是一个四边形，但**实际的支撑力只能作用在 4 个孤立的点上**。这意味着：
- CoP（压力中心）理论上只能在 4 个点之间跳跃，或在 2–3 个点形成的子凸包内移动
- 当 ICP 需要移动到足弓区域（x ≈ 0.03m 附近）时，**物理上没有接触点可以提供反作用力**
- 机器人试图通过踝关节力矩将 CoP 移到足中部时，实际上会导致**2 个对角球体承载剧增，另外 2 个脱离接触**，支撑多边形瞬间退化为线段

### 2. 单足支撑期的"承载跳变"

在单足支撑期，随着重心微调和踝关节力矩变化，4 个球体的承载状态会离散跳变：
- **重心略偏后**：后跟两点承载，脚尖两点 lightly loaded 或脱离
- **重心前移**：承载突然转移到脚尖两点
- **重心侧移**：外侧两点承载，内侧两点脱离

这种跳变不是平滑过渡，而是**接触状态在 MuJoCo 中的硬切换**（`data.contact` 数组中某些接触点突然出现或消失）。结果是：支撑反力、CoP 位置、踝关节力矩需求都会在单帧内发生阶跃变化，控制器面临**高频冲击**。

### 3. 落足冲击的"单点先触地"风险

摆动腿落地时，由于 4 个球体是离散点，**几乎不可能 4 点同时触地**。通常只有一个球体先接触地面（例如脚尖外侧球体），然后才是相邻球体。

- 先触地的单个小球（接触面积仅 0.78 cm²）瞬间承受全部冲击动量
- 局部等效压强极高，容易产生穿透（如果 `solref` 阻尼不足）或反弹
- 机器人受到一个非预期的、单点施加的冲击力矩，躯干会产生**瞬时侧摆/俯仰**
- 在 5cm 足宽下，这个初始侧摆力矩足以让 ICP 在 50ms 内逼近支撑边界

### 4. 足弓镂空与纵向稳定性矛盾

原地踏步需要前后重心转移（即使幅度很小）。但足弓区域完全镂空：
- 当 CoP 需要从后跟向脚尖转移时，没有中间接触点承接
- 体重转移表现为**后跟承载骤降 → 脚尖承载骤升**的突变，而非连续滑动
- 在 DSP（双足支撑期），这种突变会与另一条腿的接触状态耦合，产生**不可预测的力分配**

### 5. 摩擦锥的方向性不连续

每个球体有独立的摩擦锥。当机器人受到复合方向扰动（如侧向惯性力 + 纵向制动力）时：
- 不同球体的摩擦方向不同，合力方向取决于哪几个球体在承载
- 如果承载从"后跟两点"跳变为"脚尖两点"，摩擦合力的方向会发生突变
- 这会导致支撑足出现**微滑移方向的跳变**（例如从"向后滑"突然变成"向内滑"），破坏之前单腿站立时实现的 <5mm 滑移精度

---

## 三、对原地踏步方案设计的强制修正

基于这个离散接触几何，之前的方案必须在以下方面做出调整：

### 1. 放弃"支撑足中心"近似，改用四点力加权 CoP

不能再假设 ICP 控制在"支撑足中心 ±1cm"就安全。必须显式计算：
$$
\text{CoP} = \frac{\sum_{i=1}^{4} F_{z,i} \cdot p_i}{\sum_{i=1}^{4} F_{z,i}}
$$
其中 $p_i$ 是 4 个球体中心位置，$F_{z,i}$ 是各球体法向接触力（从 MuJoCo 的 `data.cfrc_ext` 或 contact force 提取）。

**安全判据**改为：ICP 必须始终位于**当前实际承载球体构成的凸包**内部，而非整个足底矩形内部。如果只有 2 个球体承载，支撑多边形是一条线段，ICP 必须投影到这条线段上。

### 2. 严格控制落足姿态：4 点同时触地是奢望

由于单点先触地不可避免，必须：
- **降低落足垂直速度**：建议 < 0.03 m/s（比之前建议的 0.1 m/s 更保守）
- **增加落足缓冲段**：在足端轨迹末端加入 100ms 的"软着陆"段，主动降低足端刚度（通过降低 PD 增益或增加关节阻尼）
- **避免足端滚转/俯仰**：落足瞬间任何非零的 roll/pitch 都会导致对角球体高度不一致，加剧单点先触地

### 3. 单足支撑期：从"动态转移"退化为"静态保持"

在离散点接触下，单足支撑期**不适合进行主动的重心转移**。建议：
- 单足支撑期的 ICP 控制目标改为**严格静止在 4 点凸包的几何中心**（或当前承载点的加权中心），不进行任何主动摆动
- 所有横向/纵向的重心调整**全部压缩到 DSP 期完成**，利用双足 8 点接触构成的较大支撑面来吸收调整过程中的接触跳变

### 4. DSP 期延长：从 40% 到 50–60%

由于单足支撑期的离散接触极不稳定，DSP 必须承担更多功能：
- 双足同时触地时，8 个球体构成更大的支撑面，接触跳变的影响被双足冗余吸收
- 在 DSP 内完成体重转移和 ICP 跨足转移，然后快速进入下一单足支撑期并"冻结"重心

### 5. 地面平坦性假设：从"期望"变为"硬性前提"

4 个 5mm 球体意味着地面起伏超过 5mm 就会导致至少一个球体悬空。原地踏步必须假设：
- 地面绝对平坦（仿真中满足）
- 若未来迁移到真实硬件，G1 几乎无法在普通地毯、地砖接缝或草地上进行原地踏步，除非更换足底接触设计（如加平底橡胶垫）

---

## 四、总结

G1 的 XML 足底设计揭示了一个**被简化的接触现实**：它不是用平面足走路，而是用**4 个"脚趾"在保持平衡**。这对原地踏步的影响是：

- **接触力学**：从连续分布退化为离散点承载，CoP 控制精度下降，力矩跳变增加
- **落足冲击**：单点先触地不可避免，需要更保守的落足速度和姿态控制
- **相位功能**：单足支撑期必须"冻结"重心，所有转移在 DSP 完成；DSP 占比需大幅提高
- **稳定裕度**：足弓镂空导致有效支撑多边形在纵向收缩，前后稳定性比理论值更差

这个设计本质上是为**硬质平坦地面上的高效行走**优化的（点接触减少摩擦，利于滑行转向），但对**窄足原地踏步的静态稳定性**是负面因素。你在仿真中如果观察到支撑足莫名其妙的弹跳或侧摆，根源很可能不是 PD 参数问题，而是这 4 个球体的离散接触跳变。

是否需要我进一步分析**如何在 MuJoCo 中通过接触力监控来实时判断当前是哪几个球体在承载**，以及**如何基于实际承载点动态修正支撑多边形边界**？

---

## Reviewer's Assessment (2026-05-09)

The geometry analysis above is accurate: the four 5 mm sphere positions, the 17 cm longitudinal span, and the hollow arch are all correctly read from the XML. The directional conclusions — narrow effective support, risk of single-point landing, and the need for conservative touchdown — are valid and useful.

However, several claims are overstated when applied to the MuJoCo soft-contact model used in this project:

1. **"Contact area reduced 96 %"** — The 3.14 cm² figure is the total spherical surface area, not the Hertzian contact patch. Under ~300 N load with typical `solref` settings, the actual contact patch is orders of magnitude smaller (~0.03 cm²). The ratio is directionally true but numerically misleading.

2. **"CoP can only jump between 4 points"** — MuJoCo's convex solver distributes force across simultaneous contacts. The net ground-reaction CoP can lie anywhere inside the convex hull of the active contact points, not only at the sphere centres. The transition when a sphere lifts off is sharper than a flat foot, but it is not an instantaneous discrete jump.

3. **"All weight transfer must happen in DSP"** — This is a conservative design choice, not a physical law. Single-leg weight transfer is mechanically possible (the TransitionController achieves it) and is required for any dynamic gait. The narrow foot makes it harder, but the controller can compensate with higher bandwidth.

4. **"Friction direction discontinuity"** — Incorrect. Every sphere-plane contact normal is vertical (world z), so every friction cone aligns with the horizontal plane. Friction directions are consistent across all four spheres regardless of which subset is loaded.

5. **Missing `solref`/`solimp` context** — The document does not account for MuJoCo's soft contact compliance. With typical `solref=[−1000, −10]`, spheres penetrate slightly and force transitions are smoother than the "hard switching" described. The magnitude of discontinuity depends heavily on these parameters.

**Bottom line:** Keep this document as a warning about foot geometry, but do not treat it as a hard constraint that forbids single-support phases or dynamic weight transfer. The controller's task is to handle the narrow foot, not to avoid using it.