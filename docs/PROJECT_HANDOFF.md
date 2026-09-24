# 理解A3-T机械臂手柄项目

> 我将在A3-T开启一个新的项目，A3-T是购买的一个机械臂型号，然后有其自带的sdk,urdf,模型以及有网页端的上位机。现在mentor计划把该机械臂做成手柄，暂时没有完全理解，我理解的是在机械臂末端装上手柄可以控制末端的6DOF,然后说在一个空间点的所有方向有弹簧阻尼。怎么理解这个项目，做一个这个有什么用，遥控吗还是什么

你现在的理解基本对，但“把机械臂做成手柄”通常不只是“末端装一个手柄控制 6DOF”，更准确地说，是把机械臂变成一个可被人拖动、感知和记录的“主端设备”。

### 这个项目可能是什么

常见架构是：

```text
人手移动机械臂末端
        ↓
读取末端位姿、速度、按键等
        ↓
控制另一台机械臂、机器人或仿真模型
```

这叫：

- 主从控制（Leader-Follower）
- 遥操作（Teleoperation）
- 示教器/示教设备（Teaching Device）
- 6D 输入设备
- 如果能给人施加力，则叫力反馈或触觉交互设备

例如你手握住 A3-T 末端的手柄：

- 手柄前后左右上下移动：控制从机械臂末端的位置
- 手柄旋转：控制从机械臂末端的姿态
- 末端按钮：夹爪开合、切换模式、记录轨迹等

如果主端和从端比例相同，就是 1:1 控制；也可以做缩放，例如主端移动 10 cm，从端只移动 2 cm。

### “一个空间点的所有方向有弹簧阻尼”是什么意思

这通常是在机械臂上实现一种“虚拟弹簧阻尼”：

```text
虚拟力 = -弹簧刚度 × 位移 - 阻尼系数 × 速度
```

假设给空间中设定一个中心点：

- 末端离开中心点越远，机械臂越想把它拉回去
- 移动速度越快，阻尼越大，动作越稳
- 如果在某个方向设置更大的刚度，该方向会更“硬”
- 如果刚度为 0，则该方向可以自由移动

这类似于：

- 鼠标的回中力
- 游戏摇杆的弹簧
- 机械臂的虚拟弹簧
- VR 中的触觉边界
- 机器人末端的柔顺控制

不过要注意：它不一定意味着“末端被固定在一个点上”。更可能是在做一种可调的操作手感。例如：

- 释放手柄后自动回到中心
- 限制机械臂不要跑出工作区域
- 在某条轨迹附近提供引导
- 碰到虚拟墙时产生阻力
- 让示教动作既容易操作又不至于漂移

如果只是给人提供回中力，属于“虚拟弹簧/阻尼”。如果同时读取外部力传感器并模拟环境反力，就进一步属于“力反馈”。

### 这个东西有什么用

最典型的用途是遥操作。

例如：

1. 人操作主端 A3-T；
2. 从端机械臂跟随主端动作；
3. 人可以远程操作危险、狭小或不方便进入的环境；
4. 通过弹簧阻尼、虚拟墙、限位等方式增强安全性和操作手感。

应用包括：

- 远程抓取、装配、焊接、打磨
- 危险环境作业
- 机器人示教：人手把机械臂拖到目标位置，系统记录轨迹
- 强化学习和模仿学习的数据采集
- VR/AR 中控制机器人
- 医疗、康复、实验室操作
- 多机器人协同控制
- 机械臂调试和现场操作

### 也可能不是控制另一台机械臂

“做成手柄”还有一种含义：把 A3-T 本身当成一个高精度 6D 输入设备。

例如它不一定驱动另一台机器人，而是：

- 控制仿真环境中的机械臂
- 控制电脑中的 3D 光标
- 控制相机云台
- 控制无人机或移动平台
- 采集人的操作轨迹
- 作为游戏或 VR 控制器

所以项目的核心问题是：A3-T 的末端运动最终要控制什么。

### 你们这个项目大概率包含哪些模块

可以把项目拆成几层：

1. **状态读取**
   从 SDK 获取关节角、末端位姿、速度、电流等。

2. **拖动/柔顺控制**
   让人可以安全地推动机械臂，或者让末端呈现弹簧阻尼手感。

3. **手柄输入映射**
   将末端的 6DOF 姿态转换成目标设备的控制指令。

4. **比例、坐标系和模式管理**
   例如世界坐标、基座坐标、末端坐标，以及位置缩放和姿态缩放。

5. **安全限制**
   速度限制、工作空间限制、关节限位、碰撞检测、急停。

6. **从端控制或数据记录**
   控制另一台机械臂，或者把轨迹保存下来供示教和训练使用。

### 最容易混淆的一点

“末端有弹簧阻尼”和“真正力反馈”不是一回事。

- 如果只是根据位置和速度计算电机控制量，让末端表现得像弹簧：这是虚拟柔顺/阻抗控制。
- 如果机械臂还能够检测外界接触力，并把从端环境的力传回人手：这才是更完整的双向力反馈遥操作。

你们目前描述的“在一个空间点的所有方向有弹簧阻尼”，我更倾向于理解为：

> 让 A3-T 成为一个带有虚拟回中力、阻尼和空间约束的 6DOF 主端控制器，用来拖动、示教或遥操作另一个机器人。

建议你先向 mentor 确认三个问题：

1. A3-T 是要控制另一台实体机械臂，还是控制仿真模型？
2. 人是要主动拖动机械臂，还是机械臂自动运动、手柄只发送指令？
3. 弹簧阻尼是为了回中、限位、引导轨迹，还是为了实现真实环境的力反馈？

这三个问题一旦明确，项目的方向基本就清楚了。

> 好像说如果sdk给的控制不满足，需要直接控制电机或者用MIT模式进行修改KP来调整，怎么理解

这句话的意思是：如果 SDK 只提供“移动到某个位姿 / 跟踪轨迹”这类高层接口，无法做出你们想要的“像手柄一样能被人推、带弹簧阻尼的手感”，就需要下沉到更底层的关节控制。

控制层级大致是：

```text
末端位姿命令（高层 SDK）
        ↓
关节位置 / 速度控制
        ↓
关节力矩控制（MIT 模式常在这里）
        ↓
电机电流 / 驱动器控制
```

MIT 模式通常不是“直接给电机上电压”，而是每个关节发送类似下面的命令：

\[
\tau = K_p(q_d-q)+K_d(\dot q_d-\dot q)+\tau_{ff}
\]

其中：

- `q_d`：目标关节角度
- `q`：当前关节角度
- `Kp`：像弹簧的刚度；越大，关节越努力回到目标位置
- `Kd`：像阻尼；越大，运动越不容易抖，但太大可能发黏或不稳定
- `τ_ff`：额外给定的前馈力矩，例如重力补偿、摩擦补偿或力反馈

所以，“修改 KP”就是在调机械臂的虚拟刚度。

举例：让机械臂末端停在某个空间点，但人能推开它：

- 设定该点对应的关节目标 `q_d`
- `Kp` 小一些：人容易推开，像软弹簧
- `Kp` 大一些：人推不动，更像固定住
- 加适当 `Kd`：推完后不来回振荡

但对“把机械臂当手柄”来说，常常还需要比单纯调关节 `Kp` 更进一步：在**末端笛卡尔空间**定义弹簧阻尼，即对末端的位置和姿态算一个六维力/力矩，再用雅可比矩阵转换成各关节力矩。这样才能自然表达“空间某一点的各方向都有弹簧阻尼”。

\[
\tau_{joint}=J(q)^T F_{task}+g(q)
\]

这里：

- `F_task`：末端的六维弹簧阻尼力/力矩
- `J(q)^T`：把末端力转换为关节力矩
- `g(q)`：重力补偿；没有它，机械臂会因为自身重量下垂，手感很差

也就是说，mentor 的意思大概率是：

> 先用 SDK 试能否实现拖动、重力补偿、柔顺或阻抗控制；若 SDK 的控制模式太封装，就使用 SDK 暴露的 MIT/力矩接口，自己实现弹簧阻尼与末端控制；只有在驱动器接口也不够时，才考虑更底层地直接控制电机。

直接控制电机风险很高：一旦单位、方向、关节限位、通信超时、力矩上限或重力补偿处理错，机械臂可能突然甩动。实际开发应优先顺序是：

1. 找 SDK 是否已有拖动示教、重力补偿、导纳/阻抗控制模式；
2. 使用官方支持的 MIT/力矩控制接口；
3. 只在厂家明确支持、且具备急停和完整安全限制时，才下沉到驱动器/电机层。

你可以直接问 mentor 一句来确认方向：

> “我们是计划在关节 MIT 模式下自行实现末端笛卡尔空间的阻抗控制和重力补偿，还是 SDK 已有 compliant/drag-teaching 模式，只需要调参数？”

> 那对于这么一个手柄，给使用者操作感受的要求是什么样的，什么样的模型可以满足

一个合格的“机械臂手柄”，使用者主观上应该感觉它不是一台会抢控制权的机器人，而是一个有边界、有手感的 6D 操纵杆。

最核心的体验要求是：

- **轻、可推**：人用较小的力就能带动末端，不会明显被电机、减速器或重力拖住。
- **稳、不抖**：停手后不会持续振荡；慢速操作也不爬行、卡顿。
- **跟手**：手推到哪里，系统读到的位姿就平滑地跟到哪里；控制从端时延迟尽可能小。
- **方向直觉**：向前推就是目标向前走；绕手柄转就是目标绕对应轴转。坐标系必须符合人的直觉。
- **有“边界感”**：接近工作空间、关节极限、危险区域或虚拟墙时阻力逐渐增加，而不是突然硬停。
- **可放手**：放手后末端应稳定停住或柔和回中，不能因重力而下坠。
- **安全可预测**：力矩、速度、加速度都有上限；急停和通信中断时进入安全状态。

最适合这件事的基础模型是**末端笛卡尔空间阻抗控制**，而不是只调每个关节的 `Kp`。

它相当于在末端设一个六维虚拟弹簧和阻尼器：

\[
\mathbf{F}_{cmd}
=
\mathbf{K}(\mathbf{x}_d-\mathbf{x})
+
\mathbf{D}(\dot{\mathbf{x}}_d-\dot{\mathbf{x}})
\]

再把末端的六维力/力矩转为各关节力矩：

\[
\boldsymbol{\tau}
=
\mathbf{J}^{T}\mathbf{F}_{cmd}
+
\mathbf{g}(q)
+
\boldsymbol{\tau}_{null}
\]

其中：

- \(\mathbf{x}\)：末端位置和姿态（6DOF）
- \(\mathbf{K}\)：六维刚度；决定“弹簧有多硬”
- \(\mathbf{D}\)：六维阻尼；决定“是否平稳”
- \(\mathbf{J}^{T}\)：把末端力/力矩转换为关节力矩
- \(\mathbf{g}(q)\)：重力补偿；这是“拿起来不沉”的关键
- \(\boldsymbol{\tau}_{null}\)：冗余姿态/远离关节限位等附加控制；若机械臂正好 6 轴，则通常不是重点

对于手柄，通常不是把刚度设得很高。一个更自然的设计是：

| 场景 | 手感 / 控制方式 |
|---|---|
| 正常自由操作 | 低刚度或近似零刚度 + 重力补偿 + 适量阻尼 |
| 手柄中心/初始点 | 弱弹簧回中，便于回到中立位置 |
| 靠近工作空间边界 | 刚度渐增，形成虚拟墙 |
| 要沿指定轨迹示教 | 横向刚度高、沿轨迹方向低，像“槽”一样引导 |
| 从端碰到障碍物 | 根据接触力增加反向阻力，形成力反馈 |
| 放手 | 保持当前位置或慢速、柔和回中 |

如果硬件本身不容易被人拖动——例如减速比大、摩擦大、不能可靠力矩控制——可以用另一条路线：**导纳控制（Admittance Control）**。

它不是“人推一下就直接动”，而是先测量人施加的力，再用虚拟质量-弹簧-阻尼模型算出目标运动：

\[
\mathbf{M}\ddot{\mathbf{x}}+
\mathbf{D}\dot{\mathbf{x}}+
\mathbf{K}(\mathbf{x}-\mathbf{x}_0)
=
\mathbf{F}_{human}
\]

然后由机械臂的位置控制器追踪该目标。它适合“机械臂本体偏硬，但有六维力传感器”的情况；缺点是手感往往不如真正的低阻抗力矩控制直接。

因此可以这样理解选型：

- **SDK 已有拖动示教/重力补偿/柔顺控制**：先用它，最快验证交互体验。
- **SDK 支持 MIT 力矩模式**：优先实现“重力补偿 + 笛卡尔阻抗控制”，这是最符合你们目标的模型。
- **只能位置控制，但有末端力传感器**：采用导纳控制。
- **可直接控制电机电流/力矩**：理论自由度最高，但安全、实时性和动力学建模要求也最高，不应作为第一步。

我会建议你们先把 MVP 定得非常具体：**末端安装手柄后，在约定工作空间内可被手拖动；放手不下坠、不明显抖动；接近边界有柔和阻力；实时输出稳定的 6DOF 位姿指令。**
这个阶段不必急着做真实力反馈。先把“轻、稳、跟手、安全”做出来，才能成为真正可用的手柄。

> 有没有做的比较好的实例，或者论文，可以给一个数学模型或者硬件配置的参考呢，我暂时不能假设末端有力传感器

<details><summary>2 previous messages</summary>

> 我会找两类可落地参考：无末端力传感器的“拖动/虚拟弹簧”控制模型，以及已有成熟产品或论文的硬件架构，重点区分是否必须依赖关节力矩传感。
>
> <details><summary>Searched the web</summary>
>
> Searched the web for `A3-T robotic arm SDK MIT mode A3-T mechanical arm ...`
>
> Searched the web for `A3-T robotic arm SDK MIT mode A3-T mechanical arm ...`
>
> Searched the web for `site:ieeexplore.ieee.org impedance control robot manipulators Hogan 1985 ...`
>
> </details>
</details>

有，而且你的前提“末端暂时没有六维力传感器”并不妨碍先做出一个很好用的主端手柄。

关键要分清两件事：

1. **虚拟弹簧、阻尼、回中、虚拟墙**：不需要末端力传感器。因为这是控制器主动施加的“手感”。
2. **感知人实际推了多大力、或把从端接触力反馈给人**：需要末端 F/T、关节力矩传感，或较可靠的无传感器力估计。

你们第一版应优先做第 1 类。

## 推荐的模型：无 F/T 的笛卡尔阻抗控制

使用关节编码器得到 \(q,\dot q\)，由 URDF 计算正运动学、雅可比矩阵和重力项即可。

令当前末端位姿为 \((p,R)\)，中心/参考位姿为 \((p_0,R_0)\)。

位置误差：

\[
e_p=p_0-p
\]

姿态误差可用旋转矩阵对数表示：

\[
e_R=\mathrm{Log}(R_0R^T)
\]

在末端空间构造六维“虚拟弹簧阻尼”：

\[
W=
\begin{bmatrix}
F\\M
\end{bmatrix}
=
\begin{bmatrix}
K_p & 0\\
0 & K_R
\end{bmatrix}
\begin{bmatrix}
e_p\\e_R
\end{bmatrix}
-
\begin{bmatrix}
D_p & 0\\
0 & D_R
\end{bmatrix}
\begin{bmatrix}
v\\\omega
\end{bmatrix}
\]

最后换算成各关节的命令力矩：

\[
\tau_{cmd}=J(q)^TW+g(q)-D_q\dot q
\]

其中 \(g(q)\) 是重力补偿。这种模型的手感就是：末端离中心越远，回拉越明显；移动越快，阻尼越大；但人始终能推开它。

这就是最值得先实现的模型。它本身不需要知道“人到底推了多少牛”，只需要让机器人按你设置的虚拟物理规律产生反作用。

如果 A3-T 的 MIT 模式允许每关节设置 \(q_d,\dot q_d,K_p,K_d,\tau_{ff}\)，但不允许直接发关节力矩，那么第一版可以退化成关节空间阻抗：

\[
\tau_i=
K_{p,i}(q_{d,i}-q_i)
+
K_{d,i}(\dot q_{d,i}-\dot q_i)
+
\tau_{g,i}
\]

它能实现“可被推动 + 有阻尼 + 回中”，但各方向手感会随姿态改变，没那么像理想的 6D 手柄。能拿到力矩接口时，优先实现上面的笛卡尔模型。

## 现成的成熟参考

Franka Panda / FR3 是很好的软件和交互参考：其官方控制接口直接支持关节阻抗和笛卡尔阻抗，官方示例还明确说明零附加力矩时由机器人补偿重力。它的内部控制频率可达 1 kHz，并在每个关节配置力矩传感器。可参考 [Franka 控制接口与阻抗参数](https://support.franka.de/docs/libfranka.html) 和 [FR3 产品手册](https://franka.de/hubfs/Product%20Manual%20Franka%20Research%203_R02210_1.4_EN.pdf?hsLang=en)。

KUKA LBR iiwa 是另一类成熟的“手推示教/柔顺协作臂”案例：它通过关节力矩传感实现接触检测、柔顺和手工引导。它说明了最终高品质体验需要什么硬件，但不代表你们第一版必须配 F/T 传感器。[KUKA LBR iiwa 官方说明](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa)

软件实现可直接研究 Mayr 与 Salt-Ducaju 的开源控制器论文：[A C++ Implementation of a Cartesian Impedance Controller for Robotic Manipulators](https://arxiv.org/abs/2212.11215)。它实现了笛卡尔阻抗、零空间关节姿态、滤波、速率限制与饱和保护，并在 KUKA iiwa 和 Franka Panda 上验证过。很接近你们之后需要的软件结构。

阻抗控制的经典理论源头是 Hogan 的 [Impedance Control: An Approach to Manipulation](https://ieeexplore.ieee.org/document/4788393)。

## 不加末端传感器时的硬件建议

第一版建议只增加必要部件：

- A3-T 本体及其官方驱动器；
- 末端轻量化握把，避免额外长杠杆和惯量；
- 握把上的“使能/死手开关”：手按住才允许运动或发送遥操作指令；
- 独立急停；
- 控制计算机与稳定、低时延通信链路；
- 清晰的机械臂工作空间、速度/力矩/关节限位配置。

初期不建议把 F/T 传感器装在末端：它会增加重量、悬臂惯量、成本和标定工作，反而可能使“小力可推”的体验变差。

## 无 F/T 情况下，能做到和做不到什么

| 能较可靠做到 | 暂时不应承诺 |
|---|---|
| 重力补偿、拖动、回中、阻尼 | 精确测量人手施加的六维力 |
| 虚拟墙、工作空间边界 | 真实接触力的高精度反馈 |
| 读取并输出末端 6DOF 指令 | 从端接触硬物时逼真的力回传 |
| 记录示教轨迹、控制仿真/从端 | 复杂装配中的稳定力控 |

以后若确实需要“从端碰到东西，主端手柄立刻变硬或推回来”，优先看 SDK 是否能读到**电机电流或关节估计力矩**。它们可用于粗略外力估计；但谐波减速器摩擦、传动间隙、负载误差会让结果不如 F/T 传感器。已有工作甚至仅用机械臂内部信号、没有末端或关节力矩传感器，完成了手推引导和插装任务，但它依赖训练数据与模型标定，不适合作为项目的起点：[Fine Robotic Manipulation without Force/Torque Sensor](https://arxiv.org/abs/2301.13413)。

你现在最值得确认的是 A3-T SDK 在 MIT 模式下到底能提供什么：`q / dq / 电流或估计力矩 / 力矩前馈 / Kp / Kd / 控制频率`。只要能拿到关节状态、允许调 MIT 的刚度阻尼，并且有可靠的重力补偿路径，就能先完成一版可用的“虚拟弹簧阻尼 6DOF 手柄”。

> 现在都是固定的KP阻抗吗，有没有adaptive或者其他什么思路以提高性能，并取得了比较先进的成果的方法或思路

<details><summary>2 previous messages</summary>

> 固定 \(K_p\) 只是最基础版本。我要检索近期的可变阻抗、稳定性约束和学习型方法，特别关注不依赖末端 F/T 传感器时哪些思路仍能成立。
>
> <details><summary>Searched the web</summary>
>
> Searched the web for `variable impedance control robot manipulation review adaptive stiffness damping learning paper 2024 ...`
>
> </details>
</details>

不是。固定 \(K_p\) 是最容易稳定和调试的基线，但“手柄”这种人机交互设备很适合做成**可变阻抗控制**（Variable Impedance Control, VIC）：刚度 \(K\)、阻尼 \(D\)，甚至虚拟质量 \(M\)，随状态、操作模式或任务阶段变化。

不过要先说结论：对你们这种“无末端 F/T 传感器的主端手柄”，最有价值、最现实的先进路线不是直接上强化学习，而是：

> 末端笛卡尔阻抗 + 基于状态的增益调度 + 被动性/能量约束 +（可选）电流残差的接触/意图估计。

它能显著改善体验，也更容易保证安全。

## 1. 最实用：状态相关的可变阻抗

把固定参数：

\[
W=K(x_0-x)-D\dot{x}
\]

改为：

\[
W=K(x,\dot{x},m)(x_0-x)-D(x,\dot{x},m)\dot{x}
\]

其中 \(m\) 是当前模式，例如“自由拖动、回中、接近边界、精确操控”。

不依赖力传感器，也能使用的位置/速度信息包括：

- 距离工作空间边界的距离；
- 距离关节极限的距离；
- 末端速度、加速度；
- 是否按住手柄使能按钮；
- 当前是否是精细操作模式；
- 是否接近奇异位形；
- 电机电流或动力学残差是否异常增大。

一个很好的手柄体验可以是：

| 状态 | 刚度 | 阻尼 | 人的感觉 |
|---|---:|---:|---|
| 按下使能、正常拖动 | 很低 | 中等 | 轻、跟手、不漂 |
| 放开使能 | 中等 | 较高 | 平稳停住或柔和回中 |
| 靠近空间边界 | 沿法向逐渐升高 | 升高 | 像碰到软墙 |
| 靠近关节限位/奇异位形 | 向安全区域升高 | 升高 | 被温和“推回来” |
| 精细对准模式 | 选定方向高、其余方向低 | 中等 | 像沿槽移动 |
| 快速运动 | 不一定增刚，但增阻尼 | 高 | 不易冲过头 |

虚拟墙可以用下面这个非常直观的模型。若 \(d\) 是末端到边界的距离，\(n\) 是边界法向量：

\[
F_{\text{wall}}=
\begin{cases}
0, & d>d_s\\
k_w(d_s-d)^2n-d_w(v^Tn)n, & d\le d_s
\end{cases}
\]

即还没接近边界时完全自由；进入安全缓冲区后，阻力平滑增大。这里不需要测量人施加的力。

## 2. 刚度可以“各向异性”，而不是六个方向一样硬

这是很值得做的一点。

比如操作员在做插入、对准或沿轨迹示教时，常希望：

- 沿目标轨迹：低刚度，方便移动；
- 偏离轨迹的横向：高刚度，防止漂走；
- 某个旋转轴：更硬或更软。

数学上，笛卡尔刚度不再只是对角矩阵常数，而可随任务坐标系变化：

\[
K=R
\begin{bmatrix}
k_{\parallel}&0&0\\
0&k_{\perp 1}&0\\
0&0&k_{\perp 2}
\end{bmatrix}
R^T
\]

姿态也有一套 \(K_R\)。这会让机械臂从“六方向都像一样的弹簧”变成“有明确引导感的操纵器”。

## 3. 阻尼不能固定随便调，应与刚度联动

如果 \(K\) 变大、\(D\) 不变，很容易出现振荡；如果 \(D\) 太大，则会感觉粘滞、迟钝。

可先按二阶系统的临界阻尼附近设置：

\[
D = 2\zeta\sqrt{M_dK}
\]

- \(M_d\)：期望虚拟质量；
- \(\zeta\)：阻尼比，初期可从约 \(0.7\sim1.0\) 的范围开始实验；
- \(K\) 增大时，\(D\) 同步增加。

实际实现中还应为 \(K,D\) 增加上下限、低通滤波和每周期最大变化量。因为“突然变硬”本身可能向人注入能量，造成不稳定。

## 4. 更先进但仍可落地：被动性 / 能量罐约束

可变阻抗真正难的不是“算出新的 \(K_p\)”，而是保证快速变参时不会抖动或失稳。

当刚度随时间变化时：

\[
K=K(t)
\]

控制器有可能主动注入能量。被动性控制（passivity control）或 Energy Tank 的思路是：维护一个虚拟能量账户，只有能量足够时才允许提高刚度、加速或输出较大的力矩；否则自动限制参数变化。

这是比“根据距离直接把 \(K_p\) 拉高”更成熟的安全层。对人直接握持的机械臂很有价值，但建议在固定阻抗和基本增益调度稳定后再加。

## 5. 无 F/T 时也可尝试“无传感器柔顺/接触估计”

如果 SDK 能读到每个关节的电流，或能给出估计力矩，可以通过动力学残差估计外力：

\[
\tau_{ext}
\approx
\tau_{motor}
-
M(q)\ddot q
-
C(q,\dot q)\dot q
-
g(q)
-
\tau_{friction}
\]

再用：

\[
W_{ext}\approx (J^T)^\dagger \tau_{ext}
\]

估计末端受到的外力/力矩。

但这不是免费的：减速器摩擦、负载变化、反向间隙和电流到力矩的标定误差都会影响结果。因此第一版应把它用于：

- 碰撞/异常接触检测；
- 轻推、重推的粗分级；
- 自适应提高阻尼或进入安全模式；

而不要直接承诺高精度力控或逼真触觉回馈。

一个很贴近你们条件的近期实例是 de Wolde 等人的工作：在没有力或力矩传感器的电流控制机械臂上，通过电流—力矩比例和摩擦标定，实现了柔顺控制与手推引导，并在 Kinova Gen3 Lite 上进行了实验验证。[Current-Based Impedance Control for Interacting with Mobile Manipulators](https://arxiv.org/abs/2403.13079)

## 6. 学习型方法：适合第二阶段，不适合第一版直接使用

研究上较先进的路线包括：

- **示教学习（LfD）**：从人示教轨迹学“到哪里该变硬、哪里该变软”；
- **强化学习（RL）**：策略输出末端目标位姿以及 \(K,D\)，而不是直接输出电机力矩；
- **逆强化学习（IRL）**：不仅模仿参数，还尝试推断“为什么专家在此处提高刚度”；
- **状态相关可变阻抗学习**：用稳定性约束或优化器限定策略输出的 \(K,D\)。

VICES 是很有代表性的思路：把末端位姿和可变阻抗共同作为策略动作，用于接触丰富任务；论文显示其在路径跟踪、开门和表面擦拭任务上改善了鲁棒性和安全性。[Variable Impedance Control in End-Effector Space](https://arxiv.org/abs/1906.08880)

另一篇使用专家示教与逆强化学习，学习随任务阶段变化的阻抗策略，并在真实 FANUC 机械臂的插装任务上验证：[Learning Variable Impedance Control via Inverse Reinforcement Learning](https://arxiv.org/abs/2102.06838)。

但这些方法一般更适用于“从端完成具体接触任务”，例如插孔、开门、擦拭；对于你们当前的**主端手柄**，先做规则型自适应会更可控。

## 我建议的项目演进

1. 固定低刚度 + 重力补偿 + 适度阻尼，先实现安全拖动。
2. 加入软工作空间边界、关节限位排斥和速度相关阻尼。
3. 加入按键切换的自由 / 回中 / 精细模式，以及方向相关刚度。
4. 加入参数限幅、滤波、速率限制和被动性保护。
5. 若 SDK 可读电流或力矩，做外力残差估计，只用于碰撞/模式切换。
6. 有了人类操作数据与明确下游任务后，再学习 \(K(x,\dot x,m)\) 和 \(D(x,\dot x,m)\)。

换句话说，目标不是“让 \(K_p\) 自动变化”本身，而是让机械臂在自由操作时轻、接近危险时稳、需要精度时有引导感，而且整个过程中不会因为变参而突然发力。

> sdk二次开发软件包和固件包的区别

两者处在不同层级：

```text
你的程序 / ROS / Python / 上位机
            ↓
SDK 二次开发包
            ↓
机械臂控制器中的固件
            ↓
驱动器、电机、编码器
```

**SDK 二次开发软件包**是给你写程序用的开发接口，通常包含：

- C++ / Python / ROS API；
- 示例程序、头文件、动态库；
- 通信协议封装；
- 文档；
- 运动、状态读取、MIT 模式等控制接口。

你在电脑上编写程序，调用 SDK，例如“读取关节角”“设置末端目标位姿”“进入 MIT 模式”“发送 \(K_p,K_d\)”。

**固件包**是运行在机械臂控制器、关节驱动器或末端设备里的底层软件，通常包含：

- 电机电流、速度、位置闭环；
- 编码器读取；
- 电机保护、过流过温保护；
- 通信解析；
- 急停、限位、故障处理；
- 某些控制模式的真正实现，例如 MIT 模式、重力补偿、碰撞检测。

固件决定“机械臂实际能做什么、怎样安全地做”；SDK 决定“你能从外部怎样调用它”。

对你们这个项目尤其要注意：

- SDK 里有 MIT 接口，不代表当前固件一定支持或启用了该功能；
- 同一个 SDK 版本常常只兼容特定固件版本；
- 更新固件可能增加控制模式或修复问题，也可能改变协议、参数范围和默认安全策略；
- 固件升级失败或版本不匹配可能让机械臂无法正常启动，因此不要为了尝试功能随意刷写。

你可以把它类比为：

- **固件**：汽车的发动机 ECU 与底盘控制；
- **SDK**：给开发者开的驾驶接口，例如油门、刹车、方向盘和仪表读数。

你现在应先确认控制器网页端显示的“控制器固件版本、关节固件版本”，再对照 SDK 的兼容性表。重点查 SDK 文档里是否明确写出：支持的固件版本、MIT 模式的命令字段、控制频率、\(K_p/K_d\)/力矩范围、通信丢失时的保护行为。

> 这俩有什么区别，下载哪一个
>
> User context
>
> Images:
> - `~/Pictures/Screenshots/Screenshot from 2026-09-23 19-35-34.png`

<details><summary>2 previous messages</summary>

> 我先确认当前开发机的系统架构；截图中的区别主要是目标系统和 CPU 架构，而不是机械臂型号或功能差异。
>
> <details><summary>Ran a command</summary>
>
> - Ran `uname -s -m`
>
> </details>
</details>

下载第一项：**Linux 系统，amd64 架构，包含 Python**。

你当前这台开发机是 `Linux x86_64`，其中 `x86_64` 就是厂商所说的 `amd64`。

三项区别：

- 第一项：Linux + amd64/x86_64，适合你的电脑。
- 第二项：Linux + arm64，适合 Jetson、树莓派 64 位、部分 ARM 工控机。
- 第三项：Windows + amd64，适合 Windows 电脑。

三者功能版本相同，差别主要是运行平台和编译出的 CPU 架构不同。不要因为机械臂控制器可能是 ARM 就下载 arm64；SDK 应按**运行你控制程序的电脑**来选。

> 现在电脑有没有解压的软件

<details><summary>2 previous messages</summary>

> 我检查一下系统里是否已有可用于 `.zip` 的命令行解压工具。
>
> <details><summary>Ran a command</summary>
>
> - Ran `command -v unzip; command -v 7z; command -v bsdtar; command -v tar`
>
> </details>
</details>

有，已安装 `unzip`，可以直接解压 ZIP 包。

下载完成后，在终端进入下载目录运行：

```bash
unzip arm_control_sdk-main.dev-1.0.260819.zip
```

若想解压到当前项目目录：

```bash
unzip ~/Downloads/arm_control_sdk-main.dev-1.0.260819.zip -d /home/user/A3-T
```

注意文件名可能与下载页面略有不同；可先在下载目录执行 `ls` 确认。

> 固件包里面这些都是什么，怎么用，deb不是下载软件的吗
>
> User context
>
> Images:
> - `~/Pictures/Screenshots/Screenshot from 2026-09-23 19-38-26.png`

对，`.deb` 确实是 Debian/Ubuntu 系统的软件安装包；但这里的“软件”不是装到你的开发电脑上的 SDK，而大概率是装到**机械臂控制器内部的 ARM Linux 系统**上的组件。

这个 ZIP 更准确说是一个“控制器软件/固件升级包集合”。它把控制器运行需要的一组 `.deb` 一起打包，避免你只升级其中一个导致版本不兼容。

从文件名可大致判断：

| 文件 | 大致作用 | 应安装的位置 |
|---|---|---|
| `arm_control-a3-can-master...deb` | A3-T 的 CAN 总线主控通信组件，负责与关节驱动器通信 | 机械臂控制器 |
| `arm_control_sdk-main...deb` | 控制器侧的 SDK/通信服务依赖 | 机械臂控制器 |
| `CarmBackend.ARM.arm64...deb` | 上位机网页端的后端服务 | 机械臂控制器 |
| `CarmDeployerWeb.ARM.arm64...deb` | 网页端部署、升级或管理界面 | 机械臂控制器 |
| `moma_common-main...deb` | 多个组件共享的基础库 | 机械臂控制器 |
| `motor_control-arm_dm...deb` | 电机/关节控制相关服务 | 机械臂控制器 |
| `arm_control_sdk...zip` | 给开发机使用的 SDK 压缩包 | 你的电脑 |

其中 `ARM.arm64` 已经明确表示：这些包是给控制器里的 ARM64 Linux 系统用的，**不能安装到你这台 x86_64/amd64 开发电脑**。

你前面应该下载的 Linux amd64 SDK ZIP，才是给当前电脑使用的。它通常解压后直接运行示例或安装 Python 依赖，不需要刷固件。

不要直接在控制器上随意执行：

```bash
sudo dpkg -i 某个包.deb
```

因为这些包彼此有依赖和升级顺序；例如 CAN 通信服务、后端服务和电机控制版本不匹配，可能导致网页端无法连接、机械臂无法使能，甚至无法正常运动。

正确用法应以厂家文档为准，通常是以下之一：

1. 在网页上位机的“固件升级/系统升级”页面上传**整个 ZIP**；
2. 用厂家提供的升级工具导入该 ZIP；
3. 厂家明确要求 SSH 到控制器后，按指定顺序安装 `.deb`。

所以这包你可以先保留，但当前先不要升级。只有当 SDK 文档明确要求某个固件版本，或厂家要求升级以支持 MIT 模式/新接口时，再按官方升级流程操作。

> 那这些固件包等于没啥用对我？因为按道理控制器应该不用我来更新？

对你当前“开发手柄控制功能”的阶段，固件包大概率**暂时没用，也不应主动更新**。

正常分工是：

- 你：在开发电脑上使用 amd64 SDK，写控制、读取状态、MIT 参数和末端位姿相关程序。
- 控制器：已预装并运行厂家交付时匹配的软件/固件。
- 固件包：用于厂家修复问题、增加新功能、处理版本兼容，或明确要求升级时才使用。

但它也不是完全无关。你需要把它当作“版本依据”保留，主要在以下情况才会用到：

- SDK 文档写明 MIT 模式或某 API 需要控制器版本 `1.0.260819` 以上；
- 当前网页端/SDK 连接报协议版本不匹配；
- mentor 或厂家要求升级；
- 机械臂有已知控制、CAN、碰撞保护或通信问题，厂商提供修复版本。

开始开发前，只需记录当前控制器网页端显示的：

- 控制器软件/固件版本；
- 关节或电机固件版本；
- SDK 版本；
- 机械臂型号与序列号。

然后核对 SDK 的兼容版本说明。若没有明确的不匹配提示，就保持“能正常运行的版本”不动。对机器人而言，稳定可复现的版本通常比盲目追新更重要。

> 检视下arm\_control\_sdk的文件夹，大致结构，以及提供了什么

<details><summary>4 previous messages</summary>

> 我会只读检查项目目录中已解压的 SDK，梳理目录、说明文件、示例和可用控制接口，不修改任何文件。
>
> <details><summary>Explored a file, 2 searches, 2 lists, ran a command</summary>
>
> - Ran `rg --files -g '*arm_control_sdk*' -g '!*' /home/user/A3-T 2>/dev/null || true
> find /home/user/A3-T -maxdepth 3 -type d -iname '*arm*control*sdk*' -print`
>
> <details><summary>Explored a file, 2 searches, 2 lists</summary>
>
> - Listed files in `arm_control_sdk`
> - Listed files in `arm_control_sdk`
> - Read `./arm_control_sdk/carm_python/README.md`
> - Searched for `^[[:space:]]*(class|struct|enum|typedef|[A-Za-z_][A-Za-z0-9_:<> ,*&]+[[:space:]]+[A-Za-z_][A-Za-z0-9_]*\()` in `arm_control_sdk`
> - Searched for `mit|impedance|torque|force|gravity|drag|teach|compliance|collision|mode|kp|kd|servo|cartesian|pose|trajectory|can` in `arm_control_sdk`
>
> </details>
>
> </details>
>
> 初步确认：这个 SDK 不只是轨迹运动接口，已经暴露了拖动模式、MIT 底层命令、关节外力矩/末端外力估计和动力学项查询。接下来我会核对这些接口的参数与单位，避免把“有接口”误解为“已可安全直接使用”。
>
> <details><summary>Explored 5 files, ran a command</summary>
>
> <details><summary>Explored 7 files</summary>
>
> - Read `./arm_control_sdk/include/arm_control_sdk/data_type_def.h`
> - Read `./arm_control_sdk/include/arm_control_sdk/carm_cobot.h`
> - Read `./arm_control_sdk/include/arm_control_sdk/carm_cobot.h`
> - Read `./arm_control_sdk/carm_python/carm/carm_kernel.py`
> - Read `./arm_control_sdk/carm_python/carm/carm_kernel.py`
> - Read `./arm_control_sdk/setup.bash`
> - Read `./arm_control_sdk/carm_python/pyproject.toml`
>
> </details>
>
> - Ran `find /home/user/A3-T/arm_control_sdk/lib/cmake -type f -maxdepth 3 -printf '%P\n' -exec sed -n '1,180p' {} \;
> printf '\n--- relevant high-level method declarations ---\n'
> sed -n '1,130p' /home/user/A3-T/arm_control_sdk/include/arm_control_sdk/carm_cobot.h
> sed -n '340,485p' /home/user/A3-T/arm_control_sdk/include/arm_control_sdk/carm_cobot.h
> printf '\n--- Python public names ---\n'
> rg -n '^    def (set_control_mode|set_ready|set_servo_enable|register_.*cbk|get_cart_external_force|get_joint_external_tau|track_joint|track_pose|emergency_stop|task_stop)' /home/user/A3-T/arm_control_sdk/carm_python/carm/carm_kernel.py`
>
> </details>
</details>

已检查完。这个 SDK 的核心能力比前面从文件名推测的更完整：它已经直接提供了**拖动模式、MIT 控制、关节力矩/末端外力估计、雅可比矩阵和动力学项查询**，很适合做你们的“机械臂手柄”项目。

目录结构如下：

```text
arm_control_sdk/
├── carm_python/                  Python SDK 源码与说明
│   ├── README.md                 最完整的 Python 接口文档
│   ├── pyproject.toml            Python 包及依赖声明
│   └── carm/
│       ├── carm.py               单臂/双臂的高层包装
│       ├── carm_kernel.py        WebSocket 协议与底层接口实现
│       └── carm_scan.py          局域网内控制器发现工具
├── include/arm_control_sdk/      C++ 头文件
│   ├── carm_cobot.h              A3 六轴单臂主要 API
│   ├── carm_dual.h               双臂 API
│   └── data_type_def.h           状态、模式、数据结构定义
├── lib/
│   ├── libarm_control_sdk.so     编译好的 C++ SDK 动态库
│   └── libmlog.so 等             SDK 运行依赖
└── setup.bash                    设置 C++ 动态库与 CMake 路径
```

SDK 有两种使用方式：

- **Python**：最适合先验证机械臂通信、状态和控制模式；通过 WebSocket 连接控制器，默认地址是 `10.42.0.101:8090`。
- **C++**：适合后续高频控制循环；编译时链接 `libarm_control_sdk.so`。执行前运行 `source setup.bash`。

Python 依赖只有 `websocket-client` 和 `zeroconf`；详细安装信息在 [README.md](./arm_control_sdk/carm_python/README.md:1)，包定义在 [pyproject.toml](./arm_control_sdk/carm_python/pyproject.toml:1)。

对手柄项目最关键的接口有：

| 能力 | SDK 接口 | 作用 |
|---|---|---|
| 初始化与安全状态 | `connect()`、`set_ready()`、`set_servo_enable()`、`emergency_stop()` | 通信、上使能、急停 |
| 状态读取 | `get_joint_pos/vel/tau()`、`get_cart_pose()` | 读取关节和末端 6DOF 状态 |
| 外力估计 | `get_joint_external_tau()`、`get_cart_external_force()` | 获取控制器估计的关节外力矩和末端六维外力 |
| 拖动模式 | `set_control_mode(3)` | 厂家内置关节拖动模式 |
| 调节拖动手感 | `set_drag_params(torque_factor, friction_compensation_factor)` | 调转矩与摩擦补偿；文档建议转矩因子约 1–1.2 |
| MIT 模式 | `set_control_mode(2)`、`low_mit_command(pos, vel, tau, kp, kd)` | 关节目标位置、速度、前馈力矩、刚度和阻尼 |
| 更底层控制 | `low_pv_command()`、`low_pf_command()`、`low_current_command()` | 位置速度、力位混合、直接关节力矩控制 |
| 动力学 | `low_get_dynamics()`、`low_get_jacobian()` | 获取惯性/科氏/重力项和雅可比矩阵 |
| 高层运动 | `track_joint()`、`track_pose()`、`move_joint/pose()` | 轨迹跟踪、点到点、笛卡尔位姿控制 |
| 示教 | `trajectory_teach()`、`trajectory_recorder()` | 录制和回放拖动轨迹 |
| 安全参数 | `get_config()`、`set_collision_config()` | 关节限制、速度限制、碰撞检测 |

最重要的两个证据在：

- MIT 指令的单位明确写为：位置 `rad`、速度 `rad/s`、前馈力矩 `N·m`，而 `kp/kd` 是每关节阻抗参数：[carm_kernel.py](./arm_control_sdk/carm_python/carm/carm_kernel.py:2538)。
- SDK 可由控制器计算重力项 \(g(q)\)、动力学项和雅可比矩阵：[carm_cobot.h](./arm_control_sdk/include/arm_control_sdk/carm_cobot.h:770)。

这意味着可以实施之前讨论的控制律：

\[
\tau =
J(q)^T\left[K(x_0-x)-D\dot{x}\right]
+
g(q)
\]

然后用 `low_current_command(tau)` 发送关节力矩，或将计算出的目标关节位置、速度、力矩和增益发送给 `low_mit_command(...)`。

但建议不要第一天就走这条路线。更安全的验证顺序是：

1. 连接后读取 `get_version()`、`get_status()`、关节状态和限位；
2. 在空旷、安全、低速条件下试厂家 `set_control_mode(3)` 的拖动模式；
3. 用 `set_drag_params()` 调整拖动轻重与摩擦补偿；
4. 验证 `get_joint_external_tau()` 和 `get_cart_external_force()` 是否在手推时有可信变化；
5. 再尝试 MIT 模式，最初使用极保守的 \(K_p,K_d\)、力矩和速度限幅；
6. 最后才实现自己的笛卡尔阻抗、虚拟墙和自适应参数。

还有一个容易误判的点：代码里虽然有 `IMPEDANCE`、`TELEOPERATION` 枚举，但 Python 中对应的 `set_impedance_mode()` 和 `set_teleoperation_mode()` 是被注释掉的，并非当前公开可调用接口。因此现阶段不要假设厂家已经提供现成的笛卡尔阻抗或主从遥操作模式；可确定可用的是 `IDLE / 点位 / MIT / 拖动 / PF` 五类公开控制模式。[控制模式定义](./arm_control_sdk/include/arm_control_sdk/carm_cobot.h:68)

整体上，这是个可行性很高的 SDK：即使不加末端 F/T 传感器，也有足够接口先完成“重力补偿拖动 + 状态采集 + MIT 调参 + 虚拟弹簧阻尼”的原型。

> 所以说我应该做的是额外写一个手柄模式，类似拖动模式，切换到这个模式就能自动实现具体功能

对，但“额外写一个手柄模式”最好理解为：在你的上位机程序中实现一个 `HANDLE_MODE` 状态，而不是立刻修改控制器固件、给 SDK 新增一种底层 FSM 模式。

SDK 已有的模式是厂家控制器底层能力：

```text
0 IDLE
1 点位
2 MIT
3 拖动
4 PF 力位混合
```

你要做的“手柄模式”是对这些能力的编排：

```text
普通模式
  ├─ 轨迹/点位控制
  └─ 手柄模式
       ├─ 厂家拖动模式：轻推、示教、记录 6DOF 位姿
       ├─ 自定义 MIT 模式：虚拟弹簧、阻尼、虚拟墙、回中
       └─ 遥操作输出：把主臂位姿映射给从臂或仿真
```

最合理的第一版是：

1. 用户按下“进入手柄模式”；
2. 程序确认急停、碰撞检测、限速和关节限位正常；
3. 保存当前末端位姿为初始参考点；
4. 切换 SDK 的 `set_control_mode(3)`，使用厂家拖动模式；
5. 配置 `set_drag_params()`，让机械臂更轻、更容易推；
6. 持续读取 `get_cart_pose()`；
7. 将末端位姿的增量转换为手柄输出，例如：
   \[
   \Delta x_{\text{cmd}} = s_p (p-p_0),\quad
   \Delta R_{\text{cmd}} = s_R (R_0^{-1}R)
   \]
8. 发送给从端机械臂、仿真器或记录为示教数据；
9. 用户松开使能开关、触发急停、超出安全范围或通信中断时，立即退出并失能/停止。

这已经是一个可用的“6DOF 机械臂手柄”。

之后，如果需求是“参考点周围各个方向有弹簧阻尼、靠边时变硬、放手回中”，仅用厂家拖动模式不够。此时在 `HANDLE_MODE` 内部再切换到 `set_control_mode(2)` 的 MIT 模式，运行你自己的实时控制循环：

```text
读 q、dq
  ↓
正运动学得到末端 x、速度 dx
  ↓
计算虚拟弹簧阻尼 / 虚拟墙 / 自适应 K、D
  ↓
Jᵀ × 末端六维力 + 重力补偿
  ↓
low_mit_command(pos, vel, tau, kp, kd)
```

因此，建议不要把“手柄模式”做成一个单一算法，而是做成一个清晰的软件模式，至少包含两个子模式：

- `DragHandle`：优先使用厂家拖动模式，快速、安全地验证操控和遥操作。
- `ImpedanceHandle`：使用 MIT，实现自定义手感、虚拟弹簧、阻尼和边界。

最终再在网页端或你自己的程序中放一个模式切换按钮，并配一个实体“按住才可操作”的使能按钮。这样用户的理解会非常直接：进入手柄模式后，机械臂不再执行既定轨迹，而成为一个有安全约束、能输出 6DOF 指令的物理控制器。

> 但是自带的上位机网页端是不是我也无法更改，只能自己做一个单独手柄模式的上位机？

对，按目前拿到的材料看，厂家自带网页上位机大概率是控制器中已经部署好的闭源服务；你没有它的前端源码、后端源码和部署工程，就不适合直接改它。

你应该做的是一个**独立的手柄控制应用**，不必重做整套上位机。

可以有三种形态：

```text
方案 A：Python 终端程序
最快，用于先验证拖动、MIT、位姿采集和手柄逻辑。

方案 B：你自己的轻量网页端
Python/C++ 控制服务 + 一个简单网页。
网页只负责“进入手柄模式、调参数、显示状态、急停”。

方案 C：独立桌面程序
例如 Qt / ROS RViz 插件。
适合后期做更专业的实时状态监控和参数面板。
```

推荐顺序是 A → B。

你的程序通过 SDK 直接连接控制器：

```text
你的手柄程序
   ↓ WebSocket / SDK
机械臂控制器 10.42.0.101:8090
   ↓
机械臂
```

厂家网页端仍然保留，用来：

- 首次配置网络和机械臂；
- 查看故障、版本、温度等原厂诊断；
- 常规点位运动和恢复；
- 必要时急停或复位。

你自己的程序只负责手柄相关能力：

- 切换 `DragHandle` / `ImpedanceHandle`；
- 读取末端 6DOF 位姿、关节状态、外力估计；
- 调节拖动参数、\(K_p\)、\(K_d\)、虚拟墙；
- 输出遥操作目标或记录示教轨迹；
- 显示当前是否使能、当前模式、通信和故障状态。

要特别避免：**厂家网页端和你的手柄程序同时对同一机械臂发送运动/MIT 指令。** 状态读取通常可以并行，但控制权应由一个程序独占。进入你的手柄模式前，应先停止网页端正在执行的任务；退出时明确停止、失能或回到 IDLE。

如果后续能向厂家拿到 `CarmDeployerWeb` 的源码、插件机制或 API 文档，才考虑把“手柄模式”嵌入原网页端。否则，独立小上位机是正常且更可控的方案。

> 那暂定方案C吧，然后这个手柄模式的代码结构以及桌面程序的代码结构怎么设计安排，需要用ros2吗

暂定方案 C 很合适：做一个独立的 C++/Qt 桌面程序，原厂网页端继续负责诊断和常规操作，你的程序专注“手柄模式”。

第一版**不需要 ROS 2**。SDK 已经直接通过 WebSocket 控制器通信，ROS 2 会额外引入节点、话题、生命周期、部署和实时性排查成本。等后续确实需要以下任一项，再接入 ROS 2：

- 控制 ROS 2 生态里的从臂、仿真器或移动机器人；
- 用 RViz / MoveIt 规划和可视化；
- 多进程、多传感器、多设备协同；
- 需要发布标准化机器人状态给别的团队模块。

第一版建议直接采用：

```text
Qt 桌面界面
     ↓ 线程安全命令队列
控制核心线程（唯一允许发送机械臂指令）
     ↓ C++ SDK / libarm_control_sdk.so
A3-T 控制器
```

不要让 Qt 的按钮回调直接调用 `low_mit_command()`；UI 线程卡顿、按钮连点或绘图卡顿都可能影响控制。界面只发送“请求切换模式/更新参数”，控制线程负责验证状态和下发指令。

建议项目结构：

```text
a3t_handle/
├── CMakeLists.txt
├── README.md
├── config/
│   ├── robot.json              控制器 IP、工具坐标系、关节限制
│   ├── safety.json             速度、力矩、工作空间与超时限制
│   └── handle_profiles.json    自由、回中、精细模式的 K/D 参数
├── third_party/
│   └── arm_control_sdk/        原厂 SDK，只引用，不修改
├── src/
│   ├── main.cpp
│   ├── app/
│   │   └── application_controller.cpp
│   ├── arm/
│   │   ├── arm_client.hpp      对原厂 CArmSingleCol 的安全封装
│   │   ├── arm_worker.cpp      独占 SDK 通信和控制循环
│   │   └── arm_state.hpp       q、dq、tau、末端位姿、故障状态
│   ├── control/
│   │   ├── mode_manager.cpp    手柄模式状态机
│   │   ├── drag_handle.cpp     使用厂家拖动模式
│   │   ├── impedance_handle.cpp# MIT / 虚拟弹簧阻尼
│   │   ├── virtual_fixture.cpp # 虚拟墙、回中、轨迹槽
│   │   └── teleop_mapper.cpp   主端位姿→从端/仿真指令
│   ├── safety/
│   │   ├── safety_monitor.cpp  限位、速度、超时、通信、故障
│   │   └── command_limiter.cpp 力矩/速度/参数变化率限制
│   ├── math/
│   │   ├── pose_math.cpp       四元数、姿态误差、坐标变换
│   │   └── impedance_math.cpp  JᵀW、重力补偿、阻抗计算
│   ├── ui/
│   │   ├── main_window.cpp
│   │   ├── status_panel.cpp
│   │   ├── handle_panel.cpp
│   │   └── safety_panel.cpp
│   └── logging/
│       └── recorder.cpp        记录状态、指令、故障和轨迹
└── tests/
    ├── test_pose_math.cpp
    ├── test_virtual_fixture.cpp
    └── fake_arm_client.cpp
```

核心状态机建议明确写出来：

```text
Disconnected
  → Connected
  → Ready / Idle
  → EnteringHandleMode
  → DragHandle 或 ImpedanceHandle
  → Exiting
  → Idle

任意状态 → Fault / EmergencyStop
```

进入手柄模式时，控制线程应依次做：

1. 检查连接、故障、伺服和急停状态；
2. 获取关节限位、速度限制、当前工具坐标；
3. 设置低速、开启碰撞检测；
4. 记录当前末端位姿为参考点；
5. 进入 `DragHandle` 或 `ImpedanceHandle`；
6. 开始固定频率读取状态、计算、限幅和发送命令；
7. 出现超限、通信超时、用户释放使能或故障时，退出到安全状态。

两个模式可先这样划分：

- `DragHandle`：调用 `set_control_mode(3)` 和 `set_drag_params()`；持续读取末端位姿，把它当作 6DOF 输入发送给仿真、从端或轨迹记录器。
- `ImpedanceHandle`：调用 `set_control_mode(2)`；使用 `low_get_jacobian()`、`low_get_dynamics()` 和 `low_mit_command()`，自行实现虚拟弹簧、阻尼、回中和虚拟墙。

Qt 界面第一版只需四块：

```text
连接状态：IP、连接、Ready、伺服、故障码
模式控制：普通 / 拖动手柄 / 阻抗手柄 / 退出
参数面板：K、D、回中、速度比例、虚拟墙、拖动参数
安全面板：使能状态、限位、急停、实时关节/末端状态
```

我建议技术栈定为：**C++17 + Qt 6 + CMake + 原厂 C++ SDK**。SDK 本身提供预编译 `.so` 和 CMake 配置文件，C++ 方案会比 Python 更适合后续 MIT 控制循环和桌面程序集成。

最重要的架构约束是：`ArmWorker` 是唯一能碰 SDK、切模式、发送运动或 MIT 指令的对象；`SafetyMonitor` 可以随时抢占它并执行停止逻辑；UI、日志和未来 ROS 2 桥接都只能读状态和提交请求。这样后面功能变复杂后仍不会失控。

> 先帮我按这个思路完成一个初稿

<details><summary>7 previous messages</summary>

> 我会在当前工作区创建一个可编译的 C++17 + Qt 桌面程序初稿：包含安全状态机、独占机械臂工作线程、拖动/阻抗模式骨架、参数配置和基础界面；不会调用真实机械臂或改动 SDK。先检查现有项目与 Qt 开发环境。
>
> <details><summary>Ran a command</summary>
>
> - Ran `git status --short
> rg --files -g 'CMakeLists.txt' -g 'README.md' -g '*.cpp' -g '*.hpp' -g '*.h' | sed -n '1,160p'
> cmake --version | head -n 1
> pkg-config --modversion Qt6Widgets 2>/dev/null || pkg-config --modversion Qt5Widgets 2>/dev/null || true`
>
> </details>
>
> 开发环境有 Qt 5.15，且工作区目前只有厂商 SDK，因此我会用 Qt 5 生成独立项目。阻抗模式会先做成“受配置开关保护的骨架”，默认不会向真机发送 MIT/力矩指令。
>
> 先搭建项目结构就行，就在A3-T下面
>
> <details><summary>Created 13 files</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+35 -0)
> - Wrote `./a3t_handle/README.md` (+32 -0)
> - Wrote `./a3t_handle/config/handle.json` (+9 -0)
> - Wrote `./a3t_handle/src/arm/arm_types.hpp` (+18 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+122 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+48 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.cpp` (+40 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.hpp` (+16 -0)
> - Wrote `./a3t_handle/src/main.cpp` (+17 -0)
> - Wrote `./a3t_handle/src/safety/safety_monitor.cpp` (+18 -0)
> - Wrote `./a3t_handle/src/safety/safety_monitor.hpp` (+10 -0)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+91 -0)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+33 -0)
>
> </details>
>
> 收到，先只保留项目骨架，不放控制逻辑、Qt 界面或任何会连接机械臂的代码。我会把刚开始生成的实现稿收回，改为 A3-T 下的目录规划、占位文件和职责说明。
>
> <details><summary>Created 10 files, edited 2 files, deleted 11 files, explored 2 lists</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+2 -35)
> - Wrote `./a3t_handle/README.md` (+17 -27)
> - Wrote `./a3t_handle/config/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/config/handle.json` (+0 -9)
> - Wrote `./a3t_handle/docs/.gitkeep` (+1 -0)
> - Wrote `./a3t_handle/src/app/.gitkeep` (+1 -0)
> - Wrote `./a3t_handle/src/arm/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/src/arm/arm_types.hpp` (+0 -18)
> - Deleted `./a3t_handle/src/arm/arm_worker.cpp` (+0 -122)
> - Deleted `./a3t_handle/src/arm/arm_worker.hpp` (+0 -48)
> - Wrote `./a3t_handle/src/control/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/src/control/handle_mode_manager.cpp` (+0 -40)
> - Deleted `./a3t_handle/src/control/handle_mode_manager.hpp` (+0 -16)
> - Wrote `./a3t_handle/src/logging/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/src/main.cpp` (+0 -17)
> - Wrote `./a3t_handle/src/math/.gitkeep` (+1 -0)
> - Wrote `./a3t_handle/src/safety/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/src/safety/safety_monitor.cpp` (+0 -18)
> - Deleted `./a3t_handle/src/safety/safety_monitor.hpp` (+0 -10)
> - Wrote `./a3t_handle/src/ui/.gitkeep` (+1 -0)
> - Deleted `./a3t_handle/src/ui/main_window.cpp` (+0 -91)
> - Deleted `./a3t_handle/src/ui/main_window.hpp` (+0 -33)
> - Wrote `./a3t_handle/tests/.gitkeep` (+1 -0)
>
> <details><summary>Explored 2 lists</summary>
>
> - Listed files in `A3-T`
> - Listed files in `a3t_handle`
>
> </details>
>
> </details>
</details>

已按“仅搭建结构”的要求完成，未保留任何 Qt、控制、连接或 MIT 实现代码。

项目骨架在 [a3t_handle](./a3t_handle)，厂商 SDK 保持独立在 `arm_control_sdk/`，不作修改。

```text
A3-T/
├── arm_control_sdk/       厂商 SDK
└── a3t_handle/            你的独立手柄项目
    ├── config/            控制器地址、安全限制、K/D 参数配置
    ├── docs/              架构、协议和测试流程
    ├── src/
    │   ├── app/           程序启动、设置和生命周期
    │   ├── arm/           唯一可访问厂商 SDK 的机械臂通信层
    │   ├── control/       拖动、MIT、阻抗、虚拟墙、遥操作映射
    │   ├── safety/        限位、看门狗、急停和模式切换校验
    │   ├── math/          位姿、四元数、雅可比和阻抗计算
    │   ├── ui/            Qt 桌面界面
    │   └── logging/       状态、指令和轨迹记录
    ├── tests/             单元测试、假机械臂客户端
    ├── CMakeLists.txt     暂为空的构建入口
    └── README.md          目录职责说明
```

结构说明见 [README.md](./a3t_handle/README.md)。下一步可以先定 `docs/` 中的模式状态机和安全流程，再开始写最小连接与状态读取程序。

> 可以，那帮我完成初稿

<details><summary>4 previous messages</summary>

> 我会把骨架扩展为可编译的第一版桌面程序：可连接、读取状态、显式进入 Ready/厂家拖动模式、退出和急停；阻抗/MIT 只保留不可执行的接口占位，避免初稿误向真机发送力矩。
>
> <details><summary>Created 11 files, edited 2 files, ran a command</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+28 -2)
> - Wrote `./a3t_handle/README.md` (+19 -1)
> - Wrote `./a3t_handle/config/handle.json` (+8 -0)
> - Wrote `./a3t_handle/src/arm/arm_types.hpp` (+22 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+105 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+45 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.cpp` (+29 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.hpp` (+15 -0)
> - Wrote `./a3t_handle/src/main.cpp` (+17 -0)
> - Wrote `./a3t_handle/src/safety/safety_monitor.cpp` (+13 -0)
> - Wrote `./a3t_handle/src/safety/safety_monitor.hpp` (+8 -0)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+88 -0)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+31 -0)
> - Ran `cmake -S . -B build && cmake --build build -j2`
>
> </details>
>
> 编译检查发现一个常见的 C++ 前置声明问题：`unique_ptr` 指向厂商类时，需要把析构函数放到包含完整 SDK 类型的实现文件中。我会做这个仅影响编译组织的修正后重新构建。
>
> <details><summary>Edited 2 files, ran a command</summary>
>
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+2 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+2 -1)
> - Ran `cmake --build build -j2`
>
> </details>
</details>

初稿已完成并成功编译，项目在 [a3t_handle](./a3t_handle)。

目前包含：

- Qt 桌面窗口：控制器 IP、模式、关节角、末端位姿、状态提示。
- 独立机械臂工作线程：唯一会调用厂商 SDK 的位置。
- 模式状态机：`Disconnected → Idle → Drag`，以及 `Fault`、`EmergencyStop`。
- 基础安全检查：连接丢失、控制器错误状态、未 Ready 时禁止进入拖动。
- 拖动模式：调用 SDK 的 `set_drag_params()` 和 `set_control_mode(3)`。
- Ready、退出手柄模式、软件急停按钮。
- 配置文件：[handle.json](./a3t_handle/config/handle.json)。

关键代码：

- [ArmWorker](./a3t_handle/src/arm/arm_worker.cpp)：SDK 通信、状态轮询、Ready/Drag/停止。
- [模式状态机](./a3t_handle/src/control/handle_mode_manager.cpp)
- [安全检查](./a3t_handle/src/safety/safety_monitor.cpp)
- [Qt 主窗口](./a3t_handle/src/ui/main_window.cpp)

构建已验证：

```bash
cd /home/user/A3-T/a3t_handle
cmake -S . -B build
cmake --build build -j
./build/a3t_handle
```

当前版本没有实现 MIT/阻抗控制，也不会发送 MIT 或关节力矩命令；下一步应先拿真实机械臂验证“连接 → Ready → 拖动 → 读取末端 6DOF 位姿 → 退出”的完整流程。

> 我感觉应该区分一下模式吧，原厂上位机是图片所示，我们是不是也该把拖动，手柄等模式和连接使能区分出来
>
> User context
>
> Images:
> - `~/Pictures/Screenshots/Screenshot from 2026-09-24 10-43-38.png`

<details><summary>8 previous messages</summary>

> 是的，应该严格区分。截图中的“连接、复位/使能、实机/仿真、位置/MIT/拖动、STOP”不是同一层的模式；“手柄模式”则应是我们应用层新增的工作模式。我会把初稿改为分层状态模型和对应界面，而不是把所有状态塞进一个枚举。
>
> <details><summary>Deleted 8 files, explored 4 files</summary>
>
> <details><summary>Explored 4 files</summary>
>
> - Read `./src/ui/main_window.cpp`
> - Read `./src/arm/arm_worker.cpp`
> - Read `./src/arm/arm_types.hpp`
> - Read `./src/control/handle_mode_manager.cpp`
>
> </details>
>
> - Deleted `./a3t_handle/src/arm/arm_types.hpp` (+0 -22)
> - Deleted `./a3t_handle/src/arm/arm_worker.cpp` (+0 -107)
> - Deleted `./a3t_handle/src/arm/arm_worker.hpp` (+0 -46)
> - Deleted `./a3t_handle/src/control/handle_mode_manager.cpp` (+0 -29)
> - Deleted `./a3t_handle/src/control/handle_mode_manager.hpp` (+0 -15)
> - Deleted `./a3t_handle/src/safety/safety_monitor.cpp` (+0 -13)
> - Deleted `./a3t_handle/src/ui/main_window.cpp` (+0 -88)
> - Deleted `./a3t_handle/src/ui/main_window.hpp` (+0 -31)
>
> </details>
>
> 而且有些写的应该读取下carm\_cobot的sdk头文件里面是否有吧，避免重复定义或者重写某些函数，例如连接使能读取等等
>
> <details><summary>Created 8 files</summary>
>
> - Wrote `./a3t_handle/src/arm/arm_types.hpp` (+29 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+144 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+48 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.cpp` (+41 -0)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.hpp` (+19 -0)
> - Wrote `./a3t_handle/src/safety/safety_monitor.cpp` (+13 -0)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+102 -0)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+34 -0)
>
> </details>
>
> 你说得对。SDK 已经提供 `connect / disconnect / is_connected / set_ready / set_servo_enable / get_status / set_control_mode / emergency_stop`，我们的代码应只做“流程编排和 UI 状态投影”，不能重新发明一套机械臂接口。尤其我会核对厂商模式枚举与注释是否一致，避免把整数模式解释错。
>
> <details><summary>Explored 2 files, a search</summary>
>
> - Read `./arm_control_sdk/include/arm_control_sdk/carm_cobot.h`
> - Read `./arm_control_sdk/include/arm_control_sdk/data_type_def.h`
> - Searched for `set_control_mode|fsm_state|set_ready|set_servo_enable|get_status|emergency_stop|task_stop|set_drag_params|low_mit_command` in `carm.py`
>
> </details>
>
> 核对结果很重要：`ArmStatus` 已经直接给出连接、伺服、控制器状态、FSM 状态和仿真标志；并且 SDK 内部 `FsmMode` 枚举与 `set_control_mode()` 的注释对“3”含义存在不一致。为避免错误解释，我会删除我们自定义的机器人模式枚举，界面直接显示厂商返回的 FSM 原始值；“拖动”仅在调用 `set_control_mode(3)` 时按该接口注释处理。
>
> <details><summary>Edited 7 files, ran a command</summary>
>
> - Wrote `./a3t_handle/src/arm/arm_types.hpp` (+2 -2)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+5 -14)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+0 -2)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.cpp` (+0 -11)
> - Wrote `./a3t_handle/src/control/handle_mode_manager.hpp` (+0 -1)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+7 -4)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+2 -1)
> - Ran `cmake --build build -j2`
>
> </details>
</details>

是，已经按这个原则调整了：程序不再自定义或猜测厂商的连接、使能、FSM 控制接口，而是直接调用 SDK，并把返回状态投影到桌面 UI。

现在分为五个独立维度：

```text
连接状态       SDK：is_connected() / ArmStatus::arm_is_connected
伺服使能       SDK：set_servo_enable() / ArmStatus::servo_status
安全状态       SDK：ArmStatus::state + 本程序的通信异常、急停记录
厂商 FSM       SDK：ArmStatus::fsm_state，原样显示
应用模式       我们自己：Monitor / Handle
```

其中只有“应用模式”是我们新增的：

- `Monitor`：仅监控/使用厂家原有模式。
- `Handle`：你们定义的手柄功能；当前底层暂时借用厂家拖动模式，后续可替换为 MIT 阻抗控制，但 UI 和逻辑状态不变。

SDK 调用的对应关系是：

| 桌面按钮 | 实际 SDK 调用 |
|---|---|
| Connect | `CArmSingleCol(...)`、`is_connected()` |
| Disconnect | `set_control_mode(0)`、`disconnect()` |
| Vendor Ready | `set_ready()` |
| Disable Servo | `task_stop()`、`set_control_mode(0)`、`set_servo_enable(false)` |
| Vendor Drag | `set_drag_params()`、`set_control_mode(3)` |
| Enter Handle | 当前也是 `set_drag_params()`、`set_control_mode(3)`，但应用模式标记为 `Handle` |
| Return to Idle | `task_stop()`、`set_control_mode(0)` |
| Emergency Stop | `emergency_stop()` |

另外发现 SDK 有一处值得警惕的不一致：

- [carm_cobot.h](./arm_control_sdk/include/arm_control_sdk/carm_cobot.h:79) 写明 `set_control_mode(3)` 是“关节拖动模式”；
- `ArmStatus` 的注释也写 `fsm_state=3` 是拖动；
- 但 [data_type_def.h](./arm_control_sdk/include/arm_control_sdk/data_type_def.h:13) 中的 `FsmMode` 枚举把数值 3 写作 `CURRENT`。

因此我删除了我们自己对 FSM 数字的映射。界面现在直接显示控制器回传的原始 `fsm_state`，不会错误地把某个状态硬解释为 MIT、拖动或电流模式。实际连上机械臂后，应验证点击“Vendor Drag”时返回的 FSM 值，并向厂家确认该枚举的真实版本定义。

相关调整在：

- [状态数据结构](./a3t_handle/src/arm/arm_types.hpp)
- [应用层手柄模式管理](./a3t_handle/src/control/handle_mode_manager.cpp)
- [SDK 调用与状态读取](./a3t_handle/src/arm/arm_worker.cpp)
- [分层状态 UI](./a3t_handle/src/ui/main_window.cpp)

已重新编译通过。

> fsm是什么

FSM 是 Finite State Machine，中文叫“有限状态机”。

它把机械臂控制器看成只能处于若干明确状态之一的系统，并规定状态之间怎么切换。

对这个 SDK，`fsm_state` 就是控制器当前运行模式。根据 `set_control_mode(mode)` 的注释，大致是：

```text
0：IDLE       空闲，不执行运动控制
1：POSITION   位置/轨迹控制
2：MIT        关节 Kp、Kd、位置、速度、力矩综合控制
3：DRAG       拖动模式，人手可推动机械臂
4：PF         力位混合控制
```

例如典型状态流程：

```text
未连接
  ↓ connect
已连接 + 未使能
  ↓ set_ready / set_servo_enable(true)
已使能 + IDLE
  ↓ set_control_mode(3)
已使能 + DRAG
  ↓ set_control_mode(0)
已使能 + IDLE
```

它和“是否连接”“是否使能”不是同一回事：

- 连接：你的电脑能否与控制器通信；
- 使能：电机是否上电、是否允许输出力矩；
- FSM：已使能后，控制器按哪种控制策略工作；
- 手柄模式：你自己的桌面程序定义的应用功能，第一版可能借用 FSM 的拖动模式，后续可借用 MIT 模式实现虚拟弹簧阻尼。

你现在界面显示原始 `fsm_state` 数字，是为了确认控制器真实进入了哪种模式；等和厂家确认枚举版本一致后，再把它显示成“拖动模式”“MIT 模式”等文字。

> ok,现在是主要做了UI界面的部分是吧

对，目前主要完成的是“桌面上位机初稿 + 控制流程骨架”，不是完整手柄控制算法。

已经有：

- Qt 桌面 UI；
- 连接 / 断开；
- Vendor Ready、伺服下使能、软件急停；
- 厂家拖动模式；
- 手柄模式这个应用层入口；
- 读取并显示关节角、末端位姿、连接/使能/安全/FSM 状态；
- SDK 调用集中在 `ArmWorker` 线程，UI 不会直接控制机械臂；
- 模式和安全检查的基础结构。

尚未实现：

- MIT 控制循环；
- 笛卡尔阻抗、虚拟弹簧、阻尼和虚拟墙；
- 手柄位姿到从端机械臂/仿真的映射；
- 实体使能开关、通信看门狗、完整限位保护；
- 实际真机联调与厂家 FSM 定义确认。

所以现在可以把它看成：先把“操作入口、状态显示、模式边界和安全框架”搭好。下一步应先用真机验证连接、Ready、拖动、状态读取和安全退出这条最小链路。

> 需不需要把SDK放入哪里，现在能直接调用里面的函数吗

现在不需要移动 SDK，已经能直接调用。

当前目录关系是：

```text
A3-T/
├── arm_control_sdk/   厂商 SDK
└── a3t_handle/        我们的桌面程序
```

[a3t_handle/CMakeLists.txt](./a3t_handle/CMakeLists.txt) 已把 SDK 路径设为：

```cmake
../arm_control_sdk
```

并且已经：

- 添加 `include/` 作为 C++ 头文件目录；
- 链接 `libarm_control_sdk.so` 和 `libmlog.so`；
- 配置运行时库路径，使 `build/a3t_handle` 能找到 SDK 的 `.so`；
- 成功编译过，说明 SDK 的 C++ 头文件和库已经被正确调用。

例如 [arm_worker.cpp](./a3t_handle/src/arm/arm_worker.cpp) 已经在使用：

```cpp
#include <arm_control_sdk/carm_cobot.h>

robot_->is_connected();
robot_->set_ready();
robot_->set_servo_enable(false);
robot_->set_control_mode(3);
robot_->get_status();
robot_->get_joint_pos();
robot_->get_cart_pose();
```

因此目前 SDK 不需要安装到系统目录，也不需要复制进 `a3t_handle/src/`。保持它作为相邻、独立的第三方依赖是更好的结构。

如果未来把 `a3t_handle` 单独拷到另一台电脑，只需要把 SDK 一起带过去，并在配置时指定路径：

```bash
cmake -S . -B build -DARM_CONTROL_SDK_DIR=/实际路径/arm_control_sdk
```

现在可以继续在这个结构上开发。

> 什么是力位混合模式

力位混合模式（PF，Position-Force）是：**有些方向控制位置，有些方向控制接触力/力矩**。

普通位置控制是：

> 末端必须到指定位置。

纯力控制是：

> 末端必须维持指定接触力。

力位混合则把六个末端自由度分开控制。例如机械臂拿着打磨工具压在一块平面上：

```text
平面内 X、Y：位置控制，沿指定路径移动
垂直方向 Z：力控制，始终以 10 N 压向表面
姿态 RX、RY、RZ：位置控制，保持工具角度
```

即：

| 方向 | 控制目标 |
|---|---|
| X、Y | 轨迹位置 |
| Z | 接触力 |
| RX、RY、RZ | 姿态 |

你们的 SDK 中：

```cpp
set_control_mode(4);  // PF 力位混合模式
low_pf_command(pos, vel, tau);
```

`low_pf_command()` 同时传入关节位置、速度和力矩前馈，底层控制器根据 PF 模式处理它们。

它常用于：

- 打磨、抛光、擦拭；
- 沿表面喷涂、涂胶；
- 插拔、装配；
- 恒力按压；
- 医疗或康复接触操作。

对你们当前“把 A3-T 做成手柄”的第一阶段，PF 不是重点。因为你们主要需要人推动主臂、读取 6DOF 位姿，以及给人虚拟弹簧阻尼手感；这更接近拖动模式或 MIT/阻抗控制。

PF 可能在后续有用：例如主端手柄控制从端机械臂，而从端在接触表面时使用 PF 保持恒定接触力。注意，真正可靠的力位混合通常需要末端 F/T 传感器、关节力矩估计或可信的外力估计。

> 那是不是先完成sdk里已有的功能？为什么现在只有拖动模式

不建议把 SDK 的所有已有功能都先完整重做一遍。

厂家网页端已经覆盖了位置、MIT、拖动、PF、轨迹录制、关节控制等通用功能；如果我们逐项复刻，会变成“重新做一套原厂上位机”，但离“机械臂手柄”并没有更近。

现在只有拖动模式，是因为它是手柄项目最安全、最直接的起点：

```text
SDK 拖动模式
  ↓
人能手推末端
  ↓
程序读取关节角 / 末端 6DOF 位姿
  ↓
记录轨迹或输出给仿真 / 从端
```

这条链路验证成功，就已经证明 A3-T 能作为主端手柄使用。

建议分阶段做：

1. **基础状态与安全**
   连接、Ready、下使能、急停、状态读取、关节/末端位姿显示。

2. **厂家拖动模式**
   `set_control_mode(3)`、`set_drag_params()`；验证“轻推、松手、状态采集、安全退出”。

3. **手柄模式 V1**
   在拖动模式上增加：
   - 捕获初始位姿；
   - 输出相对 6DOF 位姿；
   - 位置/姿态缩放；
   - 死区、低通滤波；
   - 轨迹记录；
   - 后续输出给仿真或从端。

4. **MIT 模式 V2**
   不是只做一个按钮，而是要有完整实时循环、参数限幅、通信看门狗、重力补偿和故障退出。之后才实现虚拟弹簧、阻尼、回中和虚拟墙。

5. **PF 模式 / 力反馈**
   留给从端接触任务，或者有可靠外力估计后再考虑。

因此，当前 UI 应该逐渐补齐“手柄功能必需的能力”，而不是补齐全部 SDK。位置模式可作为简单测试工具加入；MIT 和 PF 可以先显示“未实现/实验功能”，不要在算法和安全保护没准备好前开放给用户。

> 懂了，那现在已经完成了1和2？还是说现在已有的程序分别完成了什么功能

目前是：

- **第 1 阶段：代码初稿已完成，尚未真机验证。**
- **第 2 阶段：拖动模式的调用链已完成，尚未真机验证。**
- **第 3 阶段：只搭了“进入手柄模式”的入口，实际 6DOF 输出映射还没做。**

当前程序具体已经做了什么：

| 功能 | 当前状态 | 对应代码 |
|---|---|---|
| 连接控制器 | 已实现 | `CArmSingleCol(ip)`、`is_connected()` |
| 断开连接 | 已实现 | `set_control_mode(0)`、`disconnect()` |
| 状态读取 | 已实现 | `get_status()`、`get_joint_pos()`、`get_cart_pose()` |
| 显示连接/伺服/安全/FSM/仿真状态 | 已实现 | Qt 主窗口 |
| 厂家 Ready | 已实现 | `set_ready()` |
| 伺服下使能 | 已实现 | `set_servo_enable(false)` |
| 软件急停 | 已实现 | `emergency_stop()` |
| 厂家拖动模式 | 已实现 | `set_drag_params()`、`set_control_mode(3)` |
| 从拖动模式退出至 Idle | 已实现 | `task_stop()`、`set_control_mode(0)` |
| 手柄模式状态 | 已实现入口 | 当前底层暂时仍使用厂家拖动模式 |
| 6DOF 位姿增量输出 | 未实现 | 下一步 |
| 位姿缩放、死区、滤波 | 未实现 | 下一步 |
| MIT 虚拟弹簧阻尼 | 未实现 | 后续 |
| 虚拟墙、限位看门狗 | 未实现 | 后续 |
| 控制仿真/从端机械臂 | 未实现 | 后续 |
| 真机验证 | 未开始 | 需要现场测试 |

所以当前程序已经具备“上位机最小控制与状态框架”，但还不能称作完成的手柄。

真机验证时，建议按这个顺序点击：

```text
Connect
  → 确认状态正常
  → Vendor Ready
  → 确认 Servo = Enabled
  → Vendor Drag
  → 手推末端并观察关节角/末端位姿变化
  → Return to Idle
  → Disable Servo
```

这条链路通过后，才进入手柄模式 V1：记录拖动时的初始位姿与实时位姿，并计算、显示 6DOF 相对运动指令。

:codex-annotation{index="1"} `set_drag_params()` 目前在 [arm_worker.cpp](./a3t_handle/src/arm/arm_worker.cpp) 的两个入口调用：

- `enterDragMode()`：点击 **Vendor Drag** 时；
- `enterHandleMode()`：点击 **Enter Handle** 时，当前手柄模式也先以厂家拖动模式作为底层实现。

参数当前来自 [handle.json](./a3t_handle/config/handle.json)：

```json
"drag": {
  "torque_factor": 1.0,
  "friction_compensation_factor": 0.8
}
```

所以现在 UI **不能实时调参数**；你需要修改配置文件后重启程序，下一次进入拖动/手柄模式时才会读取新值并发送给控制器。

下一步可以在 UI 加两个滑块：

- `torque_factor`：SDK 范围是 `[0, 2]`，越大通常越轻；
- `friction_compensation_factor`：SDK 范围是 `[0, 2]`，用于补偿关节摩擦。

并增加“应用参数”按钮，在拖动或手柄模式中调用一次 `set_drag_params()`，实现在线调节。

> 增加吧，先完善好sdk所提供的功能，当然只是拖动模式，完成后我们再尝试完善手柄模式的逻辑

<details><summary>2 previous messages</summary>

> 我会在桌面程序中加入拖动参数面板：两个可调数值框和“应用拖动参数”按钮。参数只会在已进入厂家拖动或当前手柄模式的拖动后端时下发；不会开放 MIT 或 PF。
>
> <details><summary>Edited 5 files, ran a command</summary>
>
> - Wrote `./a3t_handle/README.md` (+3 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+38 -8)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+3 -0)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+23 -0)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+3 -0)
> - Ran `cmake --build build -j2`
>
> </details>
</details>

已增加拖动参数调节，并编译通过。

现在界面有：

- `Drag torque factor`，范围 `[0, 2]`；
- `Drag friction compensation`，范围 `[0, 2]`；
- `Apply Drag Parameters` 按钮。

点击按钮后，程序会为每个关节构造对应参数数组，并调用厂商 SDK：

```cpp
robot_->set_drag_params(torque, friction);
```

调用集中在 [arm_worker.cpp](./a3t_handle/src/arm/arm_worker.cpp)，UI 只提交参数，不直接控制机械臂。

为安全起见，只有在以下任一状态才能应用：

- 已进入 **Vendor Drag**；
- 已进入 **Handle**（当前底层仍使用 Vendor Drag）。

否则程序只提示，不会下发参数。

注意当前 UI 调参是“本次运行的在线参数”，不会自动写回 [handle.json](./a3t_handle/config/handle.json)。重新进入拖动模式时，会再次读取配置文件中的默认值。后续可以再加“保存为默认配置”按钮。

> 等于说当下的话，我们的拖动模式，以及前面的启动相关都已经完成了？

对，按“软件初稿”来说，启动相关和拖动模式的主流程已经写完了，并且编译通过。

目前已有流程：

```text
Connect
  → 获取控制器状态
  → Vendor Ready
  → 显示伺服是否已使能
  → Vendor Drag / Handle
  → 在线调节拖动参数
  → Return to Idle
  → Disable Servo / Emergency Stop / Disconnect
```

已经接入的 SDK 功能包括：

- `is_connected()`、`disconnect()`
- `get_status()`、`get_joint_pos()`、`get_cart_pose()`
- `set_ready()`
- `set_servo_enable(false)`
- `set_drag_params()`
- `set_control_mode(3)`：进入厂家拖动模式
- `task_stop()`、`set_control_mode(0)`：退出至空闲
- `emergency_stop()`

但“完成”还差真机验证。当前是：

- 代码和 UI：完成；
- 编译：完成；
- 真机连接、Ready、拖动参数实际效果、退出与急停行为：尚未验证；
- 手柄 6DOF 映射、轨迹输出、MIT 阻抗：尚未实现。

下一步应先真机验证这条最小链路，确认厂家 SDK 的 `set_ready()` 和 `set_control_mode(3)` 在你们的固件版本上行为正确。

> 那我们要增加handle模式的话该怎么做，`set_control_mode`是留了空位吗，还是我们重新写

:codex-annotation{index="1"} 暂时就按 SDK 注释约定：`set_ready()` 做厂家 Ready，`set_control_mode(3)` 进入拖动。

`set_control_mode()` 不是给我们预留“自定义模式编号”的接口。它只能切换厂家控制器已有的底层模式：

```text
0 Idle
1 Position
2 MIT
3 Drag
4 PF
```

所以不能做：

```cpp
set_control_mode(5);  // 不应这样做
```

“Handle 模式”应是我们桌面程序的**应用层模式**，不是控制器 FSM 的新编号。当前代码已经有：

```cpp
ApplicationMode::Monitor
ApplicationMode::Handle
```

它们的关系应是：

```text
应用层：Handle
    ↓ 当前第一版的底层实现
厂家控制器：Drag（set_control_mode(3)）
```

也就是说，进入 Handle 时：

1. 检查已连接、已使能、安全状态正常；
2. 调用 `set_drag_params()`；
3. 调用 `set_control_mode(3)`；
4. 记录当前末端位姿为手柄零点 \(T_0\)；
5. 持续读取 `get_cart_pose()`；
6. 计算当前相对零点的 6DOF 增量 \(T_0^{-1}T\)；
7. 对这个增量做死区、滤波、缩放；
8. 输出给日志、仿真或未来的从端机器人。

当前代码只完成了前 1–3 步，并把应用状态设为 `Handle`。下一步就是实现第 4–8 步，这才是手柄逻辑本身。

未来如果要做虚拟弹簧阻尼，应用层仍然叫 `Handle`，只是底层实现从 Drag 切换为 MIT：

```text
应用层：Handle
    ↓ 后续高级实现
厂家控制器：MIT（set_control_mode(2)）
    ↓
low_mit_command(pos, vel, tau, kp, kd)
```

因此不是“重新写控制器模式”，而是在现有 SDK 模式之上，写自己的 `HandleController`。

> 拖动和MIT模式给我们开放了什么可以做的调整的接口，因为我们手柄模式不是要固定一个中心点吗，如果不支持做调整，我们难道不应该做一个新的模式吗？MIT不是固定当前位置做阻尼吗

你抓住关键了：**拖动模式不足以实现“固定空间中心点的六维弹簧阻尼”**；这个需求本质上应通过 MIT 模式实现。

但仍然不需要创造一个新的控制器底层模式。你们的“手柄模式”是应用逻辑，底层使用 MIT。

## 拖动模式开放的调整

拖动模式主要给你：

```cpp
set_drag_params(torque_factor, friction_compensation_factor);
```

可调的是每个关节的：

- 转矩因子：让拖动更轻或更沉；
- 摩擦补偿：减轻减速器摩擦、卡顿感。

再配合：

- `set_collision_config()`：碰撞灵敏度；
- `set_speed_level()`：运动速度等级；
- `get_cart_pose()`：读取手柄末端 6DOF 位姿。

但它没有暴露“笛卡尔空间中心点”“XYZ/RPY 六维刚度”“回中力”“虚拟墙”等参数。因此它适合做自由拖动和示教，不适合做你们最终定义的弹簧手柄。

## MIT 模式开放的调整

MIT 模式给的是每个关节的完整阻抗控制指令：

```cpp
low_mit_command(pos, vel, tau, kp, kd);
```

参数含义：

\[
\tau_{cmd}
=
K_p(q_d-q)
+
K_d(\dot q_d-\dot q)
+
\tau_{ff}
\]

- `pos`：目标关节角 \(q_d\)
- `vel`：目标关节速度 \(\dot q_d\)
- `tau`：前馈力矩 \(\tau_{ff}\)，可放重力补偿等
- `kp`：每关节刚度
- `kd`：每关节阻尼

SDK 还给了实现末端空间控制需要的基础信息：

```cpp
get_joint_pos()
get_joint_vel()
low_get_jacobian()
low_get_dynamics()
get_cart_pose()
```

因此 MIT 是你们最终手柄模式的底层基础。

## MIT 是否等于“固定当前位置做阻尼”？

不完全是。

MIT 自己不会自动固定当前位置；它只会追踪你每周期传入的 `pos / vel / tau / kp / kd`。

如果进入 MIT 时记录当前关节角：

\[
q_0 = q(t_{\text{enter}})
\]

并持续发送：

\[
q_d=q_0,\quad \dot q_d=0
\]

那么它表现为“各关节回到初始位置”的关节弹簧阻尼。

但你们想要的是：

> 末端围绕一个空间中心点，在位置和姿态的六个方向都有弹簧阻尼。

这应在末端空间定义：

\[
W=
\begin{bmatrix}
K_p(p_0-p)-D_pv\\
K_R e_R-D_R\omega
\end{bmatrix}
\]

然后通过雅可比矩阵变成关节力矩：

\[
\tau=J(q)^T W+g(q)
\]

其中：

- \(p_0,R_0\)：进入手柄模式时捕获的中心位置、中心姿态；
- \(K_p,K_R\)：平移与旋转刚度；
- \(D_p,D_R\)：平移与旋转阻尼；
- \(J(q)\)：SDK 的 `low_get_jacobian()`；
- \(g(q)\)：SDK 的 `low_get_dynamics()` 返回的重力项。

最后把计算结果作为 MIT 的前馈力矩发出，同时可把 `kp/kd` 设为较低的关节稳定增益。

## 最终模式关系

```text
厂家 FSM：MIT（set_control_mode(2)）
        ↓
你们程序：HandleController
        ↓
捕获中心位姿 T0
        ↓
计算六维虚拟弹簧阻尼
        ↓
JᵀW + 重力补偿
        ↓
low_mit_command(...)
```

所以结论是：

- 自由示教/快速验证：厂家 Drag。
- 固定中心点、六维弹簧阻尼、虚拟墙：厂家 MIT + 你们自己的 `HandleController`。
- 不需要也不能随意新加 `set_control_mode(5)`；新模式应是你们程序内部的 `Handle`，其底层后端从 Drag 升级为 MIT。

> 那假如我切换到了MIT模式，那它不还是追踪我们当前的给的位置？那我们怎么拖动手臂呢，还是没完全懂，就是固定中心位置一直发布吗，然后降低tau就能拖动了？具体怎么实现

对，切到 MIT 后，机械臂仍然会追踪你给它的目标；“能拖动”恰恰来自于把这个目标固定在一个中心点，同时把刚度设成合适的低值、阻尼设成合适值。

关键不是“降低 `tau`”，而是：

- `tau` 主要用于重力补偿和你计算出的虚拟弹簧力；
- `Kp` 决定离开中心后拉回去的力度；
- `Kd` 决定拖动时的阻尼；
- 固定的是“弹簧平衡点”，不是把机械臂锁死。

## 最直观的单关节例子

假设某个关节进入手柄模式时在：

\[
q_0=0.5\ \mathrm{rad}
\]

持续发送：

\[
q_d=q_0,\quad \dot q_d=0
\]

MIT 内部计算：

\[
\tau =
K_p(q_0-q)-K_d\dot q+\tau_g
\]

其中 \(\tau_g\) 是重力补偿。

此时：

- 手不碰时，关节停在 \(q_0\)；
- 你向外推，关节角变为 \(q\neq q_0\)；
- 离 \(q_0\) 越远，回拉力越大；
- 推得越快，阻尼越大；
- 松手后，它回到 \(q_0\)。

所以它像一个有中心点的弹簧手柄。

`Kp` 小：容易推开，回弹弱。
`Kp` 大：难推开，回弹强。
`Kd` 大：拖动更稳，但太大像“黏住”。

重力补偿不能简单降低，否则机械臂会因为自身重量下坠，而不是因为你在操控它。

## 第一版：关节空间弹簧手柄

这是最简单、最适合先验证的版本：

1. 进入手柄模式前读取当前关节角 \(q_0\)；
2. 切换到 MIT；
3. 每个控制周期持续发送：

```text
pos = q0
vel = 0
kp  = 较低的关节刚度
kd  = 合适的关节阻尼
tau = 重力补偿力矩
```

4. 同时读取当前末端位姿 \(T\)；
5. 用当前位姿相对初始位姿 \(T_0\) 的变化作为手柄输出：

\[
\Delta T=T_0^{-1}T
\]

例如，你把末端向右推 5 cm，程序就输出“X 正方向 +5 cm”的控制指令；松手后，机械臂会弹回中心，输出回到零附近。

这已经很像弹簧摇杆。

缺点是：它的弹簧定义在关节空间。末端不同姿态下，人的手感可能不一致，不是严格的“空间六方向弹簧”。

## 最终版：末端笛卡尔空间弹簧手柄

进入手柄模式时，保存：

```text
中心关节角：q0
中心末端位姿：x0 = [p0, R0]
```

每个控制周期：

```text
读取当前 q、dq
        ↓
计算当前末端位姿 x、末端速度 dx
        ↓
计算相对中心的位移、姿态误差
        ↓
计算六维虚拟弹簧阻尼力 W
        ↓
用 Jᵀ 转成关节力矩
        ↓
加上重力补偿
        ↓
通过 MIT 指令发送
```

控制律是：

\[
W=
\begin{bmatrix}
K_p(p_0-p)-D_pv\\
K_R e_R-D_R\omega
\end{bmatrix}
\]

\[
\tau_{cmd}=J(q)^TW+g(q)
\]

这里：

- \(p_0,R_0\)：固定的空间中心；
- \(p,R\)：当前末端位置和姿态；
- \(K_p,K_R\)：位置、姿态弹簧刚度；
- \(D_p,D_R\)：位置、姿态阻尼；
- \(J^T\)：把末端六维力/力矩换成关节力矩；
- \(g(q)\)：重力补偿。

这种情况下，人推末端的任意方向都会感到“离中心越远、回拉越强”的效果，才是你说的“一个空间点所有方向都有弹簧阻尼”。

## MIT 指令怎么避免“位置控制抢控制权”

对于笛卡尔弹簧版本，虚拟弹簧力已经由你算成了 `tau_cmd`。此时 MIT 内部的关节位置 PD 不应再用很大的 `Kp/Kd` 抢控制权。

常见思路是每个周期：

```text
pos = 当前 q
vel = 当前 dq
tau = JᵀW + g(q)
kp/kd = 很低的稳定增益，或厂家允许的最小安全值
```

因为目标 `pos` 等于当前 `q`，位置误差接近 0；目标 `vel` 等于当前 `dq`，速度误差也接近 0。MIT 的主要输出就来自你提供的前馈力矩 `tau`，即虚拟弹簧阻尼与重力补偿。

但 `Kp/Kd` 能否设为 0、最小值是多少、MIT 的实际更新频率和失联保护如何工作，必须以厂家参数范围和真机测试为准。

## 中心点固定，会不会限制操控范围？

会；但这是弹簧手柄的正常设计。

- 中心固定：末端位移是输入量，松手回中，类似摇杆。
- 想在更大范围持续操作：做一个“离合/重置中心”按钮。
- 用户按住离合时：更新中心 \(T_0\leftarrow T\)，暂不输出控制指令。
- 用户松开离合时：以新位置为中心继续输出相对位移。

所以最终的手柄交互通常是：

```text
拖动末端 → 输出相对 6DOF 指令
松手     → 弹回中心
按离合   → 重设中心，不产生输出
```

> 那这个读取当前状态的命令，计算力矩的程序，还有别的什么吗，这些速度必须跟的上才能稳定计算返回想要的力？

对。MIT 手柄的关键不只是“读取状态 + 算力矩”，而是一个稳定、连续、低抖动的控制闭环；控制循环频率和通信抖动直接决定手感与稳定性。

当前程序的 `poll_interval_ms = 100`，即约 10 Hz，只适合 UI 状态显示，**绝不能拿来做 MIT 虚拟弹簧控制**。

MIT 手柄控制循环应单独运行：

```text
读取 q、dq、状态、时间戳
        ↓
计算末端位姿 x、速度 dx
        ↓
计算相对中心的六维误差
        ↓
计算虚拟弹簧阻尼 W
        ↓
计算 JᵀW + 重力补偿
        ↓
力矩、速度、位置、参数限幅
        ↓
发送 low_mit_command(...)
        ↓
检查通信/故障/超时
        ↓
下一周期
```

SDK 已提供或暗示的相关接口有：

| 需求 | SDK 接口 |
|---|---|
| 当前关节角 | `get_joint_pos()` |
| 当前关节速度 | `get_joint_vel()` |
| 当前关节力矩 | `get_joint_tau()` |
| 实时关节回调 | `register_joint_cbk(...)` |
| 当前末端位姿 | `get_cart_pose()` |
| 实时末端位姿回调 | `register_pose_cbk(...)` |
| 当前外力估计 | `get_joint_external_tau()`、`get_cart_external_force()` |
| 雅可比矩阵 | `low_get_jacobian(q)` |
| 重力/动力学项 | `low_get_dynamics(q, dq, ddq)` |
| MIT 下发与底层状态 | `low_mit_command(pos, vel, tau, kp, kd)` |

速度确实必须跟得上，原因是：

- 如果状态已经是 100 ms 前的，但你根据它施加“回弹力”，机械臂可能已经越过中心很远；
- 延迟会使弹簧看起来像负阻尼，产生来回振荡；
- 网络和 UI 卡顿会造成力矩突变；
- 参数 \(K,D\) 越大，对控制频率和延迟越敏感。

控制循环还必须额外具备：

- 单调时钟和真实 `dt`；
- 关节角、速度、力矩、加速度限幅；
- 力矩变化率限制；
- 低通滤波，但不能造成过大延迟；
- 通信看门狗：若连续若干周期未收到状态/未成功下发，立刻退出 MIT；
- 物理死手开关；
- 工作空间、关节限位和奇异位形保护；
- 切换模式时的平滑初始化，避免刚进入 MIT 就产生大力矩。

尤其注意：`low_get_jacobian()` 和 `low_get_dynamics()` 如果每次都是一次 WebSocket 请求，未必适合每个 MIT 周期调用。要先实测它们的响应时间和抖动。

更理想的路径是：

1. 通过 `register_joint_cbk()` 获得连续关节状态；
2. 在本地利用 URDF + 动力学库计算正运动学、雅可比和重力项；
3. 控制线程固定周期计算；
4. 每周期用 `low_mit_command()` 下发；
5. UI 线程只以低频显示状态，完全不参与控制循环。

所以接下来不是立刻写弹簧公式，而是先确认：

> SDK 的 MIT 命令最高稳定下发频率是多少？关节回调频率是多少？`low_mit_command()` 和动力学查询是否能在该频率下稳定运行？

这些数据决定是能在开发电脑上跑 MIT 控制环，还是需要把实时控制放到控制器侧。

> 这些能查询到吗，还是得测试，以及能否再不动硬件的情况下提高？
> 有没有跟踪位置模式，除了完成任务交代的，我还希望能增加一个跟踪模式。手柄模式负责记录轨迹，跟踪模式负责跟踪实现动作。但如果想实现抖动衣服或毛巾这种高速变化移动的功能，应该需要极高的频率以及速度加速度变化的能力

<details><summary>2 previous messages</summary>

> 我先查 SDK 本地文档和代码是否声明了回调或 MIT 的频率、超时与限速；这能区分“可从资料确认”的上限和“必须在你们控制器/网络上实测”的性能。
>
> <details><summary>Explored a search</summary>
>
> - Searched for `hz|frequency|频率|周期|period|timeout|实时|realtime|rate|callback|cbk|track_joint|track_pose|speed_percentage|joint_vel|joint_acc|joint_jerk|low_mit_command` in `README.md`
>
> </details>
</details>

:codex-annotation{index="1"} 能查到一部分，剩余关键指标必须测试。

本地 SDK 已经给出两个有价值的信息：

- Python 文档明确说：底层透传接口适用于“**1 ms 级控制周期**”场景，且要先进入 `set_low_mode(True)`；对应 C++ 是 `set_low_mode(true)` + `low_mit_command(...)`。[README.md](./arm_control_sdk/carm_python/README.md:779)
- Python 封装代码注明常规状态上报周期为 **20 ms**，约 50 Hz。[carm.py](./arm_control_sdk/carm_python/carm/carm.py:79)

但 SDK 没有给出：

- `low_mit_command()` 的最大稳定频率；
- 端到端延迟与最坏抖动；
- 1 ms 周期是否能通过当前 WebSocket / 控制器组合稳定实现；
- 动力学、雅可比查询能否每周期调用。

这些必须在真机和实际网络上测量。20 ms 状态上报可用于 UI、拖动和普通跟踪，但不应该直接假设能支持高刚度 MIT 弹簧。

不动硬件能做的优化包括：

- 用 C++ SDK，不用 Python；
- UI 线程和控制线程彻底分离；
- 使用 SDK 的实时回调，而不是 UI 的 100 ms 轮询；
- 进入 `set_low_mode(true)` 后使用 `low_mit_command()`；
- 控制电脑和控制器直连有线网，不经过拥挤 Wi‑Fi；
- 控制循环不做磁盘写入、日志打印、UI 更新；
- 设置线程高优先级、记录真实周期 `dt` 和抖动；
- 本地计算轨迹与控制量，避免每周期频繁请求远程接口。

但软件优化不能突破机械臂的物理上限。SDK 的 `get_config()` 能返回每个关节的最大速度、加速度和 jerk；`set_speed_level()` 只是按比例降速，不能把厂家限定的上限提高。

关于“跟踪模式”：有，SDK 已经提供两类。

```cpp
track_joint(target_joint_positions);  // 周期性发送关节目标
track_pose(target_cartesian_pose);    // 周期性发送末端位姿目标
```

这两个接口最适合增加一个独立的应用层 `Tracking` 模式：

```text
拖动模式
  人手拖动 → 记录轨迹 / 记录末端位姿

跟踪模式
  读取轨迹 / 外部实时目标 → track_joint 或 track_pose → 机械臂执行
```

另外厂商还有：

```cpp
trajectory_teach(...)
trajectory_recorder(...)
move_joint_traj(...)
move_pose_traj(...)
```

适合播放预先记录好的轨迹。对规则、低速的重复动作，优先用轨迹接口；对外部持续产生目标的位置跟随，才用 `track_joint()` 或 `track_pose()`。

“抖动衣服或毛巾”是更高动态的任务，重点不只是命令频率，还要满足轨迹的速度、加速度和 jerk 限制。若末端按正弦运动：

\[
x(t)=A\sin(2\pi ft)
\]

则：

\[
v_{max}=2\pi fA
\]

\[
a_{max}=(2\pi f)^2A
\]

\[
j_{max}=(2\pi f)^3A
\]

频率 \(f\) 翻倍时，速度翻倍、加速度变为 4 倍、jerk 变为 8 倍。所以“幅度不大但频率很高”的甩毛巾动作，仍然很容易超过机械臂上限。

这种动作不应靠 UI 按钮连续发送离散点，也不应直接对每次命令跳变。正确路径是：

1. 离线生成光滑、连续且可微的关节或末端轨迹；
2. 根据 `get_config()` 的速度、加速度、jerk 限制缩放时间；
3. 优先使用厂商内部轨迹执行接口；
4. 若必须实时跟踪，使用独立控制线程持续调用 `track_joint()`；
5. 从低振幅、低频率开始，逐步提高；
6. 毛巾等柔性负载会引入摆动和额外惯量，必须重新做碰撞、力矩和工作空间评估。

下一步可在当前程序中新增 `Tracking` 应用模式，但需要先确定它的目标来源：是“回放拖动记录的轨迹”、还是“从文件导入轨迹”、还是“接收外部设备实时目标”。

> 那是不是应该先把之前的拖动模式改成MIT模式？拖动模式反正有上位机？再安排下后续的工作内容，给个排序

不建议现在把拖动模式直接“改成” MIT 模式。更准确的安排是：

- **厂家 Drag**：保留，作为基础验证、手推示教、轨迹采集和 MIT 对照组。
- **我们自己的 Handle**：后续改为以 MIT 为底层后端。
- **Tracking**：独立功能，负责播放或实时跟踪轨迹。

原厂上位机确实已有拖动模式，所以你们不需要复刻它的完整 UI；但自己的程序仍应保留一个简化的 `Vendor Drag`，因为它能验证 SDK 通信、状态回调、工具坐标、轨迹记录和安全退出是否正确。

当前 `Enter Handle` 实际仍然调用 Drag，容易让人误解。下一步应把它明确标为：

```text
Vendor Drag        厂家自由拖动
Handle V0          基于 Drag 的位姿采集手柄
Handle MIT         未来的中心弹簧阻尼手柄
Tracking           轨迹回放 / 实时目标跟踪
```

推荐工作排序：

1. **真机验证基础链路**

   连接、Vendor Ready、状态读取、伺服下使能、急停、进入/退出 Drag。确认 SDK 的实际返回状态与注释一致。

2. **完成 Drag 采集能力**

   用 `register_joint_cbk()`、`register_pose_cbk()` 替换 UI 的低频轮询；记录带时间戳的 `q / dq / pose`，保存为轨迹文件。

3. **实现 Tracking 模式**

   先做“轨迹回放”：

   ```text
   读取拖动录制的轨迹
     → 时间重采样、滤波、限速
     → track_joint() 或 track_pose()
   ```

   这一步不需要 MIT，能先完成“拖动录制动作 → 机械臂复现动作”。

4. **做性能测试工具**

   测量关节/位姿回调周期、`track_*()` 下发耗时、`low_mit_command()` 下发耗时、网络抖动和控制器版本。只有数据明确后，才能决定 MIT 控制环部署方式。

5. **MIT 关节空间弹簧实验**

   先不做末端六维阻抗，只做固定 \(q_0\) 的低刚度关节弹簧、重力补偿、力矩限幅和通信看门狗。目标是验证“能推开、能稳定回中、不振荡”。

6. **Handle MIT V1**

   捕获中心位姿 \(T_0\)，输出当前相对位姿 \(\Delta T\)，增加死区、缩放、滤波和“离合/重置中心”。

7. **Handle MIT V2：笛卡尔空间弹簧阻尼**

   接入雅可比、动力学和重力项，实现真正的六维中心点弹簧、姿态弹簧、虚拟墙和方向相关刚度。

8. **高速轨迹与抖动柔性物体**

   在 Tracking 模式中增加正弦、扫频和轨迹缩放；先根据厂家速度/加速度/jerk 限制生成可行轨迹，再做毛巾、衣物等柔性负载测试。

这样顺序的好处是：Tracking 可以先交付一个完整功能；MIT 则在实时性、安全和参数边界明确后再做，不会一开始就把“手柄逻辑”和“底层力控稳定性”混在一起。

> 所以最后我们自己的程序是有可以切换拖动和手柄模式？

对，最终你们自己的程序应能切换多个应用模式，至少包括：

```text
Monitor          仅连接、查看状态，不控制机械臂
Vendor Drag      厂家拖动模式：自由手推、示教、采集轨迹
Handle           你们的手柄模式：中心弹簧阻尼 + 输出相对 6DOF 指令
Tracking         跟踪模式：播放已录制轨迹或跟随外部实时目标
```

其中底层关系是：

| 你们 UI 模式 | 厂家控制器底层模式 |
|---|---|
| Monitor | Idle / Position |
| Vendor Drag | Drag，`set_control_mode(3)` |
| Handle V0 | Drag，采集相对末端位姿 |
| Handle MIT | MIT，`set_control_mode(2)` |
| Tracking | Position + `track_joint()` / `track_pose()` |

所以“拖动”和“手柄”不是重复：

- **拖动**：机械臂本体是自由的，人推着做示教；一般不要求自动回中心。
- **手柄**：机械臂是一个带虚拟手感的输入设备；它有中心点、弹簧阻尼、回中、离合和 6DOF 输出。

最终 UI 可以把 `Vendor Drag` 与 `Handle` 做成并列按钮。进入其中一个模式时，程序自动退出另一个，并确保不会同时有两个控制模式发送指令。

> 现在完成了哪些，当前的功能和未来的功能，包括UI是什么样的

当前完成的是“基础桌面上位机 + 厂家拖动模式入口”，还不是完整手柄系统。

当前已有功能：

| 功能 | 状态 |
|---|---|
| 连接 / 断开控制器 | 已实现 |
| 显示连接状态 | 已实现 |
| 显示伺服使能状态 | 已实现 |
| 显示控制器安全状态 | 已实现 |
| 显示厂商原始 FSM 状态 | 已实现 |
| 显示实机 / 仿真状态 | 已实现 |
| 读取并显示关节角 | 已实现 |
| 读取并显示末端位姿 | 已实现 |
| Vendor Ready | 已实现 |
| 伺服下使能 | 已实现 |
| 软件急停 | 已实现 |
| 厂家 Drag 模式 | 已实现 |
| 拖动参数在线调节 | 已实现 |
| Return to Idle | 已实现 |
| 手柄模式入口 | 已实现，但当前仍以 Drag 作为底层后端 |
| 真机测试 | 尚未完成 |

当前 UI 大致是：

```text
控制器 IP
连接状态 / 伺服状态 / 安全状态
厂商 FSM 状态 / 实机或仿真状态
关节角 / 末端位姿

[Connect] [Disconnect]
[Vendor Ready] [Disable Servo]
[Vendor Drag] [Enter Handle]
[Return to Idle] [Emergency Stop]

拖动参数：
  torque factor [0, 2]
  friction compensation [0, 2]
  [Apply Drag Parameters]

状态消息区
```

当前按钮含义：

- `Vendor Drag`：直接进入厂家拖动模式。
- `Enter Handle`：当前同样以厂家拖动作为底层，但程序内部标记为 `Handle`；尚未输出 6DOF 指令，也没有弹簧回中。
- `Apply Drag Parameters`：仅在 Drag 或当前 Drag 后端的 Handle 中生效。

未来功能建议按阶段增加：

```text
阶段 1：验证与采集
  - 真机验证现有按钮和安全流程
  - 关节/位姿回调
  - 带时间戳轨迹记录
  - 保存、加载、查看轨迹

阶段 2：Tracking
  - 拖动录制 → 轨迹回放
  - track_joint / track_pose 实时跟踪
  - 速度、加速度、jerk 限制
  - 正弦、扫频等高动态轨迹生成

阶段 3：Handle V1
  - 捕获手柄初始中心位姿
  - 计算相对 6DOF 位姿增量
  - 死区、滤波、缩放
  - 离合 / 重置中心
  - 将指令输出给仿真或从端

阶段 4：Handle MIT
  - MIT 控制循环与性能测试
  - 重力补偿
  - 关节空间低刚度弹簧阻尼
  - 力矩、速度、通信看门狗保护

阶段 5：笛卡尔阻抗手柄
  - 末端六维中心弹簧阻尼
  - 位置与姿态独立刚度
  - 虚拟墙、工作空间边界
  - 自适应刚度/阻尼
```

最终 UI 会在现有基础上增加模式区：

```text
[Monitor] [Vendor Drag] [Handle] [Tracking]

Handle 参数：
中心点 / 重置中心 / 离合
位置缩放 / 姿态缩放
XYZ 刚度和阻尼
旋转刚度和阻尼
虚拟墙与限位

Tracking 参数：
轨迹文件 / 开始 / 暂停 / 停止
速度比例
循环次数
轨迹状态和实时误差
```

目前最接近可直接真机使用的是 `Vendor Drag`；最核心的最终目标是 `Handle MIT`。

> 因为手柄模式是主要任务，追踪示教是次要任务，所以是不是应该调整下当前任务顺序

:codex-annotation{index="1"} 你说得对。之前的排序更偏“稳妥渐进”，但如果 mentor 的主要任务就是把 A3-T 做成带中心弹簧阻尼的手柄，那么主线应直接是 **MIT + 末端六维阻抗控制**；拖动、示教和 Tracking 都应降为辅助功能。

而且你的理解是正确的，核心链路就是：

\[
\text{末端六维误差}
\rightarrow
\text{六维弹簧阻尼力/力矩 } W
\rightarrow
J^T W
\rightarrow
\text{关节力矩}
\rightarrow
\text{MIT 指令}
\]

更完整地写：

\[
W =
\begin{bmatrix}
K_p(p_0-p)-D_pv\\
K_R e_R-D_R\omega
\end{bmatrix}
\]

\[
\tau_{ff}=J(q)^TW+g(q)
\]

然后每周期下发：

```cpp
low_mit_command(q_ref, dq_ref, tau_ff, kp, kd);
```

这里 MIT 不是“返回力矩”，而是你将计算出的 `tau_ff` 作为前馈力矩发送给机械臂；机械臂底层执行 MIT 关节控制。

项目优先级应调整为：

1. **基础通信与安全确认**

   保留现有连接、Ready、下使能、急停、状态显示和厂家 Drag；但 Drag 只用于确认机械臂可手推、验证状态与安全退出，不再是主要开发目标。

2. **MIT 控制能力与周期测试**

   进入底层模式，确认 MIT 的进入/退出流程；测量 `low_mit_command()` 的实际周期、延迟、失败行为和通信抖动。
   这不是一个独立功能，而是 MIT 手柄实现的第一步。

3. **手柄核心：笛卡尔六维弹簧阻尼**

   - 进入手柄模式时记录中心位姿 \(T_0\)；
   - 读取关节状态 \(q,\dot q\)；
   - 计算当前末端位姿、速度；
   - 用 `low_get_jacobian()` 获取 \(J\)；
   - 用 `low_get_dynamics()` 获取重力项 \(g(q)\)；
   - 计算 \(J^TW+g(q)\)；
   - 持续发送 MIT 指令；
   - 从低刚度、低速度、低力矩上限开始调试。

4. **手柄安全层**

   - 力矩、速度、加速度、力矩变化率限幅；
   - 通信超时立即退出 MIT；
   - 工作空间与关节限位保护；
   - 物理使能/死手开关；
   - 进入和退出 MIT 的平滑过渡；
   - 碰撞检测与急停。

5. **手柄交互体验**

   - XYZ 与姿态分别调刚度/阻尼；
   - 回中、离合、重置中心；
   - 死区、缩放、滤波；
   - 虚拟墙、方向引导槽；
   - 记录参数与手感日志。

6. **辅助功能：拖动示教与 Tracking**

   - Drag 模式录制普通轨迹；
   - Tracking 模式回放轨迹；
   - `track_joint()` / `track_pose()` 实现位置跟随；
   - 高速抖动毛巾等任务在这里实现，并单独做速度、加速度、jerk 可行性检查。

所以最终 UI 的主按钮应是：

```text
[Vendor Drag：辅助]
[Handle MIT：主功能]
[Tracking：辅助]
```

当前的 `Enter Handle` 暂时是 Drag 后端的占位实现。下一步最合理的代码工作，不是先写 Tracking，而是把它替换成真正的 `HandleMitController`：保存中心位姿、运行独立控制线程、计算 \(W\)、映射到 \(J^TW+g(q)\)、发送 `low_mit_command()`。

> 我现在希望直接把最终版本的UI以及已经可实现的功能都完成，然后后续再慢慢补充额外的功能

<details><summary>4 previous messages</summary>

> 我会把 UI 升级为最终产品的布局：连接与安全、拖动/示教、手柄 MIT、跟踪、诊断分区。已经能安全调用的 SDK 功能会接入；MIT 手柄和外部跟踪会保留完整界面与参数位，但在控制闭环未实现前保持禁用，避免“UI 有按钮就误以为可真机运行”。
>
> <details><summary>Created 2 files, edited 2 files, deleted 2 files, ran a command</summary>
>
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+68 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+8 -0)
> - Deleted `./a3t_handle/src/ui/main_window.cpp` (+0 -128)
> - Deleted `./a3t_handle/src/ui/main_window.hpp` (+0 -38)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+269 -0)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+47 -0)
> - Ran `cmake --build build -j2`
>
> </details>
>
> 界面主体已接入；编译时遇到 Qt 控件指针混合初始化列表的类型问题，我会改为按 `QWidget*` 统一处理后重新验证。
>
> <details><summary>Edited 2 files, ran 2 commands</summary>
>
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+6 -1)
> - Ran `cmake --build build -j2`
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+1 -1)
> - Ran `cmake --build build -j2`
>
> </details>
</details>

已把 UI 扩展为最终产品的分区布局，并接入当前可安全使用的 SDK 功能；项目已重新编译通过。

当前界面包括：

- `Connection & Safety`
  - Connect / Disconnect
  - Vendor Ready
  - Disable Servo
  - Emergency Stop
  - 状态显示：连接、伺服、安全、厂商 FSM、实机/仿真、关节角、末端位姿
  - 速度等级设置：`set_speed_level()`
  - 碰撞保护开关与灵敏度：`set_collision_config()`

- `Drag & Teach`
  - 进入厂家 Drag：`set_control_mode(3)`
  - 调节转矩因子和摩擦补偿：`set_drag_params()`
  - 录制示教轨迹：`trajectory_teach(true, name)`
  - 停止录制：`trajectory_teach(false, "")`
  - 获取轨迹列表：`check_teach()`
  - 回放选中的厂家轨迹：`trajectory_recorder(name)`

- `Handle MIT (planned)`
  - 已放入最终手柄模式需要的 UI 区：位置/姿态刚度、阻尼、捕获中心、进入 MIT、离合重置中心。
  - 目前全部禁用；在未实现实时 MIT 控制环前，不会下发任何 MIT/力矩指令。

- `Tracking (planned)`
  - 已预留轨迹源、启动/停止跟踪区域。
  - 等确定轨迹文件格式或外部实时目标来源后，接入 `track_joint()` / `track_pose()`。

- `Diagnostics`
  - 显示最新状态消息；
  - 累积 SDK 操作、错误和提示日志。

主要改动在：

- [主窗口 UI](./a3t_handle/src/ui/main_window.cpp)
- [SDK 操作与示教功能](./a3t_handle/src/arm/arm_worker.cpp)
- [工作线程接口](./a3t_handle/src/arm/arm_worker.hpp)

运行：

```bash
cd /home/user/A3-T/a3t_handle
cmake --build build -j
./build/a3t_handle
```

这样后续只需逐步解除 `Handle MIT` 和 `Tracking` 标签页中的禁用控件，并接入对应控制逻辑，不需要再重做整体 UI。

> 当前的项目结构概览，每一块实现的具体功能是什么

当前项目分为“厂商 SDK”和“我们自己的桌面上位机”两部分：

```text
A3-T/
├── arm_control_sdk/        厂商提供，不修改
└── a3t_handle/             我们的独立桌面程序
```

`arm_control_sdk/` 的主要作用：

```text
include/    C++ 接口定义，例如连接、状态、拖动、MIT、轨迹
lib/        编译好的 SDK 动态库
carm_python/ Python 封装与文档，可用于快速验证接口
```

`a3t_handle/` 当前结构和实际功能：

```text
a3t_handle/
├── CMakeLists.txt
├── config/
│   └── handle.json
├── src/
│   ├── main.cpp
│   ├── arm/
│   │   ├── arm_types.hpp
│   │   ├── arm_worker.hpp
│   │   └── arm_worker.cpp
│   ├── control/
│   │   ├── handle_mode_manager.hpp
│   │   └── handle_mode_manager.cpp
│   ├── safety/
│   │   ├── safety_monitor.hpp
│   │   └── safety_monitor.cpp
│   └── ui/
│       ├── main_window.hpp
│       └── main_window.cpp
├── math/                   预留，尚未实现
├── logging/                预留，尚未实现
├── app/                    预留，尚未实现
├── tests/                  预留，尚未实现
└── docs/                   预留，尚未实现
```

各模块职责：

| 模块 | 当前实现 |
|---|---|
| `main.cpp` | 启动 Qt 程序，读取 `config/handle.json`，创建主窗口。 |
| `config/handle.json` | 控制器 IP、UI 状态轮询周期、Drag 的默认转矩因子和摩擦补偿。 |
| `ui/main_window.*` | 最终版布局：连接安全、Drag 示教、Handle MIT 预留、Tracking 预留、诊断日志。 |
| `arm/arm_types.hpp` | 定义 UI/程序使用的状态快照：连接、伺服、安全、厂商 FSM、实机/仿真、关节角、末端位姿、应用模式。 |
| `arm/arm_worker.*` | 唯一调用厂商 SDK 的线程。实现连接、Ready、下使能、急停、状态读取、Drag、速度、碰撞配置、示教录制、轨迹回放。 |
| `control/handle_mode_manager.*` | 管理应用层 `Monitor` / `Handle` 状态，并检查进入手柄模式前是否连接、使能、安全正常。 |
| `safety/safety_monitor.*` | 基础安全门：检测控制器断连和错误状态。后续要加入关节限位、看门狗、力矩限幅和工作空间限制。 |
| `math/` | 未来实现位姿误差、四元数、雅可比矩阵处理、\(J^TW\)、六维阻抗模型。 |
| `logging/` | 未来记录时间戳、状态、MIT 指令、轨迹、控制周期和错误。 |
| `tests/` | 未来放假机械臂客户端、数学单元测试、状态机测试。 |

当前真正可执行的功能集中在 `ArmWorker`：

```text
连接与断开
读取关节角和末端位姿
Vendor Ready
伺服下使能
急停
速度等级
碰撞保护
厂家 Drag 参数与拖动模式
厂家示教轨迹录制、列表读取、回放
```

当前仅有 UI、未实现控制逻辑的功能：

```text
Handle MIT：
  中心点、六维刚度/阻尼、MIT 控制环、重力补偿、虚拟墙

Tracking：
  自定义轨迹源、track_joint / track_pose 实时跟踪
```

最重要的架构约束已经建立：`ui/` 不会直接调用机械臂；所有 SDK 调用都集中在 `arm/arm_worker.cpp` 的独立线程中。

> 能不能打开当前UI看看效果

<details><summary>8 previous messages</summary>

> 我现在启动已编译的桌面程序查看实际布局；它不会自动连接机械臂，只有点击 Connect 才会发起通信。
>
> <details><summary>Ran a command</summary>
>
> - Ran `./build/a3t_handle`
>
> </details>
>
> 程序已启动到动态库加载阶段，但厂商 SDK 的间接依赖 `libjsoncpp.so.1` 没被运行时定位到。这是构建链接配置遗漏，不涉及机械臂；我会把 SDK 自带的 JSON 库显式链接进去后再启动。
>
> <details><summary>Edited a file, ran a command</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+2 -1)
> - Ran `cmake -S . -B build && cmake --build build -j2 && ./build/a3t_handle`
>
> </details>
>
> SDK 只附带了带版本号的 `libjsoncpp.so.1`，没有开发环境常见的无版本链接名；我会改为直接引用它的实际文件名。
>
> <details><summary>Edited a file, ran 2 commands</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+1 -1)
> - Ran `cmake -S . -B build && cmake --build build -j2 && ./build/a3t_handle`
> - Wrote `./a3t_handle/CMakeLists.txt` (+1 -0)
> - Ran `cmake -S . -B build && cmake --build build -j2 && ./build/a3t_handle`
>
> </details>
>
> 程序现在已成功启动；同时我补齐了厂商 SDK 的运行时 JSON 库依赖。接着我会抓取当前桌面窗口画面，确认实际 UI 布局。
>
> <details><summary>Ran a command</summary>
>
> - Ran `command -v gnome-screenshot || command -v scrot || command -v import`
>
> </details>
</details>

UI 已成功启动，应已显示在桌面上。

同时修复了启动时厂商 SDK 找不到 `libjsoncpp.so.1` 的问题，现在直接运行即可：

```bash
cd /home/user/A3-T/a3t_handle
./build/a3t_handle
```

我还更新了 [CMakeLists.txt](./a3t_handle/CMakeLists.txt)，确保之后重新编译的程序也能找到 SDK 自带运行库。

> 挺好的，现在还需要完成哪些内容，后续步骤及流程是什么

下一步的核心不是继续做 UI，而是把 `Handle MIT` 标签页从“预留界面”变成真正的实时控制后端。

建议按以下顺序推进。

1. 真机验证现有基础功能

先验证当前 UI 已有链路：

```text
Connect
→ Vendor Ready
→ 读取状态、关节角、末端位姿
→ Vendor Drag
→ 调节拖动参数
→ 示教录制 / 回放
→ Return to Idle
→ Disable Servo
```

目标是确认 SDK、网络、固件版本和安全退出流程正常。

2. 增加性能与日志模块

实现 `logging/`，记录：

- 状态回调时间戳；
- 关节角、速度、力矩；
- 末端位姿；
- 控制命令和返回码；
- 控制周期 `dt`、平均频率、最大延迟、最大抖动；
- 故障和模式切换事件。

先测清楚：

```text
关节回调频率
末端位姿回调频率
low_mit_command() 单周期耗时
通信中断时的行为
```

这是进入 MIT 的前提。

3. 实现 MIT 安全框架

在 `control/` 与 `safety/` 中新增 MIT 控制线程，但第一版不要直接做大刚度末端弹簧。

应先完成：

```text
进入 MIT
→ 读取状态
→ 周期性发送低风险 MIT 指令
→ 力矩/速度/位置/变化率限幅
→ 通信看门狗
→ 异常时退出 MIT → Idle / 下使能
```

此阶段需要新增：

```text
src/control/mit_control_loop.*
src/safety/command_limiter.*
src/logging/control_recorder.*
```

4. 做关节空间低刚度手推实验

进入 MIT 时记录当前关节中心 \(q_0\)，使用重力补偿和低刚度阻尼：

\[
\tau =
K_p(q_0-q)
-
K_d\dot q
+
g(q)
\]

目标不是最终手感，而是验证：

- 人可以推开机械臂；
- 机械臂不会下坠；
- 松手后会稳定回中；
- 参数变化不会振荡；
- MIT 退出正常。

5. 实现最终核心：末端六维阻抗手柄

在 `math/` 中实现：

```text
位姿误差
四元数姿态误差
末端速度计算
六维弹簧阻尼
雅可比转置映射
重力补偿
```

每周期执行：

\[
W=
\begin{bmatrix}
K_p(p_0-p)-D_pv\\
K_Re_R-D_R\omega
\end{bmatrix}
\]

\[
\tau_{ff}=J(q)^TW+g(q)
\]

然后调用：

```cpp
low_mit_command(...)
```

此时 `Handle MIT` UI 中的刚度、阻尼、捕获中心、离合/重置中心按钮才正式启用。

6. 完善手柄交互

实现：

- 捕获中心位姿；
- 平移与姿态分开调节刚度、阻尼；
- 按住离合时重置中心；
- 死区、缩放、滤波；
- 虚拟墙、关节限位排斥；
- 物理死手开关；
- 实时显示当前相对 6DOF 位移和输出指令。

7. 最后再做 Tracking

Tracking 是辅助功能，可分两步：

```text
拖动示教 → 厂家轨迹回放
```

然后：

```text
自定义轨迹文件 / 外部实时目标
→ track_joint() 或 track_pose()
→ 轨迹跟踪执行
```

衣物、毛巾等高速抖动任务应放在 Tracking 后期：先根据速度、加速度、jerk 限制生成可行轨迹，再从低幅度、低频率逐步测试。

整体流程可概括为：

```text
基础连接与安全
→ 真机验证 Drag
→ 测量实时能力
→ MIT 安全循环
→ 关节空间手推
→ 六维笛卡尔阻抗手柄
→ 输出给从端 / 仿真
→ 示教、Tracking 与高速柔性物体任务
```

当前最需要补的是第 2、3、4 步；它们做稳后，最终手柄模式才有可靠基础。

> 因为当前的代码项目都在台式电脑上，我计划先完成足够多的功能再去现场测试。要不先完成logging？然后再完成基于MIT的手柄模式初版？

这个顺序很合适：

```text
先完成 Logging
→ 再完成 MIT 手柄控制的软件框架与离线计算
→ 最后去现场做真实 MIT 联调和参数调试
```

这样可以尽可能把现场时间留给必须依赖真机的部分。

第一步先做 `logging/`，建议至少记录四类数据：

```text
events.csv
  模式切换、连接/断开、Ready、急停、错误、参数修改

state.csv
  时间戳、连接/伺服/FSM、q、dq、tau、末端位姿

command.csv
  时间戳、下发的模式、拖动参数、速度等级、碰撞设置
  后续加入 MIT 的 qd、dqd、tau_ff、Kp、Kd

performance.csv
  控制循环 dt、平均周期、最大周期、超时次数、SDK 调用耗时
```

日志应按“会话”创建目录，例如：

```text
logs/
└── 2026-09-24_14-30-21/
    ├── events.csv
    ├── state.csv
    ├── command.csv
    └── performance.csv
```

这样现场测试后才能回答：

- 实际状态回调频率是多少；
- 拖动时的关节速度、力矩是什么范围；
- MIT 指令是否丢包或超时；
- 某个 \(K_p/K_d\) 参数为什么产生振荡；
- 某次异常发生前机械臂处于什么状态。

Logging 完成后，再做 MIT 手柄模式的“离线可完成部分”：

```text
math/
  位姿、四元数误差、雅可比矩阵处理
  六维弹簧阻尼公式
  力矩限幅和变化率限幅

control/
  MitControlLoop 状态机
  HandleMitController
  中心位姿捕获
  离合 / 重置中心
  参数配置与应用

safety/
  控制周期看门狗
  指令限幅
  MIT 进入 / 退出流程
```

在台式机上先以 `Dry Run` 方式运行：

```text
读取或模拟 q、dq、pose
→ 计算 W
→ 计算 tau_ff
→ 写入 command.csv
→ 不调用 low_mit_command()
```

这样可以验证数学、参数范围、日志、状态机和 UI 流程，不会对机械臂发送力矩。

去现场后再把：

```text
Dry Run
```

切换为：

```text
Real MIT
```

并先从极低刚度、低阻尼、低力矩上限开始。

所以建议下一项直接实现：

> `logging/` 模块，先让现有连接、Drag、示教、速度、碰撞设置和所有事件都可记录；随后再实现 MIT 手柄的 Dry Run 控制器。

> 先完成logging吧，对了，log文件保存在哪里合适？

<details><summary>4 previous messages</summary>

> 日志适合保存到项目内、但与源码和构建产物分离的 `a3t_handle/runtime_logs/`。我会实现按运行会话自动建目录的 CSV 日志，并把该目录加入忽略规则；每次程序运行都会生成独立会话，方便带到现场分析。
>
> <details><summary>Created 3 files, edited 5 files, ran a command</summary>
>
> - Wrote `./a3t_handle/.gitignore` (+2 -0)
> - Wrote `./a3t_handle/CMakeLists.txt` (+1 -0)
> - Wrote `./a3t_handle/config/handle.json` (+1 -0)
> - Wrote `./a3t_handle/src/logging/session_logger.cpp` (+80 -0)
> - Wrote `./a3t_handle/src/logging/session_logger.hpp` (+39 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+29 -1)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+2 -0)
> - Wrote `./a3t_handle/src/logging/session_logger.cpp` (+2 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+1 -0)
> - Ran `cmake -S . -B build && cmake --build build -j2`
>
> </details>
>
> 日志模块已接入并编译通过。我会用离屏方式短暂启动一次程序，确认会话目录和四个 CSV 文件能够自动创建；不会连接机械臂。
>
> <details><summary>Explored a search, a list, ran a command</summary>
>
> - Ran `timeout 1s env QT_QPA_PLATFORM=offscreen ./build/a3t_handle`
>
> <details><summary>Explored a search, a list</summary>
>
> - Listed files in `runtime_logs`
> - Searched for `session_started|session_finished` in `runtime_logs`
>
> </details>
>
> </details>
</details>

Logging 已完成，日志会保存在：

```text
/home/user/A3-T/a3t_handle/runtime_logs/
```

每次启动程序会自动创建独立会话目录，例如：

```text
runtime_logs/
└── 20260924_131938_793/
    ├── events.csv
    ├── commands.csv
    ├── states.csv
    └── performance.csv
```

已用离屏启动验证：目录和四个 CSV 文件会自动生成；不会连接机械臂。

各文件内容：

- `events.csv`：启动、连接、断开、故障、会话结束等事件。
- `commands.csv`：Ready、伺服下使能、Drag 参数、速度等级、碰撞配置、示教开始/停止/回放、急停等命令。
- `states.csv`：连接、伺服、安全、厂商 FSM、实机/仿真、关节角、末端位姿、错误信息。
- `performance.csv`：当前记录状态轮询耗时；后续 MIT 控制环可在此记录周期、抖动和下发耗时。

新增实现位于：

- [session_logger.hpp](./a3t_handle/src/logging/session_logger.hpp)
- [session_logger.cpp](./a3t_handle/src/logging/session_logger.cpp)
- [arm_worker.cpp](./a3t_handle/src/arm/arm_worker.cpp)

`runtime_logs/` 已加入 `.gitignore`，不会混入源码或构建文件。

下一步可以开始搭建 `HandleMitController` 的 Dry Run：读取或模拟关节状态，计算六维阻抗力矩并写入日志，但暂不调用 `low_mit_command()`。

> ok, 先完成手柄模式的初版，完成后把当前的算法说明白，并说和最终版的区别，欠缺什么

<details><summary>4 previous messages</summary>

> 我会先实现可在台式机上安全完成的 `Handle V0`：以厂家 Drag 作为物理后端，捕获末端中心位姿、连续计算相对 6DOF 位移/姿态、速度和虚拟六维弹簧阻尼，并写入日志；它不会发送 MIT 力矩。这样手柄算法、UI、日志和状态机先完整跑通，现场再把同一控制器接到 MIT 力矩后端。
>
> <details><summary>Created 2 files, edited 9 files, explored a file, a search, ran a command</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+1 -0)
> - Wrote `./a3t_handle/src/math/handle_controller.cpp` (+73 -0)
> - Wrote `./a3t_handle/src/math/handle_controller.hpp` (+48 -0)
>
> - Wrote `./a3t_handle/src/logging/session_logger.cpp` (+14 -1)
> - Wrote `./a3t_handle/src/logging/session_logger.hpp` (+4 -0)
>
> - Wrote `./a3t_handle/config/handle.json` (+8 -0)
> - Wrote `./a3t_handle/src/arm/arm_types.hpp` (+3 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+64 -2)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+6 -0)
>
> <details><summary>Explored a file, a search</summary>
>
> - Searched for `handle_controller|captureHandle|enterHandle|exitHandle|application_mode ==` in `arm_worker.cpp`
> - Read `./src/ui/main_window.cpp`
>
> </details>
>
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+48 -18)
> - Wrote `./a3t_handle/src/ui/main_window.hpp` (+3 -0)
>
> - Ran `cmake -S . -B build && cmake --build build -j2`
>
> </details>
>
> Handle V0 已接入：中心捕获、六维误差/滤波/虚拟力计算、参数面板和 `handle.csv` 日志均已完成并编译通过。我会再做一次离屏启动，确认新版本能正常初始化而不连接机械臂。
>
> <details><summary>Ran a command</summary>
>
> - Ran `timeout 1s env QT_QPA_PLATFORM=offscreen ./build/a3t_handle`
>
> </details>
</details>

手柄模式初版 `Handle V0` 已完成并编译通过。它目前是“可运行的手柄算法 + Drag 物理后端 + 完整日志”，不发送 MIT 力矩。

主要新增：

- [HandleController](./a3t_handle/src/math/handle_controller.cpp)：纯数学控制器。
- [ArmWorker](./a3t_handle/src/arm/arm_worker.cpp)：捕获中心、运行控制器、写入输出日志。
- [Handle V0 UI](./a3t_handle/src/ui/main_window.cpp)：中心捕获、离合重置、参数调整、实时输出显示。
- `handle.csv`：新增到每个日志会话，记录相对位姿、虚拟弹簧阻尼力和 6DOF 输出。

当前使用流程：

```text
Connect
→ Vendor Ready
→ Capture center pose
→ 调整并 Apply Handle parameters
→ Enter Handle V0
→ 人手拖动末端
→ 实时显示并记录相对 6DOF 输出和虚拟力
→ Return to Idle
```

进入 `Handle V0` 后，物理机械臂仍在厂家 Drag 模式中：

```cpp
set_control_mode(3)
```

所以人可以安全地推着末端移动；程序持续读取当前末端位姿，完成“手柄输入”算法。

当前算法是：

\[
e_p=p-p_0
\]

\[
e_R=\mathrm{Log}(R_0^{-1}R)
\]

其中 \(p_0,R_0\) 是点击 `Capture center pose` 时保存的中心位置、中心姿态。

对位置和姿态误差分别进行死区与一阶低通滤波：

\[
\tilde e(t)=\alpha e(t)+(1-\alpha)\tilde e(t-1)
\]

再计算手柄输出：

\[
u=
\begin{bmatrix}
s_p \tilde e_p\\
s_R \tilde e_R
\end{bmatrix}
\]

这里的 \(u\) 是六维相对控制量，包含平移 XYZ 和旋转三轴。

同时计算虚拟弹簧阻尼力：

\[
W=
\begin{bmatrix}
-K_p\tilde e_p-D_p\dot{\tilde e}_p\\
-K_R\tilde e_R-D_R\dot{\tilde e}_R
\end{bmatrix}
\]

其中：

- `Position/Rotation scale`：输出给从端或仿真的控制量缩放；
- `Position/Rotation stiffness`：虚拟弹簧刚度；
- `Position/Rotation damping`：虚拟阻尼；
- 当前虚拟力 \(W\) 会显示并写入 `handle.csv`。

当前版本与最终 MIT 手柄的差异：

| 项目 | Handle V0 当前版本 | 最终 Handle MIT |
|---|---|---|
| 机械臂物理后端 | 厂家 Drag | MIT |
| 人可否拖动 | 可以，厂家负责 | 可以，由虚拟弹簧阻尼决定 |
| 中心点 | 已实现 | 已实现 |
| 6DOF 相对输出 | 已实现 | 已实现 |
| 虚拟六维弹簧阻尼计算 | 已实现 | 已实现 |
| 虚拟力是否施加到机械臂 | 否，只显示和记录 | 是 |
| 重力补偿 | 厂家 Drag 内部完成 | 由 \(g(q)\) 参与力矩控制 |
| 雅可比矩阵映射 | 未使用 | \(\tau=J^TW+g(q)\) |
| MIT 指令 | 不发送 | 每周期发送 `low_mit_command()` |
| 控制周期 | 当前 UI/轮询级别 | 独立实时控制循环 |
| 通信看门狗、力矩限幅 | 基础框架 | 必须完整实现 |

最终版还欠缺：

```text
实时关节状态回调与周期测量
MIT 进入 / 退出流程确认
雅可比矩阵与重力项计算
JᵀW → 关节前馈力矩
低风险 MIT 初始参数
力矩、速度、位置、变化率限幅
通信超时保护
物理死手开关
虚拟墙、关节限位和奇异位形保护
将 6DOF 输出连接至仿真或从端机械臂
```

所以现在的 `Handle V0` 已经把最终手柄的“中心、六维误差、滤波、缩放、虚拟弹簧阻尼、参数 UI、日志”完成了；最终版主要是把计算得到的虚拟力真正通过 MIT 安全地作用到机械臂上。

> 直接完成MIT的最终手柄版，然后说明当前算法，之后再进行改进？

<details><summary>2 previous messages</summary>

> 我会实现“可部署但默认硬件互锁”的 MIT 最终手柄后端：独立周期循环、中心六维阻抗、雅可比转置映射、重力补偿、力矩/变化率限幅、失败看门狗和安全退出。由于没有真机，默认不会允许真实 MIT 下发；现场确认接口时只需解除配置互锁并从保守参数开始。
>
> <details><summary>Created 2 files, edited 5 files, ran a command</summary>
>
> - Wrote `./a3t_handle/CMakeLists.txt` (+1 -0)
> - Wrote `./a3t_handle/config/handle.json` (+9 -0)
> - Wrote `./a3t_handle/src/control/mit_handle_controller.cpp` (+42 -0)
> - Wrote `./a3t_handle/src/control/mit_handle_controller.hpp` (+35 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+7 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+83 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.hpp` (+1 -0)
> - Wrote `./a3t_handle/src/arm/arm_worker.cpp` (+3 -1)
> - Wrote `./a3t_handle/src/ui/main_window.cpp` (+3 -0)
> - Ran `cmake -S . -B build && cmake --build build -j2`
>
> </details>
</details>

MIT 手柄后端已接入并编译通过，但默认处于硬件互锁状态，不会误向真机发送 MIT 力矩。

启用条件在 [handle.json](./a3t_handle/config/handle.json)：

```json
"mit": {
  "allow_real_mit": false
}
```

只有现场完成安全检查后才应改为 `true`。当前 UI 增加了：

```text
Enter Handle V0
Enter Handle MIT (hardware locked)
```

MIT 相关新增文件：

- [mit_handle_controller.hpp](./a3t_handle/src/control/mit_handle_controller.hpp)
- [mit_handle_controller.cpp](./a3t_handle/src/control/mit_handle_controller.cpp)

## 当前最终算法

进入 MIT 手柄前：

```text
捕获中心末端位姿 T0
→ 配置平移/旋转刚度与阻尼
→ 进入底层模式
→ set_control_mode(2)
→ 启动独立 MIT 周期定时器
```

每个 MIT 周期执行：

```text
low_refresh()
  → 得到 q、dq

low_get_forward_kine(q)
  → 得到当前末端位姿 T

HandleController
  → 计算中心相对误差 e
  → 滤波、死区
  → 计算虚拟六维弹簧阻尼 W

low_get_jacobian(q)
  → 得到 J

low_get_dynamics(q, dq, 0)
  → 得到重力项 g(q)

τff = JᵀW + g(q)
  → 力矩限幅
  → 力矩变化率限幅

low_mit_command(q, dq, τff, Kp, Kd)
```

其中：

\[
W=
\begin{bmatrix}
-K_p(p-p_0)-D_pv\\
-K_R e_R-D_R\omega
\end{bmatrix}
\]

\[
\tau_{ff}=J(q)^TW+g(q)
\]

MIT 指令中的：

```text
pos = 当前 q
vel = 当前 dq
```

用于让 MIT 内部的位置、速度误差尽量接近零；主要控制作用来自你们计算的前馈力矩 `τff`。这正是“末端中心点六维虚拟弹簧阻尼”的实现路径。

已实现的 MIT 保护：

- `allow_real_mit=false` 默认互锁；
- 关节力矩绝对值限幅；
- 力矩变化率限幅；
- 任一底层状态、运动学、动力学或 MIT 命令失败时退出 MIT；
- 退出时停止定时器、停止任务、回到 Idle、关闭底层模式；
- MIT 周期耗时写入 `performance.csv`；
- 六维误差、输出和虚拟力写入 `handle.csv`。

当前仍欠缺、需要现场改进的部分：

- 验证厂商 `set_low_mode(true)`、MIT 模式和 `low_mit_command()` 的准确调用顺序；
- 测量真实周期、延迟和抖动；
- 验证雅可比矩阵的行列方向及坐标系定义；
- 校准重力项的符号、工具负载和末端坐标；
- 确认 `Kp/Kd=0` 是否被当前固件允许，或改为厂家推荐最小稳定值；
- 加入连续失败次数阈值、完整通信看门狗；
- 加入关节限位、工作空间、奇异位形、速度和加速度保护；
- 使用实时回调或控制器侧控制，减少连续 WebSocket 动力学查询的延迟；
- 接入物理死手开关；
- 从极低刚度、极低力矩上限开始真机调参。

程序构建已验证通过。下一步不应立即解除互锁运行 MIT，而是先在现场用日志测量底层调用周期和状态，再逐步解锁低力矩测试。

> ok，再重新总结下当前进度，后续流程步骤规划

当前项目已经完成“桌面上位机框架 + 拖动示教 + 手柄算法 + MIT 后端代码骨架”。

当前结构：

```text
a3t_handle/
├── ui/         Qt 桌面界面
├── arm/        唯一调用厂商 SDK 的通信与控制线程
├── control/    应用模式、MIT 力矩控制器
├── math/       位姿、六维误差、虚拟弹簧阻尼
├── safety/     基础状态安全检查
├── logging/    CSV 会话日志
└── config/     IP、拖动参数、手柄参数、MIT 安全参数
```

当前已实现并可编译的功能：

| 模块 | 已完成内容 |
|---|---|
| 连接与安全 | 连接、断开、Vendor Ready、下使能、软件急停、状态显示 |
| 状态读取 | 关节角、末端位姿、连接、伺服、安全、厂商 FSM、实机/仿真 |
| Drag | 进入/退出厂家拖动模式、拖动转矩与摩擦补偿在线调整 |
| 示教 | 厂家轨迹录制、停止、列表获取、轨迹回放 |
| UI | 连接安全、Drag 示教、Handle、Tracking 预留、诊断五个区域 |
| Logging | 会话目录、事件、命令、状态、性能、手柄输出日志 |
| Handle V0 | 捕获中心位姿、相对 6DOF 位姿、死区、滤波、缩放、虚拟六维弹簧阻尼计算 |
| Handle MIT | 低层状态读取、正运动学、雅可比、动力学、\(J^TW+g(q)\)、MIT 指令、力矩/变化率限幅、失败退出 |
| 硬件保护 | MIT 默认由 `allow_real_mit=false` 互锁，不会误发力矩 |

尚未现场验证的部分：

```text
所有真机控制行为
厂家 Drag 的实际手感
Ready 与 FSM 的版本一致性
MIT 的准确进入顺序
MIT 周期、延迟、抖动
Jacobian 坐标系和矩阵方向
重力补偿、工具负载和力矩符号
Kp/Kd 的固件允许范围
```

后续建议严格按这个顺序进行。

1. **真机验证基础 UI**

```text
Connect
→ Vendor Ready
→ 状态是否正常
→ Vendor Drag
→ 拖动参数调整
→ Return to Idle
→ Disable Servo
→ 急停
```

目标：验证所有基础 SDK 调用与日志。

2. **拖动与示教验证**

```text
进入 Vendor Drag
→ 录制短轨迹
→ 停止
→ 刷新列表
→ 回放
```

目标：确认轨迹、日志、关节状态和末端位姿数据可信。

3. **MIT 通信性能测量**

保持 `allow_real_mit=false`，先利用现有日志确认：

- 状态读取周期；
- SDK 查询耗时；
- 网络延迟与最大抖动；
- `low_refresh()`、运动学、雅可比、动力学调用耗时。

目标：确定可用控制频率，判断开发电脑是否适合跑实时 MIT 环。

4. **低风险 MIT 联调**

在空旷工作空间、急停可达、低力矩限制下：

```text
解除 allow_real_mit 互锁
→ 极低 K/D
→ 极低 torque_limit_nm
→ 捕获中心
→ 短时间进入 MIT
→ 观察日志、姿态、力矩
→ 立即退出
```

目标：确认不下坠、不突跳、能正常退出。

5. **调试六维中心弹簧手感**

逐步调节：

```text
位置刚度
位置阻尼
旋转刚度
旋转阻尼
关节 MIT Kp / Kd
力矩上限
力矩变化率上限
```

目标：实现“可推开、稳定、松手回中、不振荡”。

6. **补齐安全层**

- 连续通信失败阈值；
- 控制周期超时检测；
- 关节限位与工作空间保护；
- 奇异位形保护；
- 速度、加速度限制；
- 物理死手开关；
- 进入/退出 MIT 平滑过渡。

7. **接入实际手柄输出**

将 `Handle V0 / MIT` 的六维相对输出接给：

```text
仿真机械臂
或从端实体机械臂
或相机、移动机器人等受控对象
```

加入缩放比例、坐标系映射、离合与重置中心。

8. **后续辅助功能**

- 完成 Tracking：自定义轨迹文件、`track_joint()`、`track_pose()`；
- 高速正弦/扫频轨迹；
- 柔性物体甩动任务；
- 虚拟墙、导向槽；
- 自适应刚度和阻尼；
- 基于电流/外力估计的触觉反馈。

当前代码最关键的状态是：

> MIT 六维阻抗的算法和程序流程已经具备，但尚未经过真机参数、时序与安全验证。下一步工作的重点应从“继续写功能”转向“记录数据、低风险联调、逐步调参”。

> 还有重力补偿怎么做，如果sdk本身给的补偿不好，有没有留个接口给我们改良。还有就是这些计算过程是能在机械臂自己的控制器算好还是依赖我们的电脑。因为之前我测试上位机好像就是可以发布移动角度命令，那些程序也是在我们电脑上跑吗，那是不是说周期时间也依赖电脑的性能？

<details><summary>3 previous messages</summary>

> Context compaction
>
> Source: automatic
> Status: completed
>
> 怎么还没好
>
> 可以改良，但要分清“能自己算”与“能自己稳定地下发”。
>
> 重力补偿的基本形式是：
>
> \[
> \tau_{\text{cmd}} = \tau_g(q) + J(q)^T W_{\text{virtual}}
> \]
>
> 其中：
>
> - \(\tau_g(q)\)：各关节为抵消机械臂自身重力所需的力矩；
> - \(W_{\text{virtual}}\)：末端虚拟弹簧阻尼产生的六维力/力矩；
> - \(J^T W\)：将末端力转换为各关节力矩。
>
> 目前 MIT 手柄代码的思路正是这个：SDK 的 `low_get_dynamics()` 给出动力学结果（主要可取重力项），我们自己计算虚拟弹簧阻尼和雅可比转置力矩，再通过 `low_mit_command()` 下发。以后当然能把 SDK 的动力学计算替换成自己的，例如基于 URDF 建立 Pinocchio / RBDL 模型，自己计算：
>
> \[
> \tau_g(q),\quad J(q),\quad M(q),\quad C(q,\dot q)
> \]
>
> 但这是后续优化项。前提是 URDF 的质量、质心、惯量、减速比、末端工具负载等参数可靠；否则“自己算的重力补偿”反而可能比厂家标定值更差。可以在代码中预留一个 `GravityCompensator` 接口：初期用 SDK，后期切换到自建模型或“SDK 结果 + 标定残差补偿”。:codex-annotation{index="1"}
>
> 至于计算在哪里跑，要分三层看：
>
> ```text
> 你的桌面程序
>   └─ 高层指令 / MIT 外环计算
>        └─ 以太网发送命令
>             └─ 机械臂控制器
>                  └─ 电机电流环、速度环、安全保护等底层实时控制
>                       └─ 电机
> ```
>
> 原厂网页上位机里“发布关节角度/末端位姿”的功能，网页本身和高层规划通常运行在电脑或控制器上的上位软件中；但机械臂控制器内部仍负责把轨迹插补成实时电机控制。因此，位置模式一般不太依赖你电脑的实时性能：你给一个“到这里去”的目标，控制器自己完成后续伺服。
>
> MIT 模式不同。若 SDK 的 `low_mit_command()` 是从你的电脑经网络持续发送 \(q_d,\dot q_d,K_p,K_d,\tau\)，那么：
>
> - 电机最底层电流闭环仍在控制器内，通常很快；
> - 但你的“虚拟弹簧、重力补偿、雅可比转换”外环跑在电脑上；
> - 外环频率和抖动会受电脑性能、操作系统调度、SDK 调用耗时、以太网延迟影响；
> - 所以它不能仅靠“电脑很快”保证稳定，更关键是通信是否稳定、控制器是否有实时接口、每周期调用是否足够轻。
>
> 我们目前的桌面程序用 `QTimer` 做 MIT 循环，只适合做接口连通性和低风险验证，不是严格实时系统。现场测试时需要记录每周期的：
>
> \[
> T_{\text{cycle}},\quad T_{\text{refresh}},\quad T_{\text{dynamics}},\quad
> T_{\text{jacobian}},\quad T_{\text{send}}
> \]
>
> 再判断它能否稳定在例如 100 Hz；若要抖毛巾那种高动态动作，通常需要远高于普通 Qt 桌面循环的确定性，并且还受机械臂本身速度、加速度、力矩、减速器和安全限制约束。
>
> 最合理的演进路径是：
>
> 1. 先用 SDK 的动力学和雅可比，验证 MIT 通信频率、重力项符号、坐标系与安全停机；
> 2. 给重力补偿抽象接口，保留“SDK / 自建模型 / 修正表”三种实现；
> 3. 用日志对比不同姿态下“松手后的下垂力矩”，标定工具重量与质心；
> 4. 若性能不足，再把控制循环从 Qt UI 线程体系中拆出，使用独立高优先级控制进程；若控制器允许部署用户程序，则优先把外环放到控制器侧。
</details>

刚才是在回答“重力补偿和计算位置”的设计问题，没有执行新的代码修改，所以不会出现新的完成项。

当前已完成的是：日志、桌面 UI、原厂拖动/示教、Handle V0，以及被安全开关锁住的 MIT 手柄控制框架。

下一项若继续做，就是把“重力补偿”改成可替换接口：先调用 SDK 动力学，后续可接入自己的 URDF 动力学模型。你确认的话我现在直接开始补这个结构。

对。若目标控制频率是 **100 Hz**，一个完整 MIT 周期必须在 **10 ms 内完成**：

\[
T_{\rm refresh}+T_{\rm FK}+T_{\rm Jacobian}+T_{\rm dynamics}
+T_{\rm controller}+T_{\rm send}<10\text{ ms}
\]

但工程上不能只“刚好小于 10 ms”，还要留出调度和网络抖动余量。比较实际的目标是平均约 3–5 ms，最坏情况也尽量不超过 8–9 ms。:codex-annotation{index="1"}

例如某一次周期：

- 读取状态：2 ms
- 雅可比与动力学：1 ms
- 算法：0.1 ms
- 下发 MIT：2 ms

合计约 5.1 ms，100 Hz 有余量。

若目标是 1 kHz，则整个周期需要小于 1 ms；普通桌面电脑 + Qt 定时器 + 以太网 SDK 通常很难稳定做到。先以稳定 100 Hz 为现场验证目标更合理，再看厂家低层接口和控制器是否支持更高频率。

> 我现在想创建一个新的repo,保存该项目，需要上传哪些文件

新仓库只保存你自己开发的 `a3t_handle/`，不建议直接上传厂家 SDK、固件包、编译产物和运行日志。

应该上传：

```text
a3t_handle/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── config/
│   └── handle.json            # 不含真实密码/令牌即可
└── src/
    ├── arm/
    ├── control/
    ├── logging/
    ├── math/
    ├── safety/
    ├── ui/
    └── main.cpp
```

不上传：

```text
a3t_handle/build/              # CMake 编译产物
a3t_handle/runtime_logs/       # 运行日志，可能很大
arm_control_sdk/               # 厂家 SDK，通常受许可限制且含二进制库
固件 .deb / .zip
IDE 配置目录，如 .vscode/、.idea/（除非有意共享）
```

SDK 应在 `README.md` 写清楚获取和放置方式，例如：

```text
依赖：
- Linux amd64 版 arm_control_sdk
- 解压后放在项目同级目录：

A3-T/
├── a3t_handle/
└── arm_control_sdk/
```

可以在 `a3t_handle` 目录执行：

```bash
git init
git add CMakeLists.txt README.md .gitignore config src
git commit -m "Initial A3-T handle control project"
```

之后在 GitHub/Gitee 新建空仓库，再添加远程并推送。当前 `.gitignore` 已忽略 `build/` 和 `runtime_logs/`，这一点是合适的。

> 要不要把sdk也上传上去，比较方便我拉取？

可以方便拉取，但取决于厂家 SDK 的许可。

不建议把 SDK 的 `.so` 二进制库、示例、固件包直接提交到公开仓库；它可能受分发限制，也会让仓库变大。更推荐：

- 自己的私有仓库：先确认厂家许可允许后，可以上传 SDK；
- 公开仓库：不上传 SDK，只在 README 写下载链接、版本号和目录要求；
- 最实用：把 SDK 单独做成一个私有仓库或压缩包存储，主项目用固定版本说明引用它。

你当前项目可写成：

```text
A3-T/
├── a3t_handle/         # Git 仓库：你的代码
└── arm_control_sdk/    # 本地依赖，不提交
```

并在 README 固定要求 `arm_control_sdk main.dev-1.0.260819` 的 Linux amd64 版本。这样别人拉取主项目后，只要按说明放入 SDK 就能编译。

如果这个仓库只给你们团队、且厂家明确允许内部再分发，上传 SDK 到私有仓库也没有技术问题。

> ok,那就按这个思路帮我创立上传新的repo

<details><summary>6 previous messages</summary>

> 我先把 `a3t_handle` 初始化为只包含自研代码的 Git 仓库，并核对忽略规则；远程仓库的创建与推送还需要你的 GitHub/Gitee 地址或仓库链接。
>
> <details><summary>Ran 2 commands</summary>
>
> - Ran `git -C a3t_handle status --short --branch; printf '\n--- files ---\n'; rg --files a3t_handle -g '!build/**' -g '!runtime_logs/**' | sed -n '1,160p'; printf '\n--- gitignore ---\n'; sed -n '1,160p' a3t_handle/.gitignore; printf '\n--- git identity ---\n'; git config --get user.name; git config --get user.email`
> - Ran `sed -n '1,220p' a3t_handle/README.md; printf '\n--- cmake ---\n'; sed -n '1,220p' a3t_handle/CMakeLists.txt; printf '\n--- global identity ---\n'; git config --global --get user.name; git config --global --get user.email; printf '\n--- gh ---\n'; command -v gh || true; gh auth status 2>&1 || true`
>
> </details>
>
> 仓库目前尚未初始化，也没有配置 Git 提交身份；SDK 不会进入版本库。我会补齐可复现构建说明和更完整的忽略规则，再创建本地仓库并暂存初始版本。
>
> <details><summary>Edited 2 files, ran 2 commands</summary>
>
> - Wrote `./a3t_handle/.gitignore` (+17 -0)
> - Wrote `./a3t_handle/README.md` (+39 -24)
> - Ran `git init -b main a3t_handle && git -C a3t_handle add CMakeLists.txt README.md .gitignore config src && git -C a3t_handle status --short && git -C a3t_handle diff --cached --stat`
> - Ran `find a3t_handle/docs a3t_handle/tests -maxdepth 2 -type f -printf '%P\n' 2>/dev/null; git -C a3t_handle check-ignore -v build/a3t_handle runtime_logs/20260924_131938_793/events.csv || true; git -C a3t_handle status --short`
>
> </details>
>
> 已确认构建目录和日志目录会被忽略，厂家 SDK 也不在仓库目录内。现在把空的文档与测试目录一并纳入初始提交内容。
>
> <details><summary>Ran a command</summary>
>
> - Ran `git -C a3t_handle add docs tests && git -C a3t_handle status --short && git -C a3t_handle diff --cached --name-only`
>
> </details>
</details>

本地仓库已创建在 [a3t_handle](./a3t_handle)，默认分支为 `main`，初始文件已全部暂存。

我还更新了 [README.md](./a3t_handle/README.md)，写明 SDK 不提交、要求的版本和放置位置；`build/`、`runtime_logs/`、IDE 文件都已忽略。

现在还差两项外部信息，才能提交并上传：

1. Git 提交身份（姓名、邮箱）——当前电脑未配置；
2. GitHub 或 Gitee 的新仓库地址——本机没有 GitHub CLI 登录，无法替你在平台账户下创建远程仓库。

把仓库 URL 和你希望写入提交记录的姓名、邮箱发我，我就完成首个 commit 和 push。

> User context
>
> Images:
> - `~/Pictures/Screenshots/Screenshot from 2026-09-24 14-25-57.png`

这里建议这样填：

- Repository name：`a3t_handle`
- Description：`A desktop application for using A3-T robot arm as a 6-DOF physical handle.`
- Visibility：选 **Private**。项目涉及厂家 SDK 接口和机械臂控制，先私有更合适。
- Add README：保持 **Off**，本地已经有 README。
- Add `.gitignore`：保持 **No .gitignore**，本地已有并且更符合项目。
- Add license：选 **No license**。先不要随便授权；以后确认代码开源范围、厂家 SDK 协议后再补。

然后点击绿色的 **Create repository**。创建完成后，把页面提供的仓库地址复制给我，格式通常是：

```text
https://github.com/jyx0309/a3t_handle.git
```

我就能把当前已暂存的项目提交并推送上去。

> [https://github.com/jyx0309/a3t\_handle.git](https://github.com/jyx0309/a3t_handle.git)
> yuxuan 202577610\@qq.com

<details><summary>2 previous messages</summary>

> 我会将提交身份仅配置在这个项目内，创建首个提交，并连接到你给出的私有 GitHub 仓库后推送；厂家 SDK、构建产物和日志不会被包含。
>
> <details><summary>Ran a command</summary>
>
> - Ran `git -C a3t_handle config user.name 'yuxuan' && git -C a3t_handle config user.email '202577610@qq.com' && git -C a3t_handle commit -m 'Initial A3-T handle control application' && git -C a3t_handle remote add origin https://github.com/jyx0309/a3t_handle.git && git -C a3t_handle push -u origin main`
>
> </details>
</details>

本地首个提交已完成：

```text
faed96d  Initial A3-T handle control application
```

远程地址也已配置为你的 GitHub 仓库，但推送时 GitHub 要求登录认证；当前终端无法弹出用户名/密码输入，因此未能上传。

你可在本机终端进入项目后执行：

```bash
cd /home/user/A3-T/a3t_handle
git push -u origin main
```

若要求登录，GitHub 的 HTTPS 密码位置需要填写 **Personal Access Token**，不是 GitHub 账户密码。也可以先在 VS Code 的 GitHub 扩展登录，再执行这条命令。

提交已经在本地，认证完成后只需这一条 `git push` 即可上传。

> git push -u origin main
> Username for '[https://github.com](https://github.com)': jyx0309
> Password for '[https://jyx0309@github.com](https://jyx0309@github.com)':
> remote: Invalid username or token. Password authentication is not supported for Git operations.
> fatal: Authentication failed for '[https://github.com/jyx0309/a3t\_handle.git/](https://github.com/jyx0309/a3t_handle.git/)'

<details><summary>2 previous messages</summary>

> 这是 GitHub 的正常安全策略：HTTPS 推送已不能使用账户密码，需要用个人访问令牌（PAT）或改用 SSH。我先核对官方创建令牌入口和最小权限。
>
> <details><summary>Searched the web</summary>
>
> Searched the web for `https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens`
>
> Searched the web for `https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens`
>
> </details>
</details>

密码位置不能填 GitHub 登录密码，必须填 **Personal Access Token（PAT）**。GitHub 官方建议优先用权限更小的 fine-grained token，并可限定到单个仓库。[官方说明](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens)

操作：

1. GitHub 右上角头像 → **Settings** → **Developer settings**。
2. **Personal access tokens** → **Fine-grained tokens** → **Generate new token**。
3. Resource owner 选 `jyx0309`。
4. Repository access 选 **Only select repositories**，勾选 `a3t_handle`。
5. Repository permissions 中将 **Contents** 设为 **Read and write**。
6. 设置一个过期时间，例如 90 天，生成并复制 token。

然后重新执行：

```bash
cd /home/user/A3-T/a3t_handle
git push -u origin main
```

- Username：`jyx0309`
- Password：粘贴刚生成的 token

粘贴时终端不会显示字符，这是正常的。不要把 token 发给我，也不要提交进代码。

> 我是否能将这个对话过程分享给另一个电脑的codex,同一个账号，以同步信息。或者有什么更好的方法？

<details><summary>2 previous messages</summary>

> 这是 Codex 的跨设备对话与项目同步问题；我会查阅官方使用说明，确认同账号下可行的同步方式，并结合你当前 Git 仓库给出最稳妥的方案。
>
> <details><summary>Searched the web</summary>
>
> <details><summary>Explored a file</summary>
>
> - Read `~/.codex/skills/.system/openai-docs/SKILL.md`
>
> </details>
>
> Searched the web for `Codex share conversation across devices same account chat history`
>
> Searched the web for `site:help.openai.com "Sharing a Codex chat" ...`
>
> </details>
</details>

可以，但不要把“对话同步”当成项目同步的唯一方式。

同账号登录另一台电脑后，若这个 Codex 对话出现在历史记录中，可以直接继续；但本地工作目录、终端状态、SDK 文件、构建产物并不会随对话自动过去。官方也说明本地任务仍运行在各自电脑上；Codex 还有只读分享快照，但快照是静态的，且不含工具调用与终端输入输出。[OpenAI 官方说明](https://help.openai.com/en/articles/20001275-chatgpt-work-and-codex)

对这个项目，最稳妥的是三层同步：

1. GitHub：同步代码、README、配置和后续文档。你刚创建的仓库就是核心。
2. `docs/PROJECT_HANDOFF.md`：项目背景、SDK 版本、当前已完成能力、算法、未完成事项、现场测试注意事项。
3. Codex 对话：用于保留讨论过程。若界面有 Share，可分享一个只读快照给另一台电脑参考；分享前检查其中是否暴露本地路径、IP、日志或任何密钥。官方提示快照虽然会处理已知敏感格式，但仍需自行检查。[OpenAI 发布说明](https://help.openai.com/en/articles/6825453-chatgpt-release-notes)

换到另一台电脑后的实际流程：

```bash
git clone https://github.com/jyx0309/a3t_handle.git
cd a3t_handle
```

然后按 README 下载并放置厂家 SDK。打开该目录给 Codex，第一句可以写：

```text
请先阅读 README.md、docs/PROJECT_HANDOFF.md 和 git log。
这是 A3-T 六自由度手柄项目；继续完成当前任务，不要修改厂家 SDK。
```

目前最缺的是把本地提交真正 `push` 到 GitHub；完成 PAT 登录后推送即可。之后我建议直接帮你新建并维护 `docs/PROJECT_HANDOFF.md`，这比依赖长对话更适合跨电脑长期协作。

> 能不能生成一个codex可读的快照？

<details><summary>2 previous messages</summary>

> 可以。我会在仓库中生成一个面向下一位 Codex 的项目快照：包含背景、SDK 约束、已实现功能、当前 MIT 算法、未完成与安全边界；它会随 Git 同步，而不是依赖聊天记录。
>
> <details><summary>Created a file, ran 2 commands</summary>
>
> - Ran `git -C a3t_handle status --short --branch && sed -n '1,220p' a3t_handle/config/handle.json && sed -n '1,220p' a3t_handle/src/control/mit_handle_controller.cpp && sed -n '1,180p' a3t_handle/src/math/handle_controller.cpp`
> - Wrote `./a3t_handle/docs/PROJECT_HANDOFF.md` (+141 -0)
> - Ran `git -C a3t_handle add docs/PROJECT_HANDOFF.md && git -C a3t_handle commit -m 'Add Codex project handoff snapshot' && git -C a3t_handle status --short --branch`
>
> </details>
</details>

已生成并提交 Codex 可读快照：[PROJECT_HANDOFF.md](./a3t_handle/docs/PROJECT_HANDOFF.md)。

它包含项目目标、SDK 边界、当前功能、MIT 算法与安全状态、现场测试顺序，以及下一位 Codex 应遵守的开发规则。

新提交为：

```text
9350ede Add Codex project handoff snapshot
```

当前本地比远程多 1 个提交；在完成 GitHub PAT 登录后执行：

```bash
cd /home/user/A3-T/a3t_handle
git push
```

另一台电脑克隆项目后，让 Codex 先阅读 `README.md` 和 `docs/PROJECT_HANDOFF.md`，就能获得足够完整的项目上下文。

Token 不是只能用一次；在到期、撤销或被 GitHub 自动失效前，可以反复用于 `git push`。:codex-annotation{index="1"}

但 GitHub 为安全起见，**token 原文通常只在刚生成时显示一次**。如果当时没有保存下来，之后无法再次查看或复制，只能：

1. 在 GitHub 的 Token 页面撤销旧 token；
2. 生成一个新的 PAT；
3. 新建后立刻复制并安全保存到密码管理器。

GitHub 官方也建议将 token 视为密码，并设置过期时间。[官方说明](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens)

成功推送一次后，Git 通常会通过系统凭据管理器缓存它，后续一般不需要每次粘贴。
