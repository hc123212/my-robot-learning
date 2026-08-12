# 串联腿平衡车 LQR + VMC 平衡控制教程

> 对象模型：`balance_sim/balance_sim.xml`（COD-2026 平衡车，串联腿闭链结构）
> 前置知识：mujoco_tutorial 第 5 章（雅可比）、第 6 章（动力学接口）、第 7 章（控制与力施加）
> 本教程的代码骨架将逐步演化为 `balance_sim/lqr_balance.py`

---

## 目录

0. [本教程要做什么](#0-本教程要做什么)
1. [第 0 步：模型改造（浮动基座）](#1-第-0-步模型改造浮动基座)
2. [控制架构总览](#2-控制架构总览)
3. [状态定义与平衡位形](#3-状态定义与平衡位形)
4. [数值线性化：闭链系统的 A、B 矩阵](#4-数值线性化闭链系统的-a-b-矩阵)
5. [LQR：状态反馈增益](#5-lqr状态反馈增益)
6. [闭环仿真与验证](#6-闭环仿真与验证)
7. [VMC：虚拟模型控制](#7-vmc虚拟模型控制)
8. [LQR + VMC 融合](#8-lqr--vmc-融合)
9. [调参指南](#9-调参指南)
10. [验收标准与实施顺序](#10-验收标准与实施顺序)
11. [常见坑清单](#11-常见坑清单)

---

## 0. 本教程要做什么

目标是让这台串联腿平衡车在 MuJoCo 里**站稳**：给车体一个初始倾斜或外力扰动，它能自动回正并保持平衡。

控制方法组合：

- **LQR（线性二次调节器）**：在平衡位形附近把非线性系统线性化，求出最优状态反馈增益 $u = -K(x - x_0)$。负责"平衡"这个核心问题——车体俯仰角、轮子速度。
- **VMC（虚拟模型控制）**：在车体质心/腿端挂"虚拟弹簧-阻尼"元件，通过雅可比 $\tau = J^{\mathsf{T}}F$ 映射成关节力矩。负责"姿态/腿形"等外环任务，以及把虚拟力合理地分配到闭链机构的各个关节。

选型理由（为什么是这两个方法而不是别的）：

| 问题 | 为什么这样解决 |
|---|---|
| 闭链机构动力学难解析建模 | 数值线性化（有限差分）直接对"带约束的闭环系统"求 A、B，MuJoCo 在 `mj_step` 内部自动解算 connect 约束，不需要手推五杆机构动力学 |
| 平衡控制有明确最优解 | 平衡位形附近的 LQR 是欠驱动系统平衡的标准解法，Q/R 有清晰的物理含义 |
| 闭链的力分配麻烦 | VMC 用虚拟元件 + 雅可比映射，闭链内力交给约束求解器自动消化 |

---

## 1. 第 0 步：模型改造（浮动基座）

**为什么必须改**：当前 `balance_sim.xml` 的 base_link 直接挂在 `<worldbody>` 下（无 joint），车体被焊死在仿真世界。没有倾倒和移动，就不存在"平衡"。

**改什么**：给 base 加一个 `free` 关节（6 自由度浮动基座），并把四条腿全部挪进 base 体内。

```xml
<worldbody>
  <body name="base_link" pos="0 0 0.3">
    <!-- ① 浮动基座：x/y/z + 四元数，共 7 个广义坐标 -->
    <joint type="free"/>

    <!-- ② 关键：base 原模型没有 inertial（static），加 freejoint 后必须补质量，
          否则动力学奇异（qacc = NaN）。质量按整机倒推：
          腿部已知质量 8.0 kg（官方 inertia 字段合计），base 装车体/电池/电控，
          整机目标 ~16 kg → base 取 10 kg。
          惯量可用 mujoco.mj_estimateInertia(model, body_id, mass, inertia)
          从 base_link STL 网格自动估计（3.x 提供）。 -->
    <inertial pos="0 0 0.05" mass="10" diaginertia="0.12 0.18 0.14"/>

    <geom type="mesh" ... mesh="base_link"/>
    <!-- 原来的 Left/Right_front/rear 四条腿照旧，全部作为 base_link 的子 body -->
  </body>
</worldbody>
```

配套修改：

1. **keyframe 补 7 个数**：freejoint 的 qpos 前 7 个是 `(x, y, z, qw, qx, qy, qz)`。初始位置建议 `0 0 0.45`（轮子离地距离，让车自然落下接触地面），四元数 `1 0 0 0`，后面 14 个沿用当前闭合位形。
2. **闭合位形重新收敛**：加了 freejoint 后重力/约束反力分配变了，闭合解会微移。做法：无控制跑 3 秒（车倒下后约束自然收敛），把收敛后的 qpos 记为新的平衡位形 $x_0$。这个位形就是后面一切工作的基准。
3. 轮子与地面的接触（conaffinity=1）和地面已经在 `balance_sim.xml` 里修好，无需再动。

**验收**：headless 跑 5 秒——车体在重力下倒下、轮子接触地面、数值稳定（无 NaN、无能量爆炸）。

---

## 2. 控制架构总览

```
┌──────────────┐  外环（低频）             ┌───────────────────────────┐
│ VMC 虚拟力    │ → 虚拟弹簧阻尼            │  τ_leg = Jᵀ · F_virtual    │
│ (姿态/腿长)   │   (质心高度/俯仰/腿形)    │   → data.ctrl / qfrc      │
└──────┬───────┘                           └──────────┬────────────────┘
       │                                              │ 叠加
┌──────▼───────┐  内环（高频）             ┌──────────▼────────────────┐
│ LQR 状态反馈  │ → u = -K·(x - x₀)       │  轮子力矩 + 腿部力矩       │
│ (俯仰/轮速)  │   [θ, ω轮]              │   → data.ctrl             │
└──────────────┘                           └───────────────────────────┘
```

分工原则：

- **LQR 管"平衡"**：状态取车体俯仰角、俯仰角速度、轮子速度（+可选车体水平速度）。这是欠驱动核心——车体俯仰没有直接执行器，只能通过轮子力矩间接平衡。
- **VMC 管"姿态与腿形"**：质心高度、车体姿态的虚拟弹簧-阻尼。作用于腿关节（front/rear 大腿 + 闭链联动）。
- **轮子力矩**：LQR 输出直接给轮子（`Left/Right_Wheel_joint`）。

---

## 3. 状态定义与平衡位形

### 3.1 广义坐标（改造后 nq = 21，nv = 20）

| 分量 | 内容 |
|---|---|
| q[0:7] | freejoint：位置 x,y,z + 四元数 qw,qx,qy,qz |
| q[7:21] | 14 个腿部/轮子关节角（顺序同官方 XML） |
| v[0:6] | freejoint 速度：线速度 vx,vy,vz + 角速度 $\omega_x,\omega_y,\omega_z$ |
| v[6:20] | 关节角速度 |

### 3.2 控制输入（nu = 6）

官方 actuator 顺序：`Left_front`、`Left_rear`、`Left_Wheel`、`Right_front`、`Right_rear`、`Right_Wheel`。

### 3.3 平衡位形 $x_0$

$x_0 = (q_0, 0)$，其中 $q_0$ 是第 1 节"重新收敛"得到的闭合位形（当前闭合位形见 `balance_sim.xml` 的 keyframe，freejoint 后需重收敛）。

**重要**：$x_0$ 必须同时满足两个条件：

1. 在闭链约束流形上（site 残差 < 0.5mm）
2. 轮子与地面接触（不悬空、不穿透）

---

## 4. 数值线性化：闭链系统的 A、B 矩阵

### 4.1 原理

考虑离散闭环映射（一步 `mj_step`）：

$$x_{k+1} = f(x_k, u_k)，x = (q, v)$$

在平衡点做泰勒展开，取一阶：

$$\delta x_{k+1} \approx A_d \cdot \delta x_k + B_d \cdot \delta u_k$$

其中 $A_d$、$B_d$ 是离散雅可比

用中心差分求 A_d、B_d：对每个状态分量扰动 $\pm\epsilon$，跑一步 `mj_step`，测状态增量变化率。**闭链约束在 `mj_step` 内部自动解算，所以差分得到的 A、B 天然包含闭链的约束动力学**——这是本方法对闭链系统最省力的地方。

### 4.2 代码骨架

```python
import numpy as np
import mujoco

def make_flow(model):
    """构造闭环一步映射 f(x, u) = (Δq, Δv)。"""
    d = mujoco.MjData(model)
    nq, nv = model.nq, model.nv

    def flow(x, u):
        d.qpos[:] = x[:nq]
        d.qvel[:] = x[nq:]
        d.ctrl[:] = u
        mujoco.mj_forward(model, d)
        mujoco.mj_step(model, d)
        return np.concatenate([d.qpos - x[:nq], d.qvel - x[nq:]])
    return flow

def linearize(model, x0, u0, eps=1e-6):
    """在 (x0, u0) 处离散线性化：x' ≈ A_d·δx + B_d·δu。"""
    nq, nv, nu = model.nq, model.nv, model.nu
    n = nq + nv
    flow = make_flow(model)
    f0 = flow(x0, u0)                       # 平衡点一步增量（应≈0）
    A = np.zeros((n, n)); B = np.zeros((n, nu))

    for i in range(n):
        e = np.zeros(n)
        if i < nq:
            # ⚠️ q 方向的扰动不能用"加减"，freejoint 的四元数必须用
            # mj_integratePos 积分施加（否则差分出的 quat 列全是垃圾）
            qp = mujoco.mj_integratePos(model, x0[:nq], eps * e_q(i, nq))
            e[:nq] = qp - x0[:nq]
        else:
            e[i] = eps
        A[:, i] = (flow(x0 + e, u0) - flow(x0 - e, u0)) / (2 * eps)

    for j in range(nu):
        e = np.zeros(nu); e[j] = eps
        B[:, j] = (flow(x0, u0 + e) - flow(x0, u0 - e)) / (2 * eps)
    return A, B
```

说明：

- `e_q(i, nq)` 是第 i 个广义坐标方向的单位向量（对 freejoint 的旋转分量，`mj_integratePos` 内部处理四元数积分）。
- 对 21+20=41 维状态、6 维输入，一次线性化需要约 (41+6)×2 ≈ 94 次 `mj_step`，耗时 < 1 秒，完全可接受。
- **验证线性化**：随机取 $\delta x$、$\delta u$，比较 `flow(x0+δx, u0+δu)` 与 $A_d \cdot \delta x + B_d \cdot \delta u$，残差应 $\sim\epsilon^2$ 量级（中心差分的高阶项）。

### 4.3 闭链扰动方向的坑

q 方向扰动会暂时把位形推出闭合流形，connect 约束一步内会把它拉回来——**只要扰动幅度 $\epsilon$ 够小（1e-6），约束拉回量在差分里被自动处理**，A 矩阵仍然是闭合流形的切线线性化。但平衡点本身的 site 残差必须 < 0.5mm，否则差分起点就不在流形上。

---

## 5. LQR：状态反馈增益

### 5.1 原理

连续时间 LQR：最小化 $\int(x^{\mathsf{T}}Qx + u^{\mathsf{T}}Ru)dt$，最优反馈 $u = -K(x - x_0)$，其中 $K$ 来自 Riccati 方程：

$$A^{\mathsf{T}}P + PA - PBR^{-1}B^{\mathsf{T}}P + Q = 0 \rightarrow K = R^{-1}B^{\mathsf{T}}P$$

我们手里是离散 A_d、B_d（一步映射），用**离散 Riccati**更贴合：

```python
import scipy.linalg

Q = np.diag([...])            # 状态权重，见 5.3
R = np.diag([...])            # 输入权重
P = scipy.linalg.solve_discrete_are(A, B, Q, R)
K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)   # 离散 LQR 增益
```

> 用离散版而非连续版的原因：`mj_step` 是隐式积分器，一步映射本身带数值阻尼，A_d 与"连续 A 乘 dt"有系统性差异。离散 Riccati 直接对 A_d 求解，无需换算。

### 5.2 可控性检查（必须先做）

欠驱动系统不是随便给 Q、R 就能稳的：

```python
ctrb = np.hstack([B] + [A @ (np.linalg.matrix_power(A, k) @ B) for k in range(1, n)])
print("可控性秩:", np.linalg.matrix_rank(ctrb), "/", n)   # 满秩才可控
```

如果不满秩：检查状态里是否缺少"车体水平位移"对应的速度状态（平衡车的经典可控状态是**车体俯仰角、俯仰角速度、轮子速度、车体水平速度**；纯"轮子角度"状态不可控——轮子转多少圈跟平衡无关）。

### 5.3 Q、R 初值（物理含义）

| 权重 | 建议初值 | 物理含义 |
|---|---|---|
| Q[俯仰角] | 1000 | 平衡是首要目标，权重最大 |
| Q[俯仰角速度] | 50 | 抑制振荡 |
| Q[轮速] | 1 | 允许轮子为保持平衡而滚动 |
| Q[车体水平速度] | 0.1 | 弱约束，让车自由移动 |
| R[轮子力矩] | 0.1~1 | 轮子是主要执行器 |
| R[腿关节] | 1~10 | 腿关节力矩成本 |

初值原则：**俯仰角权重至少比轮速大两个数量级**，否则 LQR 的解会"把车跑起来"而不是"平衡住"。

---

## 6. 闭环仿真与验证

```python
def control_law(model, data, K, x0, theta_idx):
    """LQR 状态反馈。"""
    x = np.concatenate([data.qpos, data.qvel])
    # ⚠️ 车体俯仰角从 freejoint 四元数提取（绕 y 轴），不是 data.qpos 里现成的分量
    pitch = extract_pitch(model, data)
    err = np.array([pitch - x0_pitch, pitch_dot, wheel_speed, vx, ...])
    u = -K @ err
    data.ctrl[WHEEL_ACT_ID] = u[0]      # 轮子力矩
    data.ctrl[LEG_ACT_ID]   = u[1:]     # 腿关节力矩（VMC 融合前可先给 0）
```

**测试协议（每步改动都必须跑）**：

1. **零扰动测试**：从 $x_0$ 起步，LQR 闭环跑 5 秒 → 车体不动、无 NaN。
2. **初始倾斜测试**：把车体绕 y 轴倾斜 0.1 rad 起步（`mj_forward` 前改 qpos 四元数）→ 2 秒内回正。
3. **外力扰动测试**：`data.xfrc_applied[base_body_id, :] = [Fx,0,0,0,0,0]` 持续 0.5 秒推车体 → 推力撤掉后自恢复。

> `data.xfrc_applied` 每步都要清零（教程 7.4 的教训）。

---

## 7. VMC：虚拟模型控制

### 7.1 原理

不直接控制物理关节，而是在**虚拟作用点**（车体质心、腿端 site）挂虚拟弹簧-阻尼元件，产生虚拟力 F_virtual，然后通过雅可比映射成关节力矩：

$$\tau = J^{\mathsf{T}} \cdot F_{virtual}$$

闭链内力、接触力、重力全部由 MuJoCo 在 `mj_step` 内自动解算——**你只需要把虚拟力施加进去**。

### 7.2 经典轮腿车 VMC 设计（以本模型为例）

在车体质心定义三个虚拟元件：

| 虚拟元件 | 作用点 | 输出 | 用途 |
|---|---|---|---|
| 俯仰虚拟阻尼 | 车体质心 | 绕 y 轴的虚拟力矩 | 配合 LQR 增强俯仰阻尼 |
| 高度虚拟弹簧 | 车体质心 | z 向虚拟力 | 控制质心高度（腿长） |
| 前进虚拟弹簧 | 车体质心 | x 向虚拟力 | 位置/速度跟踪（外环） |

### 7.3 代码骨架

```python
def vmc_force(model, data, target_z, kz, cz, kp, cd):
    """返回施加在车体质心的 6D 虚拟力。"""
    # 质心状态：用 mj_forward 后 data 里的 body 位姿 + 速度
    z, zd = data.xpos[COM_BODY][2], data.cvel[COM_BODY][5]   # 高度与 z 向速度
    pitch, pitch_dot = extract_pitch(model, data), ...

    F = np.zeros(6)
    F[0] = kx*(x_des - x) - cx*vx            # 前进虚拟弹簧（可先置 0）
    F[2] = kz*(target_z - z) - cz*zd         # 高度虚拟弹簧
    F[4] = -kp*pitch - cd*pitch_dot          # 俯仰虚拟阻尼
    return F

def apply_vmc(model, data, F, jac_body_id):
    """τ = JᵀF 映射到关节广义力。"""
    jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, jac_body_id)   # 教程 5 章
    data.qfrc_applied[:] = jacp.T @ F[:3] + jacr.T @ F[3:]
```

**闭链机构下的关键理解**：`qfrc_applied` 施加的是广义力，MuJoCo 会把虚拟力对闭链约束的影响一并解算——前后腿通过 connect 约束自动分担虚拟力，不需要手工做力分配。

### 7.4 VMC 的腿长控制（闭链特有）

要控制质心高度，更直接的做法是控制"腿的有效长度"。对串联腿闭链：

- 在腿端（如 `Left_rear_site2` 附近的轮轴 site）定义虚拟垂直弹簧，虚拟力向上/向下推
- 腿关节力矩 = $J^{\mathsf{T}}F$ 自动分配：前腿与后腿按闭链几何比例分担

这是 VMC 相对其他方法在闭链机构上的核心优势——**不需要知道五杆机构的解析动力学，几何（雅可比）就够**。

---

## 8. LQR + VMC 融合

分层叠加，各司其职：

```
每步循环：
  1. 读状态（俯仰角、轮速、质心高度...）
  2. u_lqr = -K·(x - x₀)                  # 内环：平衡
  3. F_virtual = vmc_force(...)           # 外环：姿态/腿形
     qfrc_applied = Jᵀ·F_virtual          # 虚拟力进动力学
  4. data.ctrl[WHEEL] = u_lqr[轮子分量]   # 轮子：LQR 直接力矩
     data.ctrl[LEG]  = u_lqr[腿分量] + τ_vmc_mapped  # 腿：LQR + VMC 叠加
  5. mujoco.mj_step(model, data)
```

**为什么要分层**：LQR 是平衡位形附近的线性最优解，管"小偏差快速回正"；VMC 是几何层的外环任务（高度、姿态、避碰），管"位形保持"。两者频率可以不同（VMC 每 2~5 步算一次也行），叠加不冲突。

**注意**：腿关节同时接收 LQR 和 VMC 的力矩时，K 矩阵的腿关节列在 VMC 接入后需要重新标定（VMC 会改变闭环静态增益）——调试顺序上**先 LQR 纯轮子平衡，再逐层加 VMC**，每层都要重跑第 6 节的测试协议。

---

## 9. 调参指南

| 现象 | 调整 |
|---|---|
| 车体持续缓慢倾倒 | 增大 Q[俯仰]，或增大 R[轮子] 让轮子更积极 |
| 高频振荡/抖动 | 增大 Q[俯仰角速度]，增大 KD（VMC 阻尼） |
| 轮子疯狂滚动、车体不倒 | 减小 R[轮子]、增大 Q[轮速] |
| 收敛太慢（>3s 才回正） | 增大 Q[俯仰]，减小 R[轮子] |
| 初始倾斜 0.1 rad 就倒 | 检查可控性秩；检查线性化残差；检查 $x_0$ 是否在闭合流形上 |
| VMC 接入后变差 | 先单独测 VMC（无 LQR），再组合；VMC 增益从 1/10 起步 |

**调参流程**：Q/R 矩阵从 5.3 的初值出发，每次只改一个权重，跑第 6 节测试协议对比。

---

## 10. 验收标准与实施顺序

| 步骤 | 内容 | 验收 |
|---|---|---|
| 1 | base 加 freejoint + 惯性 + keyframe | headless 5s 数值稳定，车体倒下接触地面 |
| 2 | 闭合位形重新收敛，记录 $x_0$ | site 残差 < 0.5mm |
| 3 | 数值线性化脚本 | A、B 维度正确；线性化残差 $\sim\epsilon^2$；可控性满秩 |
| 4 | LQR 闭环（只控轮子，腿零力矩） | 俯仰扰动 0.1 rad，2s 内回正 |
| 5 | VMC 层（先只有俯仰虚拟阻尼） | 外力 10N 推车体，撤力后自恢复 |
| 6 | VMC 全量（高度+前进） + LQR 融合 | 三项测试协议全过，无 NaN |

每步单独 commit（可回退），**不要跳步**——步骤 4 的 LQR 是最难的一关（欠驱动 + 闭链），在此之前的一切工作都是为它服务。

---

## 11. 常见坑清单

1. **freejoint 必须有 inertial**：static body 加 freejoint 后 `qacc` 直接 NaN，这是最容易踩的坑。
2. **四元数不能直接加减**：q 方向扰动用 `mj_integratePos`；提取俯仰角用 `mj_rotMat2Euler` 或从四元数算，不要假设 `qpos[5]` 就是俯仰。
3. **扰动破坏闭链**：$\epsilon$ 取 1e-6，平衡点残差先清零，差分结果才可信。
4. **轮子角度不是好状态**：平衡车 LQR 用"轮速"而非"轮角"（轮子转多少圈与平衡无关，还不可控）。
5. **离散 vs 连续 Riccati**：必须用 `solve_discrete_are` 匹配离散线性化结果。
6. **xfrc_applied 每步清零**（教程 7.4）。
7. **VMC 与 LQR 的腿关节增益冲突**：先纯 LQR 调通，再叠加 VMC，每层重跑测试协议。
8. **整机质量分配**：base 质量影响闭环极点，改质量后必须重新线性化（A、B 全变）。

---

## 下一步

教程验证通过后，把各节代码骨架合并为 `balance_sim/lqr_balance.py`（`--headless` 测试模式 + viewer 可视化模式，与 `simulate_balance.py` 风格一致），即完成本平衡控制 demo。
