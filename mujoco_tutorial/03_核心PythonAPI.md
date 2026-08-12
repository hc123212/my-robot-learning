# 第 3 章 核心 Python API：MjModel / MjData / 仿真循环

> 对应学习计划：全部阶段（第 1 阶段起每个实验都用的基础）
> 预计用时：2 小时
> 目标：掌握仿真循环的标准写法、读写状态、命名访问、numpy 内存语义

---

## 3.1 MjModel：静态模型

`MjModel` 是编译后的机器人模型——所有**结构性和参数性**信息都在里面：刚体树、关节、几何、执行器、传感器、惯性参数、默认物理参数。

```python
import mujoco

# 三种加载方式
model = mujoco.MjModel.from_xml_path("pendulum.xml")     # 从文件（最常用）
model = mujoco.MjModel.from_xml_string(xml_string)       # 从字符串（快速测试）
model = mujoco.MjModel.from_zip(...)                     # 从打包 zip

# 常用模型级信息
model.nq      # 广义坐标数（qpos 的长度）
model.nv      # 广义速度数（qvel 的长度，注意可能不等于 nq！free/ball 关节 qpos 比 qvel 长）
model.nu      # 控制输入数（ctrl 的长度，= actuator 个数）
model.nbody   # 刚体数（含 worldbody）
model.njnt    # 关节数
model.ngeom   # 几何体数
model.nsite   # site 数
model.nsensor # 传感器数
model.opt.timestep  # 积分步长
model.opt.gravity   # 重力
```

**为什么 nq ≠ nv？** free 关节用 4 元数表示姿态（4 个数）但角速度只有 3 个分量（3 个数），ball 关节同理。所以 `nq ≥ nv`。写代码时用 `model.nv` 而不是 `model.nq` 分配雅可比等矩阵——这是阶段 4 实验的常见 bug 来源。

## 3.2 MjData：动态状态

```python
data = mujoco.MjData(model)
```

`MjData` 是每一步都在变的运行时状态。**你学习计划全部实验的数据读取，核心就是下面这些字段**：

| 字段 | 形状 | 含义 |
|---|---|---|
| `data.qpos` | (nq,) | 广义坐标（关节角/位置/姿态） |
| `data.qvel` | (nv,) | 广义速度（关节角速度等） |
| `data.qacc` | (nv,) | 广义加速度（**mj_step 后有效**，阶段 5 逆动力学用） |
| `data.ctrl` | (nu,) | 控制输入（执行器目标/力矩，**每步仿真前设置**） |
| `data.time` | 标量 | 仿真时间（s） |
| `data.xpos` | (nbody,3) | 每个刚体的世界坐标位置 |
| `data.xquat` | (nbody,4) | 每个刚体的世界坐标姿态（四元数 wxyz） |
| `data.sensordata` | (nsensordata,) | 所有传感器读数拼接（第 8 章） |
| `data.qfrc_inverse` | (nv,) | **mj_inverse 的输出**：逆动力学力矩（阶段 5） |
| `data.qfrc_actuator` | (nv,) | 执行器产生的广义力 |
| `data.qfrc_applied` | (nv,) | 用户施加的广义力（注意：外力 xfrc_applied 的映射不写这里，见第 7 章） |
| `data.qfrc_bias` | (nv,) | 科氏力+离心力+重力项（阶段 5 动力学分析用） |
| `data.ncon` | 标量 | 当前接触对数量 |
| `data.contact` | (ncon,) | 接触信息数组（第 8 章） |

**索引语义**：这些数组是 `numpy.ndarray` 的**内存视图**——直接指向 MuJoCo 内部 C 内存。这意味着：

```python
data.qpos[0] = 0.5          # 直接改 C 内存里的位置
data.qpos[:] = [0.5, 0.2]   # 整段赋值
data.qpos += 0.1            # 原地运算，可以
# data.qpos = np.array([...])  # ❌ 不能整体替换！会破坏视图绑定，报错
```

这条规则（"视图可改不可换"）是 MuJoCo Python 绑定最重要的内存语义，所有 `qpos/qvel/ctrl/xfrc_applied/sensordata` 都遵守。

## 3.3 仿真循环的两种标准模式

### 模式 A：固定步数仿真（跑数据用，无窗口）

```python
model = mujoco.MjModel.from_xml_path("pendulum.xml")
data = mujoco.MjData(model)

data.ctrl[:] = ...          # 1. 每步前设置控制输入（没有执行器可跳过）
total_time = 5.0
while data.time < total_time:
    mujoco.mj_step(model, data)   # 2. 推进一步
    # 3. 在这里记录数据：qpos、末端位置、传感器……
    record.append((data.time, data.qpos.copy(), data.site("pendulum_end").xpos.copy()))
```

### 模式 B：实时可视化仿真（带窗口，第 4 章细讲）

```python
import mujoco.viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()       # 把最新状态发给渲染线程
```

两种模式的本质区别：模式 A 的 `data.time` 与真实时间无关（仿多快跑多快）；模式 B 配合 `time.sleep` 让仿真速度贴近真实（"实时"）。

### mj_step 内部发生了什么（阶段 5 理解逆动力学的钥匙）

```
mj_step(model, data)  ===  mj_step1(model, data) + mj_step2(model, data)
mj_step1: 前向动力学 + 积分
  ├─ mj_forward:  运动学 → 惯量 → 外力/执行器力 → 约束求解 → 加速度
  └─ 用加速度更新 qvel、用 qvel 更新 qpos（积分）
mj_step2: 更新传感器、能耗、统计等
```

等价地，`mj_forward` 只做前向计算**不积分**（状态不变）。调试时常用 `mj_forward`：设置好 qpos/qvel 后调它，立刻得到该位形的惯性矩阵、雅可比、接触力——阶段 4、5 实验大量使用"设置状态 → mj_forward → 读数据"这个模式。

## 3.4 命名访问：按名字读数据（3.x 起，强烈推荐）

早期 MuJoCo 教程会让你数索引：`qpos[0]` 是肩关节……模型一复杂就乱。3.x 的命名访问器按名字拿对象，自动处理索引：

```python
# 关节（动态量在 data 上）
data.joint("pivot").qpos          # 该关节的 qpos 分量（视图，可直接赋值）
data.joint("pivot").qvel          # 角速度
mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, 0)  # 索引反查名字（不太常用）

# 刚体（动态量在 data 上）
data.body("pendulum").xpos        # 世界坐标位置 (3,)
data.body("pendulum").xquat       # 世界坐标姿态
data.body("pendulum").cvel        # 空间速度 (6,)

# 几何 / site（静态量在 model 上，动态量在 data 上）
model.geom("rod").rgba            # 颜色等静态属性
data.geom("rod").xpos             # 世界坐标位置
data.site("pendulum_end").xpos    # 末端位置 ← 阶段 1~4 的核心读数

# 执行器
data.actuator("tau1").ctrl        # 该执行器的控制输入（视图）
model.actuator("tau1").gear       # 静态参数

# 传感器 / keyframe
model.sensor("jpos").id
model.keyframe("ready").id
```

**data 和 model 同名访问器的区别**（高频易错点）：

| 表达式 | 拿到什么 |
|---|---|
| `m.site("tip").pos` | **静态**：site 在所属 body 里的局部坐标（模型文件里写的那个值） |
| `d.site("tip").xpos` | **动态**：site 的世界坐标当前位置（`x` 前缀 = world frame） |

规则：**model 上读静态参数（局部），data 上读动态状态（世界）**。把 `x` 前缀理解为 "eXtrinsic/世界坐标"。

## 3.5 初始状态设置（阶段 3 起每个实验都用的基本功）

```python
# 方式 1：直接写 qpos / qvel（最灵活）
data.qpos[:] = [0.3, -0.5]          # 设置关节角
data.qvel[:] = 0.0                  # 清零速度
mujoco.mj_forward(model, data)      # 重新计算运动学，让 xpos 等字段同步

# 方式 2：keyframe（第 2 章 2.8 节定义过）
mujoco.mj_resetDataKeyframe(model, data, model.keyframe("ready").id)

# 方式 3：完全复位（回模型默认位形）
mujoco.mj_resetData(model, data)
```

**重要**：直接改 `qpos` 后，`data.xpos` 等派生量**不会自动更新**（它们是上次 forward/step 的残留值）。要么调用 `mj_forward`（不积分，只重算），要么直接 `mj_step`（会先内部做 forward）。**"改状态 → mj_forward → 读数据"是 MuJoCo 调试的标准三步**——忘记 forward 直接读 xpos，会读到上一时刻的值，阶段 2 的 FK 对比实验就栽在这。

## 3.6 完整示例：随机位形下读取 FK（阶段 2 实验的 API 预演）

```python
"""验证"改状态 → forward → 读世界坐标"的完整流程。"""
import mujoco
import numpy as np

xml = """
<mujoco model="arm2">
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom name="l1" type="capsule" size="0.03" fromto="0 0 0 0.4 0 0"/>
      <body name="link2" pos="0.4 0 0">
        <joint name="j2" type="hinge" axis="0 0 1"/>
        <geom name="l2" type="capsule" size="0.03" fromto="0 0 0 0.4 0 0"/>
        <site name="tip" pos="0.4 0 0"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

rng = np.random.default_rng(0)
for i in range(3):
    # 1. 随机关节角（两连杆都在 xy 平面，轴是 z）
    data.qpos[:] = rng.uniform(-1.5, 1.5, size=model.nq)
    # 2. 必须 forward，xpos 才会同步
    mujoco.mj_forward(model, data)
    # 3. 读末端世界坐标
    tip = data.site("tip").xpos.copy()
    # 4. 手算验证：q1=0, q2=0 时末端应在 (0.8, 0, 0)
    print(f"q = {data.qpos.round(3)}, 末端 = {tip.round(4)}")
```

（输出中的末端位置就是阶段 2 你手写 DH-FK 要对比的"真值"。）

## 3.7 数据记录与保存（所有阶段实验的收尾技能）

```python
import numpy as np

t_list, q_list, tip_list = [], [], []
while data.time < 5.0:
    mujoco.mj_step(model, data)
    t_list.append(data.time)
    q_list.append(data.joint("j1").qpos.copy())      # 注意 .copy()！
    tip_list.append(data.site("tip").xpos.copy())

t = np.array(t_list); Q = np.array(q_list); TIP = np.array(tip_list)

# 保存为 .npz（下次直接 np.load 读回来画图）
np.savez("sim_data.npz", t=t, q=Q, tip=TIP)
```

**为什么必须 `.copy()`**：前面说过 qpos 是内存视图。如果直接 `q_list.append(data.joint("j1").qpos)`，下次 mj_step 数据更新，**list 里所有"已记录"的值也会跟着变**——你会得到一列相同的数。这是阶段 2~5 实验数据对比对不上的头号原因。规则：**记录数据一律 copy，只读不改。**

## 3.8 其他常用 API（按需查阅）

```python
mujoco.mj_step1(model, data); mujoco.mj_step2(model, data)  # 分开调（调试用）
mujoco.mj_forward(model, data)    # 只计算不积分
mujoco.mj_inverse(model, data)    # 逆动力学（第 6 章）
mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "j1")  # 名字→id
mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, 2)      # id→名字
mujoco.mj_resetData(model, data)  # 复位到默认位形
mujoco.mj_saveModel(model, "out.mjb")  # 模型存为二进制（MJCF 预编译版）
```

`mujoco.rollout` 模块（3.3+）：多线程批量仿真，一次跑几千条轨迹（RL 和批量实验用，第 10 章提及）。

## 本章练习

1. 加载第 2 章的单摆模型，把摆角设成 0.5 rad，`mj_forward` 后打印 `data.site("pendulum_end").xpos`，手算验证：摆长 0.5m 时末端应在 $(\sin 0.5, 0, 1-\cos 0.5)$。
2. 验证 `nq vs nv`：给单摆的底座加个 free 关节，重新打印 nq/nv，体会 4 元数多出的 1 个坐标。
3. 故意去掉 `.copy()` 记录 qpos，跑完后打印 list 里前 3 个值——亲眼看看它们全部变成最后一帧的值（这个 bug 值得你亲手踩一次）。

> 进入下一章前，你应该能回答：改 qpos 后为什么必须 mj_forward？nq 和 nv 什么时候不相等？为什么记录数据要 .copy()？
