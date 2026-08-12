# 第 2 章 MJCF 建模语言：用 XML 描述机器人

> 对应学习计划：第 1 阶段（单摆建模）、第 2 阶段（2 连杆平面臂建模）
> 预计用时：1.5~2 小时
> 目标：能独立写出单摆、平面机械臂这类模型的完整 XML，知道每个元素的含义

---

## 2.1 概览：MJCF 是什么

MJCF（MuJoCo Configuration Format）是 MuJoCo 的原生模型格式，一个 XML 文本文件。机器人学建模的核心思路：

```
<body>（刚体，构成树）  +  <joint>（关节，允许运动）  +  <geom>（几何，负责碰撞和显示）
```

三要素构成一个可动的刚体链。再加上 `<actuator>`（动力）、`<sensor>`（测量）、`<site>`（标记点），就是一个完整的仿真模型。

**建模铁律（刚体树规则）**：
1. 所有刚体构成**一棵树**，根是世界（worldbody）。
2. 每个 body 可以有一个或多个 joint（决定它相对父体的运动自由度），没有 joint 的 body 就是"焊死"的。
3. body 的坐标（pos）是**相对父体的**（局部坐标），层层嵌套。

> 阶段 1 任务书里的"一个 base + 一个带旋转关节的杆"翻译成 MuJoCo 语言就是：`worldbody` 下放一个 base 杆（body），base 下放一个带 hinge 关节的摆杆（body），摆杆上放一个 geom 做碰撞/显示。

## 2.2 骨架：根元素、compiler、option

```xml
<mujoco model="pendulum">
  <!-- compiler：编译选项。angle="radian" 让所有角度用弧度！MuJoCo 默认是度 -->
  <compiler angle="radian"/>

  <!-- option：物理参数。timestep 是积分步长（秒），越小越精确越慢 -->
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <!-- 所有 body 都挂在这里（或挂在其他 body 下） -->
  </worldbody>
</mujoco>
```

**最容易踩的坑**：MuJoCo 所有角度属性（`euler`、joint 的 `range`、`position` 执行器的 `ctrlrange` 等）**默认单位是度**。`<compiler angle="radian"/>` 一行全改弧度。你的学习计划全程用弧度（Modern Robotics 符号），必须加这行。

常用 option 字段（阶段 6 控制实验会用到）：

```xml
<option
  timestep="0.002"        <!-- 积分步长 s，默认 0.002 -->
  gravity="0 0 -9.81"     <!-- 重力，默认 -9.81 -->
  integrator="Euler"      <!-- 积分器：Euler / RK4 / implicit / implicitfast，默认 Euler -->
  solver="Newton"         <!-- 约束求解器：PGS / CG / Newton，默认 Newton -->
/>
```

> 这些先混个脸熟即可。做动力学实验时，`integrator` 会影响数据精度（`implicit` 更稳），RK4 精度高但慢，到时候回来查。

## 2.3 body：刚体

```xml
<body name="link1" pos="0.4 0 0" quat="1 0 0 0">
  <joint .../>
  <geom .../>
  <body name="link2" ...>   <!-- 子刚体，坐标相对 link1 -->
  </body>
</body>
```

| 属性 | 含义 |
|---|---|
| `name` | 唯一名字（强烈建议全部命名，Python 端用名字访问） |
| `pos` | 相对父体的位置（m） |
| `quat` | 相对父体的姿态四元数 `[w x y z]`（或 `euler`，三个欧拉角） |
| `pos` 缺省 | 默认与父体重合 |

**注意**：body 自身没有质量属性——质量来自它下面的 geom（自动按体积×密度计算），或用 geom 的 `mass` 显式指定。

## 2.4 joint：关节（运动自由度）

```xml
<joint name="shoulder" type="hinge" axis="0 1 0" pos="0 0 0"
       limited="true" range="-2.0 2.0"
       damping="0.1" stiffness="0.0" armature="0.01"/>
```

**type 五种**（你学习计划用到前三种）：

| type | 自由度 | qpos 含义 | 典型用途 |
|---|---|---|---|
| `hinge` | 1 | 转角（标量） | 旋转关节——单摆、机械臂关节 |
| `slide` | 1 | 位移（标量） | 平移关节——滑块、导轨 |
| `free` | 6 | `[x y z qw qx qy qz]` | 自由漂浮——轮式机器人底座、被抛物体 |
| `ball` | 3 | 四元数前 4 位 `[qw qx qy qz]` | 球铰（3 自由度旋转） |
| `pin` | 1 | 转角 | 类似 hinge，但允许围绕任意轴并可沿轴滑（少用） |

常用属性：

| 属性 | 含义 |
|---|---|
| `axis` | 旋转轴（世界系/父体系内），hinge 必填 |
| `pos` | 关节在父体局部坐标系的位置 |
| `limited` + `range` | 关节限位。**hinge 默认不限位**，机械臂都要限 |
| `damping` | 关节阻尼（与速度成正比的阻力矩） |
| `stiffness` | 关节弹簧（位置比例力，配合 ref 用） |
| `armature` | 关节转子惯量——电机转动惯量折算，让动力学更真实 |

**关键认知**：joint 决定了 MuJoCo 内部广义坐标 qpos 的布局。机器人的 `data.qpos` 就是所有关节 qpos 按模型顺序拼成的向量。阶段 3 的 IK 实验把解算出的关节角写进 `data.qpos`，读末端位置，就是靠这套布局。

**命名访问**：关节的 qpos 值可以通过名字拿，不用记索引（第 3 章细讲）：

```python
q = data.joint("shoulder").qpos   # 标量数组，尺寸 = 该关节自由度
```

## 2.5 geom：几何体（碰撞 + 视觉）

```xml
<!-- 显示一个盒子和一根杆子 -->
<geom name="box1" type="box" size="0.1 0.1 0.1" pos="0 0 0.5" rgba="0.8 0.2 0.2 1"/>
<geom name="rod1" type="capsule" size="0.03" fromto="0 0 0 0.4 0 0"/>
```

**常用 type**：

| type | size 参数 | 说明 |
|---|---|---|
| `sphere` | `size="半径"` | 球 |
| `box` | `size="半长 半宽 半高"` | 立方体（size 是**半**尺寸！） |
| `capsule` | `size="半径"` + `fromto` 或 pos/euler | 胶囊 = 杆+半球端，**建模连杆首选** |
| `cylinder` | `size="半径 半高"` | 圆柱 |
| `ellipsoid` | `size="三半轴"` | 椭球 |
| `plane` | `size="半长 半宽 厚度"` | 无限平面（地面） |
| `mesh` | `size="缩放"` + `<mesh file=...>` | 三角网格（Menagerie 模型常用） |

**定位方式**：`pos`+`euler`（相对所属 body），或胶囊/圆柱用 `fromto="起点 终点"` 直接指定两端点（写机械臂连杆特别方便——直接"从上一个关节画到下一个关节"）。

**惯量与质量**（阶段 5 动力学实验的关键）：

```xml
<geom type="capsule" size="0.03" fromto="0 0 0 0.4 0 0"
      density="1000"      <!-- 默认密度 1000 kg/m³，质量=体积×密度 -->
      mass="1.5"          <!-- 或直接指定质量，与 density 二选一 -->
/>
```

默认情况下质量**由几何体体积×density 自动计算**。阶段 5 你要手算 2 连杆的质量矩阵 $M(q)$ 和 MuJoCo 对比，**必须搞清楚每个 geom 的质量和质心**，建议建模时显式写 `mass` 或用简单的 capsule 并自己心算验证。

**碰撞相关属性**（阶段 6 之后的行走、抓取才需要，先了解）：

| 属性 | 含义 |
|---|---|
| `contype` / `conaffinity` | 碰撞组（0=永不碰撞）。默认都是 1，全部互相碰撞 |
| `condim` | 接触维度：1=纯摩擦点接触，3=摩擦点接触，4=+摩擦锥，6=完整 |
| `friction` | `"滑动 扭转 滚动"` 三个摩擦系数 |

**阶段 2 建模的常见困惑**：两个 body 之间的 geom 会互相碰撞——如果你的"2 连杆臂"建模后关节处自己抖起来，多半是两个 link 的胶囊互相碰撞了。解决办法：给不相邻的 geom 设 `contype="0"`（不参与碰撞，保留显示）。

## 2.6 site：零质量标记点（末端工具坐标系）

```xml
<site name="tip" pos="0.4 0 0" size="0.02" rgba="0 1 0 1"/>
```

site 是**没有质量、不碰撞**的坐标标记点，挂在某个 body 下。用途：

1. **雅可比**：`mj_jacSite` 返回 site 的雅可比——阶段 4 用 site 当"末端"。
2. **传感器**：`framepos` 传感器挂载点——阶段 8 反馈末端位置。
3. **外力施加点**：`mj_applyFT` 的力作用点——阶段 4 力域验证。
4. **可视化**：显示一个绿色小球，标注末端在哪。

**习惯**：凡是"末端"（夹爪、笔尖、球拍），都建一个 site。阶段 2~4 实验全靠它。

## 2.7 actuator：执行器（动力来源）

```xml
<actuator>
  <!-- motor：直接输出力矩/力，ctrl 就是力矩。阶段 6 手写 PD 用它 -->
  <motor name="tau1" joint="shoulder" gear="1.0"/>

  <!-- position：内置 PD 位置控制器，ctrl 是目标角度。阶段 7 轨迹跟踪用它 -->
  <position name="pos1" joint="shoulder" kp="100" kd="10" ctrlrange="-3.14 3.14"/>

  <!-- velocity：速度控制，ctrl 是目标角速度 -->
  <velocity name="vel1" joint="elbow" kv="10" ctrlrange="-5 5"/>
</actuator>
```

执行器的本质：**把控制输入（data.ctrl 的某个分量）转换成一个广义力**。转换规则由类型决定：

| 类型 | ctrl 含义 | 输出 |
|---|---|---|
| `motor` | 力矩/力 | $\tau = ctrl \times gear$ |
| `position` | 目标位置 | $\tau = k_p\times(ctrl-q) - k_d\times\dot{q}$（内置 PD） |
| `velocity` | 目标速度 | $\tau = k_v\times(ctrl-\dot{q})$ |

三个执行器对应你学习计划的三类实验：
- 阶段 6 手写 PD / 计算力矩 → `motor`，自己在 Python 里算 $\tau$ 写 `data.ctrl`
- 阶段 7 轨迹跟踪 → `position` 直接给目标角，或 `motor`+手写 PD
- 阻抗控制 → `motor`（要自己算 $J^{\mathsf{T}}F$ 形式的力），或 `general` 执行器（进阶）

> 第 7 章会详细对比"内置 position 执行器"和"手写 PD"的差异——这是阶段 6 的重要理解点。

## 2.8 keyframe：关键帧（初始位形）

```xml
<keyframe>
  <key name="home" qpos="0 0 0.5"/>     <!-- 注意：name 写在 <key> 上，不是 <keyframe> -->
  <key name="ready" qpos="0.5 -1.0"/>
</keyframe>
```

Python 端一键恢复初始位形：

```python
mujoco.mj_resetDataKeyframe(model, data, model.keyframe("ready").id)
# 或按序号: mujoco.mj_resetDataKeyframe(model, data, 1)
```

用途：阶段 3~6 反复从同一姿态开始实验，把常用初始位形存成 keyframe，不用手写 qpos。

> 小坑（实测）：网上旧资料有 `<keyframe name="...">` 写法，3.11 会报 Schema 错误。`name` 必须在 `<key>` 子元素上。

## 2.9 完整示例：单摆（阶段 1 的建模雏形）

把下面存成 `pendulum.xml`，用 `python -m mujoco.viewer --mjcf=pendulum.xml` 打开：

```xml
<mujoco model="pendulum">
  <compiler angle="radian"/>

  <option timestep="0.002"/>

  <worldbody>
    <!-- 固定底座（没有 joint 的 body，纯显示） -->
    <body name="base" pos="0 0 1.0">
      <geom type="box" size="0.1 0.1 0.1" rgba="0.5 0.5 0.5 1"/>
    </body>

    <!-- 摆杆：挂一个 hinge 关节，绕 y 轴摆动 -->
    <body name="pendulum" pos="0 0 1.0">
      <joint name="pivot" type="hinge" axis="0 1 0" pos="0 0 0"
             damping="0.02" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.5"
            mass="1.0" rgba="0.2 0.6 1.0 1"/>
      <site name="pendulum_end" pos="0 0 -0.5"/>
    </body>
  </worldbody>
</mujoco>
```

结构拆解（对照刚体树规则）：

```
worldbody（世界）
├── body "base"          —— 固定底座（无 joint，焊死）
└── body "pendulum"      —— 摆杆（有 hinge 关节，能绕 y 轴转）
    ├── joint "pivot"    —— 旋转自由度
    ├── geom 胶囊        —— 从关节向下画 0.5m 的杆
    └── site             —— 摆端标记点
```

打开 Simulate 看：摆杆从垂直位置自由下落摆动。按空格暂停、CTRL 拖拽摆端，体会力的作用。

阶段 1 实验就在这个模型上做：**读 `data.joint("pivot").qpos`（摆角）→ 构造旋转矩阵 → 算摆端位置 → 与 `data.site("pendulum_end").xpos` 对比**。API 细节见第 3、5 章。

## 2.10 建模自查清单（每次写完 XML 都过一遍）

1. `<compiler angle="radian"/>` 加了吗？（没加的话所有角度都是度！）
2. 每个 body 都有唯一 name 吗？（没 name 在 Python 端没法用名字访问，只能数索引）
3. 需要运动的 body 都有 joint 吗？（没有 joint 的 body 永远焊死）
4. hinge 关节的 `axis` 写了吗？
5. 机械臂关节 `limited="true"` + `range` 了吗？（不限位仿真会乱转）
6. 末端位置建 `site` 了吗？
7. 质量合理吗？（默认密度 1000，胶囊体积 $\approx \pi r^2 L$，自己心算一下对不对）
8. 相邻连杆会互相碰撞吗？（不相邻的设 `contype="0"`）

## 本章练习

1. 给单摆加第二个 body：摆端再接一根杆（双摆）。观察混沌摆动——阶段 2 建模手感的预演。
2. 把摆杆从 capsule 换成 box，注意 `size` 是半尺寸，杆要放在 `pos="0 0 -0.25"` 才能让质心落在关节下方。
3. 在 Simulate 里按 F3/F11 观察：网格视图、碰撞几何显示。确认你的 geom 位置和想象一致。

> 进入下一章前，你应该能回答：hinge 和 free 关节的 qpos 各是什么？geom 的 size 是不是半尺寸？为什么 body 没有 joint 就动不了？
