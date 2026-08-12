"""MuJoCo 3.11 第一个脚本：加载内置模型并仿真 2 秒。"""
import mujoco
import numpy as np

# 1. 用 XML 字符串创建一个最简模型：一个在重力下自由下落的球
xml = """
<mujoco model="hello">
  <worldbody>
    <geom type="plane" size="2 2 0.1"/>          <!-- 地面 -->
    <body name="ball" pos="0 0 0.3">
      <joint name="free" type="free"/>            <!-- 6 自由度自由关节 -->
      <geom type="box" size="0.1 0.1 0.1" mass="1.0"/> <!-- 边长为 0.1m、质量为 1kg 的盒子 -->
    </body>
  </worldbody>
</mujoco>
"""

# 2. XML 字符串 -> MjModel（编译阶段，只做一次）
model = mujoco.MjModel.from_xml_string(xml)

# 3. MjModel -> MjData（运行状态，每步更新）
data = mujoco.MjData(model)

print(f"模型信息: {model.nq} 个广义坐标, {model.nbody} 个刚体, 时间步长 {model.opt.timestep}s")

# 4. 仿真循环：time 是仿真内部时间，单位秒
while data.time < 2.0:
    mujoco.mj_step(model, data)

# 5. 读取结果：球最终停在地面上（z = 半径 0.1）
print(f"仿真结束: t = {data.time:.2f}s")
print(f"球心位置: {data.body('ball').xpos}")   # 期望 z ≈ 0.1

# 6. 对照物理：自由落体 0.2m 用时 sqrt(2*0.2/9.81) ≈ 0.2s，此时已落地静止