"""MuJoCo 入门第一课：加载模型并实时可视化。

运行: source D:/pyenvs/robotics/Scripts/activate 后 python demo_mujoco.py
"""
import os
import sys
import gymnasium as gym
import mujoco
import mujoco.viewer

# Windows 控制台默认 GBK，打印中文会乱码，强制 UTF-8
sys.stdout.reconfigure(encoding="utf-8")

# 1. 加载 21 自由度人形机器人模型（gymnasium 自带的资产文件）
xml_path = os.path.join(
    os.path.dirname(gym.__file__), "envs", "mujoco", "assets", "humanoid.xml"
)
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print(f"模型加载成功：{model.nq} 个广义坐标，{model.nu} 个控制输入")
print("窗口操作：空格=暂停/继续，右键拖动=旋转视角，滚轮=缩放，CTRL+左键=平移")

# 2. 打开实时可视化窗口（阻塞运行，直到窗口关闭）
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 3. 仿真主循环：当前无控制输入，人形机器人在重力下会倒地
    #    （下一课会加 PD 控制让它站稳，这是动力学控制的核心练习）
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
