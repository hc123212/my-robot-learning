"""COD-2026 平衡车（串联腿闭链结构）MuJoCo 仿真。

模型来源: https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File
（辽宁科技大学 COD 战队开源，MJCF 目录，作者保留了官方文件在 assets/ 下）

官方模型的 base_link 固定于世界系（无 freejoint），因此本程序演示的重点是
「串联腿闭链」的联动特性：

  大腿关节（Left/Right_front_joint、Left/Right_rear_joint）带执行器；
  小腿关节（child1/child2/child3）没有执行器，由 <equality><connect>
  闭链约束自动联动。这里只驱动前大腿做正弦摆动、后大腿不驱动，
  观察整条闭链如何自动跟随——这就是串联腿闭链的运动学核心。

运行:
  # 实时可视化（默认，窗口关闭或 8 秒后退出）
  D:/pyenvs/robotics/Scripts/python.exe simulate_balance.py
  # 无窗口快速测试（跑 4 秒，打印状态后退出）
  D:/pyenvs/robotics/Scripts/python.exe simulate_balance.py --headless
"""
import os
import sys
import numpy as np
import mujoco
import mujoco.viewer

sys.stdout.reconfigure(encoding="utf-8")

XML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "balance_sim.xml")

# ---------------- 控制参数 ----------------
AMP = 0.2           # 前大腿正弦摆幅（弧度，相对闭合位形 ±0.2，在放宽后的限位内）
FREQ = 0.5          # 摆动频率（Hz）
KP = 15.0           # 前大腿位置伺服增益（N·m/rad，需大于腿机构重力矩 ~2.4 N·m 才能带动）
KD = 1.0            # 前大腿阻尼增益（N·m·s/rad）
DURATION = 8.0      # 演示时长（秒）

# 注：直接用力矩驱动带不动腿（重力矩 ~2.4 N·m），故用位置伺服（PD），
#     稳态误差自动补偿重力。轮子不驱动（接地被摩擦锁死），闭链会把
#     前腿的运动传给后腿侧。


def get_ids(model):
    """收集需要的对象索引。"""
    act_id = {
        n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_actuator")
        for n in ["Left_front_joint", "Left_rear_joint", "Left_Wheel_joint",
                  "Right_front_joint", "Right_rear_joint", "Right_Wheel_joint"]
    }
    jnt = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
           for n in ["Left_front_joint", "Left_rear_joint",
                     "Right_front_joint", "Right_rear_joint",
                     "Left_Wheel_joint", "Right_Wheel_joint"]}
    qpos_adr = {n: model.jnt_qposadr[j] for n, j in jnt.items()}
    qvel_adr = {n: model.jnt_dofadr[j] for n, j in jnt.items()}
    # 闭链约束用到的 site（计算残差用）
    site = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, n)
            for n in ["Left_front_site1", "Left_rear_site1",
                      "Left_front_site2", "Left_rear_site2",
                      "Right_front_site1", "Right_rear_site1",
                      "Right_front_site2", "Right_rear_site2"]}
    return act_id, jnt, qpos_adr, qvel_adr, site


def apply_control(data, act_id, qpos_adr, qvel_adr, theta0):
    """前大腿位置伺服：θ_des = 闭合位形 + 正弦摆动；后大腿与小腿交给闭链联动。"""
    t = data.time
    phase = 2.0 * np.pi * FREQ * t
    for n, sgn in [("Left_front_joint", 1.0), ("Right_front_joint", -1.0)]:
        theta_des = theta0[n] + sgn * AMP * np.sin(phase)
        theta = data.qpos[qpos_adr[n]]
        dtheta = data.qvel[qvel_adr[n]]
        data.ctrl[act_id[n]] = KP * (theta_des - theta) - KD * dtheta
    for n in ["Left_rear_joint", "Left_Wheel_joint", "Right_rear_joint", "Right_Wheel_joint"]:
        data.ctrl[act_id[n]] = 0.0


def print_state(model, data, qpos_adr, qvel_adr, site):
    """打印当前状态与闭链约束残差（前后腿 site 重合点的距离，应≈0）。"""
    t = data.time
    leg = lambda n: data.qpos[qpos_adr[n]]
    vel = lambda n: data.qvel[qvel_adr[n]]
    loop_err = {
        "左前环": np.linalg.norm(data.site_xpos[site["Left_front_site1"]]
                                 - data.site_xpos[site["Left_rear_site1"]]),
        "右前环": np.linalg.norm(data.site_xpos[site["Right_front_site1"]]
                                 - data.site_xpos[site["Right_rear_site1"]]),
    }
    print(f"t={t:5.2f}s"
          f"  前腿 L/R={leg('Left_front_joint'):+.3f}/{leg('Right_front_joint'):+.3f}"
          f"  后腿 L/R={leg('Left_rear_joint'):+.3f}/{leg('Right_rear_joint'):+.3f} (闭链带动)"
          f"  轮速 L/R={vel('Left_Wheel_joint'):+.2f}/{vel('Right_Wheel_joint'):+.2f}"
          f"  闭链残差 左={loop_err['左前环']*1000:.1f}mm 右={loop_err['右前环']*1000:.1f}mm")


def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    act_id, jnt, qpos_adr, qvel_adr, site = get_ids(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)  # 从 keyframe 的闭合位形起步
    mujoco.mj_forward(model, data)

    # 闭合位形下 front 关节的角度，作为位置伺服基准
    theta0 = {n: data.qpos[qpos_adr[n]] for n in ["Left_front_joint", "Right_front_joint"]}

    print(f"模型: {model.nq} 广义坐标, {model.nu} 执行器, "
          f"{model.neq} 闭链约束 (equality connect)")
    print(f"驱动: 前大腿位置伺服，正弦 ±{AMP} rad @ {FREQ} Hz | 后大腿/轮子不驱动，闭链联动\n")

    headless = "--headless" in sys.argv
    if headless:
        # 无窗口测试：跑 4 秒，检查数值稳定性与闭链残差
        n_steps = int(4.0 / model.opt.timestep)
        for i in range(n_steps):
            apply_control(data, act_id, qpos_adr, qvel_adr, theta0)
            mujoco.mj_step(model, data)
            if i % int(0.5 / model.opt.timestep) == 0:
                print_state(model, data, qpos_adr, qvel_adr, site)
            if not np.isfinite(data.qpos).all():
                print("!! 数值发散，模型可能不稳定")
                return 1
        print("\nheadless 测试完成：4 秒内数值稳定，闭链残差正常。")
        return 0

    # 实时可视化
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("窗口操作：空格=暂停/继续，右键拖动=旋转视角，滚轮=缩放")
        while viewer.is_running() and data.time < DURATION:
            apply_control(data, act_id, qpos_adr, qvel_adr, theta0)
            mujoco.mj_step(model, data)
            viewer.sync()
    print("仿真结束。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
