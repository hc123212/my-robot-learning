# my-robot-learning

该仓库用于记录个人的机器人学习历程

---

## 机器人运动学与动力学控制 · 学习计划

> **📖 课程主计划：[PLAN.md](PLAN.md)** —— 本 README 的 9 阶段计划已升级并入《Modern Robotics 课程学习与实操指导计划》，以 Coursera 六门课（力学/运动学/动力学/规划/控制/抓取与移动机器人）为主线，每个模块含理论学习 + 上手实操 + 量化验收 + 抗遗忘机制，并新增 6 个实操模块（C-space、闭链、运动规划、抓取、移动机器人、Capstone）。学习流程以 PLAN.md 为准。
>
> 理论 + MuJoCo 仿真实践，每天 1-2 小时（时长服从覆盖度，详见 PLAN.md）。
> 环境：`D:\pyenvs\robotics`（mujoco 3.11 / gymnasium 1.3 / scipy / numpy / matplotlib / torch）。
> 官方参考库已 clone 至 `modern_robotics_lib/`（gitignore 排除），作手写实现的对照层。

## 学习方式（重要）

每个阶段目录里有 `TASK.md`（任务书），流程固定为：

1. 读 `TASK.md`，明确本阶段目标与验收标准
2. 按教材章节学理论（Modern Robotics 为主，Craig 查细节）
3. **自己动手实现**实验（任务书只给思路和提示，不给答案代码）
4. 对照验收标准自测，把结果和心得写进本目录 `NOTES.md`
5. 完成后 commit 一次（建议 `git init`）

**卡住原则**：先重读理论 30 分钟 → 再查 MuJoCo 文档 → 仍不行就跳过，第二天回来。单点卡死不超过 2 小时。

## 教材

| 资源 | 用途 |
|---|---|
| 《Modern Robotics》Lynch & Park（Coursera 免费课 + 官网 Python 代码库） | 主教材 |
| 《机器人学导论》Craig 中文译本 | 查 DH 参数、动力学细节 |
| MuJoCo 官方文档（Python tutorial） | 实验工具书 |

## MuJoCo 仿真教程指导书

> 进入第 1 阶段前，先读 `mujoco_tutorial/`（基于 mujoco 3.11.0 实测编写，含全部所需 API 与开源项目参考）：
>
> 📖 [mujoco_tutorial/README.md](mujoco_tutorial/README.md) — 总览与各阶段对照表

## 阶段总览

| 阶段 | 目录 | 主题 | 实验（自己做） | 周数 |
|---|---|---|---|---|
| 0 | `00_math` | 数学预热：二阶系统 | 弹簧-阻尼响应三种阻尼曲线 | 1 |
| 1 | `01_pose` | 位姿描述与齐次变换 | 单摆旋转矩阵验证、欧拉角/四元数、万向锁 | 1 |
| 2 | `02_fk` | 正向运动学 DH-FK | 手写 DH-FK vs MuJoCo 仿真对比 | 1 |
| 3 | `03_ik` | 逆向运动学 | 阻尼最小二乘 IK，末端到目标点 | 1 |
| 4 | `04_jacobian` | 速度运动学与雅可比 | 雅可比对比、奇异位形、$\tau = J^{\mathsf{T}}F$ | 1 |
| 5 | `05_dynamics` | 动力学（拉格朗日） | 逆动力学 + 计算力矩控制 CTC | 2 |
| 6 | `06_control` | PD / 计算力矩 / 阻抗控制 | humanoid 站稳 + 机械臂阻抗 | 1 |
| 7 | `07_trajectory` | 轨迹规划 | 五次多项式轨迹跟踪 | 1 |
| 8 | `08_rl` | 强化学习控制 | DQN/PPO 训练 Pendulum | 1 |

## 进阶衔接（完成 8 个阶段后）

- ROS 2 化：控制节点上 Ubuntu + Gazebo → RDK 板子（TROS 接口）
- 具身智能：RL → LeRobot/ACT 模仿学习 → `rdk-embodied-lerobot` 部署
- 参考你的 RDK 开发技能：`rdk-ros`、`rdk-embodied-lerobot`

## 验收铁律

每个实验必须完成"理论公式 → 代码 → 仿真数据"三者对照。跑通不算完，**误差对不上就回头查数学**。

## 环境使用

```bash
D:\pyenvs\robotics\Scripts\Activate.ps1
python D:/robotics/demo_mujoco.py   # 第 0 课示例：humanoid 可视化
```
