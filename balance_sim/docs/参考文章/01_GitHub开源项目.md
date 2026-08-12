# 01 · GitHub 开源项目（轮腿 / 串联腿 / 平衡控制）

> 面向对象：串联腿五杆闭链平衡车，LQR + VMC 控制，MuJoCo 仿真。
> 链接验证：star 数、最后提交时间经 GitHub API 实时验证（截至 2026-08-11）。

---

## ★ 第一梯队：与本项目直接同构（五杆轮足 + LQR + VMC）

| 项目 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **RP_Balance**（深圳大学 RobotPilots 战队，RM2024） | https://github.com/WilliamGwok/RP_Balance | 五连杆轮足式机器人建模与仿真代码，MATLAB 实现，40★ | **本项目最像的仓库，强烈推荐精读**。五连杆闭链运动学解算、VMC 虚拟腿力计算、LQR（K 矩阵随腿长变化拟合）、轮地扰动 A/B 矩阵拟合（MPC 思想）、直腿模型简化，含仿真与实车视频、LQR 调参笔记 |
| **og_bruce** | https://github.com/alvister88/og_bruce | 在 MuJoCo MJCF 中建模并联机构（四杆、五杆、差速齿轮）的教程型仓库，11★ | 罕见的"如何在 MJCF 原生建五杆闭链约束"代码（闭链用 MuJoCo 原生约束而非串联近似），配套论文 arXiv:2507.00273（BRUCE 人形并联机构） |
| **Wheel-Legged-Gym**（南方科技大学 Clear Lab） | https://github.com/clearlab-sustech/Wheel-Legged-Gym | 轮腿机器人 RL 训练框架（legged_gym + rsl_rl，Isaac Gym），630★ | 任务配置含 `wheel_legged_vmc_flat` 等 VMC 相关任务，2 腿轮腿完整训练/部署流程；RoboMaster 平衡步兵训练常用。后续想 RL 化时再看 |
| **balance-robot-mujoco-sim** | https://github.com/lachlanhurst/balance-robot-mujoco-sim | 两轮自平衡机器人 MuJoCo 仿真 + LQR 控制器，PySide6 界面，30★ | 最小可运行的"MuJoCo 平衡闭环"模板（XML + LQR 回路 + 可视化），含 `calculate_lqr_gains.py`，适合作为控制循环起点 |
| **Control-strategies-for-two-wheeled-inverted-pendulum** | https://github.com/Manas-arumalla/Control-strategies-for-two-wheeled-inverted-pendulum | 两轮倒立摆（Segway 型）15+ 控制策略对比：LQR、iLQR、MPC、H∞、SMC、backstepping 等，MATLAB 推导 + MuJoCo 验证，遗传算法整定 | 完整展示"平衡点线性化 → 控制器设计 → MuJoCo 验证"全流程；升级到 MPC/iLQR 的现成对照 |
| **Self-Balancing-Robot** | https://github.com/giulioturrisi/Self-Balancing-Robot | ROS2 两轮倒立摆控制全集：LQR、LQI、自适应 LQR、SMC、iLQR/Crocoddyl、NMPC/Acados、EKF，MuJoCo/CoppeliaSim 双仿真，58★ | 控制器家族最全，可参考 LQI/自适应 LQR 与 EKF 状态估计；缺点：ROS2 依赖重 |

## 第二梯队：VMC / 平衡控制器权威实现

| 项目 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **Cheetah-Software**（MIT 仿生实验室） | https://github.com/mit-biomimetics/Cheetah-Software | MIT Cheetah 3 / Mini Cheetah 全套开源软件，3275★ | VMC（虚拟弹簧-阻尼）+ 力分配（QP/凸优化）源头级代码（BalanceController 等），VMC 思想的正典 |
| **A1-QP-MPC-Controller** | https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller | MIT Cheetah 3 控制器（QP + MPC + 状态估计）移植宇树 A1，827★ | 学习 WBC 力分配与 MPC 落足力规划的最完整中文配套教程（B 站有系列课程），理解 VMC→WBC 演进 |
| **QuadrupedSim** | https://github.com/YuXianYuan/QuadrupedSim | 四足机器人仿真，明确标注 VMC 算法（虚拟力 + 雅可比转置力矩映射），41★ | 中文仓库、代码量小，快速看 VMC 落地写法 |
| **py-apple-quadruped-robot**（菠萝狗） | https://github.com/ToanTech/py-apple-quadruped-robot | 开源四足菠萝狗，1295★ | 明确含 VMC 算法 + 8DOF 逆解，Python，VMC 简易落地的直观范例 |

## 第三梯队：轮腿/双足轮式平台与模型

| 项目 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **upkie** | https://github.com/upkie/upkie | 完全开源轮式双足自平衡机器人（硬件+固件+仿真+控制），398★，活跃维护 | 含 MPC/PID/RL 多种平衡器，Python/C++ 双实现，`pip install upkie` 即跑仿真（Bullet 引擎）；"轮子平衡"动力学与控制架构直接可参考 |
| **cassie-mujoco-sim**（OSU 动态机器人实验室） | https://github.com/osudrl/cassie-mujoco-sim | Cassie 双足 MuJoCo 仿真库（C + Python ctypes + UDP），309★ | MuJoCo 闭链腿（弹簧驱动并联踝）建模 + "仿真即控制接口"完整做法 |
| **Wheel-Legged-Lab** | https://github.com/zyicome/Wheel-Legged-Lab | Isaac Lab 轮腿平衡/跳跃/运动 RL 开源项目，13★，活跃维护 | 轮腿平衡任务定义、奖励设计、sim2real 考虑（真实传感器高度图） |
| **go2w_rl_gym** | https://github.com/ShengqianChen/go2w_rl_gym | 宇树 Go2W 轮腿完整 RL 训练 + 实机部署仓库，48★ | 轮腿状态定义、奖励、策略蒸馏部署的现成范例 |
| **Digit-MuJoCo-ROS2** | https://github.com/MindSpaceInc/Digit-MuJoCo-ROS2 | Agility Digit 人形 MuJoCo 仿真（ROS2，C++），11★ | digit.xml 模型与 MuJoCo 消息桥接示例 |
| **cassie_rl_walking** | https://github.com/HybridRobotics/cassie_rl_walking | Cassie 双足 RL 行走（MuJoCo 2.1），74★ | 学习型控制方向参考 |
| **LQR_phc** | https://github.com/Hawking-z/LQR_phc | 中文 STM32 二轮平衡车 LQR 全流程（MATLAB 建模 + 固件 + PCB），48★ | 硬件向参考，含调参细节 |

## 资源大全（强烈建议收藏）

| 项目 | 链接 | 简介 |
|---|---|---|
| **awesome-wheeled-legged** | https://github.com/XinLang2019/awesome-wheeled-legged | 轮足/轮腿四足机器人开源代码、论文、产品清单大全（54★，2025-07 仍在更新），后续扩充资料直接翻这里 |

## 关键结论

1. **最值得精读的只有一个**：`WilliamGwok/RP_Balance` —— 五连杆轮足 + LQR（K 随腿长拟合）+ VMC + A/B 矩阵拟合，与本项目（串联腿五杆闭链 + LQR + VMC）直接同构。
2. **MuJoCo 五杆闭链建模**开源资源稀缺，`alvister88/og_bruce` 是唯一以 MJCF 原生闭链建五杆的教程代码。
3. **VMC 源头代码**在 MIT `Cheetah-Software`；轮腿 RL 主线在 `Wheel-Legged-Gym` / `Wheel-Legged-Lab`。
4. **ETH Swiss-Mile 官方代码未公开开源，Ascento 无官方开源仓库**——不要浪费时间寻找。
