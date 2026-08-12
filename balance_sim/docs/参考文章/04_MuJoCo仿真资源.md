# 04 · MuJoCo 仿真资源（模型 / 官方 API / 控制示例）

> 面向对象：串联腿五杆闭链平衡车，LQR + VMC 控制，MuJoCo 3.11。
> 链接验证说明：GitHub 与 readthedocs 域名在本环境无法直接抓取，链接均经多次交叉搜索确认；mujoco.readthedocs.io 章节 URL 为官方文档标准结构，未逐一验证。

---

## 两个关键事实（先记住）

1. **mujoco_menagerie 中没有轮腿一体（wheeled-legged）模型，也没有 ballbot**。其清单含：双足 `agility_cassie`；人形 `unitree_g1/h1/z1`；四足 `unitree_a1/go1/go2`、`anybotics_anymal_b/c`、`spot`；轮式移动操作臂 `stretch_2/3`、`tiago`。最接近"带轮子的腿式机器人"的是 Stretch/Tiago（轮式底盘+臂，非平衡型），双足/四足的平衡与步态建模可借鉴。
2. **MuJoCo 线性化有官方内建 API**：`mjd_transitionFD`（自 2.2.1 起），一次性计算 A/B/C/D 四个雅可比矩阵，是 MuJoCo 平衡控制（LQR 等）的标准做法，官方 LQR 教程即用此函数。

## 资源列表（按参考价值从高到低）

| # | 名称 | 链接 | 类型 | 简介 | 参考价值 |
|---|---|---|---|---|---|
| 1 | **官方 LQR 教程笔记本（LQR.ipynb）** | https://github.com/google-deepmind/mujoco/blob/main/python/LQR.ipynb ；Colab 直跑: https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/LQR.ipynb | 官方示例代码 | 官方 LQR 教程：`mjd_transitionFD` 有限差分线性化 + `solve_discrete_are` 解黎卡提 + `mj_differentiatePos` 处理四元数状态差，单腿平衡一个 humanoid，含 `mj_inverse` 求平衡力矩设定点 | **最高**。与本项目方法论完全一致，是 A/B 矩阵线性化、控制回路（`ctrl = ctrl0 - K@dx`）、自由基座（free joint 四元数 qpos=7/qvel=6）处理的官方范本，直接可跑可改 |
| 2 | **Learning_Balance_Control_PNU2025-1** | https://github.com/noah-boeckmann/Learning_Balance_Control_PNU2025-1 | 示例代码 | 轮腿机器人 MuJoCo 平衡项目：按 S. Wang et al.（Novel Wheel-legged Robot）论文参数建模的 MJCF XML + Gymnasium 环境（左右轮转矩驱动），课程式扰动训练 PPO/SAC | **很高**。全网最接近"轮腿平衡车"的现成 MuJoCo 模型与环境，XML 建模（轮驱动、密度调参、机身传感器）可直接借鉴 |
| 3 | **MuJoCo 官方文档（Computation / Modeling / Python API）** | https://mujoco.readthedocs.io/en/stable/computation/index.html ；https://mujoco.readthedocs.io/en/stable/modeling.html ；https://mujoco.readthedocs.io/en/stable/python.html | 官方文档 | computation 章节含动力学/导数流水线；modeling 章节含 joint/free joint 规范（自由关节仅限顶层、qpos 7 维、四元数，gear 6 元组驱动）；python.html 含 `mjd_transitionFD` 签名与参数 | **很高**。五杆闭链机构用 equality constraint 建模、free joint 浮动基座、`mjd_transitionFD(eps, centered)` 拿线性化 A/B——正是本项目核心建模+控制点 |
| 4 | **mujoco_menagerie（官方模型库）** | https://github.com/google-deepmind/mujoco_menagerie | 模型库 | DeepMind 官方策展 70+ 高质量模型（cassie、unitree A1/Go1/Go2/G1/H1、ANYmal B/C、Spot、Stretch 等），BSD-3 许可，统一目录结构 | **中高**。参考腿式机器人的标准 MJCF 写法（位置执行器 kp/kv、自由基座、IMU 传感器、质量属性）；无轮腿模型，但 cassie 平衡控制细节与 unitree 执行器参数风格可借鉴 |
| 5 | **mujoco_playground** | https://github.com/google-deepmind/mujoco_playground | RL 框架 | DeepMind + UC Berkeley 的 MuJoCo XLA（MJX）RL 框架，19 个 locomotion 环境（G1、H1、Go2 等），PPO/SAC + 域随机化，RSS 2025 最佳 Demo | **中高**。后期想把平衡问题 RL 化时，环境/奖励设计（orientation 惩罚、速度跟踪）与域随机化配置可复用；对当前 LQR+VMC 直接价值有限 |
| 6 | **Gymnasium MuJoCo 环境（InvertedPendulum-v4 / InvertedDoublePendulum-v4）** | https://gymnasium.farama.org/environments/mujoco/ | RL 环境 | Gymnasium 内置 11 个 MuJoCo 环境，含倒立摆与双倒立摆，XML 在 gymnasium/envs/mujoco/assets/ 下 | **中**。现成平衡基准环境：可先在上面验证 LQR 设计再移植到自建模型；标准 gym.Env 接口参考实现 |
| 7 | **iLQG-MuJoCo** | https://github.com/MahanFathi/iLQG-MuJoCo | 示例代码 | iLQG/iLQR 轨迹优化在 MuJoCo 上的实现，含 inverted pendulum 演示，并行有限差分、MPC 模式 | **中**。LQR 只在平衡点附近有效；大扰动下的非线性稳定，iLQG 是自然升级路径 |
| 8 | **mujoco-sysid** | https://github.com/based-robotics/mujoco-sysid | 库 | MuJoCo 系统辨识库（`pip install mujoco-sysid`），含 cartpole 惯性参数辨识 + LQR 稳定示例 | **中**。之后要仿真参数匹配实物（电机模型、摩擦、质量）时可复用辨识流程 |
| 9 | **MuJoCo-Tutorial（GitCode 镜像）lqr_control.py** | https://gitcode.com/gh_mirrors/mu/MuJoCo-Tutorial/blob/main/examples/lqr_control.py | 教程代码 | cartpole LQR 完整单文件：手写有限差分线性化（mj_step 扰动法）+ `scipy.linalg.solve_discrete_are` + mujoco.viewer 可视化 | **中**。无依赖的最朴素线性化实现（不依赖 mjd_transitionFD），适合理解线性化原理后再切换到官方 API |

## 给本项目的具体建议

- **首选组合**：#1（官方 LQR 流程）+ #3（建模/线性化规范）+ #2（轮腿模型参考）即可覆盖"五杆闭链模型 → free joint 浮动基座 → mjd_transitionFD 线性化 → LQR/VMC 控制回路"完整链条。
- **注意**：MuJoCo 3.11 中 `mjd_transitionFD` 与 `mj_differentiatePos` 均可用；对含四元数/闭链的模型，状态差分必须用 `mj_differentiatePos`，不要直接 `qpos` 相减（官方 LQR 教程对此有专门处理，与你 `01_串联腿LQR_VMC平衡控制.md` 中 `mj_integratePos` 扰动方案互为印证）。
- **VMC 方面**：以上资源中没有现成的 VMC 实现（VMC 多出现在 Isaac/legged_gym 生态）；VMC 参考见 01 篇（Cheetah-Software、QuadrupedSim）与 02 篇（Pratt 论文 + wheel-leg-climber 推导）。
