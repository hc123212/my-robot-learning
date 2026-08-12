# MuJoCo 仿真教程指导书

> 配套《机器人运动学与动力学控制 · 学习计划》（仓库根目录 README.md）使用。
> 目标读者：已完成第 0 阶段（二阶系统），即将进入第 1~8 阶段仿真实验的你。
>
> **版本基线：MuJoCo 3.11.0 / Python 3.12 / gymnasium 1.3.0（本机 `D:\pyenvs\robotics` 已实测验证）**

---

## 为什么需要这本指导书

你的学习计划从第 1 阶段开始就大量依赖 MuJoCo：写 XML 建模、对比手写 FK、验证雅可比、读逆动力学、做 PD 控制、最后接 gymnasium 跑 RL。但 MuJoCo 的资料有个特点：

1. **时效性强** —— 它每月发一个新版本，API 频繁变动。网上 2024 年的教程可能已经报错。
2. **官方文档质量高但分散** —— 英文、按主题分布，初学者不知道先读哪篇。
3. **坑藏在细节里** —— 比如 3.11 版 `mj_fullM` 改了函数签名，旧教程的写法会直接 TypeError。

本指导书把**你学习计划真正会用到的 API** 全部实测验证（基于本机 3.11.0），按你 9 个阶段的顺序组织，每个知识点都给出可运行代码。

## 与学习计划的对照表

| 你的阶段 | 主题 | 配套章节 |
|---|---|---|
| 0 | 数学预热（不碰 MuJoCo） | 01（环境验证） |
| 1 | 位姿描述与齐次变换 | 02（MJCF 建模）、03（读取 qpos）、05（FK 数据对比） |
| 2 | 正向运动学 DH-FK | 02、03、05 |
| 3 | 逆向运动学 | 03（写 qpos）、05（雅可比）、07（关节控制） |
| 4 | 速度运动学与雅可比 | 05（mj_jac / mj_objectVelocity / JᵀF） |
| 5 | 动力学（拉格朗日） | 06（mj_inverse / mj_fullM） |
| 6 | PD / 计算力矩 / 阻抗控制 | 07（控制与力施加） |
| 7 | 轨迹规划 | 07（position 执行器轨迹跟踪） |
| 8 | 强化学习 | 09（gymnasium 集成） |
| 进阶 | ROS2 / 具身智能 | 10（开源项目） |

## 阅读顺序建议

**第一次通读**：01 → 02 → 03 → 04，共约 3~4 小时，建立"XML → MjModel → MjData → mj_step → viewer"的完整心智模型。

**按阶段查阅**：进入某个阶段前，先读对照表里对应的章节（10~20 分钟），再开始做 TASK.md 的实验。卡住时回来查附录。

**附录当字典**：《附录_3.11变更与排错.md》里的 API 速查表和常见错误表，遇到报错先翻它。

## 学习铁律（与仓库 README 一致）

- 本指导书**只教 API，不给实验答案**。FK/IK/PD/PPO 的算法实现请按 TASK.md 自己写。
- 卡住原则不变：先重读理论 30 分钟 → 再查本指导书 → 仍不行跳过去，第二天回来。
- 每个知识点代码都可以直接复制运行，但**建议亲手敲一遍**。

## 章节导航

| 章节 | 一句话内容 |
|---|---|
| [01_安装与快速上手.md](01_安装与快速上手.md) | 版本现状、环境验证、第一个仿真脚本 |
| [02_MJCF建模语言.md](02_MJCF建模语言.md) | 用 XML 描述机器人：body/geom/joint/site/actuator |
| [03_核心PythonAPI.md](03_核心PythonAPI.md) | MjModel/MjData、mj_step 循环、命名访问、numpy 内存语义 |
| [04_可视化.md](04_可视化.md) | viewer 三种模式、Simulate 程序、离屏渲染 |
| [05_运动学与雅可比.md](05_运动学与雅可比.md) | 读状态算 FK、mj_jac、mj_objectVelocity、τ=JᵀF 验证 |
| [06_动力学接口.md](06_动力学接口.md) | mj_inverse 逆动力学、mj_fullM 质量矩阵（3.11 新签名）、接触力 |
| [07_控制与力施加.md](07_控制与力施加.md) | 执行器与 ctrl、内置 PD、手写 PD、外力施加 |
| [08_传感器与数据采集.md](08_传感器与数据采集.md) | sensor 定义、sensordata 布局、接触信息、数据记录 |
| [09_gymnasium与强化学习.md](09_gymnasium与强化学习.md) | v4 环境、自定义 MuJoCo gym 环境、RL 实验平台 |
| [10_开源项目与进阶.md](10_开源项目与进阶.md) | 官方 notebooks、Menagerie 模型库、dm_control、robosuite 等 |
| [附录_3.11变更与排错.md](附录_3.11变更与排错.md) | 3.11 破坏性变更、常见错误对照表、API 速查表 |

## 本文档的信息来源与验证方法

1. **本机实测**（最可靠）：`D:\pyenvs\robotics` 环境里 mujoco 3.11.0 的全部关键 API 均已运行验证（验证日期 2026-08-06）。
2. **官方资料**：
   - MuJoCo 文档：<https://mujoco.readthedocs.io/en/stable/index.html>
   - MuJoCo GitHub（含教程 notebook）：<https://github.com/google-deepmind/mujoco>
   - MuJoCo 发布页：<https://github.com/google-deepmind/mujoco/releases>
   - MuJoCo Menagerie 模型库：<https://github.com/google-deepmind/mujoco_menagerie>
3. 文中所有"实测"标注指本机 3.11.0 环境验证通过；跨版本的差异见附录。

## 运行环境速查

```bash
# 激活你的环境（Windows PowerShell）
D:\pyenvs\robotics\Scripts\Activate.ps1

# 验证版本
python -c "import mujoco, gymnasium; print(mujoco.__version__, gymnasium.__version__)"
# 输出: 3.11.0 1.3.0

# 启动 MuJoCo Simulate 程序看任意模型
python -m mujoco.viewer --mjcf=你的模型.xml
```
