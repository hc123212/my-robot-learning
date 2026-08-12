# 第 9 章 gymnasium 集成：为强化学习搭台

> 对应学习计划：第 8 阶段（DQN/PPO 训练 Pendulum）
> 预计用时：1.5 小时
> 目标：理解 gymnasium 的接口约定，能在 MuJoCo 模型上做 RL 实验

---

## 9.1 现状（2026-08 本机实测）

| 项目 | 值 |
|---|---|
| gymnasium 版本 | **1.3.0** |
| MuJoCo 环境系列 | **v5 系列**（Ant-v5、HalfCheetah-v5、Humanoid-v5 等 11 个），v4 已标记过时（运行时会打 DeprecationWarning） |
| 阶段 8 的 Pendulum | `Pendulum-v1` 是 gymnasium 自带物理的**经典控制环境**（不是 MuJoCo） |

**两个注意**：

1. **用 v5 不用 v4**：`gym.make("Humanoid-v5")`。v4 虽能跑但官方已声明过时。
2. **缺依赖**：你的环境还没有 `imageio`，而**创建**任何 MuJoCo 环境（哪怕是 headless 训练）都会报 `ModuleNotFoundError: imageio`。进入阶段 8 前先装：

```bash
pip install imageio imageio-ffmpeg
```

## 9.2 gymnasium 接口约定（RL 实验的"方言"）

```python
import gymnasium as gym

env = gym.make("InvertedPendulum-v5", render_mode=None)  # 创建环境

obs, info = env.reset(seed=42)      # 1. 重置，返回初始观测 + 调试信息
for step in range(1000):
    action = env.action_space.sample()     # 2. 选动作（这里是随机）
    obs, reward, terminated, truncated, info = env.step(action)
    #    │      │      │            │
    #    │      │      │            └─ 是否超时（达到最大步数）
    #    │      │      └─────────────── 是否达成目标/失败（回合结束）
    #    │      └───────────────────── 奖励
    #    └──────────────────────────── 新观测
    if terminated or truncated:
        obs, info = env.reset()     # 3. 回合结束，重新开始
env.close()
```

**关键 API 细节**（与旧版 gym 的区别，网上旧教程容易混）：

- `step` 返回 **5 个值**（旧版 4 个）：`obs, reward, terminated, truncated, info`。
- `terminated`（目标达成）和 `truncated`（超时）**分开**——PPO 实现里两者处理不同（值函数估计要区分）。
- 回合结束条件判断 `if terminated or truncated`，不要只查 `terminated`。

## 9.3 观测与动作空间（MuJoCo 环境）

以 `Humanoid-v5` 为例（阶段 6 的站稳目标在 RL 版的对应物）：

```python
env = gym.make("Humanoid-v5")
obs, _ = env.reset()
print("观测形状:", obs.shape)        # (376,)：关节角、角速度、质心、惯性量等
print("动作空间:", env.action_space) # Box(-0.4, 0.4, (17,), float32)：17 个关节力矩
```

- 观测 = 位置 + 速度 + 质心 + 各种辅助量的**拼接**（具体构成见 `gymnasium.envs.mujoco.humanoid_v5` 源码的 `_get_obs`）。
- 动作 = 关节力矩（直接对应 MuJoCo 的 motor 执行器，范围在 `action_space` 里）。
- **奖励是人为设计的**：Humanoid-v5 鼓励前进速度并惩罚控制量——你可以改 `reward_*` 参数（`gym.make("Humanoid-v5", forward_reward_weight=1.0)`）。

## 9.4 阶段 8 的环境选择

任务书写的是"DQN/PPO 训练 Pendulum"。环境选择建议：

| 环境 | 物理 | 适合 |
|---|---|---|
| `Pendulum-v1` | gymnasium 自带（非 MuJoCo） | 任务书原意；训练快、简单 |
| `InvertedPendulum-v5` | **MuJoCo**（倒立摆） | 想用 MuJoCo 完成第 8 阶段 |
| `InvertedDoublePendulum-v5` | MuJoCo | 稍微难一点 |

**建议**：第 8 阶段用 `Pendulum-v1` 验证算法（快），再用 `InvertedPendulum-v5` 检验 MuJoCo 生态的完整链路。两者接口完全一致，你的 DQN/PPO 代码零改动切换。

## 9.5 自定义 MuJoCo 环境（把第 2 章模型变成 RL 环境）

你的学习计划如果想把单摆/2 连杆臂做成 RL 实验，标准做法是继承 `gymnasium.envs.mujoco.MuJoCoEnv`：

```python
import gymnasium as gym
import mujoco
import numpy as np
from gymnasium.envs.mujoco import MuJoCoEnv

class PendulumMuJoCoEnv(MuJoCoEnv):
    """把自定义的 pendulum.xml 包成 gym 环境（最小骨架）。"""
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, render_mode=None):
        super().__init__(xml_path="pendulum.xml",  # 你的 MJCF 文件
                         frame_skip=5,              # 每步 skip 5 个物理步
                         observation_space=gym.spaces.Box(-np.inf, np.inf, (1,), dtype=np.float64),
                         action_space=gym.spaces.Box(-1.0, 1.0, (1,), dtype=np.float64),
                         render_mode=render_mode)

    def _get_obs(self):
        return np.array([self.data.qpos[0]])   # 观测 = 摆角

    def step(self, action):
        self.do_simulation(action, self.frame_skip)  # MuJoCoEnv 提供的核心方法
        obs = self._get_obs()
        reward = -abs(self.data.qpos[0])              # 你的奖励设计
        terminated = False
        truncated = self.data.time >= 10.0
        return obs, reward, terminated, truncated, {}

    def reset_model(self):
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs()
```

要点：

- `MuJoCoEnv.do_simulation(ctrl, n_steps)` 内部就是 `data.ctrl[:] = ctrl; for _ in range(n): mj_step(...)`——你在第 7 章手写的循环被它封装了。
- `frame_skip`：每步物理推进多个 dt（RL 常用 4~5，加速训练、模拟控制频率低于物理频率）。
- `reset_model` 必须返回初始观测。

## 9.6 RL 训练的最小骨架（阶段 8 的代码组织）

阶段 8 要求**自己实现** DQN/PPO，教程只给训练骨架的接口层：

```python
env = gym.make("Pendulum-v1")   # 或你的自定义环境
obs, _ = env.reset()

# —— 你的 DQN/PPO 算法在这里实现（任务书要求自己写）——
for episode in range(1000):
    while True:
        action = your_policy(obs)               # 你的策略网络
        obs, reward, terminated, truncated, _ = env.step(action)
        your_learning_update(...)               # 你的更新逻辑
        if terminated or truncated:
            obs, _ = env.reset()
            break
```

**工程建议**（做第 8 阶段前读一遍）：
1. 训练和渲染分离：`render_mode=None` 训练，训练完用 `render_mode="human"` 看效果。
2. 用 `env.action_space.low/high` 做动作归一化，不要硬编码 -2.0/2.0。
3. 观测缓存成 numpy 数组再进网络，避免每步 numpy 转换开销。
4. 想加速收敛，先学 `InvertedPendulum-v5`（连续控制、单关节）再上 Pendulum。

## 本章小结

| 要做的事 | 结论 |
|---|---|
| 选环境 | v5 系列（`InvertedPendulum-v5`），v4 已弃用 |
| 阶段 8 环境 | `Pendulum-v1`（快）+ `InvertedPendulum-v5`（MuJoCo 全链路） |
| 装依赖 | `pip install imageio imageio-ffmpeg`（**没有它 MuJoCo 环境无法创建**） |
| step 返回值 | 5 个：obs, reward, terminated, truncated, info |
| 自定义环境 | 继承 `MuJoCoEnv`，覆写 `_get_obs` / `step` / `reset_model` |

## 本章练习

1. 装 imageio 后跑通 `InvertedPendulum-v5` 的随机策略循环（随机动作下倒立摆会倒下，观察 terminated 何时触发）。
2. 用第 2 章的单摆写一个自定义 gym 环境（9.5 骨架），控制目标：摆角归零。
3. 阶段 8 前：把 Pendulum-v1 的观测画出来（角度、角速度、时间余弦），理解连续动作空间的奖励形状。

> 进入下一章前，你应该能回答：terminated 和 truncated 的区别？frame_skip 是什么？为什么 MuJoCo 环境创建需要 imageio？
