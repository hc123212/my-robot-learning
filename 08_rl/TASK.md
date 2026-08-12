# 阶段 8：强化学习控制（第 10 周）

## 目标
- 理解 MDP、策略、值函数、奖励闭环
- 手写一个 DQN 或 PPO（torch），训练 gymnasium 环境
- 对比 RL 学出的控制 vs 你手写的 PD 控制

## 理论（李宏毅深度强化学习 B 站课，配 DeepRL 经典解读）
MDP 五元组、策略梯度直觉、DQN（经验回放 + 目标网络）、PPO（重要性采样 + clip）

## 实验（自己做）
1. `gymnasium.make("Pendulum-v1")` 或 `Pendulum-v1` 先跑通随机策略，观察环境接口（obs/action/reward 维度）
2. 手写 DQN（约 100-150 行 torch）：Q 网络 + 经验回放 + ε-greedy
3. 训练并画奖励曲线（学习曲线上升为合格）
4. 可选：`Humanoid-v5` 上跑通（环境已装），对比 RL 的站立策略 vs 阶段 6 你的 PD 策略（动作风格完全不同）

## 验收
- [ ] Pendulum 训练曲线明显上升（最终平均奖励高于随机策略）
- [ ] NOTES.md：写下"RL 和模型控制（PD/CTC）各自的适用场景"——这是具身智能面试必问题
- [ ] 能口头解释：为什么 RL 不需要动力学模型（model-free）

## 卡住
训练不收敛 → 先查环境动作范围（Pendulum 的 action ∈ $[-2, 2]$）；超参用文档默认值；batch 32、lr 1e-3 起步。
