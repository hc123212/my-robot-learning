# 模块 M3：正向运动学 —— POE 与 DH 双实现（教材 Ch4）

> 本模块同时实现两种 FK 方法：**POE（指数积，课程主线）** 与 **DH（传统，Craig）**，同一机械臂互验。
> 课程视频讲 POE，DH 靠 Craig 第 3 章自学——两种方法都会，面试/工程都覆盖。

## 目标

- 掌握螺旋轴（screw axis）与指数映射，实现 POE 正向运动学
- 掌握 DH 参数法（4 个参数：$a, \alpha, d, \theta$）与坐标系建立规则
- 同一机械臂两种方法结果一致，且与 MuJoCo 仿真一致

## 理论（B 站 BV1KV411Z7sC 第 4 章视频；Craig 第 3 章）

- POE（空间形式）：$T(\theta) = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} M$
  - $M$：零位形（所有 $\theta_i = 0$）时的末端位姿
  - $\mathcal{S}_i = (\omega_i, v_i)$：关节 $i$ 的螺旋轴（空间系下表示），$v_i = -\omega_i \times q_i$（$q_i$ 为轴上一点）
  - $e^{[\mathcal{S}]\theta}$：矩阵指数（用 Rodrigues 公式实现，衔接 00_math 任务 B）
- POE（物体形式）：$T(\theta) = M\, e^{[\mathcal{B}_1]\theta_1} e^{[\mathcal{B}_2]\theta_2} \cdots$（选做）
- DH 变换：$A_i = R_z(\theta_i)\, T_z(d_i)\, T_x(a_i)\, R_x(\alpha_i)$（标准 DH 或改进 DH 选一种，写清用的哪种）
- DH 正向运动学：$T = A_1 A_2 \cdots A_n$

## 实验（自己做）

1. 自己写一个 2 连杆平面臂的 MuJoCo XML（两个旋转关节，臂长自己定）
2. **手写 POE-FK**：为 2R 臂写出 $M$ 与螺旋轴 $\mathcal{S}_1, \mathcal{S}_2$，手写 `MatrixExp6`（指数映射）与 `FKinSpace`（numpy 实现，不调库）
3. **手写 DH-FK**：同一臂建 DH 表，手写变换链乘积
4. **三对照**：MuJoCo 仿真中随机取 10 组关节角，读仿真末端位姿，与手写 POE、手写 DH 对比，画误差分布（应 < 1e-6）
5. **与官方库对照**：同样的 $M, \mathcal{S}$ 传给 `modern_robotics_lib` 的 `FKinSpace`，验证你的实现与库一致
6. **UR5 升级**（进阶）：加载 UR5 模型，从模型文件提取螺旋轴参数，做 6 关节 POE-FK

## 验收

- [ ] 手写 POE / 手写 DH / MuJoCo 三者末端位姿误差全部 < 1e-6（10 组关节角）
- [ ] 换一组不同的臂长参数，重算仍一致（参数化，不是写死）
- [ ] 与官方库 `FKinSpace` 对照一致
- [ ] NOTES.md：DH 参数表推导（含坐标系说明）+ 螺旋轴推导 + **两者对比（POE 为什么不需要为每个连杆建坐标系、改模型时为什么更不容易错）**

## 卡住

1. "螺旋轴怎么求" → 视频 4.1.1：$\mathcal{S}_i = (\omega_i, v_i)$，$\omega_i$ 为转轴单位向量，$v_i = -\omega_i \times q_i$
2. "DH 表怎么填"是最大坎 → Craig 第 3 章标准 DH 的 4 个规则（z 轴沿关节轴、x 轴沿公垂线…），不要跳
3. 指数映射实现错 → 回看 00_math 任务 B 的 $\hat{\omega}^3 = -\hat{\omega}$ 循环规律
4. 卡超 2 小时 → 找 Claude Code 要提示，不要完整代码
