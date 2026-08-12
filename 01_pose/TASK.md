# 模块 M2：刚体运动 —— 旋转矩阵、指数坐标、齐次变换（教材 Ch3）

## 目标

- 理解旋转矩阵 $R \in SO(3)$、齐次变换矩阵 $T \in SE(3)$、变换链
- 掌握欧拉角与四元数，理解万向锁
- 掌握指数坐标（so(3)/se(3) 与矩阵指数），衔接 M3 的 POE 正向运动学

## 理论（B 站 BV1KV411Z7sC 第 3 章视频；Craig 第 2 章）

- 旋转矩阵性质（正交 $R^{\mathsf{T}}R = I$、$\det R = +1$）、绕坐标轴旋转矩阵
- 欧拉角序列、四元数表示、齐次变换的复合 $T_{13} = T_{12}\, T_{23}$、逆变换
- 角速度与 so(3)：$\dot{R} = \hat{\omega}R$，$\hat{\omega} \in so(3)$
- 指数坐标：$\exp(\hat{\omega}\theta)$（Rodrigues 公式），对数映射 $\log R$（提取轴角）

## 实验（自己做）

1. MuJoCo 加载单摆模型（自己写 XML：一个 base + 一个带旋转关节的杆，参考 MuJoCo 文档最简单的 model），读取关节角 → 自己构造旋转矩阵 → 打印
2. 代码验证 $R \cdot R^{\mathsf{T}} = I$、$\det(R) = 1$
3. 同一姿态分别用欧拉角和四元数表示，改变关节角观察两者数值变化
4. **万向锁演示**：用 scipy 的 `Rotation`（`scipy.spatial.transform.Rotation`）构造欧拉角序列，找出一组角度使两个旋转轴重合，观察自由度丢失
5. **手写指数映射对照库**（新增）：手写 `MatrixExp3` / `MatrixLog3`（Rodrigues 公式）、`RpToTrans` / `TransInv`、`Adjoint`，与 `modern_robotics_lib/core.py` 同函数对照（误差 < 1e-9），并与 scipy 对照
6. **书面回答**（NOTES.md）：为什么角速度不是一个"角度变量的导数"？（从 $\dot{R} = \hat{\omega}R$ 出发）

## 验收

- [ ] 手写 $R_x(\theta) \cdot R_y(\varphi)$ 矩阵乘法，与 numpy 结果一致
- [ ] 手写矩阵指数/对数/伴随与库、scipy 三方对照误差 < 1e-9
- [ ] 能解释"为什么旋转矩阵的逆等于转置"
- [ ] NOTES.md 写清：万向锁是什么、四元数为什么能避免它、角速度与姿态导数的关系

## 卡住

MuJoCo XML 写不出来 → 先读 MuJoCo 文档 "Models" 一节（一个 geom + 一个 joint 的最简模型示例）；卡超 2 小时再找 AI 要提示。
