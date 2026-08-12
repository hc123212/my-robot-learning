# 参考文章 · 串联腿平衡车（LQR + VMC）资料库

> 对象：COD-2026 串联腿（五杆闭链）平衡车，MuJoCo 仿真，LQR + VMC 控制。
> 搜集日期：2026-08-11。链接验证状态见各文件头部说明；知乎/古月居反爬内容以搜索摘要为准，打开时建议复核。

## 分类索引

| 文件 | 内容 | 最重要条目 |
|---|---|---|
| [05_串联腿vs并联五杆轮腿_机构与控制对比.md](05_串联腿vs并联五杆轮腿_机构与控制对比.md) | **术语澄清**：本模型是闭链（双闭环）而非纯串联；Grübler 自由度计算、动力学/控制数学差异 | ⭐ 回答"这些资料跟我模型是不是一类"——结论：RP_Balance 等五杆资料是同类参考 |
| [01_GitHub开源项目.md](01_GitHub开源项目.md) | 轮腿/平衡/五杆/闭链开源代码（RP_Balance、og_bruce、Cheetah-Software 等） | ⭐ RP_Balance（与本项目同构的 RM2024 五连杆轮足） |
| [02_论文与理论.md](02_论文与理论.md) | VMC 原始论文、Underactuated Robotics、Ascento、五杆腿设计论文 | ⭐ Pratt VMC 开山论文 + ETH Ascento（对标系统） |
| [03_RoboMaster论坛与中文社区.md](03_RoboMaster论坛与中文社区.md) | RM 平衡步兵开源（交龙/HerKules/Reborn）、知乎建模文章、B 站教程 | ⭐ 上交云汉交龙串联腿源头开源 + 知乎轮腿倒立摆建模 |
| [04_MuJoCo仿真资源.md](04_MuJoCo仿真资源.md) | 官方 LQR 教程、mjd_transitionFD、模型库、轮腿 MuJoCo 模型 | ⭐ 官方 LQR.ipynb（方法论一致，直接可跑） |

## 推荐研读路径（结合当前进度）

你现在正处于"数值线性化 → LQR → VMC"的实现阶段，建议顺序：

```
① 术语澄清（先读这篇，1 小时）
   05_串联腿vs并联五杆轮腿_机构与控制对比 → 你的模型是闭链，与五杆资料同大类；平衡层/执行层分流原则

② 理论补强（1-2 天）
   Pratt VMC 原始论文（02）→ 只需读"虚拟力 + $J^{\mathsf{T}}$ 映射"核心两节
   Tedrake Underactuated Robotics LQR 章节（02）→ 倒立摆线性化 + Q/R 选取

③ 方法对齐（对照你的 01_串联腿LQR_VMC平衡控制.md）
   官方 LQR.ipynb（04）→ mjd_transitionFD 线性化流程，与你手写差分互为验证
   wheel-leg-climber VMC 推导（02）→ $\tau = J^{\mathsf{T}}F$ 公式落地，前后腿符号差异

④ 同构代码精读（2-3 天，重点）
   RP_Balance（01/03）→ 五连杆解算 + LQR(K 随腿长) + VMC + A/B 矩阵拟合
   知乎"RoboMaster 平衡步兵控制系统设计"（03）→ 6 状态轮腿倒立摆建模，完全同构

⑤ 工程化落地参考
   LiuDingchuan Graduate_Project（03）→ Webots 力控实现，可对照移植 MuJoCo
   北科大 Reborn（03）→ LQR + VMC 嵌入式实现（后面上真车再看）
   上交云汉交龙（03）→ 串联腿机械结构源头
```

## 关键结论速览

1. **与本项目最同构的开源代码**：`WilliamGwok/RP_Balance`（RM2024 深圳大学，五连杆轮足 + LQR + VMC，K 矩阵随腿长拟合）。
2. **MuJoCo 线性化标准做法**：官方内建 `mjd_transitionFD`（一次出 A/B/C/D），官方 LQR.ipynb 有完整范本；四元数状态差必须用 `mj_differentiatePos`。
3. **五杆闭链 MJCF 建模稀缺**：`alvister88/og_bruce` 是目前唯一原生闭链建五杆的教程代码。
4. **对标系统**：ETH Ascento（LQR-Assisted WBC + 四杆闭链腿）与 IEEE 2025 五杆轮腿平台（LQR + VMC）。
5. **无参考价值勿浪费时间**：ETH Swiss-Mile 官方代码未公开、Ascento 无官方开源、Boston Dynamics Handle 无学术论文。
6. **机构类型判定见 05 篇**：本模型是闭链（每侧双闭环）而非纯串联；RP_Balance 等五杆资料是同类参考（执行层），交龙等"串联腿"资料用于平衡层（通用）。

## 许可提醒

RoboMaster 论坛开源帖均为「仅限参赛队间交流、禁止商用」；GitHub/Gitee/B 站类多为开放许可。商用或分发前请核对各自 LICENSE。
