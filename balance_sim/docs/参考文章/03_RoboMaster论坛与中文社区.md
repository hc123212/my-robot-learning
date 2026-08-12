# 03 · RoboMaster 论坛与中文社区（平衡步兵 / 轮腿 / LQR / VMC）

> 面向对象：串联腿五杆闭链平衡车，LQR + VMC 控制，MuJoCo 仿真。
> 链接验证说明：RM 论坛帖子、GitHub/Gitee/B 站链接已验证；知乎/古月居因反爬无法直接抓取，标题与 URL 经多次独立搜索确认，内容概述以搜索摘要为准，**打开时建议复核**。
> ⚠️ **许可提醒**：RoboMaster 论坛开源帖均声明「仅限参赛队间交流、禁止商用」；GitHub/Gitee/B 站类多为开放许可（RP_Balance、Graduate_Project、StackForce 均开放）。使用前注意各自许可条款。

---

## ★ 串联腿平衡步兵源头资料（RoboMaster 论坛）

| 资料 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **【RM2023 平衡步兵控制系统开源】上海交通大学·云汉交龙** | https://bbs.robomaster.com/wiki/4574/9430 | 上交云汉交龙战队平衡步兵（**串联腿**结构）控制系统设计文档与测试视频开源，21.3kg、下供弹、360° 腿旋转；后续多支战队（港大 HerKules、B站 LiuDingchuan 等）的轮腿建模均声明参考此开源 | **串联腿平衡步兵系统设计的最重要源头资料**，机械+控制一体 |
| **【RM2024 轮腿平衡电控代码、建模开源及经验分享】香港大学 HerKules** | https://bbs.robomaster.com/article/54292 | 开源轮腿电控代码（Chassis_Task.c 等）与 MATLAB LQR 建模文件 `HerKules_VOCAL_SJ_LQR_v4_with_data.m`（基于上交交龙建模编写），附机械&电控方案 PDF 与经验分享 | 轮腿 LQR 建模 MATLAB 文件直接可参考，补足从"仿真 LQR"到"嵌入式落地"的工程环节 |
| **【RM2024 轮腿步兵机器人嵌入式开源】北京科技大学 Reborn** | https://bbs.robomaster.com/article/17668 | 开源底盘代码（MATLAB 建模 + 下位机代码），实现 **LQR 及 VMC**，线性卡尔曼滤波速度融合，支持跳跃；STM32F446 + BMI088 + 宇树 A1 ×4 + 轮毂电机 ×2 | **LQR + VMC 组合的嵌入式完整实现**，1kHz 核心任务、轮毂电机半双工通信优化等工程细节（论坛热门帖，获赞 40） |
| **【分享帖】二轮平衡步兵的调试心得及仿真开源** | https://bbs.robomaster.com/article/9533 | 作者从 PID 转学 LQR 约 3 个月成功应用于平衡步兵，发布学习历程、仿真过程、MATLAB 代码及《平衡步兵嵌入式技术文档.pdf》 | 少见的"调参踩坑实录"，含 PID→LQR 对比认知与实车调试顺序，**对新手调通 LQR 参数极有帮助** |
| **【共轴麦轮平衡步兵 LQR 控制系统开源】首都师范大学 PIE** | https://bbs.robomaster.com/article/9603 | 开源 `model_b.m`（系统建模求 A、B 矩阵）、《共轴麦轮平衡步兵 LQR 控制系统.docx》（理论学习 + 调试经验）、3 篇相关论文 | 把"建模→LQR→平衡/纵向/转向/平移控制"完整学习路径写成文档，作者自称初学者视角，适合学习对照 |

## 与本项目最接近的仿真实现

| 资料 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **轮腿机器人力控 Webots 仿真（牛顿欧拉法 + LQR + VMC）** | 仓库: https://github.com/LiuDingchuan/Graduate_Project ；视频: https://www.bilibili.com/video/BV15g4y1u7CS/ | 本科毕设开源：C++（Eigen）在 Webots 中复现哈工程创梦之翼平衡步兵控制系统设计，模型、仿真、代码、MATLAB 离线调参全部开源 | **与本项目场景最接近的现成参考**——"理论建模 + LQR + 仿真环境力控实现"，可对照其控制器结构、Eigen 实现与调参流程移植到 MuJoCo |

## 建模与理论文章（知乎 / 古月居）

| 资料 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **RoboMaster 平衡步兵机器人控制系统设计** | https://zhuanlan.zhihu.com/p/563048952 | 平衡步兵底盘建模为**轮腿倒立摆模型**：状态 x=[θ,θ̇,x,ẋ,φ,φ̇]ᵀ、控制 u=[T,Tp]ᵀ，MATLAB 符号法建非线性模型并线性化，验证可控可观性，LQR 求反馈阵 | **与本项目完全同构的建模路线**（串联腿/轮腿倒立摆 6 状态 + LQR），可直接对照状态选取、线性化方法与 LQR 设计 |
| **轮腿机器人动力学建模与控制仿真工程** | https://zhuanlan.zhihu.com/p/2014082592333375099 | 6 维状态（腿杆偏角、位移、机身俯仰角）建模，泰勒展开线性化求 A、B，LQR 控制律 U=-KX，讲**腿长变化时增益调度（get_k.m）**避免摔倒 | 建模步骤极细（含 A/B 矩阵物理意义解读、Q/R 含义），可与交龙/哈工程建模互相印证 |
| **《早期人类如何驯服平衡车？LQR 控制与 Simulink 倒立摆仿真》** | https://zhuanlan.zhihu.com/p/380226006 | 平衡车 = 倒立摆建模（x, v, θ, ω 四状态）、LQR 求解步骤（Q/R → Riccati → 最优控制律）、Simulink 仿真 | 自平衡车 LQR 建模→调参→仿真的完整入门路线 |
| **《跳出课本看 LQR 控制，从公式到代码（上）》** | https://zhuanlan.zhihu.com/p/623843252 | 工程视角讲 LQR：状态空间、二次型代价、Q/R 含义、与 PID 对比、带积分的 LQR 跟踪与抗扰；案例即轮足平衡（Ascento、稚晖君单车）；作者有 RoboMaster 平衡车实战调参经验 | 解决"LQR 算出来但实际不平"的工程问题，**LQR 调参最实用的中文资料之一** |
| **webots 玩转控制论之 LQR 控制器**（罗伯特祥） | https://www.guyuehome.com/17688 | 基于 Webots 的倒立摆 LQR 平衡控制系列：系统建模、可控可观性验证、状态观测器设计（Luenberger，观测器极点比控制器快 4-10 倍） | 含状态观测器设计（部分状态不可测场景），可与 MuJoCo 仿真流程互相参照（古月居链接需自行验证） |

## 视频教程（B 站 / Gitee）

| 资料 | 链接 | 简介 | 参考价值 |
|---|---|---|---|
| **StackForce 大轮足机器人开源项目 + 【手把手教做平衡步兵轮足】系列** | 仓库: https://gitee.com/StackForce/gaint_bipedal_wheeled_robot ；第 7 集 LQR 建模: https://www.bilibili.com/video/BV1qg8JzGEZC/ ；VMC 推导: https://www.bilibili.com/video/BV1TWHrzeEbj/ | 从机械 DIY 到 MIT 电机协议、正逆解、PID 变腿高自稳、**LQR 建模与最优控制、VMC 足端力→关节扭矩**的系统化教学系列，Apache-2.0 开源 | 逐集讲解 LQR 建模与 VMC 力→扭矩推导，视频+代码配套，**最适合系统学习本项目全部知识链** |
| **《双轮足机器人控制方法总结（超详细）》** | https://www.bilibili.com/opus/1030478481038770195 | 转自知乎长文：倒立摆串级 PID → LQR → 轮足 VMC+LQR 的知识树 | 入门总览，先建立全局认知再看深 |
| **《[自制]桌面级轮腿机器人——基于 FOC 电机和 LQR 算法[开源]》** | https://www.bilibili.com/video/BV19w411r7s1/ | Ascento 风格桌面轮腿，开源地址在简介（立创开源广场） | 低成本复刻参考 |

## 其他

| 资料 | 链接 | 简介 |
|---|---|---|
| **rm_controllers**（rm-controls 团队） | https://github.com/rm-controls/rm_controllers | RoboMaster 常用 ROS 控制器集合（底盘、云台、发射机构等），仓库历史含 dev/balance、balance_standard 平衡开发分支（历史分支需自行 checkout）；将来控制层上 ROS 可参考 |
| **达妙科技开源平衡小车 balance_robot** | https://gitee.com/kit-miao/balance_robot | 达妙官方开源平衡小车（MIT），社区公认 LQR 入门级平衡车项目；另有轮足（DM4310×4 + DM6215×2）与桌面轮足开源目录；社区普遍建议先做"板凳模型"（一阶倒立摆）再上轮腿 |
| **CSDN《串联轮腿机器人：运动与稳定解耦的步兵平台设计》** | https://blog.csdn.net/weixin_36299472/article/details/159027929 | 与"串联轮腿 + 稳定解耦"直接同名相关，抓取超时未验证，**需自行验证** |

## 备注

- 港大 HerKules 另有 RM2026 考虑质心偏移的轮腿建模开源帖，**已确认 404 失效**，未列入。
- 与本项目匹配度最高的三条主线：**RP_Balance（五杆建模仿真，见 01 篇）→ 交龙/知乎建模理论 → LiuDingchuan Webots 力控仿真（可对照移植 MuJoCo）**，建议按此顺序研读。
