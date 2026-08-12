# COD-2026 平衡车（串联腿闭链）MuJoCo 仿真

基于辽宁科技大学 COD 战队开源的高保真模型（RoboMaster 2026 平衡车，串联腿闭链结构），
编写的最简 MuJoCo 仿真程序。

## 文件结构

```
balance_sim/
├── assets/                        # 官方原版模型资产（未改动）
│   ├── COD-2026RoboMaster-Balance.xml   # 官方 MJCF（作者导出）
│   └── *.STL                       # 16 个零件网格
├── balance_sim.xml                # 仿真用模型（官方基础上适配，见下）
├── simulate_balance.py            # 仿真主程序
└── README.md
```

开源仓库（单独下载保存于 `../balance_sim_repo/`，保持原样，可随时 `git pull` 更新）：
https://github.com/GrassFanWang/COD-2026RoboMaster-Balance-Simulation_File

## 模型结构

- **base_link 固定于世界系**（官方模型无 freejoint）——这是一个腿部机构测试台
- **14 个转动关节**：左右各 4 个前腿关节 + 3 个后腿关节 + 1 个轮轴
- **串联腿闭链**：用 `<equality><connect>` 约束把前腿与后腿的 2 组 site 锁定重合，
  前后腿构成 1 自由度的闭环机构——这就是"串联腿闭链"
- **6 个执行器**：4 个大腿关节（front/rear）+ 2 个轮子；小腿关节无执行器，
  完全由闭链约束联动

## 仿真内容

只驱动**前大腿**做正弦摆动（PD 位置伺服），后大腿与全部小腿**不驱动**，
观察闭链如何自动带动整条腿联动；轮子被腿的运动带动自由转动。

运行（使用你的专用环境）：

```bash
# 实时可视化（窗口打开 8 秒后自动关闭）
D:/pyenvs/robotics/Scripts/python.exe simulate_balance.py

# 无窗口快速测试（跑 4 秒打印状态）
D:/pyenvs/robotics/Scripts/python.exe simulate_balance.py --headless
```

## 对官方模型做的适配（仅 balance_sim.xml）

官方原版加载后无法直接仿真，需要 3 处适配（注释均在 XML 中）：

1. **加地面 + 车轮 conaffinity="1"**：官方 `<default>` 是 `conaffinity="0"`，
   轮子会直接穿过地面
2. **放宽 front/rear 大腿关节限位**（±1 → ±3 / ±1.5）：官方 keyframe 的 qpos=0
   并不在闭链约束流形上（初始残差约 4mm），闭链自然收敛要求 front≈±2.46 rad，
   官方限位与闭环几何不匹配，仿真中关节会被约束拉穿限位
3. **keyframe 换成闭链收敛得到的闭合位形**，避免初始冲击

## 运行结果参考（headless，前腿 ±0.2 rad @ 0.5 Hz）

```
t= 0.50s  前腿 L/R=+2.677/-2.683  后腿 L/R=-0.946/+0.945 (闭链带动)  闭链残差 左=0.5mm 右=0.1mm
t= 1.50s  前腿 L/R=+2.359/-2.410  后腿 L/R=-1.405/+1.345 (闭链带动)  闭链残差 左=0.5mm 右=0.1mm
```

前腿正弦摆动，后腿反相联动（幅度约前腿一半），闭链残差稳定在亚毫米级。
