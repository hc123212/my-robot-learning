import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# solve_ivp 要求函数签名必须是 fun(t, y, *args)，所以即使没用到 t，也必须写在参数第一位
def spring_damper(t, y, zeta):
    """
    t: 时间（虽然没直接用，但 solve_ivp 必须传这个参数）
    y: 数组 [位置 x, 速度 v]
    zeta: 阻尼比（由外部传入）
    """
    omega_n = 1.0 # m = 1, k = 1, 所以 ω_n = sqrt(k/m) = 1
    x, v = y
    dxdt = v
    dvdt = -2.0 * zeta *omega_n * v - omega_n**2 * x
    return [dxdt, dvdt]

def analytic_solution(zeta, t):
    """
    返回对应阻尼比 zeta 在时刻 t 的解析位移 x(t)
    """
    omega_n = 1.0
    x0, v0 = 1.0, 0.0  # 初始条件

    # ---------- 情况1：过阻尼 (zeta > 1) ----------
    if zeta > 1.0:
        sqrt_term = np.sqrt(zeta**2 - 1)   # 计算 sqrt(zeta^2 - 1)
        s1 = (-zeta + sqrt_term) * omega_n # 计算特征根 s1
        s2 = (-zeta - sqrt_term) * omega_n # 计算特征根 s2
        # 由初始条件确定的系数
        C1 = (v0 - s2 * x0) / (s1 - s2)
        C2 = (s1 * x0 - v0) / (s1 - s2)
        return C1 * np.exp(s1 * t) + C2 * np.exp(s2 * t)

    # ---------- 情况2：临界阻尼 (zeta ≈ 1) ----------
    # 浮点数不能直接用 == 比较：zeta 若来自计算（如 np.sqrt(4)-1），
    # 可能是 1.0000000000000002，会误入欠阻尼分支。用 np.isclose 才稳妥。
    elif np.isclose(zeta, 1.0):
        s = -omega_n # 临界阻尼下两个特征根为重根
        C1 = x0
        C2 = v0 - s * x0
        return (C1 + C2 * t) * np.exp(s * t)

    # ---------- 情况3：欠阻尼 (0 <= zeta < 1) ----------
    else:
        alpha = -zeta * omega_n # 衰减系数中的实部
        omega_d = omega_n * np.sqrt(1 - zeta**2) # 阻尼振荡频率
        A = x0
        B = (v0 - alpha * x0) / omega_d
        return np.exp(alpha * t) * (A * np.cos(omega_d * t) + B * np.sin(omega_d * t))

# 设置时间网格
t_end = 20.0                          # 仿真总时长（要改真时间只动这里）
t_span = (0, t_end)                   # 积分区间（起点，终点）
t_eval = np.linspace(0, t_end, 2000)  # 输出采样点（用于画图的横坐标）

y0 = [1.0, 0.0]                 # 初始条件向量
zeta_list = [0.2, 1.0, 2.0]     # 我们要跑的三种工况

results = {}                    # 空字典，用来存每组数据

for zeta in zeta_list:
    # 调用数值求解器
    sol = solve_ivp(
        spring_damper,   # 你刚定义的函数名
        t_span,          # 积分起止范围
        y0,              # 初始状态
        args=(zeta,),    # 额外传给 spring_damper 的参数（注意逗号）
        t_eval=t_eval    # 指定在这些时间点输出结果
    )
    # 积分失败时 sol.y 不完整，拿残缺数据去对比/画图会报出莫名其妙的错。
    # 养成习惯：任何求解器调用后都要检查 success。
    if not sol.success:
        raise RuntimeError(f"zeta = {zeta} 时积分失败: {sol.message}")

    # sol 是一个对象，sol.y 是形状 (2, 2000) 的数组，第一行是位置
    x_numeric = sol.y[0]
    
    # 调用解析函数
    x_analytic = analytic_solution(zeta, t_eval)
    
    # 存入字典，key 是 zeta，value 是另一个包含三个数组的字典
    results[zeta] = {
        't': t_eval,
        'x_num': x_numeric,
        'x_ana': x_analytic
    }

for zeta in zeta_list:
    # 计算逐点误差的最大值（无穷范数）
    err = np.max(np.abs(results[zeta]['x_num'] - results[zeta]['x_ana']))
    print(f"zeta = {zeta}, 最大误差 = {err:.2e}")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

# 左图：欠阻尼工况下，数值解 vs 解析解
zeta_show = zeta_list[0]    # 取第一种工况（0.2）做对比，避免硬编码
ax1.plot(results[zeta_show]['t'], results[zeta_show]['x_num'], 'b--', label='Numerical', linewidth=2) # 蓝色虚线
ax1.plot(results[zeta_show]['t'], results[zeta_show]['x_ana'], 'r-', label='Analytical', alpha=0.7)   # 红色实线（半透明）
ax1.set_title(f'Numerical vs Analytical (ζ = {zeta_show})')
ax1.set_xlabel('t [s]')
ax1.set_ylabel('x(t)')

# 右图：三种阻尼比的响应对比
colors = ['green', 'red', 'blue']
for zeta, color in zip(zeta_list, colors):
    ax2.plot(results[zeta]['t'], results[zeta]['x_num'], color=color, label=f'ζ = {zeta}')
ax2.set_title('Response for different damping ratios')
ax2.set_xlabel('t [s]')
ax2.set_ylabel('x(t)')

ax1.legend()          # 显示左上角图例
ax2.legend()          # 显示右上角图例
plt.tight_layout()    # 自动调整子图间距，避免标签重叠
plt.show()            # 弹出窗口显示图形（没有这行就不会显示！）