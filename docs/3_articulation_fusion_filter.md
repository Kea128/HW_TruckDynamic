# 铰接角时延融合滤波器：技术说明与移植指南

## 0. 这份文档是什么

卡车只有一个激光雷达能观测铰接角 \(\phi\)，它约 10 Hz、带 100–400 ms 的可变时延；
而横向 MPC 每 50 ms 需要**当前**的 \(\phi\) 和 \(\dot\phi\)。本文给出解决这个问题的
滤波器：状态、过程模型、量测模型、时延处理算法、参数、验收方法。

**读者**：要把这套滤波器移植到另一个代码仓的人（或 AI）。因此本文：

- 每个公式都从上一步推出来，不跳步；
- 每个算法都给出可直接实现的伪代码和数据结构；
- 每个参数都给出单位、量级来源和默认值；
- 第 11 章是分步移植清单，每步都有独立的验收条件。

**几何与车辆动力学**以
[`1_truck_model_key_formula.md`](1_truck_model_key_formula.md) 为准，
其适用边界见该文 1.4 节。本文引用的 (K…) 编号都来自那里。

**单位约定**：全文 SI。角度 rad，角速度 rad/s，速度 m/s，时间 s。
界面上显示的度只是换算，配置里一律存 rad。

### 0.1 一分钟概览

| 项 | 内容 |
|---|---|
| 状态（默认） | \(x=[\phi,\;b_{r2}]^\top\)，2 维 |
| 过程模型（默认） | 铰接运动学 (K5)，只需轴距 |
| 量测 | \(z=\phi+v\)，10 Hz，时延 100–400 ms |
| 时延处理 | 回退到扫描时刻的历史帧，更新，再用缓存输入重新前推 |
| 前提 | 扫描时间戳单调不减（代码会校验，违反则拒绝并计数） |
| 可选 | 降阶 (K27) 动态模型，3 维，\(\dot\phi\) 更准但依赖载荷参数 |

实测（本仓库默认传感器，0.5° 噪声、0.1–0.3 s 时延）：

| 量 | 运动学 K5 | 动力学 K27r | 原始迟到雷达 |
|---|---|---|---|
| \(\phi\) RMSE | 0.55° | **0.30°** | 1.19° |
| \(\dot\phi\) RMSE | 1.50°/s | **0.60°/s** | 观测不到 |

---

## 1. 问题定义

### 1.1 传感器

| 量 | 来源 | 频率 | 时延 | 在滤波器中的角色 |
|---|---|---|---|---|
| \(r_1\) | 拖头 IMU 横摆角速度 | ≥20 Hz | 可忽略 | 过程模型输入 |
| \(U\) | 车速 | ≥20 Hz | 可忽略 | 过程模型输入 |
| \(\delta\) | 前轮转角 | ≥20 Hz | 可忽略 | 动态模型输入；运动学模型仅用于诊断 |
| \(\phi_{\text{lidar}}\) | 感知雷达 | ≈10 Hz | **100–400 ms，时变** | 唯一直接量测 |
| \(r_2\) | — | 无 | — | 由模型重建 |
| 铰接编码器 | — | 无 | — | 无 |

**关键约束**：没有挂车 IMU，没有铰接编码器。\(r_2\) 和 \(\dot\phi\) 都**不可直接测量**，
只能由模型算出来。任何滤波器都不能凭空增加信息，这一点决定了后面所有取舍。

### 1.2 时间戳语义

雷达输出的是**扫描时刻** \(t_s\) 的 \(\phi(t_s)\)，在 \(t_s+\tau\) 才送达。

\[
\boxed{\text{stamp} = t_s,\quad\text{不是到达时刻。}}
\]

\(\dot\phi=10°/\text{s}\) 时，300 ms 的滞后就是 3° 的误差。把迟到的
\(\phi_{\text{lidar}}\) 直接当成当前值写进控制器是错的。

雷达工控机与车端必须时间同步（PTP/GNSS）。戳错 100 ms 等于把更新打到错误的历史时刻。

### 1.3 输出接口

MPC 的误差状态（式 (K35)）是
\(x_c=[e_y,\dot e_y,e_\psi,\dot e_\psi,\phi,\dot\phi]^\top\)。
滤波器负责填最后两维：

\[
x_c[4]=\hat\phi,\qquad x_c[5]=\hat{\dot\phi}.
\]

底盘/Plant 的积分**不要**用估计值，只有控制器用。

---

## 2. 符号

沿用 (K1)：

\[
L_1=a_1+b_1,\qquad
L_2=a_2+b_2,\qquad
\ell_h=b_1-d_1 .
\]

- \(L_1\)：卡车轴距。
- \(L_2\)：铰接点到挂车轴的距离。
- \(\ell_h\)：卡车后轴到铰接点的**有向**距离。铰接点在后轴上时 \(\ell_h=0\)，
  本仓库默认 \(d_1=b_1\) 即属此情形。

\[
\phi=\theta_1-\theta_2,\qquad
r_1=\dot\theta_1,\qquad
r_2=\dot\theta_2,\qquad
\dot\phi=r_1-r_2 .
\]

\(\ell_h\) 是本章符号最容易出错的地方，第 3 章逐步给出投影。

---

## 3. 过程模型一：铰接运动学 (K5)

默认过程模型不使用轮胎力。理由：轮胎侧偏刚度 \(C_\alpha\)、挂车质量 \(m_2\)、
转动惯量 \(I_2\) 都随载重变化，而 (K5) 只需要轴距 \(L_2\) 和 \(\ell_h\)。
代价见第 4 章。

### 3.1 铰接点在卡车体轴下的速度

纯滚动、无侧偏时，卡车后轴中心速度沿车体前向，大小 \(U\)；卡车以 \(r_1\) 横摆。
铰接点相对后轴沿车体前向偏置 \(\ell_h\)，由刚体速度关系：

\[
\boxed{
\begin{bmatrix} v_{P_h,x1}\\ v_{P_h,y1} \end{bmatrix}
=
\begin{bmatrix} U \\ \ell_h r_1 \end{bmatrix}.
}
\tag{F1}
\]

这就是式 (K4)。

### 3.2 转到挂车体轴

挂车标架相对卡车标架转过 \(\theta_2-\theta_1=-\phi\)。
把一个向量的分量从"当前标架"换算到"转过 \(\psi\) 的新标架"，用 \(R(-\psi)\)。
这里 \(\psi=-\phi\)，故用 \(R(\phi)\)：

\[
\begin{bmatrix} v_{P_h,x2}\\ v_{P_h,y2} \end{bmatrix}
=
\begin{bmatrix}\cos\phi&-\sin\phi\\[2pt] \sin\phi&\cos\phi\end{bmatrix}
\begin{bmatrix} U \\ \ell_h r_1 \end{bmatrix},
\]

展开：

\[
\boxed{
\begin{aligned}
v_{P_h,x2}&= U\cos\phi-\ell_h r_1\sin\phi,\\
v_{P_h,y2}&= U\sin\phi+\ell_h r_1\cos\phi.
\end{aligned}}
\tag{F2}
\]

**独立核对**（推荐移植时照做一遍）。用单位向量点积，由 (D25)(D26)：

\[
\boldsymbol e_{y2}^{T}\boldsymbol e_{x1}=\sin\phi,
\qquad
\boldsymbol e_{y2}^{T}\boldsymbol e_{y1}=\cos\phi,
\]

所以

\[
v_{P_h,y2}
=\boldsymbol e_{y2}^{T}\left(U\boldsymbol e_{x1}+\ell_h r_1\boldsymbol e_{y1}\right)
=U\sin\phi+\ell_h r_1\cos\phi,
\]

与 (F2) 第二式一致。

### 3.3 挂车轴纯滚动约束

铰接点位于挂车轴前方 \(L_2\)：

\[
\boldsymbol p_{P_h}=\boldsymbol p_{R_2}+L_2\boldsymbol e_{x2}.
\]

对时间求导，用 \(\dot{\boldsymbol e}_{x2}=r_2\boldsymbol e_{y2}\)：

\[
\boldsymbol v_{P_h}=\boldsymbol v_{R_2}+L_2r_2\boldsymbol e_{y2}.
\]

左乘 \(\boldsymbol e_{y2}^{T}\)。纯滚动意味着挂车轴无横向速度，
\(\boldsymbol e_{y2}^{T}\boldsymbol v_{R_2}=0\)，于是

\[
\boxed{v_{P_h,y2}=+L_2r_2 .}
\tag{F3}
\]

**符号提示**：这里是 \(+L_2r_2\)。

### 3.4 合并

(F2) 第二式与 (F3) 联立：

\[
\boxed{
r_{2,\text{kin}}(\phi,U,r_1)
=\frac{U\sin\phi+\ell_h r_1\cos\phi}{L_2}
}
\tag{F4}
\]

\[
\boxed{
\dot\phi=r_1-r_{2,\text{kin}}
=r_1-\frac{U\sin\phi+\ell_h r_1\cos\phi}{L_2}
}
\tag{F5}
\]

与 (K5) 的 \(\dot\theta_2\)、\(\dot\phi\) 两行完全一致。
\(\ell_h=0\) 时退化为 \(\dot\phi=r_1-(U/L_2)\sin\phi\)，
再代入 \(r_1=(U/L_1)\tan\delta\) 即 (K6)。

**不要用自行车公式代替 IMU 的 \(r_1\)**。轮胎侧偏时 IMU 测到的 \(r_1\) 才是真正的
\(\dot\theta_1\)；\(\delta\) 只用来算诊断量

\[
\tilde r_1=r_1-\frac{U}{L_1}\tan\delta
\]

（代码字段 `truckYawResidual`），它不进过程模型。

代码：`kinematicTrailerYawRate()` 实现 (F4)。

---

## 4. \(b_{r2}\)：残差状态的物理含义

(F4) 假设挂车轴纯滚动。真实挂车会侧偏，所以 \(r_2\neq r_{2,\text{kin}}\)。
定义残差状态

\[
b_{r2}\;\equiv\;r_2-r_{2,\text{kin}} .
\]

### 4.1 闭式来源

放开纯滚动假设，令挂车轴横向速度为 \(v_{2r}=v_{y2}-b_2r_2\)。
此时 (F3) 变成 \(v_{P_h,y2}=v_{2r}+L_2r_2\)，代入 (F2)：

\[
r_2=r_{2,\text{kin}}-\frac{v_{2r}}{L_2}
\quad\Longrightarrow\quad
b_{r2}=-\frac{v_{2r}}{L_2}.
\]

由 (K7) 的挂车轴侧偏角 \(\alpha_{2r}=-v_{2r}/U\) 及 \(F_{2r}=C_{2r}\alpha_{2r}\)：

\[
\boxed{
b_{r2}=\frac{U\alpha_{2r}}{L_2}=\frac{U\,F_{2r}}{L_2\,C_{2r}} .
}
\tag{F6}
\]

**\(b_{r2}\) 就是挂车轴侧偏角折算成的横摆角速度。** 三个推论：

1. 它随侧向力变化，不是常值。稳态过弯时正比于 \(U^2\rho\)。
2. 它有确定的动态（下节）。
3. 稳态下 \(b_{r2}=k_\beta(U)\,r_2\)，\(k_\beta=m_2a_2U^2/(L_2^2C_{2r})\)。
   本仓库参数、\(U=12\) 时 \(k_\beta=0.42\)。

### 4.2 它的时间常数

消去铰接力 \(H\) 后，挂车子系统给出
\(a_2m_2(\dot v_{y2}+Ur_2)+I_2\dot r_2=L_2F_{2r}\)。代入 (F6) 整理得

\[
\dot b_{r2}=-\lambda(U)\,b_{r2}+\frac{U}{L_2}r_2
+\frac{a_2b_2m_2-I_2}{a_2m_2L_2}\dot r_2,
\qquad
\boxed{\lambda(U)=\frac{C_{2r}L_2}{a_2m_2U}.}
\tag{F7}
\]

本仓库参数：\(\lambda(15)=3.24\,\text{s}^{-1}\)，即 \(\tau=0.31\,\text{s}\)；
\(U=25\) 时 \(\tau=0.51\,\text{s}\)。**时间常数随车速增大。**

### 4.3 因此默认模型的已知短板

\(\tau\approx0.3\text{–}0.5\,\text{s}\) 远长于 50 ms 控制周期。
把 \(b_{r2}\) 建成随机游走**跟不上**这段动态，代价主要落在 \(\dot\phi\) 上
（实测 1.50°/s，对比动态模型 0.60°/s）。

(F7) 含 \(\dot r_2\)，实现它需要对噪声很大的 \(r_1\) 求导，所以本方案**不**直接实现
(F7)，而是提供第 9 章的降阶 (K27) 模型作为可选替代 —— 它用 \(v_{y1}\) 作状态，
避免了输入微分。

---

## 5. 状态设计与可观性

### 5.1 状态与量测

\[
x=\begin{bmatrix}\phi\\ b_{r2}\end{bmatrix},
\qquad
z=\phi+v,\quad v\sim\mathcal N(0,R),
\qquad
H=\begin{bmatrix}1&0\end{bmatrix}.
\]

### 5.2 连续线性化

由 (F5)，\(\dot\phi=r_1-r_{2,\text{kin}}(\phi)-b_{r2}\)。令

\[
a\;\equiv\;\frac{\partial r_{2,\text{kin}}}{\partial\phi}
=\frac{U\cos\phi-\ell_h r_1\sin\phi}{L_2}.
\tag{F8}
\]

则

\[
\boxed{
A=\frac{\partial\dot x}{\partial x}
=\begin{bmatrix}-a&-1\\[2pt] 0&0\end{bmatrix}.
}
\tag{F9}
\]

代码：`KinematicArticulationModel::continuousJacobian()`。

### 5.3 可观性

\[
\mathcal O=\begin{bmatrix}H\\ HA\end{bmatrix}
=\begin{bmatrix}1&0\\ -a&-1\end{bmatrix},
\qquad
\boxed{\det\mathcal O=-1 .}
\tag{F10}
\]

行列式恒为 \(-1\)，**与车速、与铰接角无关**。这一对状态在任何工作点都结构可观，
不需要工况激励。

> **设计说明：为什么没有雷达偏置状态。**
> 早期版本带第三个状态 \(b_\phi\)（雷达安装偏差），\(H=[1,0,1]\)。
> 那时 \(HA^2=-a\,HA\)，可观矩阵秩只有 2，不可观方向为 \([1,-a,-1]^\top\)：
> \(\delta\phi=\varepsilon,\ \delta b_{r2}=-a\varepsilon,\ \delta b_\phi=-\varepsilon\)
> 在一阶上既不改变输出也不改变动态。它只能靠极强先验钉住，实测对精度无贡献。
> 因此移除。若实车确有安装偏差，**离线标定后从量测中减掉**，不要建成状态。

---

## 6. 噪声契约与离散化

### 6.1 唯一契约：连续功率谱密度

**所有过程噪声参数都是连续时间 PSD，不是每拍方差。**
这样把步长减半不会改变建模的噪声总量，分段（第 7 章的区间切分）才自洽。

| 符号 | 代码字段 | 单位 | 默认值 |
|---|---|---|---|
| \(q_\phi\) | `noiseDensity.articulationRate` | \(\text{rad}^2/\text{s}\) | \(2.742\times10^{-3}\) |
| \(q_b\) | `noiseDensity.trailerYawBias` | \(\text{rad}^2/\text{s}^3\) | \(2\times10^{-4}\) |
| \(\sigma_{r1}^2\) | `noiseDensity.truckYawRate` | \(\text{rad}^2/\text{s}\) | \(2.742\times10^{-5}\) |
| \(\sigma_U^2\) | `noiseDensity.speed` | \(\text{m}^2/\text{s}\) | \(0.04\) |

**量纲自检**（移植时务必做）：\(Q_{\phi\phi}\) 必须是 \(\text{rad}^2\)，
它由 \(q_\phi\) 对时间积分得到，故 \([q_\phi]=\text{rad}^2/\text{s}\)。
若把它当成"角速度方差"（\(\text{rad}^2/\text{s}^2\)）会差一个时间量纲。

### 6.2 输入噪声如何进入

\(r_1\) 和 \(U\) 的测量噪声通过 (F5) 进入 \(\dot\phi\)：

\[
\frac{\partial\dot\phi}{\partial r_1}=1-\frac{\ell_h\cos\phi}{L_2},
\qquad
\frac{\partial\dot\phi}{\partial U}=-\frac{\sin\phi}{L_2}.
\tag{F11}
\]

连续过程噪声矩阵：

\[
\boxed{
Q_c=\operatorname{diag}\!\left(
\eta\,q_\phi
+\Big(\tfrac{\partial\dot\phi}{\partial r_1}\Big)^{2}\sigma_{r1}^2
+\Big(\tfrac{\partial\dot\phi}{\partial U}\Big)^{2}\sigma_U^2,
\;\;
q_b
\right)
}
\tag{F12}
\]

\(\eta\) 是开环膨胀倍率（第 8 章），正常时为 1。

### 6.3 Van Loan 离散

对 \(\dot x=Ax+w\)、\(w\) 的 PSD 为 \(Q_c\)，构造 \(2n\times2n\) 矩阵

\[
M=\begin{bmatrix}-A & Q_c\\ 0 & A^{\top}\end{bmatrix}\Delta t,
\qquad
e^{M}=\begin{bmatrix}M_{11}&M_{12}\\ 0&M_{22}\end{bmatrix},
\]

则

\[
\boxed{
\Phi=M_{22}^{\top},
\qquad
Q_d=\Phi\,M_{12}.
}
\tag{F13}
\]

代码：`discretizeVanLoan()`，内部复用 `matrixExponential()`（缩放平方 + Taylor）。

**为什么不能只填对角。** \(b_{r2}\) 通过 \(\dot\phi=\cdots-b_{r2}\) 直接驱动 \(\phi\)。
把 (F12) 代入 (F13)，小步长展开得

\[
Q_d\simeq
\begin{bmatrix}
q_\phi T+\tfrac13 q_b T^{3} & -\tfrac12 q_b T^{2}\\[3pt]
-\tfrac12 q_b T^{2} & q_b T
\end{bmatrix}.
\tag{F14}
\]

只填 \(\operatorname{diag}(q_\phi T,\;q_b T)\) 会丢掉 \(T^3\) 项和交叉项。
单元测试 `testVanLoanMatchesAnalyticRandomWalk` 逐元素比对 (F14)。

### 6.4 状态转移与均值

- **均值**：用 (F5) 做一步欧拉，\(\phi^{+}=\text{wrap}(\phi+\Delta t(r_1-r_2))\)，
  \(b_{r2}^{+}=b_{r2}\)。
- **转移矩阵**：取该欧拉映射的精确雅可比 \(\Phi=I+A\Delta t\)，与均值自洽。
- **过程噪声**：取 (F13) 的 \(Q_d\)。

单元测试 `testKinematicJacobianMatchesFiniteDifference` 用中心差分核对 \(\Phi\)。

### 6.5 量测更新

\[
\hat z=\text{wrap}(\phi),\qquad
y=\text{wrap}(z-\hat z),\qquad
S=HPH^{\top}+R=P_{00}+R .
\]

\(S\le0\) 或非有限视为实现错误，抛异常。增益 \(K=PH^{\top}S^{-1}\)，更新

\[
x\leftarrow x+Ky,\quad \phi\leftarrow\text{wrap}(\phi),
\]
\[
\boxed{P\leftarrow(I-KH)P(I-KH)^{\top}+KRK^{\top}.}
\tag{F15}
\]

Joseph 形式在 \(K\) 与 \(P\) 略有数值不一致时仍保持对称**半**正定
（不是正定，不要过度声称）。实现另做一次显式对称化。

`wrap` 用 `std::remainder` 折到 \((-\pi,\pi]\)，上界闭合保证表示唯一；
双 `while` 循环会让 \(\pm\pi\) 都可达。

### 6.6 马氏门限

\[
d=\frac{y^2}{S}\sim\chi^2_1 .
\]

\(d>\) `mahalanobisGate`（默认 9，约 \(3\sigma\)）则拒绝，不改状态。
理想误拒率约 **0.27%**，10 Hz 下平均 37 秒一次 —— 这不是"几乎不会"，
复盘时应按此预期核对。

---

## 7. 时延处理

### 7.1 前提：扫描戳单调不减

本算法的正确性建立在一个前提上：

\[
\boxed{\text{后到达的扫描，其 stamp 不早于已融合过的任何 stamp。}}
\]

**为什么这个前提让算法变简单。** 当 \(t_s\) 的扫描到达时，所有已融合的扫描戳都
\(\le t_s\)。于是从 \(t_s\) 向前重推时，后面**不存在**任何已融合的校正会被覆盖，
只重推输入就够了。若扫描可能乱序，前向过程还必须重新应用那些更晚的量测，
需要完整的事件日志。

单雷达、单感知管线、FIFO 传输通常满足该前提。

**不要默认它成立，要校验。** 实现对 stamp \(\le\) 上一次已处理 stamp 的报文直接拒绝，
并给出独立的结局码 `outOfOrder` / `duplicate`。这样一旦管线真的乱序，
会表现为一个可见的计数器，而不是被静默损坏的状态。

### 7.2 数据结构

```
Frame:
    time         : double            # 该帧对应的时刻
    inputs       : {r1, U, delta}    # 支配"上一帧 -> 本帧"区间的输入
    state        : Vector<n>         # 处理完本帧后的后验
    covariance   : Matrix<n,n>

DelayedEkf:
    frames               : deque<Frame>   # 按时间升序
    lastAcceptedStamp    : double         # 最近一次被接受的扫描戳
    lastMeasurementStamp : double         # 最近一次处理过的扫描戳（含被拒的）
    consecutiveRejects   : int
```

区间约定沿用**右端点 ZOH**：区间 \([t_k,t_{k+1}]\) 使用 \(u_{k+1}\)。

### 7.3 predict：每个 IMU/控制拍

```
predict(u, t):
    if t <= frames.back().time:          # 不倒退时间，只刷新输入
        frames.back().inputs = u
        return
    next.time  = t
    next.inputs = u
    next.state, next.covariance =
        propagate(frames.back().state, frames.back().covariance,
                  u, t - frames.back().time, noiseScale(frames.back().time))
    frames.push_back(next)
    trim()                                # 丢弃超出 historyHorizon 的最老帧
```

复杂度 O(1)。

### 7.4 update：雷达报文到达

```
update(stamp, z):
    # --- 前置检查，顺序不可调换 ---
    if not initialized:              return notInitialized
    if stamp or z not finite:        return nonFinite
    if stamp == lastMeasurementStamp: return duplicate
    if stamp <  lastMeasurementStamp: rejects++; return outOfOrder
    if stamp <  frames.front().time:  rejects++; return staleBeyondWindow
    if stamp >  frames.back().time:   return aheadOfInputs
    lastMeasurementStamp = stamp

    # --- 定位到扫描时刻 ---
    i = locateFrame(stamp)           # 见 7.5，默认会在 stamp 处切分区间

    # --- 在该帧上做 Joseph 更新 ---
    outcome = applyMeasurement(frames[i], z)
    if outcome != accepted: return outcome

    # --- 用缓存输入把修正推回当前 ---
    for k = i .. frames.size()-2:
        dt = frames[k+1].time - frames[k].time
        frames[k+1].state, frames[k+1].covariance =
            propagate(frames[k].state, frames[k].covariance,
                      frames[k+1].inputs, dt, 1.0)
    return accepted
```

重推时噪声倍率固定为 1.0：轨迹已被校正，不再是开环，不应再叠加开环膨胀。

各结局码的含义：

| 结局 | 触发条件 | 处理 |
|---|---|---|
| `duplicate` | stamp 与上一条相同 | 拒绝，不计入连续拒绝 |
| `outOfOrder` | stamp 早于上一条 | 拒绝并计数；**前提被违反的信号** |
| `staleBeyondWindow` | stamp 早于最老帧 | 拒绝并计数；`historyHorizon` 不足的唯一症状 |
| `aheadOfInputs` | stamp 晚于最新帧 | 拒绝。外推最新输入会凭空造信息，应先调 `predict` |
| `gated` | \(d>\) 门限 | 拒绝并计数，不改状态 |
| `accepted` | 其余 | Joseph 更新 + 重推 |

### 7.5 精确切分

扫描戳一般落在两帧之间。默认做法是在 \(t_s\) 处**插入一帧**，把区间切成
\([t_k,t_s]\) 与 \([t_s,t_{k+1}]\)，**两段都沿用原区间的 \(u_{k+1}\)**。
这样切分不改变输入语义，对齐误差为零。

```
locateFrame(stamp):
    找到第一个 frames[j].time > stamp
    若 stamp 与 frames[j-1].time 相同（容差内）: return j-1
    新建 inserted:
        time       = stamp
        inputs     = frames[j].inputs       # 沿用原区间的输入
        state/cov  = propagate(frames[j-1], inputs, stamp - frames[j-1].time)
    插入到位置 j
    return j
```

替代做法是吸附到最近的已有帧（`snapToNearestFrame`），这会引入

\[
e_t\approx-\dot\phi(t_s)\cdot\epsilon,
\qquad |\epsilon|\le\tfrac12\Delta t_{\text{ctrl}}
\]

的额外量测误差。\(\Delta t=50\) ms、\(\dot\phi=40°/\text{s}\) 时最坏约 1°：
对 4° 噪声可忽略，对 0.5° 级雷达不可忽略。默认用精确切分。

### 7.6 历史窗

\[
\boxed{\texttt{historyHorizon}\;\ge\;\tau_{\max}+\Delta t_{\text{ctrl}}}
\]

窗口短于最大时延时，迟到报文落在历史之外被丢弃。默认 0.55 s 覆盖 400 ms 工况。
本仓库在配置期强制校验 `lidarDelayMax <= historyHorizon`，不满足直接报错，
避免静默丢包。

---

## 8. 开环与链路健康：两个不同的量

\[
\boxed{
\text{informationAge}=t_{\text{now}}-t_{\text{lastAccepted}},
\qquad
\text{arrivalGap}=t_{\text{now}}-t_{\text{lastSuccessfulUpdate}} .
}
\]

- **informationAge**：当前估计背后最新融合信息的年龄。
  **过程噪声倍率由它决定**：超过 `lostTimeout`（默认 0.6 s）或连续拒绝达
  `consecutiveRejectLimit`（默认 3）时 \(\eta=4\)。
  400 ms 时延下它本来就接近 0.4 s，这不是误报 —— 估计确实在外推。
- **arrivalGap**：距上次成功更新的墙钟时间。**链路健康由它决定**，
  超过 `linkTimeout`（默认 0.5 s）置 `linkStalled`。

把两者混成一个标志会让长时延被误报成断链。`lostTimeout` 默认 0.6 s > 400 ms，
正常工况不触发。

MPC 目前不消费 \(P\)。**长时间开环没有自动降级，上层必须自行定义失效上限。**

---

## 9. 可选过程模型：以实测 \(r_1\) 为输入的降阶 (K27)

### 9.1 动机

第 4 章表明 \(b_{r2}\) 有 0.3–0.5 s 的真实动态，随机游走跟不上，代价落在 \(\dot\phi\)。
直接实现 (F7) 需要 \(\dot r_2\)；降阶 (K27) 用 \(v_{y1}\) 作状态，避免输入微分。

### 9.2 推导

(K27) 给出

\[
\dot{\boldsymbol x}_p=A_p\boldsymbol x_p+B_p\delta,
\qquad
\boldsymbol x_p=[v_{y1},\,r_1,\,r_2,\,\phi]^{\top}.
\]

\(r_1\) 由 IMU 实测，所以**删掉 \(\dot r_1\) 那一行**，把 \(r_1\) 移到输入侧。取

\[
x_r=\begin{bmatrix}v_{y1}\\ r_2\\ \phi\end{bmatrix},
\qquad
u_r=\begin{bmatrix}r_1\\ \delta\end{bmatrix}.
\]

保留 (K27) 的第 1、3、4 行，把 \(r_1\) 对应的列移到 \(B\)：

\[
\boxed{
\dot x_r=
\begin{bmatrix}
a_{11}&a_{13}&a_{14}\\
a_{31}&a_{33}&a_{34}\\
0&-1&0
\end{bmatrix}x_r
+
\begin{bmatrix}
a_{12}&\beta_1\\
a_{32}&\beta_3\\
1&0
\end{bmatrix}u_r
}
\tag{F16}
\]

第三行是 \(\dot\phi=r_1-r_2\)，恒等式而非近似。量测 \(H=[0,0,1]\)。输出

\[
\hat{\dot\phi}=r_1-\hat r_2
\]

直接来自状态。**不要对 \(\hat\phi\) 做数值差分。**

模型是线性的（车速调度后），故均值与 \(\Phi\) 都用增广矩阵指数做精确 ZOH。

### 9.3 可观性

对三个物理状态，\(H_r=[0,0,1]\)：

\[
\mathcal O_r=
\begin{bmatrix}
0&0&1\\
0&-1&0\\
-a_{31}&-a_{33}&-a_{34}
\end{bmatrix},
\qquad
\boxed{\det\mathcal O_r=-a_{31}.}
\tag{F17}
\]

本仓库名义参数 \(a_{31}=0.2875\)，结构可观。
但 \(v_{y1}\) 只能通过 \(\phi\) 的二阶动态被感知，10 Hz 高噪声量测下数值可观性较弱。

### 9.4 车速调度

\(a_{ij},\beta_i\) 是 \(U\) 的函数。实现按 `modelRefreshSpeedStep`（默认 0.25 m/s）
重建矩阵，低于 `minimumModelSpeed`（默认 0.5 m/s）时取下限，避免 (K27) 的
\(1/U\) 发散。

### 9.5 何时不要用

| 条件 | 原因 |
|---|---|
| 载重、\(I_2\)、\(C_{2r}\) 未知或变化大 | (F16) 每个系数都依赖它们 |
| 低速、倒车、强制动、大侧偏、大铰接角 | (K27) 的 \(1/U\)、恒速、线性轮胎假设失效 |
| 只能用同参数 (K27) 仿真验收 | 逆犯罪，得到虚假优势 |

**因此默认仍是运动学模型。** 动态模型通过 `processModel = dynamic` 显式开启，
建议先以对照模式并跑取证。测试
`testDynamicModelSurvivesParameterMismatch` 在 \(m_2,I_2,C_{2r}\) 各 ±30% 失配下
要求动态模型不劣于运动学模型。

---

## 10. 参数表

面板上的标准差是**度**，写入配置前换成 rad 再平方。

| 项 | 字段 | 默认 | 说明 |
|---|---|---|---|
| 过程模型 | `processModel` | `kinematic` | 动态模型见第 9 章 |
| 历史窗 | `historyHorizon` | 0.55 s | 必须 \(\ge\tau_{\max}\)，配置期强制 |
| \(R\) | `measurementVariance` | \(3.046\times10^{-4}\)（1°） | 等于雷达 \(\sigma^2\) |
| \(q_\phi\) | `noiseDensity.articulationRate` | \(2.742\times10^{-3}\) | \(\text{rad}^2/\text{s}\) |
| \(q_b\) | `noiseDensity.trailerYawBias` | \(2\times10^{-4}\) | \(\text{rad}^2/\text{s}^3\) |
| \(\sigma_{r1}^2\) | `noiseDensity.truckYawRate` | \(2.742\times10^{-5}\) | \(\text{rad}^2/\text{s}\) |
| \(\sigma_U^2\) | `noiseDensity.speed` | 0.04 | \(\text{m}^2/\text{s}\) |
| \(q_{v_{y1}}\) | `noiseDensity.truckLateralVelocity` | 0.05 | 动态模型；\(0.3\,\text{m/s}^2\)、\(\tau=0.2\,\text{s}\) |
| \(q_{r_2}\) | `noiseDensity.trailerYawRate` | \(1.5\times10^{-4}\) | 动态模型；\(1°/\text{s}^2\)、\(\tau=0.2\,\text{s}\) |
| 门限 | `mahalanobisGate` | 9 | \(R\) 对齐后不要再拧死 |
| 连续拒 | `consecutiveRejectLimit` | 3 | |
| 信息年龄阈值 | `lostTimeout` | 0.6 s | 决定 \(\eta\) |
| 链路阈值 | `linkTimeout` | 0.5 s | 仅诊断 |
| \(P_\phi(0)\) | `initialArticulationVariance` | \((2°)^2\) | |
| \(P_b(0)\) | `initialTrailerYawBiasVariance` | \((2°/\text{s})^2\) | |
| \(P_{v_{y1}}(0)\) | `initialTruckLateralVelocityVariance` | 0.25 | 动态模型 |
| \(P_{r_2}(0)\) | `initialTrailerYawRateVariance` | \((2°/\text{s})^2\) | 动态模型 |
| 调度步长 | `modelRefreshSpeedStep` | 0.25 m/s | 动态模型 |
| 最低建模车速 | `minimumModelSpeed` | 0.5 m/s | 防 \(1/U\) 发散 |

**\(R\) 必须与雷达噪声同量级。** 雷达 3°–4° 噪声却用 1° 的 \(R\)，会让马氏门限批量误拒。

几何参数必须与实车一致：\(a_2,b_2,b_1,d_1\)。\(L_2\) 错 10% 会系统性扭曲 (F4)。

---

## 11. 移植清单

按顺序做，每步都有独立验收条件，不要跳步。

### 步骤 1：确认前提

- [ ] 感知链路的扫描戳单调不减？若否，本方案的时延处理需要扩成事件重放（见 7.1）。
- [ ] 雷达给的是扫描时刻而非到达时刻？
- [ ] 车端与感知时间同步（PTP/GNSS）？
- [ ] 已知 \(\tau_{\max}\)？据此设 `historyHorizon = τ_max + Δt_ctrl`。

### 步骤 2：接上已有 EKF 核

目标仓若已有 EKF 类，**在它上面扩充**，不要另起卡尔曼核。需要的能力：

| 能力 | 若缺失 |
|---|---|
| 读写完整 \(x,P\) | 增加 `snapshot()` / `restore(x,P)`，历史回退要用 |
| 每步传入不同 \(Q_d\) | 预测接口允许本步传 \(Q\) |
| 角度残差 wrap | 在本量测的残差回调里 wrap，不要改其它滤波 |
| 状态维可配 | 设为 \(n=2\)（或 \(n=3\)） |

**快照必须覆盖该核的全部内部统计状态**，不只是 \(x,P\)。若平台用平方根因子或自适应
统计，仅存 \(x,P\) 无法完整恢复。

**验收**：能对该核做 snapshot → 若干步 predict → restore → 重放，得到逐位相同的结果。

### 步骤 3：实现过程模型

实现 (F4)(F5)(F9)(F12)。

**验收**：
- \(r_{2,\text{kin}}\) 在对轴（\(\ell_h=0\)）与偏轴（\(\ell_h\ne0\)）下都与 (K5) 的
  \(\dot\theta_2\) 一致；
- \(\Phi\) 与均值映射的中心差分一致，逐元素误差 \(<10^{-6}\)；
- 可观矩阵行列式为 \(-1\)。

### 步骤 4：实现噪声离散

实现 (F13)。

**验收**：
- 对 \(A=\begin{bmatrix}0&-1\\0&0\end{bmatrix}\)、\(Q_c=\operatorname{diag}(0,q)\)，
  \(Q_d\) 逐元素等于 (F14) 的解析值；
- 分段复合成立：\(Q_{[0,T]}=\Phi_2Q_{[0,T_1]}\Phi_2^{\top}+Q_{[T_1,T]}\)。

### 步骤 5：实现时延外壳

实现 7.2–7.6。

**验收**：
- 亚帧扫描戳（如 5/17/25/43 ms 相位）的 `alignedStamp` 等于 stamp 本身；
- 五种拒绝结局都能触发并正确上报；
- 窗口边界（\(\tau\) 略小于/略大于 `historyHorizon`）行为符合预期。

### 步骤 6：接线

- IMU/控制拍调 `predict`，用车端 \(r_1,U,\delta\)；
- 雷达回调把**扫描时刻**和 \(\phi\)（rad）送进 `update`；
- \(\hat\phi,\hat{\dot\phi}\) 写入控制器的铰接通道；
- 底盘/Plant 积分不要用估计值。

**验收**：固定 200 ms 时延、无噪声的仿真下，当前 \(\hat\phi\) 的 RMSE
\(<0.015\) rad，且明显优于原始迟到量测。

### 步骤 7：诊断与记录

记录逐包的 accepted / gated / dropped 计数（不是每拍一个布尔），
以及 informationAge、arrivalGap、\(\dot\phi\)。

**验收**：能从记录里算出接受率、马氏距离均值（应 \(O(1)\)）、
\(\phi\) 与 \(\dot\phi\) 各自的 RMSE 和幅值比。

### 步骤 8：是否启用动态模型

先以对照模式并跑，不要直接替换默认。判定门槛见 12.3。

### 不要做的事

- 不要忽略时延，把迟到 \(z\) 当当前量测；
- 不要把 \(\delta\) 代入 (K6) 代替 IMU 的 \(r_1\)；
- 不要在 \(R\) 未与雷达噪声对齐时拧死门限；
- 不要把雷达安装偏差建成在线状态（见 5.3 的设计说明）；
- 不要只用同参数的 (K27) 仿真验收动态模型。

---

## 12. 验收

### 12.1 仓库内已实现的用例

| 用例 | 断言 | 测试名 |
|---|---|---|
| Van Loan vs 解析 | 逐元素等于 (F14) | `testVanLoanMatchesAnalyticRandomWalk` |
| \(Q_d\) 分段复合 | \(Q=\Phi_2Q_1\Phi_2^\top+Q_2\) | `testProcessNoiseIsStepInvariant` |
| \(\Phi\) vs 中心差分 | \(<10^{-6}\) | `testKinematicJacobianMatchesFiniteDifference` |
| 可观性 | \(\det\mathcal O=-1\) | `testKinematicObservabilityRank` |
| 降阶 (K27) 可观 | 秩 3，\(a_{31}\) 非零 | `testDynamicObservabilityRank` |
| \(r_{2,\text{kin}}\) | 对轴/偏轴均与 (K5) 一致 | `testKinematicTrailerYawRateMatchesK5` |
| 亚帧对齐 | `alignedStamp == stamp` | `testSubFrameAlignmentIsExact` |
| 迟到扫描 | 各自在自己的戳上融合 | `testDelayedScanOrdering` |
| 乱序/重复 | 拒绝并上报，不改状态 | `testDelayedScanOrdering` |
| 边界结局 | stale / ahead / gated / accepted | `testMeasurementBoundaryHandling` |
| 固定 200 ms 时延 | RMSE \(<0.015\) rad 且优于原始量测 | `testDelayedEkfCompensatesLidarLatency` |
| 大野值 | 被门限拒绝 | `testLidarOutlierIsGated` |
| 长时间无雷达 | 进入开环 | `testEstimatorCoastsAfterDropout` |
| 动态模型改善 \(\dot\phi\) | RMSE \(<0.8\times\) 运动学 | `testDynamicModelImprovesRateTracking` |
| 参数失配 | ±30% 下动态模型不劣 | `testDynamicModelSurvivesParameterMismatch` |
| 优于原始迟到量测 | 两模型都 \(<0.5\times\) | `testBothModelsBeatDelayedMeasurement` |
| 闭环冷启动带噪 | 收敛 | `testColdStartWithNoisySensors` |
| 对照模型并跑 | 动态模型 \(\dot\phi\) 更好 | `testShadowEstimatorRunsInParallel` |
| 历史窗校验 | \(\tau_{\max}>\) 窗口被拒 | `testHistoryHorizonMustCoverLatency` |

### 12.2 仓库内实测

Demo 默认 S 形路径，雷达 0.5° 噪声、0.1–0.3 s 时延，统计区间 \(t\ge1\) s：

| 量 | 运动学 K5 | 动力学 K27r | 原始迟到雷达 |
|---|---|---|---|
| \(\phi\) RMSE | 0.55° | **0.30°** | 1.19° |
| \(\dot\phi\) RMSE | 1.50°/s | **0.60°/s** | 观测不到 |

铰接角跟踪实验（控制器闭环跟 \(\phi_{\text{ref}}\)，反馈用估计值）：

| 配置 | plant/ref 幅值比 | est/plant |
|---|---|---|
| 反馈用真值 | 0.992 | 1.000 |
| 反馈用 K5 估计 | **1.186** | 0.731 |
| 反馈用 K27r 估计 | 0.988 | 1.009 |

**解读**：反馈真值时跟踪准确，说明过冲不是跟踪误差。K5 估计只读到真值的 73%，
控制器把**估计**压到参考，真实角度就被抬高到参考的 1.19 倍
（\(0.867/0.731=1.186\)，算术闭合）。这是估计器幅值偏差被闭环放大，
**该工况不能用作融合验收**。

**注意**：本仓库的仿真 Plant 就是 (K27)，与动态模型同源，
所以上表对动态模型有利的部分属于逆犯罪，不能作为最终证据。

### 12.3 替换默认过程模型的门槛

| 类别 | 门槛 |
|---|---|
| 角度 | \(\phi\) RMSE 不劣于基线；幅值比 0.97–1.03 |
| 角速度 | \(\dot\phi\) RMSE 至少降低 20%；幅值比 0.90–1.10 |
| 相位 | 不超过 1 个控制拍 |
| 闭环 | \(e_y,e_\psi\) 不恶化；转角 RMS 与速率不恶化超 5% |
| 鲁棒 | 跨载荷/速度/温度的最差分位不恶化 |
| 实时 | 20 Hz 平台 p99 \(<5\) ms |
| 统计 | 多随机种子配对实验；有真值时检查 NEES 与覆盖率 |

真值来源：开发阶段临时加装铰接编码器、挂车 IMU 或光学基准；量产车不需要。

---

## 13. 已知限制

1. **默认过程模型是 (K5)，真实车辆有轮胎侧偏。** \(b_{r2}\) 的随机游走跟不上第 4 章的
   0.3–0.5 s 动态，\(\dot\phi\) 幅值系统性偏瘦。动态模型是针对性解法，但引入参数敏感性。
2. **\(\dot\phi\) 完全是模型输出。** 雷达观测不到它，任何滤波器都不能凭空增加信息。
   它的可信度等于过程模型的可信度。
3. **雷达安装偏差不在状态里。** 必须离线标定后从量测中减掉。未标定的偏差会经
   \(\partial\dot\phi/\partial\phi\approx U/L_2\) 放大：\(U=15,L_2=7\) 时，
   1° 的未建模偏差约产生 \(2.14°/\text{s}\) 的 \(\dot\phi\) 误差。
4. **输入噪声按连续 PSD 处理**，未建 sample-and-hold 的跨分段相关性。
5. **依赖扫描戳单调。** 违反会被拒绝并计数，但那些报文的信息就丢失了。
6. **\(v_{y1}\) 在动态模型中弱可观**，(F17) 虽满秩但条件数差。
7. **单雷达、单峰噪声。** 多峰或错误关联只靠马氏门限。
8. **时间同步是前提。** 本方案不估计时钟偏差。
9. **参考实现无并发保护。** 实车 IMU 与雷达回调必须串行化。
10. **MPC 不消费 \(P\)。** 长时间开环无自动降级。
