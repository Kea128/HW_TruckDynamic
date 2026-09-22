# 铰接角时延融合滤波器：技术说明与移植指南

## 0. 这份文档是什么

卡车只有一个激光雷达能观测铰接角 \(\phi\)，它约 10 Hz、带 100–400 ms 的可变时延；
而横向 MPC 每 50 ms 需要**当前**的 \(\phi\) 和 \(\dot\phi\)。本文给出解决这个问题的
滤波器。

滤波器只有**一套框架**（量测模型、噪声离散、时延处理三者共用），其中**过程模型有两个
可选实现**：运动学 (K5) 和降阶动力学 (K27r)。本文按这两个模型**对照**展开。

**读者**：要把这套滤波器移植到另一个代码仓的人（或 AI）。因此本文：

- 每个符号在第一次使用前都在参数表中定义，含单位与物理意义；
- 每个公式都从上一步推出来，不跳步；
- 两个过程模型的每个环节都并排给出，便于取舍；
- 每一节都标注对应的代码位置（第 11 章有完整对照表）；
- 第 13 章是分步移植清单，每步有独立验收条件。

**几何与车辆动力学**以
[`1_truck_model_key_formula.md`](1_truck_model_key_formula.md) 为准，
其适用边界见该文 1.4 节。本文引用的 (K…) 编号都来自那里。

**单位约定**：全文 SI。角度 rad，角速度 rad/s，速度 m/s，时间 s。
界面显示的度只是换算，配置里一律存 rad。

### 0.1 一分钟概览

| 项 | 运动学模型 | 动力学模型 |
|---|---|---|
| 代码枚举 | `ArticulationProcessModel::kinematic` | `::dynamic` |
| 状态维数 | 2 | 3 |
| 状态 | \([\phi,\;b_{r2}]^\top\) | \([v_{y1},\;r_2,\;\phi]^\top\) |
| 依赖参数 | 仅轴距 \(L_2,\ell_h\) | 质量、惯量、轮胎刚度 |
| 量测 | \(z=\phi+v\)，\(H=[1,\,0]\) | \(z=\phi+v\)，\(H=[0,\,0,\,1]\) |
| \(\phi\) RMSE | 0.56° | **0.30°** |
| \(\dot\phi\) RMSE | 1.53°/s | **0.60°/s** |
| 默认 | **是** | 否，需显式开启 |

共用部分：量测模型（第 6 章）、噪声离散（第 7 章）、时延处理（第 8 章）。

**关键结论先说**：两个模型都**没有**把 \(\dot\phi\) 建成观测量，因为没有任何传感器测
它（详见第 6.3 节）。\(\dot\phi\) 是模型输出，不是量测。

---

## 1. 问题定义

### 1.1 传感器清单

| 量 | 符号 | 来源 | 频率 | 时延 | 在滤波器中的角色 |
|---|---|---|---|---|---|
| 拖头横摆角速度 | \(r_1\) | 拖头 IMU | ≥20 Hz | 可忽略 | **输入**（两个模型都用） |
| 车速 | \(U\) | 轮速/组合导航 | ≥20 Hz | 可忽略 | **输入**（两个模型都用） |
| 前轮转角 | \(\delta\) | 转向系统 | ≥20 Hz | 可忽略 | 动力学模型的**输入**；运动学模型仅用于诊断 |
| 铰接角 | \(\phi_{\text{lidar}}\) | 感知雷达 | ≈10 Hz | **100–400 ms，时变** | **唯一量测** |
| 挂车横摆角速度 | \(r_2\) | — | 无传感器 | — | 由模型重建 |
| 铰接角速度 | \(\dot\phi\) | — | 无传感器 | — | 由模型重建 |

**关键约束**：没有挂车 IMU，没有铰接编码器。\(r_2\) 和 \(\dot\phi\) 都**不可直接测量**。
任何滤波器都不能凭空增加信息 —— 这一点决定了后面所有取舍，也是第 6.3 节的依据。

### 1.2 时间戳语义

雷达输出的是**扫描时刻** \(t_s\) 的 \(\phi(t_s)\)，在 \(t_s+\tau\) 才送达。

\[
\boxed{\text{stamp} = t_s,\quad\text{不是到达时刻}.}
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

## 2. 全局符号表

后续所有推导只使用本表加上各模型节内声明的附加符号。

### 2.1 几何参数（常量，来自 (K1)）

| 符号 | 含义 | 单位 | 代码字段 |
|---|---|---|---|
| \(a_1\) | 卡车质心到前轴距离 | m | `Parameters::a1` |
| \(b_1\) | 卡车质心到后轴距离 | m | `Parameters::b1` |
| \(d_1\) | 卡车质心到铰接点距离 | m | `Parameters::d1` |
| \(a_2\) | 挂车质心到铰接点距离 | m | `Parameters::a2` |
| \(b_2\) | 挂车质心到挂车轴距离 | m | `Parameters::b2` |
| \(L_1=a_1+b_1\) | 卡车轴距 | m | 派生 |
| \(L_2=a_2+b_2\) | 铰接点到挂车轴距离 | m | 派生 |
| \(\ell_h=b_1-d_1\) | 卡车后轴到铰接点的**有向**距离 | m | 派生 |

\(\ell_h\) 是本文最容易搞错符号的量：铰接点在后轴**后方**为正，落在后轴上时
\(\ell_h=0\)。本仓库默认 \(d_1=b_1\)，即 \(\ell_h=0\)。

### 2.2 运动学量

| 符号 | 含义 | 单位 |
|---|---|---|
| \(\theta_1,\theta_2\) | 卡车、挂车航向角 | rad |
| \(\phi=\theta_1-\theta_2\) | 铰接角 | rad |
| \(r_1=\dot\theta_1\) | 卡车横摆角速度 | rad/s |
| \(r_2=\dot\theta_2\) | 挂车横摆角速度 | rad/s |
| \(\dot\phi=r_1-r_2\) | 铰接角速度 | rad/s |
| \(U\) | 卡车纵向速度 | m/s |
| \(\delta\) | 前轮转角 | rad |
| \(v_{y1}\) | 卡车质心横向速度 | m/s |
| \(P_h\) | 铰接点 | — |
| \(R_2\) | 挂车轴中心 | — |
| \(\boldsymbol e_{x1},\boldsymbol e_{y1}\) | 卡车体轴单位向量 | — |
| \(\boldsymbol e_{x2},\boldsymbol e_{y2}\) | 挂车体轴单位向量 | — |

### 2.3 滤波量

| 符号 | 含义 | 单位 |
|---|---|---|
| \(x\) | 状态向量（各模型不同） | — |
| \(P\) | 状态协方差 | — |
| \(A\) | 连续时间雅可比 \(\partial\dot x/\partial x\) | 1/s |
| \(\Phi\) | 离散状态转移矩阵 | — |
| \(Q_c\) | 连续过程噪声功率谱密度 | 见 7.1 |
| \(Q_d\) | 离散过程噪声协方差 | — |
| \(H\) | 量测雅可比 | — |
| \(z\) | 雷达量测 | rad |
| \(R\) | 量测方差 | rad² |
| \(y\) | 量测残差（新息） | rad |
| \(S\) | 新息方差 | rad² |
| \(K\) | 卡尔曼增益 | — |
| \(d\) | 马氏距离平方 | — |
| \(\Delta t\) | 积分步长 | s |
| \(\tau\) | 雷达时延 | s |
| \(\eta\) | 开环噪声膨胀倍率 | — |

---

## 3. 两个过程模型的分工

先说清楚为什么有两个，再分别推导。

| 维度 | 运动学 (K5) | 动力学 (K27r) |
|---|---|---|
| 物理假设 | 轮胎纯滚动，无侧偏 | 线性轮胎，小角度，恒速 |
| 需要的车辆参数 | \(L_2,\ell_h\) | \(L_1,L_2,a_1,b_1,a_2,b_2,m_1,m_2,I_1,I_2,C_{1f},C_{1r},C_{2r}\) |
| 参数随载重变化 | 否 | **是**，质量/惯量/刚度都变 |
| 侧偏的处理 | 归入残差状态 \(b_{r2}\)，按随机游走 | 显式建模 |
| \(\dot\phi\) 精度 | 弱（1.53°/s） | 强（0.60°/s） |
| 失效工况 | 大侧偏、高速急弯 | 低速、倒车、大侧偏、参数失配 |
| 逆犯罪风险 | 低（与 Plant 不同源） | **高**（本仓库 Plant 就是 K27） |

**默认选运动学**，因为它只需要两个几何尺寸，不会因为载重变化而失准。
动力学模型作为可选项，在参数可信时能显著改善 \(\dot\phi\)。

---

## 4. 过程模型 A：运动学 (K5)

代码：`KinematicArticulationModel`（`src/articulation_estimator.cpp`）。

### 4.1 本节附加符号

| 符号 | 含义 | 单位 | 首次出现 |
|---|---|---|---|
| \(v_{P_h,x1},v_{P_h,y1}\) | 铰接点速度在**卡车**体轴的分量 | m/s | (F1) |
| \(v_{P_h,x2},v_{P_h,y2}\) | 铰接点速度在**挂车**体轴的分量 | m/s | (F2) |
| \(r_{2,\text{kin}}\) | 纯滚动假设下的挂车横摆角速度 | rad/s | (F4) |
| \(b_{r2}=r_2-r_{2,\text{kin}}\) | 挂车横摆角速度残差（状态） | rad/s | 4.5 |
| \(v_{2r}\) | 挂车轴横向速度 | m/s | (F6) |
| \(\alpha_{2r}\) | 挂车轴侧偏角 | rad | (F6) |
| \(C_{2r}\) | 挂车轴侧偏刚度 | N/rad | (F6) |
| \(F_{2r}\) | 挂车轴侧向力 | N | (F6) |
| \(a\) | \(\partial r_{2,\text{kin}}/\partial\phi\) | 1/s | (F9) |

### 4.2 第一步：铰接点在卡车体轴下的速度

纯滚动、无侧偏时，卡车后轴中心速度沿车体前向，大小 \(U\)；卡车以 \(r_1\) 横摆。
铰接点相对后轴沿车体前向偏置 \(\ell_h\)。由刚体速度关系
\(\boldsymbol v_{P}=\boldsymbol v_{O}+\boldsymbol\omega\times\boldsymbol r_{OP}\)，
其中 \(\boldsymbol\omega=r_1\boldsymbol e_z\)、\(\boldsymbol r_{OP}=\ell_h\boldsymbol e_{x1}\)，
而 \(\boldsymbol e_z\times\boldsymbol e_{x1}=\boldsymbol e_{y1}\)：

\[
\boxed{
\begin{bmatrix} v_{P_h,x1}\\ v_{P_h,y1}\end{bmatrix}
=\begin{bmatrix} U\\ \ell_h r_1\end{bmatrix}.}
\tag{F1}
\]

这就是式 (K4)。

### 4.3 第二步：转到挂车体轴

挂车标架相对卡车标架转过 \(\theta_2-\theta_1=-\phi\)。把同一个向量的分量从卡车标架
换算到挂车标架，要左乘 \(R(-(-\phi))=R(\phi)\)：

\[
\begin{bmatrix} v_{P_h,x2}\\ v_{P_h,y2}\end{bmatrix}
=\begin{bmatrix}\cos\phi&-\sin\phi\\[2pt]\sin\phi&\cos\phi\end{bmatrix}
\begin{bmatrix} U\\ \ell_h r_1\end{bmatrix},
\]

展开：

\[
\boxed{
\begin{aligned}
v_{P_h,x2}&=U\cos\phi-\ell_h r_1\sin\phi,\\
v_{P_h,y2}&=U\sin\phi+\ell_h r_1\cos\phi.
\end{aligned}}
\tag{F2}
\]

**独立核对**（移植时建议照做）。用单位向量点积，由 (D25)(D26) 有
\(\boldsymbol e_{y2}^{T}\boldsymbol e_{x1}=\sin\phi\)、
\(\boldsymbol e_{y2}^{T}\boldsymbol e_{y1}=\cos\phi\)，所以

\[
v_{P_h,y2}
=\boldsymbol e_{y2}^{T}\!\left(U\boldsymbol e_{x1}+\ell_h r_1\boldsymbol e_{y1}\right)
=U\sin\phi+\ell_h r_1\cos\phi,
\]

与 (F2) 第二式一致。两条独立路径得到同一结果，说明旋转方向没搞反。

### 4.4 第三步：挂车轴纯滚动约束

铰接点位于挂车轴前方 \(L_2\)：
\(\boldsymbol p_{P_h}=\boldsymbol p_{R_2}+L_2\boldsymbol e_{x2}\)。
对时间求导，用 \(\dot{\boldsymbol e}_{x2}=r_2\boldsymbol e_{y2}\)：

\[
\boldsymbol v_{P_h}=\boldsymbol v_{R_2}+L_2r_2\boldsymbol e_{y2}.
\]

左乘 \(\boldsymbol e_{y2}^{T}\)。纯滚动意味着挂车轴无横向速度，即
\(\boldsymbol e_{y2}^{T}\boldsymbol v_{R_2}=0\)，于是

\[
\boxed{v_{P_h,y2}=+L_2r_2.}
\tag{F3}
\]

注意符号是 \(+L_2r_2\)。

联立 (F2) 第二式与 (F3)：

\[
\boxed{
r_{2,\text{kin}}(\phi,U,r_1)=\frac{U\sin\phi+\ell_h r_1\cos\phi}{L_2}}
\tag{F4}
\]

\[
\boxed{
\dot\phi=r_1-r_{2,\text{kin}}
=r_1-\frac{U\sin\phi+\ell_h r_1\cos\phi}{L_2}}
\tag{F5}
\]

与 (K5) 的 \(\dot\theta_2\)、\(\dot\phi\) 两行完全一致。
\(\ell_h=0\) 时退化为 \(\dot\phi=r_1-(U/L_2)\sin\phi\)，再代入
\(r_1=(U/L_1)\tan\delta\) 即 (K6)。

代码：`kinematicTrailerYawRate()` 实现 (F4)。

> **不要用自行车公式代替 IMU 的 \(r_1\)。** 轮胎侧偏时 IMU 测到的 \(r_1\) 才是真正的
> \(\dot\theta_1\)；\(\delta\) 只用来算诊断量 \(\tilde r_1=r_1-(U/L_1)\tan\delta\)
> （代码字段 `truckYawResidual`），它**不进**过程模型。

### 4.5 第四步：残差状态 \(b_{r2}\) 的物理含义

(F4) 假设挂车轴纯滚动，真实挂车会侧偏，所以 \(r_2\neq r_{2,\text{kin}}\)。定义

\[
b_{r2}\;\equiv\;r_2-r_{2,\text{kin}}.
\]

放开纯滚动假设，令挂车轴横向速度为 \(v_{2r}\)。此时 (F3) 变成
\(v_{P_h,y2}=v_{2r}+L_2r_2\)，与 (F2) 联立得
\(r_2=r_{2,\text{kin}}-v_{2r}/L_2\)，即 \(b_{r2}=-v_{2r}/L_2\)。
再由 (K7) 的挂车轴侧偏角 \(\alpha_{2r}=-v_{2r}/U\) 与 \(F_{2r}=C_{2r}\alpha_{2r}\)：

\[
\boxed{b_{r2}=\frac{U\alpha_{2r}}{L_2}=\frac{U\,F_{2r}}{L_2\,C_{2r}}.}
\tag{F6}
\]

**\(b_{r2}\) 就是挂车轴侧偏角折算成的横摆角速度。** 三个推论：

1. 它随侧向力变化，**不是常值**。稳态过弯时正比于 \(U^2\rho\)。
2. 它有确定的动态（下面 (F7)），不是白噪声驱动的随机游走。
3. 稳态下 \(b_{r2}=k_\beta(U)\,r_2\)，其中 \(k_\beta=m_2a_2U^2/(L_2^2C_{2r})\)。
   本仓库参数、\(U=12\) m/s 时 \(k_\beta=0.42\)，即 \(b_{r2}\) 占 \(r_2\) 的四成，
   绝不是小量。

**它的时间常数。** 消去铰接力后挂车子系统给出
\(a_2m_2(\dot v_{y2}+Ur_2)+I_2\dot r_2=L_2F_{2r}\)，代入 (F6) 整理：

\[
\dot b_{r2}=-\lambda(U)\,b_{r2}+\frac{U}{L_2}r_2
+\frac{a_2b_2m_2-I_2}{a_2m_2L_2}\dot r_2,
\qquad
\boxed{\lambda(U)=\frac{C_{2r}L_2}{a_2m_2U}.}
\tag{F7}
\]

本仓库参数：\(\lambda(15)=3.24\ \text{s}^{-1}\)，即 \(\tau_b=0.31\) s；
\(U=25\) 时 \(\tau_b=0.51\) s。**时间常数随车速增大。**

### 4.6 因此本模型的已知短板

\(\tau_b\approx0.3\text{–}0.5\) s 远长于 50 ms 控制周期，把 \(b_{r2}\) 建成随机游走
**跟不上**这段动态。代价主要落在 \(\dot\phi\) 上（实测 1.53°/s，对比动力学 0.60°/s）。

(F7) 含 \(\dot r_2\)，实现它需要对噪声很大的 \(r_1\) 求导，所以本方案**不**直接实现
(F7)，而是把第 5 章的降阶动力学模型作为可选替代 —— 它用 \(v_{y1}\) 作状态，
从根上避免了输入微分。

### 4.7 状态、雅可比与可观性

\[
x=\begin{bmatrix}\phi\\ b_{r2}\end{bmatrix},
\qquad
\dot\phi=r_1-r_{2,\text{kin}}(\phi)-b_{r2},
\qquad
\dot b_{r2}=0+w_b .
\]

令

\[
a\;\equiv\;\frac{\partial r_{2,\text{kin}}}{\partial\phi}
=\frac{U\cos\phi-\ell_h r_1\sin\phi}{L_2},
\tag{F8}
\]

则

\[
\boxed{
A_{\text{kin}}=\frac{\partial\dot x}{\partial x}
=\begin{bmatrix}-a&-1\\[2pt]0&0\end{bmatrix}.}
\tag{F9}
\]

代码：`KinematicArticulationModel::continuousJacobian()`。

量测 \(H_{\text{kin}}=[1,\;0]\)（代码 `measurementJacobian()`）。可观性矩阵

\[
\mathcal O_{\text{kin}}=\begin{bmatrix}H\\ HA\end{bmatrix}
=\begin{bmatrix}1&0\\ -a&-1\end{bmatrix},
\qquad
\boxed{\det\mathcal O_{\text{kin}}=-1 .}
\tag{F10}
\]

行列式恒为 \(-1\)，**与车速、与铰接角无关**。这两个状态在任何工作点都结构可观，
不需要工况激励。测试 `testKinematicObservabilityRank`。

### 4.8 \(\dot\phi\) 怎么出来

\[
\hat r_2=r_{2,\text{kin}}(\hat\phi,U,r_1)+\hat b_{r2},
\qquad
\boxed{\hat{\dot\phi}=r_1-\hat r_2 .}
\tag{F11}
\]

代码：`publish()` 中 `trailerRate = kinematicRate + trailerBias`，
再 `estimate.articulationRate = inputs.truckYawRate - trailerRate`。
**不是**对 \(\hat\phi\) 做数值差分。

---

## 5. 过程模型 B：降阶动力学 (K27r)

代码：`DynamicArticulationModel`（`src/articulation_estimator.cpp`）。
"r" 表示 reduced，即在 (K27) 基础上降了一阶。

### 5.1 本节附加符号

| 符号 | 含义 | 单位 |
|---|---|---|
| \(m_1,m_2\) | 卡车、挂车质量 | kg |
| \(I_1,I_2\) | 卡车、挂车绕质心转动惯量 | kg·m² |
| \(C_{1f},C_{1r}\) | 卡车前轴、后轴侧偏刚度 | N/rad |
| \(C_{2r}\) | 挂车轴侧偏刚度 | N/rad |
| \(\boldsymbol x_p\) | (K27) 的四维状态 | — |
| \(A_p,B_p\) | (K27) 的系统、输入矩阵 | — |
| \(a_{ij}\) | \(A_p\) 的元素（\(U\) 的函数） | — |
| \(\beta_i\) | \(B_p\) 的元素（\(U\) 的函数） | — |
| \(x_r\) | 降阶后的三维状态 | — |
| \(u_r\) | 降阶后的输入 | — |

### 5.2 为什么能降阶

(K27) 给出

\[
\dot{\boldsymbol x}_p=A_p\boldsymbol x_p+B_p\delta,
\qquad
\boldsymbol x_p=[v_{y1},\;r_1,\;r_2,\;\phi]^{\top}.
\]

四个状态里 \(r_1\) **是直接测量的**（拖头 IMU）。既然测得到，就没必要再估计它：
把 \(\dot r_1\) 那一行删掉，把 \(r_1\) 从状态挪到输入。这样做的收益是
少一个状态、少一份过程噪声，而且 \(r_1\) 的信息直接以传感器精度进入模型，
不必等雷达来校正。

### 5.3 降阶推导

取

\[
x_r=\begin{bmatrix}v_{y1}\\ r_2\\ \phi\end{bmatrix},
\qquad
u_r=\begin{bmatrix}r_1\\ \delta\end{bmatrix}.
\]

保留 (K27) 的第 1、3、4 行（\(\dot v_{y1},\dot r_2,\dot\phi\)），
把原先乘 \(r_1\) 的那一列整体移到输入矩阵：

\[
\boxed{
\dot x_r=
\underbrace{\begin{bmatrix}
a_{11}&a_{13}&a_{14}\\
a_{31}&a_{33}&a_{34}\\
0&-1&0
\end{bmatrix}}_{A_{\text{dyn}}}x_r
+
\underbrace{\begin{bmatrix}
a_{12}&\beta_1\\
a_{32}&\beta_3\\
1&0
\end{bmatrix}}_{B_{\text{dyn}}}u_r }
\tag{F12}
\]

代码逐元素对应（`DynamicArticulationModel::refresh()`）：

```
continuous_[0][*] = plant.a[0][{0,2,3}]   // 第 1 行：vy1
continuous_[1][*] = plant.a[2][{0,2,3}]   // 第 2 行：r2
continuous_[2][1] = -1.0                  // 第 3 行：phi
inputYawRate_     = {plant.a[0][1], plant.a[2][1], 1.0}
inputSteering_    = {plant.b[0],    plant.b[2],    0.0}
```

**第三行不是近似。** 它就是恒等式

\[
\dot\phi=r_1-r_2,
\]

其中 \(-1\) 乘状态 \(r_2\)（`continuous_[2][1]`），\(+1\) 乘输入 \(r_1\)
（`inputYawRate_[2]`）。这一行把"铰接角速度"这个**约束关系**写进了过程模型 ——
注意这与"把 \(\dot\phi\) 当成观测量"是完全不同的两件事，见第 6.3 节。

### 5.4 可观性

量测 \(H_{\text{dyn}}=[0,\;0,\;1]\)（代码 `measurementJacobian()`）。

\[
\mathcal O_{\text{dyn}}=
\begin{bmatrix}H\\ HA\\ HA^2\end{bmatrix}
=\begin{bmatrix}
0&0&1\\
0&-1&0\\
-a_{31}&-a_{33}&-a_{34}
\end{bmatrix},
\qquad
\boxed{\det\mathcal O_{\text{dyn}}=-a_{31}.}
\tag{F13}
\]

本仓库名义参数下 \(a_{31}=0.2875\)，结构可观。
但注意 \(v_{y1}\) 只能通过 \(\phi\) 的**二阶**动态被感知，在 10 Hz、
0.5° 噪声的量测下数值可观性较弱，条件数差。
测试 `testDynamicObservabilityRank`。

### 5.5 车速调度与离散

\(a_{ij},\beta_i\) 都是 \(U\) 的函数。实现按 `modelRefreshSpeedStep`（默认 0.25 m/s）
重建矩阵，低于 `minimumModelSpeed`（默认 0.5 m/s）时取下限，避免 (K27) 中
\(1/U\) 项发散。

调度后模型是线性的，所以均值与 \(\Phi\) 都用**增广矩阵指数**做精确 ZOH
（`propagate()`，5×5 增广矩阵 = 3 状态 + 2 输入），而不是欧拉。

### 5.6 \(\dot\phi\) 怎么出来

\(r_2\) 本身就是状态，所以

\[
\boxed{\hat{\dot\phi}=r_1-\hat r_2=r_1-\hat x_r[1].}
\tag{F14}
\]

代码：`publish()` 中 `trailerRate = state[1]`，再走与运动学同一行
`estimate.articulationRate = inputs.truckYawRate - trailerRate`。
同样**不做数值差分**。两者输出公式完全一致，差别只在 \(\hat r_2\) 的来源：
运动学是 \(r_{2,\text{kin}}+\hat b_{r2}\)，动力学是直接的状态分量。

### 5.7 何时不要用

| 条件 | 原因 |
|---|---|
| 载重、\(I_2\)、\(C_{2r}\) 未知或变化大 | (F12) 每个系数都依赖它们 |
| 低速、倒车、强制动 | (K27) 的 \(1/U\) 与恒速假设失效 |
| 大侧偏、大铰接角 | 线性轮胎与小角度假设失效 |
| 只能用同参数 (K27) 仿真验收 | **逆犯罪**，会得到虚假优势 |

测试 `testDynamicModelSurvivesParameterMismatch` 在 \(m_2,I_2,C_{2r}\) 各 ±30%
失配下要求动力学模型不劣于运动学模型。

---

## 6. 量测模型（两个模型共用）

### 6.1 量测方程

\[
z=\phi+v,\qquad v\sim\mathcal N(0,R).
\]

唯一区别是 \(\phi\) 在状态向量中的位置：

| 模型 | 状态 | \(H\) | 代码 |
|---|---|---|---|
| 运动学 | \([\phi,b_{r2}]\) | \([1,\;0]\) | `KinematicArticulationModel::measurementJacobian()` |
| 动力学 | \([v_{y1},r_2,\phi]\) | \([0,\;0,\;1]\) | `DynamicArticulationModel::measurementJacobian()` |

### 6.2 更新方程

\[
\hat z=\text{wrap}(\phi),\qquad
y=\text{wrap}(z-\hat z),\qquad
S=HPH^{\top}+R .
\]

\(S\le0\) 或非有限视为实现错误，抛异常。增益 \(K=PH^{\top}S^{-1}\)，更新

\[
x\leftarrow x+Ky,\quad\phi\leftarrow\text{wrap}(\phi),
\]
\[
\boxed{P\leftarrow(I-KH)P(I-KH)^{\top}+KRK^{\top}.}
\tag{F15}
\]

Joseph 形式在 \(K\) 与 \(P\) 略有数值不一致时仍保持对称**半**正定
（不是正定，不要过度声称）。实现另做一次显式对称化。

`wrap` 用 `std::remainder` 折到 \((-\pi,\pi]\)。

**马氏门限**：\(d=y^2/S\sim\chi^2_1\)，\(d>\) `mahalanobisGate`（默认 9，约 \(3\sigma\)）
则拒绝，不改状态。理想误拒率约 **0.27%**，10 Hz 下平均 37 秒一次 —— 这不是
"几乎不会发生"，复盘时应按此预期核对。

### 6.3 为什么 \(\dot\phi\) 不是观测量

这是移植时最容易做错的地方，单独说明。

**事实**：两个过程模型都**没有**把 \(\dot\phi\) 建成观测量。
两个 \(H\) 矩阵（6.1 表）都只有 \(\phi\) 对应的那一列非零。

**原因**：观测量必须对应一个真实传感器。第 1.1 节的清单里没有任何设备输出
\(\dot\phi\) —— 没有铰接编码器，没有挂车 IMU。凭空加一行
"\(z_2=\dot\phi\)" 等于向滤波器注入不存在的信息，会让 \(P\) 虚假收缩，
估计看起来很稳，实际已经错了。

**三个概念要分清**：

| 概念 | 运动学模型 | 动力学模型 |
|---|---|---|
| \(\dot\phi\) 是**状态**吗 | 否 | 否 |
| \(\dot\phi\) 是**观测量**吗 | **否** | **否** |
| \(\dot\phi\) 是**输出**吗 | 是，\(r_1-(r_{2,\text{kin}}+\hat b_{r2})\) | 是，\(r_1-\hat r_2\) |
| 过程模型里有 \(\dot\phi\) 的**方程**吗 | 有，(F5) 是 \(\phi\) 的状态方程 | 有，(F12) 第三行 |

第四行和第二行的区别是本节的要点：**过程模型里写 \(\dot\phi=r_1-r_2\)，
是在描述状态如何随时间演化**（一个运动学恒等式）；
**把 \(\dot\phi\) 写进 \(H\)，是在声称有传感器测到了它**（一个不存在的事实）。
前者必须有，后者绝不能有。

**推论**：\(\dot\phi\) 的精度完全由过程模型质量决定，与雷达精度无直接关系。
这正是两个模型在 \(\dot\phi\) 上差距（1.53 对 0.60 °/s）远大于在 \(\phi\) 上
差距（0.56 对 0.30°）的原因 —— \(\phi\) 有量测托底，\(\dot\phi\) 没有。

**如果将来真的装了铰接编码器**，那时才应该加第二行量测。届时运动学模型的
\(H\) 变成 \(\begin{bmatrix}1&0\\-a&-1\end{bmatrix}\)（因为
\(\dot\phi=r_1-r_{2,\text{kin}}(\phi)-b_{r2}\) 依赖状态），
动力学模型的 \(H\) 变成 \(\begin{bmatrix}0&0&1\\0&-1&0\end{bmatrix}\)。

### 6.4 为什么没有雷达偏置状态

早期版本在运动学模型上加过第三个状态 \(b_\phi\)（雷达安装偏差），
此时 \(H=[1,0,1]\)。可观性矩阵

\[
\mathcal O=\begin{bmatrix}1&0&1\\-a&-1&0\\ a^2&a&0\end{bmatrix}
\]

第三行等于 \(-a\) 乘第二行，秩只有 2。不可观方向为
\([1,\,-a,\,-1]^\top\)：取 \(\delta\phi=\varepsilon\)、
\(\delta b_{r2}=-a\varepsilon\)、\(\delta b_\phi=-\varepsilon\)，
在一阶上既不改变输出也不改变动态。

它只能靠极强先验钉住，实测对精度无贡献，因此移除。
**若实车确有安装偏差，离线标定后从量测中减掉，不要建成状态。**

---

## 7. 噪声与离散化（两个模型共用）

### 7.1 唯一契约：连续功率谱密度

**所有过程噪声参数都是连续时间 PSD，不是每拍方差。**
这样把步长减半不会改变建模的噪声总量，第 8 章的区间切分才自洽。

| 符号 | 代码字段 | 单位 | 默认 | 用于 |
|---|---|---|---|---|
| \(q_\phi\) | `noiseDensity.articulationRate` | rad²/s | \(2.742\times10^{-3}\) | 两者 |
| \(q_b\) | `noiseDensity.trailerYawBias` | rad²/s³ | \(2\times10^{-4}\) | 运动学 |
| \(\sigma_{r1}^2\) | `noiseDensity.truckYawRate` | rad²/s | \(2.742\times10^{-5}\) | 两者 |
| \(\sigma_U^2\) | `noiseDensity.speed` | m²/s | \(0.04\) | 运动学 |
| \(q_{v_{y1}}\) | `noiseDensity.truckLateralVelocity` | (m/s)²/s | \(0.05\) | 动力学 |
| \(q_{r_2}\) | `noiseDensity.trailerYawRate` | rad²/s³ | \(1.5\times10^{-4}\) | 动力学 |

**量纲自检**（移植时务必做）：\(Q_{\phi\phi}\) 必须是 rad²，它由 \(q_\phi\) 对时间
积分得到，故 \([q_\phi]=\text{rad}^2/\text{s}\)。若误当成"角速度方差"
（rad²/s²）会差一个时间量纲。

### 7.2 各模型的 \(Q_c\)

**运动学**。\(r_1,U\) 的噪声通过 (F5) 进入 \(\dot\phi\)：

\[
\frac{\partial\dot\phi}{\partial r_1}=1-\frac{\ell_h\cos\phi}{L_2},
\qquad
\frac{\partial\dot\phi}{\partial U}=-\frac{\sin\phi}{L_2},
\tag{F16}
\]

\[
\boxed{
Q_{c,\text{kin}}=\operatorname{diag}\!\left(
\eta q_\phi
+\Big(\tfrac{\partial\dot\phi}{\partial r_1}\Big)^{2}\sigma_{r1}^2
+\Big(\tfrac{\partial\dot\phi}{\partial U}\Big)^{2}\sigma_U^2,
\;\;q_b\right)}
\tag{F17}
\]

代码：`articulation_estimator.cpp:161-167`。

**动力学**。对角项为 \(\operatorname{diag}(q_{v_{y1}},\,q_{r_2},\,\eta q_\phi)\)，
再叠加 \(r_1\) 传感器噪声 —— 因为 \(r_1\) 是输入，其噪声按输入列
\(B_{\text{dyn}}[:,0]\) 传播，产生**满矩阵**贡献：

\[
\boxed{
Q_{c,\text{dyn}}=\operatorname{diag}(q_{v_{y1}},q_{r_2},\eta q_\phi)
+\sigma_{r1}^2\,B_{\text{dyn}}[:,0]\,B_{\text{dyn}}[:,0]^{\top}}
\tag{F18}
\]

代码：`articulation_estimator.cpp:303-312`。注意这里有交叉项，不能只填对角。

\(\eta\) 是开环膨胀倍率（第 9 章），正常时为 1。

### 7.3 Van Loan 离散

对 \(\dot x=Ax+w\)、\(w\) 的 PSD 为 \(Q_c\)，构造 \(2n\times2n\) 矩阵

\[
M=\begin{bmatrix}-A&Q_c\\ 0&A^{\top}\end{bmatrix}\Delta t,
\qquad
e^{M}=\begin{bmatrix}M_{11}&M_{12}\\ 0&M_{22}\end{bmatrix},
\]

则

\[
\boxed{\Phi=M_{22}^{\top},\qquad Q_d=\Phi\,M_{12}.}
\tag{F19}
\]

代码：`discretizeVanLoan()`（`include/truck_model/matrix_exponential.hpp`），
内部复用 `matrixExponential()`（缩放平方 + Taylor）。

**为什么不能只填对角**（以运动学为例）。\(b_{r2}\) 通过
\(\dot\phi=\cdots-b_{r2}\) 直接驱动 \(\phi\)。把 (F17) 代入 (F19)，
小步长展开得

\[
Q_{d,\text{kin}}\simeq
\begin{bmatrix}
q_\phi T+\tfrac13 q_b T^{3} & -\tfrac12 q_b T^{2}\\[3pt]
-\tfrac12 q_b T^{2} & q_b T
\end{bmatrix}.
\tag{F20}
\]

只填 \(\operatorname{diag}(q_\phi T,\,q_b T)\) 会丢掉 \(T^3\) 项和交叉项，
低估 \(\phi\) 的不确定度。测试 `testVanLoanMatchesAnalyticRandomWalk`
逐元素比对 (F20)。

### 7.4 均值与转移矩阵

| | 运动学 | 动力学 |
|---|---|---|
| 均值 | (F5) 一步欧拉，\(\phi^{+}=\text{wrap}(\phi+\Delta t\,\dot\phi)\) | 增广矩阵指数，精确 ZOH |
| \(\Phi\) | 欧拉映射的精确雅可比 \(I+A\Delta t\) | \(e^{A\Delta t}\)（同一增广指数） |
| 为何不同 | 模型非线性，\(\Phi\) 必须与均值映射自洽 | 调度后线性，可用精确解 |

测试 `testKinematicJacobianMatchesFiniteDifference` 用中心差分核对运动学的
\(\Phi\) 与均值映射一致。

---

## 8. 时延处理（两个模型共用）

代码：`DelayedEkf<Model>`（`include/truck_model/delayed_ekf.hpp`），
两个过程模型分别作为模板参数实例化。

### 8.1 前提：扫描戳单调不减

\[
\boxed{\text{后到达的扫描，其 stamp 不早于已融合过的任何 stamp}.}
\]

**为什么这个前提让算法变简单。** 当 \(t_s\) 的扫描到达时，所有已融合的扫描戳都
\(\le t_s\)。于是从 \(t_s\) 向前重推时，后面**不存在**任何已融合的校正会被覆盖，
只重推输入就够了。若扫描可能乱序，前向过程还必须重新应用那些更晚的量测，
需要完整的事件日志。

单雷达、单感知管线、FIFO 传输通常满足该前提。

**不要默认它成立，要校验。** 实现对 stamp \(\le\) 上一次已处理 stamp 的报文直接
拒绝，并给出独立结局码。这样一旦管线真的乱序，会表现为一个可见的计数器，
而不是被静默损坏的状态。

### 8.2 数据结构

```
Frame:
    time         : double            # 该帧对应的时刻
    inputs       : {r1, U, delta}    # 支配"上一帧 -> 本帧"区间的输入
    state        : Vector<n>         # 处理完本帧后的后验
    covariance   : Matrix<n,n>

DelayedEkf:
    frames               : deque<Frame>   # 按时间升序
    lastAcceptedStamp    : double
    lastMeasurementStamp : double         # 含被拒的
    consecutiveRejects   : int
```

区间约定：**右端点 ZOH**，区间 \([t_k,t_{k+1}]\) 使用 \(u_{k+1}\)。

### 8.3 predict：每个 IMU/控制拍

```
predict(u, t):
    if t <= frames.back().time:          # 不倒退时间，只刷新输入
        frames.back().inputs = u
        return
    next.time   = t
    next.inputs = u
    next.state, next.covariance =
        propagate(frames.back().state, frames.back().covariance,
                  u, t - frames.back().time, noiseScale(frames.back().time))
    frames.push_back(next)
    trim()                                # 丢弃超出 historyHorizon 的最老帧
```

复杂度 O(1)。

### 8.4 update：雷达报文到达

```
update(stamp, z):
    # --- 前置检查，顺序不可调换 ---
    if not initialized:               return notInitialized
    if stamp or z not finite:         return nonFinite
    if stamp == lastMeasurementStamp: return duplicate
    if stamp <  lastMeasurementStamp: rejects++; return outOfOrder
    if stamp <  frames.front().time:  rejects++; return staleBeyondWindow
    if stamp >  frames.back().time:   return aheadOfInputs
    lastMeasurementStamp = stamp

    i = locateFrame(stamp)            # 见 8.5，默认在 stamp 处切分

    outcome = applyMeasurement(frames[i], z)   # (F15) Joseph 更新
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

`MeasurementOutcome` 共 8 个取值，全部列出：

| 结局码 | 触发条件 | 处理 |
|---|---|---|
| `notInitialized` | 还没调过 `reset()` | 拒绝 |
| `nonFinite` | stamp 或 \(z\) 非有限 | 拒绝；上游数据损坏 |
| `duplicate` | stamp 与上一条相同 | 拒绝，不计入连续拒绝 |
| `outOfOrder` | stamp 早于上一条 | 拒绝并计数；**前提被违反的信号** |
| `staleBeyondWindow` | stamp 早于最老帧 | 拒绝并计数；`historyHorizon` 不足的唯一症状 |
| `aheadOfInputs` | stamp 晚于最新帧 | 拒绝。外推最新输入会凭空造信息，应先 `predict` |
| `gated` | \(d>\) 门限 | 拒绝并计数，不改状态 |
| `accepted` | 其余 | Joseph 更新 + 重推 |

### 8.5 精确切分

扫描戳一般落在两帧之间。默认在 \(t_s\) 处**插入一帧**，把区间切成
\([t_k,t_s]\) 与 \([t_s,t_{k+1}]\)，**两段都沿用原区间的 \(u_{k+1}\)**。
切分不改变输入语义，对齐误差为零。

替代做法是吸附到最近的已有帧（`snapToNearestFrame`），会引入

\[
e_t\approx-\dot\phi(t_s)\cdot\epsilon,\qquad|\epsilon|\le\tfrac12\Delta t_{\text{ctrl}}
\]

的额外量测误差。\(\Delta t=50\) ms、\(\dot\phi=40°/\text{s}\) 时最坏约 1°：
对 4° 噪声可忽略，对 0.5° 级雷达不可忽略。默认用精确切分。
测试 `testSubFrameAlignmentIsExact`。

### 8.6 历史窗

\[
\boxed{\texttt{historyHorizon}\;\ge\;\tau_{\max}+\Delta t_{\text{ctrl}}}
\]

窗口短于最大时延时，迟到报文落在历史之外被丢弃。默认 0.55 s 覆盖 400 ms 工况。
本仓库在配置期强制校验 `lidarDelayMax <= historyHorizon`，不满足直接报错。
测试 `testHistoryHorizonMustCoverLatency`。

---

## 9. 开环与链路健康：两个不同的量

\[
\boxed{
\text{informationAge}=t_{\text{now}}-t_{\text{lastAccepted}},
\qquad
\text{arrivalGap}=t_{\text{now}}-t_{\text{lastSuccessfulUpdate}} .}
\]

- **informationAge**：当前估计背后最新融合信息的年龄。**过程噪声倍率由它决定**：
  超过 `lostTimeout`（默认 0.6 s）或连续拒绝达 `consecutiveRejectLimit`（默认 3）
  时 \(\eta=4\)。400 ms 时延下它本来就接近 0.4 s，这不是误报 —— 估计确实在外推。
- **arrivalGap**：距上次成功更新的墙钟时间。**链路健康由它决定**，
  超过 `linkTimeout`（默认 0.5 s）置 `linkStalled`。

把两者混成一个标志会让长时延被误报成断链。`lostTimeout` 默认 0.6 s > 400 ms，
正常工况不触发。

MPC 目前不消费 \(P\)。**长时间开环没有自动降级，上层必须自行定义失效上限。**

---

## 10. 参数表

面板上的标准差是**度**，写入配置前换成 rad 再平方。

| 项 | 字段 | 默认 | 适用 | 说明 |
|---|---|---|---|---|
| 过程模型 | `processModel` | `kinematic` | — | `kinematic` / `dynamic` |
| 历史窗 | `historyHorizon` | 0.55 s | 两者 | 必须 \(\ge\tau_{\max}\)，配置期强制 |
| \(R\) | `measurementVariance` | \(3.046\times10^{-4}\)（1°） | 两者 | 等于雷达 \(\sigma^2\) |
| \(q_\phi\) | `noiseDensity.articulationRate` | \(2.742\times10^{-3}\) | 两者 | rad²/s |
| \(q_b\) | `noiseDensity.trailerYawBias` | \(2\times10^{-4}\) | 运动学 | rad²/s³ |
| \(\sigma_{r1}^2\) | `noiseDensity.truckYawRate` | \(2.742\times10^{-5}\) | 两者 | rad²/s |
| \(\sigma_U^2\) | `noiseDensity.speed` | 0.04 | 运动学 | m²/s |
| \(q_{v_{y1}}\) | `noiseDensity.truckLateralVelocity` | 0.05 | 动力学 | \(0.3\,\text{m/s}^2\)、\(\tau=0.2\) s |
| \(q_{r_2}\) | `noiseDensity.trailerYawRate` | \(1.5\times10^{-4}\) | 动力学 | \(1°/\text{s}^2\)、\(\tau=0.2\) s |
| 门限 | `mahalanobisGate` | 9 | 两者 | \(R\) 对齐后不要再拧死 |
| 连续拒 | `consecutiveRejectLimit` | 3 | 两者 | |
| 信息年龄阈值 | `lostTimeout` | 0.6 s | 两者 | 决定 \(\eta\) |
| 链路阈值 | `linkTimeout` | 0.5 s | 两者 | 仅诊断 |
| \(P_\phi(0)\) | `initialArticulationVariance` | \((2°)^2\) | 两者 | |
| \(P_b(0)\) | `initialTrailerYawBiasVariance` | \((2°/\text{s})^2\) | 运动学 | |
| \(P_{v_{y1}}(0)\) | `initialTruckLateralVelocityVariance` | 0.25 | 动力学 | |
| \(P_{r_2}(0)\) | `initialTrailerYawRateVariance` | \((2°/\text{s})^2\) | 动力学 | |
| 调度步长 | `modelRefreshSpeedStep` | 0.25 m/s | 动力学 | |
| 最低建模车速 | `minimumModelSpeed` | 0.5 m/s | 动力学 | 防 \(1/U\) 发散 |

**\(R\) 必须与雷达噪声同量级。** 雷达 3°–4° 噪声却用 1° 的 \(R\)，会让马氏门限
批量误拒。Demo 中 `ekfMeasurementFollowsLidar` 开关负责自动跟随。

几何参数必须与实车一致：\(a_2,b_2,b_1,d_1\)。\(L_2\) 错 10% 会系统性扭曲 (F4)。

---

## 11. 公式与代码对照表

| 公式 | 含义 | 代码位置 |
|---|---|---|
| (F4) | \(r_{2,\text{kin}}\) | `kinematicTrailerYawRate()` |
| (F5) | 运动学均值传播 | `KinematicArticulationModel::propagate()` |
| (F9) | \(A_{\text{kin}}\) | `KinematicArticulationModel::continuousJacobian()` |
| (F11) | 运动学 \(\hat{\dot\phi}\) | `publish()` |
| (F12) | \(A_{\text{dyn}},B_{\text{dyn}}\) | `DynamicArticulationModel::refresh()` |
| (F14) | 动力学 \(\hat{\dot\phi}\) | `publish()` |
| (F15) | Joseph 更新 | `DelayedEkf::applyMeasurement()` |
| (F17) | \(Q_{c,\text{kin}}\) | `KinematicArticulationModel::propagate()` |
| (F18) | \(Q_{c,\text{dyn}}\) | `DynamicArticulationModel::propagate()` |
| (F19) | Van Loan | `discretizeVanLoan()` in `matrix_exponential.hpp` |
| 6.1 表 | \(H\) | `measurementJacobian()`（两个模型各一份） |
| 8.3 | predict | `DelayedEkf::predict()` |
| 8.4 | update | `DelayedEkf::update()` |
| 第 9 章 | 开环判定 | `DelayedEkf::informationAge() / coasting()` |

外部 API 只有一个门面类 `ArticulationEstimator`，它按 `processModel` 在两个
`DelayedEkf` 实例之间切换：

```
configure(parameters, config)   // 建模型、设限值
reset(time, articulation)       // 清历史，置初值
predict(inputs) -> Estimate     // 每个控制拍
updateLidar(measurement) -> Estimate  // 每个雷达报文
estimate() -> Estimate          // 读当前值
```

---

## 12. Demo 中的三个开关

Studio 面板"铰接角雷达融合"下有三个开关，语义相互正交。

| 开关 | 字段 | 作用 | 默认 |
|---|---|---|---|
| **a** 启用融合滤波 | `lidarFusionEnabled` | 运行主滤波器，模型由 `articulationEstimator.processModel` 选 | 关 |
| **b** MPC 使用融合结果 | `mpcUsesFusedArticulation` | 开：控制器读 \(\hat\phi,\hat{\dot\phi}\)；关：读 Plant 真值，**滤波器照常运行** | 关 |
| **c** 启用对照估计器 | `shadowEstimatorEnabled` | 第二个滤波器，吃同样的扫描和输入，模型由 `shadowProcessModel` 选，**永不进回路** | 关 |

b 和 c 都依赖 a（配置校验会拒绝单独打开）。

**自检性质**：a 与 c 选**同一**过程模型时，两条估计曲线必须逐位重合 ——
两个滤波器拿到的是同一个 `inputs` 对象和同一个 `measurement` 对象，
唯一的差别只可能来自过程模型。不重合就说明接线出了问题。
测试 `testSameProcessModelMakesBothFiltersAgree`（阈值 \(10^{-12}\)）与
`testDifferentProcessModelsDiverge` 分别守住这两个方向。

**b 为什么默认关**：先看滤波器本身好不好，再看它进回路会怎样。这两件事混在一起
正是 13.2 节那个现象的来源。

---

## 13. 实测数据

Demo 默认 S 形路径，雷达 0.5° 噪声、0.1–0.3 s 时延，统计区间 \(t\ge1\) s，
b 关（开环评估，控制器不受估计影响）。

### 13.1 两个模型的精度

| 量 | 运动学 K5 | 动力学 K27r | 原始迟到雷达 |
|---|---|---|---|
| \(\phi\) RMSE | 0.56° | **0.30°** | 1.02° |
| \(\dot\phi\) RMSE | 1.53°/s | **0.60°/s** | 观测不到 |

两个模型都明显优于直接用迟到的雷达读数。\(\dot\phi\) 上的差距（2.5 倍）
远大于 \(\phi\) 上的差距（1.9 倍），原因见 6.3 节末：\(\phi\) 有量测托底，
\(\dot\phi\) 纯靠模型。

**注意**：本仓库的仿真 Plant 就是 (K27)，与动力学模型同源，
所以上表对动力学有利的部分属于**逆犯罪**，不能作为最终证据。

### 13.2 开关 b 对闭环的影响

铰接角跟踪实验（控制器闭环跟踪 \(\phi_{\text{ref}}\)，正弦 0.12 rad / 0.12 Hz）：

| 配置 | plant/ref | est/plant |
|---|---|---|
| 不融合 | 0.992 | 1.000 |
| K5 滤波运行，**b 关** | **0.992** | 0.866 |
| K5 **b 开**（进回路） | **1.156** | 0.865 |
| K27r **b 开**（进回路） | 0.990 | 1.005 |

**这组数据说明了三件事**：

1. **滤波器本身不会让 plant 超过参考。** 第二行 b 关时 plant/ref 保持 0.992，
   和完全不融合时一模一样 —— 滤波器在跑，但不碰控制回路。
2. **超调只在 b 打开时出现**，且量值可由估计器偏差算出：
   K5 的估计只读到真值的 86.5%，控制器把**估计**压到参考，真实角度就被抬高到
   \(1/0.865=1.156\) 倍，与实测 1.156 严丝合缝。
3. **估计足够准时超调消失。** K27r 的 est/plant = 1.005（几乎无偏），
   闭环后 plant/ref = 0.990，没有任何超调。

第 3 点正是判据：**融合结果贴近真值，就不会出现这个现象**。
反过来，看到 plant 超过参考，应当先查估计器的幅值比，而不是查控制器。

也因此，**铰接角跟踪实验不能用作融合验收**：它把估计误差经闭环放大，
测的是"估计器偏差 × 控制器增益"的乘积，不是估计精度本身。
验收要用 b 关的开环配置，即 13.1 节。

### 13.3 复现方法

```
tools/fusion_probe.cpp   # 产生上面两张表的全部数字
```

---

## 14. 移植清单

按顺序做，每步有独立验收条件，不要跳步。

### 步骤 1：确认前提

- [ ] 感知链路的扫描戳单调不减？若否，8.1 的简化不成立，需要扩成事件重放。
- [ ] 雷达给的是扫描时刻而非到达时刻？
- [ ] 车端与感知时间同步（PTP/GNSS）？
- [ ] 已知 \(\tau_{\max}\)？据此设 `historyHorizon = τ_max + Δt_ctrl`。
- [ ] 确认**没有**任何传感器测 \(\dot\phi\)（若有，见 6.3 节末的扩展方式）。

### 步骤 2：选过程模型

先只实现运动学。它只要两个几何尺寸，不会因载重变化失准。
第 3 章的对照表是取舍依据。动力学留到步骤 8。

### 步骤 3：接上已有 EKF 核

目标仓若已有 EKF 类，**在它上面扩充**，不要另起卡尔曼核。需要的能力：

| 能力 | 若缺失 |
|---|---|
| 读写完整 \(x,P\) | 增加 `snapshot()` / `restore(x,P)`，历史回退要用 |
| 每步传入不同 \(Q_d\) | 预测接口允许本步传 \(Q\) |
| 角度残差 wrap | 在本量测的残差回调里 wrap，不要改其它滤波 |
| 状态维可配 | 设为 \(n=2\)（动力学为 3） |

**快照必须覆盖该核的全部内部统计状态**，不只是 \(x,P\)。若平台用平方根因子或
自适应统计，仅存 \(x,P\) 无法完整恢复。

**验收**：snapshot → 若干步 predict → restore → 重放，得到逐位相同的结果。

### 步骤 4：实现运动学过程模型

实现 (F4)(F5)(F9)(F17)。

**验收**：
- \(r_{2,\text{kin}}\) 在 \(\ell_h=0\) 与 \(\ell_h\ne0\) 两种情形下都与 (K5) 的
  \(\dot\theta_2\) 一致；
- \(\Phi\) 与均值映射的中心差分一致，逐元素误差 \(<10^{-6}\)；
- 可观矩阵行列式为 \(-1\)。

### 步骤 5：实现噪声离散

实现 (F19)。

**验收**：
- 对 \(A=\begin{bmatrix}0&-1\\0&0\end{bmatrix}\)、\(Q_c=\operatorname{diag}(0,q)\)，
  \(Q_d\) 逐元素等于 (F20) 的解析值；
- 分段复合成立：\(Q_{[0,T]}=\Phi_2Q_{[0,T_1]}\Phi_2^{\top}+Q_{[T_1,T]}\)。

### 步骤 6：实现时延外壳

实现 8.2–8.6。

**验收**：
- 亚帧扫描戳（如 5/17/25/43 ms 相位）的 `alignedStamp` 等于 stamp 本身；
- 8 个结局码都能触发并正确上报；
- 窗口边界（\(\tau\) 略小于/略大于 `historyHorizon`）行为符合预期。

### 步骤 7：接线

- IMU/控制拍调 `predict`，用车端 \(r_1,U,\delta\)；
- 雷达回调把**扫描时刻**和 \(\phi\)（rad）送进 `update`；
- \(\hat\phi,\hat{\dot\phi}\) 写入控制器的铰接通道；
- 底盘/Plant 积分不要用估计值。

**建议**：先按 12 章的 b 关模式接，即滤波器跑起来但控制器仍用原来的信号源，
确认精度达标后再切到估计值。

**验收**：固定 200 ms 时延、无噪声的仿真下，当前 \(\hat\phi\) 的 RMSE
\(<0.015\) rad，且明显优于原始迟到量测。

### 步骤 8：是否加动力学模型

只有在载荷参数可信时才值得做。先以对照模式（12 章的 c）并跑，
不要直接替换默认。判定门槛见 15.3。

**验收**：参数 ±30% 失配下不劣于运动学模型。

### 不要做的事

- 不要忽略时延，把迟到 \(z\) 当当前量测；
- 不要把 \(\dot\phi\) 建成观测量（6.3 节）；
- 不要把 \(\delta\) 代入 (K6) 代替 IMU 的 \(r_1\)；
- 不要在 \(R\) 未与雷达噪声对齐时拧死门限；
- 不要把雷达安装偏差建成在线状态（6.4 节）；
- 不要只用同参数的 (K27) 仿真验收动力学模型；
- 不要用闭环跟踪实验验收融合精度（13.2 节）。

---

## 15. 验收

### 15.1 仓库内已实现的用例

| 用例 | 断言 | 测试名 |
|---|---|---|
| Van Loan vs 解析 | 逐元素等于 (F20) | `testVanLoanMatchesAnalyticRandomWalk` |
| \(Q_d\) 分段复合 | \(Q=\Phi_2Q_1\Phi_2^\top+Q_2\) | `testProcessNoiseIsStepInvariant` |
| 运动学 \(\Phi\) vs 中心差分 | \(<10^{-6}\) | `testKinematicJacobianMatchesFiniteDifference` |
| 运动学可观性 | \(\det\mathcal O=-1\) | `testKinematicObservabilityRank` |
| 动力学可观性 | 秩 3，\(a_{31}\) 非零 | `testDynamicObservabilityRank` |
| \(r_{2,\text{kin}}\) | 对轴/偏轴均与 (K5) 一致 | `testKinematicTrailerYawRateMatchesK5` |
| 亚帧对齐 | `alignedStamp == stamp` | `testSubFrameAlignmentIsExact` |
| 迟到扫描 | 各自在自己的戳上融合 | `testDelayedScanOrdering` |
| 乱序/重复 | 拒绝并上报，不改状态 | `testDelayedScanOrdering` |
| 边界结局 | stale / ahead / gated / accepted | `testMeasurementBoundaryHandling` |
| 固定 200 ms 时延 | RMSE \(<0.015\) rad 且优于原始量测 | `testDelayedEkfCompensatesLidarLatency` |
| 大野值 | 被门限拒绝 | `testLidarOutlierIsGated` |
| 长时间无雷达 | 进入开环 | `testEstimatorCoastsAfterDropout` |
| 动力学改善 \(\dot\phi\) | RMSE \(<0.8\times\) 运动学 | `testDynamicModelImprovesRateTracking` |
| 参数失配 | ±30% 下动力学不劣 | `testDynamicModelSurvivesParameterMismatch` |
| 优于原始迟到量测 | 两模型都 \(<0.5\times\) | `testBothModelsBeatDelayedMeasurement` |
| 带噪传感器 | 收敛 | `testFilterConvergesWithNoisySensors` |
| 历史窗校验 | \(\tau_{\max}>\) 窗口被拒 | `testHistoryHorizonMustCoverLatency` |
| **开关 a=c 同模型** | 两路逐位重合（\(10^{-12}\)） | `testSameProcessModelMakesBothFiltersAgree` |
| **开关 a≠c 异模型** | 两路确实分叉 | `testDifferentProcessModelsDiverge` |
| **开关 b** | 开→控制器读估计；关→读 Plant，滤波仍跑 | `testFusedDataReachesControllerOnlyWhenSwitchBIsOn` |
| 对照模型并跑 | 动力学 \(\dot\phi\) 更好 | `testShadowEstimatorRunsInParallel` |
| 闭环放大估计偏差 | 13.2 节现象可复现 | `testTrackingLoopAmplifiesEstimatorAmplitudeBias` |

### 15.2 开环验收（推荐）

用 12 章的 b 关配置，比较 \(\hat\phi,\hat{\dot\phi}\) 与真值。
这样测到的是估计精度本身，不含控制器反馈。

### 15.3 替换默认过程模型的门槛

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

## 16. 已知限制

1. **默认过程模型是 (K5)，真实车辆有轮胎侧偏。** \(b_{r2}\) 的随机游走跟不上 4.5 节
   算出的 0.3–0.5 s 动态，\(\dot\phi\) 幅值系统性偏瘦。动力学模型是针对性解法，
   但引入参数敏感性。
2. **\(\dot\phi\) 完全是模型输出**（6.3 节）。任何滤波器都不能凭空增加信息，
   它的可信度等于过程模型的可信度。
3. **雷达安装偏差不在状态里**（6.4 节），必须离线标定后从量测中减掉。
   未标定的偏差会经 \(\partial\dot\phi/\partial\phi\approx U/L_2\) 放大：
   \(U=15,L_2=7\) 时，1° 的未建模偏差约产生 \(2.14°/\text{s}\) 的 \(\dot\phi\) 误差。
4. **输入噪声按连续 PSD 处理**，未建 sample-and-hold 的跨分段相关性。
5. **依赖扫描戳单调**（8.1 节）。违反会被拒绝并计数，但那些报文的信息就丢失了。
6. **\(v_{y1}\) 在动力学模型中弱可观**，(F13) 虽满秩但条件数差。
7. **单雷达、单峰噪声。** 多峰或错误关联只靠马氏门限。
8. **时间同步是前提**，本方案不估计时钟偏差。
9. **参考实现无并发保护。** 实车 IMU 与雷达回调必须串行化。
10. **MPC 不消费 \(P\)。** 长时间开环无自动降级。
11. **仓库内的动力学优势含逆犯罪成分**（13.1 节注），实车需重新取证。
