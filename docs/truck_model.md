实现卡车-拖车（Truck-Trailer/Tractor-Semitrailer）铰接机器人的高级控制（如高精度的轨迹跟踪、中高速下的横向稳定控制控制、防止折头等），必须建立完整的数学模型。



### 参数符号定义

在推导前，统一物理参数符号（沿用之前的单轨/自行车模型简化）：

**卡车（主车，Index 1）**：质量 $m_1$，绕质心转动惯量 $I_{z1}$；质心到前轴距离 $a_1$，到后轴（驱动轴）距离 $b_1$。前轮侧偏刚度 $C_{1f}$，后轮侧偏刚度 $C_{1r}$。

**拖车（挂车，Index 2）**：质量 $m_2$，绕质心转动惯量 $I_{z2}$；铰接点到拖车质心距离 $a_2$，质心到拖车后轴距离 $b_2$。拖车轮侧偏刚度 $C_{2r}$。

**铰接点（Hitch/Kingpin）**：本模型假设铰接点位于卡车后轴中心，因此卡车质心到铰接点的距离 $d_1 = b_1$。

**状态与控制**：卡车纵向速度 $v_x$（假设恒定），横向速度 $v_{y1}$，偏航角速度（ yaw rate ）$r_1 = \dot{\theta}_1$；拖车横向速度 $v_{y2}$，偏航角速度 $r_2 = \dot{\theta}_2$。铰接角 $\theta_{12} = \theta_1 - \theta_2$。前轮转向角 $\delta$。



### 一、 完整非线性运动学模型

若将系统基准点设在**卡车后轴中心** $(x_1, y_1)$，其前向行驶纵向速度为 $v$（倒车时 $v$ 取负值），在没有轮胎侧滑的理想非霍洛诺米约束下，完整的4自由度运动学模型为：
$$
\begin{cases}  \dot{x}_1 = v \cos \theta_1 \\  \dot{y}_1 = v \sin \theta_1 \\  \dot{\theta}_1 = \frac{v}{L_1} \tan \delta \\  \dot{\theta}_{12} = \frac{v}{L_1} \tan \delta - \frac{v \sin \theta_{12}}{L_2}  \end{cases}
$$



### 二、 完整平面单轨动力学模型

当中高速行驶时，车轮必定产生侧偏角。我们需要利用**牛顿-欧拉法**分别对卡车和拖车建立受力平衡方程，并通过铰接点约束进行耦合。

#### 1. 卡车动力学方程

在卡车车身坐标系下，卡车受到前轮侧向力 $F_{y1f}$、后轮侧向力 $F_{y1r}$ 以及铰接点处拖车对卡车的反作用力（纵向 $F_{hx}$，横向 $F_{hy}$）：

- **横向过渡运动**：

    $$
    m_1 (\dot{v}_{y1} + v_x r_1) = F_{y1f} \cos \delta + F_{y1r} + F_{hy} \quad \text{--- (11)}
    $$

- **偏航旋转运动**：
    $$
    I_{z1} \dot{r}_1 = a_1 F_{y1f} \cos \delta - b_1 F_{y1r} - d_1 F_{hy} \quad \text{--- (12)}
    $$



#### 2. 拖车动力学方程

在拖车车身坐标系下，拖车受到后轮侧向力 $F_{y2r}$ 变以及铰接点处卡车对拖车的拉力。考虑到卡车与拖车有夹角 $\theta_{12}$，将铰接点力经坐标旋转变换至拖车坐标系：

***这里近似：$v_{x2}=v_{x1}=v_x$***

- **横向过渡运动**：

    $$
    m_2 (\dot{v}_{y2} + v_x r_2) = F_{y2r} - F_{hx} \sin \theta_{12} - F_{hy} \cos \theta_{12} \quad \text{--- (13)}
    $$

- **偏航旋转运动**：

    $$
    I_{z2} \dot{r}_2 = -b_2 F_{y2r} + a_2 (-F_{hx} \sin \theta_{12} - F_{hy} \cos \theta_{12}) \quad \text{--- (14)}
    $$

小角度假设，忽略纵向力作用，近似

$$
 m_2 (\dot{v}_{y2} + v_x r_2) = F_{y2r} - F_{hy}  \quad \text{--- (13)}
$$

$$
I_{z2} \dot{r}_2 = -b_2 F_{y2r} - a_2  F_{hy} \quad \text{--- (14)}
$$



#### 3. 铰接点运动学约束

铰接点是两车刚性连接点。卡车侧铰接点横向速度为 $v_{y1} - d_1 r_1$；拖车侧铰接点横向速度为 $v_{y2} + a_2 r_2$。**近似：在小角度假设下（$\theta_{12}$ 较小，$\cos\theta_{12} \approx 1, \sin\theta_{12} \approx \theta_{12}$），两点速度相等：**

$$v_{y2} + a_2 r_2 = v_{y1} - d_1 r_1 \implies v_{y2} = v_{y1} - d_1 r_1 - a_2 r_2 \quad \text{--- (15)}$$

对时间求导得到横向加速度约束：

$$
\dot{v}_{y2} = \dot{v}_{y1} - d_1 \dot{r}_1 - a_2 \dot{r}_2 \quad \text{--- (16)}
$$

---



#### **消除内部铰接力 $F_{hy}$**

1. **行 1（总横向力平衡）**：将式 (11) 与式 (13) 相加**（应用小角度假设 $\cos \delta \approx 1$）：**
   $$
   m_1 (\dot{v}_{y1} + v_x r_1) + m_2 (\dot{v}_{y2} + v_x r_2) = F_{y1f} + F_{y1r} + F_{y2r}
   $$
   
   将加速度约束式 (16) 代入上式中：
   
   $$
   m_1 \dot{v}_{y1} + m_1 v_x r_1 + m_2 (\dot{v}_{y1} - d_1 \dot{r}_1 - a_2 \dot{r}_2) + m_2 v_x r_2 = F_{y1f} + F_{y1r} + F_{y2r}
   $$
   
   提取加速度项，并把惯性离心力移至等式右侧：
   
   $$
   (m_1 + m_2)\dot{v}_{y1} - m_2 d_1 \dot{r}_1 - m_2 a_2 \dot{r}_2 = F_{y1f} + F_{y1r} + F_{y2r} - (m_1 r_1 + m_2 r_2)v_x \quad \text{--- (17)}
   $$
   
2. **行 2（绕卡车质心的力矩平衡）**：由式 (13) 解出 $F_{hy} = F_{y2r} - m_2(\dot{v}_{y2} + v_x r_2)$，代入卡车力矩式 (12)：
   $$
   I_{z1} \dot{r}_1 = a_1 F_{y1f} - b_1 F_{y1r} + d_1 \left[ - F_{y2r} + m_2(\dot{v}_{y1} - d_1 \dot{r}_1 - a_2 \dot{r}_2 + v_x r_2) \right]
   $$
   
   展开并移项整理：
   
   $$
   -m_2 d_1 \dot{v}_{y1} + (I_{z1} + m_2 d_1^2)\dot{r}_1 + m_2 d_1 a_2 \dot{r}_2 = a_1 F_{y1f} - b_1 F_{y1r} - d_1 F_{y2r} + m_2 d_1 v_x r_2 \quad \text{--- (18)}
   $$
   
3. **行 3（绕拖车质心的力矩平衡）**：同理，将 $F_{hy}$ 代入拖车力矩式 (14)：
   $$
   I_{z2} \dot{r}_2 = -b_2 F_{y2r} + a_2 \left[ - F_{y2r} + m_2(\dot{v}_{y1} - d_1 \dot{r}_1 - a_2 \dot{r}_2 + v_x r_2) \right]
   $$
   
   展开并移项整理：
   
   $$
   -m_2 a_2 \dot{v}_{y1} + m_2 a_2 d_1 \dot{r}_1 + (I_{z2} + m_2 a_2^2)\dot{r}_2 = -(a_2 + b_2)F_{y2r} + m_2 a_2 v_x r_2 \quad \text{--- (19)}
   $$



#### 4. 轮胎线性力模型

在线性区内，轮胎侧向力与侧偏角成正比：

$$
F_{y1f} = C_{1f} \left( \delta - \frac{v_{y1} + a_1 r_1}{v_x} \right), \quad F_{y1r} = C_{1r} \left( -\frac{v_{y1} - b_1 r_1}{v_x} \right)
$$

$$
F_{y2r} = C_{2r} \alpha_{2r} = C_{2r} \left( -\frac{v_{y2} - b_2 r_2}{v_x} \right) = C_{2r} \left( -\frac{v_{y1} - d_1 r_1 - (a_2 + b_2)r_2}{v_x} \right)
$$

#### 5. 消除铰接力，构建动力学状态空间

$$
(m_1 + m_2)\dot{v}_{y1} - m_2 d_1 \dot{r}_1 - m_2 a_2 \dot{r}_2 = F_{y1f} + F_{y1r} + F_{y2r} - (m_1 r_1 + m_2 r_2)v_x \quad \text{--- (17)}
$$

$$
-m_2 d_1 \dot{v}_{y1} + (I_{z1} + m_2 d_1^2)\dot{r}_1 + m_2 d_1 a_2 \dot{r}_2 = a_1 F_{y1f} - b_1 F_{y1r} - d_1 F_{y2r} + m_2 d_1 v_x r_2 \quad \text{--- (18)}
$$

$$
-m_2 a_2 \dot{v}_{y1} + m_2 a_2 d_1 \dot{r}_1 + (I_{z2} + m_2 a_2^2)\dot{r}_2 = -(a_2 + b_2)F_{y2r} + m_2 a_2 v_x r_2 \quad \text{--- (19)}
$$

将三个轮胎力表达式代入式 (17), (18), (19) 的右侧，并分离出状态量 $[v_{y1}, r_1, r_2]^T$ 与控制量 $\delta$。可以精确写为带有**对称质量矩阵**的形式：

$$
M \begin{bmatrix} \dot{v}_{y1} \\ \dot{r}_1 \\ \dot{r}_2 \end{bmatrix} = K \begin{bmatrix} v_{y1} \\ r_1 \\ r_2 \end{bmatrix} + G \delta
$$

其中**质量矩阵 $M$** 为：

$$
M = \begin{bmatrix}  m_1 + m_2 & -m_2 d_1 & -m_2 a_2 \\  -m_2 d_1 & I_{z1} + m_2 d_1^2 & m_2 d_1 a_2 \\  -m_2 a_2 & m_2 d_1 a_2 & I_{z2} + m_2 a_2^2  \end{bmatrix}
$$

**刚度/阻尼矩阵 $K$** 内部各元素的细化合并过程如下（令 $L_{2\_total} = a_2 + b_2$）：
$$
K = \begin{bmatrix} -\frac{C_{1f}+C_{1r}+C_{2r}}{v_x} & -\frac{a_1 C_{1f} - b_1 C_{1r} - d_1 C_{2r}}{v_x} - m_1 v_x & \frac{L_{2\_total} C_{2r}}{v_x} - m_2 v_x \\ 
-\frac{a_1 C_{1f} - b_1 C_{1r} - d_1 C_{2r}}{v_x} & -\frac{a_1^2 C_{1f} + b_1^2 C_{1r} + d_1^2 C_{2r}}{v_x} & -\frac{d_1 L_{2\_total} C_{2r}}{v_x} + m_2 d_1 v_x \\ 
\frac{L_{2\_total} C_{2r}}{v_x} & - \frac{d_1 L_{2\_total} C_{2r}}{v_x} & -\frac{L_{2\_total}^2 C_{2r}}{v_x} + m_2 a_2 v_x \end{bmatrix}
$$

**控制输入矩阵 $G$** 为：
$$
G = \begin{bmatrix} C_{1f} & a_1 C_{1f} & 0 \end{bmatrix}^T
$$

通过左乘质量矩阵的逆 $M^{-1}$，我们得到了纯动力学的标准状态空间方程，$\dot{x}_d = A_d x_d + B_d \delta$，状态量为 $[v_{y1}, r_1, r_2]^T$。

$$
\begin{bmatrix} \dot{v}_{y1} \\ \dot{r}_1 \\ \dot{r}_2 \end{bmatrix} = M^{-1}K \begin{bmatrix} v_{y1} \\ r_1 \\ r_2 \end{bmatrix} + M^{-1}G \delta \implies \begin{bmatrix} \dot{v}_{y1} \\ \dot{r}_1 \\ \dot{r}_2 \end{bmatrix} = A_{dyn} \begin{bmatrix} v_{y1} \\ r_1 \\ r_2 \end{bmatrix} + B_{dyn} \delta \quad \text{--- (20)}
$$

(为简化后续书写，记 $A_{dyn} = M^{-1}K$ 的元素为 $a_{ij}$，$B_{dyn} = M^{-1}G$ 的元素为 $b_{i1}$)



---



### 三、 面向横向控制的运动学-动力学融合模型（误差状态空间）

在自动驾驶跟踪控制（如基于 MPC 或 LQR 的车道保持）中，控制器的目标是消除**横向距离误差 $e_y$** 和**航向角误差 $e_\psi$**。因此，必须将运动学中的“几何跟踪误差”与上述“车辆动力学”结合。

设定车辆参考路径的局部曲率为 $\rho$。以**卡车质心**作为路径跟踪的基准点：

为了在 LQR/MPC 中进行路径跟踪，控制状态必须转换为**横向误差 $e_y$**、**航向误差 $e_\psi$** 以及**铰接角 $\theta_{12}$**。

#### 步骤 1：建立误差状态量与动力学状态量的物理转换

设期望路径的局部曲率为 $\rho$。根据跟踪几何学，误差导数与车身速度的关系为：

1. 横向距离误差变化率：$\dot{e}_y = v_{y1} + v_x e_\psi \implies v_{y1} = \dot{e}_y - v_x e_\psi \quad \text{--- (21)}$
2. 对式 (21) 求导：$\dot{v}_{y1} = \ddot{e}_y - v_x \dot{e}_\psi \quad \text{--- (22)}$（这里设立纵向速度恒定）
3. 航向角误差变化率：$\dot{e}_\psi = r_1 - v_x \rho \implies r_1 = \dot{e}_\psi + v_x \rho \quad \text{--- (23)}$
4. 对式 (23) 求导（假设曲率变化率 $\dot{\rho} \approx 0$）：$\dot{r}_1 = \ddot{e}_\psi \quad \text{--- (24)}$
5. 铰接角速度定义：$\dot{\theta}_{12} = r_1 - r_2 \implies r_2 = r_1 - \dot{\theta}_{12} = \dot{e}_\psi + v_x \rho - \dot{\theta}_{12} \quad \text{--- (25)}$
6. 对式 (25) 求导：$\dot{r}_2 = \ddot{e}_\psi - \ddot{\theta}_{12} \quad \text{--- (26)}$

将式 (23) 代入式 (22) 中，可得关键的加速度替换式：

$$
\dot{v}_{y1} = \ddot{e}_y - v_x(r_1 - v_x \rho) = \ddot{e}_y - v_x r_1 + v_x^2 \rho \quad \text{--- (27)}
$$

#### 步骤 2：全过程代数展开与组装

我们将上述所有转换式 (21)~(27) 依次代入动力学方程组式 (20) 的三条标量方程中：

##### 1. 推导状态 $\ddot{e}_y$ 对应的方程：

将式 (27) 代入式 (20) 的第一行：

$$
\ddot{e}_y - v_x r_1 + v_x^2 \rho = a_{11} v_{y1} + a_{12} r_1 + a_{13} r_2 + b_{11} \delta
$$

代入 $v_{y1}, r_1, r_2$ 的物理转换：

$$
\ddot{e}_y - v_x(\dot{e}_\psi + v_x \rho) + v_x^2 \rho = a_{11}(\dot{e}_y - v_x e_\psi) + a_{12}(\dot{e}_\psi + v_x \rho) + a_{13}(\dot{e}_\psi + v_x \rho - \dot{\theta}_{12}) + b_{11} \delta
$$

展开左边和右边：

$$
\ddot{e}_y - v_x \dot{e}_\psi = a_{11}\dot{e}_y - a_{11}v_x e_\psi + a_{12}\dot{e}_\psi + a_{12}v_x \rho + a_{13}\dot{e}_\psi + a_{13}v_x \rho - a_{13}\dot{\theta}_{12} + b_{11} \delta
$$

移项，将 $\ddot{e}_y$ 孤立在左侧：

$$
\ddot{e}_y = a_{11}\dot{e}_y - a_{11}v_x e_\psi + (a_{12} + a_{13} + v_x)\dot{e}_\psi - a_{13}\dot{\theta}_{12} + b_{11} \delta + (a_{12} + a_{13})v_x \rho \quad \text{--- (28)}
$$

##### 2. 推导状态 $\ddot{e}_\psi$ 对应的方程：

将式 (24) 代入式 (20) 的第二行：

$$
\ddot{e}_\psi = a_{21} v_{y1} + a_{22} r_1 + a_{23} r_2 + b_{21} \delta
$$

代入转换关系：

$$
\ddot{e}_\psi = a_{21}(\dot{e}_y - v_x e_\psi) + a_{22}(\dot{e}_\psi + v_x \rho) + a_{23}(\dot{e}_\psi + v_x \rho - \dot{\theta}_{12}) + b_{21} \delta
$$

合并相同状态项：

$$
\ddot{e}_\psi = a_{21}\dot{e}_y - a_{21}v_x e_\psi + (a_{22} + a_{23})\dot{e}_\psi - a_{23}\dot{\theta}_{12} + b_{21} \delta + (a_{22} + a_{23})v_x \rho \quad \text{--- (29)}
$$

##### 3. 推导状态 $\ddot{\theta}_{12}$ 对应的方程：

利用定义 $\ddot{\theta}_{12} = \dot{r}_1 - \dot{r}_2$，将式 (20) 的第二行减去第三行：

$$
\ddot{\theta}_{12} = (a_{21} - a_{31})v_{y1} + (a_{22} - a_{32})r_1 + (a_{23} - a_{33})r_2 + (b_{21} - b_{31})\delta
$$

代入转换关系：

$$
\ddot{\theta}_{12} = (a_{21} - a_{31})(\dot{e}_y - v_x e_\psi) + (a_{22} - a_{32})(\dot{e}_\psi + v_x \rho) + (a_{23} - a_{33})(\dot{e}_\psi + v_x \rho - \dot{\theta}_{12}) + (b_{21} - b_{31})\delta
$$

合并相同状态项：

$$
\ddot{\theta}_{12} = (a_{21} - a_{31})\dot{e}_y - (a_{21} - a_{31})v_x e_\psi + \left[(a_{22} - a_{32}) + (a_{23} - a_{33})\right]\dot{e}_\psi \\- (a_{23} - a_{33})\dot{\theta}_{12} + (b_{21} - b_{31})\delta + \left[(a_{22} - a_{32}) + (a_{23} - a_{33})\right]v_x \rho \quad \text{--- (30)}
$$

#### 步骤 3：构建标准控制状态空间矩阵

选择控制状态向量 $X = \begin{bmatrix} e_y & \dot{e}_y & e_\psi & \dot{e}_\psi & \theta_{12} & \dot{\theta}_{12} \end{bmatrix}^T$。

结合一阶恒等式（如 $\frac{d}{dt}(e_y) = \dot{e}_y$）与推导出的二阶微分方程 (28), (29), (30)，完美写成标准控制矩阵形式：

$$\dot{X} = A X + B_1 \delta + B_2 \rho$$

各矩阵内部完整元素结构如下：

$$
A = \begin{bmatrix}  0 & 1 & 0 & 0 & 0 & 0 \\  0 & a_{11} & -a_{11}v_x & a_{12}+a_{13}+v_x & 0 & -a_{13} \\  0 & 0 & 0 & 1 & 0 & 0 \\  0 & a_{21} & -a_{21}v_x & a_{22}+a_{23} & 0 & -a_{23} \\  0 & 0 & 0 & 0 & 0 & 1 \\  0 & a_{21}-a_{31} & -(a_{21}-a_{31})v_x & (a_{22}-a_{32})+(a_{23}-a_{33}) & 0 & -(a_{23}-a_{33})  \end{bmatrix}
$$

$$
B_1 = \begin{bmatrix} 0 \\ b_{11} \\ 0 \\ b_{21} \\ 0 \\ b_{21}-b_{31} \end{bmatrix}, \quad  B_2 = \begin{bmatrix}  0 \\  (a_{12}+a_{13})v_x \\  0 \\  (a_{22}+a_{23})v_x \\  0 \\  (a_{22}-a_{32}+a_{23}-a_{33})v_x  \end{bmatrix}
$$

**推导完毕。** 该模型无缝融合了底层车辆的物理动力学与上层路径几何学，可直接用于 LQR 的代数里卡蒂方程求解或 MPC 的二次规划控制。



### 总结与控制应用建议

1. **LQR / MPC 控制器输入**：有了上述 $\dot{X} = AX + B_1\delta + B_2\rho$ 模型后，你可以直接将其离散化，用于编写 **MPC（模型预测控制）** 或 **LQR（线性二次型调节器）**。
2. **前馈控制 $B_2 \rho$**：曲率 $\rho$ 项作为外部已知干扰，可以用作**曲率前馈**。前轮转向角控制量最终可表示为：$\delta = \delta_{feedback}(LQR/MPC) + \delta_{feedforward}(\rho)$。
3. **防止折头（Jack-knife）**：在该融合动力学模型中，状态量包含 $\theta_{12}$ 及其变化率 $\dot{\theta}_{12}$。在 MPC 的约束条件（Constraints）中限制 $\theta_{12}$ 的上下界（例如 $< 45^\circ$），即可在动力学层面上完美避免重型卡车在换道或转弯时发生灾难性的折头危险。