# 卡车—半挂车模型：关键公式与完整矩阵

本文给出卡车—半挂车单轨模型的统一符号、关键假设、关键公式和完整矩阵。重点修正
铰接点速度约束：卡车与拖车的横向速度分别定义在各自车体坐标系中，线性约束必须
保留纵向速度投影 \(U\phi\)。

更详细的逐式推导见
[`2_truck_model_derivation.md`](2_truck_model_derivation.md)。

---

## 1. 坐标、符号和假设

### 1.1 二维坐标与正方向

- 全局坐标为 \(X,Y\)；
- 每个车体的纵向为前方，横向为左方；
- 航向角逆时针为正；
- 卡车航向为 \(\theta_1\)，拖车航向为 \(\theta_2\)；
- 铰接角
  \[
  \boxed{\phi=\theta_1-\theta_2};
  \]
- 偏航角速度
  \[
  \boxed{r_1=\dot\theta_1,\qquad r_2=\dot\theta_2}.
  \]

全文只建立二维平面模型。\(I_1,I_2\) 分别表示两车的平面偏航转动惯量。

### 1.2 参数

| 符号 | 含义 | SI 单位 |
|---|---|---|
| \(m_1,m_2\) | 卡车、拖车质量 | \(\mathrm{kg}\) |
| \(I_1,I_2\) | 卡车、拖车平面偏航转动惯量 | \(\mathrm{kg\,m^2}\) |
| \(a_1,b_1\) | 卡车质心到前轴、后轴距离 | \(\mathrm m\) |
| \(d_1\) | 卡车质心到铰接点的向后距离 | \(\mathrm m\) |
| \(a_2,b_2\) | 拖车质心到铰接点、拖车轴距离 | \(\mathrm m\) |
| \(C_{1f},C_{1r},C_{2r}\) | 三个整轴侧偏刚度 | \(\mathrm{N/rad}\) |
| \(U_1\) | 精确运动学中卡车后轴中心的前向速度 | \(\mathrm{m/s}\) |
| \(U\) | 线性动力学中两车直线平衡点的共同恒定名义纵速 | \(\mathrm{m/s}\) |
| \(\delta\) | 卡车前轮转角 | \(\mathrm{rad}\) |
| \(H\) | 拖车作用于卡车的铰接横向力 | \(\mathrm N\) |

定义：

\[
\boxed{
L_1=a_1+b_1,\qquad
L_2=a_2+b_2,\qquad
\ell_h=b_1-d_1.
}
\tag{K1}
\]

### 1.3 模型层次

1. **非线性运动学**：纯滚动，以卡车后轴前向速度 \(U_1\) 为输入，可保留
   \(\tan\delta,\sin\phi,\cos\phi\)；
2. **线性横向动力学**：\(U>0\) 恒定，小 \(\delta,\phi,\alpha\)，线性轮胎；
3. **误差状态模型**：在线性横向动力学上叠加时间参数化参考轨迹。

动力学模型不包含纵向加减速、强制动/驱动、轮胎饱和、载荷转移和大铰接角。

---

## 2. 精确非线性运动学

铰接点记为 \(P_h\)，其全局坐标记为 \((X_h,Y_h)\)。选择铰接点位置和两车航向作为状态：

\[
\boxed{
\boldsymbol x_{\mathrm{kin}}=
\begin{bmatrix}
X_h&Y_h&\theta_1&\theta_2
\end{bmatrix}^{T}.
}
\tag{K2}
\]

卡车偏航角速度：

\[
\boxed{
r_1=\frac{U_1}{L_1}\tan\delta.
}
\tag{K3}
\]

铰接点在卡车坐标系中的速度分量为：

\[
\boxed{
\begin{bmatrix}
v_{P_h,x1}\\v_{P_h,y1}
\end{bmatrix}
=
\begin{bmatrix}
U_1\\\ell_hr_1
\end{bmatrix}.
}
\tag{K4}
\]

非线性运动学方程：

\[
\boxed{
\begin{aligned}
\dot X_h&=U_1\cos\theta_1-\ell_hr_1\sin\theta_1,\\
\dot Y_h&=U_1\sin\theta_1+\ell_hr_1\cos\theta_1,\\
\dot\theta_1&=r_1,\\
\dot\theta_2&=
\frac{U_1\sin\phi+\ell_hr_1\cos\phi}{L_2},\\
\dot\phi&=
r_1-\frac{U_1\sin\phi+\ell_hr_1\cos\phi}{L_2}.
\end{aligned}}
\tag{K5}
\]

当铰接点位于卡车后轴中心，即 \(d_1=b_1,\ell_h=0\)：

\[
\boxed{
\begin{aligned}
\dot X_h&=U_1\cos\theta_1,\\
\dot Y_h&=U_1\sin\theta_1,\\
\dot\theta_1&=\frac{U_1}{L_1}\tan\delta,\\
\dot\theta_2&=\frac{U_1}{L_2}\sin\phi,\\
\dot\phi&=\frac{U_1}{L_1}\tan\delta-\frac{U_1}{L_2}\sin\phi.
\end{aligned}}
\tag{K6}
\]

精确运动学中，拖车轴沿自身前向的速度为
\(U_2=U_1\cos\phi-\ell_hr_1\sin\phi\)，一般不等于 \(U_1\)。
在线性横向动力学中，二者在直线平衡点附近满足
\(U_1=U_2=U+O(\varepsilon^2)\)，因此后文统一使用名义纵速 \(U\)。

---

## 3. 线性横向动力学基础

### 3.1 轮胎侧偏角与横向力

定义 \(v_{y1}\) 和 \(v_{y2}\) 分别为卡车、拖车质心速度在各自车体左向上的分量；
\(r_1,r_2\) 分别为两车逆时针为正的偏航角速度。

本文定义侧偏角为“车轮指向角减去接地点速度方向角”，并采用
\(F_y=C_\alpha\alpha\)：

\[
\boxed{
\begin{aligned}
\alpha_{1f}
&=\delta-\frac{v_{y1}+a_1r_1}{U},
&F_{1f}&=C_{1f}\alpha_{1f},\\
\alpha_{1r}
&=-\frac{v_{y1}-b_1r_1}{U},
&F_{1r}&=C_{1r}\alpha_{1r},\\
\alpha_{2r}
&=-\frac{v_{y2}-b_2r_2}{U},
&F_{2r}&=C_{2r}\alpha_{2r}.
\end{aligned}}
\tag{K7}
\]

### 3.2 两车受力与偏航方程

小转角下 \(\cos\delta\approx1\)，忽略纵向铰接力对横向子系统的作用：

\[
\boxed{
\begin{aligned}
m_1(\dot v_{y1}+Ur_1)&=F_{1f}+F_{1r}+H,\\
I_1\dot r_1&=a_1F_{1f}-b_1F_{1r}-d_1H,\\
m_2(\dot v_{y2}+Ur_2)&=F_{2r}-H,\\
I_2\dot r_2&=-b_2F_{2r}-a_2H.
\end{aligned}}
\tag{K8}
\]

---

## 4. 正确的铰接点速度约束

精确的横向投影关系为：

\[
\boxed{
v_{y2}+a_2r_2
=U_1\sin\phi+(v_{y1}-d_1r_1)\cos\phi.
}
\tag{K9}
\]

在线性动力学平衡点附近取 \(U_1=U+O(\varepsilon^2)\)，一阶线性化得到：

\[
\boxed{
v_{y1}-d_1r_1-v_{y2}-a_2r_2+U\phi=0.
}
\tag{K10}
\]

因此：

\[
\boxed{
v_{y2}=v_{y1}-d_1r_1-a_2r_2+U\phi.
}
\tag{K11}
\]

在 \(U\) 恒定时，对式 (K10) 求导：

\[
\boxed{
\dot v_{y1}-d_1\dot r_1-\dot v_{y2}-a_2\dot r_2
+U(r_1-r_2)=0.
}
\tag{K12}
\]

即：

\[
\boxed{
\dot v_{y2}
=\dot v_{y1}-d_1\dot r_1-a_2\dot r_2+U(r_1-r_2).
}
\tag{K13}
\]

式 (K10) 中的 \(U\phi\) 与式 (K12) 中的 \(U(r_1-r_2)\) 都是一阶项，
不能在小角度线性化中删除。

将式 (K11) 代入拖车轮胎力：

\[
\boxed{
\begin{aligned}
F_{2r}
&=C_{2r}\left[
-\frac{v_{y1}-d_1r_1-L_2r_2+U\phi}{U}
\right]\\
&=-\frac{C_{2r}}U v_{y1}
+\frac{d_1C_{2r}}U r_1
+\frac{L_2C_{2r}}U r_2
-C_{2r}\phi.
\end{aligned}}
\tag{K14}
\]

---

## 5. 直接消除铰接力：完整矩阵

定义三维速度变量：

\[
\boxed{
\boldsymbol\eta=
\begin{bmatrix}
v_{y1}&r_1&r_2
\end{bmatrix}^{T}.
}
\tag{K15}
\]

利用式 (K13) 消除 \(\dot v_{y2}\)，并利用拖车横向方程消除 \(H\)，得到：

\[
\boxed{
M_e\dot{\boldsymbol\eta}
=K_e\boldsymbol\eta+\boldsymbol k_\phi\phi+\boldsymbol G_e\delta.
}
\tag{K16}
\]

### 5.1 有效质量矩阵

\[
\boxed{
M_e=
\begin{bmatrix}
m_1+m_2&-m_2d_1&-m_2a_2\\
-m_2d_1&I_1+m_2d_1^2&m_2d_1a_2\\
-m_2a_2&m_2d_1a_2&I_2+m_2a_2^2
\end{bmatrix}.
}
\tag{K17}
\]

### 5.2 完整速度系数矩阵

\[
\boxed{
K_e=
\begin{bmatrix}
-\dfrac{C_{1f}+C_{1r}+C_{2r}}U
&
\dfrac{-a_1C_{1f}+b_1C_{1r}+d_1C_{2r}}U-(m_1+m_2)U
&
\dfrac{L_2C_{2r}}U
\\[3mm]
\dfrac{-a_1C_{1f}+b_1C_{1r}+d_1C_{2r}}U
&
-\dfrac{a_1^2C_{1f}+b_1^2C_{1r}+d_1^2C_{2r}}U+m_2d_1U
&
-\dfrac{d_1L_2C_{2r}}U
\\[3mm]
\dfrac{L_2C_{2r}}U
&
-\dfrac{d_1L_2C_{2r}}U+m_2a_2U
&
-\dfrac{L_2^2C_{2r}}U
\end{bmatrix}.
}
\tag{K18}
\]

### 5.3 铰接角列与输入列

\[
\boxed{
\boldsymbol k_\phi=C_{2r}
\begin{bmatrix}
-1\\d_1\\L_2
\end{bmatrix},
\qquad
\boldsymbol G_e=
\begin{bmatrix}
C_{1f}\\a_1C_{1f}\\0
\end{bmatrix}.
}
\tag{K19}
\]

### 5.4 四状态质量矩阵形式

定义完整动力学状态：

\[
\boxed{
\boldsymbol x_p=
\begin{bmatrix}
v_{y1}&r_1&r_2&\phi
\end{bmatrix}^{T}.
}
\tag{K20}
\]

\[
\boxed{
\mathcal M\dot{\boldsymbol x}_p
=\mathcal K\boldsymbol x_p+\mathcal G\delta,
}
\tag{K21}
\]

其中：

\[
\boxed{
\mathcal M=
\begin{bmatrix}
m_1+m_2&-m_2d_1&-m_2a_2&0\\
-m_2d_1&I_1+m_2d_1^2&m_2d_1a_2&0\\
-m_2a_2&m_2d_1a_2&I_2+m_2a_2^2&0\\
0&0&0&1
\end{bmatrix},
}
\tag{K22}
\]

\[
\boxed{
\mathcal K=
\begin{bmatrix}
(K_e)_{11}&(K_e)_{12}&(K_e)_{13}&-C_{2r}\\
(K_e)_{21}&(K_e)_{22}&(K_e)_{23}&d_1C_{2r}\\
(K_e)_{31}&(K_e)_{32}&(K_e)_{33}&L_2C_{2r}\\
0&1&-1&0
\end{bmatrix},
\qquad
\mathcal G=
\begin{bmatrix}
C_{1f}\\a_1C_{1f}\\0\\0
\end{bmatrix}.
}
\tag{K23}
\]

第四行就是 \(\dot\phi=r_1-r_2\)。

---

## 6. 标准四状态模型：完整展开方法

定义：

\[
\boxed{
A_v=M_e^{-1}K_e=
\begin{bmatrix}
a_{11}&a_{12}&a_{13}\\
a_{21}&a_{22}&a_{23}\\
a_{31}&a_{32}&a_{33}
\end{bmatrix},
}
\tag{K24}
\]

\[
\boxed{
\boldsymbol a_\phi=M_e^{-1}\boldsymbol k_\phi=
\begin{bmatrix}
a_{14}\\a_{24}\\a_{34}
\end{bmatrix},
\qquad
\boldsymbol\beta=M_e^{-1}\boldsymbol G_e=
\begin{bmatrix}
\beta_1\\\beta_2\\\beta_3
\end{bmatrix}.
}
\tag{K25}
\]

标准模型：

\[
\boxed{
\dot{\boldsymbol x}_p=A_p\boldsymbol x_p+B_p\delta,
}
\tag{K26}
\]

\[
\boxed{
A_p=
\begin{bmatrix}
a_{11}&a_{12}&a_{13}&a_{14}\\
a_{21}&a_{22}&a_{23}&a_{24}\\
a_{31}&a_{32}&a_{33}&a_{34}\\
0&1&-1&0
\end{bmatrix},
\qquad
B_p=
\begin{bmatrix}
\beta_1\\\beta_2\\\beta_3\\0
\end{bmatrix}.
}
\tag{K27}
\]

### 6.1 \(M_e^{-1}\) 的完整形式

令：

\[
\begin{aligned}
\mu_{11}&=m_1+m_2,&
\mu_{12}&=-m_2d_1,&
\mu_{13}&=-m_2a_2,\\
\mu_{22}&=I_1+m_2d_1^2,&
\mu_{23}&=m_2d_1a_2,&
\mu_{33}&=I_2+m_2a_2^2.
\end{aligned}
\tag{K28}
\]

\[
\boxed{
\begin{aligned}
\Delta={}&
\mu_{11}(\mu_{22}\mu_{33}-\mu_{23}^2)
-\mu_{12}(\mu_{12}\mu_{33}-\mu_{13}\mu_{23})\\
&+\mu_{13}(\mu_{12}\mu_{23}-\mu_{13}\mu_{22}).
\end{aligned}}
\tag{K29}
\]

式 (K29) 可化简为
\[
\Delta=(m_1+m_2)I_1I_2
+m_1m_2\left(I_1a_2^2+I_2d_1^2\right)>0,
\]
其中 \(m_1,m_2,I_1,I_2>0\)。因此 \(M_e\) 可逆。

\[
\boxed{
M_e^{-1}=
\begin{bmatrix}
q_{11}&q_{12}&q_{13}\\
q_{12}&q_{22}&q_{23}\\
q_{13}&q_{23}&q_{33}
\end{bmatrix},
}
\tag{K30}
\]

\[
\boxed{
\begin{aligned}
q_{11}&=\frac{\mu_{22}\mu_{33}-\mu_{23}^2}{\Delta},&
q_{12}&=\frac{\mu_{13}\mu_{23}-\mu_{12}\mu_{33}}{\Delta},\\
q_{13}&=\frac{\mu_{12}\mu_{23}-\mu_{13}\mu_{22}}{\Delta},&
q_{22}&=\frac{\mu_{11}\mu_{33}-\mu_{13}^2}{\Delta},\\
q_{23}&=\frac{\mu_{12}\mu_{13}-\mu_{11}\mu_{23}}{\Delta},&
q_{33}&=\frac{\mu_{11}\mu_{22}-\mu_{12}^2}{\Delta}.
\end{aligned}}
\tag{K31}
\]

### 6.2 \(A_p,B_p\) 每个元素

对 \(i=1,2,3\)：

\[
\boxed{
\begin{aligned}
a_{i1}&=q_{i1}(K_e)_{11}+q_{i2}(K_e)_{21}+q_{i3}(K_e)_{31},\\
a_{i2}&=q_{i1}(K_e)_{12}+q_{i2}(K_e)_{22}+q_{i3}(K_e)_{32},\\
a_{i3}&=q_{i1}(K_e)_{13}+q_{i2}(K_e)_{23}+q_{i3}(K_e)_{33},\\
a_{i4}&=C_{2r}(-q_{i1}+d_1q_{i2}+L_2q_{i3}),\\
\beta_i&=C_{1f}(q_{i1}+a_1q_{i2}).
\end{aligned}}
\tag{K32}
\]

其中对称下标约定为
\(q_{21}=q_{12},q_{31}=q_{13},q_{32}=q_{23}\)。

---

## 7. 六状态路径误差模型

令时间参考点 \(\boldsymbol p_{\mathrm{ref}}(t)\) 以速度 \(U\) 沿参考切向运动，
\(\boldsymbol n_{\mathrm{ref}}\) 为参考左向单位向量。横向误差以卡车质心为参考：
\[
e_y=\boldsymbol n_{\mathrm{ref}}^T
\left(\boldsymbol p_{O_1}-\boldsymbol p_{\mathrm{ref}}\right).
\]
这里忽略纵向匹配误差，采用时间参数化的小误差模型，而不是严格最近点 Frenet 模型。
定义左转为正的参考曲率 \(\rho=\rho(t)\in C^1\)，
\(\dot\rho=d\rho/dt\)。于是：

\[
\boxed{
e_\psi=\theta_1-\theta_{\mathrm{ref}},
\qquad
\dot\theta_{\mathrm{ref}}=U\rho.
}
\tag{K33}
\]

时间参数化参考轨迹的小误差运动学：

\[
\boxed{
\dot e_y=v_{y1}+Ue_\psi,
\qquad
\dot e_\psi=r_1-U\rho.
}
\tag{K34}
\]

控制状态：

\[
\boxed{
\boldsymbol x_c=
\begin{bmatrix}
e_y&\dot e_y&e_\psi&\dot e_\psi&\phi&\dot\phi
\end{bmatrix}^{T}.
}
\tag{K35}
\]

物理状态转换：

\[
\boxed{
\boldsymbol x_p=T\boldsymbol x_c+\boldsymbol t_\rho\rho,
}
\tag{K36}
\]

\[
\boxed{
T=
\begin{bmatrix}
0&1&-U&0&0&0\\
0&0&0&1&0&0\\
0&0&0&1&0&-1\\
0&0&0&0&1&0
\end{bmatrix},
\qquad
\boldsymbol t_\rho=
\begin{bmatrix}
0\\U\\U\\0
\end{bmatrix}.
}
\tag{K37}
\]

若物理矩阵使用式 (K27) 的 \(a_{ij},\beta_i\)，则：

\[
\boxed{
\dot{\boldsymbol x}_c
=A_c\boldsymbol x_c+B_c\delta
+E_\rho\rho+E_{\dot\rho}\dot\rho.
}
\tag{K38}
\]

\[
\boxed{
A_c=
\begin{bmatrix}
0&1&0&0&0&0\\
0&a_{11}&-Ua_{11}&a_{12}+a_{13}+U&a_{14}&-a_{13}\\
0&0&0&1&0&0\\
0&a_{21}&-Ua_{21}&a_{22}+a_{23}&a_{24}&-a_{23}\\
0&0&0&0&0&1\\
0&a_{21}-a_{31}&-U(a_{21}-a_{31})&
a_{22}-a_{32}+a_{23}-a_{33}&
a_{24}-a_{34}&-a_{23}+a_{33}
\end{bmatrix}.
}
\tag{K39}
\]

\[
\boxed{
B_c=
\begin{bmatrix}
0\\\beta_1\\0\\\beta_2\\0\\\beta_2-\beta_3
\end{bmatrix}.
}
\tag{K40}
\]

\[
\boxed{
E_\rho=
U\begin{bmatrix}
0\\
a_{12}+a_{13}\\
0\\
a_{22}+a_{23}\\
0\\
a_{22}-a_{32}+a_{23}-a_{33}
\end{bmatrix},
\qquad
E_{\dot\rho}=
\begin{bmatrix}
0\\0\\0\\-U\\0\\0
\end{bmatrix}.
}
\tag{K41}
\]

