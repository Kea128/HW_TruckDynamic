# 卡车—半挂车模型：沿"铰接力不做功"方向列方程的完整推导

本文给出 [`2_truck_model_derivation.md`](2_truck_model_derivation.md)（下称"第 2 篇"）
第 5–8 章的另一条推导路线。

第 2 篇的做法是：先对卡车和拖车分别写 Newton–Euler 方程，显式引入铰接横向力 \(H\)；
再用近似 (D45) 删去铰接纵向力的横向投影，用恒速假设下的约束导数 (D60) 表示拖车加速度；
最后把两车方程相加、代入，消去 \(H\)。

本文的做法是：只沿"铰接力做功为零"的四个方向列方程。铰接力的两个分量
\(F_{hx}\)、\(H\) 从一开始就不出现，不需要任何近似，得到的四条方程对任意铰接角、
任意前轮转角和时变纵速都精确成立。之后再线性化，结果在 \(a_x=0\)、\(F_{2x,0}=0\)
时与第 2 篇完全相同，一般情况下只在铰接角列 \(\boldsymbol k_\phi\) 中多出两项。

符号沿用第 2 篇第 2 章，新符号在第 2 节完整定义。本文公式编号用前缀 W；
四条主方程记为 (E1)–(E4)，其线性化形式记为 (E2L)–(E4L)。

---

## 0. 结论摘要

1. 选四个与铰接相容的虚运动：整车沿卡车纵向平移、整车沿卡车横向平移、
   卡车绕铰接点转动、拖车绕铰接点转动。铰接力在每一个上的总功率都恒为零（第 6 节）。
2. 对应的四条方程 (E1)–(E4) 是精确的非线性平面动力学方程，不含铰接力（第 7、8 节）；
   需要铰接力时可以事后精确回代（8.4 节）。
3. 线性化后，第 2 篇的 \(M_e\)、\(K_e\)、\(\boldsymbol G_e\) 全部保持不变，
   只有铰接角列变为（第 9 节）

\[
\boldsymbol k_\phi=
C_{2r}\begin{bmatrix}-1\\d_1\\L_2\end{bmatrix}
+F_{2x,0}\begin{bmatrix}-1\\d_1\\0\end{bmatrix}
+m_2a_2a_x\begin{bmatrix}0\\0\\1\end{bmatrix}.
\]

4. 第 2 篇是本文在 \(a_x=0\)、\(F_{2x,0}=0\) 时的特例。与第 2 篇的逐项区别见第 10 节，
   数值校核见第 11 节。

---

## 1. 推导目标与模型边界

### 1.1 目标

1. 不使用 (D45)，不假设纵速恒定，推导不含铰接力的精确平面动力学方程；
2. 在直线工况附近线性化，得到可以直接替换第 2 篇式 (D96) 的线性模型；
3. 逐项说明本文推导与第 2 篇推导的区别。

### 1.2 与第 2 篇相同的假设

- 两车均为平面刚体单轨模型；
- 各轴中心和铰接点都位于各自车体的中心线上；
- 轮胎力作用在轴中心；卡车前轮只有侧向力 \(F_{1f}\)，与式 (D37) 相同
  （前轮纵向力的推广见 9.10 节）；
- 线性化阶段采用第 2 篇第 1 章的小量约定和第 7 章的线性轮胎。

### 1.3 放宽的假设

- 卡车纵速 \(U=U(t)\) 可以随时间变化，纵向加速度 \(a_x=\dot U\) 允许是 \(O(1)\) 量；
- 保留卡车后轴纵向力 \(F_{1x}\) 和拖车纵向外力 \(F_{2x}\)；
- 不对铰接力做任何近似；
- 推导精确方程时不假设 \(\phi\)、\(\delta\) 为小量。

---

## 2. 符号、投影关系与二维叉积

### 2.1 沿用的符号

以下符号与第 2 篇完全相同：基向量
\(\boldsymbol e_{x1},\boldsymbol e_{y1},\boldsymbol e_{x2},\boldsymbol e_{y2}\)（式 (D1)–(D4)），
铰接角与偏航角速度 \(\phi=\theta_1-\theta_2\)、\(r_1\)、\(r_2\)（式 (D5)），
基向量导数（式 (D6)），几何长度 \(a_1,b_1,d_1,a_2,b_2\) 与 \(L_1,L_2\)（式 (D7)），
质量 \(m_1,m_2\)，偏航惯量 \(I_1,I_2\)，质心 \(O_1,O_2\)，铰接点 \(P_h\)，
卡车质心横向速度 \(v_{y1}\)。

### 2.2 新引入的符号

**卡车纵速 \(U\)。** 卡车中心线上纵向坐标为 \(x\) 的点，速度为式 (D62)：

\[
\boldsymbol v(x)=U\boldsymbol e_{x1}+(v_{y1}+xr_1)\boldsymbol e_{y1}.
\]

其前向分量 \(\boldsymbol e_{x1}^T\boldsymbol v(x)=U\) 与 \(x\) 无关。所以 \(U\) 既是卡车质心的
纵速，也是后轴中心的纵速，即第 2 篇第 3 章的 \(U_1\)。本文允许 \(U\) 随时间变化，记

\[
\boxed{a_x=\dot U.}
\tag{W1}
\]

**铰接点横向速度 \(v_h\)。** 由式 (D50)，铰接点速度在卡车左向的分量为
\(v_{y1}-d_1r_1\)。定义

\[
\boxed{v_h=v_{y1}-d_1r_1.}
\tag{W2}
\]

**纵向力。**

- \(F_{1x}\)：卡车后轴沿 \(\boldsymbol e_{x1}\) 的纵向力，驱动为正、制动为负；
  卡车自身的滚阻、气阻可以并入其中。
- \(F_{2x}\)：拖车沿 \(\boldsymbol e_{x2}\) 的外部纵向力，包括滚阻、沿车轴方向的气阻和
  拖车制动力，向后为负；作用在拖车轴中心，即拖车中心线上。

**铰接力。** 拖车通过铰接点作用在卡车上的力记为

\[
\boxed{
\boldsymbol\Lambda=F_{hx}\boldsymbol e_{x1}+H\boldsymbol e_{y1},
}
\tag{W3}
\]

它就是式 (D38) 中 \(\boldsymbol F_H^{(1)}=[F_{hx},\ H]^T\) 对应的全局向量。由牛顿第三定律，
卡车在同一点 \(P_h\) 对拖车施加 \(-\boldsymbol\Lambda\)。

### 2.3 两车基向量的四个点积

用式 (D1)–(D4) 的完整分量逐一计算：

\[
\begin{aligned}
\boldsymbol e_{x1}^T\boldsymbol e_{x2}
&=\cos\theta_1\cos\theta_2+\sin\theta_1\sin\theta_2=\cos(\theta_1-\theta_2)=\cos\phi,\\
\boldsymbol e_{x1}^T\boldsymbol e_{y2}
&=-\cos\theta_1\sin\theta_2+\sin\theta_1\cos\theta_2=\sin(\theta_1-\theta_2)=\sin\phi,\\
\boldsymbol e_{y1}^T\boldsymbol e_{x2}
&=-\sin\theta_1\cos\theta_2+\cos\theta_1\sin\theta_2=-\sin(\theta_1-\theta_2)=-\sin\phi,\\
\boldsymbol e_{y1}^T\boldsymbol e_{y2}
&=\sin\theta_1\sin\theta_2+\cos\theta_1\cos\theta_2=\cos(\theta_1-\theta_2)=\cos\phi.
\end{aligned}
\tag{W4}
\]

第二行即式 (D25)，第四行即式 (D26)。

### 2.4 拖车基向量用卡车基向量表示

任意二维向量 \(\boldsymbol w\) 都可以按正交基 \((\boldsymbol e_{x1},\boldsymbol e_{y1})\) 展开：
\(\boldsymbol w=(\boldsymbol e_{x1}^T\boldsymbol w)\boldsymbol e_{x1}
+(\boldsymbol e_{y1}^T\boldsymbol w)\boldsymbol e_{y1}\)。
把 \(\boldsymbol e_{x2}\)、\(\boldsymbol e_{y2}\) 代入并用式 (W4)：

\[
\boxed{
\boldsymbol e_{x2}=\cos\phi\,\boldsymbol e_{x1}-\sin\phi\,\boldsymbol e_{y1},
\qquad
\boldsymbol e_{y2}=\sin\phi\,\boldsymbol e_{x1}+\cos\phi\,\boldsymbol e_{y1}.
}
\tag{W5}
\]

### 2.5 标量叉积的三条性质

第 2 篇式 (D36) 定义了标量叉积 \(\boldsymbol p\times\boldsymbol F=p_xF_y-p_yF_x\)。
本文反复用到它的三条性质。

**性质 1：与坐标系无关。** \(\boldsymbol p\times\boldsymbol F=\det[\boldsymbol p\ \ \boldsymbol F]\)。
若两个向量都改用另一个右手正交系的分量表示，即 \(\boldsymbol p=R\boldsymbol p'\)、
\(\boldsymbol F=R\boldsymbol F'\)，其中 \(R\) 是旋转矩阵、\(\det R=1\)，则

\[
\det[\boldsymbol p\ \ \boldsymbol F]
=\det R\cdot\det[\boldsymbol p'\ \ \boldsymbol F']
=\det[\boldsymbol p'\ \ \boldsymbol F'].
\]

所以叉积可以用任一车体坐标系的分量计算。

**性质 2：基向量的叉积。** 用式 (D1)、(D2)：

\[
\boldsymbol e_{x1}\times\boldsymbol e_{y1}
=\cos\theta_1\cos\theta_1-\sin\theta_1(-\sin\theta_1)=1,
\qquad
\boldsymbol e_{x1}\times\boldsymbol e_{x1}=0 .
\]

拖车基向量同理：\(\boldsymbol e_{x2}\times\boldsymbol e_{y2}=1\)，
\(\boldsymbol e_{x2}\times\boldsymbol e_{x2}=0\)。

**性质 3：中心线上的力臂。** 设 \((\boldsymbol e_x,\boldsymbol e_y)\) 是
\((\boldsymbol e_{x1},\boldsymbol e_{y1})\) 或 \((\boldsymbol e_{x2},\boldsymbol e_{y2})\)，
\(\alpha\) 为标量。把 \(\boldsymbol w\) 按这组基展开，利用叉积的双线性和性质 2：

\[
(\alpha\boldsymbol e_x)\times\boldsymbol w
=\alpha(\boldsymbol e_x^T\boldsymbol w)(\boldsymbol e_x\times\boldsymbol e_x)
+\alpha(\boldsymbol e_y^T\boldsymbol w)(\boldsymbol e_x\times\boldsymbol e_y)
=\alpha\,\boldsymbol e_y^T\boldsymbol w .
\]

\[
\boxed{(\alpha\boldsymbol e_x)\times\boldsymbol w=\alpha\,\boldsymbol e_y^T\boldsymbol w.}
\tag{W6}
\]

即：作用点在中心线上、距参考点 \(\alpha\) 的力，其力矩等于 \(\alpha\) 乘以力在该车左向的分量。

### 2.6 固连向量的导数与力的功率

定义逆时针旋转 \(90^\circ\) 的矩阵

\[
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\tag{W7}
\]

由式 (D1)、(D2) 直接计算：
\(J\boldsymbol e_{x1}=[-\sin\theta_1,\ \cos\theta_1]^T=\boldsymbol e_{y1}\)，
\(J\boldsymbol e_{y1}=[-\cos\theta_1,\ -\sin\theta_1]^T=-\boldsymbol e_{x1}\)。
所以式 (D6) 可以写成 \(\dot{\boldsymbol e}_{x1}=r_1J\boldsymbol e_{x1}\)、
\(\dot{\boldsymbol e}_{y1}=r_1J\boldsymbol e_{y1}\)。

对固连在卡车上的任意向量 \(\boldsymbol q=q_x\boldsymbol e_{x1}+q_y\boldsymbol e_{y1}\)
（\(q_x,q_y\) 为常数）：

\[
\dot{\boldsymbol q}=q_xr_1J\boldsymbol e_{x1}+q_yr_1J\boldsymbol e_{y1}=r_1J\boldsymbol q .
\]

拖车同理，把 \(r_1\) 换成 \(r_2\)。因此刚体上任一点 \(P\) 的速度为

\[
\boldsymbol v_P=\boldsymbol v_O+rJ\boldsymbol q,\qquad \boldsymbol q=\boldsymbol p_P-\boldsymbol p_O,
\tag{W8}
\]

其中 \(O\) 是同一刚体上的参考点，\(r\) 是该刚体的偏航角速度。

再看 \(J\boldsymbol q=[-q_y,\ q_x]^T\)，于是

\[
\boldsymbol F^TJ\boldsymbol q=-F_xq_y+F_yq_x=\boldsymbol q\times\boldsymbol F .
\tag{W9}
\]

把式 (W8)、(W9) 合起来，作用在刚体点 \(P\) 上的力 \(\boldsymbol F\) 的功率为

\[
\boxed{
\boldsymbol F^T\boldsymbol v_P
=\boldsymbol F^T\boldsymbol v_O+r\,(\boldsymbol q\times\boldsymbol F).
}
\tag{W10}
\]

即"力的功率 = 力点乘参考点速度 + 该力对参考点的力矩乘以角速度"。
第 6 节正是用它证明铰接力不做功。

---

## 3. 运动学：用四个独立速度描述两车

### 3.1 自由度

两个平面刚体各有 3 个自由度，共 6 个。铰接销要求两车在 \(P_h\) 处的点重合，
这是 2 个标量约束。所以车组有 \(6-2=4\) 个自由度，需要 4 个独立速度。

### 3.2 独立速度的选取

取

\[
\boxed{
\boldsymbol w=\begin{bmatrix}U\\v_h\\r_1\\r_2\end{bmatrix}.
}
\tag{W11}
\]

\(U\) 与 \(v_h\) 是铰接点速度在卡车坐标系中的两个分量，\(r_1\)、\(r_2\) 是两车偏航角速度。
第 2 篇用的是 \(v_{y1}\) 而不是 \(v_h\)，二者由式 (W2) 一一对应：\(v_{y1}=v_h+d_1r_1\)。
后文最终仍用 \(v_{y1}\) 表达结果。

### 3.3 两车的速度都是 \(\boldsymbol w\) 的线性函数

**铰接点。** 由式 (D50) 和 (W2)：

\[
\boldsymbol v_{P_h}=U\boldsymbol e_{x1}+v_h\boldsymbol e_{y1}.
\tag{W12}
\]

**卡车质心。** 由式 (D49)，\(\boldsymbol p_{O_1}=\boldsymbol p_{P_h}+d_1\boldsymbol e_{x1}\)。
求导并用式 (D6)：

\[
\boldsymbol v_{O_1}=\boldsymbol v_{P_h}+d_1r_1\boldsymbol e_{y1}
=U\boldsymbol e_{x1}+(v_h+d_1r_1)\boldsymbol e_{y1},
\qquad \omega_1=r_1 .
\tag{W13}
\]

**拖车质心。** 由式 (D51)，\(\boldsymbol p_{O_2}=\boldsymbol p_{P_h}-a_2\boldsymbol e_{x2}\)。求导：

\[
\boldsymbol v_{O_2}=\boldsymbol v_{P_h}-a_2r_2\boldsymbol e_{y2}
=U\boldsymbol e_{x1}+v_h\boldsymbol e_{y1}-a_2r_2\boldsymbol e_{y2},
\qquad \omega_2=r_2 .
\tag{W14}
\]

**关键观察。** 式 (W13) 和 (W14) 由同一个 \(\boldsymbol v_{P_h}\) 构造。所以对任意
\(\boldsymbol w\)，从卡车侧算出的铰接点速度与从拖车侧算出的完全相同，铰接约束
（式 (D50) 与 (D52) 相等，其横向分量即式 (D55)）自动满足，不需要再单独列约束方程。
反过来，任何满足铰接约束的运动都对应唯一的 \(\boldsymbol w\)：\((U,v_h)\) 取公共铰接点
速度在卡车系的分量，\(r_1,r_2\) 取两车角速度。

### 3.4 拖车速度分量（供拖车轮胎使用）

把式 (W14) 投影到拖车坐标轴，用式 (W4) 和 \(\boldsymbol e_{x2}^T\boldsymbol e_{y2}=0\)：

\[
\boxed{
U_2=\boldsymbol e_{x2}^T\boldsymbol v_{O_2}=U\cos\phi-v_h\sin\phi,
}
\tag{W15}
\]

\[
\boxed{
v_{y2}=\boldsymbol e_{y2}^T\boldsymbol v_{O_2}=U\sin\phi+v_h\cos\phi-a_2r_2 .
}
\tag{W16}
\]

式 (W16) 就是式 (D55) 移项后的形式。注意这两式都是代数关系，没有求导。

### 3.5 加速度

**卡车质心。** 对式 (W13) 的第二种写法 \(\boldsymbol v_{O_1}=U\boldsymbol e_{x1}+v_{y1}\boldsymbol e_{y1}\)
求导，此时 \(U\) 不再是常数：

\[
\begin{aligned}
\boldsymbol a_{O_1}
&=\dot U\boldsymbol e_{x1}+U\dot{\boldsymbol e}_{x1}
+\dot v_{y1}\boldsymbol e_{y1}+v_{y1}\dot{\boldsymbol e}_{y1}\\
&=\dot U\boldsymbol e_{x1}+Ur_1\boldsymbol e_{y1}
+\dot v_{y1}\boldsymbol e_{y1}-v_{y1}r_1\boldsymbol e_{x1}\\
&=(\dot U-v_{y1}r_1)\boldsymbol e_{x1}+(\dot v_{y1}+Ur_1)\boldsymbol e_{y1}.
\end{aligned}
\tag{W17}
\]

与式 (D31) 相比多出 \(\dot U\boldsymbol e_{x1}\)。

**铰接点。** 对式 (W12) 求导：

\[
\begin{aligned}
\boldsymbol a_{P_h}
&=\dot U\boldsymbol e_{x1}+Ur_1\boldsymbol e_{y1}+\dot v_h\boldsymbol e_{y1}-v_hr_1\boldsymbol e_{x1}\\
&=(\dot U-v_hr_1)\boldsymbol e_{x1}+(\dot v_h+Ur_1)\boldsymbol e_{y1}.
\end{aligned}
\]

由式 (W2)，\(\dot v_h=\dot v_{y1}-d_1\dot r_1\)。定义

\[
\boxed{
a_{hx}=\dot U-v_hr_1,
\qquad
a_{hy}=\dot v_{y1}-d_1\dot r_1+Ur_1,
\qquad
\boldsymbol a_{P_h}=a_{hx}\boldsymbol e_{x1}+a_{hy}\boldsymbol e_{y1}.
}
\tag{W18}
\]

校核：也可以从 \(\boldsymbol v_{P_h}=\boldsymbol v_{O_1}-d_1r_1\boldsymbol e_{y1}\) 求导，
\(\boldsymbol a_{P_h}=\boldsymbol a_{O_1}-d_1\dot r_1\boldsymbol e_{y1}+d_1r_1^2\boldsymbol e_{x1}\)。
代入式 (W17)，\(\boldsymbol e_{x1}\) 分量为
\(\dot U-v_{y1}r_1+d_1r_1^2=\dot U-(v_{y1}-d_1r_1)r_1=a_{hx}\)，
\(\boldsymbol e_{y1}\) 分量为 \(\dot v_{y1}+Ur_1-d_1\dot r_1=a_{hy}\)，两种算法一致。

**拖车质心。** 对式 (W14) 的 \(\boldsymbol v_{O_2}=\boldsymbol v_{P_h}-a_2r_2\boldsymbol e_{y2}\) 求导，
用 \(\dot{\boldsymbol e}_{y2}=-r_2\boldsymbol e_{x2}\)：

\[
\boxed{
\boldsymbol a_{O_2}
=\boldsymbol a_{P_h}-a_2\dot r_2\boldsymbol e_{y2}+a_2r_2^2\boldsymbol e_{x2}.
}
\tag{W19}
\]

这是精确式，\(\dot U\) 通过 \(a_{hx}\) 自然出现在其中。第 2 篇则是先把速度约束线性化成
(D58)，再在 \(U\) 恒定的前提下求导得到 (D60)。

### 3.6 后面要用到的四个加速度投影

逐项用式 (W4) 和 \(\boldsymbol e_{x2}^T\boldsymbol e_{y2}=0\)、
\(\boldsymbol e_{x2}^T\boldsymbol e_{x2}=\boldsymbol e_{y2}^T\boldsymbol e_{y2}=1\)：

\[
\boldsymbol e_{x1}^T\boldsymbol a_{O_2}
=a_{hx}-a_2\dot r_2(\boldsymbol e_{x1}^T\boldsymbol e_{y2})+a_2r_2^2(\boldsymbol e_{x1}^T\boldsymbol e_{x2})
=a_{hx}-a_2\dot r_2\sin\phi+a_2r_2^2\cos\phi,
\tag{W20}
\]

\[
\boldsymbol e_{y1}^T\boldsymbol a_{O_2}
=a_{hy}-a_2\dot r_2(\boldsymbol e_{y1}^T\boldsymbol e_{y2})+a_2r_2^2(\boldsymbol e_{y1}^T\boldsymbol e_{x2})
=a_{hy}-a_2\dot r_2\cos\phi-a_2r_2^2\sin\phi,
\tag{W21}
\]

\[
\boldsymbol e_{x2}^T\boldsymbol a_{O_2}
=a_{hx}(\boldsymbol e_{x2}^T\boldsymbol e_{x1})+a_{hy}(\boldsymbol e_{x2}^T\boldsymbol e_{y1})+a_2r_2^2
=a_{hx}\cos\phi-a_{hy}\sin\phi+a_2r_2^2,
\tag{W22}
\]

\[
\boldsymbol e_{y2}^T\boldsymbol a_{O_2}
=a_{hx}(\boldsymbol e_{y2}^T\boldsymbol e_{x1})+a_{hy}(\boldsymbol e_{y2}^T\boldsymbol e_{y1})-a_2\dot r_2
=a_{hx}\sin\phi+a_{hy}\cos\phi-a_2\dot r_2 .
\tag{W23}
\]

---

## 4. 受力

### 4.1 卡车外力

前轴力由式 (D37)，作用点为 \(\boldsymbol p_{O_1}+a_1\boldsymbol e_{x1}\)；
后轴力作用点为 \(\boldsymbol p_{O_1}-b_1\boldsymbol e_{x1}\)：

\[
\boldsymbol F_{\mathrm{front}}=-F_{1f}\sin\delta\,\boldsymbol e_{x1}+F_{1f}\cos\delta\,\boldsymbol e_{y1},
\qquad
\boldsymbol F_{\mathrm{rear}}=F_{1x}\boldsymbol e_{x1}+F_{1r}\boldsymbol e_{y1}.
\]

合力：

\[
\boxed{
\boldsymbol F_1^{\mathrm{ext}}
=(F_{1x}-F_{1f}\sin\delta)\boldsymbol e_{x1}+(F_{1f}\cos\delta+F_{1r})\boldsymbol e_{y1}.
}
\tag{W24}
\]

对 \(O_1\) 的力矩，由式 (W6)：

\[
\boxed{
M_1^{\mathrm{ext}}
=a_1\,\boldsymbol e_{y1}^T\boldsymbol F_{\mathrm{front}}-b_1\,\boldsymbol e_{y1}^T\boldsymbol F_{\mathrm{rear}}
=a_1F_{1f}\cos\delta-b_1F_{1r}.
}
\tag{W25}
\]

### 4.2 拖车外力

拖车轴力作用在 \(\boldsymbol p_{O_2}-b_2\boldsymbol e_{x2}\)：

\[
\boxed{
\boldsymbol F_2^{\mathrm{ext}}=F_{2x}\boldsymbol e_{x2}+F_{2r}\boldsymbol e_{y2},
\qquad
M_2^{\mathrm{ext}}=-b_2\,\boldsymbol e_{y2}^T\boldsymbol F_2^{\mathrm{ext}}=-b_2F_{2r}.
}
\tag{W26}
\]

\(F_{2x}\) 沿拖车中心线作用，对 \(O_2\) 没有力矩。

### 4.3 铰接力

卡车受 \(\boldsymbol\Lambda\)，作用点相对 \(O_1\) 为 \(-d_1\boldsymbol e_{x1}\)；
拖车受 \(-\boldsymbol\Lambda\)，作用点相对 \(O_2\) 为 \(a_2\boldsymbol e_{x2}\)。
由式 (W6)、(W3)、(W4)：

\[
\boxed{
M_1^{\Lambda}=(-d_1\boldsymbol e_{x1})\times\boldsymbol\Lambda=-d_1\,\boldsymbol e_{y1}^T\boldsymbol\Lambda=-d_1H,
}
\tag{W27}
\]

\[
\boxed{
M_2^{\Lambda}=(a_2\boldsymbol e_{x2})\times(-\boldsymbol\Lambda)=-a_2\,\boldsymbol e_{y2}^T\boldsymbol\Lambda
=-a_2(F_{hx}\sin\phi+H\cos\phi).
}
\tag{W28}
\]

其中
\(\boldsymbol e_{y2}^T\boldsymbol\Lambda=F_{hx}\boldsymbol e_{y2}^T\boldsymbol e_{x1}+H\boldsymbol e_{y2}^T\boldsymbol e_{y1}
=F_{hx}\sin\phi+H\cos\phi\)，与式 (D44) 一致。

---

## 5. 出发点：两车的精确 Newton–Euler 方程

### 5.1 向量形式

\[
\boxed{
\begin{aligned}
m_1\boldsymbol a_{O_1}&=\boldsymbol F_1^{\mathrm{ext}}+\boldsymbol\Lambda,&
I_1\dot r_1&=M_1^{\mathrm{ext}}+M_1^{\Lambda},\\
m_2\boldsymbol a_{O_2}&=\boldsymbol F_2^{\mathrm{ext}}-\boldsymbol\Lambda,&
I_2\dot r_2&=M_2^{\mathrm{ext}}+M_2^{\Lambda}.
\end{aligned}}
\tag{W29}
\]

### 5.2 六个标量分量

卡车方程投影到 \(\boldsymbol e_{x1}\)、\(\boldsymbol e_{y1}\)，用式 (W17)、(W24)、(W25)、(W27)：

\[
m_1(\dot U-v_{y1}r_1)=F_{1x}-F_{1f}\sin\delta+F_{hx},
\tag{W30}
\]

\[
m_1(\dot v_{y1}+Ur_1)=F_{1f}\cos\delta+F_{1r}+H,
\tag{W31}
\]

\[
I_1\dot r_1=a_1F_{1f}\cos\delta-b_1F_{1r}-d_1H .
\tag{W32}
\]

拖车方程投影到 \(\boldsymbol e_{x2}\)、\(\boldsymbol e_{y2}\)，用
\(\boldsymbol e_{x2}^T\boldsymbol\Lambda=F_{hx}\cos\phi-H\sin\phi\) 和式 (W22)、(W23)、(W26)、(W28)：

\[
m_2\big(a_{hx}\cos\phi-a_{hy}\sin\phi+a_2r_2^2\big)=F_{2x}-F_{hx}\cos\phi+H\sin\phi,
\tag{W33}
\]

\[
m_2\big(a_{hx}\sin\phi+a_{hy}\cos\phi-a_2\dot r_2\big)=F_{2r}-F_{hx}\sin\phi-H\cos\phi,
\tag{W34}
\]

\[
I_2\dot r_2=-b_2F_{2r}-a_2(F_{hx}\sin\phi+H\cos\phi).
\tag{W35}
\]

### 5.3 与第 2 篇的对应

- 式 (W31) 就是式 (D39)；式 (W32) 是式 (D43) 在取 \(\cos\delta\approx1\) 之前的形式
  （三个力矩见式 (D42)）。
- 式 (W34) 右边的铰接力部分就是式 (D44)。第 2 篇的式 (D46) 是式 (W34) 做了两步近似：
  右边用 (D45) 把 \(-F_{hx}\sin\phi-H\cos\phi\) 换成 \(-H\)；
  左边在 \(U\) 恒定、一阶线性化下写成 \(m_2(\dot v_{y2}+Ur_2)\)。
- 式 (W35) 在 (D45) 下就是式 (D48)。
- 式 (W30)、(W33) 是两车纵向方程，第 2 篇没有用到。

### 5.4 未知量计数

六个方程中的未知量是四个加速度 \(\dot U,\dot v_{y1},\dot r_1,\dot r_2\)
（拖车加速度已由式 (W19) 用它们表示）和两个铰接力分量 \(F_{hx},H\)，共六个。
从六个方程中消去两个铰接力分量，应该恰好剩下四个方程。第 2 篇在一阶近似下完成了这个消元；
下面两节给出不依赖任何近似的系统做法。

---

## 6. 核心原理：铰接力在相容虚运动上的总功率为零

### 6.1 相容虚运动

任取一个四维向量 \(\hat{\boldsymbol w}=[\hat U,\ \hat v_h,\ \hat r_1,\ \hat r_2]^T\)，
按式 (W12)–(W14) 同样的规则构造两车的"虚速度"：

\[
\begin{aligned}
\hat{\boldsymbol v}_{P_h}&=\hat U\boldsymbol e_{x1}+\hat v_h\boldsymbol e_{y1},\\
\hat{\boldsymbol v}_{O_1}&=\hat{\boldsymbol v}_{P_h}+d_1\hat r_1\boldsymbol e_{y1},\qquad \hat\omega_1=\hat r_1,\\
\hat{\boldsymbol v}_{O_2}&=\hat{\boldsymbol v}_{P_h}-a_2\hat r_2\boldsymbol e_{y2},\qquad \hat\omega_2=\hat r_2 .
\end{aligned}
\tag{W36}
\]

这里的基向量和 \(\phi\) 取当前时刻的值，\(\hat{\boldsymbol w}\) 与真实运动无关。
两车的虚速度来自同一个 \(\hat{\boldsymbol v}_{P_h}\)，所以这种虚运动与铰接相容：
两车在铰接点处的虚速度相同。

### 6.2 铰接力的虚功率恒为零

用式 (W10) 分别计算两车所受铰接力在虚运动上的功率。

卡车受 \(\boldsymbol\Lambda\)，作用点相对 \(O_1\) 的向量为 \(\boldsymbol q_1=-d_1\boldsymbol e_{x1}\)，
\(J\boldsymbol q_1=-d_1\boldsymbol e_{y1}\)：

\[
\boldsymbol\Lambda^T\hat{\boldsymbol v}_{O_1}+\hat\omega_1M_1^{\Lambda}
=\boldsymbol\Lambda^T\big(\hat{\boldsymbol v}_{O_1}+\hat r_1J\boldsymbol q_1\big)
=\boldsymbol\Lambda^T\big(\hat{\boldsymbol v}_{O_1}-d_1\hat r_1\boldsymbol e_{y1}\big)
=\boldsymbol\Lambda^T\hat{\boldsymbol v}_{P_h}.
\]

第一个等号用了式 (W9)：\(M_1^\Lambda=\boldsymbol q_1\times\boldsymbol\Lambda=\boldsymbol\Lambda^TJ\boldsymbol q_1\)；
最后一个等号用了式 (W36) 第二行。

拖车受 \(-\boldsymbol\Lambda\)，作用点相对 \(O_2\) 的向量为 \(\boldsymbol q_2=a_2\boldsymbol e_{x2}\)，
\(J\boldsymbol q_2=a_2\boldsymbol e_{y2}\)：

\[
(-\boldsymbol\Lambda)^T\hat{\boldsymbol v}_{O_2}+\hat\omega_2M_2^{\Lambda}
=(-\boldsymbol\Lambda)^T\big(\hat{\boldsymbol v}_{O_2}+a_2\hat r_2\boldsymbol e_{y2}\big)
=-\boldsymbol\Lambda^T\hat{\boldsymbol v}_{P_h}.
\]

最后一个等号用了式 (W36) 第三行。两式相加：

\[
\boxed{
\hat P_\Lambda
=\boldsymbol\Lambda^T\hat{\boldsymbol v}_{O_1}+\hat\omega_1M_1^{\Lambda}
-\boldsymbol\Lambda^T\hat{\boldsymbol v}_{O_2}+\hat\omega_2M_2^{\Lambda}
=0 .
}
\tag{W37}
\]

这对任意 \(\hat{\boldsymbol w}\)、任意铰接力 \(\boldsymbol\Lambda\)、任意 \(\phi\) 都成立，没有任何近似。
物理含义：作用力与反作用力大小相等、方向相反，而作用点（铰接点）在两车上的虚速度相同，
所以两者的功率正好抵消。

### 6.3 虚功率方程

把式 (W29) 的每一条写成"残差 \(=0\)"，第一条点乘 \(\hat{\boldsymbol v}_{O_1}\)，第二条乘
\(\hat\omega_1\)，第三条点乘 \(\hat{\boldsymbol v}_{O_2}\)，第四条乘 \(\hat\omega_2\)，全部相加：

\[
\begin{aligned}
0={}&(m_1\boldsymbol a_{O_1}-\boldsymbol F_1^{\mathrm{ext}}-\boldsymbol\Lambda)^T\hat{\boldsymbol v}_{O_1}
+(I_1\dot r_1-M_1^{\mathrm{ext}}-M_1^{\Lambda})\hat\omega_1\\
&+(m_2\boldsymbol a_{O_2}-\boldsymbol F_2^{\mathrm{ext}}+\boldsymbol\Lambda)^T\hat{\boldsymbol v}_{O_2}
+(I_2\dot r_2-M_2^{\mathrm{ext}}-M_2^{\Lambda})\hat\omega_2 .
\end{aligned}
\]

含 \(\boldsymbol\Lambda\) 的项合起来正好是 \(-\hat P_\Lambda\)，由式 (W37) 为零。剩下：

\[
\boxed{
m_1\boldsymbol a_{O_1}^T\hat{\boldsymbol v}_{O_1}+I_1\dot r_1\hat r_1
+m_2\boldsymbol a_{O_2}^T\hat{\boldsymbol v}_{O_2}+I_2\dot r_2\hat r_2
=(\boldsymbol F_1^{\mathrm{ext}})^T\hat{\boldsymbol v}_{O_1}+M_1^{\mathrm{ext}}\hat r_1
+(\boldsymbol F_2^{\mathrm{ext}})^T\hat{\boldsymbol v}_{O_2}+M_2^{\mathrm{ext}}\hat r_2 .
}
\tag{W38}
\]

左边是惯性力在虚运动上的功率，右边是外力（轮胎力、纵向力）的虚功率。
这就是 Jourdain 原理，也是 Kane 方法的出发点。式 (W38) 对 \(\hat{\boldsymbol w}\) 是线性的，
所以只需对四个基向量各写一次。

### 6.4 四个基本虚运动

把 \(\hat{\boldsymbol w}\) 依次取为四个单位向量，由式 (W36) 得到：

| 虚运动 | \(\hat{\boldsymbol w}\) | \(\hat{\boldsymbol v}_{O_1}\) | \(\hat\omega_1\) | \(\hat{\boldsymbol v}_{O_2}\) | \(\hat\omega_2\) | 物理含义 |
|---|---|---|---|---|---|---|
| 1 | \([1,0,0,0]^T\) | \(\boldsymbol e_{x1}\) | 0 | \(\boldsymbol e_{x1}\) | 0 | 整车沿卡车纵向平移 |
| 2 | \([0,1,0,0]^T\) | \(\boldsymbol e_{y1}\) | 0 | \(\boldsymbol e_{y1}\) | 0 | 整车沿卡车横向平移 |
| 3 | \([0,0,1,0]^T\) | \(d_1\boldsymbol e_{y1}\) | 1 | \(\boldsymbol 0\) | 0 | 卡车绕铰接点转动，拖车不动 |
| 4 | \([0,0,0,1]^T\) | \(\boldsymbol 0\) | 0 | \(-a_2\boldsymbol e_{y2}\) | 1 | 拖车绕铰接点转动，卡车不动 |

以第 3 行为例：\(\hat U=\hat v_h=0\) 使 \(\hat{\boldsymbol v}_{P_h}=\boldsymbol 0\)，于是
\(\hat{\boldsymbol v}_{O_1}=d_1\hat r_1\boldsymbol e_{y1}=d_1\boldsymbol e_{y1}\)；
\(\hat r_2=0\) 使 \(\hat{\boldsymbol v}_{O_2}=\boldsymbol 0\)。铰接点不动而卡车转动，
就是卡车绕铰接点转动。

### 6.5 为什么四条方程既足够又等价

- **足够。** 任意相容虚运动都可以写成
  \(\hat{\boldsymbol w}=\hat U\,\hat{\boldsymbol w}^{(1)}+\hat v_h\,\hat{\boldsymbol w}^{(2)}
  +\hat r_1\,\hat{\boldsymbol w}^{(3)}+\hat r_2\,\hat{\boldsymbol w}^{(4)}\)。
  式 (W38) 对 \(\hat{\boldsymbol w}\) 线性，所以其他任何选择得到的方程都是这四条的线性组合。
- **与 Newton–Euler 等价。** 7.5 节将证明，这四条方程恰好是式 (W30)–(W35) 中让
  \(F_{hx}\)、\(H\) 系数同时为零的四个独立线性组合。六个方程、两个铰接力未知量，
  这样的组合恰有四个。反过来，由这四条方程解出加速度后，可以用式 (W30)、(W31)
  回代出 \(F_{hx}\)、\(H\)（8.4 节），此时六个 Newton–Euler 方程全部成立。
- **选法不唯一。** 任何相容虚运动都能给出不含铰接力的方程。例如固定 \(v_{y1}\)
  让卡车绕自身质心转动也可以，只是那时铰接点会移动，拖车必须跟着平移，方程会多出不少项。
  选"绕铰接点转动"能让每条方程的项数最少。

---

## 7. 逐条推导四条方程

### 7.1 (E1)：整车沿卡车纵向平移

把虚运动 1（\(\hat{\boldsymbol v}_{O_1}=\hat{\boldsymbol v}_{O_2}=\boldsymbol e_{x1}\)，
\(\hat r_1=\hat r_2=0\)）代入式 (W38)，逐项计算。

左边：

- \(m_1\boldsymbol a_{O_1}^T\boldsymbol e_{x1}=m_1(\dot U-v_{y1}r_1)\)，由式 (W17)；
- \(m_2\boldsymbol a_{O_2}^T\boldsymbol e_{x1}=m_2(a_{hx}-a_2\dot r_2\sin\phi+a_2r_2^2\cos\phi)\)，
  由式 (W20)；
- 两个转动项乘以 0。

右边：

- \((\boldsymbol F_1^{\mathrm{ext}})^T\boldsymbol e_{x1}=F_{1x}-F_{1f}\sin\delta\)，由式 (W24)；
- \((\boldsymbol F_2^{\mathrm{ext}})^T\boldsymbol e_{x1}
  =F_{2x}\boldsymbol e_{x2}^T\boldsymbol e_{x1}+F_{2r}\boldsymbol e_{y2}^T\boldsymbol e_{x1}
  =F_{2x}\cos\phi+F_{2r}\sin\phi\)，由式 (W4)；
- 两个力矩项乘以 0。

于是

\[
\boxed{
m_1(\dot U-v_{y1}r_1)+m_2\big(a_{hx}-a_2\dot r_2\sin\phi+a_2r_2^2\cos\phi\big)
=F_{1x}-F_{1f}\sin\delta+F_{2x}\cos\phi+F_{2r}\sin\phi .
}
\tag{E1}
\]

### 7.2 (E2)：整车沿卡车横向平移

虚运动 2：\(\hat{\boldsymbol v}_{O_1}=\hat{\boldsymbol v}_{O_2}=\boldsymbol e_{y1}\)，
\(\hat r_1=\hat r_2=0\)。

左边：

- \(m_1\boldsymbol a_{O_1}^T\boldsymbol e_{y1}=m_1(\dot v_{y1}+Ur_1)\)，由式 (W17)；
- \(m_2\boldsymbol a_{O_2}^T\boldsymbol e_{y1}=m_2(a_{hy}-a_2\dot r_2\cos\phi-a_2r_2^2\sin\phi)\)，
  由式 (W21)。

右边：

- \((\boldsymbol F_1^{\mathrm{ext}})^T\boldsymbol e_{y1}=F_{1f}\cos\delta+F_{1r}\)；
- \((\boldsymbol F_2^{\mathrm{ext}})^T\boldsymbol e_{y1}
  =F_{2x}\boldsymbol e_{x2}^T\boldsymbol e_{y1}+F_{2r}\boldsymbol e_{y2}^T\boldsymbol e_{y1}
  =-F_{2x}\sin\phi+F_{2r}\cos\phi\)。

于是

\[
\boxed{
m_1(\dot v_{y1}+Ur_1)+m_2\big(a_{hy}-a_2\dot r_2\cos\phi-a_2r_2^2\sin\phi\big)
=F_{1f}\cos\delta+F_{1r}+F_{2r}\cos\phi-F_{2x}\sin\phi .
}
\tag{E2}
\]

### 7.3 (E3)：卡车绕铰接点转动

虚运动 3：\(\hat{\boldsymbol v}_{O_1}=d_1\boldsymbol e_{y1}\)，\(\hat r_1=1\)，拖车的虚速度全为零。

左边：

- \(m_1\boldsymbol a_{O_1}^T(d_1\boldsymbol e_{y1})=m_1d_1(\dot v_{y1}+Ur_1)\)；
- \(I_1\dot r_1\cdot1=I_1\dot r_1\)；
- 拖车项为零。

右边：

- \((\boldsymbol F_1^{\mathrm{ext}})^T(d_1\boldsymbol e_{y1})=d_1(F_{1f}\cos\delta+F_{1r})\)；
- \(M_1^{\mathrm{ext}}\cdot1=a_1F_{1f}\cos\delta-b_1F_{1r}\)，由式 (W25)；
- 拖车项为零。

合并右边：
\(d_1F_{1f}\cos\delta+d_1F_{1r}+a_1F_{1f}\cos\delta-b_1F_{1r}
=(a_1+d_1)F_{1f}\cos\delta-(b_1-d_1)F_{1r}\)。于是

\[
\boxed{
I_1\dot r_1+m_1d_1(\dot v_{y1}+Ur_1)
=(a_1+d_1)F_{1f}\cos\delta-(b_1-d_1)F_{1r}.
}
\tag{E3}
\]

**另一种读法。** 刚体对运动点 \(P\) 的力矩方程为
\(\sum M_P^{\mathrm{ext}}=I\dot r+(\boldsymbol p_O-\boldsymbol p_P)\times m\boldsymbol a_O\)。
对卡车取 \(P=P_h\)：\(\boldsymbol p_{O_1}-\boldsymbol p_{P_h}=d_1\boldsymbol e_{x1}\)，由式 (W6)，
\((d_1\boldsymbol e_{x1})\times m_1\boldsymbol a_{O_1}=m_1d_1\boldsymbol e_{y1}^T\boldsymbol a_{O_1}
=m_1d_1(\dot v_{y1}+Ur_1)\)；前轴相对 \(P_h\) 在 \((a_1+d_1)\boldsymbol e_{x1}\)，后轴在
\((d_1-b_1)\boldsymbol e_{x1}\)，力矩为 \((a_1+d_1)F_{1f}\cos\delta+(d_1-b_1)F_{1r}\)；
铰接力通过 \(P_h\)，力矩为零。结果与 (E3) 相同。

当 \(d_1=b_1\)（铰接点在后轴中心）时，(E3) 简化为
\(I_1\dot r_1+m_1b_1(\dot v_{y1}+Ur_1)=L_1F_{1f}\cos\delta\)。

### 7.4 (E4)：拖车绕铰接点转动

虚运动 4：\(\hat{\boldsymbol v}_{O_2}=-a_2\boldsymbol e_{y2}\)，\(\hat r_2=1\)，卡车的虚速度全为零。

左边：

- \(m_2\boldsymbol a_{O_2}^T(-a_2\boldsymbol e_{y2})
  =-m_2a_2(a_{hx}\sin\phi+a_{hy}\cos\phi-a_2\dot r_2)\)，由式 (W23)；
- \(I_2\dot r_2\cdot1=I_2\dot r_2\)。

右边：

- \((\boldsymbol F_2^{\mathrm{ext}})^T(-a_2\boldsymbol e_{y2})=-a_2F_{2r}\)；
- \(M_2^{\mathrm{ext}}\cdot1=-b_2F_{2r}\)，由式 (W26)。

左边合并为 \(I_2\dot r_2+m_2a_2^2\dot r_2-m_2a_2(a_{hx}\sin\phi+a_{hy}\cos\phi)\)，
右边合并为 \(-(a_2+b_2)F_{2r}=-L_2F_{2r}\)。于是

\[
\boxed{
(I_2+m_2a_2^2)\dot r_2-m_2a_2\big(a_{hx}\sin\phi+a_{hy}\cos\phi\big)=-L_2F_{2r}.
}
\tag{E4}
\]

这就是拖车对铰接点的力矩方程：拖车轴力对 \(P_h\) 的力臂为 \(L_2\)；
\(F_{2x}\) 沿拖车中心线、通过 \(P_h\)，没有力矩；铰接力通过 \(P_h\)，也没有力矩。

### 7.5 初等复核：四条方程就是 Newton–Euler 方程的特定线性组合

不借助虚功率，也可以直接对式 (W30)–(W35) 做线性组合，逐条验证铰接力被消去。

**(E1)。** 把拖车方程 \(m_2\boldsymbol a_{O_2}=\boldsymbol F_2^{\mathrm{ext}}-\boldsymbol\Lambda\)
投影到 \(\boldsymbol e_{x1}\)：

\[
m_2\boldsymbol e_{x1}^T\boldsymbol a_{O_2}=F_{2x}\cos\phi+F_{2r}\sin\phi-F_{hx}.
\]

与式 (W30) 相加，\(+F_{hx}\) 与 \(-F_{hx}\) 抵消，再代入式 (W20)，得到 (E1)。

**(E2)。** 把拖车方程投影到 \(\boldsymbol e_{y1}\)：

\[
m_2\boldsymbol e_{y1}^T\boldsymbol a_{O_2}=-F_{2x}\sin\phi+F_{2r}\cos\phi-H.
\]

与式 (W31) 相加，\(+H\) 与 \(-H\) 抵消，再代入式 (W21)，得到 (E2)。

**(E3)。** 式 (W32) 加 \(d_1\times\)式 (W31)：铰接力项为 \(-d_1H+d_1H=0\)，得到 (E3)。

**(E4)。** 式 (W35) 减 \(a_2\times\)式 (W34)：铰接力项为

\[
-a_2(F_{hx}\sin\phi+H\cos\phi)-a_2(-F_{hx}\sin\phi-H\cos\phi)=0,
\]

轮胎力项为 \(-b_2F_{2r}-a_2F_{2r}=-L_2F_{2r}\)，得到 (E4)。

可见虚功率方法本身并不神秘，它只是自动找出了这四个组合。要点只有两条：
平动方程必须投影到**同一个方向**（(E1)、(E2) 都投影到卡车坐标轴），
转动方程必须取**对铰接点**的力矩（(E3)、(E4)）。

---

## 8. 精确非线性模型

### 8.1 代入铰接点加速度并分离加速度项

把式 (W18) 的 \(a_{hx}=\dot U-v_hr_1\)、\(a_{hy}=\dot v_{y1}-d_1\dot r_1+Ur_1\) 代入
(E1)–(E4)，加速度项留在左边，其余移到右边。

**(E1)：** 展开左边，

\[
m_1\dot U-m_1v_{y1}r_1+m_2\dot U-m_2v_hr_1-m_2a_2\sin\phi\,\dot r_2+m_2a_2r_2^2\cos\phi
=F_{1x}-F_{1f}\sin\delta+F_{2x}\cos\phi+F_{2r}\sin\phi,
\]

移项得

\[
(m_1+m_2)\dot U-m_2a_2\sin\phi\,\dot r_2
=F_{1x}-F_{1f}\sin\delta+F_{2x}\cos\phi+F_{2r}\sin\phi
+m_1v_{y1}r_1+m_2v_hr_1-m_2a_2r_2^2\cos\phi .
\tag{W39}
\]

**(E2)：** 展开左边，

\[
m_1\dot v_{y1}+m_1Ur_1+m_2\dot v_{y1}-m_2d_1\dot r_1+m_2Ur_1
-m_2a_2\cos\phi\,\dot r_2-m_2a_2r_2^2\sin\phi
=F_{1f}\cos\delta+F_{1r}+F_{2r}\cos\phi-F_{2x}\sin\phi,
\]

移项得

\[
(m_1+m_2)\dot v_{y1}-m_2d_1\dot r_1-m_2a_2\cos\phi\,\dot r_2
=F_{1f}\cos\delta+F_{1r}+F_{2r}\cos\phi-F_{2x}\sin\phi
-(m_1+m_2)Ur_1+m_2a_2r_2^2\sin\phi .
\tag{W40}
\]

**(E3)：** 把 \(m_1d_1Ur_1\) 移到右边，

\[
m_1d_1\dot v_{y1}+I_1\dot r_1
=(a_1+d_1)F_{1f}\cos\delta-(b_1-d_1)F_{1r}-m_1d_1Ur_1 .
\tag{W41}
\]

**(E4)：** 先展开

\[
a_{hx}\sin\phi+a_{hy}\cos\phi
=\sin\phi\,\dot U-v_hr_1\sin\phi+\cos\phi\,\dot v_{y1}-d_1\cos\phi\,\dot r_1+Ur_1\cos\phi,
\]

代入 (E4) 并移项：

\[
-m_2a_2\sin\phi\,\dot U-m_2a_2\cos\phi\,\dot v_{y1}+m_2a_2d_1\cos\phi\,\dot r_1+(I_2+m_2a_2^2)\dot r_2
=-L_2F_{2r}+m_2a_2Ur_1\cos\phi-m_2a_2v_hr_1\sin\phi .
\tag{W42}
\]

### 8.2 矩阵形式

记 \(\boldsymbol\zeta=[U,\ v_{y1},\ r_1,\ r_2]^T\)，式 (W39)–(W42) 写成

\[
\boxed{
M(\phi)\,\dot{\boldsymbol\zeta}=\boldsymbol f(\boldsymbol\zeta,\phi,\delta,F_{1x},F_{2x}),
}
\tag{W43}
\]

\[
M(\phi)=
\begin{bmatrix}
m_1+m_2&0&0&-m_2a_2\sin\phi\\
0&m_1+m_2&-m_2d_1&-m_2a_2\cos\phi\\
0&m_1d_1&I_1&0\\
-m_2a_2\sin\phi&-m_2a_2\cos\phi&m_2a_2d_1\cos\phi&I_2+m_2a_2^2
\end{bmatrix},
\tag{W44}
\]

\[
\boldsymbol f=
\begin{bmatrix}
F_{1x}-F_{1f}\sin\delta+F_{2x}\cos\phi+F_{2r}\sin\phi+m_1v_{y1}r_1+m_2v_hr_1-m_2a_2r_2^2\cos\phi\\
F_{1f}\cos\delta+F_{1r}+F_{2r}\cos\phi-F_{2x}\sin\phi-(m_1+m_2)Ur_1+m_2a_2r_2^2\sin\phi\\
(a_1+d_1)F_{1f}\cos\delta-(b_1-d_1)F_{1r}-m_1d_1Ur_1\\
-L_2F_{2r}+m_2a_2Ur_1\cos\phi-m_2a_2v_hr_1\sin\phi
\end{bmatrix},
\tag{W45}
\]

其中 \(v_h=v_{y1}-d_1r_1\)。

### 8.3 质量矩阵对任意 \(\phi\) 可逆

\(M(\phi)\) 不对称，这是因为未知量用了 \(\dot v_{y1}\) 而不是 \(\dot v_h\)。
改用 \(\dot{\boldsymbol w}=[\dot U,\ \dot v_h,\ \dot r_1,\ \dot r_2]^T\) 即可看清结构。

**偏速度。** 式 (W13)、(W14) 对 \(\boldsymbol w\) 各分量的偏导数，恰好就是 6.4 节表中的
四组虚速度。把第 \(k\) 组写成六维列向量
\(\boldsymbol s_k=[\hat{\boldsymbol v}_{O_1}^{(k)};\ \hat\omega_1^{(k)};\ \hat{\boldsymbol v}_{O_2}^{(k)};\ \hat\omega_2^{(k)}]\)，
排成 \(6\times4\) 矩阵 \(S=[\boldsymbol s_1\ \boldsymbol s_2\ \boldsymbol s_3\ \boldsymbol s_4]\)。
再记 \(M_b=\mathrm{diag}(m_1,m_1,I_1,m_2,m_2,I_2)\)。两车速度可写成
\([\boldsymbol v_{O_1};\ \omega_1;\ \boldsymbol v_{O_2};\ \omega_2]=S\boldsymbol w\)，
加速度为 \(S\dot{\boldsymbol w}+\dot S\boldsymbol w\)。

**广义质量矩阵。** 把加速度代入式 (W38) 左边，\(\dot{\boldsymbol w}\) 的系数为
\(M_w=S^TM_bS\)，即

\[
(M_w)_{kj}=m_1\hat{\boldsymbol v}_{O_1}^{(k)T}\hat{\boldsymbol v}_{O_1}^{(j)}
+I_1\hat\omega_1^{(k)}\hat\omega_1^{(j)}
+m_2\hat{\boldsymbol v}_{O_2}^{(k)T}\hat{\boldsymbol v}_{O_2}^{(j)}
+I_2\hat\omega_2^{(k)}\hat\omega_2^{(j)} .
\]

用 6.4 节的表逐项计算：

- \((1,1)=m_1+m_2\)，\((2,2)=m_1+m_2\)；
- \((1,2)=m_1\boldsymbol e_{x1}^T\boldsymbol e_{y1}+m_2\boldsymbol e_{x1}^T\boldsymbol e_{y1}=0\)；
- \((1,3)=m_1\boldsymbol e_{x1}^T(d_1\boldsymbol e_{y1})=0\)，
  \((2,3)=m_1\boldsymbol e_{y1}^T(d_1\boldsymbol e_{y1})=m_1d_1\)；
- \((1,4)=m_2\boldsymbol e_{x1}^T(-a_2\boldsymbol e_{y2})=-m_2a_2\sin\phi\)，
  \((2,4)=m_2\boldsymbol e_{y1}^T(-a_2\boldsymbol e_{y2})=-m_2a_2\cos\phi\)；
- \((3,3)=m_1d_1^2+I_1\)，\((3,4)=0\)（虚运动 3 只动卡车、虚运动 4 只动拖车）；
- \((4,4)=m_2a_2^2+I_2\)。

\[
M_w(\phi)=
\begin{bmatrix}
m_1+m_2&0&0&-m_2a_2\sin\phi\\
0&m_1+m_2&m_1d_1&-m_2a_2\cos\phi\\
0&m_1d_1&I_1+m_1d_1^2&0\\
-m_2a_2\sin\phi&-m_2a_2\cos\phi&0&I_2+m_2a_2^2
\end{bmatrix}.
\tag{W46}
\]

\(M_w\) 是对称矩阵。\(M_b\) 正定；\(S\) 列满秩：若 \(S\hat{\boldsymbol w}=\boldsymbol 0\)，
则 \(\hat\omega_1=\hat r_1=0\)、\(\hat\omega_2=\hat r_2=0\)，再由
\(\hat{\boldsymbol v}_{O_1}=\hat U\boldsymbol e_{x1}+\hat v_h\boldsymbol e_{y1}=\boldsymbol 0\)
得 \(\hat U=\hat v_h=0\)。所以 \(M_w=S^TM_bS\) 对任意 \(\phi\) 正定。

**回到 \(\dot{\boldsymbol\zeta}\)。** 由 \(\dot v_h=\dot v_{y1}-d_1\dot r_1\)，
\(\dot{\boldsymbol w}=T\dot{\boldsymbol\zeta}\)，其中

\[
T=\begin{bmatrix}1&0&0&0\\0&1&-d_1&0\\0&0&1&0\\0&0&0&1\end{bmatrix},\qquad \det T=1 .
\]

于是 \(M(\phi)=M_w(\phi)\,T\)，\(\det M(\phi)=\det M_w(\phi)>0\)。
以第 3 行为例校核：
\(m_1d_1(\dot v_{y1}-d_1\dot r_1)+(I_1+m_1d_1^2)\dot r_1=m_1d_1\dot v_{y1}+I_1\dot r_1\)，
与式 (W41) 左边相同。

### 8.4 事后恢复铰接力

解出 \(\dot{\boldsymbol\zeta}\) 后，由式 (W30)、(W31) 精确得到铰接力：

\[
\boxed{
F_{hx}=m_1(\dot U-v_{y1}r_1)-F_{1x}+F_{1f}\sin\delta,
\qquad
H=m_1(\dot v_{y1}+Ur_1)-F_{1f}\cos\delta-F_{1r}.
}
\tag{W47}
\]

第 2 篇的式 (D75)–(D76) 是这里 \(H\) 在一阶、\(U\) 恒定、\(F_{2x,0}=0\) 下的近似。

### 8.5 完整仿真方程

状态取 \([X_1,\ Y_1,\ \theta_1,\ \phi,\ U,\ v_{y1},\ r_1,\ r_2]\)：

\[
\begin{aligned}
\dot X_1&=U\cos\theta_1-v_{y1}\sin\theta_1,&
\dot Y_1&=U\sin\theta_1+v_{y1}\cos\theta_1,\\
\dot\theta_1&=r_1,&
\dot\phi&=r_1-r_2,\\
\dot{\boldsymbol\zeta}&=M(\phi)^{-1}\boldsymbol f .
\end{aligned}
\tag{W48}
\]

前两行来自 \(\boldsymbol v_{O_1}=U\boldsymbol e_{x1}+v_{y1}\boldsymbol e_{y1}\) 的全局分量。

轮胎侧偏角用精确的速度方向角（式 (D63) 取小角度之前的形式），拖车轴用式 (W15)、(W16)：

\[
\alpha_{1f}=\delta-\arctan\frac{v_{y1}+a_1r_1}{U},\qquad
\alpha_{1r}=-\arctan\frac{v_{y1}-b_1r_1}{U},\qquad
\alpha_{2r}=-\arctan\frac{v_{y2}-b_2r_2}{U_2}.
\tag{W49}
\]

侧向力 \(F=f(\alpha)\) 可以是线性的 \(C\alpha\)，也可以是魔术公式等非线性模型。

由于 \(\boldsymbol w\)（或 \(\boldsymbol\zeta\)）是独立速度，积分时不存在约束漂移，
不需要 Baumgarte 稳定化，这一点优于保留铰接力乘子的微分代数方程。

---

## 9. 线性化

### 9.1 工作点与阶次约定

工作点：直线行驶，\(\phi=v_{y1}=r_1=r_2=\delta=0\)，\(U=U(t)>0\)，\(a_x=\dot U\) 允许为 \(O(1)\)。
纵向力写成工作点值加扰动：\(F_{1x}=F_{1x,0}+\delta F_{1x}\)，\(F_{2x}=F_{2x,0}+\delta F_{2x}\)，
扰动由横向运动引起，至少为一阶小量。

小量约定与第 2 篇第 1 章相同：\(\delta,\ \phi,\ v_{y1}/U,\ L_ir_i/U=O(\varepsilon)\)；
线性轮胎下 \(F_{1f},F_{1r},F_{2r}\) 与侧偏角成正比，也是一阶量。

保留规则：

- \(\sin\phi=\phi+O(\varepsilon^3)\)，\(\cos\phi=1+O(\varepsilon^2)\)，\(\delta\) 同理；
- 两个一阶量的乘积是二阶，舍去；
- \(O(1)\) 量（\(U\)、\(a_x\)、\(F_{2x,0}\)）与一阶量的乘积是一阶，**保留**。

最后一条是本文与第 2 篇结果不同的根源：第 2 篇取 \(a_x=0\)，并用 (D45) 去掉了 \(F_{hx,0}\phi\)。

### 9.2 (E1)：纵向平衡，与横向解耦

**零阶。** 式 (W39) 中所有含横向量的项都至少是一阶，剩下

\[
\boxed{(m_1+m_2)a_x=F_{1x,0}+F_{2x,0}.}
\tag{W50}
\]

这是工作点的纵向平衡，\(a_x\) 由纵向控制决定。

**一阶。** \(-m_2a_2\sin\phi\,\dot r_2\)、\(F_{1f}\sin\delta\)、\(F_{2r}\sin\phi\)、
\(m_1v_{y1}r_1\)、\(m_2v_hr_1\)、\(m_2a_2r_2^2\cos\phi\) 都是二阶；
\(F_{2x}\cos\phi=F_{2x,0}+\delta F_{2x}+O(\varepsilon^2)\)。所以一阶扰动满足
\((m_1+m_2)\delta\dot U=\delta F_{1x}+\delta F_{2x}\)，不含任何横向状态：
纵向扰动与横向动力学在一阶意义下解耦，(E1) 只通过工作点量 \(a_x\) 影响横向模型。

**铰接纵向力的工作点值。** 取式 (W33) 的零阶形式：由式 (W22)，
\(\boldsymbol e_{x2}^T\boldsymbol a_{O_2}\) 的零阶部分为 \(a_{hx}=a_x\)；右边零阶为
\(F_{2x,0}-F_{hx,0}\)。所以

\[
\boxed{F_{hx,0}=F_{2x,0}-m_2a_x .}
\tag{W51}
\]

\(U\) 恒定时 \(F_{hx,0}=F_{2x,0}\)，即拖车自身的纵向阻力；加速时卡车还要额外提供
\(m_2a_x\) 的拉力。

### 9.3 (E2) 线性化

逐项处理式 (W40)：

| 项 | 线性化结果 | 说明 |
|---|---|---|
| \((m_1+m_2)\dot v_{y1}\) | 保留 | 一阶 |
| \(-m_2d_1\dot r_1\) | 保留 | 一阶 |
| \(-m_2a_2\cos\phi\,\dot r_2\) | \(-m_2a_2\dot r_2\) | 误差 \(O(\varepsilon^3)\) |
| \(F_{1f}\cos\delta\) | \(F_{1f}\) | 误差 \(O(\varepsilon^3)\) |
| \(F_{1r}\) | 保留 | 一阶 |
| \(F_{2r}\cos\phi\) | \(F_{2r}\) | 误差 \(O(\varepsilon^3)\) |
| \(-F_{2x}\sin\phi\) | \(-F_{2x,0}\phi\) | \(O(1)\) 乘一阶，**保留**；\(\delta F_{2x}\,\phi\) 为二阶 |
| \(-(m_1+m_2)Ur_1\) | 保留 | 一阶 |
| \(m_2a_2r_2^2\sin\phi\) | 舍去 | 三阶 |

\[
\boxed{
(m_1+m_2)\dot v_{y1}-m_2d_1\dot r_1-m_2a_2\dot r_2
=F_{1f}+F_{1r}+F_{2r}-(m_1+m_2)Ur_1-F_{2x,0}\,\phi .
}
\tag{E2L}
\]

与式 (D74) 相比，只多出最后一项 \(-F_{2x,0}\phi\)。注意 (E2) 中根本没有 \(\dot U\)，
所以这一行不含 \(a_x\)。

### 9.4 (E3) 线性化

式 (W41) 中只有 \(\cos\delta\) 需要处理，取 \(\cos\delta=1+O(\varepsilon^2)\)：

\[
\boxed{
m_1d_1\dot v_{y1}+I_1\dot r_1
=(a_1+d_1)F_{1f}-(b_1-d_1)F_{1r}-m_1d_1Ur_1 .
}
\tag{E3L}
\]

这一行既不含 \(F_{2x}\)，也不含 \(a_x\)。

### 9.5 (E4) 线性化

逐项处理式 (W42)：

| 项 | 线性化结果 | 说明 |
|---|---|---|
| \(-m_2a_2\sin\phi\,\dot U\) | \(-m_2a_2a_x\phi\) | \(O(1)\) 乘一阶，**保留**；\(\delta\dot U\,\phi\) 为二阶 |
| \(-m_2a_2\cos\phi\,\dot v_{y1}\) | \(-m_2a_2\dot v_{y1}\) | 误差 \(O(\varepsilon^3)\) |
| \(m_2a_2d_1\cos\phi\,\dot r_1\) | \(m_2a_2d_1\dot r_1\) | 误差 \(O(\varepsilon^3)\) |
| \((I_2+m_2a_2^2)\dot r_2\) | 保留 | 一阶 |
| \(-L_2F_{2r}\) | 保留 | 一阶 |
| \(m_2a_2Ur_1\cos\phi\) | \(m_2a_2Ur_1\) | 误差 \(O(\varepsilon^3)\) |
| \(-m_2a_2v_hr_1\sin\phi\) | 舍去 | 三阶 |

把 \(-m_2a_2a_x\phi\) 移到右边：

\[
\boxed{
-m_2a_2\dot v_{y1}+m_2a_2d_1\dot r_1+(I_2+m_2a_2^2)\dot r_2
=-L_2F_{2r}+m_2a_2Ur_1+m_2a_2a_x\,\phi .
}
\tag{E4L}
\]

与式 (D82) 相比，只多出最后一项 \(m_2a_2a_x\phi\)。

### 9.6 轮胎力

式 (W15)、(W16) 一阶展开：

\[
U_2=U+O(\varepsilon^2)U,\qquad
v_{y2}=U\phi+v_h-a_2r_2+O(\varepsilon^3)U=v_{y1}-d_1r_1-a_2r_2+U\phi .
\tag{W52}
\]

第二式与式 (D58) 完全相同，但这里是每个时刻都成立的代数关系，\(U\) 随时间变化也不影响。
式 (W49) 取小角度后就是式 (D64)、(D65)、(D66)，代入式 (W52) 得到式 (D68)，
侧向力仍为式 (D69)，只是其中的 \(U\) 取当前值 \(U(t)\)。

整个推导从未对约束求导：拖车加速度直接由运动链 (W19) 得到。所以第 2 篇式 (D59)–(D60)
的恒速前提，以及第 11.2 节式 (D152) 补充的 \(\dot U\phi\)，在这里都不需要单独处理——
\(\dot U\) 已经包含在 (E4) 的 \(a_{hx}\sin\phi\) 里。

### 9.7 换到第 2 篇的行组合

第 2 篇第 8 章的三行对应本文的：

- 第一行 = (E2L)；
- 第二行 = (E3L) \(-\,d_1\times\)(E2L)；
- 第三行 = (E4L)。

这个行变换的矩阵是单位下三角阵，行列式为 1，不改变方程组的解。下面逐项计算第二行。

左边：

\[
\begin{aligned}
&\big[m_1d_1-d_1(m_1+m_2)\big]\dot v_{y1}
+\big[I_1-d_1(-m_2d_1)\big]\dot r_1
+\big[0-d_1(-m_2a_2)\big]\dot r_2\\
&=-m_2d_1\dot v_{y1}+(I_1+m_2d_1^2)\dot r_1+m_2d_1a_2\dot r_2 ,
\end{aligned}
\]

与式 (D79) 左边相同。

右边：

\[
\begin{aligned}
&(a_1+d_1)F_{1f}-(b_1-d_1)F_{1r}-m_1d_1Ur_1
-d_1\big[F_{1f}+F_{1r}+F_{2r}-(m_1+m_2)Ur_1-F_{2x,0}\phi\big]\\
&=a_1F_{1f}-b_1F_{1r}-d_1F_{2r}+m_2d_1Ur_1+d_1F_{2x,0}\,\phi ,
\end{aligned}
\]

即式 (D79) 右边再加 \(d_1F_{2x,0}\phi\)。

### 9.8 与第 2 篇矩阵逐项对照

三行左边分别与式 (D74)、(D79)、(D82) 左边相同，所以**有效质量矩阵 \(M_e\) 就是式 (D93)**。

右边代入式 (D69) 的轮胎力，过程与式 (D83)–(D91) 完全相同，所以
**\(K_e\)（式 (D94)）和 \(\boldsymbol G_e\)（式 (D95)）不变**，\(K_e\) 中的 \(U\) 取当前值。

铰接角项逐行收集：

- 第一行：\(F_{2r}\) 贡献 \(-C_{2r}\phi\)，新项 \(-F_{2x,0}\phi\)；
- 第二行：\(-d_1F_{2r}\) 贡献 \(d_1C_{2r}\phi\)，新项 \(d_1F_{2x,0}\phi\)；
- 第三行：\(-L_2F_{2r}\) 贡献 \(L_2C_{2r}\phi\)，新项 \(m_2a_2a_x\phi\)。

\[
\boxed{
\boldsymbol k_\phi=
C_{2r}\begin{bmatrix}-1\\d_1\\L_2\end{bmatrix}
+F_{2x,0}\begin{bmatrix}-1\\d_1\\0\end{bmatrix}
+m_2a_2a_x\begin{bmatrix}0\\0\\1\end{bmatrix}.
}
\tag{W53}
\]

最终线性模型：

\[
\boxed{
M_e\dot{\boldsymbol\eta}=K_e(U)\,\boldsymbol\eta+\boldsymbol k_\phi(F_{2x,0},a_x)\,\phi+\boldsymbol G_e\delta .
}
\tag{W54}
\]

\(\boldsymbol\eta=[v_{y1},\ r_1,\ r_2]^T\) 同式 (D92)。状态与输入的维数都没有增加，
\(a_x\)、\(F_{2x,0}\) 只作为调度参数出现，与 \(U\) 的地位相同。

### 9.9 四状态与六状态矩阵中需要修改的元素

四状态模型 (D113) 中，只有铰接角列 \(a_{14},a_{24},a_{34}\) 改变。记 \(k_1,k_2,k_3\) 为
\(\boldsymbol k_\phi\) 的三个分量：

\[
k_1=-(C_{2r}+F_{2x,0}),\qquad
k_2=d_1(C_{2r}+F_{2x,0}),\qquad
k_3=L_2C_{2r}+m_2a_2a_x .
\]

与式 (D110) 同样用逆质量矩阵的元素（式 (D107)），\(a_{i4}=q_{i1}k_1+q_{i2}k_2+q_{i3}k_3\)：

\[
\boxed{
\begin{aligned}
a_{14}&=(C_{2r}+F_{2x,0})(-q_{11}+d_1q_{12})+(L_2C_{2r}+m_2a_2a_x)\,q_{13},\\
a_{24}&=(C_{2r}+F_{2x,0})(-q_{12}+d_1q_{22})+(L_2C_{2r}+m_2a_2a_x)\,q_{23},\\
a_{34}&=(C_{2r}+F_{2x,0})(-q_{13}+d_1q_{23})+(L_2C_{2r}+m_2a_2a_x)\,q_{33}.
\end{aligned}}
\tag{W55}
\]

取 \(F_{2x,0}=a_x=0\) 即回到式 (D110)。其余 \(a_{ij}\) 和 \(\beta_i\) 不变。

六状态误差模型中，\(a_{14},a_{24},a_{34}\) 只出现在式 (D145) 的三个元素
\(A_c(2,5)=a_{14}\)、\(A_c(4,5)=a_{24}\)、\(A_c(6,5)=a_{24}-a_{34}\) 中；
\(B_c\)、\(E_\rho\)、\(E_{\dot\rho}\) 不含它们，保持不变。另外，变速时误差运动学本身的
\(\dot Ue_\psi\)、\(-\dot U\rho\) 项（第 2 篇式 (D154)、(D155)）与本文无关，
仍需按第 2 篇第 11.2 节 (b)(c) 处理。

### 9.10 特例与推广

**特例一：\(a_x=0\)、\(F_{2x,0}=0\)。** 式 (W53) 退化为 \(C_{2r}[-1,\ d_1,\ L_2]^T\)，
式 (W54) 就是第 2 篇的式 (D96)。第 2 篇是本文的特例。

**特例二：\(a_x=0\)、\(F_{2x,0}\ne0\)。** 由式 (W51)，\(F_{hx,0}=F_{2x,0}\)。
第 2 篇的条件 (D45a) 在这种情况下等价于 \(F_{2x,0}=0\)。

**用 \(F_{hx,0}\) 改写。** 把式 (W51) 给出的 \(F_{2x,0}=F_{hx,0}+m_2a_x\) 代入式 (W53)：

\[
\boldsymbol k_\phi=
C_{2r}\begin{bmatrix}-1\\d_1\\L_2\end{bmatrix}
+\underbrace{F_{hx,0}\begin{bmatrix}-1\\d_1\\0\end{bmatrix}}_{\text{补回 }F_{hx}\sin\phi}
+\underbrace{m_2a_x\begin{bmatrix}-1\\d_1\\a_2\end{bmatrix}}_{\text{补回 }\dot U\phi}.
\tag{W56}
\]

第二项正是在 (D46)、(D48) 中保留 \(-F_{hx,0}\phi\)、\(-a_2F_{hx,0}\phi\) 后按第 8 章消元所得；
第三项正是在 (D60) 中补上 \(\dot U\phi\) 后按第 8 章消元所得。两项必须同时保留，
缺任何一项都不对（数值见第 11 节）。

**推广：前轮纵向力。** 若前轴还有纵向力 \(F_{1xf}\)（例如前轮制动），前轴力变为
\((F_{1xf}\cos\delta-F_{1f}\sin\delta)\boldsymbol e_{x1}+(F_{1xf}\sin\delta+F_{1f}\cos\delta)\boldsymbol e_{y1}\)。
它在 (E2) 中多出 \(F_{1xf}\sin\delta\)，线性化为 \(F_{1xf,0}\delta\)；在 (E3) 中多出
\((a_1+d_1)F_{1xf,0}\delta\)。经过 9.7 节的行变换，第二行为
\((a_1+d_1)F_{1xf,0}\delta-d_1F_{1xf,0}\delta=a_1F_{1xf,0}\delta\)。所以

\[
\boldsymbol G_e=(C_{1f}+F_{1xf,0})\begin{bmatrix}1\\a_1\\0\end{bmatrix}.
\tag{W57}
\]

这与铰接力无关，但属于同一类"纵向力经转角投影到横向"的项。

---

## 10. 与第 2 篇推导过程的区别

### 10.1 总览

| 环节 | 第 2 篇（第 5–8 章） | 本文 |
|---|---|---|
| 出发方程 | 每车 Newton–Euler，显式写出 \(H\) | 同样的 Newton–Euler (W29)，但只使用它们的虚功率组合 |
| 线性化时机 | 先线性化再消元：(D40)、(D43)、(D45)、(D46)、(D48) 已是一阶形式 | 先精确消元得到 (E1)–(E4)，再线性化 |
| 铰接纵向力 | 在 (D44) 出现，随即由 (D45) 删去 | 在四条方程中自动抵消，无需近似 |
| 拖车加速度 | 线性化速度约束 (D58) 在 \(U\) 恒定下求导得 (D60) | 运动链 (W19) 直接求导，\(\dot U\) 自动出现 |
| 第一行 | (D72)：卡车沿 \(\boldsymbol e_{y1}\) 的方程加拖车沿 \(\boldsymbol e_{y2}\) 的方程，两个方向不同，\(H\) 只在 (D45) 下抵消 | (E2)：两车方程都投影到 \(\boldsymbol e_{y1}\)，铰接力按牛顿第三定律精确抵消 |
| 第二行 | 由拖车横向方程解出 \(H\)（(D75)–(D76)），代入卡车偏航方程（(D77)–(D79)） | (E3)：卡车对铰接点的力矩方程，不需要 \(H\)；第 2 篇第二行 = (E3) \(-\,d_1\times\)(E2) |
| 第三行 | \(H\) 代入拖车偏航方程（(D80)–(D82)），结构上就是拖车对铰接点的力矩方程 | (E4)：同一方程，但保留 \(-m_2a_2\sin\phi\,\dot U\) |
| 纵向方程 | 不出现 | (E1)：零阶给出 (W50)，一阶与横向解耦 |
| 适用范围 | 一阶、\(U\) 恒定、\(F_{hx,0}=0\) | (E1)–(E4) 对任意 \(\phi\)、\(\delta\)、\(U(t)\) 精确；线性化只要求横向量为小量 |
| 线性结果 | \(M_e,K_e,\boldsymbol G_e\)，\(\boldsymbol k_\phi=C_{2r}[-1,\ d_1,\ L_2]^T\) | \(M_e,K_e,\boldsymbol G_e\) 相同，\(\boldsymbol k_\phi\) 按 (W53) |
| 铰接力输出 | (D76) 给出 \(H\) 的一阶近似 | (W47) 精确回代 \(F_{hx}\)、\(H\) |

### 10.2 第 2 篇的三处近似在本文中是怎样消失的

**（1）(D45)：删去 \(-F_{hx}\sin\phi\)。**
第 2 篇需要这一步，是因为第一行 (D72) 把拖车方程投影到了 \(\boldsymbol e_{y2}\)，而卡车方程投影在
\(\boldsymbol e_{y1}\)。两个方向相差 \(\phi\)，卡车侧的 \(+H\) 和拖车侧的
\(-F_{hx}\sin\phi-H\cos\phi\) 不能精确抵消。

本文 (E2) 把拖车方程也投影到 \(\boldsymbol e_{y1}\)，拖车所受铰接力的投影恰好是
\(-\boldsymbol e_{y1}^T\boldsymbol\Lambda=-H\)，与卡车侧的 \(+H\) 精确抵消。
第 2 篇删掉的 \(-F_{hx,0}\phi\) 也有了明确去向：由式 (W51) 把它拆成
\(-F_{2x,0}\phi\) 和 \(+m_2a_x\phi\) 两部分。前者就是 (E2) 中拖车纵向外力的投影
\(-F_{2x}\sin\phi\)；后者与拖车惯性项中的 \(m_2a_x\phi\)（式 (W23) 的 \(a_{hx}\sin\phi\)）
相消——(E2) 用 \(\boldsymbol e_{y1}\) 投影，惯性项 (W21) 里本来就不含 \(\dot U\)。

**（2）(D60)：恒速下的约束导数。**
第 2 篇先把位置级约束线性化成 (D58)，再在 \(U\) 恒定下求导。本文直接对运动链求导得到
(W19)，\(\dot U\) 包含在 \(a_{hx}\) 中。在四条方程里，\(\dot U\) 只出现在 (E1) 的
\((m_1+m_2)\dot U\) 和 (E4) 的 \(-m_2a_2\sin\phi\,\dot U\) 中，(E2)、(E3) 完全不含 \(\dot U\)。
这解释了第 2 篇语言下的"抵消"：(D152) 补上的 \(\dot U\phi\) 与 \(-F_{hx,0}\phi\) 中的
\(m_2a_x\phi\) 部分，在横向那一行正好相消。

**（3）行的构造方式。**
第 2 篇的第一行不是整车动量在某一个方向上的投影，而是两车在不同方向上投影的和；
只有在 (D45) 下它才与 (E2) 一阶等价。第二行需要先从拖车横向方程解出 \(H\)，
因此继承了 (D45) 和恒速两个近似。第三行在第 2 篇中结构上已是拖车对铰接点的力矩方程，
铰接力（包括 \(F_{hx}\)）在其中本来就会抵消，唯一的近似是恒速。

### 10.3 为什么第 2 篇的结果在其假设下仍然正确

在一阶、\(a_x=0\)、\(F_{2x,0}=0\) 时，(E2L) = (D74)，(E3L) \(-\,d_1\times\)(E2L) = (D79)，
(E4L) = (D82)，三行完全相同，矩阵也就完全相同。所以第 2 篇的近似只影响
\(\boldsymbol k_\phi\)，而且只在 \(F_{2x,0}\ne0\) 或 \(a_x\ne0\) 时才产生差别。

### 10.4 对第 1 篇 1.4.2、1.4.3 节和第 2 篇第 11.2 节的修正

1. **两个修正不能分开做。** 第 1 篇 1.4.2 节（(K8a)、(D45a)）处理 \(F_{hx}\)，
   1.4.3 节和第 2 篇式 (D152)–(D153) 处理 \(\dot U\phi\)，写法上是两条独立的边界。
   实际上二者通过式 (W51) 耦合：\(F_{hx,0}=F_{2x,0}-m_2a_x\)。由式 (W56)，
   只补 \(F_{hx}\) 项会漏掉 \(m_2a_x[-1,\ d_1,\ a_2]^T\)，只补 \(\dot U\phi\) 会漏掉
   \(F_{hx,0}[-1,\ d_1,\ 0]^T\)，两者都补才得到正确的 (W53)。
2. **不需要增加维数。** (K8a) 的失效条件写道"必须把 \(F_{hx}\) 显式引入 (K8)，
   模型维数和输入都会增加"。由 9.2 节，一阶意义下只有工作点量 \(a_x\)、\(F_{2x,0}\)
   进入横向模型；铰接纵向力的扰动 \(\delta F_{hx}\) 只以 \(\delta F_{hx}\phi\) 的二阶形式出现。
   所以只需把 \(a_x\)、\(F_{2x,0}\) 作为调度参数，状态和输入维数不变。
3. **1.4.3 节的 17% 估计。** 该估计比较的是单独的 \(\dot U\phi\) 与 \(U(r_1-r_2)\)。
   但在正确的模型中，\(\dot U\phi\) 不会单独出现：它在横向行中与 \(m_2a_x\phi\) 抵消，
   剩下的是拖车铰接点力矩行中的 \(m_2a_2a_x\phi\)。按仓库参数，\(a_x=-2.5\,\mathrm{m/s^2}\) 时
   \(m_2a_2|a_x|/(L_2C_{2r})=18000\times4\times2.5/(7\times500000)\approx5\%\)。
4. **过时的引用。** 第 2 篇式 (D153) 之后写道"它会进入第 8 章的 \(\boldsymbol s\) 向量，
   从而改变 \(A_q\)"，但当前第 8 章采用直接消元，并没有 \(\boldsymbol s\) 向量和 \(A_q\)。
   按本文，变速修正落在 \(\boldsymbol k_\phi\) 的第三个分量上，见式 (W53)。

### 10.5 两个新增项的物理意义

**\(m_2a_2a_x\phi\)：加速参考系中的复摆。** 在随铰接点平动的参考系中，拖车质心受惯性力
\(-m_2\boldsymbol a_{P_h}\)，零阶为 \(-m_2a_x\boldsymbol e_{x1}\)。拖车质心相对铰接点在
\(-a_2\boldsymbol e_{x2}\)，由式 (W6)，该惯性力对铰接点的力矩为

\[
(-a_2\boldsymbol e_{x2})\times(-m_2a_x\boldsymbol e_{x1})
=m_2a_2a_x\,(\boldsymbol e_{x2}\times\boldsymbol e_{x1})
=m_2a_2a_x\,\boldsymbol e_{y2}^T\boldsymbol e_{x1}
=m_2a_2a_x\sin\phi .
\]

\(\phi>0\) 表示卡车相对拖车左偏。加速（\(a_x>0\)）时该力矩为正，使 \(r_2\) 增大、
\(\phi\) 减小，拖车像挂在铰接点后方的摆一样被"拉直"，等效恢复刚度为 \(m_2a_2a_x\)。
只用卡车制动（\(a_x<0\)、拖车不制动）时，拖车变成倒立摆，刚度为负，铰接角趋于发散——
这就是折叠（jackknife）倾向。

**\(F_{2x,0}[-1,\ d_1,\ 0]^T\)：拖车自身阻力或制动力。** 这个力沿拖车轴线作用，
作用线通过铰接点，所以对铰接点没有力矩，(E4) 不受影响。但在整车横向平衡 (E2) 中，
它在卡车左向的投影为 \(-F_{2x,0}\sin\phi\)；经过 9.7 节的行变换，它在卡车偏航方程中给出
\(d_1F_{2x,0}\phi\)。\(F_{2x,0}<0\)（阻力或拖车制动）、\(\phi>0\) 时该力矩为负，
使卡车向拖车方向回转，车组趋于拉直。拖车先于卡车制动（stretch braking）能稳定车组，
就是这个机理。

---

## 11. 数值校核

为验证本文推导，另写了一个不依赖本文推导的 Python 脚本：直接实现两车 6 个
Newton–Euler 方程和 2 个铰接加速度约束，每步求解 8 元线性方程组，同时得到 4 个加速度和
2 个铰接力分量。参数取 `tests/test_articulated_vehicle.cpp`：
\(m_1=8000\,\mathrm{kg}\)、\(I_1=25000\,\mathrm{kg\,m^2}\)、\(a_1=1.5\,\mathrm m\)、
\(b_1=d_1=2.5\,\mathrm m\)、\(C_{1f}=220\,\mathrm{kN/rad}\)、\(C_{1r}=300\,\mathrm{kN/rad}\)、
\(m_2=18000\,\mathrm{kg}\)、\(I_2=140000\,\mathrm{kg\,m^2}\)、\(a_2=4\,\mathrm m\)、
\(b_2=3\,\mathrm m\)、\(C_{2r}=500\,\mathrm{kN/rad}\)。卡车的驱动力和制动力都加在后轴。

### 11.1 精确方程

在 200 个随机状态下（\(U\in[5,25]\,\mathrm{m/s}\)、\(|v_{y1}|\le2\,\mathrm{m/s}\)、
\(|r_i|\le0.5\,\mathrm{rad/s}\)、\(|\phi|\le1\,\mathrm{rad}\)、\(|\delta|\le0.4\,\mathrm{rad}\)、
\(F_{1x}\in[-60,60]\,\mathrm{kN}\)、\(F_{2x}\in[-60,10]\,\mathrm{kN}\)），
式 (W43) 解出的加速度与带铰接力乘子的模型最大相对偏差为 \(1.45\times10^{-14}\)，即机器精度。

### 11.2 线性化

在直线工况对带乘子模型做中心差分线性化，与式 (W53) 比较，所有工况吻合到 \(10^{-13}\)。
下表比较四种 \(\boldsymbol k_\phi\) 处理方式的铰接角列相对误差（这一列与车速无关）：

| 工况 | \(F_{2x,0}\) | \(a_x\ (\mathrm{m/s^2})\) | \(F_{hx,0}\) | 第 2 篇 | 只补 \(F_{hx}\) | 只补 \(\dot U\phi\) | 本文 (W53) |
|---|---|---|---|---|---|---|---|
| 巡航 | −2 kN | 0 | −2 kN | 0.78% | 0 | 0.78% | 0 |
| 牵引 | −2 kN | +1 | −20 kN | 5.30% | 2.21% | 7.52% | 0 |
| 仅卡车制动 | −2 kN | −2 | +34 kN | 9.46% | 5.01% | 14.48% | 0 |
| 按质量比例制动 | −54 kN | −3 | 0 | 7.05% | 7.05% | 0 | 0 |
| 仅拖车制动 | −26 kN | −1 | −8 kN | 5.40% | 2.29% | 3.11% | 0 |
| 仅卡车制动 | −2 kN | −4 | +70 kN | 21.71% | 11.01% | 32.72% | 0 |

"只补 \(F_{hx}\)"只在 \(a_x=0\) 时正确；"只补 \(\dot U\phi\)"只在 \(F_{hx,0}=0\) 时正确，
在牵引和仅卡车制动工况下甚至比第 2 篇更差。

### 11.3 对稳定性的影响

默认参数下这组车是过度转向的：驱动轴 \(C_{1r}=300\,\mathrm{kN/rad}\) 要承担约 105 kN 载荷
（其中约 76 kN 来自牵引销），而前轴 \(C_{1f}=220\,\mathrm{kN/rad}\) 只承担约 49 kN。
线性模型的静态发散临界车速如下：

| 工况 | 临界车速 (m/s) |
|---|---|
| 第 2 篇（\(a_x=0\)、\(F_{2x,0}=0\)） | 17.55 |
| 巡航，拖车阻力 2 kN | 17.75 |
| 牵引 \(a_x=+1\) | 18.60 |
| 仅卡车制动 \(a_x=-2\) | 15.85 |
| 仅卡车制动 \(a_x=-4\) | 13.55 |
| 按质量比例制动 \(a_x=-3\) | 19.60 |
| 仅拖车制动 \(a_x=-1\) | 18.90 |

趋势与 10.5 节的物理解释一致：牵引和拖车制动提高临界车速，只用卡车制动降低临界车速。
若把 \(C_{1r}\) 换成与轴荷相称的 600 kN/rad，车组在 60 m/s 以内没有临界车速；
此时 15 m/s 下稳态铰接角增益的变化为：巡航 −0.7%，牵引 \(+1\,\mathrm{m/s^2}\) 为 −6.3%，
仅卡车制动 \(-2\)、\(-4\,\mathrm{m/s^2}\) 分别为 +12.5%、+29.8%，按质量比例制动 −2.5%。

---

## 附录 A：与 Kane 方法、零空间投影和 Lagrange 方程的关系

**Kane 方法。** 以 \(\boldsymbol w\) 为广义速率，6.4 节表中的四组虚速度就是偏速度，
式 (W38) 对四个基向量写出的四个方程就是 Kane 方程 \(F_k+F_k^*=0\)：
右边是广义主动力 \(F_k\)，左边取负是广义惯性力 \(F_k^*\)。

**零空间投影。** 把六个体速度记为
\(\boldsymbol\nu=[\boldsymbol v_{O_1};\ \omega_1;\ \boldsymbol v_{O_2};\ \omega_2]\)，
铰接约束写成 \(G\boldsymbol\nu=\boldsymbol 0\)（两车算出的铰接点速度之差为零），
式 (W29) 写成 \(M_b\dot{\boldsymbol\nu}=\boldsymbol f_{\mathrm{ext}}+G^T\boldsymbol\lambda\)，
其中 \(\boldsymbol\lambda\) 就是铰接力。取 \(\boldsymbol\nu=S\boldsymbol w\)，
\(S\) 的列张成 \(G\) 的零空间，即 \(GS=0\)。左乘 \(S^T\)：

\[
S^TM_bS\,\dot{\boldsymbol w}+S^TM_b\dot S\boldsymbol w
=S^T\boldsymbol f_{\mathrm{ext}}+(GS)^T\boldsymbol\lambda
=S^T\boldsymbol f_{\mathrm{ext}} .
\]

铰接力因 \((GS)^T=0\) 而消失，这与式 (W37) 是同一件事的矩阵表述；
\(S^TM_bS\) 就是式 (W46)。

**Lagrange 方程。** 取广义坐标 \((X_h,Y_h,\theta_1,\theta_2)\)（铰接点全局位置和两车航向），
动能
\(T=\tfrac12m_1|\boldsymbol v_{O_1}|^2+\tfrac12I_1\dot\theta_1^2
+\tfrac12m_2|\boldsymbol v_{O_2}|^2+\tfrac12I_2\dot\theta_2^2\)。
铰接力在与约束相容的虚位移上不做功，所以不出现在广义力中。四个 Lagrange 方程对应的
虚运动是"整车沿全局 \(X\)、\(Y\) 平移"和"两车分别绕铰接点转动"，它们是 (E1)–(E4)
的线性组合（把全局 \(X\)、\(Y\) 方向旋转到 \(\boldsymbol e_{x1}\)、\(\boldsymbol e_{y1}\) 即可），
内容完全相同。附录 B 中 Chen 与 Tomizuka（1995）、Ghandriz 等（2024）和李道飞等（2022）
采用的正是这一路线。

---

## 附录 B：文献中的对应做法

[`papers/`](papers/) 目录下与铰接力处理有关的 11 篇论文按处理方式归为四类
（另有 Ljungqvist 等 2019 为纯运动学模型，用作第 2 篇第 3 章的文献对照，不列入下表）：

| 做法 | 论文 | 与本文、第 2 篇的关系 |
|---|---|---|
| 沿铰接力不做功的方向精确消元 | Gäfvert 与 Lindgärde 2001，期刊版 VSD 2004（[EN2](papers/EN2_Gafvert_2001_9DOFTractorSemitrailerHandlingModel.pdf)） | 与本文相同：两车平动方程在牵引车系中用精确旋转矩阵求和（原文式 (21)–(22)），转动方程对铰接点取矩，状态取铰接点速度 \((U,V,W)\)；另外含侧倾、俯仰和第五轮力矩传递 |
| Lagrange 方程，铰接力不出现 | Chen 与 Tomizuka 1995（[EN1](papers/EN1_Chen_1995_DynamicModelingTractorSemitrailerAHS.pdf)）、Ghandriz 等 2024（[EN5](papers/EN5_Ghandriz_2024_LCVTrajectoryFollowingNMPC.pdf)）、李道飞等 2022（[CN3](papers/CN3_LiDaofei_2022_EmergencyAvoidanceTrajectoryTracking.pdf)） | 与本文等价（附录 A）。EN5 不做小角度和恒速近似；CN3 在运动学中取了小角度近似 \(x_2=x_1-f-a_2\) |
| 显式铰接力，再用线性化约束消元 | Morrison 与 Cebon 2016（[EN3](papers/EN3_Morrison_2016_SideslipEstimationArticulatedHGV.pdf)）、Sharma 与 He 2024（[EN4](papers/EN4_Sharma_2024_StaticDynamicLateralStabilityTradeoff.pdf)）、周猛等 2025（[CN4](papers/CN4_ZhouMeng_2025_KeyParametersDynamicStability.pdf)）、赵树恩等 2019（[CN2](papers/CN2_ZhaoShuen_2019_AntiRolloverHierarchicalControl.pdf)）、彭涛等 2018（[CN1](papers/CN1_PengTao_2018_LaneChangeStabilityRegion.pdf)）、郑雪莲等 2013（[CN6](papers/CN6_ZhengXuelian_2013_SemitrailerDrivingStability.pdf)） | 与第 2 篇同一路线：横向方程相加，横摆方程对铰接点取矩，用线性化的铰接点侧向加速度约束闭合。纵向铰接力的处理：CN4 直接设铰接力垂直于牵引车纵轴；CN1 令挂车系分量等于牵引车系分量，与 (D45) 相同；EN4 保留力矩项 \(F_{fx}\gamma L_{fs}\)，但用恒速下的牵引车纵向方程代入后在线性化中消失；CN6 先写出含 \(\sin\theta\) 的精确投影（原文式 (2)），再取 \(\cos\theta\approx1\)、\(\sin\theta\approx0\)（第 58 页） |
| 显式铰接力，与约束精确联立 | 赵晋海等 2025（[CN5](papers/CN5_ZhaoJinhai_2025_ImprovedParticleFilterStateEstimation.pdf)） | 保留四个铰接力分量和精确力变换 \(F_{jy}=F'_{jy}\cos\theta-F'_{jx}\sin\theta\)，与精确速度约束及其导数联立求解，属于微分代数形式 |

EN2、EN5、CN5 三篇不做小角度和恒速近似，在直线工况线性化后会自然给出式 (W53) 的两项。
EN1 的线性控制模型以及 EN3、EN4、CN2、CN4 的线性模型都取恒速、忽略纵向铰接力，
相当于第 2 篇 \(a_x=0\)、\(F_{2x,0}=0\) 的特例。
