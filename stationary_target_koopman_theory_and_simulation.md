# 静止目标二维制导模型的 Koopman 理论、KDPC 推导与仿真验证

## 1. 核心结论

对于静止目标、恒定速度二维侧向制导模型：

1. 在 \(R>0\) 上可以定义 Koopman operator；
2. 直接使用 \(R,\lambda,\sigma\) 及有限个三角/有理函数，一般不能精确闭合为有限维 \(Az+Bu\)；
3. 改用相对笛卡尔位置和航向角三角函数，可以得到精确有限维连续时间双线性 Koopman 表示；
4. 为获得凸 QP，可在限定工作域内辨识线性输入多步 EDMD predictor，并将不完备性表示为预测残差；
5. 本次仿真验证了双线性闭合、线性多步预测和静止目标约束捕获，但没有复现完整的终端不变集定理。

相关理论参见 [Brunton 的 Koopman 笔记](<C:/Users/qiuya/Zotero/storage/9SS52VFK/Brunton - 2019 - Notes on Koopman Operator Theory.pdf>)、[Bevanda 等的综述](<C:/Users/qiuya/Zotero/storage/SUFPDGSG/Bevanda 等 - 2021 - Koopman operator dynamical models Learning, analysis and control.pdf>) 以及 [Jong 等的 KDPC 论文](<C:/Users/qiuya/Zotero/storage/EQYE282S/Jong 等 - 2024 - Koopman Data-Driven Predictive Control with Robust Stability and Recursive Feasibility Guarantees.pdf>)。

## 2. 原始制导模型

定义距离 \(R\)、视线角 \(\lambda\)、飞行路径角 \(\gamma\)、有符号航向误差 \(\sigma=\gamma-\lambda\) 和侧向加速度 \(A\)。动力学为

\[
\dot R=-V\cos\sigma,
\tag{1}
\]

\[
\dot\lambda=-\frac{V\sin\sigma}{R},
\tag{2}
\]

\[
\dot\gamma=\frac{A}{V},
\tag{3}
\]

\[
\dot\sigma=\frac{A}{V}+\frac{V\sin\sigma}{R}.
\tag{4}
\]

必须使用有符号的 \(\sigma\)。若使用 \(|\gamma-\lambda|\)，系统在零点不可微并且丢失转向方向。

取 \(x=[R,\lambda,\gamma]^{\mathrm T}\)、\(u=A\)，并令 \(\sigma=\gamma-\lambda\)。系统可写成控制仿射形式

\[
\dot x=f_0(x)+u f_1(x),
\]

其中

\[
f_0(x)=
\begin{bmatrix}
-V\cos(\gamma-\lambda)\\
-V\sin(\gamma-\lambda)/R\\
0
\end{bmatrix},
\qquad
f_1(x)=
\begin{bmatrix}
0\\0\\1/V
\end{bmatrix}.
\]

后续推导采用如下假设。

**假设 1.** 目标静止，导弹速度 \(V>0\) 为常数；控制输入 \(u(t)\) 分段连续且有界；在捕获前始终有 \(R(t)>0\)；航向角 \(\gamma\) 采用连续展开角，而不是仅定义在 \(\mathbb S^1\) 上的模 \(2\pi\) 角。

在这些条件下，对任意允许输入 \(u(\cdot)\)，系统在到达 \(R=0\) 之前具有唯一局部流映射

\[
\Phi_u^t:x_0\mapsto x(t;x_0,u).
\]

因此可以定义受控 Koopman 演化算子

\[
\mathcal K_u^t g(x)=g(\Phi_u^t(x)).
\]

对光滑观测量 \(g\)，对应的无穷小生成元为

\[
\mathcal L_u g
=\nabla g(x)^{\mathrm T}\big(f_0(x)+u f_1(x)\big)
=\mathcal L_0g+u\mathcal L_1g.
\tag{5}
\]

这里的 Koopman 表述应理解为受控 Koopman 生成元的表示，而不是自治系统中单个固定 Koopman operator 的表示。

## 3. 为什么原极坐标难以有限维闭合

若观测量包含 \(1/R,\sin\sigma,\cos\sigma\)，则

\[
\frac{\mathrm d}{\mathrm dt}\frac1R=\frac{V\cos\sigma}{R^2},
\]

\[
\frac{\mathrm d}{\mathrm dt}\sin\sigma
=\frac{u}{V}\cos\sigma+\frac{V\sin\sigma\cos\sigma}{R},
\]

\[
\frac{\mathrm d}{\mathrm dt}\cos\sigma
=-\frac{u}{V}\sin\sigma-\frac{V\sin^2\sigma}{R}.
\]

反复求导继续产生 \(R^{-n}\) 和更高阶三角乘积，形成无限观测链。因此，有限个常规极坐标观测量一般不能精确得到

\[
\dot z=Az+Bu.
\]

EDMD 仍可在有界工作域内拟合该形式，但必须承认并处理预测残差。

## 4. 精确有限维双线性表示

定义相对笛卡尔坐标

\[
r_x=R\cos\lambda,\qquad r_y=R\sin\lambda.
\]

由 \(\gamma=\lambda+\sigma\) 和式（1）--（2）可得

\[
\dot r_x=-V\cos\gamma,\qquad
\dot r_y=-V\sin\gamma.
\tag{6}
\]

推导为

\[
\begin{aligned}
\dot r_x
&=\dot R\cos\lambda-R\sin\lambda\,\dot\lambda\\
&=-V\cos\sigma\cos\lambda+V\sin\sigma\sin\lambda\\
&=-V\cos(\lambda+\sigma)=-V\cos\gamma,
\end{aligned}
\]

\[
\begin{aligned}
\dot r_y
&=\dot R\sin\lambda+R\cos\lambda\,\dot\lambda\\
&=-V\cos\sigma\sin\lambda-V\sin\sigma\cos\lambda\\
&=-V\sin(\lambda+\sigma)=-V\sin\gamma.
\end{aligned}
\]

选择

\[
z=[1,r_x,r_y,\gamma,\cos\gamma,\sin\gamma]^{\mathrm T}.
\]

考虑由这些观测量张成的有限维线性空间

\[
\mathcal V_z=\operatorname{span}
\{1,r_x,r_y,\gamma,\cos\gamma,\sin\gamma\}.
\]

下面证明该空间在 \(\mathcal L_0\) 和 \(\mathcal L_1\) 下均不变。

漂移生成元作用为

\[
\mathcal L_0 1=0,\quad
\mathcal L_0 r_x=-V\cos\gamma,\quad
\mathcal L_0 r_y=-V\sin\gamma,
\]

\[
\mathcal L_0\gamma=0,\quad
\mathcal L_0\cos\gamma=0,\quad
\mathcal L_0\sin\gamma=0.
\]

控制生成元作用为

\[
\mathcal L_1 1=0,\quad
\mathcal L_1 r_x=0,\quad
\mathcal L_1 r_y=0,\quad
\mathcal L_1\gamma=\frac1V,
\]

\[
\mathcal L_1\cos\gamma=-\frac1V\sin\gamma,\qquad
\mathcal L_1\sin\gamma=\frac1V\cos\gamma.
\]

所有结果仍落在 \(\mathcal V_z\) 中，因此 \(\mathcal V_z\) 是受控 Koopman 生成元的不变有限维子空间。于是提升状态精确满足

\[
\boxed{\dot z=A_0z+uB_1z}.
\tag{7}
\]

非零矩阵元素为

\[
A_0(2,5)=-V,\qquad A_0(3,6)=-V,
\]

\[
B_1(4,1)=\frac1V,\qquad
B_1(5,6)=-\frac1V,\qquad
B_1(6,5)=\frac1V.
\]

更完整地，

\[
A_0=
\begin{bmatrix}
0&0&0&0&0&0\\
0&0&0&0&-V&0\\
0&0&0&0&0&-V\\
0&0&0&0&0&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0
\end{bmatrix},
\quad
B_1=
\begin{bmatrix}
0&0&0&0&0&0\\
0&0&0&0&0&0\\
0&0&0&0&0&0\\
1/V&0&0&0&0&0\\
0&0&0&0&0&-1/V\\
0&0&0&0&1/V&0
\end{bmatrix}.
\]

因此，式（7）不是近似 EDMD 模型，而是由所选观测子空间精确闭合得到的连续时间双线性 Koopman 表示。该结论的边界也很明确：若速度 \(V\) 变为状态、目标机动进入相对运动、存在自动驾驶仪动态，或必须在 \(R=0\) 继续推进原极坐标模型，上述 6 维空间一般不再闭合。

### 4.1 零阶保持离散化

若采样区间 \([kT_s,(k+1)T_s)\) 内控制保持常值 \(u_k\)，则精确离散提升模型为

\[
z_{k+1}=K(u_k)z_k,\qquad
K(u_k)=\exp\{T_s(A_0+u_kB_1)\}.
\tag{8}
\]

由于 \(A_0+u_kB_1\) 依赖 \(u_k\)，该离散映射是输入参数化的线性映射，而不是固定矩阵的线性输入模型。尤其是航向三角子系统满足

\[
\begin{bmatrix}
\cos\gamma_{k+1}\\ \sin\gamma_{k+1}
\end{bmatrix}
=
\begin{bmatrix}
\cos\Delta_k&-\sin\Delta_k\\
\sin\Delta_k&\cos\Delta_k
\end{bmatrix}
\begin{bmatrix}
\cos\gamma_k\\ \sin\gamma_k
\end{bmatrix},
\qquad
\Delta_k=\frac{T_su_k}{V}.
\]

因此零阶保持离散化后会出现 \(\sin(T_su_k/V)\) 和 \(\cos(T_su_k/V)\)。只有在小采样周期或小控制输入的一阶近似下，才可写成

\[
z_{k+1}\approx (I+T_sA_0)z_k+u_kT_sB_1z_k.
\]

这说明严格的离散模型更自然地写为 \(z_{k+1}=K(u_k)z_k\)，而不是固定的 \(A_dz_k+u_kB_dz_k\)。

## 5. 线性多步 EDMD predictor

双线性模型在滚动预测中包含 \(u_kB_1z_k\)，直接优化一般不是固定 Hessian 的凸 QP。为接入 KDPC，采用

\[
z_{k+1}=A_Kz_k+B_Ku_k+w_k,
\tag{9}
\]

其中 \(w_k\) 是有限维投影、离散化和工作域外推误差。

仿真使用归一化状态

\[
x_c=[r_x/R_s,r_y/R_s,\gamma]^{\mathrm T}
\]

和观测量

\[
\psi(x_c)=
[1,x_1,x_2,x_3,\cos x_3,\sin x_3,
x_1\cos x_3,x_1\sin x_3,
x_2\cos x_3,x_2\sin x_3,x_3^2]^{\mathrm T}.
\tag{10}
\]

原状态满足 \(x_c=C_Kz\)。

式（9）与式（7）的角色不同。式（7）是连续时间精确双线性模型；式（9）是为了得到在线凸 QP 而在有限工作域内辨识的线性输入 predictor。其残差可定义为

\[
w_k=\psi(x_{k+1})-A_K\psi(x_k)-B_Ku_k.
\tag{11}
\]

若存在紧工作域

\[
\mathcal X_c=\{x_c:\ R\ge R_{\min},\ |\gamma|\le\gamma_{\max},\ \|r\|\le r_{\max}\}
\]

和紧控制集 \(\mathcal U=[-u_{\max},u_{\max}]\)，且训练与闭环轨迹均限制在 \(\mathcal X_c\times\mathcal U\) 内，则可用测试集或覆盖采样估计残差界

\[
\|w_k\|_2\le \bar w,\qquad w_k\in\mathcal W.
\tag{12}
\]

任何关于约束满足、递归可行性或稳定性的严格结论，都应显式依赖该残差集合 \(\mathcal W\)，而不能只依赖名义 predictor。

### 5.1 一步 EDMDc

构造 \(Z_-,Z_+,U_-\)，求解

\[
\min_{A_K,B_K}
\|Z_+-A_KZ_--B_KU_-\|_F^2
+\rho\|[A_K\ B_K]\|_F^2.
\tag{13}
\]

### 5.2 直接多步预测

为避免一步误差反复传播，直接辨识

\[
\mathbf Z_k=\Theta_zz_k+\Theta_u\mathbf U_k+\mathbf W_k,
\tag{14}
\]

其中

\[
\mathbf Z_k=[z_{1|k}^{\mathrm T},\ldots,z_{N|k}^{\mathrm T}]^{\mathrm T},
\quad
\mathbf U_k=[u_{0|k},\ldots,u_{N-1|k}]^{\mathrm T}.
\]

这对应 Jong 等直接学习多步 prediction matrices 的思想。本实现使用全状态测量，是其输入输出历史 lifting 的全状态特例。

### 5.3 双线性 EDMD 与序列凸 MPC

为了更好匹配控制仿射结构，进一步辨识一步双线性 predictor：

\[
z_{k+1}=A_bz_k+B_0u_k+u_kB_1z_k+w_k.
\tag{14a}
\]

对应回归矩阵为

\[
\Omega_b=
\begin{bmatrix}
Z_-\\ U_-\\ U_-\odot Z_-
\end{bmatrix},
\]

其中 \(U_-\odot Z_-\) 表示每个样本的 \(u_kz_k\)。在线求解时，直接使用式（15）会产生非凸优化。本文采用序列凸近似：在第 \(j\) 次迭代中，用上一轮预测轨迹 \(\bar z_{i|k}^{(j)}\) 冻结输入矩阵

\[
B_{\rm eff,i}^{(j)}=B_0+B_1\bar z_{i|k}^{(j)},
\tag{14b}
\]

得到时变线性预测模型

\[
z_{i+1|k}=A_bz_{i|k}+B_{\rm eff,i}^{(j)}u_{i|k}.
\tag{14c}
\]

基于式（17）构造 QP，求解后更新控制序列和预测轨迹，重复少量迭代。当前代码使用3次序列凸迭代，并用阻尼更新控制序列，以避免双线性近似导致的数值振荡。

## 6. 插值初始状态与 QP

定义测量提升状态 \(z_k^m=\psi(x_k)\) 和上一时刻给出的当前预测 \(z_k^p=z_{1|k-1}^*\)。插值状态为

\[
z_{0|k}=(1-\xi_k)z_k^m+\xi_kz_k^p,\qquad0\le\xi_k\le1.
\tag{15}
\]

因此

\[
\mathbf Z_k
=\Theta_zz_k^m+\Theta_u\mathbf U_k
+\Theta_z(z_k^p-z_k^m)\xi_k.
\tag{16}
\]

令 \(\nu_k=[\mathbf U_k^{\mathrm T},\xi_k]^{\mathrm T}\)，则 \(\mathbf Z_k=c_k+M_k\nu_k\)。性能指标为

\[
J=
\mathbf Z_k^{\mathrm T}\bar Q_z\mathbf Z_k
+\mathbf U_k^{\mathrm T}\bar R\mathbf U_k
+z_{0|k}^{\mathrm T}Q_0z_{0|k}
+\Lambda\xi_k^2\|z_k^p-z_k^m\|_2^2.
\tag{17}
\]

代入式（16）或其序列凸时变线性预测矩阵后得到

\[
\min_{\nu_k}\frac12\nu_k^{\mathrm T}H_k\nu_k+f_k^{\mathrm T}\nu_k,
\tag{18}
\]

约束为

\[
-1\le u_{i|k}\le1,\qquad
0\le\xi_k\le1,\qquad
|\gamma_{i|k}|\le\gamma_{\max}.
\]

因此在线问题是凸 QP，并由 MATLAB quadprog 求解。

## 7. 捕获集与稳定性边界

恒速拦截是有限时间捕获问题。由于 \(V\ne0\)，\(r_x=r_y=0\) 不是原系统平衡点，到达目标后飞行器仍会继续运动。因此 Jong 等关于“原点为可稳定平衡点、终端不变集和 ISS”的理论不能原封不动套用。

本实验采用

\[
\sqrt{r_x^2+r_y^2}\le25\ {\rm m}
\]

作为捕获终端集。若要建立完整递归可行性证明，应进一步采用 time-to-go 横向误差系统、阶段切换模型、可达捕获集，或加入纵向速度控制。

更严谨地说，当前结果可以支持以下命题。

**命题 1.** 在假设 1 下，式（7）给出静止目标恒速二维制导模型的精确连续时间有限维双线性 Koopman 表示。

**命题 2.** 在零阶保持输入下，精确离散提升模型为式（8）。线性输入 EDMD predictor 式（9）是该离散映射在所选字典和工作域内的回归近似，其误差由式（11）定义。

**命题 3.** 若在线 QP 可行，且实际轨迹在预测时域内保持在训练工作域及残差界 \(\mathcal W\) 内，则 QP 解给出满足名义约束的 Koopman 预测轨迹；对真实非线性轨迹的约束保证还需要对 \(\mathcal W\) 做约束收紧或 tube 设计。

因此，当前文档不应声称已经证明了完整闭环渐近稳定性或递归可行性。可以声称的是：精确双线性闭合已证明，线性多步 predictor 已数值验证，闭环捕获已在给定仿真工况下观察到。

## 8. 面向递归可行性的 time-to-go 横向误差系统

为把有限时间捕获问题转化为具有稳定原点的调节问题，需要引入期望撞击方向和剩余时间坐标。设期望终端航向角为 \(\gamma_f\)，并定义

\[
e_f=
\begin{bmatrix}
\cos\gamma_f\\ \sin\gamma_f
\end{bmatrix},
\qquad
n_f=
\begin{bmatrix}
-\sin\gamma_f\\ \cos\gamma_f
\end{bmatrix}.
\]

其中 \(e_f\) 是期望撞击方向，\(n_f\) 是其法向方向。令

\[
s=e_f^{\mathrm T}r,\qquad y=n_f^{\mathrm T}r,\qquad
\theta=\gamma-\gamma_f,
\tag{19}
\]

其中 \(s\) 是沿期望撞击方向的剩余距离，\(y\) 是横向偏差，\(\theta\) 是终端航向误差。若采用几何 time-to-go

\[
\tau=\frac{s}{V},\qquad s>0,
\tag{20}
\]

则原始相对位置可写成

\[
r=V\tau e_f+yn_f.
\tag{21}
\]

因此终端条件 \(r=0,\gamma=\gamma_f\) 等价于

\[
\tau=0,\qquad y=0,\qquad \theta=0.
\]

这一步的意义是把捕获半径条件从 \(\|r\|\le r_c\) 改写成关于 \((\tau,y,\theta)\) 的终端原点邻域：

\[
\|r\|^2=V^2\tau^2+y^2.
\tag{22}
\]

在 \(s>0\) 且 \(|\theta|<\pi/2\) 的工作域内，

\[
\dot s=-V\cos\theta,\qquad
\dot y=-V\sin\theta,\qquad
\dot\theta=\frac{u}{V},
\tag{23}
\]

并且

\[
\dot\tau=-\cos\theta.
\tag{24}
\]

若以 \(\tau\) 作为独立变量，则

\[
\frac{\mathrm d y}{\mathrm d\tau}
=V\tan\theta,\qquad
\frac{\mathrm d\theta}{\mathrm d\tau}
=-\frac{u}{V\cos\theta}.
\tag{25}
\]

于是 \((y,\theta)=(0,0)\) 是横向误差系统的原点。与直接使用 \((r_x,r_y,\gamma)\) 不同，这个原点在剩余时间坐标下具有明确的终端含义：只要 \(\tau\to0\)、\(y\to0\)、\(\theta\to0\)，就有 \(r\to0\) 且 \(\gamma\to\gamma_f\)。

实际实现中可以避免在 \(\tau=0\) 附近除以 \(\tau\)，采用如下状态：

\[
\chi=[y,\theta]^{\mathrm T},
\qquad
\tau=\max(s/V,\tau_{\min}),
\tag{26}
\]

并在 \(\tau\le\tau_{\min}\) 或 \(\sqrt{V^2\tau^2+y^2}\le r_c\) 时切换到捕获终止逻辑。局部小角度近似为

\[
\dot y=-V\theta+O(\theta^3),\qquad
\dot\theta=\frac{u}{V}.
\tag{27}
\]

该近似是一个可稳定的二阶系统。取局部终端反馈

\[
u=\kappa(\chi)=V(k_y y-k_\theta\theta),
\qquad k_y>0,\quad k_\theta>0,
\tag{28}
\]

线性化闭环矩阵为

\[
A_c=
\begin{bmatrix}
0&-V\\
k_y&-k_\theta
\end{bmatrix},
\]

其特征多项式为

\[
\lambda^2+k_\theta\lambda+Vk_y.
\]

因此线性化横向误差原点指数稳定。选择任意正定矩阵 \(Q_f\)，令 \(P_f\succ0\) 满足

\[
A_c^{\mathrm T}P_f+P_fA_c=-Q_f.
\tag{29}
\]

则

\[
V_f(\chi)=\chi^{\mathrm T}P_f\chi
\]

可作为终端 Lyapunov 函数。由于非线性项在 \(\theta=0\) 附近为高阶项，存在足够小的 \(\alpha>0\)，使得终端集

\[
\mathcal X_f(\tau)=
\{\chi:\ \chi^{\mathrm T}P_f\chi\le\alpha,\ 
|\kappa(\chi)|\le u_{\max}-\eta_u,\ 
|\gamma_f+\theta|\le\gamma_{\max}-\eta_\gamma\}
\tag{30}
\]

在局部反馈 \(\kappa\) 下满足名义正不变性，其中 \(\eta_u,\eta_\gamma>0\) 是留给模型误差和预测残差的控制与航向裕度。若还要求

\[
V^2\tau^2+y^2\le r_c^2,
\tag{31}
\]

则该集合也是捕获终端集的子集。

### 8.1 递归可行性条件

将 KDPC 的预测状态从 \(x_c=[r_x/R_s,r_y/R_s,\gamma]^{\mathrm T}\) 改为

\[
x_\tau=[\tau,y,\theta]^{\mathrm T}
\quad\text{或}\quad
x_\tau=[s,y,\theta]^{\mathrm T},
\tag{32}
\]

并在代价中惩罚 \((y,\theta)\)，在终端约束中加入 \(\chi_N\in\mathcal X_f(\tau_N)\)。名义 MPC 问题可写成

若研究重点是打击时间约束，应优先采用日历剩余时间

\[
\tau_t=t_f-t
\tag{33}
\]

作为已知调度变量，而不是完全依赖几何估计 \(s/V\)。此时指定打击时间和打击角可统一写成

\[
\tau_{t,N}=0,\qquad y_N=0,\qquad \theta_N=0.
\tag{34}
\]

在采样实现中，严格等式通常放宽为容差约束

\[
|\tau_{t,N}|\le \varepsilon_t,\qquad
|y_N|\le \varepsilon_y,\qquad
|\theta_N|\le \varepsilon_\theta,
\tag{35}
\]

其中 \(\varepsilon_t\) 由采样周期决定，\(\varepsilon_y\) 对应允许脱靶量，\(\varepsilon_\theta\) 对应允许打击角误差。几何量 \(s=e_f^{\mathrm T}r\) 仍应保留，用于检查时间可行性和构造参考轨迹：

\[
r_{\rm ref}(t)=V(t_f-t)e_f,\qquad
\gamma_{\rm ref}(t)=\gamma_f.
\tag{36}
\]

因此，本文仿真将 Koopman predictor 的状态切换为

\[
x_\tau=[\tau/t_f,\ y/R_s,\ \theta]^{\mathrm T},
\qquad
\tau=s/V.
\]

在线 QP 中令 \(\tau/t_f\) 跟踪日历剩余时间 \((t_f-t)/t_f\)，并令 \(y\) 和 \(\theta\) 跟踪零。这样，\(\tau-(t_f-t)\) 表示沿期望撞击方向的时间误差，\(y\) 表示横向脱靶误差，\(\theta\) 表示打击角误差。

\[
\min_{\mathbf U,\xi}
\sum_{i=0}^{N-1}
\left(\chi_{i|k}^{\mathrm T}Q_\chi\chi_{i|k}
+u_{i|k}^{\mathrm T}Ru_{i|k}
+\Delta u_{i|k}^{\mathrm T}R_\Delta\Delta u_{i|k}\right)
+\chi_{N|k}^{\mathrm T}P_f\chi_{N|k}
\tag{37}
\]

其中

\[
\Delta u_{0|k}=u_{0|k}-u_{k-1},\qquad
\Delta u_{i|k}=u_{i|k}-u_{i-1|k},\quad i\ge1.
\]

该项用于抑制侧向加速度指令抖振，使优化结果更接近实际自动驾驶仪可执行的平滑指令。

并满足

\[
u_{i|k}\in\mathcal U,\qquad
\gamma_f+\theta_{i|k}\in[-\gamma_{\max},\gamma_{\max}],
\qquad
\chi_{N|k}\in\mathcal X_f(\tau_{N|k}).
\tag{38}
\]

当前实现中，归一化控制输入满足 \(-1\le u_{i|k}\le1\)，对应实际侧向加速度 \(|A_{i|k}|\le100\ {\rm m/s^2}\)。因此主要硬约束包括控制输入约束、航向路径约束、插值变量约束以及计划打击时刻的带松弛终端约束。

若名义模型无误差，且终端反馈 \(\kappa\) 使 \(\mathcal X_f\) 正不变，则标准移位论证成立：下一时刻可行控制序列由上一时刻最优序列删除第一项，并在末端追加 \(\kappa(\chi_N)\) 得到。因此名义递归可行性成立。

### 8.2 残差下的 ISS 表述

有限维 EDMD predictor 存在残差。把误差系统写成

\[
\chi_{k+1}=F_\chi(\chi_k,u_k,\tau_k)+d_k,
\qquad d_k\in\mathcal D.
\tag{39}
\]

若对所有 \(\chi\in\mathcal X_f\)、\(d\in\mathcal D\) 有

\[
V_f(F_\chi(\chi,\kappa(\chi),\tau)+d)-V_f(\chi)
\le
-\ell_f(\chi)+c_d\|d\|^2,
\tag{40}
\]

其中 \(\ell_f(\chi)\) 正定，则闭环关于预测残差满足 ISS 型界：

\[
\|\chi_k\|
\le
\beta(\|\chi_0\|,k)
+\gamma_d\left(\sup_{0\le j<k}\|d_j\|\right).
\tag{41}
\]

这意味着残差为零时横向误差收敛到原点；残差有界时，横向误差收敛到由 \(\mathcal D\) 决定的小邻域。为了把名义约束转化为真实约束，需要做约束收紧：

\[
\mathcal X_{\rm tight}=\mathcal X\ominus\mathcal E,
\qquad
\mathcal U_{\rm tight}=\mathcal U\ominus K\mathcal E,
\tag{42}
\]

其中 \(\mathcal E\) 是由 \(\mathcal D\) 和局部反馈增益诱导的误差管。

### 8.3 在线自适应更新

在线自适应的目标不是取消鲁棒设计，而是缩小残差集合 \(\mathcal D\)。对线性输入 predictor，可令

\[
\phi_k=
\begin{bmatrix}
z_k\\u_k
\end{bmatrix},
\qquad
z_{k+1}=K_k\phi_k+\varepsilon_k.
\tag{43}
\]

递推最小二乘更新为

\[
L_k=
\frac{P_k\phi_k}{\lambda+\phi_k^{\mathrm T}P_k\phi_k},
\]

\[
K_{k+1}=K_k+(z_{k+1}-K_k\phi_k)L_k^{\mathrm T},
\]

\[
P_{k+1}=\lambda^{-1}
\left(P_k-L_k\phi_k^{\mathrm T}P_k\right),
\tag{44}
\]

其中 \(0<\lambda\le1\) 是遗忘因子。为避免在线更新破坏可行性，需要加入投影和接受准则：

\[
K_{k+1}\leftarrow\Pi_{\mathcal K}(K_{k+1}),
\tag{45}
\]

并仅在滑动窗口验证残差不增大、参数变化有界、QP 仍可行时接受新模型。残差集合可同步更新为

\[
\bar d_{k+1}
=
\max\{(1-\rho)\bar d_k,\ \|\varepsilon_k\|+\delta_{\rm conf}\},
\tag{46}
\]

其中 \(\delta_{\rm conf}\) 是有限样本置信裕度。这样得到的理论闭环结构是：

1. 用 \(x_\tau=[\tau,y,\theta]\) 或 \([s,y,\theta]\) 作为终端稳定状态；
2. 用 \(\mathcal X_f(\tau)\) 和 \(V_f\) 建立名义递归可行性；
3. 用 \(\mathcal D\)、约束收紧和式（40）建立 ISS 鲁棒性；
4. 用投影 RLS 或滑动窗口 EDMD 在线缩小 \(\mathcal D\)，但不把自适应更新作为稳定性的唯一来源。

这一路线补齐后，理论表述可以从“仿真观察到捕获”提升为“在残差有界和约束收紧条件下，闭环对 time-to-go 横向误差具有递归可行性和 ISS 性质”。

## 9. 仿真设置

- 静止目标，标称速度 \(300\ {\rm m/s}\)；
- 采样周期 \(0.1\ {\rm s}\)；
- 最大侧向加速度 \(100\ {\rm m/s^2}\)；
- 35步直接预测，即3.5 s；
- 控制变化率惩罚 \(R_\Delta=1.5\)；
- 初始相对位置 \((6000,1500)\ {\rm m}\)；
- 初始航向角 \(25^\circ\)；
- 指定打击时间 \(21.7\ {\rm s}\)；
- 指定打击角度 \(-2^\circ\)；
- 捕获半径25 m；
- 260条训练轨迹、60条测试轨迹。

压力测试仍使用静止目标，但实际速度降低3%、控制效能降低6%，用于检验模型失配，而不是目标机动。

## 10. 实际仿真结果

### 10.1 双线性闭合

解析双线性提升与原模型使用相同 RK4 积分器推进300步，最大状态差为

\[
3.99\times10^{-11}.
\]

这数值验证了式（7）的连续时间闭合关系。

### 10.2 线性与双线性 EDMD 预测

| 状态 | 一步线性 NRMSE | 35步线性多步 NRMSE | 35步双线性滚动 NRMSE |
|---|---:|---:|---:|
| \(\tau/t_f\) | \(7.26\times10^{-5}\) | 0.0265 | \(2.77\times10^{-4}\) |
| \(y/R_s\) | \(3.28\times10^{-5}\) | 0.0119 | \(2.11\times10^{-4}\) |
| \(\theta\) | \(2.13\times10^{-9}\) | \(8.29\times10^{-9}\) | \(6.59\times10^{-8}\) |

预测图同时显示真实非线性轨迹、旧的线性多步 EDMD predictor 和当前控制器使用的双线性滚动 predictor。双线性 predictor 在 \(\tau/t_f\) 和 \(y/R_s\) 上的35步误差显著低于线性多步 predictor，这也是采用序列凸双线性 Koopman-MPC 的主要数值依据。

测试集95%一步预测残差为

\[
[4.12\times10^{-5},\ 1.68\times10^{-5},\ 2.04\times10^{-9}],
\]

对应 \([\tau/t_f,y/R_s,\theta]\)。当前实现用其20%作为终端约束收紧量，并保留至少25%的原始容差，避免有限样本残差估计使终端集退化为空集。

### 10.3 打击时间与打击角约束

| 工况 | 约束满足 | 计划时刻距离 | 时间误差 | 打击角误差 | 最大加速度 | QP失败 |
|---|---:|---:|---:|---:|---:|---:|
| 标称 | 是 | 6.01 m | 0.00 s | \(0.47^\circ\) | \(93.60\ {\rm m/s^2}\) | 0 |
| 速度/效能失配 | 否 | 190.75 m | 0.00 s | \(1.27^\circ\) | \(93.60\ {\rm m/s^2}\) | 0 |

当前 QP 没有失败。标称工况在 \(t_f=21.7\ {\rm s}\) 时进入25 m捕获半径，并满足打击角容差，说明 \([\tau/t_f,y/R_s,\theta]\) predictor、残差收紧、多面体终端集近似和序列凸双线性 Koopman-MPC 可以实现指定时间/角度约束。加入 \(\Delta u\) 惩罚后，标称工况平均归一化输入跳变量降至0.010，同时计划时刻脱靶量降至6.01 m。速度/效能失配工况仍未进入捕获半径，说明当前残差收紧还不足以覆盖该级别模型失配，需要进一步引入 tube 设计、在线自适应更新或更保守的可达时间选择。

## 11. 结论

该制导模型可以转化为 Koopman 模型。严谨表述是：

- 在笛卡尔位置和航向三角观测量下，存在精确有限维连续时间双线性表示；
- 为获得凸 QP，可以使用有限维线性输入多步 EDMD predictor；为提高末端精度，也可以使用双线性 EDMD predictor 和序列凸 Koopman-MPC；
- 线性 predictor 不是精确模型，残差应进入鲁棒性和稳定性分析；
- 当前代码已经采用 time-to-go 横向误差状态 \([\tau/t_f,y/R_s,\theta]\)，并加入计划打击时刻的带松弛终端约束、多面体终端集近似、基于测试残差的约束收紧，以及双线性 EDMD 序列凸 MPC；
- 若进一步加入严格终端 Lyapunov 集、残差 tube 和投影在线更新，则可以形成递归可行性与 ISS 证明框架；
- 当前实验验证了闭合关系、预测精度、QP 可行性，以及标称工况下的打击时间/角度/命中约束满足；失配工况仍需要鲁棒 tube 或在线自适应来恢复命中保证。

下一步最重要的实现工作是把当前多面体终端集近似升级为严格可证明的 \(\mathcal X_f(\tau)\) 与残差 tube，并加入投影在线自适应更新，以提高速度/效能失配下的命中保证。

## 12. 文件

- [主程序](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m)
- [数值指标](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/metrics.txt)
- [完整结果](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/stationary_target_results.mat)
- [预测图](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/prediction_validation.png)
- [闭环图](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/closed_loop.png)
