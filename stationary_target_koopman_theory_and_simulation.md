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

取 \(x=[R,\lambda,\sigma]^{\mathrm T}\)、\(u=A\)，系统是控制仿射形式 \(\dot x=f(x)+g(x)u\)。只要 \(R>0\)，它具有良定义状态流，因此 Koopman operator 存在。

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
\tag{5}
\]

选择

\[
z=[1,r_x,r_y,\gamma,\cos\gamma,\sin\gamma]^{\mathrm T}.
\]

由于

\[
\dot\gamma=\frac{u}{V},\qquad
\frac{\mathrm d}{\mathrm dt}\cos\gamma=-\frac{u}{V}\sin\gamma,\qquad
\frac{\mathrm d}{\mathrm dt}\sin\gamma=\frac{u}{V}\cos\gamma,
\]

可精确写成

\[
\boxed{\dot z=A_0z+uB_1z}.
\tag{6}
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

这是连续时间精确闭合。零阶保持离散化后会出现

\[
\sin(T_su/V),\qquad \cos(T_su/V),
\]

所以严格的离散模型更自然地写为 \(z_{k+1}=K(u_k)z_k\)，而不是固定的 \(A_dz_k+u_kB_dz_k\)。

## 5. 线性多步 EDMD predictor

双线性模型在滚动预测中包含 \(u_kB_1z_k\)，直接优化一般不是固定 Hessian 的凸 QP。为接入 KDPC，采用

\[
z_{k+1}=A_Kz_k+B_Ku_k+w_k,
\tag{7}
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
\tag{8}
\]

原状态满足 \(x_c=C_Kz\)。

### 5.1 一步 EDMDc

构造 \(Z_-,Z_+,U_-\)，求解

\[
\min_{A_K,B_K}
\|Z_+-A_KZ_--B_KU_-\|_F^2
+\rho\|[A_K\ B_K]\|_F^2.
\tag{9}
\]

### 5.2 直接多步预测

为避免一步误差反复传播，直接辨识

\[
\mathbf Z_k=\Theta_zz_k+\Theta_u\mathbf U_k+\mathbf W_k,
\tag{10}
\]

其中

\[
\mathbf Z_k=[z_{1|k}^{\mathrm T},\ldots,z_{N|k}^{\mathrm T}]^{\mathrm T},
\quad
\mathbf U_k=[u_{0|k},\ldots,u_{N-1|k}]^{\mathrm T}.
\]

这对应 Jong 等直接学习多步 prediction matrices 的思想。本实现使用全状态测量，是其输入输出历史 lifting 的全状态特例。

## 6. 插值初始状态与 QP

定义测量提升状态 \(z_k^m=\psi(x_k)\) 和上一时刻给出的当前预测 \(z_k^p=z_{1|k-1}^*\)。插值状态为

\[
z_{0|k}=(1-\xi_k)z_k^m+\xi_kz_k^p,\qquad0\le\xi_k\le1.
\tag{11}
\]

因此

\[
\mathbf Z_k
=\Theta_zz_k^m+\Theta_u\mathbf U_k
+\Theta_z(z_k^p-z_k^m)\xi_k.
\tag{12}
\]

令 \(w_k=[\mathbf U_k^{\mathrm T},\xi_k]^{\mathrm T}\)，则 \(\mathbf Z_k=c_k+M_kw_k\)。性能指标为

\[
J=
\mathbf Z_k^{\mathrm T}\bar Q_z\mathbf Z_k
+\mathbf U_k^{\mathrm T}\bar R\mathbf U_k
+z_{0|k}^{\mathrm T}Q_0z_{0|k}
+\Lambda\xi_k^2\|z_k^p-z_k^m\|_2^2.
\tag{13}
\]

代入式（12）后得到

\[
\min_{w_k}\frac12w_k^{\mathrm T}H_kw_k+f_k^{\mathrm T}w_k,
\tag{14}
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

## 8. 仿真设置

- 静止目标，标称速度 \(300\ {\rm m/s}\)；
- 采样周期 \(0.1\ {\rm s}\)；
- 最大侧向加速度 \(60\ {\rm m/s^2}\)；
- 35步直接预测，即3.5 s；
- 初始相对位置 \((6000,1500)\ {\rm m}\)；
- 初始航向角 \(25^\circ\)；
- 捕获半径25 m；
- 260条训练轨迹、60条测试轨迹。

压力测试仍使用静止目标，但实际速度降低3%、控制效能降低6%，用于检验模型失配，而不是目标机动。

## 9. 实际仿真结果

### 9.1 双线性闭合

解析双线性提升与原模型使用相同 RK4 积分器推进300步，最大状态差为

\[
5.17\times10^{-12}.
\]

这数值验证了式（6）的连续时间闭合关系。

### 9.2 线性 EDMD 预测

| 状态 | 一步 NRMSE | 35步 NRMSE |
|---|---:|---:|
| \(r_x/R_s\) | \(4.03\times10^{-5}\) | 0.0151 |
| \(r_y/R_s\) | \(1.72\times10^{-5}\) | 0.00642 |
| \(\gamma\) | \(4.40\times10^{-9}\) | \(1.44\times10^{-8}\) |

### 9.3 闭环捕获

| 工况 | 捕获 | 终止距离 | 捕获时间 | 最终航向角 | QP失败 |
|---|---:|---:|---:|---:|---:|
| 标称 | 是 | 9.40 m | 21.3 s | \(-1.67^\circ\) | 0 |
| 速度/效能失配 | 是 | 22.39 m | 21.9 s | \(-1.65^\circ\) | 0 |

两种工况的最大指令均达到 \(60\ {\rm m/s^2}\) 的约束边界。平均插值系数由标称工况的0.041增加到失配工况的0.092，说明失配时优化器更频繁地使用插值自由度，但该现象本身不构成递归可行性的理论证明。

## 10. 结论

该制导模型可以转化为 Koopman 模型。严谨表述是：

- 在笛卡尔位置和航向三角观测量下，存在精确有限维连续时间双线性表示；
- 为获得凸 QP，可以使用有限维线性输入多步 EDMD predictor；
- 线性 predictor 不是精确模型，残差应进入鲁棒性和稳定性分析；
- 当前实验验证了闭合关系、预测精度、约束满足和静止目标捕获能力，但没有证明完整递归可行性。

下一步最重要的理论工作是构造 time-to-go 横向误差状态，使终端条件成为真正的稳定原点，再建立终端集、ISS 和在线自适应更新。

## 11. 文件

- [主程序](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m)
- [数值指标](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/metrics.txt)
- [完整结果](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/stationary_target_results.mat)
- [预测图](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/prediction_validation.png)
- [闭环图](C:/Users/qiuya/Documents/koopman_guidance/KDPC_RSRFG-main/StationaryTargetKoopman/results/closed_loop.png)

