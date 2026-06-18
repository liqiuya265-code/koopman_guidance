# 二维制导模型的 Koopman-EDMD 建模与最优控制推导

## 1. 问题定义

考虑离散时间受控非线性制导系统

\[
x_{k+1}=F(x_k,u_k,d_k),\qquad y_k=h(x_k),
\]

其中 \(x\) 为相对运动和执行机构状态，\(u\) 为制导指令，\(d\) 为目标机动或模型扰动。EDMD 建模的首要条件不是系统“弱非线性”，而是所选 \(x\) 必须使系统近似满足 Markov 性：给定 \((x_k,u_k,d_k)\) 后，\(x_{k+1}\) 的条件分布不应再依赖未包含的历史量。

## 2. 常规二维相对运动模型

在 LOS 坐标系中，平面相对运动可写成

\[
\ddot R-R\dot q^2=a_{Tr}-a_{Mr},
\]

\[
R\ddot q+2\dot R\dot q=a_{Tq}-a_{Mq}.
\]

取

\[
x=\begin{bmatrix}R&\dot R&q&\dot q\end{bmatrix}^{\mathrm T},
\qquad u=a_{Mq},
\qquad d=\begin{bmatrix}a_{Tr}&a_{Tq}&a_{Mr}\end{bmatrix}^{\mathrm T},
\]

则

\[
\dot x_1=x_2,
\]

\[
\dot x_2=x_1x_4^2+a_{Tr}-a_{Mr},
\]

\[
\dot x_3=x_4,
\]

\[
\dot x_4=\frac{a_{Tq}-u-2x_2x_4}{x_1}.
\]

该模型是非线性、控制仿射且在 \(R>0\) 时具有良定义状态流，因此可以定义 Koopman operator，也可以用 EDMD 构建有限维近似模型。但 \(1/R\)、\(\dot R\dot q/R\) 和 \(R\dot q^2\) 会产生无限观测链，通常不存在简单的精确有限维线性闭合。

### 2.1 推荐的状态扩展

若考虑一阶自动驾驶仪

\[
\tau_a\dot a_M=-a_M+u_c,
\]

以及终端角约束，推荐采用

\[
x=\begin{bmatrix}
R&\dot R&q&\dot q&\gamma_M&a_M
\end{bmatrix}^{\mathrm T},
\qquad u=u_c.
\]

若目标机动可以估计，可将 \(\hat a_{Tr},\hat a_{Tq}\) 作为已知外生输入；若不可测，应将其视为有界扰动，或增加目标机动状态模型。不能在训练时随机改变目标机动，却既不把它放入状态/输入，也不把它作为随机扰动建模，否则同一个 \((x_k,u_k)\) 会对应多个 \(x_{k+1}\)，违背确定性 EDMD 模型的基本假设。

角度状态应先 unwrap，或增加

\[
s_q=\sin q,\qquad c_q=\cos q
\]

以避免 \(-\pi/\pi\) 跳变。LOS 模型只在

\[
R\in[R_{\min},R_{\max}],\qquad R_{\min}>0
\]

内训练和使用；到达 \(R_{\min}\) 后进入捕获终端集，不把模型推进到 \(R=0\)。

## 3. EDMDc 线性提升模型

选择包含原状态的观测向量

\[
z=\psi(x)=
\begin{bmatrix}
x\\
\eta(x)
\end{bmatrix}\in\mathbb R^{n_z}.
\]

对于 LOS 模型，可从以下物理特征开始：

\[
\eta(x)=\left[
\sin q,\cos q,\frac{1}{R},\frac{\dot R}{R},R\dot q,
\dot q^2,\frac{\dot R\dot q}{R},a_M\dot q
\right]^{\mathrm T}.
\]

不要一开始使用全部高阶多项式。建议先保留 20--50 个有物理意义的特征，再通过交叉验证、稀疏回归或神经 lifting 扩展。

由多条轨迹收集

\[
\mathcal D=\{x_k,u_k,d_k,x_{k+1}\}_{k=1}^{N_d},
\]

构造

\[
Z_-=[\psi(x_1),\ldots,\psi(x_{N_d})],
\]

\[
Z_+=[\psi(x_2),\ldots,\psi(x_{N_d+1})],
\]

\[
U_-=[u_1,\ldots,u_{N_d}],\qquad D_-=[d_1,\ldots,d_{N_d}].
\]

求解

\[
\min_{A_K,B_K,E_K}
\left\|Z_+-A_KZ_--B_KU_--E_KD_-\right\|_F^2
+\lambda\left\|[A_K\ B_K\ E_K]\right\|_F^2.
\]

其岭回归解为

\[
[A_K\ B_K\ E_K]
=Z_+\Omega^{\mathrm T}
(\Omega\Omega^{\mathrm T}+\lambda I)^{-1},
\]

\[
\Omega=\begin{bmatrix}Z_-\\U_-\\D_-\end{bmatrix}.
\]

如果原状态位于 \(\psi\) 的前几项，则

\[
x=C_Kz,\qquad C_K=[I\ 0].
\]

最终模型为

\[
z_{k+1}=A_Kz_k+B_Ku_k+E_Kd_k+w_k,
\]

其中 \(w_k\) 是有限维投影、数据和未建模动态造成的残差。

## 4. 为什么建议同时测试双线性 EDMD

LOS 方程包含

\[
-\frac{u}{R}.
\]

若观测量中包含 \(1/R\)，输入实际以 \(u\psi_j(x)\) 的形式作用。因此固定 \(B_K\) 的模型可能存在明显结构误差。更合适的模型是

\[
z_{k+1}=A_Kz_k+B_0u_k+u_kB_1z_k+E_Kd_k.
\]

其回归特征矩阵可写为

\[
\Omega_b=
\begin{bmatrix}
Z_-\\U_-\\U_-\odot Z_-\\D_-
\end{bmatrix},
\]

其中 \(U_-\odot Z_-\) 表示每个样本的 \(u_kz_k\)。线性 EDMDc 适合形成 QP；双线性 EDMD 更符合控制仿射结构，但需要时变 LQR、迭代 LQR 或序列凸 MPC。

## 5. EDMD 模型的合格判据

模型不应只检查一步拟合误差，至少需要：

1. 独立测试集上的 1、5、10、20 步滚动预测误差；
2. \(R,\dot R,q,\dot q,\gamma_M,a_M\) 各状态的归一化误差；
3. 训练包线边界和目标机动变化下的误差；
4. 约束相关输出的预测误差；
5. 闭环 MPC 中的可行率、约束违反率和计算时间；
6. 线性 EDMDc 与双线性 EDMD 的消融比较。

## 6. Koopman-LQR 最优控制律

暂不考虑扰动，设

\[
z_{k+1}=A_Kz_k+B_Ku_k,
\qquad x_k=C_Kz_k.
\]

对于平衡点 \((x_s,u_s)\)，令

\[
z_s=\psi(x_s),\qquad
z_s=A_Kz_s+B_Ku_s.
\]

定义误差

\[
\delta z_k=z_k-z_s,qquad
\delta u_k=u_k-u_s.
\]

则

\[
\delta z_{k+1}=A_K\delta z_k+B_K\delta u_k.
\]

选取有限时域性能指标

\[
J=\delta z_N^{\mathrm T}P_N\delta z_N+
\sum_{k=0}^{N-1}
\left(
\delta x_k^{\mathrm T}Q_x\delta x_k+
\delta u_k^{\mathrm T}R\delta u_k
\right).
\]

由于 \(\delta x=C_K\delta z\)，令

\[
Q_z=C_K^{\mathrm T}Q_xC_K.
\]

动态规划给出反向 Riccati 递推：

\[
K_k=
(R+B_K^{\mathrm T}P_{k+1}B_K)^{-1}
B_K^{\mathrm T}P_{k+1}A_K,
\]

\[
P_k=Q_z+A_K^{\mathrm T}P_{k+1}A_K
-A_K^{\mathrm T}P_{k+1}B_K
(R+B_K^{\mathrm T}P_{k+1}B_K)^{-1}
B_K^{\mathrm T}P_{k+1}A_K.
\]

最优控制律为

\[
u_k^*=u_s-K_k\big(\psi(x_k)-z_s\big).
\]

无限时域情况下，若 \((A_K,B_K)\) 可稳定且 \((A_K,Q_z^{1/2})\) 可检测，则 \(P_k\) 收敛到离散代数 Riccati 方程的解 \(P\)，得到常增益

\[
u_k^*=u_s-K\big(\psi(x_k)-z_s\big).
\]

这里的控制律对原状态是非线性的，因为 \(\psi(x)\) 是非线性映射。

## 7. Koopman-MPC 最优控制

当存在控制、视场角、过载和终端角约束时，LQR 不再足够，应采用 Koopman-MPC。

对预测时域 \(N_p\)，定义

\[
U=\begin{bmatrix}u_{0|k}^{\mathrm T}&\cdots&u_{N_p-1|k}^{\mathrm T}\end{bmatrix}^{\mathrm T}.
\]

堆叠预测状态：

\[
Z=\mathcal A z_k+\mathcal B U,
\]

其中

\[
\mathcal A=
\begin{bmatrix}
A_K\\A_K^2\\\vdots\\A_K^{N_p}
\end{bmatrix},
\]

\[
\mathcal B=
\begin{bmatrix}
B_K&0&\cdots&0\\
A_KB_K&B_K&\cdots&0\\
\vdots&\vdots&\ddots&\vdots\\
A_K^{N_p-1}B_K&A_K^{N_p-2}B_K&\cdots&B_K
\end{bmatrix}.
\]

原状态预测为

\[
X=\bar C Z,\qquad \bar C=I_{N_p}\otimes C_K.
\]

选择

\[
J=(X-X_{\mathrm ref})^{\mathrm T}\bar Q(X-X_{\mathrm ref})
+U^{\mathrm T}\bar R U.
\]

代入预测方程后得到标准 QP：

\[
\min_U\frac12U^{\mathrm T}HU+f(z_k)^{\mathrm T}U,
\]

其中

\[
H=2(\mathcal B^{\mathrm T}\bar C^{\mathrm T}\bar Q\bar C\mathcal B+\bar R),
\]

\[
f(z_k)=2\mathcal B^{\mathrm T}\bar C^{\mathrm T}\bar Q
(\bar C\mathcal A z_k-X_{\mathrm ref}).
\]

控制约束、状态约束和终端约束统一写成

\[
GU\le h+Sz_k.
\]

每个采样时刻求解

\[
U_k^*=\arg\min_U\frac12U^{\mathrm T}HU+f(z_k)^{\mathrm T}U
\quad\text{s.t.}\quad GU\le h+Sz_k,
\]

并实施第一项

\[
u_k=\begin{bmatrix}I&0&\cdots&0\end{bmatrix}U_k^*.
\]

这就是基于 Koopman 预测模型的约束最优控制律。无约束时，其解退化为有限时域线性二次最优控制；有约束时，控制律由在线 QP 隐式给出。

## 8. 双线性 Koopman 模型的最优控制

若模型为

\[
z_{k+1}=A_Kz_k+B_0u_k+u_kB_1z_k,
\]

则不能直接使用固定矩阵 Riccati 方程。定义

\[
B_{\mathrm eff}(z_k)=B_0+B_1z_k,
\]

可写成

\[
z_{k+1}=A_Kz_k+B_{\mathrm eff}(z_k)u_k.
\]

实用求解方法为：

1. 用上一轮预测轨迹 \(\bar z_{i|k}\) 固定 \(B_{\mathrm eff}\)；
2. 得到时变线性模型；
3. 求解时变 LQR 或 QP；
4. 更新预测轨迹并迭代，直到控制序列收敛。

这相当于 lifted-space iLQR 或 sequential convex Koopman-MPC。

## 9. 稳定性与模型误差

有限维 Koopman 模型一定存在残差：

\[
z_{k+1}=A_Kz_k+B_Ku_k+w_k.
\]

因此，仅在提升空间设计终端权重和终端集，不自动保证原非线性系统稳定。可采用：

1. 估计 \(w_k\in\mathcal W\)，设计 tube Koopman-MPC 和约束收紧；
2. 采用多步预测学习，减少一步模型误差的递推传播；
3. 使用插值初始提升状态维持递归可行性；
4. 将闭环稳定性表述为相对于 Koopman 预测误差的 ISS；
5. 通过残差触发 RLS/滑动窗口 EDMD 在线更新 \(A_K,B_K\)。

## 10. 推荐实施路线

1. 用无机动目标和理想执行机构的 \([R,\dot R,q,\dot q]\) 模型验证 EDMD；
2. 比较线性 EDMDc 和双线性 EDMD；
3. 加入 \(\gamma_M,a_M\) 和一阶自动驾驶仪；
4. 加入目标机动作为可测外生输入或有界扰动；
5. 首先实现 Koopman-LQR，确认稳定和跟踪行为；
6. 再实现含视场角、过载、终端角约束的 Koopman-MPC；
7. 最后增加模型误差集合、递归可行设计和在线更新。
