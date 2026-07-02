# Koopman Guidance 论文与仿真修改记录

记录日期：2026-07-01

本文档记录当前 Koopman 制导论文与 MATLAB 仿真代码中已经进行过的主要尝试、代码修改、仿真结果和阶段性结论。目的是方便后续论文写作、结果筛选和方法复盘。

## 1. 论文结构调整

### 1.1 总体思路调整

原论文写法容易让读者认为整个方法一开始就是围绕带打击时间、打击角约束的复杂 Koopman 模型展开。经过讨论后，论文结构调整为：

1. 先写不加任何终端约束的理想制导核心模型。
2. 再说明该理想模型在 Cartesian 相对坐标下存在精确有限维双线性 Koopman 结构。
3. 然后扩展到带打击角、打击时间约束的 terminal-error 模型。
4. 最后写实际使用的 EDMD 数据驱动 Koopman-MPC 和仿真。

这样处理的逻辑是：

- 理想模型给出理论来源；
- 约束模型给出任务需求；
- EDMD/MPC 给出可实现控制器。

### 1.2 论文中 Koopman lift 的表述

理论部分采用理想模型的 lift：

```math
z_c = [1,\ r_x,\ r_y,\ \gamma,\ \cos\gamma,\ \sin\gamma]^T
```

该 lift 用于说明理想 Cartesian 运动模型可以写成精确双线性 Koopman 结构。

实际控制部分采用任务相关的 lifted dictionary。原始版本为：

```math
\psi(x_\tau,u_a)=
[1,\tau,y,\theta,u_a,\cos\theta,\sin\theta,
\tau\cos\theta,\tau\sin\theta,
y\cos\theta,y\sin\theta,\theta^2,
u_a\cos\theta,u_a\sin\theta]^T
```

后来为了处理大打击角问题，增加了可选的 `sincos` 表达，把角度状态从 `theta` 改为 `sin(theta), cos(theta)`。

### 1.3 论文文件

主要修改文件：

- `koopman_guidance_ieee_draft.tex`
- `koopman_guidance_ieee_draft.pdf`

论文当前已同步到 GitHub，提交号：

```text
977f427 Update Koopman guidance simulations and manuscript
```

## 2. 主程序和 QP 控制器修改

### 2.1 主程序

核心主程序为：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

该文件中实际求解 QP 的函数为：

```matlab
solve_kdpc_qp(...)
```

QP 最终调用：

```matlab
quadprog(H,f,Aineq,bineq,[],[],lb,ub,[],c.options)
```

### 2.2 QP 的含义

当前 QP 的作用是：在 Koopman 预测模型下，优化未来控制序列，使导弹在预测窗口内尽量满足位置和打击角要求，同时限制控制幅值、抑制控制抖动，并允许必要的终端 slack。

代价函数包括：

```math
J =
J_{\rm tracking}
+J_{\rm control}
+J_{\Delta u}
+J_{\xi}
+J_{\rm slack}
```

其中：

- `J_tracking`：预测状态跟踪参考；
- `J_control`：控制输入大小惩罚；
- `J_delta u`：控制变化率软惩罚，对应 `Rd`；
- `J_xi`：measured lifted state 与 previous predicted lifted state 之间的插值惩罚；
- `J_slack`：终端约束松弛惩罚。

### 2.3 新增 QP 功能

主程序中新增或扩展了以下功能：

1. `horizonOverride`：覆盖预测时域。
2. `qxOverride`、`qterminalOverride`：覆盖状态权重和终端权重。
3. `rbarScaleOverride`、`rdOverride`：覆盖控制幅值惩罚和控制变化惩罚。
4. `slackPenaltyOverride`：覆盖终端 slack 惩罚。
5. `duMaxOverride`：加入硬约束

```math
|u_k-u_{k-1}| \leq \Delta u_{\max}
```

6. `enableImpactStageCostOverride`：将终端高权重对准真实打击时刻。
7. `enableTerminalRefinementOverride`：末端几秒切换到 NMPC 的尝试。
8. `trainingDataModeOverride='largeAngle'`：大角度机动训练数据模式。
9. `angleStateModeOverride='sincos'`：用 `sin(theta), cos(theta)` 替代直接的 `theta`。
10. `angleOnlyTerminalTauOverride` 和 `angleOnlyZeroTauRefOverride`：不指定打击时间时仍将位置终端目标拉到目标点。

## 3. 打击时间约束相关尝试

### 3.1 初始问题

加入 40 s 打击时间约束后，效果较差。原因主要是：

- 自然命中时间约为 22 s 左右，而指定 40 s 命中需要显著拖延；
- 终端位置约束 5 m 很紧；
- 速度扰动会显著放大时间误差；
- Koopman 预测误差在终端时刻会被放大。

### 3.2 调参脚本

新增脚本：

```text
run_time_constraint_tuning.m
run_time_constraint_tuning_refined.m
run_time_window_scan_h50.m
run_time_constraint_tuning_position_priority.m
run_two_stage_cost_tuning.m
run_terminal_refinement_tuning.m
run_terminal_refinement_refined.m
run_terminal_refinement_minimal.m
```

### 3.3 主要结果

最初加打击时间约束时，名义脱靶量约为：

```text
521.47 m
```

经过 `h50-progress` 调整后，名义脱靶量改善为：

```text
18.54 m
```

加入 two-stage cost 后效果变差，最佳仍在 180 m 左右，不如 `h50-progress`。

### 3.4 terminal refinement NMPC 尝试

尝试末端几秒切换到原始动力学 NMPC：

- 3 s / 4 s / 5 s 末端接管；
- 10/14/18 个 move blocks；
- 多组终端权重。

结果：

- terminal NMPC 可以把 18.54 m 改善到约 10 m；
- 但没有进入 5 m 捕获半径；
- 后来发现真正有效的是 impact-stage terminal cost alignment。

### 3.5 impact-stage terminal cost alignment

关键发现：

原本终端高权重默认放在预测窗口最后一步，但真实打击时刻可能在预测窗口中间。将终端高权重放到预测窗口内真实打击时刻后，名义工况结果为：

```text
miss = 4.29 m
angle error = -1.74 deg
```

对应结果文件：

```text
results/terminal_refinement_refined_summary.csv
```

结论：

该方法比 terminal NMPC 接管更干净，也更符合 Koopman-MPC 主线。

## 4. 不考虑打击时间约束的主方案

### 4.1 主入口

当前不考虑打击时间约束的推荐主入口为：

```text
run_angle_only_no_time_check.m
```

该脚本固定：

```matlab
angleOnlyModeOverride = true;
impactGammaDegOverride = 30;
initialGammaDegOverride = -30;
angleOnlyMaxTimeOverride = 50;
rdOverride = 1.5;
duMaxOverride = 0.05;
```

### 4.2 结果

名义工况：

```text
miss = 3.05 m
angle error = 0.11 deg
```

速度扰动工况：

```text
miss = 4.15 m
angle error = 0.12 deg
```

结论：

不加打击时间约束后，问题明显更容易，名义和扰动工况都能满足 5 m 捕获半径和 3 deg 打击角误差要求。

## 5. 控制平滑性尝试

### 5.1 增大 `Rd`

新增脚本：

```text
run_angle_only_rd_scan.m
```

扫描：

```matlab
Rd = [0.9, 1.5, 3, 6, 10, 15]
```

主要结果：

| Rd | 名义脱靶量 | 角度误差 | RMS Delta A | 结论 |
|---:|---:|---:|---:|---|
| 0.9 | 3.15 m | -0.076 deg | 36.56 | 满足 |
| 1.5 | 2.51 m | -0.073 deg | 36.00 | 满足 |
| 3 | 3.42 m | -0.067 deg | 33.49 | 较好折中 |
| 6 | 6.07 m | -0.060 deg | 25.55 | 名义超 5 m |
| 10 | 6.12 m | -0.053 deg | 19.30 | 名义超 5 m |
| 15 | 1.53 m | -0.047 deg | 19.66 | 名义好，扰动略差 |

结论：

单纯增大 `Rd` 能改善平滑性，但效果不完全单调。`Rd=3` 是一个较稳的折中。

### 5.2 加硬输入变化率约束

新增脚本：

```text
run_angle_only_du_constraint_scan.m
```

加入硬约束：

```math
|u_k-u_{k-1}| \leq \Delta u_{\max}
```

扫描：

```matlab
duMax = [inf, 0.5, 0.3, 0.2, 0.1, 0.05]
```

最佳结果：

```text
duMax = 0.05
nominal miss = 3.05 m
stress miss = 4.15 m
RMS Delta A = 1.41 m/s^2
```

结论：

`duMax=0.05` 比单纯调大 `Rd` 更工程化，也更有效。它表示每个采样周期实际加速度最多变化约：

```text
5 m/s^2
```

## 6. 大打击角场景尝试

### 6.1 初始设置

用户要求初始航向角始终固定：

```text
gamma_0 = -30 deg
```

只改变打击角：

```text
gamma_f = 60 deg, 90 deg
```

对应脚本：

```text
run_angle_only_large_angle_scenarios.m
run_angle_only_large_angle_no_du_scenarios.m
```

### 6.2 初始结果

保留 `duMax=0.05` 时，60 deg 和 90 deg 均失败。

取消 `duMax` 后：

| 场景 | 名义脱靶量 | 角度误差 |
|---|---:|---:|
| 60 deg | 238.0 m | -15.9 deg |
| 90 deg | 5255.2 m | -48.7 deg |

结论：

当前小角度调好的 Koopman-MPC 不能直接推广到 60/90 deg。

## 7. 大角度训练数据扩展

### 7.1 新增训练数据模式

在 `run_stationary_target_demo.m` 中新增：

```matlab
trainingDataModeOverride = 'largeAngle';
```

该模式在期望打击方向坐标系下采样：

- 更宽的 along-track 距离；
- 更宽的 cross-track 偏差；
- 更大的航向误差；
- 更强的控制输入变化。

对应脚本：

```text
run_angle_only_large_angle_broad_data_scenarios.m
```

### 7.2 broad data 结果

使用原始 `theta` 表达时：

| 场景 | 脱靶量 | 角度误差 |
|---|---:|---:|
| 60 deg | 90.6 m | -4.82 deg |
| 90 deg | 5847.7 m | -123.9 deg |

结论：

扩大数据覆盖有帮助，60 deg 从 238 m 改善到 90.6 m，但仍不足够。

## 8. 将角度状态改为 sin/cos 表达

### 8.1 修改内容

新增：

```matlab
angleStateModeOverride = 'sincos';
```

状态输出从：

```math
[\tau,\ y,\ \theta]
```

变为：

```math
[\tau,\ y,\ \sin\theta,\ \cos\theta]
```

终端目标从：

```math
\theta \to 0
```

变为：

```math
\sin\theta \to 0,\quad \cos\theta \to 1
```

### 8.2 sin/cos 结果

| 场景 | raw theta 结果 | sin/cos 结果 |
|---|---:|---:|
| 60 deg | 90.6 m, -4.82 deg | 59.9 m, -2.93 deg |
| 90 deg | 5847.7 m, -123.9 deg | 2309.1 m, -54.7 deg |

结论：

`sin/cos` 表达确实改善了大角度预测和闭环效果，尤其 60 deg 的角度误差接近约束范围；但位置误差仍然较大。

## 9. 位置命中调参

### 9.1 问题

观察到 60 deg 场景中，轨迹常表现为：

```text
先直线飞行，然后末端急转
```

这说明控制器更像末端修正器，而不是全程轨迹塑形器。

### 9.2 新增位置命中开关

新增：

```matlab
angleOnlyTerminalTauOverride = true;
angleOnlyZeroTauRefOverride = true;
```

含义：

虽然不指定固定打击时间，但在预测窗口末端将目标位置也作为终端目标：

```math
\tau \to 0,\quad y\to0
```

### 9.3 调参脚本

新增：

```text
run_large_angle_position_tuning.m
```

### 9.4 结果

| 方法 | 打击角 | 脱靶量 | 角度误差 | 说明 |
|---|---:|---:|---:|---|
| baseline-sincos | 60 deg | 59.87 m | -2.93 deg | 角度接近，位置差 |
| strong-y-terminal | 60 deg | 30.75 m | 1.71 deg | 位置改善 |
| zero-tau-terminal | 60 deg | 4.67 m | -3.57 deg | 位置满足，角度略超 |
| zero-tau-h70 | 60 deg | 22.30 m | -10.88 deg | 变差 |
| zero-tau-90deg | 90 deg | 326.29 m | -39.54 deg | 仍失败 |

结论：

60 deg 已经可以把位置误差压入 5 m，但角度误差略超 3 deg，且 QP failures 较多。90 deg 仍不适合当前控制结构。

## 10. 当前阶段性结论

### 10.1 已经比较可靠的设置

30 deg、不考虑打击时间约束、初始航向 -30 deg：

```text
run_angle_only_no_time_check.m
```

推荐设置：

```matlab
rdOverride = 1.5;
duMaxOverride = 0.05;
```

该设置名义和扰动工况均满足要求。

### 10.2 60 deg 当前最接近成功的设置

```text
run_large_angle_position_tuning.m
```

其中：

```text
zero-tau-terminal
```

结果：

```text
miss = 4.67 m
angle error = -3.57 deg
```

位置已经满足，但角度略超。

### 10.3 90 deg 当前结论

90 deg 对当前初始条件和控制结构太难。即使加入：

- broad training data；
- sin/cos angle representation；
- zero-tau terminal position targeting；

仍然不能稳定满足要求。

### 10.4 主要原因

1. 大打击角需要提前绕飞，当前 MPC 更像短时域末端修正器。
2. 当前代价函数没有明确鼓励提前形成弧形/绕飞轨迹。
3. 预测窗口有限，大角度机动需要更长视野。
4. 终端约束过硬时 QP failures 增多。
5. 控制幅值、控制变化率约束会限制大机动能力。

## 11. 后续建议

### 11.1 针对 60 deg

下一步建议：

1. 在 `zero-tau-terminal` 基础上加强角度权重，但不要过度压位置；
2. 降低 QP failures；
3. 设计中段轨迹塑形参考，例如非零 `y_ref(tau)`；
4. 尝试圆弧或多项式参考路径，而不是一直要求 `y -> 0`。

### 11.2 针对 90 deg

下一步不建议继续只调权重。更合理的路线：

1. 分段制导：中段绕飞塑形，末端 Koopman-MPC 精修；
2. 设计大角度专用参考轨迹；
3. 增加更长预测时域或多阶段优化；
4. 重新定义 terminal reachable set，而不是只靠单一 QP 终端约束。

## 12. GitHub 同步记录

当前已同步到 GitHub：

```text
https://github.com/liqiuya265-code/koopman_guidance
```

最新提交：

```text
977f427 Update Koopman guidance simulations and manuscript
```

同步内容包括：

- 论文 `tex/pdf`；
- 主程序 `run_stationary_target_demo.m`；
- 新增调参和验证脚本；
- 关键汇总图和 summary csv；
- FOV 扩展相关文件。

未同步的大量文件主要是：

- 中间仿真 `.mat`；
- 大量单次扫描图；
- 外部论文 PDF；
- 临时 LaTeX 文件。

这些文件保留在本地，未主动推送到 GitHub。

## 13. 联合参考中的 `tauScale` 调整

用户提出：在联合参考轨迹基础上继续调整 `tau`。

新增脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_joint_tau_scan.m
```

该脚本保持如下条件不变：

- 打击角约束：`gamma_f = 60 deg`；
- 初始航向角：`gamma_0 = -30 deg`；
- 不加入打击时间约束；
- 使用大角度训练数据；
- 角度状态采用 `sin(theta), cos(theta)`；
- 终端位置采用 `zero-tau` 方式；
- 联合参考同时给出 `y_ref(tau)` 和 `theta_ref(tau)`。

扫描参数：

```text
amplitude = 0.01, 0.02
tauScale  = 0.25, 0.35, 0.45, 0.60, 0.75, 0.90, 1.10
```

关键结果：

| case | miss | angle error | stress miss | stress angle |
|---|---:|---:|---:|---:|
| zero-tau-baseline | 4.67 m | -3.57 deg | 6.33 m | -3.80 deg |
| joint-a0.01-tau0.75 | 10.66 m | -0.49 deg | 10.33 m | -0.40 deg |
| joint-a0.02-tau1.10 | 13.17 m | 0.26 deg | 13.98 m | 0.34 deg |
| joint-a0.01-tau1.10 | 13.88 m | -1.77 deg | 10.37 m | -1.84 deg |
| joint-a0.02-tau0.90 | 14.97 m | 0.60 deg | 14.91 m | 0.68 deg |

结果文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_joint_tau_scan_summary.csv
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_joint_tau_scan_summary.png
```

结论：

调整 `tauScale` 后可以明显改善角度误差，很多联合参考工况都能把角度误差压到 `3 deg` 以内；但是脱靶量会从 baseline 的 `4.67 m` 放大到 `10 m` 以上，无法同时满足位置和角度约束。

因此，简单的正弦 `y_ref(tau)` + 斜率导出的 `theta_ref(tau)` 不是最终可用方案。它说明“提前转弯”方向是对的，但手工参考轨迹与当前 QP 终端命中目标存在冲突。后续更合理的方向是：

1. 用可行轨迹优化生成参考，而不是手工正弦参考；
2. 或者采用前段轨迹塑形、末段终端精修的分段制导；
3. 或者保留 baseline 作为位置命中主方法，再增加末端几秒角度修正。

## 14. 加速度振动硬约束

用户明确要求：添加硬约束抑制加速度振动。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_angle_only_no_time_check.m
```

新增脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_angle_only_accel_vibration_constraint_scan.m
```

新增 QP 硬约束：

```text
|A_cmd(k)-A_cmd(k-1)| <= Delta A_max
```

其中 `A_cmd = amax * u`，`u` 是 QP 决策得到的归一化加速度指令。代码中使用：

```matlab
deltaAccelMaxOverride = 5;
```

表示每个控制周期内加速度指令变化不超过 `5 m/s^2`。该约束会被换算为：

```text
|u_k-u_{k-1}| <= Delta A_max / amax
```

并加入 QP 线性不等式约束。

扫描参数：

```text
Delta A_max = inf, 15, 10, 8, 5, 3  m/s^2
```

关键结果：

| Delta A max | nominal miss | nominal angle | stress miss | stress angle | 是否推荐 |
|---:|---:|---:|---:|---:|---|
| inf | 13.49 m | -0.08 deg | 3.21 m | -0.08 deg | 否 |
| 15 | 10.75 m | -0.09 deg | 10.71 m | -0.06 deg | 否 |
| 10 | 8.21 m | -0.07 deg | 8.09 m | -0.07 deg | 否 |
| 8 | 14.90 m | -0.06 deg | 13.84 m | -0.05 deg | 否 |
| 5 | 3.05 m | 0.11 deg | 4.15 m | 0.12 deg | 推荐 |
| 3 | 17.71 m | 0.96 deg | 19.82 m | 1.11 deg | 过紧 |

结论：

`Delta A_max = 5 m/s^2` 是当前最合适的加速度振动硬约束。它同时满足：

- nominal 命中；
- stress 命中；
- 角度误差小；
- QP failures 为 0；
- 加速度曲线明显比无约束更平滑。

当前不考虑打击时间约束的主入口已改为：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_angle_only_no_time_check.m
```

默认使用：

```matlab
deltaAccelMaxOverride = 5;
resultSuffixOverride = 'angle_no_time_dA_5_main';
```

结果文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/angle_only_accel_vibration_constraint_scan_summary.csv
KDPC_RSRFG-main/StationaryTargetKoopman/results/angle_only_accel_vibration_constraint_scan_summary.png
KDPC_RSRFG-main/StationaryTargetKoopman/results/angle_only_accel_vibration_comparison.png
```

## 15. 60 deg 工况下的加速度振动硬约束

用户指出：需要在 `60 deg` 大打击角工况中添加加速度振动硬约束，而不是只在 `30 deg` 工况中验证。

新增脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_constraint_scan.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_h50_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_balanced_accel_h50_check.m
```

### 15.1 10 s 预测窗口尝试

用户要求将预测时域改成 `10 s`：

```matlab
horizonOverride = 100;
p.Ts = 0.1;
```

即：

```text
100 * 0.1 s = 10 s
```

同时为保证训练数据长度，`nStepsOverride` 从 `180` 提高到 `260`。

但是 10 s 预测窗口下效果变差：

| case | horizon | Delta A max | Rd | miss | angle error | QP failures |
|---|---:|---:|---:|---:|---:|---:|
| large60 zero-tau | 10 s | 15 m/s^2 | 1.5 | 192.86 m | -6.58 deg | 54 |
| large60 agile | 10 s | 30 m/s^2 | 0.2 | 154.88 m | -6.64 deg | 77 |

结论：

当前 Koopman-QP 结构下，`10 s` 预测窗口并没有让 60 deg 工况变好，反而导致终端约束更难满足、QP failures 增多、轨迹变得不够有效。

### 15.2 5 s 预测窗口对照

为了判断问题是否来自硬约束本身，做了 `5 s` 预测窗口对照：

| case | horizon | Delta A max | Rd | miss | angle error | QP failures |
|---|---:|---:|---:|---:|---:|---:|
| large60 agile h50 | 5 s | 30 m/s^2 | 0.2 | 11.88 m | -0.28 deg | 121 |
| large60 balanced h50 | 5 s | 30 m/s^2 | 0.5 | 8.19 m | -1.38 deg | 163 |

该结果说明：

1. `60 deg` 工况下，过强的加速度振动约束确实会抑制转弯；
2. 但主要问题不只是 `Delta A max`，而是大角度机动与当前终端 QP 结构之间存在冲突；
3. `10 s` 窗口下多步预测误差和终端约束冲突更明显；
4. `5 s` 窗口能明显弯起来，但位置命中仍未达到 `5 m`；
5. 当前最接近的 60 deg 加速度约束结果是 `large60 balanced h50`：

```text
nominal miss = 8.19 m
nominal angle error = -1.38 deg
stress miss = 9.25 m
stress angle error = -1.73 deg
```

结论：

对于 `60 deg`，不能直接套用 `30 deg` 的平滑约束。更合适的方向是：

1. 前段使用较大的 `Delta A max`，允许形成曲率；
2. 末段再收紧加速度变化率，抑制振动；
3. 或采用分段制导/末端精修，而不是单一 QP 从头到尾同时满足大角度、命中和平滑。

## 16. 命中判据修改

用户要求：当前“打没打到目标”的判断仅使用脱靶量，不再把打击角误差或打击时间误差纳入 `impactSatisfied`。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

原判据：

- angle-only 工况：`miss <= 5 m` 且 `|angle error| <= 3 deg`；
- time-constrained 工况：`time error`、`miss`、`angle error` 同时满足。

现判据：

```matlab
out.impactSatisfied = out.impactRange_m <= p.captureRadius;
```

其中：

```matlab
p.captureRadius = 5;
```

因此当前 `impactOK=1` 仅表示：

```text
脱靶量 <= 5 m
```

打击角误差和打击时间误差仍然会正常计算、保存和输出，但只作为性能指标，不再参与“是否命中目标”的布尔判断。

## 17. 60 deg 工况仿真终止时间缩短

用户指出：当前 60 deg 工况运行时间较长，可以减小闭环仿真最大终止时间。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_constraint_scan.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_h50_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_balanced_accel_h50_check.m
```

修改内容：

```matlab
angleOnlyMaxTimeOverride = 75;
```

改为：

```matlab
angleOnlyMaxTimeOverride = 50;
```

说明：

- 该参数只控制闭环仿真的最长运行时间；
- 不改变 MPC 预测时域；
- 不改变训练数据长度；
- 如果导弹在 50 s 内进入 `5 m` 捕获半径，仿真仍会提前终止；
- 之前 60 deg 结果的最近点大多出现在约 `39-44 s`，因此 50 s 对当前验证已经基本足够。

## 18. 初始位置可配置化

用户提出：希望可以方便调整初始位置。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

新增两个入口：

```matlab
initialPositionMetersOverride = [10000; 0];
```

以及：

```matlab
initialPositionOverride = [1.0; 0.0];
```

推荐使用米制入口 `initialPositionMetersOverride`。主程序会自动换算为归一化状态：

```matlab
initial = [initialPosition_m / p.Rscale; gamma0; u_a0];
```

当前：

```matlab
p.Rscale = 10000;
```

因此：

```matlab
initialPositionMetersOverride = [10000; 2000];
```

等价于：

```matlab
initialPositionOverride = [1.0; 0.2];
```

同时为当前 60 deg 主线脚本显式加入：

```matlab
initialPositionMetersOverride = [10000; 0];
```

涉及文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_accel_vibration_constraint_scan.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_agile_accel_h50_check.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_balanced_accel_h50_check.m
```

另修正：

```text
run_large_angle_60_accel_vibration_check.m
```

中初始航向角恢复为：

```matlab
initialGammaDegOverride = -30;
```

以符合此前“初始航向角一直保持 -30 deg”的要求。

## 19. Koopman-QP 中加入 Move Blocking

用户询问 Move Blocking 后，进一步要求修改代码实现该方法。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

新增参数：

```matlab
moveBlockSizeOverride = 5;
```

含义：

```text
每 5 个预测步共用一个控制变量。
```

例如：

```matlab
horizonOverride = 100;
moveBlockSizeOverride = 5;
```

原来 QP 需要优化：

```text
100 个控制变量
```

现在只需要优化：

```text
ceil(100 / 5) = 20 个控制变量
```

内部实现方式：

```text
U = Umap * v
```

其中：

- `v` 是块控制变量；
- `U` 是展开后的完整控制序列；
- 控制幅值惩罚、控制变化惩罚、加速度变化硬约束都作用在展开后的 `U` 上。

新增验证脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_check.m
```

该脚本设置：

```matlab
impactGammaDegOverride = 60;
initialGammaDegOverride = -30;
initialPositionMetersOverride = [10000; 0];
horizonOverride = 100;
moveBlockSizeOverride = 5;
deltaAccelMaxOverride = 30;
rdOverride = 0.5;
angleOnlyMaxTimeOverride = 50;
```

运行结果：

| case | horizon | block size | miss | angle error | stress miss | stress angle | QP failures |
|---|---:|---:|---:|---:|---:|---:|---:|
| large60 move-blocking | 10 s | 5 | 9.08 m | -9.45 deg | 11.47 m | -9.51 deg | 146 |

对比此前 10 s 无 move blocking 的 agile 版本：

```text
miss = 154.88 m
angle error = -6.64 deg
```

结论：

Move Blocking 显著改善了 10 s 预测窗口下的轨迹可行性，使脱靶量从百米级降到约 `9 m`，说明它确实缓解了长时域 QP 控制序列过自由、轨迹不稳定的问题。

但该版本仍未进入 `5 m` 命中圈，且 QP failures 仍较多。下一步更合理的组合是：

1. Move Blocking；
2. 位置-only 终端约束；
3. 弱化或取消终端角度约束；
4. 分段加速度变化约束。

## 20. 60 deg 快速调参脚本

用户同意增加一个快速版本，用于缩短每次仿真等待时间。

新增脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_quick.m
```

当前快速版设置：

```matlab
horizonOverride = 50;              % 5 s prediction horizon
moveBlockSizeOverride = 5;         % 10 block controls
seqIterationsOverride = 2;
nStepsOverride = 180;
nTrainTrajOverride = 500;
nTestTrajOverride = 100;
angleOnlyMaxTimeOverride = 45;
```

第一次尝试 `seqIterationsOverride = 1`，运行较快，但 QP failures 过多，结果严重失真。因此改为 `seqIterationsOverride = 2`。

第二版快速脚本运行时间约 `40 s`，结果：

| case | miss | angle error | stress miss | stress angle | QP failures |
|---|---:|---:|---:|---:|---:|
| quick seq2 | 17.03 m | 0.57 deg | 88.95 m | 0.73 deg | 182 |

结论：

该快速脚本适合用来快速筛选参数趋势，但不适合作为最终论文结果。最终结果仍应使用更完整的训练数据和更稳定的设置复跑。

### 20.1 快速脚本固定图片输出

用户要求 `run_large_angle_60_move_blocking_quick` 输出图片。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_quick.m
```

新增固定输出图片：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_quick_output.png
```

脚本运行结束后会将主程序生成的：

```text
closed_loop_<resultSuffix>.png
```

复制为上述固定文件名，方便每次快速查看。

## 21. 去掉 QP 惩罚项的尝试

用户要求尝试去掉以下 QP 项：

1. 控制变化惩罚；
2. 插值/模型一致性惩罚；
3. 终端松弛变量惩罚。

新增脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_no_penalty_quick.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_keep_slack_quick.m
```

### 21.1 三项全部去掉

设置：

```matlab
rdOverride = 0;
q0ScaleOverride = 0;
xiPenaltyOverride = 0;
slackPenaltyOverride = zeros(1,4);
deltaAccelMaxOverride = 10;
```

结果：

| case | miss | angle error | stress miss | stress angle | QP failures |
|---|---:|---:|---:|---:|---:|
| no-penalty quick | 205.07 m | -13.56 deg | 246.55 m | -14.52 deg | 0 |

现象：

QP failures 为 0，但轨迹明显偏离目标，说明完全去掉这些惩罚后，优化器虽然容易找到数学可行解，但控制解不再具有好的制导意义。

输出图片：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_no_penalty_quick_output.png
```

### 21.2 只去掉控制变化惩罚和模型一致性惩罚，保留 slack 惩罚

设置：

```matlab
rdOverride = 0;
q0ScaleOverride = 0;
xiPenaltyOverride = 0;
slackPenaltyOverride = [1e8,5e8,1e6,1e6];
deltaAccelMaxOverride = 10;
```

结果：

| case | miss | angle error | stress miss | stress angle | QP failures |
|---|---:|---:|---:|---:|---:|
| keep-slack quick | 3854.16 m | -62.21 deg | 4133.40 m | -68.80 deg | 367 |

现象：

保留强 slack 惩罚但去掉控制变化和模型一致性项后，QP failures 大量增加，轨迹严重失效。

输出图片：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_keep_slack_quick_output.png
```

结论：

这三类惩罚项不能简单全部去掉。它们不仅是“美化控制”的项，也在维持 Koopman-QP 闭环稳定性和预测一致性。后续更合理的方向不是删除，而是：

1. 降低权重；
2. 分阶段设置权重；
3. 只弱化角度相关 slack；
4. 保留适度控制变化惩罚，避免控制序列无意义跳动。

### 21.3 将 no-penalty quick 改为 balanced penalty quick

用户希望按“位置优先、角度保留、控制代价也考虑”的思路修改：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_no_penalty_quick.m
```

虽然文件名仍保留 `no_penalty`，但内部已改为 balanced penalty 设置。

修改后的主要参数：

```matlab
qxOverride = [800,4000,100,80];
qterminalOverride = [500000,2000000,10000,10000];
slackPenaltyOverride = [5e8,1e9,1e5,1e5];

rbarScaleOverride = 0.01;
rdOverride = 0.1;
deltaAccelMaxOverride = 30;

q0ScaleOverride = 0.002;
xiPenaltyOverride = 1e3;
```

输出文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_balanced_penalty_quick_summary.csv
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_balanced_penalty_quick_output.png
```

运行结果：

| case | miss | angle error | stress miss | stress angle | QP failures |
|---|---:|---:|---:|---:|---:|
| balanced penalty quick | 11.46 m | -1.31 deg | 16.20 m | -0.86 deg | 233 |

结论：

该 balanced penalty 版本明显优于三项惩罚全去掉的版本：

```text
no-penalty miss = 205.07 m
balanced penalty miss = 11.46 m
```

说明“保留但降低权重”的方向是正确的。不过当前仍未进入 `5 m` 命中圈，且 QP failures 偏多。下一步应继续软化终端角度约束，或显式将 angle-only 终端约束改成位置优先结构。

## 22. 闭环图线截断到最近目标点

用户要求：修改绘图代码，让图线在最靠近目标点的地方停止。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

修改内容：

新增辅助函数：

```matlab
truncate_to_impact(out)
```

该函数根据：

```matlab
out.impactIndex
```

截断闭环输出，使主闭环图中的轨迹、距离、航向角和加速度曲线都只画到最近目标点为止。

主闭环图现在使用：

```matlab
nominalPlot = truncate_to_impact(nominal);
stressPlot = truncate_to_impact(stress);
```

然后绘制：

```text
closed_loop_<resultSuffix>.png
```

这样可以避免图中出现“已经过了最近点后还继续飞行”的尾段，更直观地展示实际打击过程。

验证脚本：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_large_angle_60_move_blocking_no_penalty_quick.m
```

重新生成输出图片：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/results/large_angle_60_move_blocking_balanced_penalty_quick_output.png
```

本次运行结果：

```text
nominal miss = 6.47 m
nominal angle error = 0.36 deg
stress miss = 12.97 m
stress angle error = 0.93 deg
```

注意：本次结果对应当前脚本中的 `deltaAccelMaxOverride = 11.2`。

## 23. FOV 约束兼容 sin/cos 角度状态

用户在 `run_large_angle_60_move_blocking_no_penalty_quick.m` 中加入 FOV 约束后出现求解困难。

当前脚本设置包括：

```matlab
impactGammaDegOverride = 45;
angleStateModeOverride = 'sincos';
disableFovOverride = false;
fovMaxDegOverride = 60;
fovMinRangeOverride = 500;
```

问题原因：

原 QP 中的 FOV 线性约束默认第三个状态就是 `theta`，约束近似为：

```text
theta - y / s
```

但在 `sincos` 模式下，第三、第四个状态为：

```text
sin(theta), cos(theta)
```

因此原约束实际上把 `sin(theta)` 当作 `theta` 使用，在大角度机动下会造成不准确甚至过硬的 FOV 约束。

修改文件：

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m
```

修改内容：

在 QP 的 FOV 约束中，当 `angleStateMode='sincos'` 时，使用：

```matlab
theta = atan2(sin(theta), cos(theta));
```

并在当前预测点附近线性化：

```matlab
dtheta/dsin = cos(theta)/(sin^2(theta)+cos^2(theta))
dtheta/dcos = -sin(theta)/(sin^2(theta)+cos^2(theta))
```

从而构造兼容 `sincos` 状态的 FOV 线性约束。

同时同步修正 NMPC 约束检查中的 FOV 计算。

验证：

运行：

```text
run_large_angle_60_move_blocking_no_penalty_quick.m
```

可以正常跑完。当前结果：

```text
nominal miss = 29.74 m
nominal angle error = -2.88 deg
stress miss = 35.30 m
stress angle error = -2.66 deg
```

结论：

修正后 FOV 约束可以求解，但当前 `FOV = 60 deg` 会明显收窄可行机动空间，因此尚未命中目标。后续可先用更宽的 FOV，例如 `80 deg`，验证轨迹可行性后再逐步收紧。

## 24. GPOPS-II offline optimal-control comparison plan and scripts

Date: 2026-07-02

User request:
Compare both Koopman-MPC guidance laws in the manuscript against guidance
solutions obtained from GPOPS.

Design decision:
Use GPOPS-II as an offline optimal-control benchmark, not as an online
receding-horizon guidance law.  The comparison is split into two matched
problems:

1. Basic guidance comparison:
   - Koopman side: `run_exact_bilinear_koopman_mpc_precision_guidance.m`.
   - GPOPS side: fixed-final-time minimum-control-energy interception.
   - Model: ideal three-state stationary-target guidance core
     `[r_x, r_y, gamma]`.
   - Terminal constraints: `r_x(tf)=0`, `r_y(tf)=0`.
   - Final time: fixed to the Koopman-MPC closest-approach/capture time.

2. Impact-angle guidance comparison:
   - Koopman side: current angle-only run
     `stationary_target_results_angle_no_time_dA_5_main.mat`.
   - GPOPS side: fixed-final-time minimum-control-energy interception with
     terminal impact-angle constraint.
   - Model: four-state plant `[r_x, r_y, gamma, u_a]` with first-order
     autopilot lag.
   - Terminal constraints: `r_x(tf)=0`, `r_y(tf)=0`,
     `gamma(tf)=gamma_f`.
   - Final time: fixed to the Koopman-MPC actual impact/capture time.

Added scripts:

```text
KDPC_RSRFG-main/StationaryTargetKoopman/run_gpops_basic_guidance_comparison.m
KDPC_RSRFG-main/StationaryTargetKoopman/run_gpops_impact_angle_guidance_comparison.m
```

Expected outputs when GPOPS-II is installed:

```text
results/gpops_basic_guidance_comparison_summary.csv
results/gpops_basic_guidance_comparison.png
results/gpops_basic_guidance_comparison.mat
results/gpops_impact_angle_guidance_comparison_summary.csv
results/gpops_impact_angle_guidance_comparison.png
results/gpops_impact_angle_guidance_comparison.mat
```

Current environment note:
No `gpops2` executable/function was found on the MATLAB path in the current
workspace.  Therefore the scripts include a path check and write a status file
instead of failing if GPOPS-II is not installed.

Follow-up correction:
The impact-angle comparison plot now converts the Koopman trajectory back to
meters using `problem.Rscale`, matching the existing plotting convention in
`run_stationary_target_demo.m`.

## 25. GPOPS-II path setup and NLP solver adjustment

Date: 2026-07-02

User placed the downloaded GPOPS-II folder in the workspace:

```text
C:\Users\qiuya\Documents\koopman_guidance\gpops2\gpops2
```

Installed/configured by running:

```matlab
cd('C:\Users\qiuya\Documents\koopman_guidance\gpops2\gpops2')
gpopsMatlabPathSetup
savepath
```

Verification in a fresh MATLAB process:

```text
which_gpops2 = C:\Users\qiuya\Documents\koopman_guidance\gpops2\gpops2\lib\gpopsCommon\gpops2.m
exist_gpops2 = 2
```

First solve test:
`run_gpops_basic_guidance_comparison.m` successfully entered GPOPS-II, but the
IPOPT MEX failed at initialization:

```text
ipopt.mexw64 invalid: DLL initialization routine failed
```

This indicates that the GPOPS path itself is installed, but the bundled 2013
IPOPT binary is not compatible with or not fully supported by the current
MATLAB R2025b/runtime environment.  SNOPT was found on the MATLAB path:

```text
snoptcmex.mexw64
```

Therefore the two GPOPS comparison scripts were changed from
`setup.nlp.solver='ipopt'` to `setup.nlp.solver='snopt'`.

Basic comparison verification:
`run_gpops_basic_guidance_comparison.m` completed successfully with SNOPT and
generated:

```text
results/gpops_basic_guidance_comparison_summary.csv
results/gpops_basic_guidance_comparison.png
results/gpops_basic_guidance_comparison.mat
```

Representative result:

```text
Koopman-MPC miss = 2.6658 m, energy = 1.7587e+03
GPOPS miss       = 0.0000 m, energy = 4.8124e+02
```

Impact-angle script compatibility fix:
The saved stationary-target parameter struct uses `p.Vnom` rather than `p.V`.
Updated `run_gpops_impact_angle_guidance_comparison.m` accordingly.

Impact-angle comparison verification:
`run_gpops_impact_angle_guidance_comparison.m` completed successfully with
SNOPT and generated:

```text
results/gpops_impact_angle_guidance_comparison_summary.csv
results/gpops_impact_angle_guidance_comparison.png
results/gpops_impact_angle_guidance_comparison.mat
```

Representative result:

```text
Koopman-MPC miss = 3.0476 m, angle error = 0.1088 deg, energy = 1.6676e+04
GPOPS miss       = 0.0000 m, angle error = 0.0000 deg, energy = 5.3126e+03
```

Current conclusion:
GPOPS-II is installed and callable from MATLAB after path setup.  The bundled
IPOPT binary is not usable in the current MATLAB R2025b environment, but SNOPT
works and both GPOPS comparison scripts now run to completion.
