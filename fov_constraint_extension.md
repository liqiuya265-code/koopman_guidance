# FOV-Constrained Koopman-MPC Guidance Extension

## Objective

This note extends the current stationary-target bilinear Koopman-MPC guidance framework by adding a seeker field-of-view (FOV) path constraint. The goal is to test whether the same Koopman-MPC structure can handle an additional engineering constraint while preserving the prescribed impact time and impact angle.

The baseline terminal task is unchanged:

- prescribed impact time: \(t_f=21.7\) s;
- prescribed impact angle: \(\gamma_f=-2^\circ\);
- capture radius: 10 m;
- acceleration limit: 100 m/s\(^2\);
- prediction horizon: \(N=35\), \(T_s=0.1\) s.

## FOV Constraint

The seeker look angle is defined as

\[
\sigma = \gamma-\lambda,
\]

where \(\gamma\) is the pursuer flight-path angle and \(\lambda=\mathrm{atan2}(r_y,r_x)\) is the line-of-sight angle. The FOV constraint is

\[
|\sigma| \leq \sigma_{\max}.
\]

In the time-to-go impact frame used by the current paper, the relative position can be represented approximately by the along-impact distance \(s\) and cross-track displacement \(y\). Since

\[
\lambda \approx \gamma_f + \arctan(y/s),
\]

and \(\theta=\gamma-\gamma_f\), the look angle satisfies

\[
\sigma \approx \theta-\arctan(y/s).
\]

For the first FOV-constrained implementation, this is linearized in the MPC prediction model as

\[
\sigma \approx \theta-\frac{y}{s_{\rm ref}},
\]

where \(s_{\rm ref}=V(t_f-t)\) is lower-bounded by a 300 m terminal cutoff to avoid singular behavior near intercept. This keeps the online controller in the same sequential-convex QP form as the original bilinear Koopman-MPC.

## Code Files

The FOV extension is implemented in:

- `KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_demo.m`
  - Adds optional FOV constraint support.
  - Records maximum valid look angle and FOV violation.

- `KDPC_RSRFG-main/StationaryTargetKoopman/run_stationary_target_fov_demo.m`
  - Runs the FOV application study.
  - Compares no-FOV, 12 deg, 20 deg, 30 deg, and 45 deg FOV limits.

The generated outputs are:

- `KDPC_RSRFG-main/StationaryTargetKoopman/results/fov_guidance_comparison.png`
- `KDPC_RSRFG-main/StationaryTargetKoopman/results/fov_guidance_summary.csv`
- `KDPC_RSRFG-main/StationaryTargetKoopman/results/fov_guidance_results.mat`

## Numerical Results

| Case | Satisfied | Miss distance (m) | Impact-angle error (deg) | Max look angle (deg) | FOV violation (deg) | QP failures |
|---|---:|---:|---:|---:|---:|---:|
| No FOV constraint | Yes | 5.978 | 0.647 | 45.809 | 0.000 | 0 |
| FOV <= 12 deg | No | 229.608 | -3.023 | 11.465 | 0.000 | 0 |
| FOV <= 20 deg | No | 151.003 | -0.006 | 19.689 | 0.000 | 0 |
| FOV <= 30 deg | No | 63.386 | -0.057 | 29.722 | 0.000 | 0 |
| FOV <= 45 deg | Yes | 4.632 | 0.644 | 44.351 | 0.000 | 0 |

## Interpretation

The FOV constraint is successfully enforced in all constrained cases: the maximum valid look angle remains below the prescribed FOV limit, and no QP failures occur. However, the constraint substantially reduces the reachable terminal set for the original fixed-time, fixed-angle task.

The 12 deg, 20 deg, and 30 deg FOV cases satisfy the path constraint but fail the 10 m terminal capture requirement. The miss distance decreases as the FOV limit is relaxed, from 229.608 m at 12 deg to 63.386 m at 30 deg. When the FOV limit is relaxed to 45 deg, the controller again satisfies the terminal requirements and reaches a 4.632 m miss distance.

This indicates that FOV-constrained guidance should not only add a path constraint to the existing controller. It should also include reachability-aware scheduling of the impact time and impact angle. For tight FOV limits, the original \(t_f=21.7\) s and \(\gamma_f=-2^\circ\) command is too restrictive under the current acceleration bound and horizon.

## Next Research Step

The most natural application-oriented extension is a reachability-aware FOV-constrained Koopman-MPC guidance law:

\[
(t_f,\gamma_f) \in \mathcal R_{\rm FOV}(\sigma_{\max}),
\]

where \(\mathcal R_{\rm FOV}\) is the feasible terminal-time and terminal-angle envelope under the FOV constraint. The controller would first select a feasible terminal schedule and then solve the constrained Koopman-MPC problem.

This would turn the current result from a constraint-feasibility test into a complete FOV-constrained guidance application.
