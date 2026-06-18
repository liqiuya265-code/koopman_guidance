# Guidance KDPC demo

This folder adapts the core ideas in the KDPC_RSRFG repository to a
two-dimensional impact-angle guidance model. It uses only built-in MATLAB
toolboxes (`quadprog` and `dlqr`) and does not require YALMIP, MOSEK, or MPT.

The nonlinear plant is

\[
\dot y_r = V\sin(\bar\gamma), \qquad
\dot{\bar\gamma}=a_M/V,
\]

with scaled lateral displacement, acceleration saturation, and a bounded
flight-path angle. The implementation preserves three features of the paper:

1. a directly identified multi-step linear-in-control Koopman predictor;
2. interpolation between the measured lifted state and the previous predicted
   lifted state;
3. a prediction-error-dependent regularization term.

Run from MATLAB:

```matlab
cd('KDPC_RSRFG-main/Guidance')
run_guidance_demo
```

Outputs are written to `results/`:

- `guidance_results.mat`
- `metrics.txt`
- `closed_loop.png`
- `prediction_validation.png`

This is a simple numerical validation, not a reproduction of the paper's full
terminal invariant-set construction or its recursive-feasibility theorem.

