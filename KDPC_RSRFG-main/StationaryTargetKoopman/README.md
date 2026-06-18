# Stationary-target Koopman guidance

This experiment uses the stationary-target, constant-speed planar guidance
model

\[
\dot R=-V\cos\sigma,\quad
\dot\lambda=-V\sin\sigma/R,\quad
\dot\gamma=A/V,\quad \sigma=\gamma-\lambda.
\]

It validates:

1. the exact continuous-time bilinear Koopman lifting obtained from Cartesian
   relative position and heading trigonometric observables;
2. a finite-dimensional linear-in-control direct multi-step EDMD predictor;
3. constrained KDPC capture for a non-maneuvering target;
4. a speed/input-effectiveness mismatch stress test.

Run:

```matlab
cd('KDPC_RSRFG-main/StationaryTargetKoopman')
run_stationary_target_demo
```

The outputs are written to `results/`.

