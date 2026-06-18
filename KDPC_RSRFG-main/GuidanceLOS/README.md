# LOS relative-motion KDPC demo

This demo uses the conventional planar line-of-sight relative-motion model
described in `koopman_edmd_optimal_control_derivation.md`:

\[
\ddot R-R\dot q^2=a_{Tr}-a_{Mr},\qquad
R\ddot q+2\dot R\dot q=a_{Tq}-a_M.
\]

The Markov state is

\[
x=[R,\dot R,q,\dot q,a_M]^T,
\]

and the commanded lateral acceleration passes through a first-order actuator.
The simulation stops at a 300 m capture/terminal radius because the polar
equations are singular at `R = 0`. A separate terminal guidance law would take
over inside this set in a complete implementation.

Run:

```matlab
cd('KDPC_RSRFG-main/GuidanceLOS')
run_los_guidance_demo
```

This validates the Koopman predictor and constrained KDPC numerically. The
paper's origin-stabilization and invariant-terminal-set theorem is not directly
applicable to finite-time interception because range is not an equilibrium
state; a capture-set or time-to-go error reformulation is required for a full
theoretical extension.
