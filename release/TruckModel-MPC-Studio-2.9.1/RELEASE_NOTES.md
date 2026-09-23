# TruckModel MPC Studio 2.9.1

This correctness release supersedes 2.9.0 for new integrations. The 2.9.0
binary remains usable, but its bundled fusion document assumed a trustworthy
scan timestamp and contained several derivation and migration-contract errors.

## Delay contract

- Split the integration contract into two cases:
  - a trustworthy per-scan timestamp, where replay can place the update at the
    actual scan instant;
  - only an arrival time and an approximate 100–400 ms range, where a synthetic
    timestamp removes the mean delay but leaves frame-to-frame jitter.
- Quantified that residual timing error as
  \(e_\tau\simeq-\dot\phi\varepsilon\), documented the resulting Mahalanobis
  rejection risk, and specified
  \(R_{\rm eff}=R_{\rm sensor}+\dot\phi^2\sigma_\varepsilon^2\) as the required
  extension for the second case. This extension is documented but is not yet
  implemented in the reference code.
- Restored the end-to-end architecture, timing diagram and alternatives table
  that were present in the original handover note but missing from 2.9.0.
- Corrected the replay-window contract. On this fixed 20 Hz control grid the
  integration layer now requires
  `historyHorizon >= lidarDelayMax + 2 * sampleTime`: one period covers service
  quantisation and one preserves the predecessor frame needed for an
  asynchronous sub-frame update. The general rule is visible scan age plus
  maximum service wait plus maximum retained-frame gap.

## Formula corrections

- Corrected the hitch-offset sign convention:
  \(\ell_h=b_1-d_1>0\) places the hitch ahead of the truck rear axle.
- Corrected the physical meaning of the kinematic residual:
  \(b_{r2}\simeq U(\alpha_{2r}-\alpha_{1r})/L_2\); it contains both rear-axle
  slips, not only the trailer slip.
- Corrected the inertia sign when eliminating the hitch force, the fixed-
  curvature scaling \(b_{r2}\propto U^3\rho\), and the associated 8x speed-
  doubling result.
- Corrected the covariance explanation: state propagation already creates
  \(P_{10}\); the Van Loan cross term is important but is not its only source.
- Scoped the analytic process-noise covariance to its actual
  \(a=0,q_\phi=0\) special case, separated the Euler covariance transition
  \(F\) from the Van Loan transition \(\Phi_{\rm VL}\), and documented the
  mixed-discretisation error.
- Clarified that \(\dot\phi\) is a process-model output, not a measurement.
  An articulation encoder measures \(\phi\); a trailer IMU measuring \(r_2\)
  would provide direct rate support.

## Code fixes

- Fixed undefined behaviour in `testDynamicObservabilityRank`, which read a
  3x3 matrix as 4x4 after the lidar-bias state was removed.
- `updateLidar()` now reports corrupt measurements as `nonFinite` instead of
  throwing through the control loop, and clears per-measurement diagnostics on
  every update attempt.
- `predict()` now rejects non-finite steering as well as non-finite time, yaw
  rate and speed.
- Repropagation now applies the coasting noise scale according to the age since
  the corrected scan instead of forcing it to 1.0.
- Added regression tests for non-finite packets and inputs, replay spans that
  cross `lostTimeout`, the dynamic observability determinant, and the two-
  control-period history-window rule.

## Verification

- Both normal test suites pass.
- Both suites pass with `_GLIBCXX_ASSERTIONS`, which is what exposed the former
  out-of-bounds access.
- The full Studio GUI builds in Release mode.
- README examples compile and run.
- All 25 test names cited by the fusion document exist, and all internal
  chapter references resolve.

The archived `docs/3_articulation_fusion_filter_origin.md` remains in the
source repository for historical comparison. It is intentionally omitted from
the release package because it contains known sign and observability errors.
