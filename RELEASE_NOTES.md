# TruckModel MPC Studio 2.8.0

- Session logs: each run can export vehicle/MPC/EKF settings, per-step
  plant-control-lidar-filter timeseries, and the MPC horizon to
  `runs/<timestamp>/`. Finished or faulted demos auto-export.
- Delayed EKF diagnostics (innovation, Mahalanobis, kinematic r2, P, K)
  are stored in telemetry for offline amplitude analysis.

- Added a Delayed EKF for lidar articulation: K5 prediction from \(r_1,U\),
  400 ms history replay, and Mahalanobis gating of 10 Hz delayed scans.
- Studio can inject 100–300 ms lidar delay and feed the current
  \(\hat\phi,\hat{\dot\phi}\) to MPC; telemetry overlays plant, radar, and
  estimate. See `docs/3_articulation_fusion_filter.md`.

- Added a time-varying articulation reference \(\phi_{\mathrm{ref}}(t)\) with
  sine, periodic-step, and hand-drawn sources.
- Extended linear MPC dynamic programming to track a state-reference preview
  \((x-r)^\top Q(x-r)\) without changing the K27 plant or K35–K41 model.
- Added an articulation-tracking experiment that zeros path-error weights and
  feeds the K27 physical state to MPC so steering can follow
  \(\phi_{\mathrm{ref}}\) without fighting \(e_y,e_\psi\).
- Overlayed measured and reference articulation in telemetry, and reported
  the live tracking error in the toolbar.
- Retained path-tracking regulation to zero when the reference mode is off.
