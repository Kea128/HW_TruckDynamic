# TruckModel MPC Studio 2.9.0

First release carrying the articulation fusion filter. The lidar is the only
sensor that sees the articulation angle \(\phi\); it runs at about 10 Hz and
arrives 100–400 ms late, while the MPC needs the current \(\phi\) and
\(\dot\phi\) every 50 ms. This release closes that gap.

## Fusion filter

- Added a delayed EKF that fuses each scan at its own scan timestamp rather
  than at its arrival time, then re-propagates the cached inputs forward to
  the present. Under a fixed 200 ms latency the current-time estimate lands
  within 0.015 rad, well ahead of the raw delayed reading.
- Added two interchangeable process models behind one facade. The kinematic
  (K5) model carries \([\phi, b_{r2}]\) and needs only the trailer wheelbase,
  so it is immune to load changes; the reduced dynamic (K27r) model carries
  \([v_{y1}, r_2, \phi]\), takes the measured truck yaw rate as an input
  instead of a state, and tracks \(\dot\phi\) roughly 2.5 times better when
  the load parameters are trustworthy. The kinematic model is the default.
- Discretized the process noise with Van Loan rather than a diagonal
  approximation. The cross term this produces is what lets a lidar update of
  \(\phi\) also correct \(b_{r2}\), which is the only path by which
  \(\dot\phi\) ever improves.
- Declared all process-noise parameters as continuous power spectral
  densities, so halving the step size no longer changes the modelled noise.
- Rejected out-of-order, duplicate, stale and ahead-of-inputs scans with a
  distinct outcome code each, so a perception pipeline that violates the
  monotonic-timestamp assumption shows up as a counter instead of silently
  corrupting the state.
- Split coasting from link health: the process-noise inflation follows the age
  of the newest fused information, while the stalled-link flag follows wall
  clock since the last successful update. A long but healthy latency no longer
  reads as a dead link.
- Rejected at configuration time any history horizon shorter than the
  worst-case latency, which otherwise discards late scans without warning.

## Studio

- Replaced the single fusion toggle with three independent switches:
  run the filter, let the controller consume its output, and run a second
  filter for comparison. Each of the two filters picks its own process model.
- Selecting the same process model for both filters now has to produce
  identical output, since they receive the same inputs and the same scan
  objects. That equality is enforced by test and serves as a wiring check.
- Made the controller's articulation source explicit in the status bar, since
  whether the loop is closed on the estimate changes how every plot reads.
- Removed the truth-initialisation toggle; both filters now start from the
  same seed so later divergence can only come from the process model.

## Documentation

- Rewrote `docs/3_articulation_fusion_filter.md` as a side-by-side treatment of
  the two process models, with a symbol table ahead of every derivation, each
  formula mapped to its function in the source, and a step-by-step port
  checklist that carries its own acceptance condition per step.
- Recorded why \(\dot\phi\) is not an observation in either model: no sensor
  measures it, so it stays a model output, and writing it into \(H\) would
  claim information the vehicle does not have.
- Recorded why the lidar mounting bias is not an online state: adding it drops
  the observability matrix to rank 2, and the unobservable direction is given
  explicitly. Calibrate it offline and subtract it from the measurement.
- Documented that the articulation tracking experiment is not a fusion
  acceptance test. Closing the loop on an estimate that reads low inflates the
  true angle by roughly the inverse of the estimator's amplitude ratio, so it
  measures estimator bias multiplied by controller gain rather than accuracy.

## Notes

- Built with MinGW-w64 GCC 13.2 in Release mode. The binary is larger than the
  2.8.x MSVC builds because the GCC runtime is linked statically; no runtime
  redistributable is required.
- The simulation plant is the K27 model, which is also the basis of the K27r
  process model. The dynamic model's advantage in these figures is therefore
  partly inverse crime and has to be re-established on a real vehicle.
