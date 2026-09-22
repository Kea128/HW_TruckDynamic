#pragma once

#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/delayed_ekf.hpp"

#include <cstddef>
#include <memory>
#include <string>

namespace truck_model {

struct ArticulationInputs {
    double time{};
    double truckYawRate{};
    double speed{};
    double steering{};
};

struct ArticulationLidarMeasurement {
    // Scan instant, never the arrival instant.
    double stamp{};
    double articulation{};
};

enum class ArticulationProcessModel {
    // Two states [phi, b_r2] driven by the (K5) rolling kinematics. Needs only
    // wheelbases, so it is insensitive to load and tyre data, but it models the
    // trailer sideslip residual as a random walk.
    kinematic,
    // Three states [v_y1, r2, phi] built from the (K27) lateral dynamics with
    // the measured truck yaw rate as an input. Resolves the residual dynamics
    // the random walk cannot follow, at the cost of depending on trailer mass,
    // inertia and cornering stiffness.
    dynamic
};

// All process-noise fields are continuous-time power spectral densities, never
// per-sample variances. The discretization integrates them over the actual step
// (Van Loan), so halving the step does not change the modelled noise.
struct ArticulationNoiseDensities {
    // Unmodelled articulation-rate excitation. [rad^2/s]
    double articulationRate{2.741556778080377e-3};
    // Trailer yaw residual random walk. [rad^2/s^3]
    double trailerYawBias{2.0e-4};
    // Truck yaw-rate sensor noise entering phiDot. [rad^2/s]
    double truckYawRate{2.741556778080377e-5};
    // Speed sensor noise entering phiDot. [m^2/s]
    double speed{0.04};
    // Truck lateral-velocity excitation, dynamic model only. [m^2/s^3]
    // Sized from an unmodelled lateral acceleration of about 0.3 m/s^2 with a
    // 0.2 s correlation time: 0.3^2 * 2 * 0.2 = 0.036.
    double truckLateralVelocity{0.05};
    // Trailer yaw-rate excitation, dynamic model only. [rad^2/s^3]
    // Sized from an unmodelled trailer yaw acceleration of about 1 deg/s^2 with
    // a 0.2 s correlation time: 0.0175^2 * 2 * 0.2 = 1.2e-4.
    double trailerYawRate{1.5e-4};

    [[nodiscard]] std::string validationError() const;
};

struct ArticulationEstimatorConfig {
    ArticulationProcessModel processModel{ArticulationProcessModel::kinematic};

    // Must be at least the largest scan latency plus one control period, or
    // late packets fall outside the replay window and are discarded.
    double historyHorizon{0.55};
    double measurementVariance{3.046174197867086e-4};
    ArticulationNoiseDensities noiseDensity{};

    double mahalanobisGate{9.0};
    int consecutiveRejectLimit{3};
    // Information-age threshold for inflating the process noise.
    double lostTimeout{0.6};
    // Arrival-gap threshold for the link-health diagnostic only.
    double linkTimeout{0.5};

    double initialArticulationVariance{1.2180736252517745e-3};
    double initialTrailerYawBiasVariance{1.2180736252517745e-3};
    double initialTruckLateralVelocityVariance{0.25};
    double initialTrailerYawRateVariance{1.2180736252517745e-3};

    // Rebuild the speed-scheduled (K27) matrices once the speed moves this far.
    double modelRefreshSpeedStep{0.25};
    double minimumModelSpeed{0.5};

    [[nodiscard]] std::string validationError() const;
};

struct ArticulationEstimate {
    double time{};
    double articulation{};
    double articulationRate{};
    double trailerYawRate{};
    double kinematicTrailerYawRate{};
    double truckYawResidual{};
    double trailerYawBias{};
    double truckLateralVelocity{};

    double covariancePhi{};
    double covarianceTrailerBias{};

    double innovation{};
    double innovationCovariance{};
    double mahalanobis{};
    double kalmanGainPhi{};
    double kalmanGainTrailerBias{};

    MeasurementOutcome outcome{MeasurementOutcome::notInitialized};
    bool measurementAccepted{};
    bool measurementGated{};
    // Running open loop: the newest fused scan is older than lostTimeout.
    bool coasting{};
    // Link health: nothing has been fused for longer than linkTimeout of wall
    // clock. Distinct from coasting, which measures information age.
    bool linkStalled{};
    double lastAcceptedStamp{};
    double alignedStamp{};
    // Age of the newest fused scan, in seconds.
    double informationAge{};
    // Wall-clock time since the last accepted update, in seconds.
    double arrivalGap{};
    int consecutiveRejects{};
    std::size_t historySize{};
    std::size_t repropagatedFrames{};
};

// (K5) trailer yaw rate under pure rolling. Shared by the kinematic process
// model and by diagnostics.
[[nodiscard]] double kinematicTrailerYawRate(
    const Parameters& parameters,
    double speed,
    double truckYawRate,
    double articulation);

// Two-state (K5) process model: x = [phi, b_r2].
class KinematicArticulationModel {
public:
    static constexpr std::size_t kStateSize = 2;
    using Inputs = ArticulationInputs;

    KinematicArticulationModel() = default;
    KinematicArticulationModel(
        Parameters parameters,
        ArticulationEstimatorConfig config);

    void propagate(
        Vector<kStateSize>& state,
        Matrix<kStateSize, kStateSize>& covariance,
        const Inputs& inputs,
        double dt,
        double processNoiseScale) const;
    [[nodiscard]] double predictMeasurement(
        const Vector<kStateSize>& state) const;
    [[nodiscard]] Vector<kStateSize> measurementJacobian(
        const Vector<kStateSize>& state) const;
    [[nodiscard]] double measurementVariance() const;
    [[nodiscard]] double residual(double measurement, double predicted) const;
    void normalize(Vector<kStateSize>& state) const;

    // Continuous linearization at the given operating point. Exposed so tests
    // can check the observability rank directly.
    [[nodiscard]] Matrix<kStateSize, kStateSize> continuousJacobian(
        const Vector<kStateSize>& state,
        const Inputs& inputs) const;

    [[nodiscard]] const Parameters& parameters() const { return parameters_; }

private:
    Parameters parameters_{};
    ArticulationEstimatorConfig config_{};
};

// Three-state reduced (K27) process model: x = [v_y1, r2, phi], with the
// measured truck yaw rate and steering angle as inputs.
class DynamicArticulationModel {
public:
    static constexpr std::size_t kStateSize = 3;
    using Inputs = ArticulationInputs;

    DynamicArticulationModel() = default;
    DynamicArticulationModel(
        Parameters parameters,
        ArticulationEstimatorConfig config);

    void propagate(
        Vector<kStateSize>& state,
        Matrix<kStateSize, kStateSize>& covariance,
        const Inputs& inputs,
        double dt,
        double processNoiseScale) const;
    [[nodiscard]] double predictMeasurement(
        const Vector<kStateSize>& state) const;
    [[nodiscard]] Vector<kStateSize> measurementJacobian(
        const Vector<kStateSize>& state) const;
    [[nodiscard]] double measurementVariance() const;
    [[nodiscard]] double residual(double measurement, double predicted) const;
    void normalize(Vector<kStateSize>& state) const;

    [[nodiscard]] Matrix<kStateSize, kStateSize> continuousJacobian(
        double speed) const;

    [[nodiscard]] const Parameters& parameters() const { return parameters_; }

private:
    void refresh(double speed) const;

    Parameters parameters_{};
    ArticulationEstimatorConfig config_{};
    // Speed-scheduled (K27) coefficients, rebuilt when the speed moves past
    // modelRefreshSpeedStep.
    mutable double scheduledSpeed_{-1.0};
    mutable Matrix<kStateSize, kStateSize> continuous_{};
    mutable Vector<kStateSize> inputYawRate_{};
    mutable Vector<kStateSize> inputSteering_{};
};

// Facade over the delayed-measurement shell. Selects the process model from the
// configuration and publishes a model-independent estimate.
class ArticulationEstimator {
public:
    ArticulationEstimator();
    ~ArticulationEstimator();
    ArticulationEstimator(ArticulationEstimator&&) noexcept;
    ArticulationEstimator& operator=(ArticulationEstimator&&) noexcept;
    ArticulationEstimator(const ArticulationEstimator&) = delete;
    ArticulationEstimator& operator=(const ArticulationEstimator&) = delete;

    explicit ArticulationEstimator(
        Parameters parameters,
        ArticulationEstimatorConfig config = {});

    void configure(
        Parameters parameters,
        ArticulationEstimatorConfig config = {});
    void reset(double time, double articulation);

    ArticulationEstimate predict(const ArticulationInputs& inputs);
    ArticulationEstimate updateLidar(
        const ArticulationLidarMeasurement& measurement);

    [[nodiscard]] const ArticulationEstimate& estimate() const noexcept;
    [[nodiscard]] const ArticulationEstimatorConfig& config() const noexcept;
    [[nodiscard]] const Parameters& parameters() const noexcept;
    [[nodiscard]] bool initialized() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace truck_model
