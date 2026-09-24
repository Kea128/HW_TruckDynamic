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
    // An unmodelled lateral acceleration of about 0.3 m/s^2 with a 0.2 s
    // correlation time gives 2 * 0.3^2 * 0.2 = 0.036; the default carries
    // about 40 % margin on top of that.
    double truckLateralVelocity{0.05};
    // Trailer yaw-rate excitation, dynamic model only. [rad^2/s^3]
    // An unmodelled trailer yaw acceleration of about 1 deg/s^2 with a 0.2 s
    // correlation time gives 2 * 0.0175^2 * 0.2 = 1.2e-4; the default carries
    // about 25 % margin on top of that.
    double trailerYawRate{1.5e-4};

    [[nodiscard]] std::string validationError() const;
};

struct ArticulationEstimatorConfig {
    ArticulationProcessModel processModel{ArticulationProcessModel::kinematic};

    // At least the largest scan age the filter can see, plus the worst service
    // wait, plus the largest gap trim() may drop. On a fixed control grid that
    // is latency + 2 periods; see docs/3 section 9.6. validationError() only
    // checks that this is positive, because the library does not know your
    // latency bound: the integration layer has to enforce the relation.
    double historyHorizon{0.55};
    double measurementVariance{3.046174197867086e-4};
    ArticulationNoiseDensities noiseDensity{};

    double mahalanobisGate{9.0};
    int consecutiveRejectLimit{3};
    // Information-age threshold for inflating the process noise.
    double lostTimeout{0.6};
    // Multiplies the articulation-rate density while the filter runs open
    // loop. 1 disables the inflation but not the coasting flag.
    double coastingProcessNoiseScale{4.0};
    // Threshold on arrivalGap for the linkStalled diagnostic only.
    double linkTimeout{0.5};
    // Hard cap on stored frames, on top of the historyHorizon span.
    std::size_t maximumFrames{4096};
    // Legacy mode: fuse at the nearest stored frame instead of at the stamp.
    bool snapToNearestFrame{false};

    double initialArticulationVariance{1.2184696791468343e-3};
    double initialTrailerYawBiasVariance{1.2184696791468343e-3};
    double initialTruckLateralVelocityVariance{0.25};
    double initialTrailerYawRateVariance{1.2184696791468343e-3};

    // The (K27) matrices are built for a quantized speed; see
    // scheduledModelSpeed().
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
    // Running open loop: either the newest fused scan is older than
    // lostTimeout, or consecutiveRejects has reached consecutiveRejectLimit.
    bool coasting{};
    // arrivalGap exceeds linkTimeout. Gated and out-of-order scans do not reset
    // it, so this flags "nothing fused for a while", whatever the cause.
    bool linkStalled{};
    // False until the first scan is accepted after reset. Until then
    // informationAge and arrivalGap count from the reset instant.
    bool hasAcceptedMeasurement{};
    double lastAcceptedStamp{};
    double alignedStamp{};
    // Age of the newest fused scan, in seconds.
    double informationAge{};
    // Filter time since the last accepted update, in seconds. The filter clock
    // is the newest input time, not the packet arrival time.
    double arrivalGap{};
    // Rejections counted since the last accepted scan: outOfOrder,
    // staleBeyondWindow and gated. Other outcomes neither count nor reset it.
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

// Speed the dynamic model is built for:
// max(minimumModelSpeed, step * round(speed / step)). A pure function of the
// sample, so a replay rebuilds exactly the matrices of the forward pass.
[[nodiscard]] double scheduledModelSpeed(
    double speed,
    const ArticulationEstimatorConfig& config);

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
    // Coefficients cached for the last scheduled speed.
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
