#pragma once

#include "truck_model/articulated_vehicle.hpp"

#include <cstddef>
#include <deque>
#include <string>

namespace truck_model {

struct ArticulationEstimatorConfig {
    double historyHorizon{0.4};
    double measurementVariance{3.046174197867086e-4};
    double processArticulationRateVariance{2.741556778080377e-3};
    double processTrailerYawBiasVariance{2.0e-4};
    double processLidarBiasVariance{1.0e-10};
    double truckYawRateVariance{2.741556778080377e-5};
    double speedVariance{0.04};
    double mahalanobisGate{9.0};
    int consecutiveRejectLimit{3};
    double lostTimeout{0.4};
    double initialArticulationVariance{1.2180736252517745e-3};
    double initialTrailerYawBiasVariance{1.2180736252517745e-3};
    double initialLidarBiasVariance{2.741556778080377e-5};

    [[nodiscard]] std::string validationError() const;
};

struct ArticulationInputs {
    double time{};
    double truckYawRate{};
    double speed{};
    double steering{};
};

struct ArticulationLidarMeasurement {
    double stamp{};
    double articulation{};
};

struct ArticulationEstimate {
    double time{};
    double articulation{};
    double articulationRate{};
    double trailerYawRate{};
    double kinematicTrailerYawRate{};
    double truckYawResidual{};
    double trailerYawBias{};
    double lidarBias{};
    Matrix<3, 3> covariance{};
    double covariancePhi{};
    double covarianceTrailerBias{};
    double covarianceLidarBias{};
    double innovation{};
    double innovationCovariance{};
    double mahalanobis{};
    double kalmanGainPhi{};
    double kalmanGainTrailerBias{};
    double kalmanGainLidarBias{};
    bool measurementAccepted{};
    bool measurementGated{};
    bool coasting{};
    double lastAcceptedStamp{};
    double alignedStamp{};
    int consecutiveRejects{};
    std::size_t historySize{};
};

[[nodiscard]] double kinematicTrailerYawRate(
    const Parameters& parameters,
    double speed,
    double truckYawRate,
    double articulation);

class ArticulationEstimator {
public:
    ArticulationEstimator() = default;
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
    [[nodiscard]] bool initialized() const noexcept;

private:
    struct HistoryFrame {
        double time{};
        Vector<3> state{};
        Matrix<3, 3> covariance{};
        ArticulationInputs inputs{};
    };

    void requireConfigured() const;
    void publishFromState(
        double time,
        const Vector<3>& state,
        const Matrix<3, 3>& covariance,
        const ArticulationInputs& inputs);
    void trimHistory();
    [[nodiscard]] std::size_t nearestFrame(double stamp) const;

    Parameters parameters_{};
    ArticulationEstimatorConfig config_{};
    bool configured_{};
    bool initialized_{};
    ArticulationEstimate estimate_{};
    std::deque<HistoryFrame> history_;
};

}  // namespace truck_model
