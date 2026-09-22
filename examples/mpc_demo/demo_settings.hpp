#pragma once

#include "articulation_reference.hpp"
#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/lateral_mpc.hpp"

#include <string>

namespace truck_demo {

struct DemoSettings {
    truck_model::Parameters vehicle;
    truck_model::MpcConfig mpc;
    ArticulationReferenceConfig articulationReference{};
    truck_model::ArticulationEstimatorConfig articulationEstimator{};
    bool articulationTrackingExperiment{false};
    bool lidarFusionEnabled{false};
    double lidarPeriod{0.1};
    double lidarDelayMin{0.1};
    double lidarDelayMax{0.3};
    double lidarNoiseStd{0.008726646167865535};
    unsigned lidarRandomSeed{1};
    bool ekfMeasurementFollowsLidar{true};

    // Constant lidar mounting error injected into the simulated scan. The
    // estimator only removes it through lidarBiasCalibration, so a mismatch
    // here is the calibration-drift experiment.
    double lidarInstallationBias{0.0};

    // Truck yaw rate and speed reach the estimator through sensors. Injecting
    // noise and bias keeps the demo from feeding the filter plant truth.
    double inputYawRateNoiseStd{0.0};
    double inputYawRateBias{0.0};
    double inputSpeedNoiseStd{0.0};
    unsigned inputRandomSeed{7};

    // Seeding the filter with the plant articulation is not reproducible on a
    // vehicle without an encoder; off by default.
    bool initializeEstimatorFromTruth{false};

    // Second estimator fed the same events for comparison. It never reaches the
    // controller.
    bool shadowEstimatorEnabled{false};
    truck_model::ArticulationProcessModel shadowProcessModel{
        truck_model::ArticulationProcessModel::dynamic};
    double initialLateralError{1.0};
    double initialLateralErrorRate{0.0};
    double initialHeadingError{0.08};
    double initialHeadingErrorRate{0.0};
    double initialArticulation{0.0};
    double initialArticulationRate{0.0};
    double initialSteering{0.0};
    double terminalWeightFactor{2.5};
    bool adaptiveSpeedEnabled{true};
    double minimumSpeed{3.0};
    double maximumLateralAcceleration{3.5};
    double maximumLateralJerk{8.0};
    double maximumAcceleration{1.2};
    double maximumDeceleration{2.5};
    double speedLookaheadDistance{45.0};

    [[nodiscard]] std::string validationError() const;
    void applyTerminalFactor();
    void applyEstimatorMeasurementFromLidar();
};

}  // namespace truck_demo
