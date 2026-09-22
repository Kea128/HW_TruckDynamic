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

    // Switch a: run the fusion filter. The process model it uses is
    // articulationEstimator.processModel.
    bool lidarFusionEnabled{false};

    // Switch b: feed the controller the estimate instead of the plant
    // articulation. Off by default so enabling fusion first shows filter
    // quality on its own; the filter keeps running either way, so the estimate
    // is still plotted and logged. Turning this on closes the loop, which is
    // what a real vehicle does but which also lets an estimator bias steer the
    // plant away from the reference.
    bool mpcUsesFusedArticulation{false};

    double lidarPeriod{0.1};
    double lidarDelayMin{0.1};
    double lidarDelayMax{0.3};
    double lidarNoiseStd{0.008726646167865535};
    unsigned lidarRandomSeed{1};
    bool ekfMeasurementFollowsLidar{true};

    // Truck yaw rate and speed reach the estimator through sensors. Zero by
    // default so the demo behaves like an ideal rig; raise them to see how the
    // filter copes with a realistic front end.
    double inputYawRateNoiseStd{0.0};
    double inputSpeedNoiseStd{0.0};
    unsigned inputRandomSeed{7};

    // Switch c: a second filter fed exactly the same scans and inputs as the
    // primary one, so the two process models can be compared on identical
    // data. It never reaches the controller. Requires lidarFusionEnabled,
    // since without it there are no scans to compare on.
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
