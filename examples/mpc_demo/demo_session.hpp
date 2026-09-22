#pragma once

#include "articulation_reference.hpp"
#include "demo_settings.hpp"

#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/lateral_mpc.hpp"
#include "truck_model/linear_discretization.hpp"
#include "truck_model/reference_path.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace truck_demo {

struct VehicleSnapshot {
    double truckX{};
    double truckY{};
    double truckHeading{};
    double hitchX{};
    double hitchY{};
    double trailerX{};
    double trailerY{};
    double trailerHeading{};
};

struct Telemetry {
    double time{};
    double distance{};
    truck_model::Vector<6> state{};
    truck_model::Vector<6> controllerState{};
    truck_model::Vector<4> physicalState{};
    double plantVy1{};
    double plantR1{};
    double plantR2{};
    double plantVy2{};
    double plantPhi{};
    double plantPhiDot{};
    double steering{};
    double unconstrainedSteering{};
    double speed{};
    double plannedSpeed{};
    double cruiseSpeed{};
    double scheduledModelSpeed{};
    bool modelsRebuilt{};
    double referenceCurvature{};
    double referenceCurvatureRate{};
    double referenceArticulation{};
    double referenceArticulationRate{};
    double articulationTrackingError{};
    double plantArticulation{};
    double plantArticulationRate{};
    double estimatedArticulation{};
    double estimatedArticulationRate{};
    double shadowArticulation{};
    double shadowArticulationRate{};
    double measuredTruckYawRate{};
    double measuredSpeed{};
    double lidarArticulation{};
    double lidarStamp{};
    double lidarDelay{};
    std::size_t lidarQueueSize{};
    bool lidarDelivered{};
    bool lidarAccepted{};
    bool lidarGated{};
    // Per-step packet counters. A control step can service several scans, so a
    // single boolean cannot describe the step.
    std::size_t lidarDeliveredCount{};
    std::size_t lidarAcceptedCount{};
    std::size_t lidarGatedCount{};
    std::size_t lidarDroppedCount{};
    const char* lidarOutcome{"none"};
    bool estimatorCoasting{};
    truck_model::ArticulationEstimate estimator{};
    truck_model::ArticulationEstimate shadowEstimator{};
    std::vector<truck_model::Vector<6>> predictedStates{};
    bool warningActive{};
    std::string warning;
    VehicleSnapshot vehicle{};
};

enum class SimulationState {
    stopped,
    running,
    paused,
    finished,
    faulted
};

class DemoSession {
public:
    DemoSession();

    void configure(DemoSettings settings);
    void setArticulationReference(ArticulationReference reference);
    void beginArticulationTrackingExperiment();
    void endArticulationTrackingExperiment();
    void setPath(truck_model::ReferencePath path);
    void start();
    void pauseOrResume();
    void reset();
    void step();

    [[nodiscard]] const DemoSettings& settings() const noexcept;
    [[nodiscard]] const truck_model::ReferencePath& path() const noexcept;
    [[nodiscard]] const truck_model::Vector<6>& state() const noexcept;
    [[nodiscard]] const std::vector<Telemetry>& history() const noexcept;
    [[nodiscard]] std::size_t telemetryRevision() const noexcept;
    [[nodiscard]] const std::vector<truck_model::Vector<6>>&
    predictedStates() const noexcept;
    [[nodiscard]] SimulationState simulationState() const noexcept;
    [[nodiscard]] double time() const noexcept;
    [[nodiscard]] double distance() const noexcept;
    [[nodiscard]] double steering() const noexcept;
    [[nodiscard]] double currentSpeed() const noexcept;
    [[nodiscard]] const std::string& faultReason() const noexcept;
    [[nodiscard]] const std::string& warningReason() const noexcept;
    [[nodiscard]] const ArticulationReference&
    articulationReference() const noexcept;
    [[nodiscard]] ArticulationSample currentArticulationReference() const;
    [[nodiscard]] const truck_model::ArticulationEstimate&
    articulationEstimate() const noexcept;
    [[nodiscard]] const truck_model::ArticulationEstimate&
    shadowArticulationEstimate() const noexcept;
    [[nodiscard]] const truck_model::ErrorLinearModel&
    errorModel() const noexcept;
    [[nodiscard]] const truck_model::DiscreteDynamicModel&
    physicalPlant() const noexcept;
    [[nodiscard]] double scheduledModelSpeed() const noexcept;
    [[nodiscard]] double plannedSpeedNow() const;
    [[nodiscard]] const truck_model::ArticulationEstimatorConfig&
    estimatorConfig() const noexcept;
    [[nodiscard]] const truck_model::LateralMpc* controller() const noexcept;
    [[nodiscard]] std::string exportRunLog(const std::string& directory) const;
    [[nodiscard]] VehicleSnapshot currentVehicle() const;
    [[nodiscard]] std::vector<truck_model::Point2d>
    predictedWorldPositions() const;

    [[nodiscard]] static DemoSettings defaultSettings();
    [[nodiscard]] static truck_model::ReferencePath defaultPath();
    [[nodiscard]] static truck_model::ReferencePath highCurvaturePath();

private:
    [[nodiscard]] VehicleSnapshot vehicleSnapshot() const;
    void validatePathForSettings(
        const truck_model::ReferencePath& path,
        const DemoSettings& settings) const;
    [[nodiscard]] double pathSpeedLimit(
        double pathDistance,
        const DemoSettings& settings) const;
    [[nodiscard]] double plannedSpeed(double pathDistance) const;
    void updateAdaptiveSpeed();
    void rebuildModelsForSpeed(double speed);
    void updatePathProgress();
    void updateMeasuredErrorState();
    void resetArticulationEstimator();
    void captureDelayedLidar(double plantArticulation);
    void deliverDueLidar();
    [[nodiscard]] double nextLidarUniform();
    [[nodiscard]] double nextLidarNormal();
    [[nodiscard]] double nextInputNormal();
    [[nodiscard]] truck_model::ArticulationInputs sensedInputs();
    [[nodiscard]] bool updateRuntimeAlerts();
    void record();

    void applyIsolatedArticulationWeights(DemoSettings& settings) const;
    void rebuildArticulationReference();
    [[nodiscard]] truck_model::Vector<6> controllerState() const;

    DemoSettings settings_{};
    ArticulationReference articulationReference_{};
    bool savedWeights_{};
    truck_model::Vector<6> savedStateWeight_{};
    double savedTerminalWeightFactor_{2.5};
    double savedSteeringWeight_{30.0};
    double savedSteeringRateWeight_{10000.0};
    truck_model::ReferencePath path_{};
    std::unique_ptr<truck_model::LateralMpc> controller_;
    truck_model::ErrorLinearModel errorModel_{};
    truck_model::DiscreteDynamicModel physicalPlant_{};
    truck_model::Vector<4> physicalState_{};
    truck_model::Vector<6> state_{};
    std::vector<Telemetry> history_;
    std::size_t telemetryRevision_{};
    std::vector<truck_model::Vector<6>> predictedStates_;
    SimulationState simulationState_{SimulationState::stopped};
    double time_{};
    double distance_{};
    double steering_{};
    double currentSpeed_{};
    double scheduledModelSpeed_{};
    double truckX_{};
    double truckY_{};
    double truckHeading_{};
    std::string faultReason_;
    std::string warningReason_;
    truck_model::ArticulationEstimator articulationEstimator_;
    truck_model::ArticulationEstimator shadowEstimator_;
    truck_model::ArticulationEstimate articulationEstimate_{};
    truck_model::ArticulationEstimate shadowEstimate_{};
    struct PendingLidar {
        double deliverTime{};
        truck_model::ArticulationLidarMeasurement measurement{};
    };
    // Not a FIFO: packets are serviced in arrival-time order, so a short-delay
    // scan can overtake an earlier one and reach the filter out of stamp order.
    std::vector<PendingLidar> pendingLidar_;
    std::uint32_t lidarRng_{1};
    std::uint32_t inputRng_{7};
    std::uint64_t nextLidarId_{1};
    std::size_t lastLidarDeliveredCount_{};
    std::size_t lastLidarAcceptedCount_{};
    std::size_t lastLidarGatedCount_{};
    std::size_t lastLidarDroppedCount_{};
    const char* lastLidarOutcome_{"none"};
    double measuredYawRate_{};
    double measuredSpeed_{};
    double lastLidarScanTime_{-1.0};
    double lastLidarArticulation_{};
    double lastLidarStamp_{};
    double lastLidarDelay_{};
    bool lastLidarAccepted_{};
    bool lastLidarDelivered_{};
    bool lastLidarGated_{};
    double unconstrainedSteering_{};
    bool modelsRebuiltThisStep_{};
};

}  // namespace truck_demo
