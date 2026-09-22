#include "demo_session.hpp"

#include "demo_paths.hpp"
#include "session_log.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace truck_demo {
namespace {

constexpr double kPi = 3.14159265358979323846;

truck_model::Point2d errorOffsetPosition(
    const truck_model::ReferencePathPoint& reference,
    double lateralError) {
    return {
        reference.x - lateralError * std::sin(reference.heading),
        reference.y + lateralError * std::cos(reference.heading)};
}

double normalizedAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double speedLimitForReference(
    const truck_model::ReferencePathPoint& reference,
    const DemoSettings& settings) {
    constexpr double epsilon = 1.0e-9;
    const double curvature =
        std::max(std::abs(reference.curvature), epsilon);
    const double curvatureDerivative =
        std::max(std::abs(reference.curvatureDerivative), epsilon);
    const double wheelbase =
        settings.vehicle.a1 + settings.vehicle.b1;
    const double lateralAccelerationSpeed =
        std::sqrt(settings.maximumLateralAcceleration / curvature);
    const double lateralJerkSpeed =
        std::cbrt(settings.maximumLateralJerk / curvatureDerivative);
    const double steeringRateSpeed =
        0.8 * settings.mpc.maxSteeringRate *
        (1.0 + std::pow(wheelbase * curvature, 2.0)) /
        (wheelbase * curvatureDerivative);
    return std::min(
        {settings.vehicle.vx,
         lateralAccelerationSpeed,
         lateralJerkSpeed,
         steeringRateSpeed});
}

}  // namespace

const char* estimatorDisplayName(
    const truck_model::ArticulationEstimatorConfig& config) {
    const bool legacy = config.compatibility.nearestFrameAlignment ||
                        config.compatibility.diagonalEulerProcessNoise;
    if (legacy) {
        return "v1";
    }
    return config.processModel == truck_model::ArticulationProcessModel::dynamic
               ? "v2 K27r"
               : "v2 K5";
}

std::string DemoSettings::validationError() const {
    std::ostringstream errors;
    const auto vehicleError = vehicle.validationError();
    if (!vehicleError.empty()) {
        errors << vehicleError;
    }
    const auto mpcError = mpc.validationError();
    if (!mpcError.empty()) {
        errors << mpcError;
    }
    if (!std::isfinite(initialLateralError) ||
        std::abs(initialLateralError) > 20.0) {
        errors << "initialLateralError must be finite and within +/-20 m; ";
    }
    if (!std::isfinite(initialLateralErrorRate) ||
        std::abs(initialLateralErrorRate) > 10.0) {
        errors <<
            "initialLateralErrorRate must be finite and within +/-10 m/s; ";
    }
    if (!std::isfinite(initialHeadingError) ||
        std::abs(initialHeadingError) > 1.2) {
        errors << "initialHeadingError must be finite and within +/-1.2 rad; ";
    }
    if (!std::isfinite(initialHeadingErrorRate) ||
        std::abs(initialHeadingErrorRate) > 2.0) {
        errors <<
            "initialHeadingErrorRate must be finite and within +/-2 rad/s; ";
    }
    if (!std::isfinite(initialArticulation) ||
        std::abs(initialArticulation) > 0.7) {
        errors << "initialArticulation must be finite and within +/-0.7 rad; ";
    }
    if (!std::isfinite(initialArticulationRate) ||
        std::abs(initialArticulationRate) > 2.0) {
        errors <<
            "initialArticulationRate must be finite and within +/-2 rad/s; ";
    }
    if (!std::isfinite(initialSteering) ||
        std::abs(initialSteering) > mpc.maxSteering + 1.0e-12) {
        errors <<
            "initialSteering must be finite and within +/-maxSteering; ";
    }
    if (!(terminalWeightFactor > 0.0) ||
        !std::isfinite(terminalWeightFactor)) {
        errors << "terminalWeightFactor must be finite and > 0; ";
    }
    const auto requirePositive =
        [&errors](double value, const char* name) {
            if (!(value > 0.0) || !std::isfinite(value)) {
                errors << name << " must be finite and > 0; ";
            }
        };
    requirePositive(minimumSpeed, "minimumSpeed");
    requirePositive(
        maximumLateralAcceleration, "maximumLateralAcceleration");
    requirePositive(maximumLateralJerk, "maximumLateralJerk");
    requirePositive(maximumAcceleration, "maximumAcceleration");
    requirePositive(maximumDeceleration, "maximumDeceleration");
    requirePositive(speedLookaheadDistance, "speedLookaheadDistance");
    if (minimumSpeed > vehicle.vx) {
        errors << "minimumSpeed must not exceed cruise speed vx; ";
    }
    if (!(lidarPeriod > 0.0) || !std::isfinite(lidarPeriod) ||
        lidarPeriod > 1.0) {
        errors << "lidarPeriod must be finite and in (0, 1] s; ";
    }
    if (!std::isfinite(lidarDelayMin) || lidarDelayMin < 0.0) {
        errors << "lidarDelayMin must be finite and >= 0; ";
    }
    if (!std::isfinite(lidarDelayMax) || lidarDelayMax < lidarDelayMin) {
        errors << "lidarDelayMax must be finite and >= lidarDelayMin; ";
    }
    if (lidarDelayMax > 0.5) {
        errors << "lidarDelayMax must be <= 0.5 s; ";
    }
    if (!std::isfinite(lidarNoiseStd) || lidarNoiseStd < 0.0 ||
        lidarNoiseStd > 0.2) {
        errors << "lidarNoiseStd must be finite and within [0, 0.2] rad; ";
    }
    if (!std::isfinite(lidarInstallationBias) ||
        std::abs(lidarInstallationBias) > 0.2) {
        errors <<
            "lidarInstallationBias must be finite and within +/-0.2 rad; ";
    }
    if (!std::isfinite(inputYawRateNoiseStd) || inputYawRateNoiseStd < 0.0 ||
        inputYawRateNoiseStd > 0.5) {
        errors <<
            "inputYawRateNoiseStd must be finite and within [0, 0.5] rad/s; ";
    }
    if (!std::isfinite(inputYawRateBias) ||
        std::abs(inputYawRateBias) > 0.5) {
        errors << "inputYawRateBias must be finite and within +/-0.5 rad/s; ";
    }
    if (!std::isfinite(inputSpeedNoiseStd) || inputSpeedNoiseStd < 0.0 ||
        inputSpeedNoiseStd > 5.0) {
        errors << "inputSpeedNoiseStd must be finite and within [0, 5] m/s; ";
    }
    if (lidarDelayMax > articulationEstimator.historyHorizon) {
        errors << "ekf.historyHorizon must be >= lidarDelayMax so that late "
                  "scans stay inside the replay window; ";
    }
    if (shadowEstimatorEnabled) {
        const auto shadowError = shadowEstimator.validationError();
        if (!shadowError.empty()) {
            errors << "shadow " << shadowError;
        }
        // The shadow may deliberately run a short window: reproducing the v1
        // packet loss is the point of the comparison, so this is not an error.
    }
    if (mpc.horizon > 200) {
        errors << "horizon must be <= 200; ";
    }
    const auto estimatorError = articulationEstimator.validationError();
    if (!estimatorError.empty()) {
        errors << estimatorError;
    }
    const auto referenceError = articulationReference.validationError();
    if (!referenceError.empty()) {
        errors << referenceError;
    }
    return errors.str();
}

void DemoSettings::applyTerminalFactor() {
    for (std::size_t i = 0; i < mpc.stateWeight.size(); ++i) {
        mpc.terminalWeight[i] =
            terminalWeightFactor * mpc.stateWeight[i];
    }
}

void DemoSettings::applyEstimatorMeasurementFromLidar() {
    if (!ekfMeasurementFollowsLidar) {
        return;
    }
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kMinimumLidarStd = 0.25 * kPi / 180.0;
    const double lidarStd = std::max(lidarNoiseStd, kMinimumLidarStd);
    articulationEstimator.measurementVariance = lidarStd * lidarStd;
    shadowEstimator.measurementVariance = lidarStd * lidarStd;
}

DemoSession::DemoSession() {
    configure(defaultSettings());
    setPath(defaultPath());
}

void DemoSession::configure(DemoSettings settings) {
    if (settings.articulationTrackingExperiment) {
        if (!savedWeights_) {
            savedStateWeight_ = settings.mpc.stateWeight;
            savedTerminalWeightFactor_ = settings.terminalWeightFactor;
            savedSteeringWeight_ = settings.mpc.steeringWeight;
            savedSteeringRateWeight_ = settings.mpc.steeringRateWeight;
            savedWeights_ = true;
        }
        applyIsolatedArticulationWeights(settings);
    }
    settings.applyTerminalFactor();
    settings.applyEstimatorMeasurementFromLidar();
    const auto error = settings.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
    if (!path_.empty()) {
        validatePathForSettings(path_, settings);
    }
    const auto errorModel = truck_model::buildErrorModel(settings.vehicle);
    const auto dynamicModel =
        truck_model::buildDynamicModel(settings.vehicle);
    const auto physicalPlant = truck_model::discretizeZeroOrderHold(
        dynamicModel, settings.mpc.sampleTime);
    auto controller = std::make_unique<truck_model::LateralMpc>(
        errorModel, settings.mpc);
    settings_ = settings;
    errorModel_ = errorModel;
    physicalPlant_ = physicalPlant;
    controller_ = std::move(controller);
    currentSpeed_ = settings_.vehicle.vx;
    scheduledModelSpeed_ = currentSpeed_;
    rebuildArticulationReference();
    reset();
}

void DemoSession::setArticulationReference(ArticulationReference reference) {
    settings_.articulationReference = reference.config();
    articulationReference_ = std::move(reference);
    reset();
}

void DemoSession::beginArticulationTrackingExperiment() {
    auto settings = settings_;
    settings.articulationTrackingExperiment = true;
    if (settings.articulationReference.kind ==
        ArticulationReferenceKind::none) {
        settings.articulationReference.kind = ArticulationReferenceKind::sine;
    }
    settings.initialLateralError = 0.0;
    settings.initialLateralErrorRate = 0.0;
    settings.initialHeadingError = 0.0;
    settings.initialHeadingErrorRate = 0.0;
    settings.initialSteering = 0.0;
    const auto start = ArticulationReference::fromConfig(
                           settings.articulationReference)
                           .sample(0.0);
    if (settings.articulationReference.kind !=
        ArticulationReferenceKind::drawn) {
        settings.initialArticulation = start.value;
        settings.initialArticulationRate = start.rate;
    } else {
        const auto drawnStart = articulationReference_.sample(0.0);
        settings.initialArticulation = drawnStart.value;
        settings.initialArticulationRate = drawnStart.rate;
    }
    configure(settings);
}

void DemoSession::endArticulationTrackingExperiment() {
    auto settings = settings_;
    settings.articulationTrackingExperiment = false;
    settings.articulationReference.kind = ArticulationReferenceKind::none;
    if (savedWeights_) {
        settings.mpc.stateWeight = savedStateWeight_;
        settings.terminalWeightFactor = savedTerminalWeightFactor_;
        settings.mpc.steeringWeight = savedSteeringWeight_;
        settings.mpc.steeringRateWeight = savedSteeringRateWeight_;
        savedWeights_ = false;
    }
    configure(settings);
}

void DemoSession::setPath(truck_model::ReferencePath path) {
    validatePathForSettings(path, settings_);
    path_ = std::move(path);
    reset();
}

void DemoSession::start() {
    if (path_.empty()) {
        throw std::logic_error("draw or load a reference path first");
    }
    if (simulationState_ == SimulationState::finished ||
        simulationState_ == SimulationState::faulted ||
        simulationState_ == SimulationState::stopped) {
        reset();
    }
    simulationState_ = SimulationState::running;
}

void DemoSession::pauseOrResume() {
    if (simulationState_ == SimulationState::running) {
        simulationState_ = SimulationState::paused;
    } else if (simulationState_ == SimulationState::paused) {
        simulationState_ = SimulationState::running;
    } else {
        start();
    }
}

void DemoSession::reset() {
    state_ = {
        settings_.initialLateralError,
        settings_.initialLateralErrorRate,
        settings_.initialHeadingError,
        settings_.initialHeadingErrorRate,
        settings_.initialArticulation,
        settings_.initialArticulationRate};
    if (settings_.articulationTrackingExperiment) {
        const auto start = articulationReference_.sample(0.0);
        state_[0] = 0.0;
        state_[1] = 0.0;
        state_[2] = 0.0;
        state_[3] = 0.0;
        state_[4] = start.value;
        state_[5] = start.rate;
    }
    time_ = 0.0;
    distance_ = 0.0;
    steering_ = settings_.articulationTrackingExperiment
                    ? 0.0
                    : settings_.initialSteering;
    faultReason_.clear();
    warningReason_.clear();
    physicalState_ = {};
    truckX_ = 0.0;
    truckY_ = 0.0;
    truckHeading_ = 0.0;
    history_.clear();
    predictedStates_.clear();
    simulationState_ = SimulationState::stopped;
    currentSpeed_ = settings_.vehicle.vx;
    if (!path_.empty() && settings_.adaptiveSpeedEnabled) {
        currentSpeed_ = plannedSpeed(0.0);
    }
    if (std::abs(currentSpeed_ - scheduledModelSpeed_) > 1.0e-9) {
        rebuildModelsForSpeed(currentSpeed_);
    } else if (controller_) {
        controller_->reset(steering_);
    }
    if (!path_.empty()) {
        const auto reference = path_.sample(0.0);
        const auto position =
            errorOffsetPosition(reference, state_[0]);
        truckX_ = position.x;
        truckY_ = position.y;
        truckHeading_ = reference.heading + state_[2];
        physicalState_ =
            truck_model::multiply(errorModel_.stateToPhysical, state_);
        for (std::size_t i = 0; i < physicalState_.size(); ++i) {
            physicalState_[i] +=
                errorModel_.curvatureToPhysical[i] * reference.curvature;
        }
        resetArticulationEstimator();
        updateMeasuredErrorState();
        (void)updateRuntimeAlerts();
        record();
    } else {
        resetArticulationEstimator();
    }
}

void DemoSession::step() {
    if (simulationState_ != SimulationState::running) {
        return;
    }
    updateAdaptiveSpeed();
    const auto& config = settings_.mpc;
    std::vector<truck_model::CurvatureSample> preview;
    preview.reserve(config.horizon);
    for (std::size_t i = 0; i < config.horizon; ++i) {
        const double previewDistance =
            distance_ + static_cast<double>(i) *
                            currentSpeed_ * config.sampleTime;
        const auto reference = path_.sample(previewDistance);
        preview.push_back(
            {reference.curvature,
             currentSpeed_ * reference.curvatureDerivative});
    }

    const auto stateReference = articulationReference_.preview(
        time_, config.sampleTime, config.horizon);
    const auto command = controller_->update(
        controllerState(),
        settings_.articulationTrackingExperiment
            ? std::vector<truck_model::CurvatureSample>{}
            : preview,
        stateReference);
    steering_ = command.steering;
    unconstrainedSteering_ = command.unconstrainedSteering;
    predictedStates_ = command.predictedStates;
    auto nextPhysical =
        truck_model::multiply(physicalPlant_.a, physicalState_);
    for (std::size_t i = 0; i < nextPhysical.size(); ++i) {
        nextPhysical[i] += physicalPlant_.b[i] * steering_;
    }

    const double lateralVelocity =
        0.5 * (physicalState_[0] + nextPhysical[0]);
    const double yawRate =
        0.5 * (physicalState_[1] + nextPhysical[1]);
    const double middleHeading =
        truckHeading_ + 0.5 * config.sampleTime * yawRate;
    truckX_ += config.sampleTime *
               (currentSpeed_ * std::cos(middleHeading) -
                lateralVelocity * std::sin(middleHeading));
    truckY_ += config.sampleTime *
               (currentSpeed_ * std::sin(middleHeading) +
                lateralVelocity * std::cos(middleHeading));
    truckHeading_ =
        normalizedAngle(truckHeading_ + config.sampleTime * yawRate);
    physicalState_ = nextPhysical;
    time_ += config.sampleTime;
    updatePathProgress();
    updateMeasuredErrorState();
    if (updateRuntimeAlerts()) {
        record();
        return;
    }
    record();
    if (distance_ >= path_.length() -
                         currentSpeed_ * config.sampleTime) {
        simulationState_ = SimulationState::finished;
    }
}

const DemoSettings& DemoSession::settings() const noexcept {
    return settings_;
}

const truck_model::ReferencePath& DemoSession::path() const noexcept {
    return path_;
}

const truck_model::Vector<6>& DemoSession::state() const noexcept {
    return state_;
}

const std::vector<Telemetry>& DemoSession::history() const noexcept {
    return history_;
}

std::size_t DemoSession::telemetryRevision() const noexcept {
    return telemetryRevision_;
}

const std::vector<truck_model::Vector<6>>&
DemoSession::predictedStates() const noexcept {
    return predictedStates_;
}

SimulationState DemoSession::simulationState() const noexcept {
    return simulationState_;
}

double DemoSession::time() const noexcept {
    return time_;
}

double DemoSession::distance() const noexcept {
    return distance_;
}

double DemoSession::steering() const noexcept {
    return steering_;
}

double DemoSession::currentSpeed() const noexcept {
    return currentSpeed_;
}

const std::string& DemoSession::faultReason() const noexcept {
    return faultReason_;
}

const std::string& DemoSession::warningReason() const noexcept {
    return warningReason_;
}

const ArticulationReference&
DemoSession::articulationReference() const noexcept {
    return articulationReference_;
}

ArticulationSample DemoSession::currentArticulationReference() const {
    return articulationReference_.sample(time_);
}

const truck_model::ArticulationEstimate&
DemoSession::articulationEstimate() const noexcept {
    return articulationEstimate_;
}

const truck_model::ArticulationEstimate&
DemoSession::shadowArticulationEstimate() const noexcept {
    return shadowEstimate_;
}

const truck_model::ErrorLinearModel&
DemoSession::errorModel() const noexcept {
    return errorModel_;
}

const truck_model::DiscreteDynamicModel&
DemoSession::physicalPlant() const noexcept {
    return physicalPlant_;
}

double DemoSession::scheduledModelSpeed() const noexcept {
    return scheduledModelSpeed_;
}

double DemoSession::plannedSpeedNow() const {
    return plannedSpeed(distance_);
}

const truck_model::ArticulationEstimatorConfig&
DemoSession::estimatorConfig() const noexcept {
    return articulationEstimator_.config();
}

const truck_model::LateralMpc* DemoSession::controller() const noexcept {
    return controller_.get();
}

VehicleSnapshot DemoSession::currentVehicle() const {
    return vehicleSnapshot();
}

std::vector<truck_model::Point2d>
DemoSession::predictedWorldPositions() const {
    std::vector<truck_model::Point2d> result;
    result.reserve(predictedStates_.size());
    for (std::size_t i = 0; i < predictedStates_.size(); ++i) {
        const double predictedDistance =
            distance_ + static_cast<double>(i) *
                            currentSpeed_ * settings_.mpc.sampleTime;
        const auto reference = path_.sample(predictedDistance);
        result.push_back(
            errorOffsetPosition(reference, predictedStates_[i][0]));
    }
    return result;
}

DemoSettings DemoSession::defaultSettings() {
    DemoSettings settings;
    auto& p = settings.vehicle;
    p.m1 = 8000.0;
    p.iz1 = 25000.0;
    p.a1 = 1.5;
    p.b1 = 2.5;
    p.c1f = 220000.0;
    p.c1r = 300000.0;
    p.m2 = 18000.0;
    p.iz2 = 140000.0;
    p.a2 = 4.0;
    p.b2 = 3.0;
    p.c2r = 500000.0;
    p.d1 = p.b1;
    p.vx = 15.0;
    settings.mpc.sampleTime = 0.05;
    settings.mpc.horizon = 40;
    settings.terminalWeightFactor = 2.5;
    settings.applyTerminalFactor();
    settings.applyEstimatorMeasurementFromLidar();
    return settings;
}

truck_model::ArticulationEstimatorConfig DemoSession::legacyFusionConfig() {
    truck_model::ArticulationEstimatorConfig config;
    config.processModel = truck_model::ArticulationProcessModel::kinematic;
    config.compatibility.nearestFrameAlignment = true;
    config.compatibility.diagonalEulerProcessNoise = true;
    config.estimateLidarBias = true;
    // v1 shipped a 0.4 s window, which has no margin at the specified 400 ms
    // worst-case latency. Keeping it is what makes the comparison meaningful.
    config.historyHorizon = 0.4;
    config.lostTimeout = 0.4;
    return config;
}

DemoSettings DemoSession::fusionComparisonSettings() {
    auto settings = defaultSettings();
    settings.lidarFusionEnabled = true;
    settings.lidarPeriod = 0.1;
    settings.lidarDelayMin = 0.1;
    settings.lidarDelayMax = 0.4;
    // 4 degrees, the perception figure the handover document quotes.
    settings.lidarNoiseStd = 0.0698131700798;
    settings.ekfMeasurementFollowsLidar = true;
    settings.initializeEstimatorFromTruth = false;
    settings.inputYawRateNoiseStd = 0.005;
    settings.inputSpeedNoiseStd = 0.2;
    settings.adaptiveSpeedEnabled = false;
    settings.shadowEstimatorEnabled = true;
    settings.shadowEstimator = legacyFusionConfig();
    settings.applyEstimatorMeasurementFromLidar();
    return settings;
}

truck_model::ReferencePath DemoSession::defaultPath() {
    return defaultScenarioPath();
}

truck_model::ReferencePath DemoSession::highCurvaturePath() {
    return highCurvatureScenarioPath();
}

VehicleSnapshot DemoSession::vehicleSnapshot() const {
    VehicleSnapshot result;
    result.truckX = truckX_;
    result.truckY = truckY_;
    result.truckHeading = truckHeading_;
    result.hitchX =
        result.truckX - settings_.vehicle.d1 * std::cos(result.truckHeading);
    result.hitchY =
        result.truckY - settings_.vehicle.d1 * std::sin(result.truckHeading);
    result.trailerHeading =
        normalizedAngle(result.truckHeading - physicalState_[3]);
    result.trailerX =
        result.hitchX -
        settings_.vehicle.a2 * std::cos(result.trailerHeading);
    result.trailerY =
        result.hitchY -
        settings_.vehicle.a2 * std::sin(result.trailerHeading);
    return result;
}

void DemoSession::validatePathForSettings(
    const truck_model::ReferencePath& path,
    const DemoSettings& settings) const {
    if (path.length() < 10.0) {
        throw std::invalid_argument("reference path must be at least 10 m long");
    }
    const double wheelbase = settings.vehicle.a1 + settings.vehicle.b1;
    const double maximumGeometricCurvature = std::min(
        kMaximumDemoCurvature,
        0.85 * std::tan(settings.mpc.maxSteering) / wheelbase);
    double observedCurvature = 0.0;
    double curvatureLocation = 0.0;
    double minimumAllowedSpeed = settings.vehicle.vx;
    double speedLimitLocation = 0.0;
    for (const auto& point : path.points()) {
        if (std::abs(point.curvature) > observedCurvature) {
            observedCurvature = std::abs(point.curvature);
            curvatureLocation = point.s;
        }
        const double speedLimit =
            speedLimitForReference(point, settings);
        if (speedLimit < minimumAllowedSpeed) {
            minimumAllowedSpeed = speedLimit;
            speedLimitLocation = point.s;
        }
    }
    if (observedCurvature > maximumGeometricCurvature) {
        std::ostringstream message;
        message << "path curvature " << observedCurvature
                << " 1/m at s=" << curvatureLocation
                << " m exceeds the steering geometry limit "
                << maximumGeometricCurvature
                << " 1/m; draw a wider curve";
        throw std::invalid_argument(message.str());
    }
    const double requiredMinimumSpeed =
        settings.adaptiveSpeedEnabled ? settings.minimumSpeed
                                      : settings.vehicle.vx;
    if (minimumAllowedSpeed + 1.0e-9 < requiredMinimumSpeed) {
        std::ostringstream message;
        message << "path requires " << minimumAllowedSpeed
                << " m/s at s=" << speedLimitLocation
                << " m, below the configured minimum "
                << requiredMinimumSpeed
                << " m/s; lower minimum speed or smooth the path";
        throw std::invalid_argument(message.str());
    }
}

double DemoSession::pathSpeedLimit(
    double pathDistance,
    const DemoSettings& settings) const {
    if (path_.empty()) {
        return settings.vehicle.vx;
    }
    return speedLimitForReference(path_.sample(pathDistance), settings);
}

double DemoSession::plannedSpeed(double pathDistance) const {
    if (!settings_.adaptiveSpeedEnabled || path_.empty()) {
        return settings_.vehicle.vx;
    }
    double target = pathSpeedLimit(pathDistance, settings_);
    constexpr double previewSpacing = 1.0;
    for (double ahead = previewSpacing;
         ahead <= settings_.speedLookaheadDistance;
         ahead += previewSpacing) {
        const double localLimit =
            pathSpeedLimit(pathDistance + ahead, settings_);
        const double brakingLimit = std::sqrt(
            localLimit * localLimit +
            2.0 * settings_.maximumDeceleration * ahead);
        target = std::min(target, brakingLimit);
    }
    return std::clamp(
        target, settings_.minimumSpeed, settings_.vehicle.vx);
}

void DemoSession::updateAdaptiveSpeed() {
    const double target = plannedSpeed(distance_);
    const double sampleTime = settings_.mpc.sampleTime;
    if (target < currentSpeed_) {
        currentSpeed_ = std::max(
            target,
            currentSpeed_ -
                settings_.maximumDeceleration * sampleTime);
    } else {
        currentSpeed_ = std::min(
            target,
            currentSpeed_ +
                settings_.maximumAcceleration * sampleTime);
    }
    const double scheduleError =
        std::abs(currentSpeed_ - scheduledModelSpeed_);
    const bool targetReached = std::abs(currentSpeed_ - target) < 1.0e-9;
    if (scheduleError >= kModelSpeedScheduleThreshold ||
        (targetReached && scheduleError > 1.0e-9)) {
        rebuildModelsForSpeed(currentSpeed_);
        updateMeasuredErrorState();
    }
}

void DemoSession::rebuildModelsForSpeed(double speed) {
    auto scheduledVehicle = settings_.vehicle;
    scheduledVehicle.vx = speed;
    const auto errorModel =
        truck_model::buildErrorModel(scheduledVehicle);
    const auto dynamicModel =
        truck_model::buildDynamicModel(scheduledVehicle);
    errorModel_ = errorModel;
    physicalPlant_ = truck_model::discretizeZeroOrderHold(
        dynamicModel, settings_.mpc.sampleTime);
    controller_ = std::make_unique<truck_model::LateralMpc>(
        errorModel_, settings_.mpc);
    controller_->reset(steering_);
    scheduledModelSpeed_ = speed;
    modelsRebuiltThisStep_ = true;
}

void DemoSession::updatePathProgress() {
    const double lateralWindow = std::abs(state_[0]) + 8.0;
    const double forward = std::max(
        {5.0,
         3.0 * currentSpeed_ * settings_.mpc.sampleTime,
         lateralWindow});
    const double backward = std::min(
        distance_,
        std::max(2.0, 0.5 * lateralWindow));
    const auto projection = path_.project(
        {truckX_, truckY_},
        distance_ - backward,
        distance_ + forward);
    distance_ = projection.s;
}

void DemoSession::updateMeasuredErrorState() {
    const auto reference = path_.sample(distance_);
    const double deltaX = truckX_ - reference.x;
    const double deltaY = truckY_ - reference.y;
    state_[0] =
        -deltaX * std::sin(reference.heading) +
        deltaY * std::cos(reference.heading);
    state_[1] =
        physicalState_[0] +
        currentSpeed_ *
            normalizedAngle(truckHeading_ - reference.heading);
    state_[2] = normalizedAngle(truckHeading_ - reference.heading);
    state_[3] =
        physicalState_[1] -
        currentSpeed_ * reference.curvature;
    state_[4] = physicalState_[3];
    state_[5] = physicalState_[1] - physicalState_[2];
    if (!settings_.lidarFusionEnabled) {
        articulationEstimate_ = {};
        articulationEstimate_.time = time_;
        articulationEstimate_.articulation = state_[4];
        articulationEstimate_.articulationRate = state_[5];
        articulationEstimate_.trailerYawRate = physicalState_[2];
        shadowEstimate_ = articulationEstimate_;
        lastLidarArticulation_ = state_[4];
        lastLidarDelay_ = 0.0;
        lastLidarAccepted_ = false;
        measuredYawRate_ = physicalState_[1];
        measuredSpeed_ = currentSpeed_;
        return;
    }

    const auto inputs = sensedInputs();
    articulationEstimate_ = articulationEstimator_.predict(inputs);
    if (settings_.shadowEstimatorEnabled) {
        shadowEstimate_ = shadowEstimator_.predict(inputs);
    }
    captureDelayedLidar(physicalState_[3]);
    deliverDueLidar();
    articulationEstimate_ = articulationEstimator_.estimate();
    if (settings_.shadowEstimatorEnabled) {
        shadowEstimate_ = shadowEstimator_.estimate();
    } else {
        shadowEstimate_ = articulationEstimate_;
    }
    state_[4] = articulationEstimate_.articulation;
    state_[5] = articulationEstimate_.articulationRate;
}

bool DemoSession::updateRuntimeAlerts() {
    bool finite = std::isfinite(truckX_) && std::isfinite(truckY_) &&
                  std::isfinite(truckHeading_) &&
                  std::isfinite(steering_);
    for (const double value : physicalState_) {
        finite = finite && std::isfinite(value);
    }
    for (const double value : state_) {
        finite = finite && std::isfinite(value);
    }
    if (!finite) {
        faultReason_ = "simulation produced a non-finite state";
        warningReason_.clear();
        steering_ = 0.0;
        simulationState_ = SimulationState::faulted;
        return true;
    }

    std::ostringstream warning;
    if (std::abs(state_[0]) > 12.0) {
        warning << "lateral error " << state_[0]
                << " m exceeds the 12 m linear-model advisory range; "
                << "simulation continues";
    }
    if (std::abs(state_[2]) > 0.9) {
        if (warning.tellp() > 0) {
            warning << "; ";
        }
        warning << "heading error " << state_[2]
                << " rad exceeds the 0.9 rad linear-model advisory range; "
                << "simulation continues";
    }
    if (std::abs(state_[4]) > 0.8) {
        if (warning.tellp() > 0) {
            warning << "; ";
        }
        warning << "articulation " << state_[4]
                << " rad exceeds the 0.8 rad linear-model advisory range; "
                << "simulation continues";
    }
    warningReason_ = warning.str();
    return false;
}

void DemoSession::record() {
    const auto reference = path_.sample(distance_);
    Telemetry sample;
    sample.time = time_;
    sample.distance = distance_;
    sample.state = state_;
    sample.controllerState = controllerState();
    sample.physicalState = physicalState_;
    sample.plantVy1 = physicalState_[0];
    sample.plantR1 = physicalState_[1];
    sample.plantR2 = physicalState_[2];
    sample.plantPhi = physicalState_[3];
    sample.plantPhiDot = physicalState_[1] - physicalState_[2];
    auto plantParameters = settings_.vehicle;
    plantParameters.vx = currentSpeed_;
    sample.plantVy2 = truck_model::reconstructTrailerLateralVelocity(
        plantParameters, physicalState_);
    sample.steering = steering_;
    sample.unconstrainedSteering = unconstrainedSteering_;
    sample.speed = currentSpeed_;
    sample.plannedSpeed = plannedSpeed(distance_);
    sample.cruiseSpeed = settings_.vehicle.vx;
    sample.scheduledModelSpeed = scheduledModelSpeed_;
    sample.modelsRebuilt = modelsRebuiltThisStep_;
    sample.referenceCurvature = reference.curvature;
    sample.referenceCurvatureRate =
        currentSpeed_ * reference.curvatureDerivative;
    const auto articulation = articulationReference_.sample(time_);
    sample.referenceArticulation = articulation.value;
    sample.referenceArticulationRate = articulation.rate;
    sample.articulationTrackingError = state_[4] - articulation.value;
    sample.plantArticulation = physicalState_[3];
    sample.plantArticulationRate = sample.plantPhiDot;
    sample.estimatedArticulation = articulationEstimate_.articulation;
    sample.estimatedArticulationRate = articulationEstimate_.articulationRate;
    sample.shadowArticulation = shadowEstimate_.articulation;
    sample.shadowArticulationRate = shadowEstimate_.articulationRate;
    sample.measuredTruckYawRate = measuredYawRate_;
    sample.measuredSpeed = measuredSpeed_;
    sample.lidarArticulation = lastLidarArticulation_;
    sample.lidarStamp = lastLidarStamp_;
    sample.lidarDelay = lastLidarDelay_;
    sample.lidarQueueSize = pendingLidar_.size();
    sample.lidarDelivered = lastLidarDelivered_;
    sample.lidarAccepted = lastLidarAccepted_;
    sample.lidarGated = lastLidarGated_;
    sample.lidarDeliveredCount = lastLidarDeliveredCount_;
    sample.lidarAcceptedCount = lastLidarAcceptedCount_;
    sample.lidarGatedCount = lastLidarGatedCount_;
    sample.lidarDroppedCount = lastLidarDroppedCount_;
    sample.shadowAcceptedCount = lastShadowAcceptedCount_;
    sample.shadowDroppedCount = lastShadowDroppedCount_;
    sample.lidarOutcome = lastLidarOutcome_;
    sample.estimatorCoasting = articulationEstimate_.coasting;
    sample.estimator = articulationEstimate_;
    sample.shadowEstimator = shadowEstimate_;
    sample.predictedStates = predictedStates_;
    sample.warningActive = !warningReason_.empty();
    sample.warning = warningReason_;
    sample.vehicle = vehicleSnapshot();
    history_.push_back(sample);
    constexpr std::size_t kMaxTelemetryHistory = 20000;
    constexpr std::size_t kTelemetryTrim = 2000;
    if (history_.size() > kMaxTelemetryHistory) {
        history_.erase(
            history_.begin(), history_.begin() + static_cast<std::ptrdiff_t>(kTelemetryTrim));
    }
    modelsRebuiltThisStep_ = false;
    ++telemetryRevision_;
}

void DemoSession::applyIsolatedArticulationWeights(
    DemoSettings& settings) const {
    // Path-error weights must be exactly zero. Any leftover Q_ey/Q_epsi
    // punishes the yaw needed to change phi, so the solver keeps delta tiny.
    settings.mpc.stateWeight = {0.0, 0.0, 0.0, 0.0, 40.0, 4.0};
    settings.terminalWeightFactor = 2.5;
    settings.mpc.steeringWeight = 4.0;
    settings.mpc.steeringRateWeight = 20.0;
}

truck_model::Vector<6> DemoSession::controllerState() const {
    if (!settings_.articulationTrackingExperiment) {
        return state_;
    }
    // Geometric ey/epsi are not plant states. Feed a state that reconstructs
    // the K27 physical vector so path wander cannot steal the phi channel.
    const double curvature =
        path_.empty() ? 0.0 : path_.sample(distance_).curvature;
    truck_model::Vector<6> isolated{};
    isolated[1] = physicalState_[0];
    isolated[3] = physicalState_[1] - currentSpeed_ * curvature;
    isolated[4] = state_[4];
    isolated[5] = state_[5];
    return isolated;
}

void DemoSession::resetArticulationEstimator() {
    pendingLidar_.clear();
    lastLidarScanTime_ = -1.0;
    lastLidarArticulation_ = physicalState_[3];
    lastLidarStamp_ = time_;
    lastLidarDelay_ = 0.0;
    lastLidarAccepted_ = false;
    lastLidarDelivered_ = false;
    lastLidarGated_ = false;
    lastLidarDeliveredCount_ = 0;
    lastLidarAcceptedCount_ = 0;
    lastLidarGatedCount_ = 0;
    lastLidarDroppedCount_ = 0;
    lastShadowAcceptedCount_ = 0;
    lastShadowDroppedCount_ = 0;
    totalShadowDropped_ = 0;
    lastLidarOutcome_ = "none";
    nextLidarId_ = 1;
    unconstrainedSteering_ = steering_;
    modelsRebuiltThisStep_ = false;
    lidarRng_ = settings_.lidarRandomSeed == 0 ? 1u : settings_.lidarRandomSeed;
    inputRng_ = settings_.inputRandomSeed == 0 ? 7u : settings_.inputRandomSeed;

    const double seedArticulation =
        settings_.initializeEstimatorFromTruth ? physicalState_[3] : 0.0;

    auto estimatorConfig = settings_.articulationEstimator;
    articulationEstimator_.configure(settings_.vehicle, estimatorConfig);
    articulationEstimator_.reset(time_, seedArticulation);
    articulationEstimate_ = articulationEstimator_.estimate();

    auto shadowConfig = settings_.shadowEstimator;
    shadowEstimator_.configure(settings_.vehicle, shadowConfig);
    shadowEstimator_.reset(time_, seedArticulation);
    shadowEstimate_ = shadowEstimator_.estimate();
    measuredYawRate_ = physicalState_[1];
    measuredSpeed_ = currentSpeed_;
}

double DemoSession::nextInputNormal() {
    inputRng_ = inputRng_ * 1664525u + 1013904223u;
    const double u1 = std::max(
        static_cast<double>(inputRng_ >> 8) * (1.0 / 16777216.0), 1.0e-12);
    inputRng_ = inputRng_ * 1664525u + 1013904223u;
    const double u2 = static_cast<double>(inputRng_ >> 8) * (1.0 / 16777216.0);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

truck_model::ArticulationInputs DemoSession::sensedInputs() {
    truck_model::ArticulationInputs inputs;
    inputs.time = time_;
    inputs.truckYawRate = physicalState_[1] + settings_.inputYawRateBias +
                          settings_.inputYawRateNoiseStd * nextInputNormal();
    inputs.speed =
        currentSpeed_ + settings_.inputSpeedNoiseStd * nextInputNormal();
    // A negative sensed speed would break the 1/U tyre terms in the scheduled
    // model; clamp rather than propagate an impossible reading.
    inputs.speed = std::max(inputs.speed, 0.1);
    inputs.steering = steering_;
    measuredYawRate_ = inputs.truckYawRate;
    measuredSpeed_ = inputs.speed;
    return inputs;
}

void DemoSession::captureDelayedLidar(double plantArticulation) {
    if (lastLidarScanTime_ >= 0.0 &&
        time_ + 1.0e-12 < lastLidarScanTime_ + settings_.lidarPeriod) {
        return;
    }
    const double span = settings_.lidarDelayMax - settings_.lidarDelayMin;
    const double delay =
        settings_.lidarDelayMin + span * nextLidarUniform();
    PendingLidar pending;
    pending.deliverTime = time_ + delay;
    pending.measurement.stamp = time_;
    pending.measurement.articulation = plantArticulation +
                                       settings_.lidarInstallationBias +
                                       settings_.lidarNoiseStd * nextLidarNormal();
    pending.measurement.id = nextLidarId_++;
    pendingLidar_.push_back(pending);
    lastLidarScanTime_ = time_;
}

void DemoSession::deliverDueLidar() {
    lastLidarAccepted_ = false;
    lastLidarDelivered_ = false;
    lastLidarGated_ = false;
    lastLidarDeliveredCount_ = 0;
    lastLidarAcceptedCount_ = 0;
    lastLidarGatedCount_ = 0;
    lastLidarDroppedCount_ = 0;
    lastShadowAcceptedCount_ = 0;
    lastShadowDroppedCount_ = 0;
    lastLidarOutcome_ = "none";

    // Service packets in arrival order rather than scan order. With a variable
    // latency a later scan can arrive first, which is exactly the out-of-order
    // case the replay has to survive; a plain FIFO would hide it.
    while (true) {
        auto earliest = pendingLidar_.end();
        for (auto it = pendingLidar_.begin(); it != pendingLidar_.end(); ++it) {
            if (it->deliverTime > time_ + 1.0e-12) {
                continue;
            }
            if (earliest == pendingLidar_.end() ||
                it->deliverTime < earliest->deliverTime) {
                earliest = it;
            }
        }
        if (earliest == pendingLidar_.end()) {
            break;
        }

        const auto pending = *earliest;
        pendingLidar_.erase(earliest);

        lastLidarDelivered_ = true;
        ++lastLidarDeliveredCount_;
        lastLidarArticulation_ = pending.measurement.articulation;
        lastLidarStamp_ = pending.measurement.stamp;
        lastLidarDelay_ = time_ - pending.measurement.stamp;

        articulationEstimate_ =
            articulationEstimator_.updateLidar(pending.measurement);
        if (settings_.shadowEstimatorEnabled) {
            shadowEstimate_ = shadowEstimator_.updateLidar(pending.measurement);
            if (shadowEstimate_.measurementAccepted) {
                ++lastShadowAcceptedCount_;
            } else if (!shadowEstimate_.measurementGated) {
                ++lastShadowDroppedCount_;
                ++totalShadowDropped_;
            }
        }

        lastLidarAccepted_ = articulationEstimate_.measurementAccepted;
        lastLidarGated_ = articulationEstimate_.measurementGated;
        lastLidarOutcome_ =
            truck_model::measurementOutcomeName(articulationEstimate_.outcome);
        if (articulationEstimate_.measurementAccepted) {
            ++lastLidarAcceptedCount_;
        } else if (articulationEstimate_.measurementGated) {
            ++lastLidarGatedCount_;
        } else {
            ++lastLidarDroppedCount_;
        }
    }
}

double DemoSession::nextLidarUniform() {
    lidarRng_ = lidarRng_ * 1664525u + 1013904223u;
    return static_cast<double>(lidarRng_ >> 8) * (1.0 / 16777216.0);
}

double DemoSession::nextLidarNormal() {
    const double u1 = std::max(nextLidarUniform(), 1.0e-12);
    const double u2 = nextLidarUniform();
    return std::sqrt(-2.0 * std::log(u1)) *
           std::cos(2.0 * kPi * u2);
}

void DemoSession::rebuildArticulationReference() {
    if (settings_.articulationReference.kind ==
        ArticulationReferenceKind::drawn) {
        if (articulationReference_.config().kind !=
            ArticulationReferenceKind::drawn) {
            articulationReference_ = {};
        }
        return;
    }
    articulationReference_ =
        ArticulationReference::fromConfig(settings_.articulationReference);
}

std::string DemoSession::exportRunLog(const std::string& directory) const {
    return writeSessionLog(directory, *this);
}

}  // namespace truck_demo
