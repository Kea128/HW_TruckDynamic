#include "truck_model/articulation_estimator.hpp"

#include "truck_model/matrix_exponential.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace truck_model {
namespace {

constexpr double kPi = 3.14159265358979323846;

void requireValid(const Parameters& parameters) {
    const auto error = parameters.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
}

// Folds an angle into (-pi, pi]. The upper bound is inclusive so that the
// representation is unique; a bare pair of while loops would leave -pi and +pi
// both reachable.
double wrapAngle(double angle) {
    if (!std::isfinite(angle)) {
        return angle;
    }
    const double shifted = std::remainder(angle, 2.0 * kPi);
    return shifted <= -kPi ? shifted + 2.0 * kPi : shifted;
}

double trailerLength(const Parameters& parameters) {
    return parameters.a2 + parameters.b2;
}

double hitchOffset(const Parameters& parameters) {
    return parameters.b1 - parameters.d1;
}

void requirePositive(
    std::ostringstream& errors,
    double value,
    const char* name) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        errors << name << " must be finite and > 0; ";
    }
}

void requireNonnegative(
    std::ostringstream& errors,
    double value,
    const char* name) {
    if (value < 0.0 || !std::isfinite(value)) {
        errors << name << " must be finite and >= 0; ";
    }
}

}  // namespace

std::string ArticulationNoiseDensities::validationError() const {
    std::ostringstream errors;
    requirePositive(errors, articulationRate, "noiseDensity.articulationRate");
    requireNonnegative(errors, trailerYawBias, "noiseDensity.trailerYawBias");
    requireNonnegative(errors, lidarBias, "noiseDensity.lidarBias");
    requireNonnegative(errors, truckYawRate, "noiseDensity.truckYawRate");
    requireNonnegative(errors, speed, "noiseDensity.speed");
    requireNonnegative(
        errors, truckLateralVelocity, "noiseDensity.truckLateralVelocity");
    requireNonnegative(errors, trailerYawRate, "noiseDensity.trailerYawRate");
    return errors.str();
}

std::string ArticulationEstimatorConfig::validationError() const {
    std::ostringstream errors;
    requirePositive(errors, historyHorizon, "historyHorizon");
    requirePositive(errors, measurementVariance, "measurementVariance");
    requirePositive(errors, mahalanobisGate, "mahalanobisGate");
    requirePositive(errors, lostTimeout, "lostTimeout");
    requirePositive(errors, linkTimeout, "linkTimeout");
    requirePositive(
        errors, initialArticulationVariance, "initialArticulationVariance");
    requirePositive(
        errors,
        initialTrailerYawBiasVariance,
        "initialTrailerYawBiasVariance");
    requirePositive(
        errors, initialLidarBiasVariance, "initialLidarBiasVariance");
    requirePositive(
        errors,
        initialTruckLateralVelocityVariance,
        "initialTruckLateralVelocityVariance");
    requirePositive(
        errors, initialTrailerYawRateVariance, "initialTrailerYawRateVariance");
    requirePositive(errors, modelRefreshSpeedStep, "modelRefreshSpeedStep");
    requirePositive(errors, minimumModelSpeed, "minimumModelSpeed");
    if (!std::isfinite(lidarBiasCalibration)) {
        errors << "lidarBiasCalibration must be finite; ";
    }
    if (consecutiveRejectLimit < 1) {
        errors << "consecutiveRejectLimit must be >= 1; ";
    }
    errors << noiseDensity.validationError();
    return errors.str();
}

double kinematicTrailerYawRate(
    const Parameters& parameters,
    double speed,
    double truckYawRate,
    double articulation) {
    requireValid(parameters);
    const double length = trailerLength(parameters);
    return (speed * std::sin(articulation) +
            hitchOffset(parameters) * truckYawRate * std::cos(articulation)) /
           length;
}

// ---------------------------------------------------------------------------
// Kinematic (K5) process model
// ---------------------------------------------------------------------------

KinematicArticulationModel::KinematicArticulationModel(
    Parameters parameters,
    ArticulationEstimatorConfig config)
    : parameters_(std::move(parameters)), config_(std::move(config)) {}

Matrix<3, 3> KinematicArticulationModel::continuousJacobian(
    const Vector<3>& state,
    const Inputs& inputs) const {
    const double length = trailerLength(parameters_);
    const double offset = hitchOffset(parameters_);
    // d(r2_kin)/d(phi)
    const double sensitivity =
        (inputs.speed * std::cos(state[0]) -
         offset * inputs.truckYawRate * std::sin(state[0])) /
        length;
    Matrix<3, 3> jacobian{};
    jacobian[0][0] = -sensitivity;
    jacobian[0][1] = -1.0;
    return jacobian;
}

void KinematicArticulationModel::propagate(
    Vector<3>& state,
    Matrix<3, 3>& covariance,
    const Inputs& inputs,
    double dt,
    double processNoiseScale) const {
    if (!(dt > 0.0)) {
        normalize(state);
        return;
    }

    const double length = trailerLength(parameters_);
    const double offset = hitchOffset(parameters_);
    const double trailerRate =
        kinematicTrailerYawRate(
            parameters_, inputs.speed, inputs.truckYawRate, state[0]) +
        state[1];

    const auto continuous = continuousJacobian(state, inputs);

    // Sensitivity of phiDot to the measured inputs.
    const double rateSensitivity = 1.0 - offset * std::cos(state[0]) / length;
    const double speedSensitivity = -std::sin(state[0]) / length;

    const auto& density = config_.noiseDensity;
    Matrix<3, 3> continuousNoise{};
    continuousNoise[0][0] =
        processNoiseScale * density.articulationRate +
        rateSensitivity * rateSensitivity * density.truckYawRate +
        speedSensitivity * speedSensitivity * density.speed;
    continuousNoise[1][1] = density.trailerYawBias;
    continuousNoise[2][2] = config_.estimateLidarBias ? density.lidarBias : 0.0;

    // Euler mean; the transition below is the exact Jacobian of that map.
    state[0] = wrapAngle(state[0] + dt * (inputs.truckYawRate - trailerRate));

    Matrix<3, 3> transition = identityMatrix<3>();
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            transition[row][column] += dt * continuous[row][column];
        }
    }

    Matrix<3, 3> discreteNoise{};
    if (config_.compatibility.diagonalEulerProcessNoise) {
        discreteNoise[0][0] =
            processNoiseScale * density.articulationRate * dt +
            dt * dt *
                (rateSensitivity * rateSensitivity * density.truckYawRate +
                 speedSensitivity * speedSensitivity * density.speed);
        discreteNoise[1][1] = density.trailerYawBias * dt;
        discreteNoise[2][2] = continuousNoise[2][2] * dt;
    } else {
        discreteNoise =
            discretizeVanLoan(continuous, continuousNoise, dt).processNoise;
    }

    const auto left = matrixProduct(transition, covariance);
    covariance = matrixProduct(left, transposed(transition));
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            covariance[row][column] += discreteNoise[row][column];
        }
    }
}

double KinematicArticulationModel::predictMeasurement(
    const Vector<3>& state) const {
    return wrapAngle(state[0] + state[2]);
}

Vector<3> KinematicArticulationModel::measurementJacobian(
    const Vector<3>&) const {
    return {1.0, 0.0, 1.0};
}

double KinematicArticulationModel::measurementVariance() const {
    return config_.measurementVariance;
}

double KinematicArticulationModel::residual(
    double measurement,
    double predicted) const {
    return wrapAngle(measurement - predicted);
}

void KinematicArticulationModel::normalize(Vector<3>& state) const {
    state[0] = wrapAngle(state[0]);
}

// ---------------------------------------------------------------------------
// Dynamic reduced-(K27) process model
// ---------------------------------------------------------------------------

DynamicArticulationModel::DynamicArticulationModel(
    Parameters parameters,
    ArticulationEstimatorConfig config)
    : parameters_(std::move(parameters)), config_(std::move(config)) {}

void DynamicArticulationModel::refresh(double speed) const {
    const double scheduled = std::max(speed, config_.minimumModelSpeed);
    if (scheduledSpeed_ > 0.0 &&
        std::abs(scheduled - scheduledSpeed_) < config_.modelRefreshSpeedStep) {
        return;
    }

    Parameters scheduledParameters = parameters_;
    scheduledParameters.vx = scheduled;
    const auto plant = buildDynamicModel(scheduledParameters);

    // Rows 0 and 2 of the (K27) matrix are vy1Dot and r2Dot; row 1 (r1Dot) is
    // dropped because the truck yaw rate is measured and enters as an input.
    continuous_ = Matrix<4, 4>{};
    continuous_[0][0] = plant.a[0][0];
    continuous_[0][1] = plant.a[0][2];
    continuous_[0][2] = plant.a[0][3];
    continuous_[1][0] = plant.a[2][0];
    continuous_[1][1] = plant.a[2][2];
    continuous_[1][2] = plant.a[2][3];
    continuous_[2][1] = -1.0;

    inputYawRate_ = {plant.a[0][1], plant.a[2][1], 1.0, 0.0};
    inputSteering_ = {plant.b[0], plant.b[2], 0.0, 0.0};
    scheduledSpeed_ = scheduled;
}

Matrix<4, 4> DynamicArticulationModel::continuousJacobian(double speed) const {
    refresh(speed);
    return continuous_;
}

void DynamicArticulationModel::propagate(
    Vector<4>& state,
    Matrix<4, 4>& covariance,
    const Inputs& inputs,
    double dt,
    double processNoiseScale) const {
    if (!(dt > 0.0)) {
        normalize(state);
        return;
    }
    refresh(inputs.speed);

    // Exact zero-order hold: the model is linear once the speed is scheduled,
    // so the mean uses an augmented matrix exponential rather than Euler.
    Matrix<6, 6> generator{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            generator[row][column] = continuous_[row][column] * dt;
        }
        generator[row][4] = inputYawRate_[row] * dt;
        generator[row][5] = inputSteering_[row] * dt;
    }
    const auto expanded = matrixExponential(generator);

    Matrix<4, 4> transition{};
    Vector<4> yawRateGain{};
    Vector<4> steeringGain{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            transition[row][column] = expanded[row][column];
        }
        yawRateGain[row] = expanded[row][4];
        steeringGain[row] = expanded[row][5];
    }

    Vector<4> next{};
    for (std::size_t row = 0; row < 4; ++row) {
        double accumulated = yawRateGain[row] * inputs.truckYawRate +
                             steeringGain[row] * inputs.steering;
        for (std::size_t column = 0; column < 4; ++column) {
            accumulated += transition[row][column] * state[column];
        }
        next[row] = accumulated;
    }
    state = next;
    normalize(state);

    const auto& density = config_.noiseDensity;
    Matrix<4, 4> continuousNoise{};
    continuousNoise[0][0] = density.truckLateralVelocity;
    continuousNoise[1][1] = density.trailerYawRate;
    continuousNoise[2][2] = processNoiseScale * density.articulationRate;
    continuousNoise[3][3] = config_.estimateLidarBias ? density.lidarBias : 0.0;
    // Truck yaw-rate sensor noise enters through the same column as the input.
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            continuousNoise[row][column] +=
                inputYawRate_[row] * density.truckYawRate * inputYawRate_[column];
        }
    }

    Matrix<4, 4> discreteNoise{};
    if (config_.compatibility.diagonalEulerProcessNoise) {
        for (std::size_t i = 0; i < 4; ++i) {
            discreteNoise[i][i] = continuousNoise[i][i] * dt;
        }
    } else {
        discreteNoise =
            discretizeVanLoan(continuous_, continuousNoise, dt).processNoise;
    }

    const auto left = matrixProduct(transition, covariance);
    covariance = matrixProduct(left, transposed(transition));
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            covariance[row][column] += discreteNoise[row][column];
        }
    }
}

double DynamicArticulationModel::predictMeasurement(
    const Vector<4>& state) const {
    return wrapAngle(state[2] + state[3]);
}

Vector<4> DynamicArticulationModel::measurementJacobian(
    const Vector<4>&) const {
    return {0.0, 0.0, 1.0, 1.0};
}

double DynamicArticulationModel::measurementVariance() const {
    return config_.measurementVariance;
}

double DynamicArticulationModel::residual(
    double measurement,
    double predicted) const {
    return wrapAngle(measurement - predicted);
}

void DynamicArticulationModel::normalize(Vector<4>& state) const {
    state[2] = wrapAngle(state[2]);
}

// ---------------------------------------------------------------------------
// Facade
// ---------------------------------------------------------------------------

struct ArticulationEstimator::Impl {
    Parameters parameters{};
    ArticulationEstimatorConfig config{};
    bool configured{false};
    bool initialized{false};
    DelayedEkf<KinematicArticulationModel> kinematic;
    DelayedEkf<DynamicArticulationModel> dynamic;
    ArticulationEstimate estimate{};
    std::uint64_t nextIdentifier{1};
    double lastAcceptedArrival{0.0};
    bool everAcceptedArrival{false};
    double resetTime{0.0};

    [[nodiscard]] bool useDynamic() const {
        return config.processModel == ArticulationProcessModel::dynamic;
    }

    [[nodiscard]] DelayedEkfLimits limits() const {
        DelayedEkfLimits value;
        value.historyHorizon = config.historyHorizon;
        value.mahalanobisGate = config.mahalanobisGate;
        value.consecutiveRejectLimit = config.consecutiveRejectLimit;
        value.lostTimeout = config.lostTimeout;
        return value;
    }

    void publish(const ArticulationInputs& inputs, double time) {
        const double length = parameters.a2 + parameters.b2;
        (void)length;
        double articulation = 0.0;
        double trailerRate = 0.0;
        double trailerBias = 0.0;
        double lidarBias = 0.0;
        double lateralVelocity = 0.0;
        double variancePhi = 0.0;
        double varianceTrailer = 0.0;
        double varianceLidar = 0.0;
        double informationAge = 0.0;
        double lastAccepted = 0.0;
        int rejects = 0;
        std::size_t historySize = 0;
        bool coasting = false;

        const double kinematicRate = kinematicTrailerYawRate(
            parameters,
            inputs.speed,
            inputs.truckYawRate,
            useDynamic() ? dynamic.state()[2] : kinematic.state()[0]);

        if (useDynamic()) {
            const auto& state = dynamic.state();
            const auto& covariance = dynamic.covariance();
            lateralVelocity = state[0];
            trailerRate = state[1];
            articulation = state[2];
            lidarBias = state[3];
            trailerBias = trailerRate - kinematicRate;
            variancePhi = covariance[2][2];
            varianceTrailer = covariance[1][1];
            varianceLidar = covariance[3][3];
            informationAge = dynamic.informationAge();
            lastAccepted = dynamic.lastAcceptedStamp();
            rejects = dynamic.consecutiveRejects();
            historySize = dynamic.timelineSize();
            coasting = dynamic.coasting();
        } else {
            const auto& state = kinematic.state();
            const auto& covariance = kinematic.covariance();
            articulation = state[0];
            trailerBias = state[1];
            lidarBias = state[2];
            trailerRate = kinematicRate + trailerBias;
            variancePhi = covariance[0][0];
            varianceTrailer = covariance[1][1];
            varianceLidar = covariance[2][2];
            informationAge = kinematic.informationAge();
            lastAccepted = kinematic.lastAcceptedStamp();
            rejects = kinematic.consecutiveRejects();
            historySize = kinematic.timelineSize();
            coasting = kinematic.coasting();
        }

        const double wheelbase = parameters.a1 + parameters.b1;
        const double bicycleYaw =
            wheelbase > 0.0
                ? inputs.speed * std::tan(inputs.steering) / wheelbase
                : 0.0;

        estimate.time = time;
        estimate.articulation = articulation;
        estimate.trailerYawRate = trailerRate;
        estimate.kinematicTrailerYawRate = kinematicRate;
        estimate.articulationRate = inputs.truckYawRate - trailerRate;
        estimate.trailerYawBias = trailerBias;
        estimate.lidarBias = lidarBias;
        estimate.truckLateralVelocity = lateralVelocity;
        estimate.truckYawResidual = inputs.truckYawRate - bicycleYaw;
        estimate.covariancePhi = variancePhi;
        estimate.covarianceTrailerBias = varianceTrailer;
        estimate.covarianceLidarBias = varianceLidar;
        estimate.informationAge = informationAge;
        estimate.lastAcceptedStamp = lastAccepted;
        estimate.consecutiveRejects = rejects;
        estimate.historySize = historySize;
        estimate.coasting = coasting;
        estimate.arrivalGap =
            everAcceptedArrival ? time - lastAcceptedArrival : time - resetTime;
        estimate.linkStalled = estimate.arrivalGap > config.linkTimeout;
    }
};

ArticulationEstimator::ArticulationEstimator() : impl_(std::make_unique<Impl>()) {}

ArticulationEstimator::~ArticulationEstimator() = default;

ArticulationEstimator::ArticulationEstimator(
    ArticulationEstimator&&) noexcept = default;

ArticulationEstimator& ArticulationEstimator::operator=(
    ArticulationEstimator&&) noexcept = default;

ArticulationEstimator::ArticulationEstimator(
    Parameters parameters,
    ArticulationEstimatorConfig config)
    : impl_(std::make_unique<Impl>()) {
    configure(std::move(parameters), std::move(config));
}

void ArticulationEstimator::configure(
    Parameters parameters,
    ArticulationEstimatorConfig config) {
    requireValid(parameters);
    const auto error = config.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }

    impl_->parameters = std::move(parameters);
    impl_->config = std::move(config);
    impl_->configured = true;
    impl_->initialized = false;
    impl_->estimate = {};
    impl_->nextIdentifier = 1;
    impl_->everAcceptedArrival = false;

    if (impl_->useDynamic()) {
        impl_->dynamic.configure(
            DynamicArticulationModel(impl_->parameters, impl_->config),
            impl_->limits());
    } else {
        impl_->kinematic.configure(
            KinematicArticulationModel(impl_->parameters, impl_->config),
            impl_->limits());
    }
}

void ArticulationEstimator::reset(double time, double articulation) {
    if (!impl_->configured) {
        throw std::logic_error("ArticulationEstimator is not configured");
    }
    if (!std::isfinite(time) || !std::isfinite(articulation)) {
        throw std::invalid_argument(
            "estimator reset time and articulation must be finite");
    }

    const auto& config = impl_->config;
    ArticulationInputs inputs;
    inputs.time = time;

    if (impl_->useDynamic()) {
        Vector<4> state{0.0, 0.0, wrapAngle(articulation),
                        config.lidarBiasCalibration};
        Matrix<4, 4> covariance{};
        covariance[0][0] = config.initialTruckLateralVelocityVariance;
        covariance[1][1] = config.initialTrailerYawRateVariance;
        covariance[2][2] = config.initialArticulationVariance;
        covariance[3][3] =
            config.estimateLidarBias ? config.initialLidarBiasVariance : 0.0;
        impl_->dynamic.reset(time, state, covariance, inputs);
    } else {
        Vector<3> state{wrapAngle(articulation), 0.0,
                        config.lidarBiasCalibration};
        Matrix<3, 3> covariance{};
        covariance[0][0] = config.initialArticulationVariance;
        covariance[1][1] = config.initialTrailerYawBiasVariance;
        covariance[2][2] =
            config.estimateLidarBias ? config.initialLidarBiasVariance : 0.0;
        impl_->kinematic.reset(time, state, covariance, inputs);
    }

    impl_->initialized = true;
    impl_->resetTime = time;
    impl_->lastAcceptedArrival = time;
    impl_->everAcceptedArrival = false;
    impl_->estimate = {};
    impl_->estimate.lastAcceptedStamp = time;
    impl_->publish(inputs, time);
}

ArticulationEstimate ArticulationEstimator::predict(
    const ArticulationInputs& inputs) {
    if (!impl_->configured) {
        throw std::logic_error("ArticulationEstimator is not configured");
    }
    if (!std::isfinite(inputs.time) || !std::isfinite(inputs.truckYawRate) ||
        !std::isfinite(inputs.speed)) {
        throw std::invalid_argument("estimator inputs must be finite");
    }
    if (!impl_->initialized) {
        reset(inputs.time, 0.0);
    }

    if (impl_->useDynamic()) {
        impl_->dynamic.predict(inputs, inputs.time);
    } else {
        impl_->kinematic.predict(inputs, inputs.time);
    }

    impl_->estimate.outcome = MeasurementOutcome::notInitialized;
    impl_->estimate.measurementAccepted = false;
    impl_->estimate.measurementGated = false;
    impl_->estimate.replayedEntries = 0;
    impl_->publish(inputs, inputs.time);
    return impl_->estimate;
}

ArticulationEstimate ArticulationEstimator::updateLidar(
    const ArticulationLidarMeasurement& measurement) {
    if (!impl_->configured) {
        throw std::logic_error("ArticulationEstimator is not configured");
    }
    impl_->estimate.measurementAccepted = false;
    impl_->estimate.measurementGated = false;
    if (!impl_->initialized) {
        impl_->estimate.outcome = MeasurementOutcome::notInitialized;
        return impl_->estimate;
    }
    if (!std::isfinite(measurement.stamp) ||
        !std::isfinite(measurement.articulation)) {
        throw std::invalid_argument("lidar measurement must be finite");
    }

    double stamp = measurement.stamp;
    if (impl_->config.compatibility.nearestFrameAlignment) {
        stamp = impl_->useDynamic() ? impl_->dynamic.time()
                                    : impl_->kinematic.time();
        // The legacy path could only align to a stored frame; approximate it by
        // snapping onto the newest one when the scan is not older than it.
        if (measurement.stamp < stamp) {
            const double horizon = impl_->config.historyHorizon;
            const double oldest = stamp - horizon;
            stamp = std::max(measurement.stamp, oldest);
        }
    }

    const std::uint64_t identifier =
        measurement.id != 0 ? measurement.id : impl_->nextIdentifier++;

    ArticulationInputs inputs;
    double now = 0.0;
    typename DelayedEkf<KinematicArticulationModel>::MeasurementReport
        kinematicReport;
    typename DelayedEkf<DynamicArticulationModel>::MeasurementReport
        dynamicReport;

    MeasurementOutcome outcome = MeasurementOutcome::notInitialized;
    double innovation = 0.0;
    double innovationCovariance = 0.0;
    double mahalanobis = 0.0;
    double alignedStamp = 0.0;
    std::size_t replayed = 0;
    Vector<3> kinematicGain{};
    Vector<4> dynamicGain{};

    if (impl_->useDynamic()) {
        dynamicReport = impl_->dynamic.update(
            stamp, measurement.articulation, identifier);
        outcome = dynamicReport.outcome;
        innovation = dynamicReport.innovation;
        innovationCovariance = dynamicReport.innovationCovariance;
        mahalanobis = dynamicReport.mahalanobis;
        alignedStamp = dynamicReport.alignedStamp;
        replayed = dynamicReport.replayedEntries;
        dynamicGain = dynamicReport.gain;
        inputs = impl_->dynamic.latestInputs();
        now = impl_->dynamic.time();
    } else {
        kinematicReport = impl_->kinematic.update(
            stamp, measurement.articulation, identifier);
        outcome = kinematicReport.outcome;
        innovation = kinematicReport.innovation;
        innovationCovariance = kinematicReport.innovationCovariance;
        mahalanobis = kinematicReport.mahalanobis;
        alignedStamp = kinematicReport.alignedStamp;
        replayed = kinematicReport.replayedEntries;
        kinematicGain = kinematicReport.gain;
        inputs = impl_->kinematic.latestInputs();
        now = impl_->kinematic.time();
    }

    if (outcome == MeasurementOutcome::accepted) {
        impl_->lastAcceptedArrival = now;
        impl_->everAcceptedArrival = true;
    }

    impl_->publish(inputs, now);
    impl_->estimate.outcome = outcome;
    impl_->estimate.measurementAccepted =
        outcome == MeasurementOutcome::accepted;
    impl_->estimate.measurementGated = outcome == MeasurementOutcome::gated;
    impl_->estimate.innovation = innovation;
    impl_->estimate.innovationCovariance = innovationCovariance;
    impl_->estimate.mahalanobis = mahalanobis;
    impl_->estimate.alignedStamp = alignedStamp;
    impl_->estimate.replayedEntries = replayed;
    if (impl_->useDynamic()) {
        impl_->estimate.kalmanGainPhi = dynamicGain[2];
        impl_->estimate.kalmanGainTrailerBias = dynamicGain[1];
        impl_->estimate.kalmanGainLidarBias = dynamicGain[3];
    } else {
        impl_->estimate.kalmanGainPhi = kinematicGain[0];
        impl_->estimate.kalmanGainTrailerBias = kinematicGain[1];
        impl_->estimate.kalmanGainLidarBias = kinematicGain[2];
    }
    return impl_->estimate;
}

const ArticulationEstimate& ArticulationEstimator::estimate() const noexcept {
    return impl_->estimate;
}

const ArticulationEstimatorConfig& ArticulationEstimator::config()
    const noexcept {
    return impl_->config;
}

const Parameters& ArticulationEstimator::parameters() const noexcept {
    return impl_->parameters;
}

bool ArticulationEstimator::initialized() const noexcept {
    return impl_->initialized;
}

}  // namespace truck_model
