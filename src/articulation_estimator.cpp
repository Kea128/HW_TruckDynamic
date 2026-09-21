#include "truck_model/articulation_estimator.hpp"

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

double wrapAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

Vector<3> add(const Vector<3>& left, const Vector<3>& right) {
    return {left[0] + right[0], left[1] + right[1], left[2] + right[2]};
}

Vector<3> scale(const Vector<3>& value, double factor) {
    return {value[0] * factor, value[1] * factor, value[2] * factor};
}

Matrix<3, 3> identity3() {
    Matrix<3, 3> result{};
    result[0][0] = 1.0;
    result[1][1] = 1.0;
    result[2][2] = 1.0;
    return result;
}

Matrix<3, 3> multiply(const Matrix<3, 3>& left, const Matrix<3, 3>& right) {
    Matrix<3, 3> result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t inner = 0; inner < 3; ++inner) {
                result[row][column] += left[row][inner] * right[inner][column];
            }
        }
    }
    return result;
}

Matrix<3, 3> transpose(const Matrix<3, 3>& matrix) {
    Matrix<3, 3> result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result[row][column] = matrix[column][row];
        }
    }
    return result;
}

Matrix<3, 3> add(const Matrix<3, 3>& left, const Matrix<3, 3>& right) {
    Matrix<3, 3> result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result[row][column] = left[row][column] + right[row][column];
        }
    }
    return result;
}

double trailerLength(const Parameters& parameters) {
    return parameters.a2 + parameters.b2;
}

double hitchOffset(const Parameters& parameters) {
    return parameters.b1 - parameters.d1;
}

double trailerYawRateFromState(
    const Parameters& parameters,
    const ArticulationInputs& inputs,
    const Vector<3>& state) {
    return kinematicTrailerYawRate(
               parameters,
               inputs.speed,
               inputs.truckYawRate,
               state[0]) +
           state[1];
}

double trailerYawSensitivity(
    const Parameters& parameters,
    const ArticulationInputs& inputs,
    double articulation) {
    const double length = trailerLength(parameters);
    return (inputs.speed * std::cos(articulation) -
            hitchOffset(parameters) * inputs.truckYawRate *
                std::sin(articulation)) /
           length;
}

struct Propagated {
    Vector<3> state{};
    Matrix<3, 3> covariance{};
};

Propagated propagate(
    const Parameters& parameters,
    const ArticulationEstimatorConfig& config,
    const Vector<3>& state,
    const Matrix<3, 3>& covariance,
    const ArticulationInputs& inputs,
    double dt,
    bool coasting) {
    Propagated result;
    if (!(dt > 0.0)) {
        result.state = state;
        result.state[0] = wrapAngle(result.state[0]);
        result.covariance = covariance;
        return result;
    }

    const double r2 = trailerYawRateFromState(parameters, inputs, state);
    result.state[0] = wrapAngle(state[0] + dt * (inputs.truckYawRate - r2));
    result.state[1] = state[1];
    result.state[2] = state[2];

    Matrix<3, 3> jacobian = identity3();
    jacobian[0][0] = 1.0 - dt * trailerYawSensitivity(
                                    parameters, inputs, state[0]);
    jacobian[0][1] = -dt;

    const double length = trailerLength(parameters);
    const double offset = hitchOffset(parameters);
    const double dPhiDr1 = 1.0 - offset * std::cos(state[0]) / length;
    const double dPhiDu = -std::sin(state[0]) / length;
    const double extraPhiVariance =
        dt * dt *
        (dPhiDr1 * dPhiDr1 * config.truckYawRateVariance +
         dPhiDu * dPhiDu * config.speedVariance);
    const double coastInflation = coasting ? 4.0 : 1.0;

    Matrix<3, 3> processNoise{};
    processNoise[0][0] =
        coastInflation * config.processArticulationRateVariance * dt +
        extraPhiVariance;
    processNoise[1][1] = config.processTrailerYawBiasVariance * dt;
    processNoise[2][2] = config.processLidarBiasVariance * dt;

    result.covariance = add(
        multiply(multiply(jacobian, covariance), transpose(jacobian)),
        processNoise);
    return result;
}

void applyLidarUpdate(
    Vector<3>& state,
    Matrix<3, 3>& covariance,
    double measurement,
    double measurementVariance,
    double& innovation,
    double& innovationCovariance,
    Vector<3>& gain) {
    const double predicted = wrapAngle(state[0] + state[2]);
    innovation = wrapAngle(measurement - predicted);
    innovationCovariance =
        covariance[0][0] + covariance[0][2] + covariance[2][0] +
        covariance[2][2] + measurementVariance;
    if (!(innovationCovariance > 0.0) || !std::isfinite(innovationCovariance)) {
        throw std::runtime_error("articulation innovation covariance is invalid");
    }

    gain = {
        (covariance[0][0] + covariance[0][2]) / innovationCovariance,
        (covariance[1][0] + covariance[1][2]) / innovationCovariance,
        (covariance[2][0] + covariance[2][2]) / innovationCovariance};
    state = add(state, scale(gain, innovation));
    state[0] = wrapAngle(state[0]);

    Matrix<3, 3> kalmanH{};
    kalmanH[0][0] = gain[0];
    kalmanH[1][0] = gain[1];
    kalmanH[2][0] = gain[2];
    kalmanH[0][2] = gain[0];
    kalmanH[1][2] = gain[1];
    kalmanH[2][2] = gain[2];
    Matrix<3, 3> iMinusKh = identity3();
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            iMinusKh[row][column] -= kalmanH[row][column];
        }
    }
    Matrix<3, 3> krk{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            krk[row][column] = gain[row] * measurementVariance * gain[column];
        }
    }
    covariance = add(
        multiply(multiply(iMinusKh, covariance), transpose(iMinusKh)),
        krk);
}

}  // namespace

std::string ArticulationEstimatorConfig::validationError() const {
    std::ostringstream errors;
    const auto requirePositive = [&errors](double value, const char* name) {
        if (!(value > 0.0) || !std::isfinite(value)) {
            errors << name << " must be finite and > 0; ";
        }
    };
    const auto requireNonnegative =
        [&errors](double value, const char* name) {
            if (value < 0.0 || !std::isfinite(value)) {
                errors << name << " must be finite and >= 0; ";
            }
        };
    requirePositive(historyHorizon, "historyHorizon");
    requirePositive(measurementVariance, "measurementVariance");
    requirePositive(
        processArticulationRateVariance, "processArticulationRateVariance");
    requireNonnegative(
        processTrailerYawBiasVariance, "processTrailerYawBiasVariance");
    requireNonnegative(processLidarBiasVariance, "processLidarBiasVariance");
    requireNonnegative(truckYawRateVariance, "truckYawRateVariance");
    requireNonnegative(speedVariance, "speedVariance");
    requirePositive(mahalanobisGate, "mahalanobisGate");
    requirePositive(lostTimeout, "lostTimeout");
    requirePositive(
        initialArticulationVariance, "initialArticulationVariance");
    requirePositive(
        initialTrailerYawBiasVariance, "initialTrailerYawBiasVariance");
    requirePositive(initialLidarBiasVariance, "initialLidarBiasVariance");
    if (consecutiveRejectLimit < 1) {
        errors << "consecutiveRejectLimit must be >= 1; ";
    }
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

ArticulationEstimator::ArticulationEstimator(
    Parameters parameters,
    ArticulationEstimatorConfig config) {
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
    parameters_ = std::move(parameters);
    config_ = std::move(config);
    configured_ = true;
    initialized_ = false;
    history_.clear();
    estimate_ = {};
}

void ArticulationEstimator::reset(double time, double articulation) {
    requireConfigured();
    if (!std::isfinite(time) || !std::isfinite(articulation)) {
        throw std::invalid_argument(
            "estimator reset time and articulation must be finite");
    }
    Vector<3> state{wrapAngle(articulation), 0.0, 0.0};
    Matrix<3, 3> covariance{};
    covariance[0][0] = config_.initialArticulationVariance;
    covariance[1][1] = config_.initialTrailerYawBiasVariance;
    covariance[2][2] = config_.initialLidarBiasVariance;
    history_.clear();
    history_.push_back({time, state, covariance, {time, 0.0, 0.0, 0.0}});
    initialized_ = true;
    estimate_ = {};
    estimate_.lastAcceptedStamp = time;
    publishFromState(time, state, covariance, history_.back().inputs);
}

ArticulationEstimate ArticulationEstimator::predict(
    const ArticulationInputs& inputs) {
    requireConfigured();
    if (!std::isfinite(inputs.time) ||
        !std::isfinite(inputs.truckYawRate) ||
        !std::isfinite(inputs.speed)) {
        throw std::invalid_argument("estimator inputs must be finite");
    }
    if (!initialized_) {
        reset(inputs.time, 0.0);
        history_.back().inputs = inputs;
        publishFromState(
            inputs.time, history_.back().state, history_.back().covariance, inputs);
        return estimate_;
    }

    auto& latest = history_.back();
    if (inputs.time <= latest.time + 1.0e-15) {
        latest.inputs = inputs;
        publishFromState(latest.time, latest.state, latest.covariance, inputs);
        return estimate_;
    }

    const double dt = inputs.time - latest.time;
    const auto next = propagate(
        parameters_,
        config_,
        latest.state,
        latest.covariance,
        inputs,
        dt,
        estimate_.coasting);
    history_.push_back({inputs.time, next.state, next.covariance, inputs});
    trimHistory();
    publishFromState(inputs.time, next.state, next.covariance, inputs);
    return estimate_;
}

ArticulationEstimate ArticulationEstimator::updateLidar(
    const ArticulationLidarMeasurement& measurement) {
    requireConfigured();
    estimate_.measurementAccepted = false;
    estimate_.measurementGated = false;
    if (!initialized_ || history_.empty()) {
        return estimate_;
    }
    if (!std::isfinite(measurement.stamp) ||
        !std::isfinite(measurement.articulation)) {
        throw std::invalid_argument("lidar measurement must be finite");
    }

    const double oldest = history_.front().time;
    const double newest = history_.back().time;
    if (measurement.stamp < oldest - 1.0e-9 ||
        measurement.stamp > newest + 0.05) {
        ++estimate_.consecutiveRejects;
        estimate_.coasting =
            estimate_.consecutiveRejects >= config_.consecutiveRejectLimit ||
            newest - estimate_.lastAcceptedStamp > config_.lostTimeout;
        return estimate_;
    }

    const std::size_t index = nearestFrame(measurement.stamp);
    auto state = history_[index].state;
    auto covariance = history_[index].covariance;
    double innovation = 0.0;
    double innovationCovariance = 0.0;
    const double predicted = wrapAngle(state[0] + state[2]);
    innovation = wrapAngle(measurement.articulation - predicted);
    innovationCovariance =
        covariance[0][0] + covariance[0][2] + covariance[2][0] +
        covariance[2][2] + config_.measurementVariance;
    const double mahalanobis =
        innovation * innovation / std::max(innovationCovariance, 1.0e-18);
    estimate_.innovation = innovation;
    estimate_.innovationCovariance = innovationCovariance;
    if (!(mahalanobis <= config_.mahalanobisGate)) {
        estimate_.measurementGated = true;
        estimate_.mahalanobis = mahalanobis;
        estimate_.alignedStamp = history_[index].time;
        estimate_.kalmanGainPhi = 0.0;
        estimate_.kalmanGainTrailerBias = 0.0;
        estimate_.kalmanGainLidarBias = 0.0;
        ++estimate_.consecutiveRejects;
        estimate_.coasting =
            estimate_.consecutiveRejects >= config_.consecutiveRejectLimit ||
            newest - estimate_.lastAcceptedStamp > config_.lostTimeout;
        estimate_.historySize = history_.size();
        return estimate_;
    }

    Vector<3> gain{};
    applyLidarUpdate(
        state,
        covariance,
        measurement.articulation,
        config_.measurementVariance,
        innovation,
        innovationCovariance,
        gain);
    history_[index].state = state;
    history_[index].covariance = covariance;

    for (std::size_t i = index; i + 1 < history_.size(); ++i) {
        const double dt = history_[i + 1].time - history_[i].time;
        const auto next = propagate(
            parameters_,
            config_,
            history_[i].state,
            history_[i].covariance,
            history_[i + 1].inputs,
            dt,
            false);
        history_[i + 1].state = next.state;
        history_[i + 1].covariance = next.covariance;
    }

    estimate_.measurementAccepted = true;
    estimate_.consecutiveRejects = 0;
    estimate_.lastAcceptedStamp = measurement.stamp;
    estimate_.alignedStamp = history_[index].time;
    estimate_.coasting = false;
    estimate_.kalmanGainPhi = gain[0];
    estimate_.kalmanGainTrailerBias = gain[1];
    estimate_.kalmanGainLidarBias = gain[2];
    estimate_.mahalanobis = mahalanobis;
    const auto& current = history_.back();
    publishFromState(current.time, current.state, current.covariance, current.inputs);
    estimate_.measurementAccepted = true;
    estimate_.innovation = innovation;
    estimate_.innovationCovariance = innovationCovariance;
    estimate_.mahalanobis = mahalanobis;
    estimate_.alignedStamp = history_[index].time;
    estimate_.kalmanGainPhi = gain[0];
    estimate_.kalmanGainTrailerBias = gain[1];
    estimate_.kalmanGainLidarBias = gain[2];
    return estimate_;
}

const ArticulationEstimate&
ArticulationEstimator::estimate() const noexcept {
    return estimate_;
}

const ArticulationEstimatorConfig&
ArticulationEstimator::config() const noexcept {
    return config_;
}

bool ArticulationEstimator::initialized() const noexcept {
    return initialized_;
}

void ArticulationEstimator::requireConfigured() const {
    if (!configured_) {
        throw std::logic_error("ArticulationEstimator is not configured");
    }
}

void ArticulationEstimator::publishFromState(
    double time,
    const Vector<3>& state,
    const Matrix<3, 3>& covariance,
    const ArticulationInputs& inputs) {
    const int rejects = estimate_.consecutiveRejects;
    const double lastAccepted = estimate_.lastAcceptedStamp;
    const bool accepted = estimate_.measurementAccepted;
    const bool gated = estimate_.measurementGated;
    const double innovation = estimate_.innovation;
    const double innovationCovariance = estimate_.innovationCovariance;
    const double mahalanobis = estimate_.mahalanobis;
    const double gainPhi = estimate_.kalmanGainPhi;
    const double gainBias = estimate_.kalmanGainTrailerBias;
    const double gainLidar = estimate_.kalmanGainLidarBias;
    const double aligned = estimate_.alignedStamp;
    estimate_.time = time;
    estimate_.articulation = wrapAngle(state[0]);
    estimate_.trailerYawBias = state[1];
    estimate_.lidarBias = state[2];
    estimate_.kinematicTrailerYawRate = kinematicTrailerYawRate(
        parameters_, inputs.speed, inputs.truckYawRate, state[0]);
    estimate_.trailerYawRate =
        estimate_.kinematicTrailerYawRate + state[1];
    estimate_.articulationRate = inputs.truckYawRate - estimate_.trailerYawRate;
    const double wheelbase = parameters_.a1 + parameters_.b1;
    const double bicycleYaw =
        wheelbase > 0.0 ? inputs.speed * std::tan(inputs.steering) / wheelbase
                        : 0.0;
    estimate_.truckYawResidual = inputs.truckYawRate - bicycleYaw;
    estimate_.covariance = covariance;
    estimate_.covariancePhi = covariance[0][0];
    estimate_.covarianceTrailerBias = covariance[1][1];
    estimate_.covarianceLidarBias = covariance[2][2];
    estimate_.consecutiveRejects = rejects;
    estimate_.lastAcceptedStamp = lastAccepted;
    estimate_.measurementAccepted = accepted;
    estimate_.measurementGated = gated;
    estimate_.innovation = innovation;
    estimate_.innovationCovariance = innovationCovariance;
    estimate_.mahalanobis = mahalanobis;
    estimate_.kalmanGainPhi = gainPhi;
    estimate_.kalmanGainTrailerBias = gainBias;
    estimate_.kalmanGainLidarBias = gainLidar;
    estimate_.alignedStamp = aligned;
    estimate_.historySize = history_.size();
    estimate_.coasting =
        rejects >= config_.consecutiveRejectLimit ||
        (initialized_ && time - lastAccepted > config_.lostTimeout);
}

void ArticulationEstimator::trimHistory() {
    while (history_.size() > 1 &&
           history_.back().time - history_.front().time >
               config_.historyHorizon + 1.0e-12) {
        history_.pop_front();
    }
}

std::size_t ArticulationEstimator::nearestFrame(double stamp) const {
    std::size_t best = 0;
    double bestError = std::abs(history_.front().time - stamp);
    for (std::size_t i = 1; i < history_.size(); ++i) {
        const double error = std::abs(history_[i].time - stamp);
        if (error < bestError) {
            best = i;
            bestError = error;
        }
    }
    return best;
}

}  // namespace truck_model
