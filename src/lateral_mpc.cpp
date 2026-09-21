#include "truck_model/lateral_mpc.hpp"
#include "truck_model/linear_discretization.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace truck_model {
namespace {

constexpr std::size_t kPlantStates = 6;
constexpr std::size_t kAugmentedStates = 7;

template <std::size_t Size>
Matrix<Size, Size> matrixProduct(
    const Matrix<Size, Size>& lhs,
    const Matrix<Size, Size>& rhs) {
    Matrix<Size, Size> result{};
    for (std::size_t row = 0; row < Size; ++row) {
        for (std::size_t k = 0; k < Size; ++k) {
            for (std::size_t column = 0; column < Size; ++column) {
                result[row][column] += lhs[row][k] * rhs[k][column];
            }
        }
    }
    return result;
}

CurvatureSample previewAt(
    const std::vector<CurvatureSample>& preview,
    std::size_t index) {
    if (preview.empty()) {
        return {};
    }
    return preview[std::min(index, preview.size() - 1)];
}

Vector<6> stateReferenceAt(
    const std::vector<Vector<6>>& preview,
    std::size_t index) {
    if (preview.empty()) {
        return {};
    }
    return preview[std::min(index, preview.size() - 1)];
}

bool isFinite(double value) {
    return std::isfinite(value);
}

}  // namespace

std::string MpcConfig::validationError() const {
    std::ostringstream errors;
    if (!(sampleTime > 0.0) || !isFinite(sampleTime)) {
        errors << "sampleTime must be finite and > 0; ";
    }
    if (horizon == 0) {
        errors << "horizon must be > 0; ";
    }
    for (const double weight : stateWeight) {
        if (weight < 0.0 || !isFinite(weight)) {
            errors << "stateWeight entries must be finite and >= 0; ";
            break;
        }
    }
    for (const double weight : terminalWeight) {
        if (weight < 0.0 || !isFinite(weight)) {
            errors << "terminalWeight entries must be finite and >= 0; ";
            break;
        }
    }
    if (!(steeringWeight > 0.0) || !isFinite(steeringWeight)) {
        errors << "steeringWeight must be finite and > 0; ";
    }
    if (steeringRateWeight < 0.0 || !isFinite(steeringRateWeight)) {
        errors << "steeringRateWeight must be finite and >= 0; ";
    }
    if (!(maxSteering > 0.0) || !isFinite(maxSteering)) {
        errors << "maxSteering must be finite and > 0; ";
    }
    if (!(maxSteeringRate > 0.0) || !isFinite(maxSteeringRate)) {
        errors << "maxSteeringRate must be finite and > 0; ";
    }
    return errors.str();
}

LateralMpc::LateralMpc(
    const ErrorLinearModel& continuousModel,
    MpcConfig config)
    : config_(config) {
    const auto error = config_.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }

    const auto discrete =
        discretizeZeroOrderHold(continuousModel, config_.sampleTime);
    ad_ = discrete.a;
    bd_ = discrete.b;
    curvatureD_ = discrete.eCurvature;
    curvatureRateD_ = discrete.eCurvatureRate;
}

MpcStep LateralMpc::update(
    const Vector<6>& state,
    const std::vector<CurvatureSample>& curvaturePreview,
    const std::vector<Vector<6>>& stateReferencePreview) {
    for (const double value : state) {
        if (!isFinite(value)) {
            throw std::invalid_argument("MPC state entries must be finite");
        }
    }

    Matrix<kAugmentedStates, kAugmentedStates> augmentedA{};
    Vector<kAugmentedStates> augmentedB{};
    for (std::size_t row = 0; row < kPlantStates; ++row) {
        for (std::size_t column = 0; column < kPlantStates; ++column) {
            augmentedA[row][column] = ad_[row][column];
        }
        augmentedB[row] = bd_[row];
    }
    augmentedB[6] = 1.0;

    Matrix<kAugmentedStates, kAugmentedStates> valueMatrix{};
    Vector<kAugmentedStates> valueLinear{};
    for (std::size_t i = 0; i < kPlantStates; ++i) {
        valueMatrix[i][i] = config_.terminalWeight[i];
    }

    std::vector<Vector<kAugmentedStates>> gain(config_.horizon);
    std::vector<double> feedforward(config_.horizon, 0.0);

    // Regulate e = x - r. The affine term A r_k - r_{k+1} is the discrete
    // tracking disturbance; curvature uses the same channel.
    for (std::size_t reverse = config_.horizon; reverse-- > 0;) {
        const auto curvature = previewAt(curvaturePreview, reverse);
        const auto reference = stateReferenceAt(stateReferencePreview, reverse);
        const auto nextReference =
            stateReferenceAt(stateReferencePreview, reverse + 1);
        Vector<kAugmentedStates> disturbance{};
        for (std::size_t i = 0; i < kPlantStates; ++i) {
            disturbance[i] =
                curvatureD_[i] * curvature.curvature +
                curvatureRateD_[i] * curvature.curvatureRate -
                nextReference[i];
            for (std::size_t column = 0; column < kPlantStates; ++column) {
                disturbance[i] += ad_[i][column] * reference[column];
            }
        }

        Vector<kAugmentedStates> pTimesB{};
        Vector<kAugmentedStates> pTimesDPlusLinear = valueLinear;
        for (std::size_t row = 0; row < kAugmentedStates; ++row) {
            for (std::size_t column = 0; column < kAugmentedStates; ++column) {
                pTimesB[row] += valueMatrix[row][column] * augmentedB[column];
                pTimesDPlusLinear[row] +=
                    valueMatrix[row][column] * disturbance[column];
            }
        }

        double denominator =
            config_.steeringWeight + config_.steeringRateWeight;
        double affine = 0.0;
        for (std::size_t i = 0; i < kAugmentedStates; ++i) {
            denominator += augmentedB[i] * pTimesB[i];
            affine += augmentedB[i] * pTimesDPlusLinear[i];
        }

        Vector<kAugmentedStates> rowGain{};
        for (std::size_t column = 0; column < kAugmentedStates; ++column) {
            for (std::size_t row = 0; row < kAugmentedStates; ++row) {
                rowGain[column] += pTimesB[row] * augmentedA[row][column];
            }
        }
        rowGain[6] -= config_.steeringRateWeight;
        gain[reverse] = rowGain;
        for (double& value : gain[reverse]) {
            value /= denominator;
        }
        feedforward[reverse] = affine / denominator;

        Matrix<kAugmentedStates, kAugmentedStates> nextMatrix{};
        Vector<kAugmentedStates> nextLinear{};
        for (std::size_t row = 0; row < kAugmentedStates; ++row) {
            for (std::size_t column = 0; column < kAugmentedStates; ++column) {
                for (std::size_t k = 0; k < kAugmentedStates; ++k) {
                    nextMatrix[row][column] +=
                        augmentedA[k][row] * valueMatrix[k][column];
                }
            }
        }
        nextMatrix = matrixProduct(nextMatrix, augmentedA);
        for (std::size_t i = 0; i < kPlantStates; ++i) {
            nextMatrix[i][i] += config_.stateWeight[i];
        }
        nextMatrix[6][6] += config_.steeringRateWeight;
        for (std::size_t row = 0; row < kAugmentedStates; ++row) {
            for (std::size_t column = 0; column < kAugmentedStates; ++column) {
                nextMatrix[row][column] -=
                    rowGain[row] * rowGain[column] / denominator;
            }
        }

        for (std::size_t column = 0; column < kAugmentedStates; ++column) {
            for (std::size_t row = 0; row < kAugmentedStates; ++row) {
                nextLinear[column] +=
                    augmentedA[row][column] * pTimesDPlusLinear[row];
            }
            nextLinear[column] -= rowGain[column] * affine / denominator;
        }
        valueMatrix = nextMatrix;
        valueLinear = nextLinear;
    }

    const auto initialReference =
        stateReferenceAt(stateReferencePreview, 0);
    Vector<kAugmentedStates> predicted{};
    for (std::size_t i = 0; i < kPlantStates; ++i) {
        predicted[i] = state[i] - initialReference[i];
    }
    predicted[6] = previousSteering_;

    MpcStep result;
    result.predictedStates.reserve(config_.horizon + 1);
    result.predictedStates.push_back(state);
    double precedingSteering = previousSteering_;
    for (std::size_t step = 0; step < config_.horizon; ++step) {
        double steering = -feedforward[step];
        for (std::size_t i = 0; i < kAugmentedStates; ++i) {
            steering -= gain[step][i] * predicted[i];
        }
        const double unconstrained = steering;
        const double maximumChange =
            config_.maxSteeringRate * config_.sampleTime;
        steering = std::clamp(
            steering,
            precedingSteering - maximumChange,
            precedingSteering + maximumChange);
        steering = std::clamp(
            steering, -config_.maxSteering, config_.maxSteering);
        if (step == 0) {
            result.unconstrainedSteering = unconstrained;
            result.steering = steering;
        }

        const auto curvature = previewAt(curvaturePreview, step);
        const auto reference = stateReferenceAt(stateReferencePreview, step);
        const auto nextReference =
            stateReferenceAt(stateReferencePreview, step + 1);
        Vector<kAugmentedStates> next{};
        for (std::size_t row = 0; row < kPlantStates; ++row) {
            for (std::size_t column = 0; column < kPlantStates; ++column) {
                next[row] += ad_[row][column] * predicted[column];
            }
            next[row] += bd_[row] * steering +
                         curvatureD_[row] * curvature.curvature +
                         curvatureRateD_[row] * curvature.curvatureRate -
                         nextReference[row];
            for (std::size_t column = 0; column < kPlantStates; ++column) {
                next[row] += ad_[row][column] * reference[column];
            }
        }
        next[6] = steering;
        predicted = next;
        precedingSteering = steering;

        Vector<6> plantState{};
        for (std::size_t i = 0; i < kPlantStates; ++i) {
            plantState[i] = predicted[i] + nextReference[i];
        }
        result.predictedStates.push_back(plantState);
    }
    previousSteering_ = result.steering;
    return result;
}

void LateralMpc::reset(double steering) {
    previousSteering_ =
        std::clamp(steering, -config_.maxSteering, config_.maxSteering);
}

const Matrix<6, 6>& LateralMpc::discreteA() const noexcept {
    return ad_;
}

const Vector<6>& LateralMpc::discreteB() const noexcept {
    return bd_;
}

const Vector<6>& LateralMpc::discreteCurvature() const noexcept {
    return curvatureD_;
}

const Vector<6>& LateralMpc::discreteCurvatureRate() const noexcept {
    return curvatureRateD_;
}

const MpcConfig& LateralMpc::config() const noexcept {
    return config_;
}

}  // namespace truck_model
