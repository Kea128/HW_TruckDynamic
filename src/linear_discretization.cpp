#include "truck_model/linear_discretization.hpp"

#include "truck_model/matrix_exponential.hpp"

#include <cmath>
#include <stdexcept>

namespace truck_model {
namespace {

void requireSampleTime(double sampleTime) {
    if (!(sampleTime > 0.0) || !std::isfinite(sampleTime)) {
        throw std::invalid_argument("sampleTime must be finite and > 0");
    }
}

}  // namespace

DiscreteDynamicModel discretizeZeroOrderHold(
    const DynamicLinearModel& continuous,
    double sampleTime) {
    requireSampleTime(sampleTime);
    Matrix<5, 5> generator{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            generator[row][column] =
                continuous.a[row][column] * sampleTime;
        }
        generator[row][4] = continuous.b[row] * sampleTime;
    }
    const auto zoh = matrixExponential(generator);
    DiscreteDynamicModel result;
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            result.a[row][column] = zoh[row][column];
        }
        result.b[row] = zoh[row][4];
    }
    return result;
}

DiscreteErrorModel discretizeZeroOrderHold(
    const ErrorLinearModel& continuous,
    double sampleTime) {
    requireSampleTime(sampleTime);
    Matrix<9, 9> generator{};
    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            generator[row][column] =
                continuous.a[row][column] * sampleTime;
        }
        generator[row][6] = continuous.b[row] * sampleTime;
        generator[row][7] =
            continuous.eCurvature[row] * sampleTime;
        generator[row][8] =
            continuous.eCurvatureRate[row] * sampleTime;
    }
    const auto zoh = matrixExponential(generator);
    DiscreteErrorModel result;
    for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            result.a[row][column] = zoh[row][column];
        }
        result.b[row] = zoh[row][6];
        result.eCurvature[row] = zoh[row][7];
        result.eCurvatureRate[row] = zoh[row][8];
    }
    return result;
}

}  // namespace truck_model
