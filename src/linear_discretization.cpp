#include "truck_model/linear_discretization.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace truck_model {
namespace {

template <std::size_t Size>
Matrix<Size, Size> identity() {
    Matrix<Size, Size> result{};
    for (std::size_t i = 0; i < Size; ++i) {
        result[i][i] = 1.0;
    }
    return result;
}

template <std::size_t Size>
Matrix<Size, Size> product(
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

template <std::size_t Size>
Matrix<Size, Size> exponential(Matrix<Size, Size> value) {
    double norm = 0.0;
    for (const auto& row : value) {
        double rowSum = 0.0;
        for (const double element : row) {
            rowSum += std::abs(element);
        }
        norm = std::max(norm, rowSum);
    }
    int squarings = 0;
    if (norm > 0.5) {
        squarings = static_cast<int>(std::ceil(std::log2(norm / 0.5)));
        const double scale = std::ldexp(1.0, squarings);
        for (auto& row : value) {
            for (double& element : row) {
                element /= scale;
            }
        }
    }

    auto result = identity<Size>();
    auto term = identity<Size>();
    for (std::size_t order = 1; order <= 40; ++order) {
        term = product(term, value);
        double largest = 0.0;
        for (auto& row : term) {
            for (double& element : row) {
                element /= static_cast<double>(order);
                largest = std::max(largest, std::abs(element));
            }
        }
        for (std::size_t row = 0; row < Size; ++row) {
            for (std::size_t column = 0; column < Size; ++column) {
                result[row][column] += term[row][column];
            }
        }
        if (largest < 1.0e-15) {
            break;
        }
    }
    for (int i = 0; i < squarings; ++i) {
        result = product(result, result);
    }
    return result;
}

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
    const auto zoh = exponential(generator);
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
    const auto zoh = exponential(generator);
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
