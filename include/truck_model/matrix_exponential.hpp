#pragma once

#include "truck_model/articulated_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace truck_model {

template <std::size_t Size>
[[nodiscard]] inline Matrix<Size, Size> identityMatrix() {
    Matrix<Size, Size> result{};
    for (std::size_t i = 0; i < Size; ++i) {
        result[i][i] = 1.0;
    }
    return result;
}

template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
[[nodiscard]] inline Matrix<Rows, Cols> matrixProduct(
    const Matrix<Rows, Inner>& lhs,
    const Matrix<Inner, Cols>& rhs) {
    Matrix<Rows, Cols> result{};
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t k = 0; k < Inner; ++k) {
            for (std::size_t column = 0; column < Cols; ++column) {
                result[row][column] += lhs[row][k] * rhs[k][column];
            }
        }
    }
    return result;
}

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] inline Matrix<Cols, Rows> transposed(
    const Matrix<Rows, Cols>& value) {
    Matrix<Cols, Rows> result{};
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t column = 0; column < Cols; ++column) {
            result[column][row] = value[row][column];
        }
    }
    return result;
}

// Matrix exponential by Taylor series with scaling and squaring. The series is
// truncated once the incremental term is negligible; scaling keeps the argument
// norm below 0.5 so the truncation stays accurate.
template <std::size_t Size>
[[nodiscard]] inline Matrix<Size, Size> matrixExponential(
    Matrix<Size, Size> value) {
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

    auto result = identityMatrix<Size>();
    auto term = identityMatrix<Size>();
    for (std::size_t order = 1; order <= 40; ++order) {
        term = matrixProduct(term, value);
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
        result = matrixProduct(result, result);
    }
    return result;
}

// Van Loan discretization of xDot = A x + w, where w has continuous-time power
// spectral density continuousNoise. Returns the discrete transition matrix and
// the discrete process-noise covariance over sampleTime.
//
// The construction exponentiates
//     M = [[-A, Qc], [0, A^T]] * dt
// and reads
//     transition = (M22)^T,  discreteNoise = transition * M12.
//
// Unlike a diagonal Euler approximation this keeps the cross-covariance that
// integrating noise-driven states inject into the states they drive.
template <std::size_t Size>
struct VanLoanDiscretization {
    Matrix<Size, Size> transition{};
    Matrix<Size, Size> processNoise{};
};

template <std::size_t Size>
[[nodiscard]] inline VanLoanDiscretization<Size> discretizeVanLoan(
    const Matrix<Size, Size>& continuousState,
    const Matrix<Size, Size>& continuousNoise,
    double sampleTime) {
    constexpr std::size_t kDouble = 2 * Size;
    Matrix<kDouble, kDouble> generator{};
    for (std::size_t row = 0; row < Size; ++row) {
        for (std::size_t column = 0; column < Size; ++column) {
            generator[row][column] = -continuousState[row][column] * sampleTime;
            generator[row][Size + column] =
                continuousNoise[row][column] * sampleTime;
            generator[Size + row][Size + column] =
                continuousState[column][row] * sampleTime;
        }
    }
    const auto expanded = matrixExponential(generator);

    VanLoanDiscretization<Size> result;
    Matrix<Size, Size> upperRight{};
    for (std::size_t row = 0; row < Size; ++row) {
        for (std::size_t column = 0; column < Size; ++column) {
            result.transition[row][column] =
                expanded[Size + column][Size + row];
            upperRight[row][column] = expanded[row][Size + column];
        }
    }
    result.processNoise = matrixProduct(result.transition, upperRight);

    // The exact result is symmetric; average away the asymmetry introduced by
    // the truncated series so downstream Cholesky-free updates stay stable.
    for (std::size_t row = 0; row < Size; ++row) {
        for (std::size_t column = row + 1; column < Size; ++column) {
            const double mean = 0.5 * (result.processNoise[row][column] +
                                       result.processNoise[column][row]);
            result.processNoise[row][column] = mean;
            result.processNoise[column][row] = mean;
        }
    }
    return result;
}

}  // namespace truck_model
