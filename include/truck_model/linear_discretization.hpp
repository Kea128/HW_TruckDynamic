#pragma once

#include "truck_model/articulated_vehicle.hpp"

namespace truck_model {

struct DiscreteDynamicModel {
    Matrix<4, 4> a{};
    Vector<4> b{};
};

struct DiscreteErrorModel {
    Matrix<6, 6> a{};
    Vector<6> b{};
    Vector<6> eCurvature{};
    Vector<6> eCurvatureRate{};
};

// Exact zero-order-hold discretization using an augmented matrix exponential.
[[nodiscard]] DiscreteDynamicModel discretizeZeroOrderHold(
    const DynamicLinearModel& continuous,
    double sampleTime);

[[nodiscard]] DiscreteErrorModel discretizeZeroOrderHold(
    const ErrorLinearModel& continuous,
    double sampleTime);

}  // namespace truck_model
