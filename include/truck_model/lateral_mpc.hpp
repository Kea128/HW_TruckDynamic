#pragma once

#include "truck_model/articulated_vehicle.hpp"

#include <array>
#include <cstddef>
#include <vector>

namespace truck_model {

// curvatureRate is the time derivative dRho/dt [1/(m*s)], not the arc-length
// derivative dKappa/ds [1/m^2] stored in ReferencePathPoint. Convert with
// dRho/dt = speed * dKappa/ds; see docs/1 section 1.4.4.
struct CurvatureSample {
    double curvature{};
    double curvatureRate{};
};

struct MpcConfig {
    double sampleTime{0.05};
    std::size_t horizon{40};

    // Weights for [ey, eyDot, ePsi, ePsiDot, articulation, articulationRate].
    Vector<6> stateWeight{18.0, 0.8, 35.0, 1.5, 20.0, 1.0};
    Vector<6> terminalWeight{45.0, 2.0, 80.0, 3.0, 45.0, 2.0};
    double steeringWeight{30.0};
    double steeringRateWeight{10000.0};

    double maxSteering{0.35};
    double maxSteeringRate{0.45};

    [[nodiscard]] std::string validationError() const;
};

struct MpcStep {
    double steering{};
    double unconstrainedSteering{};
    std::vector<Vector<6>> predictedStates;
};

// Finite-horizon linear MPC for the six-state error model. The quadratic
// program is solved by backward dynamic programming. Steering magnitude and
// rate limits are applied to the receding-horizon command.
class LateralMpc {
public:
    LateralMpc(const ErrorLinearModel& continuousModel, MpcConfig config = {});

    // Empty stateReferencePreview regulates to zero (path-tracking default).
    // Otherwise the DP tracks (x - r_k) over the horizon.
    [[nodiscard]] MpcStep update(
        const Vector<6>& state,
        const std::vector<CurvatureSample>& curvaturePreview,
        const std::vector<Vector<6>>& stateReferencePreview = {});

    void reset(double steering = 0.0);

    [[nodiscard]] const Matrix<6, 6>& discreteA() const noexcept;
    [[nodiscard]] const Vector<6>& discreteB() const noexcept;
    [[nodiscard]] const Vector<6>& discreteCurvature() const noexcept;
    [[nodiscard]] const Vector<6>& discreteCurvatureRate() const noexcept;
    [[nodiscard]] const MpcConfig& config() const noexcept;

private:
    Matrix<6, 6> ad_{};
    Vector<6> bd_{};
    Vector<6> curvatureD_{};
    Vector<6> curvatureRateD_{};
    MpcConfig config_{};
    double previousSteering_{};
};

}  // namespace truck_model
