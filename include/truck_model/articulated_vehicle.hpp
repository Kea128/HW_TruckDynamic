#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace truck_model {

template <std::size_t Rows, std::size_t Cols>
using Matrix = std::array<std::array<double, Cols>, Rows>;

template <std::size_t Size>
using Vector = std::array<double, Size>;

struct Parameters {
    double m1{};
    double iz1{};
    double a1{};
    double b1{};
    double c1f{};
    double c1r{};

    double m2{};
    double iz2{};
    double a2{};
    double b2{};
    double c2r{};

    // Truck CG to hitch. The requested rear-axle hitch case is d1 = b1.
    double d1{};
    double vx{};

    [[nodiscard]] std::string validationError() const;
};

// Minimal nonlinear kinematic state. (xh, yh) is the hitch center.
// It is also the truck rear-axle center when d1 = b1.
struct KinematicState {
    double xh{};
    double yh{};
    double theta1{};
    double theta2{};
};

struct KinematicDerivative {
    double xhDot{};
    double yhDot{};
    double theta1Dot{};
    double theta2Dot{};
};

struct BodyPoses {
    double x1{};
    double y1{};
    double theta1{};
    double x2{};
    double y2{};
    double theta2{};
};

// z = [vy1, r1, r2, theta12]^T. vy2 is reconstructed by the hitch constraint.
struct DynamicLinearModel {
    Matrix<4, 4> a{};
    Vector<4> b{};

    // q = [vy1, r1, vy2, r2]^T = reconstruction * z.
    Matrix<4, 4> reconstruction{};

    // H = hitchState * z + hitchInput * delta.
    // H is the lateral force exerted by the trailer on the truck.
    Vector<4> hitchState{};
    double hitchInput{};
};

// xc = [ey, eyDot, ePhi, ePhiDot, theta12, theta12Dot]^T.
// xcDot = A xc + B delta + Ecurvature*kappa + EcurvatureRate*kappaDot.
struct ErrorLinearModel {
    Matrix<6, 6> a{};
    Vector<6> b{};
    Vector<6> eCurvature{};
    Vector<6> eCurvatureRate{};

    // z = stateToPhysical * xc + curvatureToPhysical * kappa.
    Matrix<4, 6> stateToPhysical{};
    Vector<4> curvatureToPhysical{};
};

[[nodiscard]] KinematicDerivative nonlinearKinematics(
    const Parameters& parameters,
    const KinematicState& state,
    double steering);

[[nodiscard]] BodyPoses bodyPoses(
    const Parameters& parameters,
    const KinematicState& state);

[[nodiscard]] DynamicLinearModel buildDynamicModel(
    const Parameters& parameters);

[[nodiscard]] ErrorLinearModel buildErrorModel(
    const Parameters& parameters);

[[nodiscard]] double reconstructTrailerLateralVelocity(
    const Parameters& parameters,
    const Vector<4>& minimalState);

[[nodiscard]] double hitchConstraintResidual(
    const Parameters& parameters,
    const Vector<4>& redundantVelocity,
    double articulationAngle);

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] Vector<Rows> multiply(
    const Matrix<Rows, Cols>& matrix,
    const Vector<Cols>& vector) {
    Vector<Rows> result{};
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t column = 0; column < Cols; ++column) {
            result[row] += matrix[row][column] * vector[column];
        }
    }
    return result;
}

}  // namespace truck_model
