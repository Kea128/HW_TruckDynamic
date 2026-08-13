#include "truck_model/articulated_vehicle.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace truck_model {
namespace {

void requireValid(const Parameters& parameters) {
    const auto error = parameters.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
}

}  // namespace

std::string Parameters::validationError() const {
    std::ostringstream errors;
    const auto requirePositive = [&errors](double value, const char* name) {
        if (!(value > 0.0) || !std::isfinite(value)) {
            errors << name << " must be finite and > 0; ";
        }
    };
    const auto requireNonnegative = [&errors](double value, const char* name) {
        if (value < 0.0 || !std::isfinite(value)) {
            errors << name << " must be finite and >= 0; ";
        }
    };

    requirePositive(m1, "m1");
    requirePositive(iz1, "iz1");
    requirePositive(a1, "a1");
    requirePositive(b1, "b1");
    requirePositive(c1f, "c1f");
    requirePositive(c1r, "c1r");
    requirePositive(m2, "m2");
    requirePositive(iz2, "iz2");
    requirePositive(a2, "a2");
    requirePositive(b2, "b2");
    requirePositive(c2r, "c2r");
    requireNonnegative(d1, "d1");
    requirePositive(vx, "vx");
    return errors.str();
}

KinematicDerivative nonlinearKinematics(
    const Parameters& parameters,
    const KinematicState& state,
    double steering) {
    requireValid(parameters);

    const double articulation = state.theta1 - state.theta2;
    const double truckWheelbase = parameters.a1 + parameters.b1;
    const double trailerWheelbase = parameters.a2 + parameters.b2;
    const double rearAxleToHitch = parameters.b1 - parameters.d1;
    const double truckYawRate =
        parameters.vx * std::tan(steering) / truckWheelbase;
    const double hitchLateralVelocity = rearAxleToHitch * truckYawRate;

    KinematicDerivative derivative;
    derivative.xhDot =
        parameters.vx * std::cos(state.theta1) -
        hitchLateralVelocity * std::sin(state.theta1);
    derivative.yhDot =
        parameters.vx * std::sin(state.theta1) +
        hitchLateralVelocity * std::cos(state.theta1);
    derivative.theta1Dot = truckYawRate;
    derivative.theta2Dot =
        (parameters.vx * std::sin(articulation) +
         hitchLateralVelocity * std::cos(articulation)) /
        trailerWheelbase;
    return derivative;
}

BodyPoses bodyPoses(
    const Parameters& parameters,
    const KinematicState& state) {
    requireValid(parameters);

    BodyPoses poses;
    poses.x1 = state.xh + parameters.d1 * std::cos(state.theta1);
    poses.y1 = state.yh + parameters.d1 * std::sin(state.theta1);
    poses.theta1 = state.theta1;
    poses.x2 = state.xh - parameters.a2 * std::cos(state.theta2);
    poses.y2 = state.yh - parameters.a2 * std::sin(state.theta2);
    poses.theta2 = state.theta2;
    return poses;
}

DynamicLinearModel buildDynamicModel(const Parameters& p) {
    requireValid(p);

    // Redundant velocity q = [vy1, r1, vy2, r2].
    const Vector<4> inverseMass{
        1.0 / p.m1, 1.0 / p.iz1, 1.0 / p.m2, 1.0 / p.iz2};
    const Vector<4> constraintGradient{1.0, -p.d1, -1.0, -p.a2};
    Vector<4> weightedGradient{};
    double schur = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        weightedGradient[i] = inverseMass[i] * constraintGradient[i];
        schur += constraintGradient[i] * weightedGradient[i];
    }

    Matrix<4, 4> projection{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            const double diagonal = row == column ? inverseMass[row] : 0.0;
            projection[row][column] =
                diagonal -
                weightedGradient[row] * weightedGradient[column] / schur;
        }
    }

    // f = L q + fDelta*delta before enforcing the acceleration constraint.
    Matrix<4, 4> l{};
    l[0][0] = -(p.c1f + p.c1r) / p.vx;
    l[0][1] =
        (-p.a1 * p.c1f + p.b1 * p.c1r) / p.vx - p.m1 * p.vx;
    l[1][0] = (-p.a1 * p.c1f + p.b1 * p.c1r) / p.vx;
    l[1][1] =
        -(p.a1 * p.a1 * p.c1f + p.b1 * p.b1 * p.c1r) / p.vx;
    l[2][2] = -p.c2r / p.vx;
    l[2][3] = p.b2 * p.c2r / p.vx - p.m2 * p.vx;
    l[3][2] = p.b2 * p.c2r / p.vx;
    l[3][3] = -p.b2 * p.b2 * p.c2r / p.vx;
    const Vector<4> fDelta{p.c1f, p.a1 * p.c1f, 0.0, 0.0};

    // g^T qDot = s^T q is the time derivative of the hitch constraint.
    const Vector<4> s{0.0, -p.vx, 0.0, p.vx};
    Matrix<4, 4> aRedundant{};
    Vector<4> bRedundant{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t k = 0; k < 4; ++k) {
                aRedundant[row][column] += projection[row][k] * l[k][column];
            }
            aRedundant[row][column] +=
                weightedGradient[row] * s[column] / schur;
        }
        for (std::size_t k = 0; k < 4; ++k) {
            bRedundant[row] += projection[row][k] * fDelta[k];
        }
    }

    DynamicLinearModel model;
    // q = R z, z = [vy1, r1, r2, theta12].
    model.reconstruction[0] = {1.0, 0.0, 0.0, 0.0};
    model.reconstruction[1] = {0.0, 1.0, 0.0, 0.0};
    model.reconstruction[2] = {1.0, -p.d1, -p.a2, p.vx};
    model.reconstruction[3] = {0.0, 0.0, 1.0, 0.0};

    const std::array<std::size_t, 3> selectedRows{0, 1, 3};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            for (std::size_t k = 0; k < 4; ++k) {
                model.a[row][column] +=
                    aRedundant[selectedRows[row]][k] *
                    model.reconstruction[k][column];
            }
        }
        model.b[row] = bRedundant[selectedRows[row]];
    }
    model.a[3] = {0.0, 1.0, -1.0, 0.0};

    // H = (s^T q - g^T M^-1(Lq + fDelta*delta)) / schur.
    Vector<4> hitchRedundant{};
    for (std::size_t column = 0; column < 4; ++column) {
        hitchRedundant[column] = s[column];
        for (std::size_t row = 0; row < 4; ++row) {
            hitchRedundant[column] -=
                constraintGradient[row] * inverseMass[row] * l[row][column];
        }
        hitchRedundant[column] /= schur;
    }
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t k = 0; k < 4; ++k) {
            model.hitchState[column] +=
                hitchRedundant[k] * model.reconstruction[k][column];
        }
    }
    for (std::size_t row = 0; row < 4; ++row) {
        model.hitchInput -=
            constraintGradient[row] * inverseMass[row] * fDelta[row] / schur;
    }
    return model;
}

ErrorLinearModel buildErrorModel(const Parameters& p) {
    const DynamicLinearModel dynamics = buildDynamicModel(p);
    ErrorLinearModel model;

    // z = T xc + tk*kappa.
    model.stateToPhysical[0] = {0.0, 1.0, -p.vx, 0.0, 0.0, 0.0};
    model.stateToPhysical[1] = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    model.stateToPhysical[2] = {0.0, 0.0, 0.0, 1.0, 0.0, -1.0};
    model.stateToPhysical[3] = {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    model.curvatureToPhysical = {0.0, p.vx, p.vx, 0.0};

    Matrix<4, 6> physicalStateDynamics{};
    Vector<4> physicalCurvatureDynamics{};
    for (std::size_t row = 0; row < 4; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
            for (std::size_t k = 0; k < 4; ++k) {
                physicalStateDynamics[row][column] +=
                    dynamics.a[row][k] * model.stateToPhysical[k][column];
            }
        }
        for (std::size_t k = 0; k < 4; ++k) {
            physicalCurvatureDynamics[row] +=
                dynamics.a[row][k] * model.curvatureToPhysical[k];
        }
    }

    model.a[0][1] = 1.0;
    model.a[2][3] = 1.0;
    model.a[4][5] = 1.0;
    for (std::size_t column = 0; column < 6; ++column) {
        model.a[1][column] = physicalStateDynamics[0][column];
        model.a[3][column] = physicalStateDynamics[1][column];
        model.a[5][column] =
            physicalStateDynamics[1][column] -
            physicalStateDynamics[2][column];
    }
    model.a[1][3] += p.vx;

    model.b = {
        0.0,
        dynamics.b[0],
        0.0,
        dynamics.b[1],
        0.0,
        dynamics.b[1] - dynamics.b[2]};
    model.eCurvature = {
        0.0,
        physicalCurvatureDynamics[0],
        0.0,
        physicalCurvatureDynamics[1],
        0.0,
        physicalCurvatureDynamics[1] - physicalCurvatureDynamics[2]};
    model.eCurvatureRate = {0.0, 0.0, 0.0, -p.vx, 0.0, 0.0};
    return model;
}

double reconstructTrailerLateralVelocity(
    const Parameters& p,
    const Vector<4>& z) {
    requireValid(p);
    return z[0] - p.d1 * z[1] - p.a2 * z[2] + p.vx * z[3];
}

double hitchConstraintResidual(
    const Parameters& p,
    const Vector<4>& q,
    double articulationAngle) {
    requireValid(p);
    return q[0] - p.d1 * q[1] - q[2] - p.a2 * q[3] +
           p.vx * articulationAngle;
}

}  // namespace truck_model
