#include "truck_model/articulated_vehicle.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1.0e-7;

void expectNear(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > kTolerance) {
        std::cerr << message << ": expected " << expected << ", got " << actual
                  << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void expectFinite(double value, const char* message) {
    if (!std::isfinite(value)) {
        std::cerr << message << ": value is not finite\n";
        std::exit(EXIT_FAILURE);
    }
}

truck_model::Parameters parameters() {
    truck_model::Parameters p;
    p.m1 = 8000.0;
    p.iz1 = 25000.0;
    p.a1 = 1.5;
    p.b1 = 2.5;
    p.c1f = 220000.0;
    p.c1r = 300000.0;
    p.m2 = 18000.0;
    p.iz2 = 140000.0;
    p.a2 = 4.0;
    p.b2 = 3.0;
    p.c2r = 500000.0;
    p.d1 = p.b1;
    p.vx = 15.0;
    return p;
}

void testNonlinearKinematics() {
    const auto p = parameters();
    const truck_model::KinematicState state{3.0, -2.0, 0.0, 0.0};
    const auto derivative = truck_model::nonlinearKinematics(p, state, 0.0);
    expectNear(derivative.xhDot, p.vx, "straight hitch x velocity");
    expectNear(derivative.yhDot, 0.0, "straight hitch y velocity");
    expectNear(derivative.theta1Dot, 0.0, "straight truck yaw rate");
    expectNear(derivative.theta2Dot, 0.0, "straight trailer yaw rate");

    const auto poses = truck_model::bodyPoses(p, state);
    expectNear(poses.x1, state.xh + p.d1, "truck CG geometry");
    expectNear(poses.x2, state.xh - p.a2, "trailer CG geometry");

    auto offAxle = p;
    offAxle.d1 = 1.8;
    const double steering = 0.1;
    const auto offAxleDerivative =
        truck_model::nonlinearKinematics(offAxle, state, steering);
    const double r1 =
        offAxle.vx * std::tan(steering) / (offAxle.a1 + offAxle.b1);
    const double hitchVy = (offAxle.b1 - offAxle.d1) * r1;
    expectNear(
        offAxleDerivative.yhDot,
        hitchVy,
        "off-axle hitch lateral velocity");
    expectNear(
        offAxleDerivative.theta2Dot,
        hitchVy / (offAxle.a2 + offAxle.b2),
        "off-axle trailer yaw rate");
}

void testConstraintReconstructionAndDerivative() {
    const auto p = parameters();
    const auto model = truck_model::buildDynamicModel(p);
    const truck_model::Vector<4> z{0.7, 0.08, 0.03, 0.04};
    const double steering = 0.015;

    const auto q = truck_model::multiply(model.reconstruction, z);
    expectNear(
        truck_model::hitchConstraintResidual(p, q, z[3]),
        0.0,
        "position-level velocity constraint");
    expectNear(
        q[2],
        truck_model::reconstructTrailerLateralVelocity(p, z),
        "trailer lateral velocity reconstruction");

    auto zDot = truck_model::multiply(model.a, z);
    for (std::size_t i = 0; i < 4; ++i) {
        zDot[i] += model.b[i] * steering;
    }
    const truck_model::Vector<4> qDot{
        zDot[0],
        zDot[1],
        zDot[0] - p.d1 * zDot[1] - p.a2 * zDot[2] +
            p.vx * zDot[3],
        zDot[2]};
    const double differentiatedConstraint =
        qDot[0] - p.d1 * qDot[1] - qDot[2] - p.a2 * qDot[3] +
        p.vx * (z[1] - z[2]);
    expectNear(
        differentiatedConstraint,
        0.0,
        "acceleration-level hitch constraint");

    const double alpha1f = steering - (q[0] + p.a1 * q[1]) / p.vx;
    const double alpha1r = -(q[0] - p.b1 * q[1]) / p.vx;
    const double alpha2r = -(q[2] - p.b2 * q[3]) / p.vx;
    const double f1f = p.c1f * alpha1f;
    const double f1r = p.c1r * alpha1r;
    const double f2r = p.c2r * alpha2r;
    double hitch = model.hitchInput * steering;
    for (std::size_t i = 0; i < 4; ++i) {
        hitch += model.hitchState[i] * z[i];
    }
    expectNear(
        p.m1 * (qDot[0] + p.vx * q[1]),
        f1f + f1r + hitch,
        "truck lateral force balance");
    expectNear(
        p.iz1 * qDot[1],
        p.a1 * f1f - p.b1 * f1r - p.d1 * hitch,
        "truck yaw moment balance");
    expectNear(
        p.m2 * (qDot[2] + p.vx * q[3]),
        f2r - hitch,
        "trailer lateral force balance");
    expectNear(
        p.iz2 * qDot[3],
        -p.b2 * f2r - p.a2 * hitch,
        "trailer yaw moment balance");

    for (const auto& row : model.a) {
        for (const double value : row) {
            expectFinite(value, "dynamic A");
        }
    }
}

void testErrorModelTransformation() {
    const auto p = parameters();
    const auto dynamics = truck_model::buildDynamicModel(p);
    const auto errors = truck_model::buildErrorModel(p);
    const truck_model::Vector<6> x{0.4, -0.2, 0.03, 0.01, 0.05, -0.02};
    const double curvature = 0.008;
    const double curvatureRate = -0.0003;
    const double steering = 0.025;

    auto z = truck_model::multiply(errors.stateToPhysical, x);
    for (std::size_t i = 0; i < 4; ++i) {
        z[i] += errors.curvatureToPhysical[i] * curvature;
    }
    auto expectedZDot = truck_model::multiply(dynamics.a, z);
    for (std::size_t i = 0; i < 4; ++i) {
        expectedZDot[i] += dynamics.b[i] * steering;
    }

    auto xDot = truck_model::multiply(errors.a, x);
    for (std::size_t i = 0; i < 6; ++i) {
        xDot[i] += errors.b[i] * steering +
                   errors.eCurvature[i] * curvature +
                   errors.eCurvatureRate[i] * curvatureRate;
    }
    auto transformedZDot = truck_model::multiply(errors.stateToPhysical, xDot);
    for (std::size_t i = 0; i < 4; ++i) {
        transformedZDot[i] +=
            errors.curvatureToPhysical[i] * curvatureRate;
        expectNear(
            transformedZDot[i],
            expectedZDot[i],
            "error-to-physical derivative transformation");
    }
}

void testInvalidSpeedRejected() {
    auto p = parameters();
    p.vx = 0.0;
    bool rejected = false;
    try {
        (void)truck_model::buildDynamicModel(p);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    if (!rejected) {
        std::cerr << "zero speed must be rejected\n";
        std::exit(EXIT_FAILURE);
    }
}

}  // namespace

int main() {
    testNonlinearKinematics();
    testConstraintReconstructionAndDerivative();
    testErrorModelTransformation();
    testInvalidSpeedRejected();
    std::cout << "All articulated vehicle model tests passed.\n";
    return EXIT_SUCCESS;
}
