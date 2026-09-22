#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/lateral_mpc.hpp"
#include "truck_model/linear_discretization.hpp"
#include "truck_model/matrix_exponential.hpp"
#include "truck_model/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

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

void testMpcStabilizesStraightPath() {
    const auto p = parameters();
    const auto continuous = truck_model::buildErrorModel(p);
    truck_model::MpcConfig config;
    config.horizon = 35;
    truck_model::LateralMpc controller(continuous, config);

    truck_model::Vector<6> state{1.0, 0.0, 0.08, 0.0, 0.03, 0.0};
    double previousSteering = 0.0;
    for (std::size_t step = 0; step < 240; ++step) {
        const auto command = controller.update(state, {});
        expectFinite(command.steering, "MPC steering");
        if (std::abs(command.steering) > config.maxSteering + kTolerance) {
            std::cerr << "MPC steering limit violated\n";
            std::exit(EXIT_FAILURE);
        }
        const double steeringChange = std::abs(command.steering - previousSteering);
        if (steeringChange >
            config.maxSteeringRate * config.sampleTime + kTolerance) {
            std::cerr << "MPC steering rate limit violated\n";
            std::exit(EXIT_FAILURE);
        }

        auto next = truck_model::multiply(controller.discreteA(), state);
        for (std::size_t i = 0; i < state.size(); ++i) {
            next[i] += controller.discreteB()[i] * command.steering;
        }
        state = next;
        previousSteering = command.steering;
    }

    if (std::abs(state[0]) > 0.08 || std::abs(state[2]) > 0.015 ||
        std::abs(state[4]) > 0.015) {
        std::cerr << "MPC failed to stabilize straight-path errors: "
                  << state[0] << ", " << state[2] << ", " << state[4] << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testMpcTracksConstantArticulationReference() {
    const auto p = parameters();
    const auto continuous = truck_model::buildErrorModel(p);
    truck_model::MpcConfig config;
    config.horizon = 35;
    config.stateWeight = {0.0, 0.0, 0.0, 0.0, 40.0, 4.0};
    config.terminalWeight = {0.0, 0.0, 0.0, 0.0, 100.0, 10.0};
    config.steeringWeight = 4.0;
    config.steeringRateWeight = 20.0;
    truck_model::LateralMpc controller(continuous, config);

    truck_model::Vector<6> state{};
    truck_model::Vector<6> reference{};
    reference[4] = 0.1;
    std::vector<truck_model::Vector<6>> preview(config.horizon + 1, reference);
    const auto command = controller.update(state, {}, preview);
    expectFinite(command.steering, "MPC tracking steering");
    if (command.predictedStates.size() < 2) {
        std::cerr << "MPC tracking produced no prediction\n";
        std::exit(EXIT_FAILURE);
    }
    const double predictedPhi = command.predictedStates.back()[4];
    if (std::abs(predictedPhi - 0.1) >= std::abs(state[4] - 0.1) - 1.0e-6) {
        std::cerr << "MPC prediction did not move toward articulation "
                  << "reference: " << predictedPhi << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testReferencePathFromDrawnWaypoints() {
    std::vector<truck_model::Point2d> waypoints;
    for (std::size_t i = 0; i <= 40; ++i) {
        const double x = static_cast<double>(i);
        waypoints.push_back({x, 2.0 * std::sin(0.08 * x)});
    }
    const auto path = truck_model::ReferencePath::fromWaypoints(waypoints);
    if (path.length() < 39.0 || path.points().size() < 70) {
        std::cerr << "drawn reference path was not uniformly resampled\n";
        std::exit(EXIT_FAILURE);
    }
    double previousS = -1.0;
    for (const auto& point : path.points()) {
        if (!(point.s > previousS) || !std::isfinite(point.heading) ||
            !std::isfinite(point.curvature) ||
            !std::isfinite(point.curvatureDerivative)) {
            std::cerr << "reference path contains invalid differential data\n";
            std::exit(EXIT_FAILURE);
        }
        previousS = point.s;
    }
    const auto middle = path.sample(0.5 * path.length());
    expectFinite(middle.x, "interpolated path x");
    expectFinite(middle.curvature, "interpolated path curvature");

    bool shortPathRejected = false;
    try {
        (void)truck_model::ReferencePath::fromWaypoints({{0.0, 0.0}, {0.1, 0.0}});
    } catch (const std::invalid_argument&) {
        shortPathRejected = true;
    }
    if (!shortPathRejected) {
        std::cerr << "too-short drawn path must be rejected\n";
        std::exit(EXIT_FAILURE);
    }
}

void testPurePursuitPathSmoothing() {
    std::vector<truck_model::Point2d> raw;
    for (std::size_t i = 0; i <= 80; ++i) {
        const double x = static_cast<double>(i);
        const double noise = i % 2 == 0 ? 0.28 : -0.28;
        raw.push_back({x, 3.0 * std::sin(0.055 * x) + noise});
    }
    std::vector<truck_model::Point2d> smoothed;
    try {
        smoothed = truck_model::smoothWaypointsPurePursuit(raw);
    } catch (const std::exception& error) {
        std::cerr << "Pure Pursuit smoothing failed: " << error.what() << '\n';
        std::exit(EXIT_FAILURE);
    }
    if (smoothed.size() < raw.size() ||
        std::hypot(
            smoothed.front().x - raw.front().x,
            smoothed.front().y - raw.front().y) > 1.0e-9 ||
        std::hypot(
            smoothed.back().x - raw.back().x,
            smoothed.back().y - raw.back().y) > 10.0) {
        std::cerr << "Pure Pursuit smoother did not preserve path coverage\n";
        std::exit(EXIT_FAILURE);
    }
    truck_model::PathBuildConfig build;
    build.smoothingPasses = 0;
    const auto path =
        truck_model::ReferencePath::fromWaypoints(smoothed, build);
    double maximumCurvature = 0.0;
    for (const auto& point : path.points()) {
        maximumCurvature =
            std::max(maximumCurvature, std::abs(point.curvature));
        if (!std::isfinite(point.curvature)) {
            std::cerr << "Pure Pursuit output contains invalid curvature\n";
            std::exit(EXIT_FAILURE);
        }
    }
    if (maximumCurvature > 0.075) {
        std::cerr << "Pure Pursuit output curvature is too high: "
                  << maximumCurvature << '\n';
        std::exit(EXIT_FAILURE);
    }
}

double wrapAngle(double angle) {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

void testKinematicTrailerYawRateMatchesK5() {
    const auto p = parameters();
    const double speed = p.vx;
    const double articulation = 0.25;
    expectNear(
        truck_model::kinematicTrailerYawRate(p, speed, 0.0, articulation),
        speed * std::sin(articulation) / (p.a2 + p.b2),
        "on-axle trailer yaw rate");

    auto offAxle = p;
    offAxle.d1 = 1.8;
    const double steering = 0.12;
    const truck_model::KinematicState state{0.0, 0.0, articulation, 0.0};
    const auto derivative =
        truck_model::nonlinearKinematics(offAxle, state, steering);
    expectNear(
        truck_model::kinematicTrailerYawRate(
            offAxle, offAxle.vx, derivative.theta1Dot, articulation),
        derivative.theta2Dot,
        "off-axle K5 trailer yaw rate");
}

void testDelayedEkfCompensatesLidarLatency() {
    const auto p = parameters();
    truck_model::ArticulationEstimator estimator(p);
    const double dt = 0.02;
    const double speed = 12.0;
    const double r1 = 0.08;
    double truth = 0.15;
    estimator.reset(0.0, truth);

    double estimateSse = 0.0;
    double delayedSse = 0.0;
    int scoreCount = 0;
    double lastDelivered = truth;
    for (int step = 1; step <= 150; ++step) {
        const double time = dt * static_cast<double>(step);
        const double r2 =
            truck_model::kinematicTrailerYawRate(p, speed, r1, truth);
        truth = wrapAngle(truth + dt * (r1 - r2));

        truck_model::ArticulationInputs inputs;
        inputs.time = time;
        inputs.truckYawRate = r1;
        inputs.speed = speed;
        estimator.predict(inputs);

        if (step % 5 == 0) {
            const double stamp = time - 0.20;
            if (stamp >= 0.0) {
                double delayedTruth = 0.15;
                for (int past = 1; past <= step - 10; ++past) {
                    const double pastR2 =
                        truck_model::kinematicTrailerYawRate(
                            p, speed, r1, delayedTruth);
                    delayedTruth = wrapAngle(
                        delayedTruth + dt * (r1 - pastR2));
                }
                lastDelivered = delayedTruth;
                truck_model::ArticulationLidarMeasurement measurement;
                measurement.stamp = stamp;
                measurement.articulation = delayedTruth;
                estimator.updateLidar(measurement);
            }
        }

        if (time >= 0.6) {
            const double estimateError = wrapAngle(
                estimator.estimate().articulation - truth);
            const double delayedError = wrapAngle(lastDelivered - truth);
            estimateSse += estimateError * estimateError;
            delayedSse += delayedError * delayedError;
            ++scoreCount;
        }
    }

    const double estimateRmse = std::sqrt(estimateSse / scoreCount);
    const double delayedRmse = std::sqrt(delayedSse / scoreCount);
    if (estimateRmse > 0.015 || estimateRmse > 0.35 * delayedRmse) {
        std::cerr << "delayed EKF did not recover current articulation: "
                  << "estimate RMSE=" << estimateRmse
                  << " delayed RMSE=" << delayedRmse << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testLidarOutlierIsGated() {
    const auto p = parameters();
    truck_model::ArticulationEstimator estimator(p);
    estimator.reset(0.0, 0.05);
    truck_model::ArticulationInputs inputs;
    inputs.time = 0.05;
    inputs.truckYawRate = 0.02;
    inputs.speed = 10.0;
    estimator.predict(inputs);
    truck_model::ArticulationLidarMeasurement good;
    good.stamp = 0.05;
    good.articulation = 0.05;
    estimator.updateLidar(good);
    inputs.time = 0.10;
    estimator.predict(inputs);
    truck_model::ArticulationLidarMeasurement outlier;
    outlier.stamp = 0.10;
    outlier.articulation = 0.55;
    const auto after = estimator.updateLidar(outlier);
    if (!after.measurementGated || after.measurementAccepted) {
        std::cerr << "gross lidar outlier was not gated\n";
        std::exit(EXIT_FAILURE);
    }
    if (std::abs(after.articulation - 0.55) < 0.1) {
        std::cerr << "gated outlier still pulled the estimate\n";
        std::exit(EXIT_FAILURE);
    }
}

void testEstimatorCoastsAfterDropout() {
    const auto p = parameters();
    truck_model::ArticulationEstimator estimator(p);
    estimator.reset(0.0, 0.0);
    for (int step = 1; step <= 30; ++step) {
        truck_model::ArticulationInputs inputs;
        inputs.time = 0.05 * static_cast<double>(step);
        inputs.speed = 10.0;
        inputs.truckYawRate = 0.03;
        estimator.predict(inputs);
    }
    if (!estimator.estimate().coasting) {
        std::cerr << "estimator did not enter coasting after lidar dropout\n";
        std::exit(EXIT_FAILURE);
    }
    expectFinite(estimator.estimate().articulation, "coasting articulation");
    expectFinite(
        estimator.estimate().articulationRate, "coasting articulation rate");
}

void testEstimatorRejectsInvalidConfig() {
    auto config = truck_model::ArticulationEstimatorConfig{};
    config.historyHorizon = 0.0;
    bool rejected = false;
    try {
        truck_model::ArticulationEstimator estimator(parameters(), config);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    if (!rejected) {
        std::cerr << "invalid estimator config must be rejected\n";
        std::exit(EXIT_FAILURE);
    }
}

// --- Delayed EKF v2 ---------------------------------------------------------

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

// Van Loan must reproduce the closed form of an integrating random walk:
// for xDot = [[0,-1],[0,0]] x + w with w = diag(0, q), the exact covariance is
// [[q T^3/3, -q T^2/2], [-q T^2/2, q T]].
void testVanLoanMatchesAnalyticRandomWalk() {
    constexpr double q = 3.7e-3;
    constexpr double dt = 0.05;
    truck_model::Matrix<2, 2> continuous{};
    continuous[0][1] = -1.0;
    truck_model::Matrix<2, 2> density{};
    density[1][1] = q;

    const auto discrete =
        truck_model::discretizeVanLoan(continuous, density, dt);

    expectNear(discrete.transition[0][0], 1.0, "van Loan transition 00");
    expectNear(discrete.transition[0][1], -dt, "van Loan transition 01");
    expectNear(discrete.transition[1][1], 1.0, "van Loan transition 11");
    expectNear(
        discrete.processNoise[0][0],
        q * dt * dt * dt / 3.0,
        "van Loan integrated bias variance");
    expectNear(
        discrete.processNoise[0][1],
        -0.5 * q * dt * dt,
        "van Loan cross covariance");
    expectNear(
        discrete.processNoise[1][0],
        -0.5 * q * dt * dt,
        "van Loan cross covariance symmetry");
    expectNear(discrete.processNoise[1][1], q * dt, "van Loan bias variance");

    // The cross term is what a diagonal Euler approximation throws away.
    require(
        std::abs(discrete.processNoise[0][1]) > 1.0e-9,
        "cross covariance must not be negligible at the demo sample time");
}

// Q must scale with the interval, so splitting a step in two may not change the
// accumulated covariance beyond the second-order transition coupling.
void testProcessNoiseIsStepInvariant() {
    constexpr double q = 2.0e-3;
    truck_model::Matrix<2, 2> continuous{};
    continuous[0][1] = -1.0;
    truck_model::Matrix<2, 2> density{};
    density[0][0] = 1.5e-3;
    density[1][1] = q;

    const auto whole = truck_model::discretizeVanLoan(continuous, density, 0.05);
    const auto first = truck_model::discretizeVanLoan(continuous, density, 0.02);
    const auto second = truck_model::discretizeVanLoan(continuous, density, 0.03);

    // Compose the two half steps: Q = F2 Q1 F2^T + Q2.
    const auto left = truck_model::matrixProduct(second.transition, first.processNoise);
    auto composed =
        truck_model::matrixProduct(left, truck_model::transposed(second.transition));
    for (std::size_t row = 0; row < 2; ++row) {
        for (std::size_t column = 0; column < 2; ++column) {
            composed[row][column] += second.processNoise[row][column];
            expectNear(
                composed[row][column],
                whole.processNoise[row][column],
                "split-step process noise must compose");
        }
    }
}

// The kinematic linearization must agree with a numerical derivative of the
// propagated mean.
void testKinematicJacobianMatchesFiniteDifference() {
    const auto p = parameters();
    truck_model::ArticulationEstimatorConfig config;
    truck_model::KinematicArticulationModel model(p, config);

    truck_model::ArticulationInputs inputs;
    inputs.time = 0.0;
    inputs.truckYawRate = 0.11;
    inputs.speed = 13.0;
    const double dt = 0.05;

    const truck_model::Vector<2> base{0.21, 0.013};
    const auto analytic = model.continuousJacobian(base, inputs);

    constexpr double step = 1.0e-6;
    for (std::size_t column = 0; column < 2; ++column) {
        auto forward = base;
        auto backward = base;
        forward[column] += step;
        backward[column] -= step;
        truck_model::Matrix<2, 2> ignored{};
        model.propagate(forward, ignored, inputs, dt, 1.0);
        truck_model::Matrix<2, 2> ignoredToo{};
        model.propagate(backward, ignoredToo, inputs, dt, 1.0);
        for (std::size_t row = 0; row < 2; ++row) {
            const double numeric =
                (forward[row] - backward[row]) / (2.0 * step);
            const double expected =
                (row == column ? 1.0 : 0.0) + dt * analytic[row][column];
            if (std::abs(numeric - expected) > 1.0e-6) {
                std::cerr << "kinematic Jacobian mismatch at (" << row << ','
                          << column << "): " << numeric << " vs " << expected
                          << '\n';
                std::exit(EXIT_FAILURE);
            }
        }
    }
}

std::size_t observabilityRank(
    std::vector<std::vector<double>> rows,
    double tolerance) {
    const std::size_t columns = rows.empty() ? 0 : rows.front().size();
    std::size_t rank = 0;
    for (std::size_t column = 0; column < columns && rank < rows.size();
         ++column) {
        std::size_t pivot = rank;
        for (std::size_t row = rank; row < rows.size(); ++row) {
            if (std::abs(rows[row][column]) > std::abs(rows[pivot][column])) {
                pivot = row;
            }
        }
        if (std::abs(rows[pivot][column]) <= tolerance) {
            continue;
        }
        std::swap(rows[rank], rows[pivot]);
        for (std::size_t row = 0; row < rows.size(); ++row) {
            if (row == rank) {
                continue;
            }
            const double factor = rows[row][column] / rows[rank][column];
            for (std::size_t k = column; k < columns; ++k) {
                rows[row][k] -= factor * rows[rank][k];
            }
        }
        ++rank;
    }
    return rank;
}

// With the angle and the trailer yaw residual as the only states, the pair is
// observable at any operating point: det([[1,0],[-a,-1]]) = -1, independent of
// speed and of the articulation angle.
void testKinematicObservabilityRank() {
    const auto p = parameters();
    truck_model::ArticulationEstimatorConfig config;
    truck_model::KinematicArticulationModel model(p, config);

    truck_model::ArticulationInputs inputs;
    inputs.truckYawRate = 0.05;
    inputs.speed = 15.0;
    const truck_model::Vector<2> state{0.08, 0.0};
    const auto a = model.continuousJacobian(state, inputs);

    // Rows of the observability matrix for H = [1, 0].
    std::vector<double> h{1.0, 0.0};
    std::vector<std::vector<double>> rows;
    rows.push_back(h);
    std::vector<double> next(2, 0.0);
    for (std::size_t column = 0; column < 2; ++column) {
        for (std::size_t k = 0; k < 2; ++k) {
            next[column] += h[k] * a[k][column];
        }
    }
    rows.push_back(next);
    require(
        observabilityRank(rows, 1.0e-9) == 2,
        "two-state kinematic model must be observable");
    expectNear(
        rows[0][0] * rows[1][1] - rows[0][1] * rows[1][0],
        -1.0,
        "observability determinant must be exactly -1");
}

// The reduced (K27) model is observable through a31, the sensitivity of the
// trailer yaw acceleration to the truck lateral velocity.
void testDynamicObservabilityRank() {
    const auto p = parameters();
    truck_model::ArticulationEstimatorConfig config;
    config.processModel = truck_model::ArticulationProcessModel::dynamic;
    truck_model::DynamicArticulationModel model(p, config);
    const auto a = model.continuousJacobian(p.vx);

    const auto plant = truck_model::buildDynamicModel(p);
    expectNear(a[1][0], plant.a[2][0], "dynamic model must reuse a31");
    require(
        std::abs(plant.a[2][0]) > 1.0e-3,
        "a31 must be well clear of zero for the reduced model to be observable");

    // H = [0, 0, 1] reads the articulation only. Stack H, HA, HA^2.
    std::vector<double> h{0.0, 0.0, 1.0};
    std::vector<std::vector<double>> rows;
    rows.push_back(h);
    for (int power = 0; power < 2; ++power) {
        std::vector<double> next(3, 0.0);
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t k = 0; k < 3; ++k) {
                next[column] += h[k] * a[k][column];
            }
        }
        rows.push_back(next);
        h = next;
    }
    require(
        observabilityRank(rows, 1.0e-9) == 3,
        "reduced K27 model must be observable in its three states");

    // det O = -a31, the only route by which the lidar reaches vy1.
    const double determinant =
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1]) -
        rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0]) +
        rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]);
    expectNear(
        determinant, -a[1][0], "observability determinant must equal -a31");
}

// Successive delayed scans must each be fused at their own stamp, and a stamp
// older than one already fused must be refused rather than misapplied.
void testDelayedScanOrdering() {
    const auto p = parameters();
    truck_model::ArticulationEstimatorConfig config;
    config.historyHorizon = 1.2;
    truck_model::ArticulationEstimator estimator(p, config);
    estimator.reset(0.0, 0.0);

    for (int step = 1; step <= 20; ++step) {
        truck_model::ArticulationInputs inputs;
        inputs.time = 0.05 * static_cast<double>(step);
        inputs.truckYawRate = 0.05;
        inputs.speed = 12.0;
        estimator.predict(inputs);
    }

    truck_model::ArticulationLidarMeasurement recent;
    recent.stamp = 0.80;
    recent.articulation = 0.12;
    const auto afterRecent = estimator.updateLidar(recent);
    require(afterRecent.measurementAccepted, "recent scan must be accepted");
    const double corrected = afterRecent.articulation;

    // The shell is only correct while stamps are non-decreasing, so it checks
    // rather than assumes. A reordering pipeline shows up as a counter instead
    // of a silently corrupted state.
    truck_model::ArticulationLidarMeasurement stale;
    stale.stamp = 0.60;
    stale.articulation = 0.02;
    const auto afterStale = estimator.updateLidar(stale);
    require(
        afterStale.outcome == truck_model::MeasurementOutcome::outOfOrder,
        "a scan older than one already fused must be reported as out of order");
    expectNear(
        afterStale.articulation,
        corrected,
        "a rejected out-of-order scan must not move the estimate");

    truck_model::ArticulationLidarMeasurement repeat;
    repeat.stamp = 0.80;
    repeat.articulation = 0.12;
    require(
        estimator.updateLidar(repeat).outcome ==
            truck_model::MeasurementOutcome::duplicate,
        "the same stamp twice must be reported as a duplicate");
    expectNear(
        estimator.estimate().articulation,
        corrected,
        "a rejected duplicate must not move the estimate either");
}

void testMeasurementBoundaryHandling() {
    const auto p = parameters();
    truck_model::ArticulationEstimatorConfig config;
    config.historyHorizon = 0.4;
    truck_model::ArticulationEstimator estimator(p, config);
    estimator.reset(0.0, 0.0);
    for (int step = 1; step <= 20; ++step) {
        truck_model::ArticulationInputs inputs;
        inputs.time = 0.05 * static_cast<double>(step);
        inputs.truckYawRate = 0.03;
        inputs.speed = 10.0;
        estimator.predict(inputs);
    }

    truck_model::ArticulationLidarMeasurement stale;
    stale.stamp = 0.10;
    stale.articulation = 0.01;
    require(
        estimator.updateLidar(stale).outcome ==
            truck_model::MeasurementOutcome::staleBeyondWindow,
        "a scan older than the window must report staleBeyondWindow");

    truck_model::ArticulationLidarMeasurement future;
    future.stamp = 1.30;
    future.articulation = 0.01;
    require(
        estimator.updateLidar(future).outcome ==
            truck_model::MeasurementOutcome::aheadOfInputs,
        "a scan newer than the newest input must be refused, not extrapolated");

    truck_model::ArticulationLidarMeasurement good;
    good.stamp = 0.90;
    good.articulation = 0.012;
    require(
        estimator.updateLidar(good).outcome ==
            truck_model::MeasurementOutcome::accepted,
        "an in-window scan must be accepted");
    require(
        estimator.updateLidar(good).outcome ==
            truck_model::MeasurementOutcome::duplicate,
        "replaying the same identifier must be refused");
}

// A corrupt packet must be reported, not thrown: unwinding out of a control
// loop is worse than dropping one scan. It also has to leave the state alone.
void testNonFiniteMeasurementIsReportedNotThrown() {
    const auto p = parameters();
    truck_model::ArticulationEstimator estimator(p);
    estimator.reset(0.0, 0.05);
    for (int step = 1; step <= 10; ++step) {
        truck_model::ArticulationInputs inputs;
        inputs.time = 0.05 * static_cast<double>(step);
        inputs.truckYawRate = 0.04;
        inputs.speed = 11.0;
        estimator.predict(inputs);
    }
    // Land a real update first, so the diagnostics below are genuinely
    // non-zero and the clearing has something to clear.
    truck_model::ArticulationLidarMeasurement good;
    good.stamp = 0.35;
    good.articulation = 0.08;
    const auto accepted = estimator.updateLidar(good);
    require(accepted.measurementAccepted, "setup scan must be accepted");
    require(
        accepted.innovationCovariance > 0.0 && accepted.alignedStamp > 0.0,
        "setup scan must leave non-zero diagnostics behind");

    const double before = estimator.estimate().articulation;

    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
        truck_model::ArticulationLidarMeasurement corrupt;
        corrupt.stamp = 0.40;
        corrupt.articulation = bad;
        const auto report = estimator.updateLidar(corrupt);
        require(
            report.outcome == truck_model::MeasurementOutcome::nonFinite,
            "a non-finite articulation must report nonFinite");

        truck_model::ArticulationLidarMeasurement badStamp;
        badStamp.stamp = bad;
        badStamp.articulation = 0.05;
        require(
            estimator.updateLidar(badStamp).outcome ==
                truck_model::MeasurementOutcome::nonFinite,
            "a non-finite stamp must report nonFinite");
    }
    expectNear(
        estimator.estimate().articulation,
        before,
        "a rejected non-finite packet must not move the estimate");

    // The outcome must not be published next to the previous packet's
    // innovation, or a corrupt frame reads as a healthy update in the log.
    const auto& stale = estimator.estimate();
    require(
        stale.innovation == 0.0 && stale.innovationCovariance == 0.0 &&
            stale.mahalanobis == 0.0 && stale.alignedStamp == 0.0 &&
            stale.repropagatedFrames == 0 && stale.kalmanGainPhi == 0.0 &&
            stale.kalmanGainTrailerBias == 0.0,
        "a rejected packet must not carry the previous packet's diagnostics");
}

// Steering reaches the dynamic state and both models' yaw residual, so a
// non-finite value has to be caught at the door like the other inputs.
void testNonFiniteInputsAreRejected() {
    const auto p = parameters();
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
        for (int field = 0; field < 4; ++field) {
            truck_model::ArticulationEstimator estimator(p);
            estimator.reset(0.0, 0.0);
            truck_model::ArticulationInputs inputs;
            inputs.time = 0.05;
            inputs.truckYawRate = 0.03;
            inputs.speed = 10.0;
            inputs.steering = 0.02;
            switch (field) {
                case 0: inputs.time = bad; break;
                case 1: inputs.truckYawRate = bad; break;
                case 2: inputs.speed = bad; break;
                default: inputs.steering = bad; break;
            }
            bool rejected = false;
            try {
                estimator.predict(inputs);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(
                rejected, "every non-finite predict input must be rejected");
        }
    }
}

// The replay span from the corrected frame to now carries no measurement, so
// it must be inflated when it is long enough to count as coasting. Only the
// replay differs between the two runs below: the frames before the update are
// younger than either lostTimeout, and everything after it is recomputed.
void testReplaySpanIsCoastedWhenItExceedsLostTimeout() {
    const auto p = parameters();
    const auto run = [&p](double lostTimeout) {
        truck_model::ArticulationEstimatorConfig config;
        config.historyHorizon = 1.5;   // deliberately longer than lostTimeout
        config.lostTimeout = lostTimeout;
        truck_model::ArticulationEstimator estimator(p, config);
        estimator.reset(0.0, 0.0);
        for (int step = 1; step <= 24; ++step) {
            truck_model::ArticulationInputs inputs;
            inputs.time = 0.05 * static_cast<double>(step);
            inputs.truckYawRate = 0.05;
            inputs.speed = 12.0;
            estimator.predict(inputs);
        }
        truck_model::ArticulationLidarMeasurement late;
        late.stamp = 0.25;
        late.articulation = 0.02;
        const auto report = estimator.updateLidar(late);
        require(report.measurementAccepted, "late scan must be accepted");
        return report.covariancePhi;
    };

    const double coasted = run(0.3);     // replay crosses the threshold
    const double steady = run(10.0);     // replay never counts as coasting
    require(
        coasted > steady * 1.05,
        "a replay span longer than lostTimeout must inflate the covariance");
}

// Scans that fall between input samples must be fused at their own stamp.
void testSubFrameAlignmentIsExact() {
    const auto p = parameters();
    for (const double phase : {0.005, 0.017, 0.025, 0.043}) {
        truck_model::ArticulationEstimatorConfig config;
        config.historyHorizon = 1.0;
        truck_model::ArticulationEstimator estimator(p, config);
        estimator.reset(0.0, 0.0);
        for (int step = 1; step <= 20; ++step) {
            truck_model::ArticulationInputs inputs;
            inputs.time = 0.05 * static_cast<double>(step);
            inputs.truckYawRate = 0.06;
            inputs.speed = 12.0;
            estimator.predict(inputs);
        }
        truck_model::ArticulationLidarMeasurement measurement;
        measurement.stamp = 0.50 + phase;
        measurement.articulation = 0.05;
        const auto report = estimator.updateLidar(measurement);
        require(
            report.measurementAccepted,
            "sub-frame scan must be accepted");
        expectNear(
            report.alignedStamp,
            measurement.stamp,
            "sub-frame scan must align to its own stamp, not a stored frame");
    }
}

// Truth for the latency benchmark comes from the (K27) plant, not from the
// filter's own process model, so the comparison is not an inverse crime.
struct PlantTrace {
    std::vector<double> time;
    std::vector<double> articulation;
    std::vector<double> articulationRate;
    std::vector<double> truckYawRate;
};

PlantTrace simulatePlant(
    const truck_model::Parameters& p,
    double steeringAmplitude,
    double steeringFrequency,
    double dt,
    int steps) {
    const auto plant = truck_model::buildDynamicModel(p);
    const auto discrete = truck_model::discretizeZeroOrderHold(plant, dt);
    truck_model::Vector<4> state{};
    PlantTrace trace;
    trace.time.push_back(0.0);
    trace.articulation.push_back(state[3]);
    trace.articulationRate.push_back(state[1] - state[2]);
    trace.truckYawRate.push_back(state[1]);
    for (int step = 1; step <= steps; ++step) {
        const double time = dt * static_cast<double>(step);
        const double steering =
            steeringAmplitude * std::sin(2.0 * 3.14159265358979323846 *
                                         steeringFrequency * (time - dt));
        auto next = truck_model::multiply(discrete.a, state);
        for (std::size_t i = 0; i < 4; ++i) {
            next[i] += discrete.b[i] * steering;
        }
        state = next;
        trace.time.push_back(time);
        trace.articulation.push_back(state[3]);
        trace.articulationRate.push_back(state[1] - state[2]);
        trace.truckYawRate.push_back(state[1]);
    }
    return trace;
}

struct TrackingScore {
    double articulationRmse{};
    double rateRmse{};
    double rateAmplitudeRatio{};
    double delayedRmse{};
};

TrackingScore scoreEstimator(
    const truck_model::Parameters& filterParameters,
    const truck_model::Parameters& plantParameters,
    truck_model::ArticulationProcessModel model,
    double latency,
    double noiseStd) {
    constexpr double dt = 0.05;
    constexpr int steps = 400;
    const auto trace = simulatePlant(plantParameters, 0.06, 0.15, dt, steps);

    truck_model::ArticulationEstimatorConfig config;
    config.processModel = model;
    config.historyHorizon = latency + 4.0 * dt;
    config.measurementVariance = std::max(noiseStd * noiseStd, 1.0e-8);
    truck_model::ArticulationEstimator estimator(filterParameters, config);
    estimator.reset(0.0, 0.0);

    const auto latencySteps = static_cast<std::size_t>(std::lround(latency / dt));
    double estimateSse = 0.0;
    double rateSse = 0.0;
    double rateEnergy = 0.0;
    double estimateRateEnergy = 0.0;
    double delayedSse = 0.0;
    int scored = 0;
    std::uint32_t rng = 12345u;
    const auto gaussian = [&rng]() {
        rng = rng * 1664525u + 1013904223u;
        const double u1 = std::max(
            static_cast<double>(rng >> 8) * (1.0 / 16777216.0), 1.0e-12);
        rng = rng * 1664525u + 1013904223u;
        const double u2 = static_cast<double>(rng >> 8) * (1.0 / 16777216.0);
        return std::sqrt(-2.0 * std::log(u1)) *
               std::cos(2.0 * 3.14159265358979323846 * u2);
    };

    // Baseline: the newest scan actually delivered, carrying both its latency
    // and its noise. Comparing against noiseless delayed truth would understate
    // what the controller would otherwise consume.
    double lastDelivered = 0.0;
    bool hasDelivered = false;

    for (std::size_t index = 1; index < trace.time.size(); ++index) {
        truck_model::ArticulationInputs inputs;
        inputs.time = trace.time[index];
        inputs.truckYawRate = trace.truckYawRate[index];
        inputs.speed = plantParameters.vx;
        estimator.predict(inputs);

        if (index % 2 == 0 && index >= latencySteps) {
            const std::size_t scanIndex = index - latencySteps;
            truck_model::ArticulationLidarMeasurement measurement;
            measurement.stamp = trace.time[scanIndex];
            measurement.articulation =
                trace.articulation[scanIndex] + noiseStd * gaussian();
            estimator.updateLidar(measurement);
            lastDelivered = measurement.articulation;
            hasDelivered = true;
        }

        if (trace.time[index] < 4.0 || !hasDelivered) {
            continue;
        }
        const auto& estimate = estimator.estimate();
        const double articulationError =
            estimate.articulation - trace.articulation[index];
        const double rateError =
            estimate.articulationRate - trace.articulationRate[index];
        estimateSse += articulationError * articulationError;
        rateSse += rateError * rateError;
        rateEnergy +=
            trace.articulationRate[index] * trace.articulationRate[index];
        estimateRateEnergy += estimate.articulationRate * estimate.articulationRate;
        const double delayed = lastDelivered - trace.articulation[index];
        delayedSse += delayed * delayed;
        ++scored;
    }

    TrackingScore score;
    score.articulationRmse = std::sqrt(estimateSse / scored);
    score.rateRmse = std::sqrt(rateSse / scored);
    score.rateAmplitudeRatio =
        rateEnergy > 0.0 ? std::sqrt(estimateRateEnergy / rateEnergy) : 0.0;
    score.delayedRmse = std::sqrt(delayedSse / scored);
    return score;
}

// The dynamic model exists to fix the articulation rate, so the rate is a
// first-class acceptance metric rather than a footnote.
void testDynamicModelImprovesRateTracking() {
    const auto p = parameters();
    // One degree of scan noise. A noiseless scan would drive R to its floor and
    // make both filters chase measurement jitter instead of the model.
    constexpr double noiseStd = 0.01745;
    const auto kinematic = scoreEstimator(
        p, p, truck_model::ArticulationProcessModel::kinematic, 0.20, noiseStd);
    const auto dynamic = scoreEstimator(
        p, p, truck_model::ArticulationProcessModel::dynamic, 0.20, noiseStd);

    require(
        kinematic.articulationRmse < 0.05,
        "kinematic model must still track the articulation angle");
    require(
        dynamic.articulationRmse <= kinematic.articulationRmse,
        "dynamic model must not degrade the articulation angle");
    if (!(dynamic.rateRmse < 0.8 * kinematic.rateRmse)) {
        std::cerr << "dynamic model did not improve the rate: "
                  << dynamic.rateRmse << " vs " << kinematic.rateRmse << '\n';
        std::exit(EXIT_FAILURE);
    }

    // phiDot is the difference of two much larger yaw rates, so a 10 Hz noisy
    // angle leaves residual jitter in the rate for any process model. Compare
    // the amplitude against the kinematic baseline rather than against an
    // absolute band: the kinematic random walk is systematically thin, and the
    // dynamic model has to move the ratio toward unity without overshooting.
    const double kinematicGap = std::abs(kinematic.rateAmplitudeRatio - 1.0);
    const double dynamicGap = std::abs(dynamic.rateAmplitudeRatio - 1.0);
    if (!(dynamicGap < kinematicGap)) {
        std::cerr << "dynamic model rate amplitude is not closer to unity: "
                  << dynamic.rateAmplitudeRatio << " vs "
                  << kinematic.rateAmplitudeRatio << '\n';
        std::exit(EXIT_FAILURE);
    }
    if (dynamic.rateAmplitudeRatio < 0.85 ||
        dynamic.rateAmplitudeRatio > 1.20) {
        std::cerr << "dynamic model rate amplitude ratio out of band: "
                  << dynamic.rateAmplitudeRatio << '\n';
        std::exit(EXIT_FAILURE);
    }
}

// The dynamic model depends on trailer mass and cornering stiffness, so it must
// be checked against a plant it does not match exactly.
void testDynamicModelSurvivesParameterMismatch() {
    const auto plantParameters = parameters();
    for (const double massScale : {0.7, 1.3}) {
        for (const double stiffnessScale : {0.7, 1.3}) {
            auto filterParameters = plantParameters;
            filterParameters.m2 *= massScale;
            filterParameters.iz2 *= massScale;
            filterParameters.c2r *= stiffnessScale;

            const auto kinematic = scoreEstimator(
                filterParameters,
                plantParameters,
                truck_model::ArticulationProcessModel::kinematic,
                0.20,
                0.01745);
            const auto dynamic = scoreEstimator(
                filterParameters,
                plantParameters,
                truck_model::ArticulationProcessModel::dynamic,
                0.20,
                0.01745);
            if (!(dynamic.articulationRmse < 0.05) ||
                !(dynamic.rateRmse <= kinematic.rateRmse)) {
                std::cerr << "dynamic model lost to the kinematic model under "
                          << "mismatch m2x" << massScale << " c2rx"
                          << stiffnessScale << ": phi "
                          << dynamic.articulationRmse << " rate "
                          << dynamic.rateRmse << " vs " << kinematic.rateRmse
                          << '\n';
                std::exit(EXIT_FAILURE);
            }
        }
    }
}

// Both models must beat the raw delayed scan they are built from.
void testBothModelsBeatDelayedMeasurement() {
    const auto p = parameters();
    for (const auto model : {truck_model::ArticulationProcessModel::kinematic,
                             truck_model::ArticulationProcessModel::dynamic}) {
        const auto score = scoreEstimator(p, p, model, 0.30, 0.0698);
        if (!(score.articulationRmse < 0.5 * score.delayedRmse)) {
            std::cerr << "estimator failed to beat the delayed scan: "
                      << score.articulationRmse << " vs " << score.delayedRmse
                      << '\n';
            std::exit(EXIT_FAILURE);
        }
    }
}

}  // namespace

int main() {
    testNonlinearKinematics();
    testConstraintReconstructionAndDerivative();
    testErrorModelTransformation();
    testInvalidSpeedRejected();
    testMpcStabilizesStraightPath();
    testMpcTracksConstantArticulationReference();
    testReferencePathFromDrawnWaypoints();
    testPurePursuitPathSmoothing();
    testKinematicTrailerYawRateMatchesK5();
    testDelayedEkfCompensatesLidarLatency();
    testLidarOutlierIsGated();
    testEstimatorCoastsAfterDropout();
    testEstimatorRejectsInvalidConfig();
    testVanLoanMatchesAnalyticRandomWalk();
    testProcessNoiseIsStepInvariant();
    testKinematicJacobianMatchesFiniteDifference();
    testKinematicObservabilityRank();
    testDynamicObservabilityRank();
    testDelayedScanOrdering();
    testMeasurementBoundaryHandling();
    testNonFiniteMeasurementIsReportedNotThrown();
    testNonFiniteInputsAreRejected();
    testReplaySpanIsCoastedWhenItExceedsLostTimeout();
    testSubFrameAlignmentIsExact();
    testDynamicModelImprovesRateTracking();
    testDynamicModelSurvivesParameterMismatch();
    testBothModelsBeatDelayedMeasurement();
    std::cout << "All articulated vehicle model tests passed.\n";
    return EXIT_SUCCESS;
}
