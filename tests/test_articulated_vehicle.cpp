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

void expectRelative(
    double actual,
    double expected,
    double tolerance,
    const char* message) {
    if (std::abs(actual - expected) >
        tolerance * std::max(std::abs(expected), 1.0e-300)) {
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

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
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

// The truth carries a trailer yaw residual the filter has to learn, and the
// filter starts from the wrong angle, so an open-loop prediction cannot pass:
// the 200 ms late scans have to do the work.
void testDelayedEkfCompensatesLidarLatency() {
    const auto p = parameters();
    truck_model::ArticulationEstimator estimator(p);
    truck_model::ArticulationEstimator openLoop(p);
    const double dt = 0.02;
    const double speed = 12.0;
    const double residual = 0.03;
    constexpr int steps = 250;
    constexpr int latencySteps = 10;
    // A time-varying yaw rate keeps the articulation moving, so a 200 ms old
    // scan stays measurably wrong instead of converging onto a constant.
    const auto yawRate = [dt](int step) {
        return 0.08 + 0.06 * std::sin(2.0 * 3.14159265358979323846 * 0.5 * dt *
                                      static_cast<double>(step));
    };

    std::vector<double> truth(steps + 1);
    truth[0] = 0.15;
    for (int step = 1; step <= steps; ++step) {
        const double r1 = yawRate(step);
        const double r2 =
            truck_model::kinematicTrailerYawRate(p, speed, r1, truth[step - 1]) +
            residual;
        truth[step] = wrapAngle(truth[step - 1] + dt * (r1 - r2));
    }
    estimator.reset(0.0, 0.05);
    openLoop.reset(0.0, 0.05);

    double estimateSse = 0.0;
    double openLoopSse = 0.0;
    double delayedSse = 0.0;
    int scoreCount = 0;
    double lastDelivered = truth[0];
    for (int step = 1; step <= steps; ++step) {
        const double time = dt * static_cast<double>(step);
        truck_model::ArticulationInputs inputs;
        inputs.time = time;
        inputs.truckYawRate = yawRate(step);
        inputs.speed = speed;
        estimator.predict(inputs);
        openLoop.predict(inputs);

        if (step % 5 == 0 && step >= latencySteps) {
            truck_model::ArticulationLidarMeasurement measurement;
            measurement.stamp = dt * static_cast<double>(step - latencySteps);
            measurement.articulation = truth[step - latencySteps];
            lastDelivered = measurement.articulation;
            const double before = estimator.estimate().articulation;
            const auto report = estimator.updateLidar(measurement);
            require(report.measurementAccepted, "latency scan must be accepted");
            require(
                report.repropagatedFrames >= latencySteps,
                "a 200 ms late scan must replay the frames after its stamp");
            require(
                (report.articulation - before) * report.innovation > 0.0,
                "the current estimate must move toward the late measurement");
        }

        if (time >= 1.0) {
            const double estimateError =
                wrapAngle(estimator.estimate().articulation - truth[step]);
            const double openLoopError =
                wrapAngle(openLoop.estimate().articulation - truth[step]);
            const double delayedError = wrapAngle(lastDelivered - truth[step]);
            estimateSse += estimateError * estimateError;
            openLoopSse += openLoopError * openLoopError;
            delayedSse += delayedError * delayedError;
            ++scoreCount;
        }
    }

    const double estimateRmse = std::sqrt(estimateSse / scoreCount);
    const double openLoopRmse = std::sqrt(openLoopSse / scoreCount);
    const double delayedRmse = std::sqrt(delayedSse / scoreCount);
    // Measured: 0.0039 / 0.0105 / 0.0153 rad. Dropping the update leaves the
    // open-loop error; fusing late scans as current leaves the delayed error.
    if (estimateRmse > 0.008 || estimateRmse > 0.5 * delayedRmse ||
        estimateRmse > 0.4 * openLoopRmse) {
        std::cerr << "delayed EKF did not recover current articulation: "
                  << "estimate RMSE=" << estimateRmse
                  << " delayed RMSE=" << delayedRmse
                  << " open-loop RMSE=" << openLoopRmse << '\n';
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
    const auto rejects = [](const truck_model::ArticulationEstimatorConfig& config) {
        try {
            truck_model::ArticulationEstimator estimator(parameters(), config);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    auto horizon = truck_model::ArticulationEstimatorConfig{};
    horizon.historyHorizon = 0.0;
    auto scale = truck_model::ArticulationEstimatorConfig{};
    scale.coastingProcessNoiseScale = 0.5;
    auto frames = truck_model::ArticulationEstimatorConfig{};
    frames.maximumFrames = 1;
    if (!rejects(horizon) || !rejects(scale) || !rejects(frames)) {
        std::cerr << "invalid estimator config must be rejected\n";
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

    // Relative tolerances: the entries span four decades, so an absolute band
    // wide enough for Q11 would accept a wrong coefficient in Q00.
    constexpr double tight = 1.0e-10;
    expectRelative(discrete.transition[0][0], 1.0, tight, "van Loan transition 00");
    expectRelative(discrete.transition[0][1], -dt, tight, "van Loan transition 01");
    expectRelative(discrete.transition[1][1], 1.0, tight, "van Loan transition 11");
    expectRelative(
        discrete.processNoise[0][0],
        q * dt * dt * dt / 3.0,
        tight,
        "van Loan integrated bias variance");
    expectRelative(
        discrete.processNoise[0][1],
        -0.5 * q * dt * dt,
        tight,
        "van Loan cross covariance");
    expectRelative(
        discrete.processNoise[1][0],
        -0.5 * q * dt * dt,
        tight,
        "van Loan cross covariance symmetry");
    expectRelative(
        discrete.processNoise[1][1], q * dt, tight, "van Loan bias variance");

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
            expectRelative(
                composed[row][column],
                whole.processNoise[row][column],
                1.0e-10,
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
        const std::size_t framesBefore = estimator.estimate().historySize;
        const double before = estimator.estimate().articulation;
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
        require(
            report.historySize == framesBefore + 1 &&
                report.repropagatedFrames > 0,
            "an accepted sub-frame scan must add its frame and replay forward");
        require(
            (report.articulation - before) * report.innovation > 1.0e-6,
            "the current estimate must be corrected toward the scan");
    }
}

constexpr double kPi = 3.14159265358979323846;

truck_model::ArticulationInputs steadyInputs(
    double time,
    double truckYawRate,
    double speed,
    double steering = 0.0) {
    truck_model::ArticulationInputs inputs;
    inputs.time = time;
    inputs.truckYawRate = truckYawRate;
    inputs.speed = speed;
    inputs.steering = steering;
    return inputs;
}

// A filter that was never reset has no state to predict from, and inventing
// one would publish a plausible-looking estimate.
void testPredictBeforeResetThrows() {
    truck_model::ArticulationEstimator estimator(parameters());
    bool rejected = false;
    try {
        estimator.predict(steadyInputs(0.05, 0.0, 10.0));
    } catch (const std::invalid_argument&) {
        rejected = false;
    } catch (const std::logic_error&) {
        rejected = true;
    }
    require(rejected, "predict before reset must throw std::logic_error");
    require(
        !estimator.initialized(),
        "a refused predict must not initialize the filter");
}

void testResetRejectsInvalidCovariance() {
    using Shell =
        truck_model::DelayedEkf<truck_model::KinematicArticulationModel>;
    const truck_model::KinematicArticulationModel model(
        parameters(), truck_model::ArticulationEstimatorConfig{});
    const truck_model::Vector<2> state{0.0, 0.0};
    truck_model::Matrix<2, 2> asymmetric{};
    asymmetric[0][0] = 1.0e-3;
    asymmetric[1][1] = 1.0e-3;
    asymmetric[0][1] = 1.0e-4;
    truck_model::Matrix<2, 2> negative{};
    negative[0][0] = -1.0e-3;
    negative[1][1] = 1.0e-3;
    for (const auto& covariance : {asymmetric, negative}) {
        Shell shell;
        shell.configure(model, truck_model::DelayedEkfLimits{});
        bool rejected = false;
        try {
            shell.reset(
                0.0, state, covariance, truck_model::ArticulationInputs{});
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(
            rejected,
            "an asymmetric or negative reset covariance must be refused");
    }
}

// Input time is the filter clock; letting it run backwards would rewrite a
// propagated frame or publish an estimate older than the state behind it.
void testOlderInputThrows() {
    truck_model::ArticulationEstimator estimator(parameters());
    estimator.reset(0.0, 0.0);
    estimator.predict(steadyInputs(0.10, 0.02, 10.0));
    const double timeBefore = estimator.estimate().time;
    bool rejected = false;
    try {
        estimator.predict(steadyInputs(0.05, 0.02, 10.0));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "an input older than the newest frame must be refused");
    require(
        estimator.estimate().time == timeBefore,
        "a refused input must not move the published time");
}

// A second sample at the same instant (the demo sends one when it rebuilds the
// controller) may refresh the published rate but not the stored history, or a
// later replay would use an input the forward pass never saw.
void testSameTimePredictKeepsHistory() {
    const auto p = parameters();
    for (const auto model : {truck_model::ArticulationProcessModel::kinematic,
                             truck_model::ArticulationProcessModel::dynamic}) {
        truck_model::ArticulationEstimatorConfig config;
        config.processModel = model;
        truck_model::ArticulationEstimator refreshed(p, config);
        truck_model::ArticulationEstimator plain(p, config);
        refreshed.reset(0.0, 0.02);
        plain.reset(0.0, 0.02);
        for (int step = 1; step <= 10; ++step) {
            const auto inputs = steadyInputs(
                0.05 * static_cast<double>(step), 0.05, 12.0, 0.01);
            refreshed.predict(inputs);
            plain.predict(inputs);
            if (step == 5) {
                const auto after = refreshed.predict(
                    steadyInputs(inputs.time, 0.20, 14.0, 0.05));
                require(
                    after.articulation == plain.estimate().articulation &&
                        after.covariancePhi == plain.estimate().covariancePhi,
                    "a same-time sample must not touch the state");
                require(
                    std::abs(
                        after.articulationRate -
                        plain.estimate().articulationRate) > 1.0e-3,
                    "the published rate must use the refreshed sample");
            }
        }
        // 0.235 s lies inside the interval whose input was refreshed.
        truck_model::ArticulationLidarMeasurement late;
        late.stamp = 0.235;
        late.articulation = 0.03;
        const auto a = refreshed.updateLidar(late);
        const auto b = plain.updateLidar(late);
        require(
            a.measurementAccepted && b.measurementAccepted,
            "the late scan must be accepted");
        require(
            a.articulation == b.articulation &&
                a.covariancePhi == b.covariancePhi,
            "the replay must use the input the forward pass used");
    }
}

// After a time jump trim() keeps the two newest frames, so the oldest frame
// alone would admit a stamp far older than historyHorizon.
void testHardWindowAfterTimeJump() {
    truck_model::ArticulationEstimator estimator(parameters());
    estimator.reset(0.0, 0.0);
    estimator.predict(steadyInputs(0.05, 0.02, 10.0));
    estimator.predict(steadyInputs(5.0, 0.02, 10.0));
    require(
        estimator.estimate().historySize == 2,
        "a time jump leaves the two newest frames");
    truck_model::ArticulationLidarMeasurement old;
    old.stamp = 0.06;
    old.articulation = 0.0;
    require(
        estimator.updateLidar(old).outcome ==
            truck_model::MeasurementOutcome::staleBeyondWindow,
        "a stamp older than historyHorizon must be stale even inside the "
        "stored span");
}

// A gated scan must leave nothing behind. The gate is evaluated on a detached
// split frame, so the history, and every later replay, is exactly what it
// would have been had the scan never arrived.
void testGatedSubFrameScanLeavesHistoryUntouched() {
    const auto p = parameters();
    for (const auto model : {truck_model::ArticulationProcessModel::kinematic,
                             truck_model::ArticulationProcessModel::dynamic}) {
        const auto run = [&p, model](bool withOutlier) {
            truck_model::ArticulationEstimatorConfig config;
            config.processModel = model;
            config.historyHorizon = 1.0;
            truck_model::ArticulationEstimator estimator(p, config);
            estimator.reset(0.0, 0.0);
            for (int step = 1; step <= 10; ++step) {
                estimator.predict(steadyInputs(
                    0.05 * static_cast<double>(step), 0.06, 12.0, 0.01));
            }
            if (withOutlier) {
                const auto before = estimator.estimate();
                truck_model::ArticulationLidarMeasurement outlier;
                outlier.stamp = 0.325;
                outlier.articulation = 0.8;
                const auto report = estimator.updateLidar(outlier);
                require(report.measurementGated, "the outlier must be gated");
                require(
                    report.historySize == before.historySize,
                    "a gated sub-frame scan must not add a frame");
                require(
                    report.articulation == before.articulation &&
                        report.covariancePhi == before.covariancePhi,
                    "a gated scan must not move the state");
            }
            truck_model::ArticulationLidarMeasurement good;
            good.stamp = 0.335;
            good.articulation = 0.03;
            const auto report = estimator.updateLidar(good);
            require(
                report.measurementAccepted, "the good scan must be accepted");
            return report;
        };
        const auto clean = run(false);
        const auto dirty = run(true);
        require(
            clean.historySize == dirty.historySize &&
                clean.articulation == dirty.articulation &&
                clean.covariancePhi == dirty.covariancePhi,
            "a gated scan must not change what a later update produces");
    }
}

void testUpdateRespectsMaximumFrames() {
    truck_model::ArticulationEstimatorConfig config;
    config.maximumFrames = 4;
    config.historyHorizon = 2.0;
    truck_model::ArticulationEstimator estimator(parameters(), config);
    estimator.reset(0.0, 0.0);
    for (int step = 1; step <= 3; ++step) {
        estimator.predict(
            steadyInputs(0.05 * static_cast<double>(step), 0.0, 10.0));
    }
    require(estimator.estimate().historySize == 4, "setup must fill the cap");
    for (const double stamp : {0.12, 0.13, 0.14}) {
        truck_model::ArticulationLidarMeasurement measurement;
        measurement.stamp = stamp;
        measurement.articulation = 0.0;
        const auto report = estimator.updateLidar(measurement);
        require(report.measurementAccepted, "in-window scan must be accepted");
        require(
            report.historySize <= 4,
            "a frame inserted by update must respect maximumFrames");
    }
}

// The coasting flag reports the open-loop condition itself, so it must not
// depend on whether the inflation factor happens to be above one.
void testCoastingFlagWithUnitNoiseScale() {
    truck_model::ArticulationEstimatorConfig config;
    config.coastingProcessNoiseScale = 1.0;
    config.lostTimeout = 0.2;
    truck_model::ArticulationEstimator estimator(parameters(), config);
    estimator.reset(0.0, 0.0);
    for (int step = 1; step <= 10; ++step) {
        estimator.predict(
            steadyInputs(0.05 * static_cast<double>(step), 0.02, 10.0));
    }
    require(
        estimator.estimate().coasting,
        "coasting must be reported even when the inflation factor is 1");
    require(
        !estimator.estimate().hasAcceptedMeasurement,
        "no scan has been accepted yet");
}

bool closeRelative(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <=
           tolerance * std::max(std::abs(actual), std::abs(expected)) +
               1.0e-300;
}

// Delayed fusion must reproduce, operation for operation, the EKF that fuses
// every scan the moment it is taken, on the input grid refined by the scan
// stamps. The run crosses speed buckets, mixes on-grid and between-frame
// stamps, and sends same-time input samples the history must ignore.
template <typename Model>
void checkDelayedFusionEqualsOnTime(
    const Model& model,
    const truck_model::Vector<Model::kStateSize>& initialState,
    const truck_model::Matrix<Model::kStateSize, Model::kStateSize>&
        initialCovariance,
    std::size_t phiIndex) {
    constexpr std::size_t n = Model::kStateSize;
    constexpr double dt = 0.05;
    constexpr int steps = 80;
    truck_model::DelayedEkfLimits limits;
    limits.historyHorizon = 0.6;
    truck_model::DelayedEkf<Model> delayed;
    truck_model::DelayedEkf<Model> onTime;
    delayed.configure(model, limits);
    onTime.configure(model, limits);
    delayed.reset(
        0.0, initialState, initialCovariance, truck_model::ArticulationInputs{});
    onTime.reset(
        0.0, initialState, initialCovariance, truck_model::ArticulationInputs{});

    const auto inputsAt = [](int k) {
        const double t = dt * static_cast<double>(k);
        return steadyInputs(
            t,
            0.08 * std::sin(2.0 * kPi * 0.4 * t),
            10.0 + 2.0 * static_cast<double>(k) / steps,
            0.03 * std::sin(2.0 * kPi * 0.4 * t + 0.3));
    };

    struct Scan {
        double stamp{};
        double arrival{};
        double value{};
    };
    std::vector<Scan> scans;
    double lastArrival = 0.0;
    const double phases[3] = {0.037, 0.013, 0.071};
    for (int j = 0; 0.1 * j + 0.6 < dt * steps; ++j) {
        Scan scan;
        scan.stamp = j % 4 == 1 ? dt * static_cast<double>(2 * j + 1)
                                : 0.1 * j + phases[j % 3];
        const double latency = 0.12 + 0.17 * std::fmod(0.618034 * j, 1.0);
        scan.arrival = std::max(scan.stamp + latency, lastArrival);
        lastArrival = scan.arrival;
        scans.push_back(scan);
    }

    std::size_t next = 0;
    for (int k = 1; k <= steps; ++k) {
        const auto u = inputsAt(k);
        while (next < scans.size() && scans[next].stamp <= u.time + 1.0e-12) {
            Scan& scan = scans[next];
            onTime.predict(u, std::min(scan.stamp, u.time));
            // The reference prior defines the value, so every innovation is
            // known, non-zero and inside the gate.
            scan.value =
                onTime.state()[phiIndex] + 0.01 * std::sin(7.0 * scan.stamp);
            require(
                onTime.update(scan.stamp, scan.value).outcome ==
                    truck_model::MeasurementOutcome::accepted,
                "reference scan must be accepted");
            ++next;
        }
        onTime.predict(u, u.time);
    }

    std::size_t delivered = 0;
    for (int k = 1; k <= steps; ++k) {
        const auto u = inputsAt(k);
        delayed.predict(u, u.time);
        if (k % 7 == 0) {
            delayed.predict(
                steadyInputs(
                    u.time,
                    u.truckYawRate + 0.1,
                    u.speed + 0.5,
                    u.steering - 0.02),
                u.time);
        }
        while (delivered < scans.size() &&
               scans[delivered].arrival <= u.time + 1.0e-12) {
            const Scan& scan = scans[delivered];
            const auto report = delayed.update(scan.stamp, scan.value);
            require(
                report.outcome == truck_model::MeasurementOutcome::accepted,
                "delayed scan must be accepted");
            require(
                std::abs(report.alignedStamp - scan.stamp) <= 1.0e-12,
                "delayed scan must land on its own stamp");
            require(
                report.repropagatedFrames > 0, "a late scan must be replayed");
            ++delivered;
        }
    }
    require(delivered == scans.size(), "every scan must arrive before the end");
    require(delayed.time() == onTime.time(), "both filters must end together");
    for (std::size_t row = 0; row < n; ++row) {
        require(
            closeRelative(delayed.state()[row], onTime.state()[row], 1.0e-12),
            "delayed fusion must equal on-time fusion: state");
        for (std::size_t column = 0; column < n; ++column) {
            require(
                closeRelative(
                    delayed.covariance()[row][column],
                    onTime.covariance()[row][column],
                    1.0e-12),
                "delayed fusion must equal on-time fusion: covariance");
        }
    }
}

void testDelayedFusionEqualsOnTimeFusion() {
    const auto p = parameters();
    const truck_model::ArticulationEstimatorConfig config;

    truck_model::Matrix<2, 2> kinematicCovariance{};
    kinematicCovariance[0][0] = config.initialArticulationVariance;
    kinematicCovariance[1][1] = config.initialTrailerYawBiasVariance;
    checkDelayedFusionEqualsOnTime(
        truck_model::KinematicArticulationModel(p, config),
        truck_model::Vector<2>{0.02, 0.0},
        kinematicCovariance,
        0);

    truck_model::Matrix<3, 3> dynamicCovariance{};
    dynamicCovariance[0][0] = config.initialTruckLateralVelocityVariance;
    dynamicCovariance[1][1] = config.initialTrailerYawRateVariance;
    dynamicCovariance[2][2] = config.initialArticulationVariance;
    checkDelayedFusionEqualsOnTime(
        truck_model::DynamicArticulationModel(p, config),
        truck_model::Vector<3>{0.0, 0.0, 0.02},
        dynamicCovariance,
        2);

    // Against the unrefined grid: the linear ZOH model composes exactly, so
    // splitting an interval changes nothing; the Euler kinematic mean does not.
    const auto u = steadyInputs(0.0, 0.07, 11.3, 0.02);
    const truck_model::DynamicArticulationModel dynamic(p, config);
    truck_model::Vector<3> xWhole{0.12, 0.03, 0.05};
    truck_model::Matrix<3, 3> pWhole = dynamicCovariance;
    pWhole[1][2] = -2.0e-4;
    pWhole[2][1] = -2.0e-4;
    auto xSplit = xWhole;
    auto pSplit = pWhole;
    dynamic.propagate(xWhole, pWhole, u, 0.05, 1.0);
    dynamic.propagate(xSplit, pSplit, u, 0.02, 1.0);
    dynamic.propagate(xSplit, pSplit, u, 0.03, 1.0);
    for (std::size_t row = 0; row < 3; ++row) {
        require(
            closeRelative(xSplit[row], xWhole[row], 1.0e-10),
            "a split dynamic interval must reproduce the whole one: state");
        for (std::size_t column = 0; column < 3; ++column) {
            require(
                std::abs(pSplit[row][column] - pWhole[row][column]) <=
                    1.0e-10 * std::abs(pWhole[row][column]) + 1.0e-16,
                "a split dynamic interval must reproduce the whole one: P");
        }
    }

    const truck_model::KinematicArticulationModel kinematic(p, config);
    truck_model::Vector<2> kWhole{0.1, 0.01};
    truck_model::Matrix<2, 2> kpWhole = kinematicCovariance;
    auto kSplit = kWhole;
    auto kpSplit = kpWhole;
    kinematic.propagate(kWhole, kpWhole, u, 0.05, 1.0);
    kinematic.propagate(kSplit, kpSplit, u, 0.02, 1.0);
    kinematic.propagate(kSplit, kpSplit, u, 0.03, 1.0);
    const double eulerGap = std::abs(kSplit[0] - kWhole[0]);
    require(
        eulerGap > 1.0e-9 && eulerGap < 1.0e-3,
        "a split kinematic interval differs from the whole one by O(dt^2)");
}

// The speed bucket is part of the replay contract, so the rounding rule is too:
// an exact midpoint rounds away from zero, as std::round does.
void testScheduledSpeedQuantization() {
    const truck_model::ArticulationEstimatorConfig config;
    require(
        truck_model::scheduledModelSpeed(10.125, config) == 10.25,
        "a midpoint between buckets must round away from zero");
    require(
        truck_model::scheduledModelSpeed(10.124, config) == 10.0,
        "just below the midpoint must round down");
    require(
        truck_model::scheduledModelSpeed(15.0, config) == 15.0,
        "an on-bucket speed must be kept");
    require(
        truck_model::scheduledModelSpeed(0.2, config) ==
                config.minimumModelSpeed &&
            truck_model::scheduledModelSpeed(-3.0, config) ==
                config.minimumModelSpeed,
        "low and reverse speeds must be scheduled at minimumModelSpeed");
}

// With this repository's axle loads the truck rear axle slips 1.74 times more
// than the trailer axle in a steady turn, so the trailer yaw residual opposes
// the trailer yaw rate: b_r2 = k_full * r2 with k_full < 0 (docs/3 5.5.1).
// The kinematic filter has to learn that sign from the plant.
void testTrailerYawResidualOpposesTrailerYawRate() {
    auto p = parameters();
    p.vx = 12.0;
    const double l1 = p.a1 + p.b1;
    const double l2 = p.a2 + p.b2;
    const double kFull =
        p.vx * p.vx / l2 *
        (p.a2 * p.m2 / (l2 * p.c2r) -
         (p.a1 * p.m1 * l2 + (p.a1 + p.d1) * p.b2 * p.m2) /
             (l1 * l2 * p.c1r));
    require(kFull < 0.0, "k_full must be negative for the nominal vehicle");

    constexpr double dt = 0.01;
    constexpr double steering = 0.01;
    const auto discrete =
        truck_model::discretizeZeroOrderHold(truck_model::buildDynamicModel(p), dt);
    truck_model::Vector<4> state{};
    truck_model::ArticulationEstimator estimator(p);
    estimator.reset(0.0, 0.0);
    for (int step = 1; step <= 3000; ++step) {
        auto next = truck_model::multiply(discrete.a, state);
        for (std::size_t i = 0; i < 4; ++i) {
            next[i] += discrete.b[i] * steering;
        }
        state = next;
        const double time = dt * static_cast<double>(step);
        estimator.predict(steadyInputs(time, state[1], p.vx, steering));
        if (step % 10 == 0) {
            truck_model::ArticulationLidarMeasurement scan;
            scan.stamp = time;
            scan.articulation = state[3];
            estimator.updateLidar(scan);
        }
    }

    const double r1 = state[1];
    const double r2 = state[2];
    const double residual =
        r2 - truck_model::kinematicTrailerYawRate(p, p.vx, r1, state[3]);
    require(r2 > 0.0, "positive steering must give a left turn");
    expectRelative(
        residual / r2, kFull, 0.02, "steady residual must follow k_full * r2");
    const double learned = estimator.estimate().trailerYawBias;
    require(
        learned < 0.0 && std::abs(learned - residual) < 0.05 * std::abs(residual),
        "the kinematic filter must learn a residual that opposes r2");
}

// Truth for the latency benchmark comes from the (K27) plant, not from the
// filter's own process model, so the comparison is not an inverse crime.
struct PlantTrace {
    std::vector<double> time;
    std::vector<double> articulation;
    std::vector<double> articulationRate;
    std::vector<double> truckYawRate;
    // Steering held over the step that ends at time[i], i.e. the input a
    // right-endpoint zero-order hold attaches to sample i.
    std::vector<double> steering;
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
    trace.steering.push_back(0.0);
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
        trace.steering.push_back(steering);
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
        inputs.steering = trace.steering[index];
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
    testPredictBeforeResetThrows();
    testResetRejectsInvalidCovariance();
    testOlderInputThrows();
    testSameTimePredictKeepsHistory();
    testHardWindowAfterTimeJump();
    testGatedSubFrameScanLeavesHistoryUntouched();
    testUpdateRespectsMaximumFrames();
    testCoastingFlagWithUnitNoiseScale();
    testDelayedFusionEqualsOnTimeFusion();
    testScheduledSpeedQuantization();
    testTrailerYawResidualOpposesTrailerYawRate();
    testDynamicModelImprovesRateTracking();
    testDynamicModelSurvivesParameterMismatch();
    testBothModelsBeatDelayedMeasurement();
    std::cout << "All articulated vehicle model tests passed.\n";
    return EXIT_SUCCESS;
}
