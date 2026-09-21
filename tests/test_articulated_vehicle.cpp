#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/lateral_mpc.hpp"
#include "truck_model/reference_path.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
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
    std::cout << "All articulated vehicle model tests passed.\n";
    return EXIT_SUCCESS;
}
