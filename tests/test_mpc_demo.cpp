#include "articulation_reference.hpp"
#include "demo_session.hpp"
#include "demo_paths.hpp"
#include "path_editor.hpp"
#include "session_log.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testSettingsCanRebuildModel() {
    truck_demo::DemoSession session;
    const auto initialTelemetryRevision = session.telemetryRevision();
    auto settings = session.settings();
    settings.vehicle.vx = 12.0;
    settings.mpc.horizon = 30;
    settings.mpc.stateWeight[0] = 24.0;
    settings.terminalWeightFactor = 3.0;
    session.configure(settings);
    require(
        std::abs(session.settings().vehicle.vx - 12.0) < 1.0e-12,
        "vehicle speed was not applied");
    require(
        session.settings().mpc.horizon == 30,
        "MPC horizon was not applied");
    require(
        std::abs(session.settings().mpc.terminalWeight[0] - 72.0) < 1.0e-12,
        "terminal weight factor was not applied");
    require(
        session.telemetryRevision() > initialTelemetryRevision,
        "model rebuild did not invalidate cached telemetry");
}

void testDemoPathPresetsStayCompatible() {
    require(
        truck_demo::kMaximumDrawnPathCurvature <
            truck_demo::kMaximumDemoCurvature,
        "drawn-path smoothing limit must remain below the demo safety limit");
    const auto scenario = truck_demo::highCurvatureScenarioPath();
    const auto legacyEntryPoint =
        truck_demo::DemoSession::highCurvaturePath();
    require(
        std::abs(scenario.length() - legacyEntryPoint.length()) < 1.0e-9,
        "high-curvature scenario entry points diverged");
}

void testDefaultCurvedPathClosedLoop() {
    truck_demo::DemoSession session;
    session.start();
    double maximumLateralError = 0.0;
    double maximumArticulation = 0.0;
    for (std::size_t step = 0; step < 500; ++step) {
        session.step();
        for (const double value : session.state()) {
            require(std::isfinite(value), "demo state became non-finite");
        }
        maximumLateralError =
            std::max(maximumLateralError, std::abs(session.state()[0]));
        maximumArticulation =
            std::max(maximumArticulation, std::abs(session.state()[4]));
        require(
            std::abs(session.steering()) <=
                session.settings().mpc.maxSteering + 1.0e-10,
            "demo steering limit was violated");
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(maximumLateralError < 2.0, "curved path lateral error is unbounded");
    require(maximumArticulation < 0.35, "curved path articulation is unbounded");
    require(session.history().size() > 100, "demo telemetry was not recorded");
    const auto& latest = session.history().back();
    for (const double value : latest.physicalState) {
        require(
            std::isfinite(value),
            "expanded physical-state telemetry became non-finite");
    }
    require(
        std::isfinite(latest.speed) &&
            std::isfinite(latest.referenceCurvature) &&
            std::isfinite(latest.referenceCurvatureRate),
        "expanded speed or curvature telemetry became non-finite");
    require(
        !session.predictedWorldPositions().empty(),
        "MPC world prediction was not generated");
}

void testDrawSmoothReviewConfirmWorkflow() {
    truck_demo::PathEditor editor;
    editor.begin();
    for (std::size_t i = 0; i <= 70; ++i) {
        const double x = 1.2 * static_cast<double>(i);
        const double noise = i % 2 == 0 ? 0.2 : -0.2;
        editor.addPoint({x, 2.5 * std::sin(0.045 * x) + noise});
    }
    editor.finish();
    require(
        editor.state() == truck_demo::PathEditorState::review,
        "drawn path did not enter explicit review state");
    require(
        !editor.smoothedPoints().empty(),
        "Pure Pursuit smoothing preview was not generated");
    bool rejected = false;
    try {
        editor.confirm(
            [](const truck_model::ReferencePath&) {
                throw std::invalid_argument("simulated feasibility failure");
            });
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(
        rejected &&
            editor.state() == truck_demo::PathEditorState::review &&
            !editor.previewPath().empty(),
        "failed path confirmation did not preserve the review preview");
    const auto confirmed = editor.confirm();
    require(
        editor.state() == truck_demo::PathEditorState::idle,
        "path editor did not leave review state after confirmation");
    require(
        confirmed.length() > 60.0,
        "confirmed path lost too much user-drawn coverage");
    truck_demo::DemoSession session;
    session.setPath(confirmed);
    require(
        session.path().length() > 60.0,
        "UI-equivalent path confirmation was rejected");
}

void testExtremeFeasibleCurvatureRemainsBounded() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.vehicle.vx = 8.0;
    settings.initialLateralError = 0.5;
    settings.initialHeadingError = 0.03;
    session.configure(settings);
    session.setPath(truck_demo::DemoSession::highCurvaturePath());
    session.start();
    double maximumLateralError = 0.0;
    double maximumArticulation = 0.0;
    for (std::size_t step = 0; step < 700; ++step) {
        session.step();
        maximumLateralError =
            std::max(maximumLateralError, std::abs(session.state()[0]));
        maximumArticulation =
            std::max(maximumArticulation, std::abs(session.state()[4]));
        require(
            session.simulationState() !=
                truck_demo::SimulationState::faulted,
            "feasible high-curvature path triggered safety stop");
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(
        maximumLateralError < 3.0,
        "feasible high-curvature path lateral error is unbounded");
    require(
        maximumArticulation < 0.5,
        "feasible high-curvature path articulation is unbounded");
    require(
        session.simulationState() ==
            truck_demo::SimulationState::finished,
        "feasible high-curvature path did not reach its endpoint");
}

void testUnsafeCurvatureIsRejectedForSpeed() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.adaptiveSpeedEnabled = false;
    session.configure(settings);
    bool rejected = false;
    try {
        session.setPath(
            truck_demo::curvatureWavePath(0.03, 70.0, 140.0));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(
        rejected,
        "dynamically infeasible high-speed path was not rejected");
}

void testCurvatureAdaptiveSpeedPlanning() {
    truck_demo::DemoSession session;
    session.setPath(
        truck_demo::curvatureWavePath(0.03, 70.0, 180.0));
    session.start();
    double minimumObservedSpeed = session.currentSpeed();
    for (std::size_t step = 0; step < 800; ++step) {
        session.step();
        minimumObservedSpeed =
            std::min(minimumObservedSpeed, session.currentSpeed());
        require(
            session.currentSpeed() >=
                session.settings().minimumSpeed - 1.0e-9,
            "adaptive speed dropped below configured minimum");
        require(
            session.currentSpeed() <=
                session.settings().vehicle.vx + 1.0e-9,
            "adaptive speed exceeded cruise speed");
        require(
            session.simulationState() !=
                truck_demo::SimulationState::faulted,
            "adaptive-speed high-curvature tracking became unstable");
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(
        minimumObservedSpeed < 12.0,
        "curvature planner did not reduce speed for the tight curve");
    require(
        session.simulationState() ==
            truck_demo::SimulationState::finished,
        "adaptive-speed trajectory did not reach its endpoint");
}

void testAdaptiveSpeedKeepsCruiseOnStraightPath() {
    truck_demo::DemoSession session;
    session.setPath(
        truck_demo::curvatureWavePath(0.0, 80.0, 120.0));
    require(
        std::abs(
            session.currentSpeed() -
            session.settings().vehicle.vx) < 1.0e-9,
        "adaptive planner reduced speed on a straight path");
}

void testInitialArticulationAndSteeringAreApplied() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.initialLateralError = 0.0;
    settings.initialLateralErrorRate = 0.0;
    settings.initialHeadingError = 0.0;
    settings.initialHeadingErrorRate = 0.0;
    settings.initialArticulation = 0.25;
    settings.initialArticulationRate = 0.0;
    settings.initialSteering = 0.1;
    session.configure(settings);
    require(
        std::abs(session.state()[4] - 0.25) < 1.0e-9,
        "initial articulation was not applied to the error state");
    require(
        std::abs(session.steering() - 0.1) < 1.0e-9,
        "initial steering was not applied");
    const auto vehicle = session.currentVehicle();
    const double articulationFromPose =
        vehicle.truckHeading - vehicle.trailerHeading;
    require(
        std::abs(articulationFromPose - 0.25) < 1.0e-6,
        "trailer heading is inconsistent with the initial articulation");
    require(
        session.warningReason().empty(),
        "a feasible initial articulation should not raise a range warning");
    session.start();
    const double initialArticulation = std::abs(session.state()[4]);
    for (std::size_t step = 0; step < 200; ++step) {
        session.step();
        require(
            session.simulationState() !=
                truck_demo::SimulationState::faulted,
            "articulation recovery experiment triggered a hard stop");
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(
        std::abs(session.state()[4]) < initialArticulation,
        "MPC did not reduce an isolated initial articulation");
}

void testLargeLateralErrorWarnsButContinues() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.initialLateralError = 13.0;
    settings.initialHeadingError = 0.0;
    settings.initialArticulation = 0.0;
    settings.initialSteering = 0.0;
    session.configure(settings);
    require(
        !session.warningReason().empty(),
        "a 13 m initial lateral error did not raise a warning");
    require(
        session.simulationState() !=
            truck_demo::SimulationState::faulted,
        "a large initial lateral error hard-stopped before running");
    session.start();
    for (std::size_t step = 0; step < 40; ++step) {
        session.step();
        require(
            session.simulationState() !=
                truck_demo::SimulationState::faulted,
            "large lateral error stopped the simulation");
        require(
            std::isfinite(session.state()[0]),
            "large-error simulation became non-finite");
    }
    require(
        session.history().back().warningActive,
        "telemetry did not record the active linear-range warning");
}

void testSineArticulationReferenceTracking() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.articulationTrackingExperiment = true;
    settings.articulationReference.kind =
        truck_demo::ArticulationReferenceKind::sine;
    settings.articulationReference.amplitude = 0.10;
    settings.articulationReference.frequency = 0.08;
    settings.articulationReference.duration = 20.0;
    session.configure(settings);
    session.setPath(truck_demo::curvatureWavePath(0.0, 80.0, 180.0));
    session.start();
    double squaredError = 0.0;
    double maximumPhi = 0.0;
    std::size_t count = 0;
    for (std::size_t step = 0; step < 200; ++step) {
        session.step();
        const double error =
            session.history().back().articulationTrackingError;
        squaredError += error * error;
        maximumPhi = std::max(maximumPhi, std::abs(session.state()[4]));
        ++count;
        require(
            session.simulationState() !=
                truck_demo::SimulationState::faulted,
            "sine articulation tracking faulted");
    }
    const double rms = std::sqrt(squaredError / static_cast<double>(count));
    require(
        rms < 0.4 * 0.10,
        "sine articulation tracking RMS error is too large");
    require(
        maximumPhi > 0.05,
        "sine articulation tracking did not move the hitch angle");
}

void testPeriodicStepArticulationReferenceSettles() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.articulationTrackingExperiment = true;
    settings.articulationReference.kind =
        truck_demo::ArticulationReferenceKind::periodicStep;
    settings.articulationReference.amplitude = 0.10;
    settings.articulationReference.period = 6.0;
    settings.articulationReference.dutyCycle = 0.5;
    settings.articulationReference.duration = 20.0;
    session.configure(settings);
    session.setPath(truck_demo::curvatureWavePath(0.0, 80.0, 180.0));
    session.start();
    for (std::size_t step = 0; step < 50; ++step) {
        session.step();
    }
    const double target = session.currentArticulationReference().value;
    require(
        std::abs(session.state()[4] - target) < 0.05,
        "periodic-step articulation reference did not settle");
}

void testNoneReferenceKeepsPathTrackingWeights() {
    truck_demo::DemoSession session;
    const auto original = session.settings().mpc.stateWeight;
    auto settings = session.settings();
    settings.articulationReference.kind =
        truck_demo::ArticulationReferenceKind::sine;
    settings.articulationTrackingExperiment = true;
    session.configure(settings);
    require(
        session.settings().mpc.stateWeight[0] == 0.0 &&
            session.settings().mpc.stateWeight[4] > original[4],
        "articulation experiment did not isolate path weights");
    session.endArticulationTrackingExperiment();
    require(
        session.settings().articulationReference.kind ==
            truck_demo::ArticulationReferenceKind::none,
        "leaving the experiment did not disable phi_ref");
    require(
        std::abs(session.settings().mpc.stateWeight[0] - original[0]) <
            1.0e-12,
        "leaving the experiment did not restore path Q");
}

void testDrawnArticulationReferenceSamples() {
    std::vector<truck_demo::TimeArticulationPoint> points;
    for (std::size_t i = 0; i <= 20; ++i) {
        const double time = 0.25 * static_cast<double>(i);
        points.push_back({time, 0.2 * std::sin(0.6 * time)});
    }
    const auto reference = truck_demo::ArticulationReference::fromDrawn(points);
    require(reference.config().duration >= 2.0, "drawn duration was lost");
    double previousTime = -1.0;
    for (const auto& point : reference.curve(0.1)) {
        require(point.time >= previousTime, "drawn curve time is not monotone");
        require(
            std::abs(point.articulation) <= 0.7,
            "drawn articulation exceeded the safety limit");
        previousTime = point.time;
    }
    require(
        std::abs(reference.sample(0.5).value) < 0.7,
        "drawn sample is out of range");
}

void testDrawnPathCurvatureIsSmooth() {
    truck_demo::PathEditor editor;
    editor.begin();
    for (std::size_t i = 0; i <= 140; ++i) {
        const double x = static_cast<double>(i);
        const double jitter = i % 2 == 0 ? 0.55 : -0.55;
        editor.addPoint(
            {x, 5.0 * std::sin(0.055 * x) + jitter});
    }
    editor.finish();
    const auto path = editor.confirm();
    double maximumCurvatureStep = 0.0;
    double maximumDerivativeStep = 0.0;
    for (std::size_t i = 1; i < path.points().size(); ++i) {
        maximumCurvatureStep = std::max(
            maximumCurvatureStep,
            std::abs(
                path.points()[i].curvature -
                path.points()[i - 1].curvature));
        maximumDerivativeStep = std::max(
            maximumDerivativeStep,
            std::abs(
                path.points()[i].curvatureDerivative -
                path.points()[i - 1].curvatureDerivative));
    }
    if (maximumCurvatureStep >= 0.0032 ||
        maximumDerivativeStep >= 0.004) {
        std::cerr << "curvature step=" << maximumCurvatureStep
                  << ", derivative step=" << maximumDerivativeStep << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void testLidarFusionTracksPlantArticulation() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarPeriod = 0.1;
    settings.lidarDelayMin = 0.2;
    settings.lidarDelayMax = 0.2;
    settings.lidarNoiseStd = 0.0;
    settings.adaptiveSpeedEnabled = false;
    session.configure(settings);
    session.start();

    double estimateSse = 0.0;
    double delayedSse = 0.0;
    int scoreCount = 0;
    for (std::size_t step = 0; step < 200; ++step) {
        session.step();
        require(
            std::isfinite(session.state()[4]) &&
                std::isfinite(session.state()[5]),
            "fused articulation became non-finite");
        if (session.history().empty()) {
            continue;
        }
        const auto& latest = session.history().back();
        if (latest.time < 0.6) {
            continue;
        }
        const double estimateError =
            latest.estimatedArticulation - latest.plantArticulation;
        const double delayedError =
            latest.lidarArticulation - latest.plantArticulation;
        estimateSse += estimateError * estimateError;
        delayedSse += delayedError * delayedError;
        ++scoreCount;
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(scoreCount > 20, "lidar fusion telemetry was too short");
    const double estimateRmse = std::sqrt(estimateSse / scoreCount);
    const double delayedRmse = std::sqrt(delayedSse / scoreCount);
    require(
        estimateRmse < 0.08,
        "fused articulation drifted too far from the plant");
    require(
        estimateRmse < delayedRmse + 1.0e-6,
        "delayed EKF was worse than the raw delayed lidar");
}

// A variable latency lets a later scan overtake an earlier one. The estimator
// has to reorder them by stamp and still converge.
void testOutOfOrderDeliveryStillTracks() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarPeriod = 0.1;
    settings.lidarDelayMin = 0.1;
    settings.lidarDelayMax = 0.5;
    settings.lidarNoiseStd = 0.0;
    settings.adaptiveSpeedEnabled = false;
    settings.articulationEstimator.historyHorizon = 0.7;
    session.configure(settings);
    session.start();

    std::size_t outOfOrder = 0;
    double previousStamp = -1.0;
    double estimateSse = 0.0;
    int scored = 0;
    for (std::size_t step = 0; step < 300; ++step) {
        session.step();
        if (session.history().empty()) {
            continue;
        }
        const auto& latest = session.history().back();
        if (latest.lidarDeliveredCount > 0) {
            if (previousStamp >= 0.0 && latest.lidarStamp < previousStamp) {
                ++outOfOrder;
            }
            previousStamp = latest.lidarStamp;
        }
        if (latest.time >= 1.0) {
            const double error =
                latest.estimatedArticulation - latest.plantArticulation;
            estimateSse += error * error;
            ++scored;
        }
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }

    require(
        outOfOrder > 0,
        "a 0.1-0.5 s latency spread must produce out-of-order arrivals");
    require(scored > 20, "out-of-order run produced too little telemetry");
    const double rmse = std::sqrt(estimateSse / scored);
    require(rmse < 0.05, "out-of-order arrivals broke articulation tracking");
}

// Without truth initialization and with noisy, biased sensors the filter still
// has to converge; that is the configuration a vehicle actually runs.
void testColdStartWithNoisySensors() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarNoiseStd = 0.0174;
    settings.initializeEstimatorFromTruth = false;
    settings.inputYawRateNoiseStd = 0.005;
    settings.inputYawRateBias = 0.004;
    settings.inputSpeedNoiseStd = 0.2;
    settings.initialArticulation = 0.12;
    settings.adaptiveSpeedEnabled = false;
    session.configure(settings);

    require(
        std::abs(session.articulationEstimate().articulation) < 1.0e-9,
        "a cold start must not seed the filter with the plant articulation");

    session.start();
    double estimateSse = 0.0;
    int scored = 0;
    for (std::size_t step = 0; step < 300; ++step) {
        session.step();
        const auto& latest = session.history().back();
        require(
            std::isfinite(latest.estimatedArticulation),
            "cold-start estimate became non-finite");
        if (latest.time >= 2.0) {
            const double error =
                latest.estimatedArticulation - latest.plantArticulation;
            estimateSse += error * error;
            ++scored;
        }
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(scored > 20, "cold-start run produced too little telemetry");
    require(
        std::sqrt(estimateSse / scored) < 0.06,
        "filter failed to converge from a cold start with noisy sensors");
}

// The shadow estimator sees the same events but never reaches the controller.
void testShadowEstimatorRunsInParallel() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarNoiseStd = 0.0;
    settings.adaptiveSpeedEnabled = false;
    settings.shadowEstimatorEnabled = true;
    settings.articulationEstimator.processModel =
        truck_model::ArticulationProcessModel::kinematic;
    settings.shadowProcessModel =
        truck_model::ArticulationProcessModel::dynamic;
    session.configure(settings);
    session.start();

    double primarySse = 0.0;
    double shadowSse = 0.0;
    int scored = 0;
    for (std::size_t step = 0; step < 300; ++step) {
        session.step();
        const auto& latest = session.history().back();
        if (latest.time >= 1.0) {
            const double primary =
                latest.estimatedArticulationRate - latest.plantArticulationRate;
            const double shadow =
                latest.shadowArticulationRate - latest.plantArticulationRate;
            primarySse += primary * primary;
            shadowSse += shadow * shadow;
            ++scored;
        }
        if (session.simulationState() ==
            truck_demo::SimulationState::finished) {
            break;
        }
    }
    require(scored > 20, "shadow run produced too little telemetry");
    require(
        std::sqrt(shadowSse / scored) < std::sqrt(primarySse / scored),
        "the dynamic shadow model must track the articulation rate better");
    require(
        std::abs(session.state()[4] -
                 session.articulationEstimate().articulation) < 1.0e-12,
        "the controller must consume the primary estimate, not the shadow");
}

// A replay window shorter than the worst-case latency silently discards scans,
// so the configuration has to be rejected up front.
void testHistoryHorizonMustCoverLatency() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarDelayMax = 0.5;
    settings.articulationEstimator.historyHorizon = 0.4;
    bool rejected = false;
    try {
        session.configure(settings);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(
        rejected,
        "history horizon below the worst-case latency must be rejected");
}

// An uncalibrated lidar mounting bias propagates straight into the estimate,
// which is why the bias is frozen at a calibration rather than fitted online.
void testLidarInstallationBiasNeedsCalibration() {
    const double bias = 0.0175;
    const auto run = [bias](double calibration) {
        truck_demo::DemoSession session;
        auto settings = session.settings();
        settings.lidarFusionEnabled = true;
        settings.lidarNoiseStd = 0.0;
        settings.adaptiveSpeedEnabled = false;
        settings.lidarInstallationBias = bias;
        settings.articulationEstimator.lidarBiasCalibration = calibration;
        session.configure(settings);
        session.start();
        double sse = 0.0;
        int scored = 0;
        for (std::size_t step = 0; step < 250; ++step) {
            session.step();
            const auto& latest = session.history().back();
            if (latest.time >= 2.0) {
                const double error =
                    latest.estimatedArticulation - latest.plantArticulation;
                sse += error * error;
                ++scored;
            }
            if (session.simulationState() ==
                truck_demo::SimulationState::finished) {
                break;
            }
        }
        return std::sqrt(sse / std::max(scored, 1));
    };

    const double uncalibrated = run(0.0);
    const double calibrated = run(bias);
    require(
        calibrated < 0.5 * uncalibrated,
        "calibrating the lidar mounting bias must remove most of the error");
}

void testSessionLogExportContainsFusionColumns() {
    truck_demo::DemoSession session;
    auto settings = session.settings();
    settings.lidarFusionEnabled = true;
    settings.lidarPeriod = 0.1;
    settings.lidarDelayMin = 0.2;
    settings.lidarDelayMax = 0.2;
    settings.lidarNoiseStd = 0.0;
    session.configure(settings);
    session.start();
    for (std::size_t step = 0; step < 40; ++step) {
        session.step();
    }

    const auto directory =
        std::filesystem::temp_directory_path() /
        "truck_model_session_log_test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    std::filesystem::create_directories(directory, error);
    require(!error, "could not create session log test directory");
    const auto writeError = session.exportRunLog(directory.string());
    require(writeError.empty(), writeError.c_str());

    std::ifstream settingsFile((directory / "settings.txt").string());
    require(static_cast<bool>(settingsFile), "settings.txt was not written");
    std::string settingsText(
        (std::istreambuf_iterator<char>(settingsFile)),
        std::istreambuf_iterator<char>());
    require(
        settingsText.find("lidarFusionEnabled=true") != std::string::npos,
        "settings.txt is missing lidarFusionEnabled");
    require(
        settingsText.find("mpc.Q_phi=") != std::string::npos,
        "settings.txt is missing MPC weights");
    require(
        settingsText.find("ekf.processModel=") != std::string::npos &&
            settingsText.find("ekf.noiseDensity.articulationRate=") !=
                std::string::npos &&
            settingsText.find("ekf.estimateLidarBias=") != std::string::npos,
        "settings.txt is missing the v2 estimator configuration");

    std::ifstream timeseries((directory / "timeseries.csv").string());
    require(static_cast<bool>(timeseries), "timeseries.csv was not written");
    std::string header;
    std::getline(timeseries, header);
    require(
        header.find("plant_phi") != std::string::npos &&
            header.find("ekf_phi") != std::string::npos &&
            header.find("lidar_z") != std::string::npos,
        "timeseries.csv is missing fusion columns");
    require(
        header.find("ekf_information_age") != std::string::npos &&
            header.find("ekf_arrival_gap") != std::string::npos &&
            header.find("ekf_link_stalled") != std::string::npos &&
            header.find("ekf_outcome") != std::string::npos,
        "timeseries.csv is missing the v2 diagnostic columns");
    require(
        header.find("lidar_delivered_count") != std::string::npos &&
            header.find("lidar_accepted_count") != std::string::npos &&
            header.find("lidar_gated_count") != std::string::npos &&
            header.find("lidar_dropped_count") != std::string::npos,
        "timeseries.csv is missing the per-step packet counters");
    require(
        header.find("shadow_phi") != std::string::npos &&
            header.find("sensed_r1") != std::string::npos,
        "timeseries.csv is missing the shadow and sensed-input columns");
    std::size_t rows = 0;
    std::string line;
    while (std::getline(timeseries, line)) {
        if (!line.empty()) {
            ++rows;
        }
    }
    require(
        rows == session.history().size(),
        "timeseries.csv row count does not match history");
    require(
        std::filesystem::exists(directory / "mpc_horizon.csv") &&
            std::filesystem::exists(directory / "path.csv") &&
            std::filesystem::exists(directory / "README.txt"),
        "session log is missing companion files");
    std::filesystem::remove_all(directory, error);
}

}  // namespace

int main() {
    try {
        testSettingsCanRebuildModel();
        testDemoPathPresetsStayCompatible();
        testDefaultCurvedPathClosedLoop();
        testDrawSmoothReviewConfirmWorkflow();
        testExtremeFeasibleCurvatureRemainsBounded();
        testUnsafeCurvatureIsRejectedForSpeed();
        testCurvatureAdaptiveSpeedPlanning();
        testAdaptiveSpeedKeepsCruiseOnStraightPath();
        testInitialArticulationAndSteeringAreApplied();
        testLargeLateralErrorWarnsButContinues();
        testSineArticulationReferenceTracking();
        testPeriodicStepArticulationReferenceSettles();
        testNoneReferenceKeepsPathTrackingWeights();
        testDrawnArticulationReferenceSamples();
        testDrawnPathCurvatureIsSmooth();
        testLidarFusionTracksPlantArticulation();
        testOutOfOrderDeliveryStillTracks();
        testColdStartWithNoisySensors();
        testShadowEstimatorRunsInParallel();
        testHistoryHorizonMustCoverLatency();
        testLidarInstallationBiasNeedsCalibration();
        testSessionLogExportContainsFusionColumns();
    } catch (const std::exception& error) {
        std::cerr << "Unexpected demo exception: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "All MPC demo tests passed.\n";
    return EXIT_SUCCESS;
}
