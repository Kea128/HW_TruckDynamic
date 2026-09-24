// Scratch measurement harness: runs the demo session under the nominal sensor
// and prints the accuracy numbers quoted in docs/3. Not part of the build.
#include "demo_paths.hpp"
#include "demo_session.hpp"

#include <cmath>
#include <cstdio>

namespace {

struct Score {
    double phiRmse{};
    double rateRmse{};
    double rawRmse{};
    std::size_t delivered{};
    std::size_t accepted{};
    std::size_t gated{};
    // Neither accepted nor gated. With the demo's validated window and unique,
    // input-covered stamps this can only be outOfOrder.
    std::size_t dropped{};
};

Score run(truck_model::ArticulationProcessModel model,
          double lidarNoiseStd,
          const truck_model::ArticulationNoiseDensities* densities = nullptr) {
    auto settings = truck_demo::DemoSession::fusionComparisonSettings();
    settings.articulationEstimator.processModel = model;
    if (densities != nullptr) {
        settings.articulationEstimator.noiseDensity = *densities;
    }
    settings.shadowEstimatorEnabled = false;
    settings.mpcUsesFusedArticulation = false;
    settings.lidarNoiseStd = lidarNoiseStd;
    settings.applyEstimatorMeasurementFromLidar();

    truck_demo::DemoSession session;
    session.configure(settings);
    session.start();
    for (std::size_t step = 0; step < 1200; ++step) {
        session.step();
        if (session.simulationState() != truck_demo::SimulationState::running) {
            break;
        }
    }

    double phiSse = 0.0;
    double rateSse = 0.0;
    double rawSse = 0.0;
    std::size_t scored = 0;
    std::size_t rawScored = 0;
    Score out;
    for (const auto& s : session.history()) {
        out.delivered += s.lidarDeliveredCount;
        out.accepted += s.lidarAcceptedCount;
        out.gated += s.lidarGatedCount;
        out.dropped += s.lidarDroppedCount;
        if (s.time < 1.0) {
            continue;
        }
        const double ePhi = s.estimatedArticulation - s.plantArticulation;
        const double eRate =
            s.estimatedArticulationRate - s.plantArticulationRate;
        phiSse += ePhi * ePhi;
        rateSse += eRate * eRate;
        ++scored;
        if (s.lidarDeliveredCount > 0) {
            const double eRaw = s.lidarArticulation - s.plantArticulation;
            rawSse += eRaw * eRaw;
            ++rawScored;
        }
    }
    out.phiRmse = std::sqrt(phiSse / static_cast<double>(scored));
    out.rateRmse = std::sqrt(rateSse / static_cast<double>(scored));
    out.rawRmse =
        rawScored > 0 ? std::sqrt(rawSse / static_cast<double>(rawScored)) : 0.0;
    return out;
}

constexpr double kDeg = 57.29577951308232;

struct Amplitudes {
    double plantOverReference{};
    double estimateOverPlant{};
};

// The articulation tracking experiment, with switch b deciding whether the
// controller closes the loop on the estimate or on the plant.
Amplitudes track(bool fusion,
                 bool closeLoopOnEstimate,
                 truck_model::ArticulationProcessModel model) {
    auto settings = truck_demo::DemoSession::fusionComparisonSettings();
    settings.articulationTrackingExperiment = true;
    settings.articulationReference.kind =
        truck_demo::ArticulationReferenceKind::sine;
    settings.articulationReference.amplitude = 0.12;
    settings.articulationReference.frequency = 0.12;
    settings.articulationReference.duration = 40.0;
    settings.lidarFusionEnabled = fusion;
    settings.mpcUsesFusedArticulation = closeLoopOnEstimate;
    settings.shadowEstimatorEnabled = false;
    settings.articulationEstimator.processModel = model;
    settings.applyEstimatorMeasurementFromLidar();

    truck_demo::DemoSession session;
    session.configure(settings);
    session.setPath(truck_demo::curvatureWavePath(0.0, 80.0, 400.0));
    session.beginArticulationTrackingExperiment();
    session.start();
    for (std::size_t step = 0; step < 1200; ++step) {
        session.step();
        if (session.simulationState() != truck_demo::SimulationState::running) {
            break;
        }
    }
    double reference = 0.0;
    double plant = 0.0;
    double estimate = 0.0;
    for (const auto& s : session.history()) {
        if (s.time < 8.5) {
            continue;
        }
        reference += s.referenceArticulation * s.referenceArticulation;
        plant += s.plantArticulation * s.plantArticulation;
        estimate += s.estimatedArticulation * s.estimatedArticulation;
    }
    Amplitudes a;
    a.plantOverReference = std::sqrt(plant / reference);
    a.estimateOverPlant = std::sqrt(estimate / plant);
    return a;
}

// How much does the trailer-bias random-walk density matter to the kinematic
// model? Sweep it and watch both channels, since raising it trades phi noise
// for phi-dot responsiveness.
void sweepTrailerBiasDensity() {
    // 2.56e-2 is the figure the OU match (F7a) gave before its operating
    // points were unified; it stays in the sweep as an order-of-magnitude
    // point above the useful range.
    const double values[] = {2.0e-5, 1.0e-4, 2.0e-4, 1.0e-3,
                             5.0e-3, 2.56e-2, 5.0e-2};
    std::printf("\nq_b [rad^2/s^3]   phi[deg]   phidot[deg/s]\n");
    for (const double q : values) {
        auto settings = truck_demo::DemoSession::fusionComparisonSettings();
        settings.articulationEstimator.processModel =
            truck_model::ArticulationProcessModel::kinematic;
        settings.articulationEstimator.noiseDensity.trailerYawBias = q;
        settings.shadowEstimatorEnabled = false;
        settings.mpcUsesFusedArticulation = false;
        settings.applyEstimatorMeasurementFromLidar();

        truck_demo::DemoSession session;
        session.configure(settings);
        session.start();
        for (std::size_t step = 0; step < 1200; ++step) {
            session.step();
            if (session.simulationState() !=
                truck_demo::SimulationState::running) {
                break;
            }
        }
        double phiSse = 0.0;
        double rateSse = 0.0;
        std::size_t scored = 0;
        for (const auto& s : session.history()) {
            if (s.time < 1.0) {
                continue;
            }
            const double ePhi = s.estimatedArticulation - s.plantArticulation;
            const double eRate =
                s.estimatedArticulationRate - s.plantArticulationRate;
            phiSse += ePhi * ePhi;
            rateSse += eRate * eRate;
            ++scored;
        }
        const double n = static_cast<double>(scored);
        std::printf("%14.2e   %8.3f   %13.3f\n", q,
                    std::sqrt(phiSse / n) * kDeg,
                    std::sqrt(rateSse / n) * kDeg);
    }
}

}  // namespace

int main() {
    // 0.5 deg is the demo's nominal scan; 4 deg is an engineering comparison
    // level the README mentions for simulation, not a calibrated perception
    // figure. Both are reported so docs/3 cannot show only the flattering one.
    for (const double noiseDegrees : {0.5, 2.0, 4.0}) {
        const double sigma = noiseDegrees / kDeg;
        const auto k = run(truck_model::ArticulationProcessModel::kinematic,
                           sigma);
        const auto d = run(truck_model::ArticulationProcessModel::dynamic,
                           sigma);
        std::printf("\nlidar noise %.1f deg\n", noiseDegrees);
        std::printf("model        phi[deg]  phidot[deg/s]\n");
        std::printf("kinematic    %8.3f  %13.3f\n", k.phiRmse * kDeg,
                    k.rateRmse * kDeg);
        std::printf("dynamic      %8.3f  %13.3f\n", d.phiRmse * kDeg,
                    d.rateRmse * kDeg);
        std::printf("raw lidar    %8.3f  %13s\n", k.rawRmse * kDeg, "n/a");
        for (const auto* s : {&k, &d}) {
            const double n = static_cast<double>(s->delivered);
            std::printf(
                "%-10s   delivered %4zu  accepted %5.1f%%  gated %4.1f%%  "
                "dropped(outOfOrder) %4.1f%%\n",
                s == &k ? "kinematic" : "dynamic",
                s->delivered,
                100.0 * static_cast<double>(s->accepted) / n,
                100.0 * static_cast<double>(s->gated) / n,
                100.0 * static_cast<double>(s->dropped) / n);
        }
    }

    // The dynamic-model densities: defaults versus the 2 sigma^2 tau_c values
    // their rationale gives.
    std::printf("\ndynamic densities (q_vy1, q_r2)   noise  phi[deg]  phidot[deg/s]\n");
    for (const double noiseDegrees : {0.5, 4.0}) {
        truck_model::ArticulationNoiseDensities rationale;
        rationale.truckLateralVelocity = 0.036;
        rationale.trailerYawRate = 1.2e-4;
        const auto nominal = run(truck_model::ArticulationProcessModel::dynamic,
                                 noiseDegrees / kDeg);
        const auto derived = run(truck_model::ArticulationProcessModel::dynamic,
                                 noiseDegrees / kDeg, &rationale);
        std::printf("default  (0.05, 1.5e-4)          %4.1f   %7.3f  %13.3f\n",
                    noiseDegrees, nominal.phiRmse * kDeg,
                    nominal.rateRmse * kDeg);
        std::printf("2s^2tau  (0.036, 1.2e-4)         %4.1f   %7.3f  %13.3f\n",
                    noiseDegrees, derived.phiRmse * kDeg,
                    derived.rateRmse * kDeg);
    }

    using PM = truck_model::ArticulationProcessModel;
    const auto truth = track(false, false, PM::kinematic);
    const auto kinOpen = track(true, false, PM::kinematic);
    const auto kinClosed = track(true, true, PM::kinematic);
    const auto dynClosed = track(true, true, PM::dynamic);
    std::printf("\ntracking experiment      plant/ref   est/plant\n");
    std::printf("b off, no fusion        %9.3f   %9.3f\n",
                truth.plantOverReference, truth.estimateOverPlant);
    std::printf("b off, K5 filter runs   %9.3f   %9.3f\n",
                kinOpen.plantOverReference, kinOpen.estimateOverPlant);
    std::printf("b on,  K5 in the loop   %9.3f   %9.3f\n",
                kinClosed.plantOverReference, kinClosed.estimateOverPlant);
    std::printf("b on,  K27r in the loop %9.3f   %9.3f\n",
                dynClosed.plantOverReference, dynClosed.estimateOverPlant);
    sweepTrailerBiasDensity();
    return 0;
}
