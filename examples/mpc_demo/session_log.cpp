#include "session_log.hpp"

#include "demo_session.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace truck_demo {
namespace {

std::string csvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (char ch : text) {
        if (ch == '"') {
            escaped += "\"\"";
        } else {
            escaped += ch;
        }
    }
    escaped += '"';
    return escaped;
}

const char* processModelName(truck_model::ArticulationProcessModel model) {
    switch (model) {
        case truck_model::ArticulationProcessModel::dynamic:
            return "dynamic";
        case truck_model::ArticulationProcessModel::kinematic:
        default:
            return "kinematic";
    }
}

const char* referenceKindName(ArticulationReferenceKind kind) {
    switch (kind) {
        case ArticulationReferenceKind::sine:
            return "sine";
        case ArticulationReferenceKind::periodicStep:
            return "periodicStep";
        case ArticulationReferenceKind::drawn:
            return "drawn";
        case ArticulationReferenceKind::none:
        default:
            return "none";
    }
}

void writeKey(
    std::ostream& out, const char* key, const std::string& value) {
    out << key << '=' << value << '\n';
}

void writeKey(std::ostream& out, const char* key, double value) {
    out << key << '=' << std::setprecision(12) << value << '\n';
}

void writeKey(std::ostream& out, const char* key, int value) {
    out << key << '=' << value << '\n';
}

void writeKey(std::ostream& out, const char* key, std::size_t value) {
    out << key << '=' << value << '\n';
}

void writeKey(std::ostream& out, const char* key, bool value) {
    out << key << '=' << (value ? "true" : "false") << '\n';
}

std::string settingsText(const DemoSession& session) {
    const auto& settings = session.settings();
    const auto& p = settings.vehicle;
    const auto& mpc = settings.mpc;
    const auto& ekf = session.estimatorConfig();
    std::ostringstream out;
    out << std::setprecision(12);
    writeKey(out, "vehicle.m1", p.m1);
    writeKey(out, "vehicle.iz1", p.iz1);
    writeKey(out, "vehicle.a1", p.a1);
    writeKey(out, "vehicle.b1", p.b1);
    writeKey(out, "vehicle.c1f", p.c1f);
    writeKey(out, "vehicle.c1r", p.c1r);
    writeKey(out, "vehicle.m2", p.m2);
    writeKey(out, "vehicle.iz2", p.iz2);
    writeKey(out, "vehicle.a2", p.a2);
    writeKey(out, "vehicle.b2", p.b2);
    writeKey(out, "vehicle.c2r", p.c2r);
    writeKey(out, "vehicle.d1", p.d1);
    writeKey(out, "vehicle.vx", p.vx);
    writeKey(out, "mpc.sampleTime", mpc.sampleTime);
    writeKey(out, "mpc.horizon", mpc.horizon);
    writeKey(out, "mpc.Q_ey", mpc.stateWeight[0]);
    writeKey(out, "mpc.Q_eyDot", mpc.stateWeight[1]);
    writeKey(out, "mpc.Q_epsi", mpc.stateWeight[2]);
    writeKey(out, "mpc.Q_epsiDot", mpc.stateWeight[3]);
    writeKey(out, "mpc.Q_phi", mpc.stateWeight[4]);
    writeKey(out, "mpc.Q_phiDot", mpc.stateWeight[5]);
    writeKey(out, "mpc.Qf_ey", mpc.terminalWeight[0]);
    writeKey(out, "mpc.Qf_eyDot", mpc.terminalWeight[1]);
    writeKey(out, "mpc.Qf_epsi", mpc.terminalWeight[2]);
    writeKey(out, "mpc.Qf_epsiDot", mpc.terminalWeight[3]);
    writeKey(out, "mpc.Qf_phi", mpc.terminalWeight[4]);
    writeKey(out, "mpc.Qf_phiDot", mpc.terminalWeight[5]);
    writeKey(out, "mpc.R", mpc.steeringWeight);
    writeKey(out, "mpc.Rd", mpc.steeringRateWeight);
    writeKey(out, "mpc.maxSteering", mpc.maxSteering);
    writeKey(out, "mpc.maxSteeringRate", mpc.maxSteeringRate);
    writeKey(out, "terminalWeightFactor", settings.terminalWeightFactor);
    writeKey(out, "initialLateralError", settings.initialLateralError);
    writeKey(out, "initialLateralErrorRate", settings.initialLateralErrorRate);
    writeKey(out, "initialHeadingError", settings.initialHeadingError);
    writeKey(out, "initialHeadingErrorRate", settings.initialHeadingErrorRate);
    writeKey(out, "initialArticulation", settings.initialArticulation);
    writeKey(out, "initialArticulationRate", settings.initialArticulationRate);
    writeKey(out, "initialSteering", settings.initialSteering);
    writeKey(out, "adaptiveSpeedEnabled", settings.adaptiveSpeedEnabled);
    writeKey(out, "minimumSpeed", settings.minimumSpeed);
    writeKey(out, "maximumLateralAcceleration", settings.maximumLateralAcceleration);
    writeKey(out, "maximumLateralJerk", settings.maximumLateralJerk);
    writeKey(out, "maximumAcceleration", settings.maximumAcceleration);
    writeKey(out, "maximumDeceleration", settings.maximumDeceleration);
    writeKey(out, "speedLookaheadDistance", settings.speedLookaheadDistance);
    writeKey(
        out,
        "articulationTrackingExperiment",
        settings.articulationTrackingExperiment);
    writeKey(
        out,
        "articulationReference.kind",
        std::string(referenceKindName(settings.articulationReference.kind)));
    writeKey(out, "lidarFusionEnabled", settings.lidarFusionEnabled);
    writeKey(out, "lidarPeriod", settings.lidarPeriod);
    writeKey(out, "lidarDelayMin", settings.lidarDelayMin);
    writeKey(out, "lidarDelayMax", settings.lidarDelayMax);
    writeKey(out, "lidarNoiseStd", settings.lidarNoiseStd);
    writeKey(out, "lidarRandomSeed", static_cast<int>(settings.lidarRandomSeed));
    writeKey(out, "inputYawRateNoiseStd", settings.inputYawRateNoiseStd);
    writeKey(out, "inputSpeedNoiseStd", settings.inputSpeedNoiseStd);
    writeKey(out, "inputRandomSeed", static_cast<int>(settings.inputRandomSeed));
    writeKey(
        out,
        "mpcUsesFusedArticulation",
        settings.mpcUsesFusedArticulation);
    writeKey(out, "shadowEstimatorEnabled", settings.shadowEstimatorEnabled);
    writeKey(
        out,
        "shadow.processModel",
        std::string(processModelName(settings.shadowProcessModel)));
    writeKey(
        out,
        "ekf.processModel",
        std::string(processModelName(ekf.processModel)));
    writeKey(out, "ekf.historyHorizon", ekf.historyHorizon);
    writeKey(out, "ekf.measurementVariance", ekf.measurementVariance);
    // Process noise is specified as continuous power spectral density; see
    // docs/3 chapter 8.
    writeKey(
        out,
        "ekf.noiseDensity.articulationRate",
        ekf.noiseDensity.articulationRate);
    writeKey(
        out,
        "ekf.noiseDensity.trailerYawBias",
        ekf.noiseDensity.trailerYawBias);
    writeKey(out, "ekf.noiseDensity.truckYawRate", ekf.noiseDensity.truckYawRate);
    writeKey(out, "ekf.noiseDensity.speed", ekf.noiseDensity.speed);
    writeKey(
        out,
        "ekf.noiseDensity.truckLateralVelocity",
        ekf.noiseDensity.truckLateralVelocity);
    writeKey(
        out,
        "ekf.noiseDensity.trailerYawRate",
        ekf.noiseDensity.trailerYawRate);
    writeKey(out, "ekf.mahalanobisGate", ekf.mahalanobisGate);
    writeKey(out, "ekf.consecutiveRejectLimit", ekf.consecutiveRejectLimit);
    writeKey(out, "ekf.lostTimeout", ekf.lostTimeout);
    writeKey(
        out, "ekf.coastingProcessNoiseScale", ekf.coastingProcessNoiseScale);
    writeKey(out, "ekf.linkTimeout", ekf.linkTimeout);
    writeKey(out, "ekf.maximumFrames", ekf.maximumFrames);
    writeKey(out, "ekf.snapToNearestFrame", ekf.snapToNearestFrame);
    writeKey(
        out,
        "ekf.initialArticulationVariance",
        ekf.initialArticulationVariance);
    writeKey(
        out,
        "ekf.initialTrailerYawBiasVariance",
        ekf.initialTrailerYawBiasVariance);
    writeKey(
        out,
        "ekf.initialTruckLateralVelocityVariance",
        ekf.initialTruckLateralVelocityVariance);
    writeKey(
        out,
        "ekf.initialTrailerYawRateVariance",
        ekf.initialTrailerYawRateVariance);
    writeKey(out, "scheduledModelSpeed", session.scheduledModelSpeed());
    writeKey(out, "sampleCount", session.history().size());
    if (session.controller() != nullptr) {
        const auto& ad = session.controller()->discreteA();
        const auto& bd = session.controller()->discreteB();
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t column = 0; column < 6; ++column) {
                std::ostringstream key;
                key << "mpc.Ad_" << row << column;
                writeKey(out, key.str().c_str(), ad[row][column]);
            }
            std::ostringstream bKey;
            bKey << "mpc.Bd_" << row;
            writeKey(out, bKey.str().c_str(), bd[row]);
        }
    }
    return out.str();
}

void writeTimeseries(std::ostream& out, const DemoSession& session) {
    out << "time,distance,"
        << "ey,ey_dot,epsi,epsi_dot,ctrl_phi,ctrl_phi_dot,"
        << "ctrl_ey,ctrl_ey_dot,ctrl_epsi,ctrl_epsi_dot,ctrl_phi_in,ctrl_phi_dot_in,"
        << "plant_vy1,plant_r1,plant_r2,plant_vy2,plant_phi,plant_phi_dot,"
        << "delta,delta_unconstrained,"
        << "speed,planned_speed,cruise_speed,scheduled_model_speed,models_rebuilt,"
        << "rho,rho_dot,phi_ref,phi_ref_dot,phi_tracking_error,"
        << "ekf_phi,ekf_phi_dot,ekf_r2,ekf_r2_kin,ekf_b_r2,"
        << "ekf_vy1,ekf_truck_yaw_residual,ekf_P_phi,ekf_P_br2,"
        << "ekf_innovation,ekf_S,ekf_mahalanobis,"
        << "ekf_K_phi,ekf_K_br2,"
        << "ekf_history_size,ekf_replayed,ekf_aligned_stamp,"
        << "ekf_last_accepted_stamp,ekf_information_age,ekf_arrival_gap,"
        << "ekf_consecutive_rejects,ekf_accepted,ekf_gated,ekf_coasting,"
        << "ekf_link_stalled,ekf_has_accepted,ekf_outcome,"
        << "shadow_phi,shadow_phi_dot,shadow_r2,shadow_b_r2,shadow_P_phi,"
        << "sensed_r1,sensed_speed,"
        << "lidar_z,lidar_stamp,lidar_delay,lidar_queue,lidar_delivered,"
        << "lidar_accepted,lidar_gated,"
        << "lidar_delivered_count,lidar_accepted_count,lidar_gated_count,"
        << "lidar_dropped_count,"
        << "truck_x,truck_y,truck_heading,hitch_x,hitch_y,"
        << "trailer_x,trailer_y,trailer_heading,warning_active,warning\n";
    out << std::setprecision(12);
    for (const auto& sample : session.history()) {
        const auto& e = sample.estimator;
        out << sample.time << ',' << sample.distance << ','
            << sample.state[0] << ',' << sample.state[1] << ','
            << sample.state[2] << ',' << sample.state[3] << ','
            << sample.state[4] << ',' << sample.state[5] << ','
            << sample.controllerState[0] << ',' << sample.controllerState[1] << ','
            << sample.controllerState[2] << ',' << sample.controllerState[3] << ','
            << sample.controllerState[4] << ',' << sample.controllerState[5] << ','
            << sample.plantVy1 << ',' << sample.plantR1 << ','
            << sample.plantR2 << ',' << sample.plantVy2 << ','
            << sample.plantPhi << ',' << sample.plantPhiDot << ','
            << sample.steering << ',' << sample.unconstrainedSteering << ','
            << sample.speed << ',' << sample.plannedSpeed << ','
            << sample.cruiseSpeed << ',' << sample.scheduledModelSpeed << ','
            << (sample.modelsRebuilt ? 1 : 0) << ','
            << sample.referenceCurvature << ',' << sample.referenceCurvatureRate
            << ',' << sample.referenceArticulation << ','
            << sample.referenceArticulationRate << ','
            << sample.articulationTrackingError << ','
            << sample.estimatedArticulation << ','
            << sample.estimatedArticulationRate << ','
            << e.trailerYawRate << ',' << e.kinematicTrailerYawRate << ','
            << e.trailerYawBias << ','
            << e.truckLateralVelocity << ',' << e.truckYawResidual << ','
            << e.covariancePhi << ','
            << e.covarianceTrailerBias << ','
            << e.innovation << ',' << e.innovationCovariance << ','
            << e.mahalanobis << ',' << e.kalmanGainPhi << ','
            << e.kalmanGainTrailerBias << ','
            << e.historySize << ',' << e.repropagatedFrames << ','
            << e.alignedStamp << ','
            << e.lastAcceptedStamp << ',' << e.informationAge << ','
            << e.arrivalGap << ',' << e.consecutiveRejects << ','
            << (e.measurementAccepted ? 1 : 0) << ','
            << (e.measurementGated ? 1 : 0) << ','
            << (e.coasting ? 1 : 0) << ','
            << (e.linkStalled ? 1 : 0) << ','
            << (e.hasAcceptedMeasurement ? 1 : 0) << ','
            << truck_model::measurementOutcomeName(e.outcome) << ','
            << sample.shadowArticulation << ','
            << sample.shadowArticulationRate << ','
            << sample.shadowEstimator.trailerYawRate << ','
            << sample.shadowEstimator.trailerYawBias << ','
            << sample.shadowEstimator.covariancePhi << ','
            << sample.measuredTruckYawRate << ',' << sample.measuredSpeed << ','
            << sample.lidarArticulation << ',' << sample.lidarStamp << ','
            << sample.lidarDelay << ',' << sample.lidarQueueSize << ','
            << (sample.lidarDelivered ? 1 : 0) << ','
            << (sample.lidarAccepted ? 1 : 0) << ','
            << (sample.lidarGated ? 1 : 0) << ','
            << sample.lidarDeliveredCount << ','
            << sample.lidarAcceptedCount << ','
            << sample.lidarGatedCount << ','
            << sample.lidarDroppedCount << ','
            << sample.vehicle.truckX << ',' << sample.vehicle.truckY << ','
            << sample.vehicle.truckHeading << ',' << sample.vehicle.hitchX << ','
            << sample.vehicle.hitchY << ',' << sample.vehicle.trailerX << ','
            << sample.vehicle.trailerY << ',' << sample.vehicle.trailerHeading
            << ',' << (sample.warningActive ? 1 : 0) << ','
            << csvEscape(sample.warning) << '\n';
    }
}

void writeHorizon(std::ostream& out, const DemoSession& session) {
    out << "time,k,ey,ey_dot,epsi,epsi_dot,phi,phi_dot\n";
    out << std::setprecision(12);
    for (const auto& sample : session.history()) {
        for (std::size_t k = 0; k < sample.predictedStates.size(); ++k) {
            const auto& x = sample.predictedStates[k];
            out << sample.time << ',' << k << ','
                << x[0] << ',' << x[1] << ',' << x[2] << ','
                << x[3] << ',' << x[4] << ',' << x[5] << '\n';
        }
    }
}

void writePath(std::ostream& out, const DemoSession& session) {
    out << "s,x,y,heading,curvature,curvature_derivative\n";
    out << std::setprecision(12);
    for (const auto& point : session.path().points()) {
        out << point.s << ',' << point.x << ',' << point.y << ','
            << point.heading << ',' << point.curvature << ','
            << point.curvatureDerivative << '\n';
    }
}

const char* kReadme = R"(TruckModel demo run log
See docs/3_articulation_fusion_filter.md section 10.

Files:
  settings.txt     vehicle, MPC Q/R, lidar, EKF, discrete Ad/Bd
  timeseries.csv   one row per control step (20 Hz default)
  mpc_horizon.csv  predicted x_c at each horizon index k
  path.csv         reference path samples

Articulation amplitude:
  plant_phi, ekf_phi, lidar_z
  plant_r2, ekf_r2, ekf_r2_kin
  plant_phi_dot, ekf_phi_dot        rate is a first-class metric, not optional

Shadow model (second estimator, never drives the controller):
  shadow_phi, shadow_phi_dot, shadow_r2, shadow_b_r2

Filter health:
  ekf_innovation, ekf_S, ekf_mahalanobis
  ekf_information_age   age of the newest fused scan
  ekf_arrival_gap       filter time since the last accepted update
  ekf_coasting          open loop: information age or counted rejections
  ekf_link_stalled      arrival gap above linkTimeout, whatever the cause
  ekf_has_accepted      0 until the first scan is accepted after reset
  ekf_replayed          timeline entries recomputed by the last update

Lidar packets. The *_count columns are per control step; a step can service
several scans, so the boolean columns and ekf_outcome only describe the last
one.
  lidar_delay, lidar_queue, ekf_outcome
  lidar_delivered_count, lidar_accepted_count
  lidar_gated_count, lidar_dropped_count

Sensed inputs actually fed to the filter (noisy when injection is enabled):
  sensed_r1, sensed_speed
)";

}  // namespace

std::string makeRunDirectory(const std::string& parent) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &time);
#else
    localtime_r(&time, &local);
#endif
    std::ostringstream name;
    name << std::put_time(&local, "%Y%m%d_%H%M%S");
    const std::filesystem::path directory =
        std::filesystem::path(parent) / name.str();
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        return {};
    }
    return directory.string();
}

std::string writeSessionLog(
    const std::string& directory,
    const DemoSession& session) {
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if (error) {
        return "cannot create log directory: " + error.message();
    }
    const std::filesystem::path root(directory);

    std::ofstream settings((root / "settings.txt").string());
    if (!settings) {
        return "cannot write settings.txt";
    }
    settings << settingsText(session);

    std::ofstream timeseries((root / "timeseries.csv").string());
    if (!timeseries) {
        return "cannot write timeseries.csv";
    }
    writeTimeseries(timeseries, session);

    std::ofstream horizon((root / "mpc_horizon.csv").string());
    if (!horizon) {
        return "cannot write mpc_horizon.csv";
    }
    writeHorizon(horizon, session);

    std::ofstream path((root / "path.csv").string());
    if (!path) {
        return "cannot write path.csv";
    }
    writePath(path, session);

    std::ofstream readme((root / "README.txt").string());
    if (!readme) {
        return "cannot write README.txt";
    }
    readme << kReadme;
    return {};
}

}  // namespace truck_demo
