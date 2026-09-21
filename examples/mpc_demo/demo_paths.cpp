#include "demo_paths.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace truck_demo {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

truck_model::PurePursuitSmoothingConfig drawnPathSmoothingConfig() {
    truck_model::PurePursuitSmoothingConfig config;
    config.lookaheadDistance = 7.0;
    config.outputSpacing = 0.3;
    config.maximumCurvature = kMaximumDrawnPathCurvature;
    config.curvatureResponse = 0.18;
    return config;
}

truck_model::PathBuildConfig drawnPathBuildConfig() {
    truck_model::PathBuildConfig config;
    config.sampleSpacing = 0.3;
    config.minimumInputSpacing = 0.2;
    config.smoothingPasses = 0;
    config.splineControlSpacing = 3.0;
    return config;
}

truck_model::ReferencePath defaultScenarioPath() {
    std::vector<truck_model::Point2d> waypoints;
    for (double x = 0.0; x <= 240.0; x += 3.0) {
        const double y = 5.0 * std::sin(2.0 * kPi * x / 180.0);
        waypoints.push_back({x, y});
    }
    truck_model::PathBuildConfig config;
    config.smoothingPasses = 2;
    return truck_model::ReferencePath::fromWaypoints(waypoints, config);
}

truck_model::ReferencePath highCurvatureScenarioPath() {
    return curvatureWavePath(0.025, 80.0, 180.0);
}

truck_model::ReferencePath curvatureWavePath(
    double maximumCurvature,
    double wavelength,
    double length) {
    if (!(maximumCurvature >= 0.0) ||
        !std::isfinite(maximumCurvature) ||
        !(wavelength > 0.0) || !std::isfinite(wavelength) ||
        !(length > 0.0) || !std::isfinite(length)) {
        throw std::invalid_argument(
            "curvature-wave parameters must be finite and non-negative");
    }
    std::vector<truck_model::Point2d> waypoints;
    constexpr double spacing = 0.25;
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    waypoints.push_back({x, y});
    for (double s = spacing; s <= length; s += spacing) {
        const double middleS = s - 0.5 * spacing;
        const double curvature =
            maximumCurvature *
            std::sin(2.0 * kPi * middleS / wavelength);
        heading += curvature * spacing;
        x += spacing * std::cos(heading);
        y += spacing * std::sin(heading);
        waypoints.push_back({x, y});
    }
    truck_model::PathBuildConfig config;
    config.sampleSpacing = spacing;
    config.minimumInputSpacing = 0.1;
    config.smoothingPasses = 0;
    config.splineControlSpacing = 0.0;
    return truck_model::ReferencePath::fromWaypoints(waypoints, config);
}

}  // namespace truck_demo
