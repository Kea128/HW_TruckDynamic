#pragma once

#include "truck_model/reference_path.hpp"

namespace truck_demo {

inline constexpr double kMaximumDemoCurvature = 0.08;
inline constexpr double kMaximumDrawnPathCurvature = 0.075;
inline constexpr double kModelSpeedScheduleThreshold = 0.25;

[[nodiscard]] truck_model::PurePursuitSmoothingConfig
drawnPathSmoothingConfig();
[[nodiscard]] truck_model::PathBuildConfig drawnPathBuildConfig();
[[nodiscard]] truck_model::ReferencePath defaultScenarioPath();
[[nodiscard]] truck_model::ReferencePath highCurvatureScenarioPath();
[[nodiscard]] truck_model::ReferencePath curvatureWavePath(
    double maximumCurvature,
    double wavelength,
    double length);

}  // namespace truck_demo
