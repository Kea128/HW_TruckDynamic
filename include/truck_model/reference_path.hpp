#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace truck_model {

struct Point2d {
    double x{};
    double y{};
};

struct ReferencePathPoint {
    double s{};
    double x{};
    double y{};
    double heading{};
    double curvature{};
    double curvatureDerivative{};
};

struct PathBuildConfig {
    double sampleSpacing{0.5};
    double minimumInputSpacing{0.35};
    std::size_t smoothingPasses{3};
    // Positive values fit a uniform cubic B-spline whose control points use
    // approximately this arc-length spacing. Zero disables spline fitting.
    double splineControlSpacing{0.0};

    [[nodiscard]] std::string validationError() const;
};

struct PurePursuitSmoothingConfig {
    double lookaheadDistance{7.0};
    double outputSpacing{0.35};
    double maximumCurvature{0.065};
    double curvatureResponse{0.18};

    [[nodiscard]] std::string validationError() const;
};

// Uses Pure Pursuit geometry as an offline path preprocessor. A virtual point
// follows the raw polyline and emits a smoother path. This function never
// computes or modifies the truck steering command.
[[nodiscard]] std::vector<Point2d> smoothWaypointsPurePursuit(
    const std::vector<Point2d>& waypoints,
    PurePursuitSmoothingConfig config = {});

// Arc-length parameterized path built from user-provided waypoints. Input
// points are filtered, corner-cut for C1-like visual continuity, uniformly
// resampled, and differentiated to provide curvature preview for MPC.
class ReferencePath {
public:
    [[nodiscard]] static ReferencePath fromWaypoints(
        const std::vector<Point2d>& waypoints,
        PathBuildConfig config = {});

    [[nodiscard]] ReferencePathPoint sample(double s) const;
    // Finds the closest point inside a monotonic arc-length search interval.
    [[nodiscard]] ReferencePathPoint project(
        Point2d position,
        double minimumS,
        double maximumS) const;
    [[nodiscard]] const std::vector<ReferencePathPoint>& points() const noexcept;
    [[nodiscard]] double length() const noexcept;
    [[nodiscard]] bool empty() const noexcept;

private:
    std::vector<ReferencePathPoint> points_;
};

}  // namespace truck_model
