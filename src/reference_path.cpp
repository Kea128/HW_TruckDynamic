#include "truck_model/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace truck_model {
namespace {

double distance(const Point2d& lhs, const Point2d& rhs) {
    return std::hypot(rhs.x - lhs.x, rhs.y - lhs.y);
}

Point2d interpolate(const Point2d& lhs, const Point2d& rhs, double ratio) {
    return {
        lhs.x + ratio * (rhs.x - lhs.x),
        lhs.y + ratio * (rhs.y - lhs.y)};
}

std::vector<Point2d> filterWaypoints(
    const std::vector<Point2d>& input,
    double minimumSpacing) {
    std::vector<Point2d> filtered;
    for (const auto& point : input) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            throw std::invalid_argument("path waypoints must be finite");
        }
        if (filtered.empty() ||
            distance(filtered.back(), point) >= minimumSpacing) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<Point2d> smoothPolyline(
    std::vector<Point2d> points,
    std::size_t passes) {
    for (std::size_t pass = 0; pass < passes; ++pass) {
        std::vector<Point2d> next;
        next.reserve(points.size() * 2);
        next.push_back(points.front());
        for (std::size_t i = 0; i + 1 < points.size(); ++i) {
            next.push_back(interpolate(points[i], points[i + 1], 0.25));
            next.push_back(interpolate(points[i], points[i + 1], 0.75));
        }
        next.push_back(points.back());
        points = std::move(next);
    }
    return points;
}

std::vector<Point2d> resample(
    const std::vector<Point2d>& points,
    double spacing) {
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + distance(points[i - 1], points[i]);
    }
    const double totalLength = cumulative.back();
    if (totalLength < 2.0 * spacing) {
        throw std::invalid_argument("drawn path is too short");
    }

    const std::size_t sampleCount = std::max(
        std::size_t{3},
        static_cast<std::size_t>(std::ceil(totalLength / spacing)) + 1);
    const double actualSpacing =
        totalLength / static_cast<double>(sampleCount - 1);
    std::vector<Point2d> result;
    result.reserve(sampleCount);
    std::size_t segment = 0;
    for (std::size_t i = 0; i < sampleCount; ++i) {
        const double target =
            std::min(totalLength, static_cast<double>(i) * actualSpacing);
        while (segment + 1 < cumulative.size() - 1 &&
               cumulative[segment + 1] < target) {
            ++segment;
        }
        const double segmentLength =
            cumulative[segment + 1] - cumulative[segment];
        const double ratio =
            segmentLength > 0.0
                ? (target - cumulative[segment]) / segmentLength
                : 0.0;
        result.push_back(
            interpolate(points[segment], points[segment + 1], ratio));
    }
    return result;
}

std::vector<Point2d> sampleUniformCubicBSpline(
    const std::vector<Point2d>& points,
    double controlSpacing,
    double outputSpacing) {
    const auto controls = resample(points, controlSpacing);
    std::vector<Point2d> extended;
    extended.reserve(controls.size() + 2);
    extended.push_back(
        {2.0 * controls.front().x - controls[1].x,
         2.0 * controls.front().y - controls[1].y});
    extended.insert(extended.end(), controls.begin(), controls.end());
    extended.push_back(
        {2.0 * controls.back().x - controls[controls.size() - 2].x,
         2.0 * controls.back().y - controls[controls.size() - 2].y});

    const std::size_t subdivisions = std::max(
        std::size_t{3},
        static_cast<std::size_t>(
            std::ceil(controlSpacing / outputSpacing)));
    std::vector<Point2d> curve;
    curve.reserve((extended.size() - 3) * subdivisions + 1);
    for (std::size_t segment = 0;
         segment + 3 < extended.size();
         ++segment) {
        for (std::size_t sample = 0; sample < subdivisions; ++sample) {
            const double t =
                static_cast<double>(sample) /
                static_cast<double>(subdivisions);
            const double t2 = t * t;
            const double t3 = t2 * t;
            const double weights[4] = {
                (1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0,
                (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0,
                (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0,
                t3 / 6.0};
            Point2d point{};
            for (std::size_t i = 0; i < 4; ++i) {
                point.x += weights[i] * extended[segment + i].x;
                point.y += weights[i] * extended[segment + i].y;
            }
            curve.push_back(point);
        }
    }
    curve.push_back(controls.back());
    return curve;
}

void smoothScalar(std::vector<double>& values, std::size_t passes) {
    if (values.size() < 3) {
        return;
    }
    for (std::size_t pass = 0; pass < passes; ++pass) {
        const auto source = values;
        values.front() = 0.75 * source.front() + 0.25 * source[1];
        for (std::size_t i = 1; i + 1 < values.size(); ++i) {
            values[i] = 0.25 * source[i - 1] +
                        0.5 * source[i] +
                        0.25 * source[i + 1];
        }
        values.back() =
            0.25 * source[values.size() - 2] + 0.75 * source.back();
    }
}

Point2d samplePolyline(
    const std::vector<Point2d>& points,
    const std::vector<double>& cumulative,
    double distanceAlongPath) {
    const double target =
        std::clamp(distanceAlongPath, 0.0, cumulative.back());
    const auto upper = std::lower_bound(
        cumulative.begin(), cumulative.end(), target);
    if (upper == cumulative.begin()) {
        return points.front();
    }
    if (upper == cumulative.end()) {
        return points.back();
    }
    const std::size_t after =
        static_cast<std::size_t>(upper - cumulative.begin());
    const std::size_t before = after - 1;
    const double segment = cumulative[after] - cumulative[before];
    const double ratio =
        segment > 0.0 ? (target - cumulative[before]) / segment : 0.0;
    return interpolate(points[before], points[after], ratio);
}

double nearestPolylineProgress(
    const Point2d& position,
    const std::vector<Point2d>& points,
    const std::vector<double>& cumulative,
    std::size_t& nearestSegment) {
    double bestSquaredDistance = std::numeric_limits<double>::infinity();
    double bestProgress = cumulative[nearestSegment];
    std::size_t bestSegment = nearestSegment;
    for (std::size_t segment = nearestSegment;
         segment + 1 < points.size();
         ++segment) {
        const double dx = points[segment + 1].x - points[segment].x;
        const double dy = points[segment + 1].y - points[segment].y;
        const double squaredLength = dx * dx + dy * dy;
        if (!(squaredLength > 0.0)) {
            continue;
        }
        const double ratio = std::clamp(
            ((position.x - points[segment].x) * dx +
             (position.y - points[segment].y) * dy) /
                squaredLength,
            0.0,
            1.0);
        const double projectedX = points[segment].x + ratio * dx;
        const double projectedY = points[segment].y + ratio * dy;
        const double errorX = position.x - projectedX;
        const double errorY = position.y - projectedY;
        const double squaredDistance =
            errorX * errorX + errorY * errorY;
        if (squaredDistance < bestSquaredDistance) {
            bestSquaredDistance = squaredDistance;
            bestSegment = segment;
            bestProgress =
                cumulative[segment] + ratio * std::sqrt(squaredLength);
        }
    }
    nearestSegment = bestSegment;
    return bestProgress;
}

}  // namespace

std::string PathBuildConfig::validationError() const {
    std::ostringstream errors;
    if (!(sampleSpacing > 0.0) || !std::isfinite(sampleSpacing)) {
        errors << "sampleSpacing must be finite and > 0; ";
    }
    if (!(minimumInputSpacing > 0.0) ||
        !std::isfinite(minimumInputSpacing)) {
        errors << "minimumInputSpacing must be finite and > 0; ";
    }
    if (smoothingPasses > 8) {
        errors << "smoothingPasses must be <= 8; ";
    }
    if (splineControlSpacing < 0.0 ||
        !std::isfinite(splineControlSpacing)) {
        errors << "splineControlSpacing must be finite and >= 0; ";
    } else if (
        splineControlSpacing > 0.0 &&
        splineControlSpacing < sampleSpacing) {
        errors << "splineControlSpacing must be >= sampleSpacing; ";
    }
    return errors.str();
}

std::string PurePursuitSmoothingConfig::validationError() const {
    std::ostringstream errors;
    if (!(lookaheadDistance > 0.0) ||
        !std::isfinite(lookaheadDistance)) {
        errors << "lookaheadDistance must be finite and > 0; ";
    }
    if (!(outputSpacing > 0.0) || !std::isfinite(outputSpacing)) {
        errors << "outputSpacing must be finite and > 0; ";
    }
    if (!(maximumCurvature > 0.0) ||
        !std::isfinite(maximumCurvature)) {
        errors << "maximumCurvature must be finite and > 0; ";
    }
    if (!(curvatureResponse > 0.0 && curvatureResponse <= 1.0) ||
        !std::isfinite(curvatureResponse)) {
        errors << "curvatureResponse must be within (0, 1]; ";
    }
    return errors.str();
}

std::vector<Point2d> smoothWaypointsPurePursuit(
    const std::vector<Point2d>& waypoints,
    PurePursuitSmoothingConfig config) {
    const auto error = config.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
    const auto points =
        filterWaypoints(waypoints, 0.5 * config.outputSpacing);
    if (points.size() < 2) {
        throw std::invalid_argument(
            "draw at least two separated path points");
    }
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        cumulative[i] =
            cumulative[i - 1] + distance(points[i - 1], points[i]);
    }
    const double totalLength = cumulative.back();
    if (totalLength < 2.0 * config.lookaheadDistance) {
        throw std::invalid_argument(
            "drawn path must be at least twice the Pure Pursuit lookahead");
    }

    Point2d position = points.front();
    const auto initialTarget =
        samplePolyline(points, cumulative, config.lookaheadDistance);
    double heading = std::atan2(
        initialTarget.y - position.y, initialTarget.x - position.x);
    double progress = 0.0;
    double filteredCurvature = 0.0;
    std::size_t nearestSegment = 0;
    std::vector<Point2d> result{position};
    const std::size_t maximumSteps = static_cast<std::size_t>(
        std::ceil(
            (totalLength + 4.0 * config.lookaheadDistance) /
            config.outputSpacing));

    for (std::size_t stepIndex = 0;
         stepIndex < maximumSteps;
         ++stepIndex) {
        progress = std::max(
            progress,
            nearestPolylineProgress(
                position, points, cumulative, nearestSegment));
        const auto target = samplePolyline(
            points,
            cumulative,
            std::min(totalLength, progress + config.lookaheadDistance));
        const double dx = target.x - position.x;
        const double dy = target.y - position.y;
        const double targetDistance = std::hypot(dx, dy);
        const double localY =
            -std::sin(heading) * dx + std::cos(heading) * dy;
        const double localX =
            std::cos(heading) * dx + std::sin(heading) * dy;
        if (progress >= totalLength - config.outputSpacing &&
            (targetDistance <= config.outputSpacing ||
             (localX <= 0.0 &&
              targetDistance <= 2.0 * config.outputSpacing))) {
            break;
        }
        const double denominator =
            std::max(targetDistance * targetDistance, 1.0e-9);
        const double requestedCurvature = std::clamp(
            2.0 * localY / denominator,
            -config.maximumCurvature,
            config.maximumCurvature);
        filteredCurvature += config.curvatureResponse *
                             (requestedCurvature - filteredCurvature);
        const double integrationStep =
            progress >= totalLength - config.lookaheadDistance
                ? std::min(
                      config.outputSpacing,
                      std::max(
                          0.12 * config.outputSpacing,
                          0.3 * targetDistance))
                : config.outputSpacing;
        const double middleHeading =
            heading + 0.5 * filteredCurvature * integrationStep;
        position.x += integrationStep * std::cos(middleHeading);
        position.y += integrationStep * std::sin(middleHeading);
        heading += filteredCurvature * integrationStep;
        result.push_back(position);
    }
    if (distance(result.back(), points.back()) >
        2.0 * config.lookaheadDistance) {
        throw std::invalid_argument(
            "Pure Pursuit smoother could not converge to the path endpoint");
    }
    return result;
}

ReferencePath ReferencePath::fromWaypoints(
    const std::vector<Point2d>& waypoints,
    PathBuildConfig config) {
    const auto error = config.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
    auto filtered = filterWaypoints(waypoints, config.minimumInputSpacing);
    if (filtered.size() < 2) {
        throw std::invalid_argument("draw at least two separated path points");
    }
    filtered = smoothPolyline(std::move(filtered), config.smoothingPasses);
    double filteredLength = 0.0;
    for (std::size_t i = 1; i < filtered.size(); ++i) {
        filteredLength += distance(filtered[i - 1], filtered[i]);
    }
    if (config.splineControlSpacing > 0.0 &&
        filteredLength >= 2.0 * config.splineControlSpacing) {
        filtered = sampleUniformCubicBSpline(
            filtered,
            config.splineControlSpacing,
            std::min(
                config.sampleSpacing,
                0.25 * config.splineControlSpacing));
    }
    const auto sampled = resample(filtered, config.sampleSpacing);

    ReferencePath path;
    path.points_.resize(sampled.size());
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        path.points_[i].x = sampled[i].x;
        path.points_[i].y = sampled[i].y;
        if (i > 0) {
            path.points_[i].s =
                path.points_[i - 1].s + distance(sampled[i - 1], sampled[i]);
        }
    }

    std::vector<double> headings(sampled.size(), 0.0);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after =
            std::min(i + 1, sampled.size() - 1);
        headings[i] = std::atan2(
            sampled[after].y - sampled[before].y,
            sampled[after].x - sampled[before].x);
        if (i > 0) {
            while (headings[i] - headings[i - 1] > 3.14159265358979323846) {
                headings[i] -= 2.0 * 3.14159265358979323846;
            }
            while (headings[i] - headings[i - 1] <
                   -3.14159265358979323846) {
                headings[i] += 2.0 * 3.14159265358979323846;
            }
        }
        path.points_[i].heading = headings[i];
    }

    std::vector<double> curvatures(sampled.size(), 0.0);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after =
            std::min(i + 1, sampled.size() - 1);
        const double ds =
            path.points_[after].s - path.points_[before].s;
        if (ds > 0.0) {
            curvatures[i] = (headings[after] - headings[before]) / ds;
        }
    }
    smoothScalar(curvatures, 16);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        path.points_[i].curvature = curvatures[i];
    }
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        const std::size_t before = i == 0 ? 0 : i - 1;
        const std::size_t after =
            std::min(i + 1, sampled.size() - 1);
        const double ds =
            path.points_[after].s - path.points_[before].s;
        if (ds > 0.0) {
            path.points_[i].curvatureDerivative =
                (curvatures[after] - curvatures[before]) / ds;
        }
    }
    std::vector<double> curvatureDerivatives(sampled.size(), 0.0);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        curvatureDerivatives[i] =
            path.points_[i].curvatureDerivative;
    }
    smoothScalar(curvatureDerivatives, 10);
    for (std::size_t i = 0; i < sampled.size(); ++i) {
        path.points_[i].curvatureDerivative =
            curvatureDerivatives[i];
    }
    return path;
}

ReferencePathPoint ReferencePath::sample(double s) const {
    if (points_.empty()) {
        throw std::logic_error("cannot sample an empty reference path");
    }
    const double clamped = std::clamp(s, 0.0, length());
    const auto upper = std::lower_bound(
        points_.begin(),
        points_.end(),
        clamped,
        [](const ReferencePathPoint& point, double value) {
            return point.s < value;
        });
    if (upper == points_.begin()) {
        return points_.front();
    }
    if (upper == points_.end()) {
        return points_.back();
    }
    const auto& before = *(upper - 1);
    const double interval = upper->s - before.s;
    if (!(interval > 0.0) || !std::isfinite(interval)) {
        throw std::runtime_error(
            "reference path contains a non-increasing arc-length interval");
    }
    const double ratio = (clamped - before.s) / interval;
    ReferencePathPoint result;
    result.s = clamped;
    const auto blend = [ratio](double lhs, double rhs) {
        return lhs + ratio * (rhs - lhs);
    };
    result.x = blend(before.x, upper->x);
    result.y = blend(before.y, upper->y);
    result.heading = blend(before.heading, upper->heading);
    result.curvature = blend(before.curvature, upper->curvature);
    result.curvatureDerivative =
        blend(before.curvatureDerivative, upper->curvatureDerivative);
    return result;
}

ReferencePathPoint ReferencePath::project(
    Point2d position,
    double minimumS,
    double maximumS) const {
    if (points_.size() < 2) {
        throw std::logic_error(
            "cannot project onto a path with fewer than two points");
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(minimumS) || !std::isfinite(maximumS)) {
        throw std::invalid_argument("path projection inputs must be finite");
    }
    const double lower = std::clamp(minimumS, 0.0, length());
    const double upper = std::clamp(
        std::max(minimumS, maximumS), lower, length());
    double bestS = lower;
    double bestSquaredDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
        const auto& first = points_[i];
        const auto& second = points_[i + 1];
        if (second.s < lower || first.s > upper) {
            continue;
        }
        const double dx = second.x - first.x;
        const double dy = second.y - first.y;
        const double squaredLength = dx * dx + dy * dy;
        if (!(squaredLength > 0.0)) {
            continue;
        }
        const double minimumRatio =
            std::clamp((lower - first.s) / (second.s - first.s), 0.0, 1.0);
        const double maximumRatio =
            std::clamp((upper - first.s) / (second.s - first.s), 0.0, 1.0);
        const double ratio = std::clamp(
            ((position.x - first.x) * dx +
             (position.y - first.y) * dy) /
                squaredLength,
            minimumRatio,
            maximumRatio);
        const double projectedX = first.x + ratio * dx;
        const double projectedY = first.y + ratio * dy;
        const double errorX = position.x - projectedX;
        const double errorY = position.y - projectedY;
        const double squaredDistance =
            errorX * errorX + errorY * errorY;
        if (squaredDistance < bestSquaredDistance) {
            bestSquaredDistance = squaredDistance;
            bestS = first.s + ratio * (second.s - first.s);
        }
    }
    if (!std::isfinite(bestSquaredDistance)) {
        throw std::runtime_error(
            "path projection interval contains no valid segment");
    }
    return sample(bestS);
}

const std::vector<ReferencePathPoint>& ReferencePath::points() const noexcept {
    return points_;
}

double ReferencePath::length() const noexcept {
    return points_.empty() ? 0.0 : points_.back().s;
}

bool ReferencePath::empty() const noexcept {
    return points_.empty();
}

}  // namespace truck_model
