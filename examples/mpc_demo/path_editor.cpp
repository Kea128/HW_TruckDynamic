#include "path_editor.hpp"

#include "demo_paths.hpp"

#include <cmath>
#include <stdexcept>

namespace truck_demo {

void PathEditor::begin() {
    rawPoints_.clear();
    smoothedPoints_.clear();
    preview_ = {};
    state_ = PathEditorState::drawing;
}

void PathEditor::addPoint(truck_model::Point2d point) {
    if (state_ != PathEditorState::drawing) {
        return;
    }
    if (rawPoints_.empty() ||
        std::hypot(
            point.x - rawPoints_.back().x,
            point.y - rawPoints_.back().y) >= 0.2) {
        rawPoints_.push_back(point);
    }
}

void PathEditor::finish() {
    if (state_ != PathEditorState::drawing) {
        throw std::logic_error("path editor is not in drawing mode");
    }
    auto smoothed =
        truck_model::smoothWaypointsPurePursuit(
            rawPoints_, drawnPathSmoothingConfig());

    auto preview =
        truck_model::ReferencePath::fromWaypoints(
            smoothed, drawnPathBuildConfig());
    smoothedPoints_.clear();
    smoothedPoints_.reserve(preview.points().size());
    for (const auto& point : preview.points()) {
        smoothedPoints_.push_back({point.x, point.y});
    }
    preview_ = std::move(preview);
    state_ = PathEditorState::review;
}

truck_model::ReferencePath PathEditor::confirm() {
    if (state_ != PathEditorState::review || preview_.empty()) {
        throw std::logic_error(
            "finish and review the smoothed path before confirming it");
    }
    auto confirmed = preview_;
    cancel();
    return confirmed;
}

void PathEditor::confirm(
    const std::function<void(
        const truck_model::ReferencePath&)>& commit) {
    if (state_ != PathEditorState::review || preview_.empty()) {
        throw std::logic_error(
            "finish and review the smoothed path before confirming it");
    }
    if (!commit) {
        throw std::invalid_argument(
            "path confirmation callback must be valid");
    }
    commit(preview_);
    cancel();
}

void PathEditor::cancel() {
    rawPoints_.clear();
    smoothedPoints_.clear();
    preview_ = {};
    state_ = PathEditorState::idle;
}

PathEditorState PathEditor::state() const noexcept {
    return state_;
}

const std::vector<truck_model::Point2d>&
PathEditor::rawPoints() const noexcept {
    return rawPoints_;
}

const std::vector<truck_model::Point2d>&
PathEditor::smoothedPoints() const noexcept {
    return smoothedPoints_;
}

const truck_model::ReferencePath&
PathEditor::previewPath() const noexcept {
    return preview_;
}

}  // namespace truck_demo
