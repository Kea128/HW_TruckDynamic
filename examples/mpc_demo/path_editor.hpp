#pragma once

#include "truck_model/reference_path.hpp"

#include <functional>
#include <vector>

namespace truck_demo {

enum class PathEditorState {
    idle,
    drawing,
    review
};

class PathEditor {
public:
    void begin();
    void addPoint(truck_model::Point2d point);
    void finish();
    [[nodiscard]] truck_model::ReferencePath confirm();
    void confirm(
        const std::function<void(
            const truck_model::ReferencePath&)>& commit);
    void cancel();

    [[nodiscard]] PathEditorState state() const noexcept;
    [[nodiscard]] const std::vector<truck_model::Point2d>&
    rawPoints() const noexcept;
    [[nodiscard]] const std::vector<truck_model::Point2d>&
    smoothedPoints() const noexcept;
    [[nodiscard]] const truck_model::ReferencePath&
    previewPath() const noexcept;

private:
    PathEditorState state_{PathEditorState::idle};
    std::vector<truck_model::Point2d> rawPoints_;
    std::vector<truck_model::Point2d> smoothedPoints_;
    truck_model::ReferencePath preview_;
};

}  // namespace truck_demo
