#pragma once

#include "demo_session.hpp"

#include "imgui.h"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace truck_demo {

class TelemetryPanel {
public:
    void render(
        const DemoSession& session,
        bool darkTheme,
        const ImVec2& available);

private:
    void rebuildPlotData(const DemoSession& session);
    void renderStateSummary(
        const DemoSession& session,
        bool darkTheme);
    void summaryMetric(
        bool darkTheme,
        const char* label,
        double value,
        const char* unit,
        const char* format);
    [[nodiscard]] bool beginPlotGrid(const char* id);
    void beginPlotCell();
    [[nodiscard]] float plotHeight() const;
    [[nodiscard]] float currentMarkerSize(float plotHeight) const;
    void plotArticulationCell(
        const DemoSession& session,
        bool darkTheme);
    // phiDot is reconstructed from the model, never measured, so it needs the
    // same truth-versus-estimate treatment as the angle itself.
    void plotArticulationRateCell(
        const DemoSession& session,
        bool darkTheme);
    void plotCell(
        const DemoSession& session,
        bool darkTheme,
        const char* id,
        const char* title,
        const char* yLabel,
        const std::vector<double>& values,
        double minimum,
        double maximum,
        const char* valueFormat);

    std::vector<double> time_;
    std::vector<double> lateralError_;
    std::vector<double> lateralErrorRate_;
    std::vector<double> headingError_;
    std::vector<double> headingErrorRate_;
    std::vector<double> articulation_;
    std::vector<double> articulationReference_;
    std::vector<double> articulationRate_;
    std::vector<double> plantArticulationRate_;
    std::vector<double> shadowArticulationRate_;
    std::vector<double> plantArticulation_;
    std::vector<double> shadowArticulation_;
    std::vector<double> lidarArticulation_;
    std::vector<double> articulationTrackingError_;
    std::vector<double> truckYawRate_;
    std::vector<double> trailerYawRate_;
    std::vector<double> steering_;
    std::vector<double> speed_;
    std::vector<double> curvature_;
    std::vector<double> curvatureRate_;
    // Running accuracy against the plant, in degrees, over the scored window.
    // Shown next to the articulation plot so the comparison is a number and not
    // an eyeball judgement of two overlapping curves.
    double primaryRmseDeg_{};
    double shadowRmseDeg_{};
    double lidarRmseDeg_{};
    double primaryRateRmseDeg_{};
    double shadowRateRmseDeg_{};
    bool comparisonValid_{};
    // Top of the current grid cell, so plot height can be derived from the row
    // height instead of from whatever space happens to be left in the window.
    float cellTop_{};
    std::size_t cachedTelemetryRevision_{
        std::numeric_limits<std::size_t>::max()};
    float plotRowHeight_{220.0f};
    int plotCellsInRow_{};
};

}  // namespace truck_demo
