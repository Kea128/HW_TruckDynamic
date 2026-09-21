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
    void plotArticulationCell(
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
    std::vector<double> plantArticulation_;
    std::vector<double> lidarArticulation_;
    std::vector<double> articulationTrackingError_;
    std::vector<double> truckYawRate_;
    std::vector<double> trailerYawRate_;
    std::vector<double> steering_;
    std::vector<double> speed_;
    std::vector<double> curvature_;
    std::vector<double> curvatureRate_;
    std::size_t cachedTelemetryRevision_{
        std::numeric_limits<std::size_t>::max()};
};

}  // namespace truck_demo
