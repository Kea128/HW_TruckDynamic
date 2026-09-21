#include "telemetry_panel.hpp"

#include "implot.h"

#include <algorithm>
#include <cmath>

namespace truck_demo {
namespace {

constexpr double kPi = 3.14159265358979323846;

double degrees(double radians) {
    return radians * 180.0 / kPi;
}

}  // namespace

void TelemetryPanel::render(
    const DemoSession& session,
    bool darkTheme,
    const ImVec2& available) {
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 10.0f);
    ImGui::BeginChild("##Telemetry", available, true);
    if (cachedTelemetryRevision_ != session.telemetryRevision()) {
        rebuildPlotData(session);
        cachedTelemetryRevision_ = session.telemetryRevision();
    }
    renderStateSummary(session, darkTheme);
    if (ImGui::BeginTabBar("##TelemetryTabs")) {
        if (ImGui::BeginTabItem(u8"跟踪误差状态")) {
            if (beginPlotGrid("##TrackingPlots")) {
                plotCell(
                    session,
                    darkTheme,
                    "ey_plot",
                    u8"横向误差",
                    u8"e_y [m]",
                    lateralError_,
                    -2.0,
                    2.0,
                    "%.3f");
                plotCell(
                    session,
                    darkTheme,
                    "ey_rate_plot",
                    u8"横向误差速度",
                    u8"e_y_dot [m/s]",
                    lateralErrorRate_,
                    -5.0,
                    5.0,
                    "%.3f");
                plotCell(
                    session,
                    darkTheme,
                    "heading_plot",
                    u8"航向误差",
                    u8"e_psi [deg]",
                    headingError_,
                    -12.0,
                    12.0,
                    "%.2f");
                plotCell(
                    session,
                    darkTheme,
                    "heading_rate_plot",
                    u8"航向误差速度",
                    u8"e_psi_dot [deg/s]",
                    headingErrorRate_,
                    -35.0,
                    35.0,
                    "%.2f");
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem(u8"铰接与横摆动态")) {
            if (beginPlotGrid("##ArticulationPlots")) {
                plotArticulationCell(session, darkTheme);
                plotCell(
                    session,
                    darkTheme,
                    "articulation_rate_plot",
                    u8"铰接角速度",
                    u8"phi_dot [deg/s]",
                    articulationRate_,
                    -35.0,
                    35.0,
                    "%.2f");
                plotCell(
                    session,
                    darkTheme,
                    "truck_yaw_rate_plot",
                    u8"卡车横摆角速度",
                    u8"r_1 [deg/s]",
                    truckYawRate_,
                    -60.0,
                    60.0,
                    "%.2f");
                plotCell(
                    session,
                    darkTheme,
                    "trailer_yaw_rate_plot",
                    u8"挂车横摆角速度",
                    u8"r_2 [deg/s]",
                    trailerYawRate_,
                    -60.0,
                    60.0,
                    "%.2f");
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem(u8"控制、车速与曲率")) {
            if (beginPlotGrid("##ControlPlots")) {
                const double steeringLimit = std::max(
                    5.0,
                    degrees(session.settings().mpc.maxSteering));
                plotCell(
                    session,
                    darkTheme,
                    "steering_plot",
                    u8"前轮转角",
                    u8"delta [deg]",
                    steering_,
                    -steeringLimit,
                    steeringLimit,
                    "%.2f");
                plotCell(
                    session,
                    darkTheme,
                    "speed_plot",
                    u8"自适应车速",
                    u8"v [m/s]",
                    speed_,
                    0.0,
                    1.1 * session.settings().vehicle.vx,
                    "%.2f");
                plotCell(
                    session,
                    darkTheme,
                    "curvature_plot",
                    u8"参考曲率",
                    u8"rho [1/m]",
                    curvature_,
                    -0.08,
                    0.08,
                    "%.5f");
                plotCell(
                    session,
                    darkTheme,
                    "curvature_rate_plot",
                    u8"参考曲率变化率",
                    u8"rho_dot [1/(m*s)]",
                    curvatureRate_,
                    -0.12,
                    0.12,
                    "%.5f");
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
}

void TelemetryPanel::renderStateSummary(
    const DemoSession& session,
    bool darkTheme) {
    const auto& state = session.state();
    double curvature = 0.0;
    if (!session.history().empty()) {
        curvature = session.history().back().referenceCurvature;
    }
    if (ImGui::BeginTable(
            "##StateSummary",
            6,
            ImGuiTableFlags_SizingStretchSame |
                ImGuiTableFlags_BordersInnerV)) {
        summaryMetric(darkTheme, u8"横向误差", state[0], "m", "%.2f");
        summaryMetric(
            darkTheme,
            u8"航向误差",
            degrees(state[2]),
            "deg",
            "%.2f");
        summaryMetric(
            darkTheme,
            u8"铰接角",
            degrees(state[4]),
            "deg",
            "%.2f");
        summaryMetric(
            darkTheme,
            u8"实际车速",
            session.currentSpeed(),
            "m/s",
            "%.1f");
        summaryMetric(
            darkTheme,
            u8"前轮转角",
            degrees(session.steering()),
            "deg",
            "%.2f");
        summaryMetric(
            darkTheme,
            u8"参考曲率",
            curvature,
            "1/m",
            "%.4f");
        ImGui::EndTable();
    }
    if (!session.warningReason().empty()) {
        ImGui::TextColored(
            darkTheme ? ImVec4(0.98f, 0.78f, 0.28f, 1.0f)
                      : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
            u8"警告：%s",
            session.warningReason().c_str());
    }
    ImGui::Separator();
}

void TelemetryPanel::summaryMetric(
    bool darkTheme,
    const char* label,
    double value,
    const char* unit,
    const char* format) {
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", label);
    ImGui::TextColored(
        darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                  : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
        format,
        value);
    ImGui::SameLine();
    ImGui::TextDisabled("%s", unit);
}

bool TelemetryPanel::beginPlotGrid(const char* id) {
    const ImVec2 available = ImGui::GetContentRegionAvail();
    plotRowHeight_ = std::max(132.0f, (available.y - 8.0f) * 0.5f);
    plotCellsInRow_ = 0;
    return ImGui::BeginTable(
        id,
        2,
        ImGuiTableFlags_SizingStretchSame,
        available);
}

void TelemetryPanel::beginPlotCell() {
    if (plotCellsInRow_ == 0) {
        ImGui::TableNextRow(ImGuiTableRowFlags_None, plotRowHeight_);
    }
    ImGui::TableNextColumn();
    plotCellsInRow_ = (plotCellsInRow_ + 1) % 2;
}

float TelemetryPanel::plotHeight() const {
    return std::max(88.0f, ImGui::GetContentRegionAvail().y - 4.0f);
}

float TelemetryPanel::currentMarkerSize(float plotHeight) const {
    return std::clamp(plotHeight * 0.032f, 3.5f, 10.0f);
}

void TelemetryPanel::rebuildPlotData(const DemoSession& session) {
    const auto& history = session.history();
    time_.resize(history.size());
    lateralError_.resize(history.size());
    lateralErrorRate_.resize(history.size());
    headingError_.resize(history.size());
    headingErrorRate_.resize(history.size());
    articulation_.resize(history.size());
    articulationReference_.resize(history.size());
    articulationRate_.resize(history.size());
    plantArticulation_.resize(history.size());
    lidarArticulation_.resize(history.size());
    articulationTrackingError_.resize(history.size());
    truckYawRate_.resize(history.size());
    trailerYawRate_.resize(history.size());
    steering_.resize(history.size());
    speed_.resize(history.size());
    curvature_.resize(history.size());
    curvatureRate_.resize(history.size());
    for (std::size_t i = 0; i < history.size(); ++i) {
        time_[i] = history[i].time;
        lateralError_[i] = history[i].state[0];
        lateralErrorRate_[i] = history[i].state[1];
        headingError_[i] = degrees(history[i].state[2]);
        headingErrorRate_[i] = degrees(history[i].state[3]);
        articulation_[i] = degrees(history[i].state[4]);
        articulationReference_[i] = degrees(history[i].referenceArticulation);
        articulationRate_[i] = degrees(history[i].state[5]);
        plantArticulation_[i] = degrees(history[i].plantArticulation);
        lidarArticulation_[i] = degrees(history[i].lidarArticulation);
        articulationTrackingError_[i] =
            degrees(history[i].articulationTrackingError);
        truckYawRate_[i] = degrees(history[i].physicalState[1]);
        trailerYawRate_[i] = degrees(history[i].physicalState[2]);
        steering_[i] = degrees(history[i].steering);
        speed_[i] = history[i].speed;
        curvature_[i] = history[i].referenceCurvature;
        curvatureRate_[i] = history[i].referenceCurvatureRate;
    }
}

void TelemetryPanel::plotArticulationCell(
    const DemoSession& session,
    bool darkTheme) {
    const double error = articulationTrackingError_.empty()
                             ? 0.0
                             : articulationTrackingError_.back();
    beginPlotCell();
    ImGui::TextUnformatted(u8"铰接角");
    ImGui::SameLine();
    ImGui::TextDisabled(u8"· 当前");
    ImGui::SameLine();
    ImGui::TextColored(
        darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                  : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
        "%.2f",
        articulation_.empty() ? 0.0 : articulation_.back());
    ImGui::SameLine();
    ImGui::TextDisabled(u8"ref");
    ImGui::SameLine();
    ImGui::TextColored(
        ImVec4(1.0f, 0.56f, 0.20f, 1.0f),
        "%.2f",
        articulationReference_.empty() ? 0.0 : articulationReference_.back());
    ImGui::SameLine();
    ImGui::TextDisabled(u8"err");
    ImGui::SameLine();
    ImGui::TextColored(
        darkTheme ? ImVec4(0.95f, 0.72f, 0.28f, 1.0f)
                  : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
        "%.2f",
        error);
    const float height = plotHeight();
    if (ImPlot::BeginPlot(
            "##articulation_plot",
            ImVec2(-1.0f, height),
            session.settings().lidarFusionEnabled ? 0
                                                  : ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxes(
            u8"时间 [s]",
            u8"phi [deg]",
            ImPlotAxisFlags_NoHighlight,
            ImPlotAxisFlags_NoHighlight);
        const double right = std::max(15.0, session.time());
        ImPlot::SetupAxisLimits(
            ImAxis_X1, right - 15.0, right, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -25.0, 25.0, ImGuiCond_Once);
        if (!articulation_.empty()) {
            ImPlot::SetNextLineStyle(
                ImVec4(0.22f, 0.68f, 1.0f, 1.0f), 2.0f);
            ImPlot::PlotLine(
                session.settings().lidarFusionEnabled ? u8"估计"
                                                      : u8"实测",
                time_.data(),
                articulation_.data(),
                static_cast<int>(articulation_.size()));
            if (session.settings().lidarFusionEnabled) {
                ImPlot::SetNextLineStyle(
                    ImVec4(0.45f, 0.82f, 0.48f, 1.0f), 1.6f);
                ImPlot::PlotLine(
                    u8"Plant",
                    time_.data(),
                    plantArticulation_.data(),
                    static_cast<int>(plantArticulation_.size()));
                ImPlot::SetNextLineStyle(
                    ImVec4(0.78f, 0.55f, 1.0f, 1.0f), 1.4f);
                ImPlot::PlotLine(
                    u8"雷达",
                    time_.data(),
                    lidarArticulation_.data(),
                    static_cast<int>(lidarArticulation_.size()));
            }
            ImPlot::SetNextLineStyle(
                ImVec4(1.0f, 0.56f, 0.20f, 1.0f), 2.0f);
            ImPlot::PlotLine(
                u8"参考",
                time_.data(),
                articulationReference_.data(),
                static_cast<int>(articulationReference_.size()));
            const double currentTime = time_.back();
            const double currentValue = articulation_.back();
            ImPlot::SetNextMarkerStyle(
                ImPlotMarker_Circle,
                currentMarkerSize(height),
                ImVec4(1.0f, 0.56f, 0.20f, 1.0f),
                IMPLOT_AUTO,
                ImVec4(1.0f, 0.82f, 0.45f, 1.0f));
            ImPlot::PlotScatter(
                "##CurrentArticulation",
                &currentTime,
                &currentValue,
                1);
        }
        ImPlot::EndPlot();
    }
}

void TelemetryPanel::plotCell(
    const DemoSession& session,
    bool darkTheme,
    const char* id,
    const char* title,
    const char* yLabel,
    const std::vector<double>& values,
    double minimum,
    double maximum,
    const char* valueFormat) {
    beginPlotCell();
    ImGui::TextUnformatted(title);
    ImGui::SameLine();
    ImGui::TextDisabled(u8"· 当前");
    ImGui::SameLine();
    ImGui::TextColored(
        darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                  : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
        valueFormat,
        values.empty() ? 0.0 : values.back());
    const float height = plotHeight();
    std::string plotId = "##";
    plotId += id;
    if (ImPlot::BeginPlot(
            plotId.c_str(),
            ImVec2(-1.0f, height),
            ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxes(
            u8"时间 [s]",
            yLabel,
            ImPlotAxisFlags_NoHighlight,
            ImPlotAxisFlags_NoHighlight);
        const double right = std::max(15.0, session.time());
        ImPlot::SetupAxisLimits(
            ImAxis_X1,
            right - 15.0,
            right,
            ImGuiCond_Always);
        ImPlot::SetupAxisLimits(
            ImAxis_Y1, minimum, maximum, ImGuiCond_Once);
        if (!values.empty()) {
            ImPlot::SetNextLineStyle(
                ImVec4(0.22f, 0.68f, 1.0f, 1.0f), 2.0f);
            ImPlot::PlotLine(
                title,
                time_.data(),
                values.data(),
                static_cast<int>(values.size()));
            const double currentTime = time_.back();
            const double currentValue = values.back();
            ImPlot::SetNextMarkerStyle(
                ImPlotMarker_Circle,
                currentMarkerSize(height),
                ImVec4(1.0f, 0.56f, 0.20f, 1.0f),
                IMPLOT_AUTO,
                ImVec4(1.0f, 0.82f, 0.45f, 1.0f));
            ImPlot::PlotScatter(
                "##CurrentValue",
                &currentTime,
                &currentValue,
                1);
        }
        ImPlot::EndPlot();
    }
}

}  // namespace truck_demo
