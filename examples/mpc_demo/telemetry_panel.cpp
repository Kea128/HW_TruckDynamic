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

// One palette shared by the angle and rate plots. The traces carry different
// meanings, so they are separated by hue rather than by shade: the previous
// reference and shadow colours were both amber and read as the same line.
const ImVec4 kPlantColor{0.30f, 0.86f, 0.42f, 1.0f};    // ground truth, green
const ImVec4 kReferenceColor{1.00f, 0.38f, 0.76f, 1.0f};  // target, magenta
const ImVec4 kLidarColor{0.58f, 0.55f, 0.70f, 1.0f};    // raw sensor, muted
const ImVec4 kShadowColor{1.00f, 0.66f, 0.10f, 1.0f};   // v1, amber
const ImVec4 kPrimaryColor{0.20f, 0.74f, 1.00f, 1.0f};  // v2, cyan
const ImVec4 kMarkerFill{0.96f, 0.97f, 1.00f, 1.0f};
const ImVec4 kMarkerEdge{0.45f, 0.52f, 0.62f, 1.0f};

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
                plotArticulationRateCell(session, darkTheme);
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
    // Closing the loop on the estimate makes the plant absorb the estimator's
    // amplitude error, so this configuration reads as a tracking overshoot that
    // is really a filter bias. Saying so beats letting it look like a bug.
    if (session.settings().articulationTrackingExperiment &&
        session.settings().lidarFusionEnabled) {
        ImGui::TextColored(
            darkTheme ? ImVec4(0.98f, 0.78f, 0.28f, 1.0f)
                      : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
            u8"闭环反馈用的是估计值：估计偏瘦时 Plant 会按 1/幅值比 过冲，"
            u8"参考与真值幅值不等属正常。此工况不作融合验收。");
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
    cellTop_ = ImGui::GetCursorPosY();
    plotCellsInRow_ = (plotCellsInRow_ + 1) % 2;
}

float TelemetryPanel::plotHeight() const {
    // Subtract whatever the caller already emitted as a header. Measuring the
    // remaining window space instead makes the second row shorter than the
    // first, and makes a cell with a taller header shrink its neighbours.
    const float consumed = ImGui::GetCursorPosY() - cellTop_;
    return std::max(88.0f, plotRowHeight_ - consumed - 6.0f);
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
    plantArticulationRate_.resize(history.size());
    shadowArticulationRate_.resize(history.size());
    plantArticulation_.resize(history.size());
    shadowArticulation_.resize(history.size());
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
        plantArticulationRate_[i] = degrees(history[i].plantArticulationRate);
        shadowArticulationRate_[i] = degrees(history[i].shadowArticulationRate);
        plantArticulation_[i] = degrees(history[i].plantArticulation);
        shadowArticulation_[i] = degrees(history[i].shadowArticulation);
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

    primaryRmseDeg_ = 0.0;
    shadowRmseDeg_ = 0.0;
    lidarRmseDeg_ = 0.0;
    primaryRateRmseDeg_ = 0.0;
    shadowRateRmseDeg_ = 0.0;
    comparisonValid_ = false;
    if (!session.settings().lidarFusionEnabled) {
        return;
    }
    // Skip the first second so the initial transient does not dominate.
    double primarySse = 0.0;
    double shadowSse = 0.0;
    double lidarSse = 0.0;
    double primaryRateSse = 0.0;
    double shadowRateSse = 0.0;
    std::size_t scored = 0;
    for (const auto& sample : history) {
        if (sample.time < 1.0) {
            continue;
        }
        const double primary =
            sample.estimatedArticulation - sample.plantArticulation;
        const double shadow =
            sample.shadowArticulation - sample.plantArticulation;
        const double lidar =
            sample.lidarArticulation - sample.plantArticulation;
        const double primaryRate =
            sample.estimatedArticulationRate - sample.plantArticulationRate;
        const double shadowRate =
            sample.shadowArticulationRate - sample.plantArticulationRate;
        primarySse += primary * primary;
        shadowSse += shadow * shadow;
        lidarSse += lidar * lidar;
        primaryRateSse += primaryRate * primaryRate;
        shadowRateSse += shadowRate * shadowRate;
        ++scored;
    }
    if (scored == 0) {
        return;
    }
    const auto rmse = [scored](double sse) {
        return degrees(std::sqrt(sse / static_cast<double>(scored)));
    };
    primaryRmseDeg_ = rmse(primarySse);
    shadowRmseDeg_ = rmse(shadowSse);
    lidarRmseDeg_ = rmse(lidarSse);
    primaryRateRmseDeg_ = rmse(primaryRateSse);
    shadowRateRmseDeg_ = rmse(shadowRateSse);
    comparisonValid_ = true;
}

void TelemetryPanel::plotArticulationCell(
    const DemoSession& session,
    bool darkTheme) {
    const double error = articulationTrackingError_.empty()
                             ? 0.0
                             : articulationTrackingError_.back();
    const bool fusion = session.settings().lidarFusionEnabled;
    const bool shadow = session.settings().shadowEstimatorEnabled;

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
    if (fusion) {
        // Against the plant, so the fused estimate can be judged by a number
        // rather than by two overlapping curves.
        ImGui::SameLine();
        ImGui::TextDisabled(u8"· RMSE");
        ImGui::SameLine();
        ImGui::TextColored(
            darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                      : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
            "%s %.2f",
            estimatorDisplayName(session.settings().articulationEstimator),
            comparisonValid_ ? primaryRmseDeg_ : 0.0);
        if (shadow) {
            ImGui::SameLine();
            ImGui::TextColored(
                ImVec4(0.95f, 0.72f, 0.28f, 1.0f),
                "%s %.2f",
                estimatorDisplayName(session.settings().shadowEstimator),
                comparisonValid_ ? shadowRmseDeg_ : 0.0);
        }
        ImGui::SameLine();
        ImGui::TextColored(
            ImVec4(0.78f, 0.55f, 1.0f, 1.0f),
            u8"雷达 %.2f",
            comparisonValid_ ? lidarRmseDeg_ : 0.0);
        ImGui::SameLine();
        ImGui::TextDisabled("deg");
    } else {
        ImGui::SameLine();
        ImGui::TextDisabled(u8"ref");
        ImGui::SameLine();
        ImGui::TextColored(
            ImVec4(1.0f, 0.56f, 0.20f, 1.0f),
            "%.2f",
            articulationReference_.empty() ? 0.0
                                           : articulationReference_.back());
        ImGui::SameLine();
        ImGui::TextDisabled(u8"err");
        ImGui::SameLine();
        ImGui::TextColored(
            darkTheme ? ImVec4(0.95f, 0.72f, 0.28f, 1.0f)
                      : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
            "%.2f",
            error);
    }
    const float height = plotHeight();
    if (ImPlot::BeginPlot("##articulation_plot", ImVec2(-1.0f, height))) {
        ImPlot::SetupAxes(
            u8"时间 [s]",
            u8"phi [deg]",
            ImPlotAxisFlags_NoHighlight,
            ImPlotAxisFlags_NoHighlight);
        const double right = std::max(15.0, session.time());
        ImPlot::SetupAxisLimits(
            ImAxis_X1, right - 15.0, right, ImGuiCond_Always);
        // Fit the vertical range to the data. A fixed +/-25 deg window makes a
        // few-degree articulation look like a single flat line, which is what
        // hides the difference between the raw scan and the fused estimates.
        ImPlot::SetupAxisLimits(ImAxis_Y1, -25.0, 25.0, ImGuiCond_Once);
        if (fusion) {
            ImPlot::SetupAxes(nullptr, nullptr, 0, ImPlotAxisFlags_AutoFit);
        }
        if (!articulation_.empty()) {
            // Raw sensor first so it sits behind everything, then truth, then
            // the estimates that are judged against it.
            if (fusion) {
                ImPlot::SetNextLineStyle(kLidarColor, 1.2f);
                ImPlot::PlotLine(
                    u8"雷达原始",
                    time_.data(),
                    lidarArticulation_.data(),
                    static_cast<int>(lidarArticulation_.size()));
                ImPlot::SetNextLineStyle(kPlantColor, 2.4f);
                ImPlot::PlotLine(
                    u8"Plant 真值",
                    time_.data(),
                    plantArticulation_.data(),
                    static_cast<int>(plantArticulation_.size()));
                if (shadow) {
                    ImPlot::SetNextLineStyle(kShadowColor, 1.7f);
                    ImPlot::PlotLine(
                        estimatorDisplayName(
                            session.settings().shadowEstimator),
                        time_.data(),
                        shadowArticulation_.data(),
                        static_cast<int>(shadowArticulation_.size()));
                }
            }
            ImPlot::SetNextLineStyle(kPrimaryColor, 2.0f);
            ImPlot::PlotLine(
                fusion ? estimatorDisplayName(
                             session.settings().articulationEstimator)
                       : u8"实测",
                time_.data(),
                articulation_.data(),
                static_cast<int>(articulation_.size()));
            // Drawn whenever a reference exists, including while a tracking
            // experiment runs alongside the fusion comparison. In that closed
            // loop the plant overshoots the reference, so the two are expected
            // to differ in amplitude rather than overlap.
            if (!session.articulationReference().empty() ||
                session.settings().articulationTrackingExperiment) {
                ImPlot::SetNextLineStyle(kReferenceColor, 2.2f);
                ImPlot::PlotLine(
                    u8"参考 phi_ref",
                    time_.data(),
                    articulationReference_.data(),
                    static_cast<int>(articulationReference_.size()));
            }
            const double currentTime = time_.back();
            const double currentValue = articulation_.back();
            ImPlot::SetNextMarkerStyle(
                ImPlotMarker_Circle,
                currentMarkerSize(height),
                kMarkerFill,
                IMPLOT_AUTO,
                kMarkerEdge);
            ImPlot::PlotScatter(
                "##CurrentArticulation",
                &currentTime,
                &currentValue,
                1);
        }
        ImPlot::EndPlot();
    }
}

void TelemetryPanel::plotArticulationRateCell(
    const DemoSession& session,
    bool darkTheme) {
    const bool fusion = session.settings().lidarFusionEnabled;
    const bool shadow = session.settings().shadowEstimatorEnabled;

    beginPlotCell();
    ImGui::TextUnformatted(u8"铰接角速度");
    ImGui::SameLine();
    ImGui::TextDisabled(u8"· 当前");
    ImGui::SameLine();
    ImGui::TextColored(
        darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                  : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
        "%.2f",
        articulationRate_.empty() ? 0.0 : articulationRate_.back());
    if (fusion) {
        // The lidar cannot observe phiDot at all, so this is pure model output
        // and the only honest check is against the plant.
        ImGui::SameLine();
        ImGui::TextDisabled(u8"· 纯模型推算 RMSE");
        ImGui::SameLine();
        ImGui::TextColored(
            darkTheme ? ImVec4(0.35f, 0.78f, 1.0f, 1.0f)
                      : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
            "%.2f",
            comparisonValid_ ? primaryRateRmseDeg_ : 0.0);
        if (shadow) {
            ImGui::SameLine();
            ImGui::TextColored(
                ImVec4(0.95f, 0.72f, 0.28f, 1.0f),
                "%.2f",
                comparisonValid_ ? shadowRateRmseDeg_ : 0.0);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("deg/s");
    }

    const float height = plotHeight();
    if (ImPlot::BeginPlot("##articulation_rate_plot", ImVec2(-1.0f, height))) {
        ImPlot::SetupAxes(
            u8"时间 [s]",
            u8"phi_dot [deg/s]",
            ImPlotAxisFlags_NoHighlight,
            ImPlotAxisFlags_NoHighlight);
        const double right = std::max(15.0, session.time());
        ImPlot::SetupAxisLimits(
            ImAxis_X1, right - 15.0, right, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -35.0, 35.0, ImGuiCond_Once);
        if (fusion) {
            ImPlot::SetupAxes(nullptr, nullptr, 0, ImPlotAxisFlags_AutoFit);
        }
        if (!articulationRate_.empty()) {
            if (fusion) {
                ImPlot::SetNextLineStyle(kPlantColor, 2.4f);
                ImPlot::PlotLine(
                    u8"Plant 真值",
                    time_.data(),
                    plantArticulationRate_.data(),
                    static_cast<int>(plantArticulationRate_.size()));
                if (shadow) {
                    ImPlot::SetNextLineStyle(kShadowColor, 1.7f);
                    ImPlot::PlotLine(
                        estimatorDisplayName(
                            session.settings().shadowEstimator),
                        time_.data(),
                        shadowArticulationRate_.data(),
                        static_cast<int>(shadowArticulationRate_.size()));
                }
            }
            ImPlot::SetNextLineStyle(kPrimaryColor, 2.0f);
            ImPlot::PlotLine(
                fusion ? estimatorDisplayName(
                             session.settings().articulationEstimator)
                       : u8"实测",
                time_.data(),
                articulationRate_.data(),
                static_cast<int>(articulationRate_.size()));
            const double currentTime = time_.back();
            const double currentValue = articulationRate_.back();
            ImPlot::SetNextMarkerStyle(
                ImPlotMarker_Circle,
                currentMarkerSize(height),
                kMarkerFill,
                IMPLOT_AUTO,
                kMarkerEdge);
            ImPlot::PlotScatter(
                "##CurrentArticulationRate", &currentTime, &currentValue, 1);
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
