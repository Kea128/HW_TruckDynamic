#define NOMINMAX
#include <windows.h>
#include <d3d11.h>

#include "demo_session.hpp"
#include "path_editor.hpp"
#include "session_log.hpp"
#include "telemetry_panel.hpp"

#include "imgui.h"
#include "imgui_impl_dx11.h"
#include "imgui_impl_win32.h"
#include "implot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

extern LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND window,
    UINT message,
    WPARAM wParam,
    LPARAM lParam);

namespace {

constexpr double kPi = 3.14159265358979323846;

ID3D11Device* gDevice = nullptr;
ID3D11DeviceContext* gDeviceContext = nullptr;
IDXGISwapChain* gSwapChain = nullptr;
ID3D11RenderTargetView* gRenderTarget = nullptr;
UINT gResizeWidth = 0;
UINT gResizeHeight = 0;

double degrees(double radians) {
    return radians * 180.0 / kPi;
}

double radians(double degreesValue) {
    return degreesValue * kPi / 180.0;
}

double varianceToStdDegrees(double variance) {
    return degrees(std::sqrt(std::max(variance, 0.0)));
}

double stdDegreesToVariance(double stdDegrees) {
    const double stdRadians = radians(std::max(stdDegrees, 0.0));
    return stdRadians * stdRadians;
}

void createRenderTarget() {
    ID3D11Texture2D* backBuffer = nullptr;
    gSwapChain->GetBuffer(0, IID_PPV_ARGS(&backBuffer));
    if (backBuffer) {
        gDevice->CreateRenderTargetView(backBuffer, nullptr, &gRenderTarget);
        backBuffer->Release();
    }
}

void cleanupRenderTarget() {
    if (gRenderTarget) {
        gRenderTarget->Release();
        gRenderTarget = nullptr;
    }
}

bool createDevice(HWND window) {
    DXGI_SWAP_CHAIN_DESC description{};
    description.BufferCount = 2;
    description.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    description.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    description.OutputWindow = window;
    description.SampleDesc.Count = 1;
    description.Windowed = TRUE;
    description.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    const D3D_FEATURE_LEVEL requested[] = {
        D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_0};
    D3D_FEATURE_LEVEL obtained{};
    const HRESULT result = D3D11CreateDeviceAndSwapChain(
        nullptr,
        D3D_DRIVER_TYPE_HARDWARE,
        nullptr,
        0,
        requested,
        2,
        D3D11_SDK_VERSION,
        &description,
        &gSwapChain,
        &gDevice,
        &obtained,
        &gDeviceContext);
    if (FAILED(result)) {
        return false;
    }
    createRenderTarget();
    return true;
}

void cleanupDevice() {
    cleanupRenderTarget();
    if (gSwapChain) {
        gSwapChain->Release();
        gSwapChain = nullptr;
    }
    if (gDeviceContext) {
        gDeviceContext->Release();
        gDeviceContext = nullptr;
    }
    if (gDevice) {
        gDevice->Release();
        gDevice = nullptr;
    }
}

const char* stateText(truck_demo::SimulationState state) {
    switch (state) {
        case truck_demo::SimulationState::running:
            return u8"运行中";
        case truck_demo::SimulationState::paused:
            return u8"已暂停";
        case truck_demo::SimulationState::finished:
            return u8"已完成";
        case truck_demo::SimulationState::faulted:
            return u8"安全停止";
        default:
            return u8"待机";
    }
}

ImU32 color(unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255) {
    return IM_COL32(r, g, b, a);
}

void configureThemeColors(bool dark);

class StudioApp {
public:
    StudioApp() {
        syncDraft();
    }

    void frame(float frameTime) {
        advanceSimulation(frameTime);
        handleShortcuts();
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        constexpr ImGuiWindowFlags rootFlags =
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
            ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 12.0f));
        ImGui::Begin("##TruckMpcStudioRoot", nullptr, rootFlags);
        renderToolbar();
        if (themeChangeRequested_) {
            configureThemeColors(darkTheme_);
            themeChangeRequested_ = false;
        }
        renderNotifications();
        ImGui::Spacing();

        if (!showSettings_) {
            renderWorkspace();
        } else if (ImGui::BeginTable(
                       "##MainLayout",
                       2,
                       ImGuiTableFlags_Resizable |
                           ImGuiTableFlags_BordersInnerV,
                       ImVec2(0.0f, 0.0f))) {
            ImGui::TableSetupColumn(
                u8"参数", ImGuiTableColumnFlags_WidthFixed, 350.0f);
            ImGui::TableSetupColumn(
                u8"仿真", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            renderSettings();
            ImGui::TableSetColumnIndex(1);
            renderWorkspace();
            ImGui::EndTable();
        }
        ImGui::End();
        ImGui::PopStyleVar();
    }

private:
    struct View {
        double centerX{110.0};
        double centerY{0.0};
        double pixelsPerMeter{4.0};
    };

    void handleShortcuts() {
        const ImGuiIO& io = ImGui::GetIO();
        if (io.WantTextInput) {
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) {
            if (session_.simulationState() ==
                truck_demo::SimulationState::running) {
                session_.pauseOrResume();
            } else {
                runSimulation();
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
            session_.reset();
            simulationAccumulator_ = 0.0;
            autoExportedThisRun_ = false;
            status_ = u8"仿真已重置";
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            PostQuitMessage(0);
        }
    }

    void advanceSimulation(float frameTime) {
        if (session_.simulationState() !=
            truck_demo::SimulationState::running) {
            simulationAccumulator_ = 0.0;
            return;
        }
        simulationAccumulator_ += std::min<double>(frameTime, 0.1);
        const double sampleTime = session_.settings().mpc.sampleTime;
        int steps = 0;
        while (simulationAccumulator_ >= sampleTime && steps < 8) {
            session_.step();
            simulationAccumulator_ -= sampleTime;
            ++steps;
            if (session_.simulationState() ==
                truck_demo::SimulationState::faulted) {
                error_ = session_.faultReason();
                status_ = u8"仿真产生非有限状态，已安全停止";
                simulationAccumulator_ = 0.0;
                break;
            }
        }
        maybeAutoExportLog();
    }

    void renderToolbar() {
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 10.0f);
        ImGui::PushStyleColor(
            ImGuiCol_ChildBg,
            darkTheme_ ? ImVec4(0.075f, 0.105f, 0.16f, 1.0f)
                       : ImVec4(0.985f, 0.99f, 1.0f, 1.0f));
        ImGui::BeginChild("##Toolbar", ImVec2(0.0f, 58.0f), true);
        ImGui::SetCursorPos(ImVec2(16.0f, 10.0f));
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            darkTheme_ ? ImVec4(0.95f, 0.97f, 1.0f, 1.0f)
                       : ImVec4(0.10f, 0.14f, 0.20f, 1.0f));
        ImGui::TextUnformatted(u8"TruckModel");
        ImGui::SameLine();
        ImGui::TextColored(
            darkTheme_ ? ImVec4(0.25f, 0.75f, 1.0f, 1.0f)
                       : ImVec4(0.04f, 0.42f, 0.72f, 1.0f),
            u8"MPC Studio 2.8.1");
        ImGui::PopStyleColor();
        ImGui::SameLine(235.0f);

        const auto state = session_.simulationState();
        if (state == truck_demo::SimulationState::running) {
            if (accentButton(u8"暂停", ImVec2(78.0f, 36.0f), false)) {
                session_.pauseOrResume();
            }
        } else {
            if (accentButton(u8"运行", ImVec2(78.0f, 36.0f), true)) {
                runSimulation();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(u8"重置", ImVec2(70.0f, 36.0f))) {
            session_.reset();
            simulationAccumulator_ = 0.0;
            autoExportedThisRun_ = false;
            status_ = u8"仿真已重置";
        }
        ImGui::SameLine();
        const auto editorState = pathEditor_.state();
        if (editorState != truck_demo::PathEditorState::review) {
            if (ImGui::Button(
                    editorState == truck_demo::PathEditorState::drawing
                        ? u8"取消绘制"
                        : u8"绘制轨迹",
                    ImVec2(100.0f, 36.0f))) {
                drawingStroke_ = false;
                session_.reset();
                if (editorState ==
                    truck_demo::PathEditorState::drawing) {
                    pathEditor_.cancel();
                    status_ = u8"已退出绘制模式";
                } else {
                    pathEditor_.begin();
                    status_ = u8"请在轨迹画布中按住左键绘制";
                }
            }
            ImGui::SameLine();
        }
        if (pathEditor_.state() == truck_demo::PathEditorState::review) {
            if (accentButton(
                    u8"确认轨迹", ImVec2(92.0f, 36.0f), true)) {
                confirmSmoothedPath();
            }
            ImGui::SameLine();
            if (ImGui::Button(u8"重画", ImVec2(66.0f, 36.0f))) {
                pathEditor_.begin();
                drawingStroke_ = false;
                status_ = u8"请重新绘制目标轨迹";
            }
            ImGui::SameLine();
        }
        if (ImGui::Button(u8"适应轨迹", ImVec2(90.0f, 36.0f))) {
            fitRequested_ = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(
                darkTheme_ ? u8"浅色" : u8"深色",
                ImVec2(62.0f, 36.0f))) {
            darkTheme_ = !darkTheme_;
            themeChangeRequested_ = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(
                showSettings_ ? u8"隐藏参数" : u8"显示参数",
                ImVec2(82.0f, 36.0f))) {
            showSettings_ = !showSettings_;
            fitRequested_ = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(
                telemetryFocus_ ? u8"平衡布局" : u8"聚焦图表",
                ImVec2(82.0f, 36.0f))) {
            telemetryFocus_ = !telemetryFocus_;
            fitRequested_ = true;
        }

        ImGui::SameLine(0.0f, 22.0f);
        ImGui::TextColored(
            state == truck_demo::SimulationState::running
                ? ImVec4(0.10f, 0.68f, 0.38f, 1.0f)
            : state == truck_demo::SimulationState::faulted
                ? ImVec4(0.90f, 0.25f, 0.18f, 1.0f)
            : darkTheme_ ? ImVec4(0.72f, 0.78f, 0.88f, 1.0f)
                         : ImVec4(0.30f, 0.36f, 0.44f, 1.0f),
            "● %s",
            stateText(state));
        ImGui::SameLine();
        ImGui::TextDisabled(
            "t %.1fs   s %.1fm   v %.1f/%.1fm/s   delta %.2f deg   phi %.1f/%.1f deg",
            session_.time(),
            session_.distance(),
            session_.currentSpeed(),
            session_.settings().vehicle.vx,
            degrees(session_.steering()),
            degrees(session_.state()[4]),
            degrees(session_.currentArticulationReference().value));
        if (!session_.warningReason().empty()) {
            ImGui::SameLine();
            ImGui::TextColored(
                darkTheme_ ? ImVec4(0.98f, 0.78f, 0.28f, 1.0f)
                           : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
                u8"⚠ %s",
                session_.warningReason().c_str());
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }

    bool accentButton(
        const char* label,
        const ImVec2& size,
        bool positive) {
        const ImVec4 base = positive
                                ? ImVec4(0.10f, 0.55f, 0.86f, 1.0f)
                                : ImVec4(0.85f, 0.48f, 0.15f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, base);
        ImGui::PushStyleColor(
            ImGuiCol_Text, ImVec4(0.97f, 0.98f, 1.0f, 1.0f));
        ImGui::PushStyleColor(
            ImGuiCol_ButtonHovered,
            ImVec4(
                std::min(1.0f, base.x + 0.1f),
                std::min(1.0f, base.y + 0.1f),
                std::min(1.0f, base.z + 0.1f),
                1.0f));
        const bool pressed = ImGui::Button(label, size);
        ImGui::PopStyleColor(3);
        return pressed;
    }

    void renderSettings() {
        ImGui::BeginChild(
            "##Settings",
            ImVec2(0.0f, 0.0f),
            true,
            ImGuiWindowFlags_AlwaysVerticalScrollbar);
        ImGui::TextColored(
            darkTheme_ ? ImVec4(0.25f, 0.72f, 1.0f, 1.0f)
                       : ImVec4(0.04f, 0.38f, 0.68f, 1.0f),
            u8"模型与控制参数");
        ImGui::TextDisabled(u8"修改后点击底部“应用并重建模型”");
        ImGui::Separator();

        const bool running =
            session_.simulationState() ==
            truck_demo::SimulationState::running;
        ImGui::BeginDisabled(running);
        if (ImGui::CollapsingHeader(
                u8"卡车参数", ImGuiTreeNodeFlags_DefaultOpen)) {
            parameterTableBegin("truck_parameters");
            property(u8"质量 m1 [kg]", draft_.vehicle.m1, 100.0);
            property(u8"偏航惯量 I1 [kg·m²]", draft_.vehicle.iz1, 100.0);
            property(u8"质心到前轴 a1 [m]", draft_.vehicle.a1, 0.1);
            property(u8"质心到后轴 b1 [m]", draft_.vehicle.b1, 0.1);
            property(u8"前轴侧偏刚度 C1f", draft_.vehicle.c1f, 1000.0);
            property(u8"后轴侧偏刚度 C1r", draft_.vehicle.c1r, 1000.0);
            property(u8"质心到铰接点 d1 [m]", draft_.vehicle.d1, 0.1);
            property(u8"最高巡航车速 U [m/s]", draft_.vehicle.vx, 0.5);
            ImGui::EndTable();
        }
        if (ImGui::CollapsingHeader(
                u8"挂车参数", ImGuiTreeNodeFlags_DefaultOpen)) {
            parameterTableBegin("trailer_parameters");
            property(u8"质量 m2 [kg]", draft_.vehicle.m2, 100.0);
            property(u8"偏航惯量 I2 [kg·m²]", draft_.vehicle.iz2, 100.0);
            property(u8"质心到铰接点 a2 [m]", draft_.vehicle.a2, 0.1);
            property(u8"质心到车轴 b2 [m]", draft_.vehicle.b2, 0.1);
            property(u8"车轴侧偏刚度 C2r", draft_.vehicle.c2r, 1000.0);
            ImGui::EndTable();
        }
        if (ImGui::CollapsingHeader(
                u8"MPC 状态权重 Q", ImGuiTreeNodeFlags_DefaultOpen)) {
            static const char* labels[6] = {
                u8"横向误差 e_y",
                u8"横向误差速度",
                u8"航向误差 e_psi",
                u8"航向误差速度",
                u8"铰接角 phi",
                u8"铰接角速度"};
            parameterTableBegin("mpc_q");
            for (std::size_t i = 0; i < 6; ++i) {
                property(labels[i], draft_.mpc.stateWeight[i], 0.5);
            }
            ImGui::EndTable();
        }
        if (ImGui::CollapsingHeader(
                u8"MPC 求解与执行器", ImGuiTreeNodeFlags_DefaultOpen)) {
            parameterTableBegin("mpc_solver");
            property(u8"终端权重倍率", draft_.terminalWeightFactor, 0.1);
            property(u8"转角权重 R", draft_.mpc.steeringWeight, 1.0);
            property(
                u8"转角增量权重 Rd",
                draft_.mpc.steeringRateWeight,
                100.0);
            int horizon = static_cast<int>(draft_.mpc.horizon);
            integerProperty(u8"预测步数 N", horizon, 1);
            draft_.mpc.horizon =
                static_cast<std::size_t>(std::max(1, horizon));
            property(u8"控制周期 [s]", draft_.mpc.sampleTime, 0.005, "%.3f");
            property(u8"最大转角 [deg]", maximumSteeringDegrees_, 0.5);
            property(
                u8"最大转角速率 [deg/s]",
                maximumSteeringRateDegrees_,
                1.0);
            booleanProperty(
                u8"启用曲率车速自适应",
                draft_.adaptiveSpeedEnabled);
            property(
                u8"最低自适应车速 [m/s]",
                draft_.minimumSpeed,
                0.5);
            property(
                u8"最大横向加速度 [m/s²]",
                draft_.maximumLateralAcceleration,
                0.2);
            property(
                u8"最大横向加加速度 [m/s³]",
                draft_.maximumLateralJerk,
                0.5);
            property(
                u8"最大纵向加速度 [m/s²]",
                draft_.maximumAcceleration,
                0.1);
            property(
                u8"最大纵向减速度 [m/s²]",
                draft_.maximumDeceleration,
                0.1);
            property(
                u8"车速规划前视距离 [m]",
                draft_.speedLookaheadDistance,
                1.0);
            ImGui::EndTable();
        }
        if (ImGui::CollapsingHeader(
                u8"初始状态", ImGuiTreeNodeFlags_DefaultOpen)) {
            parameterTableBegin("initial_state");
            property(u8"初始横向误差 [m]", draft_.initialLateralError, 0.1);
            property(
                u8"初始航向误差 [deg]",
                initialHeadingDegrees_,
                0.5);
            property(
                u8"初始铰接角 [deg]",
                initialArticulationDegrees_,
                0.5);
            property(
                u8"初始前轮转角 [deg]",
                initialSteeringDegrees_,
                0.5);
            ImGui::EndTable();
            if (ImGui::CollapsingHeader(u8"初始速度量（可选）")) {
                parameterTableBegin("initial_rates");
                property(
                    u8"初始横向误差速度 [m/s]",
                    draft_.initialLateralErrorRate,
                    0.1);
                property(
                    u8"初始航向误差速度 [deg/s]",
                    initialHeadingRateDegrees_,
                    0.5);
                property(
                    u8"初始铰接角速度 [deg/s]",
                    initialArticulationRateDegrees_,
                    0.5);
                ImGui::EndTable();
            }
        }
        if (ImGui::CollapsingHeader(
                u8"铰接角参考", ImGuiTreeNodeFlags_DefaultOpen)) {
            renderArticulationReferenceSettings();
        }
        if (ImGui::CollapsingHeader(u8"铰接角雷达融合")) {
            parameterTableBegin("lidar_fusion");
            booleanProperty(
                u8"启用 Delayed EKF",
                draft_.lidarFusionEnabled);
            property(u8"雷达周期 [s]", draft_.lidarPeriod, 0.01, "%.3f");
            property(u8"最小时延 [s]", draft_.lidarDelayMin, 0.01, "%.3f");
            property(u8"最大时延 [s]", draft_.lidarDelayMax, 0.01, "%.3f");
            double lidarNoiseDegrees = degrees(draft_.lidarNoiseStd);
            property(u8"雷达噪声 [deg]", lidarNoiseDegrees, 0.1);
            draft_.lidarNoiseStd = radians(lidarNoiseDegrees);
            booleanProperty(
                u8"R 跟随雷达噪声",
                draft_.ekfMeasurementFollowsLidar);
            if (draft_.ekfMeasurementFollowsLidar) {
                ekfMeasurementStdDegrees_ = std::max(lidarNoiseDegrees, 0.25);
                draft_.articulationEstimator.measurementVariance =
                    stdDegreesToVariance(ekfMeasurementStdDegrees_);
            }
            ImGui::BeginDisabled(draft_.ekfMeasurementFollowsLidar);
            property(u8"量测 R 标准差 [deg]", ekfMeasurementStdDegrees_, 0.1);
            ImGui::EndDisabled();
            property(
                u8"Q_phi 标准差 [deg/s]",
                ekfPhiProcessStdDegrees_,
                0.1);
            property(
                u8"Q_br2 标准差 [deg/s]",
                ekfTrailerBiasStdDegrees_,
                0.01,
                "%.3f");
            property(
                u8"Q_bphi 标准差 [deg]",
                ekfLidarBiasStdDegrees_,
                0.001,
                "%.4f");
            ImGui::EndTable();
            ImGui::TextDisabled(
                u8"R 为量测方差 (σ°)²；Q 为过程噪声标准差。改后点应用。");
        }

        ImGui::Spacing();
        if (accentButton(
                u8"应用并重建模型", ImVec2(-1.0f, 38.0f), true)) {
            applyDraft();
        }
        if (ImGui::Button(u8"导出本次记录", ImVec2(-1.0f, 34.0f))) {
            exportSessionLog(false);
        }
        if (ImGui::Button(u8"恢复全部默认值", ImVec2(-1.0f, 34.0f))) {
            session_.configure(truck_demo::DemoSession::defaultSettings());
            session_.setPath(truck_demo::DemoSession::defaultPath());
            syncDraft();
            fitRequested_ = true;
            status_ = u8"已恢复默认车辆、控制器和 S 形路径";
        }
        if (ImGui::Button(
                u8"加载高曲率连续 S 弯（8 m/s）",
                ImVec2(-1.0f, 34.0f))) {
            loadHighCurvatureScenario();
        }
        if (ImGui::Button(
                u8"铰接角恢复实验",
                ImVec2(-1.0f, 34.0f))) {
            loadArticulationRecoveryExperiment();
        }
        if (ImGui::Button(
                u8"铰接角跟踪实验",
                ImVec2(-1.0f, 34.0f))) {
            beginArticulationTrackingExperiment();
        }
        if (ImGui::Button(
                u8"退出实验并恢复路径 Q",
                ImVec2(-1.0f, 34.0f))) {
            endArticulationTrackingExperiment();
        }
        ImGui::EndDisabled();
        ImGui::Spacing();
        if (!error_.empty()) {
            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                darkTheme_ ? ImVec4(0.28f, 0.08f, 0.08f, 0.7f)
                           : ImVec4(1.0f, 0.90f, 0.89f, 1.0f));
            ImGui::BeginChild("##Error", ImVec2(0.0f, 58.0f), true);
            ImGui::TextColored(
                darkTheme_ ? ImVec4(1.0f, 0.48f, 0.42f, 1.0f)
                           : ImVec4(0.70f, 0.10f, 0.07f, 1.0f),
                u8"无法继续");
            ImGui::TextWrapped("%s", error_.c_str());
            ImGui::EndChild();
            ImGui::PopStyleColor();
        } else if (!session_.warningReason().empty()) {
            ImGui::PushStyleColor(
                ImGuiCol_ChildBg,
                darkTheme_ ? ImVec4(0.28f, 0.22f, 0.08f, 0.7f)
                           : ImVec4(1.0f, 0.96f, 0.86f, 1.0f));
            ImGui::BeginChild("##Warning", ImVec2(0.0f, 58.0f), true);
            ImGui::TextColored(
                darkTheme_ ? ImVec4(0.98f, 0.78f, 0.28f, 1.0f)
                           : ImVec4(0.72f, 0.42f, 0.04f, 1.0f),
                u8"线性模型范围警告");
            ImGui::TextWrapped("%s", session_.warningReason().c_str());
            ImGui::EndChild();
            ImGui::PopStyleColor();
        } else {
            ImGui::TextColored(
                darkTheme_ ? ImVec4(0.35f, 0.82f, 0.62f, 1.0f)
                           : ImVec4(0.06f, 0.48f, 0.28f, 1.0f),
                "%s",
                status_.c_str());
        }
        ImGui::TextDisabled(
            u8"画布：左键绘制 · 右键平移 · 滚轮缩放\n"
            u8"物理 Plant：K27 四状态动力学\n"
            u8"预测模型：K35–K41 六状态误差模型\n"
            u8"铰接角由前轮转角间接控制；跟踪实验将路径权重置零以跟随 phi_ref(t)\n"
            u8"Delayed EKF：K5 预测 + 10 Hz 时延雷达历史重传播");
        ImGui::EndChild();
    }

    void parameterTableBegin(const char* id) {
        ImGui::BeginTable(
            id,
            2,
            ImGuiTableFlags_SizingStretchProp |
                ImGuiTableFlags_BordersInnerH);
        ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 1.65f);
        ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    }

    void property(
        const char* label,
        double& value,
        double step,
        const char* format = "%.2f") {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1.0f);
        std::string id = "##";
        id += label;
        ImGui::InputDouble(id.c_str(), &value, step, step * 10.0, format);
    }

    void integerProperty(const char* label, int& value, int step) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1.0f);
        std::string id = "##";
        id += label;
        ImGui::InputInt(id.c_str(), &value, step, step * 10);
    }

    void booleanProperty(const char* label, bool& value) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableSetColumnIndex(1);
        std::string id = "##";
        id += label;
        ImGui::Checkbox(id.c_str(), &value);
    }

    void syncDraft() {
        draft_ = session_.settings();
        maximumSteeringDegrees_ = degrees(draft_.mpc.maxSteering);
        maximumSteeringRateDegrees_ =
            degrees(draft_.mpc.maxSteeringRate);
        initialHeadingDegrees_ = degrees(draft_.initialHeadingError);
        initialHeadingRateDegrees_ =
            degrees(draft_.initialHeadingErrorRate);
        initialArticulationDegrees_ = degrees(draft_.initialArticulation);
        initialArticulationRateDegrees_ =
            degrees(draft_.initialArticulationRate);
        initialSteeringDegrees_ = degrees(draft_.initialSteering);
        amplitudeDegrees_ =
            degrees(draft_.articulationReference.amplitude);
        offsetDegrees_ = degrees(draft_.articulationReference.offset);
        phaseDegrees_ = degrees(draft_.articulationReference.phase);
        ekfMeasurementStdDegrees_ = varianceToStdDegrees(
            draft_.articulationEstimator.measurementVariance);
        ekfPhiProcessStdDegrees_ = varianceToStdDegrees(
            draft_.articulationEstimator.processArticulationRateVariance);
        ekfTrailerBiasStdDegrees_ = varianceToStdDegrees(
            draft_.articulationEstimator.processTrailerYawBiasVariance);
        ekfLidarBiasStdDegrees_ = varianceToStdDegrees(
            draft_.articulationEstimator.processLidarBiasVariance);
        error_.clear();
    }

    void applyDraftAngles() {
        draft_.mpc.maxSteering = radians(maximumSteeringDegrees_);
        draft_.mpc.maxSteeringRate =
            radians(maximumSteeringRateDegrees_);
        draft_.initialHeadingError = radians(initialHeadingDegrees_);
        draft_.initialHeadingErrorRate =
            radians(initialHeadingRateDegrees_);
        draft_.initialArticulation = radians(initialArticulationDegrees_);
        draft_.initialArticulationRate =
            radians(initialArticulationRateDegrees_);
        draft_.initialSteering = radians(initialSteeringDegrees_);
        draft_.articulationReference.amplitude = radians(amplitudeDegrees_);
        draft_.articulationReference.offset = radians(offsetDegrees_);
        draft_.articulationReference.phase = radians(phaseDegrees_);
        draft_.articulationEstimator.measurementVariance =
            stdDegreesToVariance(ekfMeasurementStdDegrees_);
        draft_.articulationEstimator.processArticulationRateVariance =
            stdDegreesToVariance(ekfPhiProcessStdDegrees_);
        draft_.articulationEstimator.processTrailerYawBiasVariance =
            stdDegreesToVariance(ekfTrailerBiasStdDegrees_);
        draft_.articulationEstimator.processLidarBiasVariance =
            stdDegreesToVariance(ekfLidarBiasStdDegrees_);
    }

    void applyDraft() {
        try {
            applyDraftAngles();
            session_.configure(draft_);
            syncDraft();
            simulationAccumulator_ = 0.0;
            autoExportedThisRun_ = false;
            status_ = u8"参数已应用，物理 Plant 与 MPC 已重新构建";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void loadHighCurvatureScenario() {
        try {
            applyDraftAngles();
            auto settings = session_.settings();
            settings.vehicle.vx = 8.0;
            settings.initialLateralError = 0.5;
            settings.initialLateralErrorRate = 0.0;
            settings.initialHeadingError = radians(2.0);
            settings.initialHeadingErrorRate = 0.0;
            settings.initialArticulation = 0.0;
            settings.initialArticulationRate = 0.0;
            settings.initialSteering = 0.0;
            session_.configure(settings);
            session_.setPath(
                truck_demo::DemoSession::highCurvaturePath());
            syncDraft();
            fitRequested_ = true;
            simulationAccumulator_ = 0.0;
            status_ =
                u8"已加载高曲率连续 S 弯：8 m/s，峰值曲率 0.025 1/m";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void beginArticulationTrackingExperiment() {
        try {
            applyDraftAngles();
            session_.configure(draft_);
            if (draft_.articulationReference.kind ==
                    truck_demo::ArticulationReferenceKind::drawn &&
                !drawnReference_.empty()) {
                session_.setArticulationReference(drawnReference_);
            }
            session_.beginArticulationTrackingExperiment();
            syncDraft();
            simulationAccumulator_ = 0.0;
            status_ =
                u8"已进入铰接角跟踪实验：路径权重已降低，主跟踪 phi_ref";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void endArticulationTrackingExperiment() {
        try {
            session_.endArticulationTrackingExperiment();
            syncDraft();
            simulationAccumulator_ = 0.0;
            status_ = u8"已退出铰接角跟踪实验，路径权重已恢复";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void renderArticulationReferenceSettings() {
        parameterTableBegin("articulation_reference");
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(u8"参考模式");
        ImGui::TableSetColumnIndex(1);
        ImGui::SetNextItemWidth(-1.0f);
        int kind = static_cast<int>(draft_.articulationReference.kind);
        const char* items[] = {
            u8"关闭", u8"正弦", u8"周期阶跃", u8"手绘"};
        if (ImGui::Combo("##phi_ref_kind", &kind, items, 4)) {
            draft_.articulationReference.kind =
                static_cast<truck_demo::ArticulationReferenceKind>(kind);
        }
        property(u8"时长 [s]", draft_.articulationReference.duration, 0.5);
        property(u8"幅值 [deg]", amplitudeDegrees_, 0.5);
        property(u8"偏置 [deg]", offsetDegrees_, 0.5);
        if (draft_.articulationReference.kind ==
            truck_demo::ArticulationReferenceKind::sine) {
            property(
                u8"频率 [Hz]",
                draft_.articulationReference.frequency,
                0.01,
                "%.3f");
            property(u8"相位 [deg]", phaseDegrees_, 5.0);
        }
        if (draft_.articulationReference.kind ==
            truck_demo::ArticulationReferenceKind::periodicStep) {
            property(
                u8"周期 [s]", draft_.articulationReference.period, 0.2);
            property(
                u8"占空比",
                draft_.articulationReference.dutyCycle,
                0.05,
                "%.2f");
        }
        ImGui::EndTable();
        renderArticulationReferencePlot();
        if (draft_.articulationReference.kind ==
            truck_demo::ArticulationReferenceKind::drawn) {
            if (ImGui::Button(u8"确认手绘参考", ImVec2(-1.0f, 30.0f))) {
                confirmDrawnArticulationReference();
            }
            if (ImGui::Button(u8"重画参考", ImVec2(-1.0f, 28.0f))) {
                drawnRawPoints_.clear();
                drawnReference_ = {};
                status_ = u8"已清空手绘铰接角参考";
            }
            ImGui::TextDisabled(u8"在图中按住左键绘制 phi(t)，松手后点确认");
        }
    }

    void renderArticulationReferencePlot() {
        rebuildArticulationPreview();
        if (ImPlot::BeginPlot(
                "##PhiRefEditor",
                ImVec2(-1.0f, 170.0f),
                ImPlotFlags_NoLegend)) {
            ImPlot::SetupAxes(
                u8"时间 [s]",
                u8"phi_ref [deg]",
                ImPlotAxisFlags_NoHighlight,
                ImPlotAxisFlags_NoHighlight);
            const double duration =
                std::max(2.0, draft_.articulationReference.duration);
            ImPlot::SetupAxisLimits(
                ImAxis_X1, 0.0, duration, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(
                ImAxis_Y1, -40.0, 40.0, ImGuiCond_Always);
            if (!previewTimes_.empty()) {
                ImPlot::SetNextLineStyle(
                    ImVec4(0.20f, 0.78f, 0.42f, 1.0f), 2.0f);
                ImPlot::PlotLine(
                    u8"预览",
                    previewTimes_.data(),
                    previewDegrees_.data(),
                    static_cast<int>(previewTimes_.size()));
            }
            if (draft_.articulationReference.kind ==
                    truck_demo::ArticulationReferenceKind::drawn &&
                ImPlot::IsPlotHovered()) {
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    drawnRawPoints_.clear();
                    drawingReference_ = true;
                }
                if (drawingReference_ &&
                    ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                    const ImPlotPoint mouse = ImPlot::GetPlotMousePos();
                    truck_demo::TimeArticulationPoint point;
                    point.time = std::max(0.0, mouse.x);
                    point.articulation = std::clamp(
                        radians(mouse.y),
                        -truck_demo::kMaximumArticulationReference,
                        truck_demo::kMaximumArticulationReference);
                    if (drawnRawPoints_.empty() ||
                        point.time - drawnRawPoints_.back().time >= 0.02) {
                        drawnRawPoints_.push_back(point);
                    }
                }
                if (drawingReference_ &&
                    ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    drawingReference_ = false;
                }
            }
            ImPlot::EndPlot();
        }
    }

    void rebuildArticulationPreview() {
        previewTimes_.clear();
        previewDegrees_.clear();
        try {
            applyDraftAngles();
            truck_demo::ArticulationReference reference;
            if (draft_.articulationReference.kind ==
                    truck_demo::ArticulationReferenceKind::drawn &&
                drawnRawPoints_.size() >= 8) {
                auto config = draft_.articulationReference;
                reference = truck_demo::ArticulationReference::fromDrawn(
                    drawnRawPoints_, config);
            } else if (
                draft_.articulationReference.kind !=
                truck_demo::ArticulationReferenceKind::drawn) {
                reference = truck_demo::ArticulationReference::fromConfig(
                    draft_.articulationReference);
            } else {
                return;
            }
            const auto curve = reference.curve(0.05);
            previewTimes_.reserve(curve.size());
            previewDegrees_.reserve(curve.size());
            for (const auto& point : curve) {
                previewTimes_.push_back(point.time);
                previewDegrees_.push_back(degrees(point.articulation));
            }
        } catch (const std::exception&) {
        }
    }

    void confirmDrawnArticulationReference() {
        try {
            applyDraftAngles();
            drawnReference_ = truck_demo::ArticulationReference::fromDrawn(
                drawnRawPoints_, draft_.articulationReference);
            session_.setArticulationReference(drawnReference_);
            draft_.articulationReference = drawnReference_.config();
            syncDraft();
            status_ = u8"手绘铰接角参考已确认";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void loadArticulationRecoveryExperiment() {
        try {
            applyDraftAngles();
            auto settings = session_.settings();
            const double articulation =
                std::abs(draft_.initialArticulation) > 1.0e-6
                    ? draft_.initialArticulation
                    : radians(15.0);
            settings.initialLateralError = 0.0;
            settings.initialLateralErrorRate = 0.0;
            settings.initialHeadingError = 0.0;
            settings.initialHeadingErrorRate = 0.0;
            settings.initialArticulation = articulation;
            settings.initialArticulationRate = 0.0;
            settings.initialSteering = 0.0;
            session_.configure(settings);
            syncDraft();
            fitRequested_ = true;
            simulationAccumulator_ = 0.0;
            status_ =
                u8"已加载铰接角恢复实验：仅设置初始铰接角，其余误差为零";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void exportSessionLog(bool automatic) {
        const std::string directory = truck_demo::makeRunDirectory();
        if (directory.empty()) {
            if (!automatic) {
                error_ = u8"无法创建 runs 记录目录";
            }
            return;
        }
        const std::string writeError = session_.exportRunLog(directory);
        if (!writeError.empty()) {
            if (!automatic) {
                error_ = writeError;
            }
            return;
        }
        autoExportedThisRun_ = true;
        status_ = (automatic ? std::string(u8"运行结束，已自动导出 ")
                             : std::string(u8"已导出记录 ")) +
                  directory;
    }

    void maybeAutoExportLog() {
        const auto state = session_.simulationState();
        if (autoExportedThisRun_) {
            return;
        }
        if (state == truck_demo::SimulationState::finished ||
            state == truck_demo::SimulationState::faulted) {
            exportSessionLog(true);
        }
    }

    void runSimulation() {
        try {
            if (pathEditor_.state() != truck_demo::PathEditorState::idle) {
                throw std::logic_error(
                    "confirm or cancel the user-drawn path before running");
            }
            drawingStroke_ = false;
            session_.start();
            simulationAccumulator_ = 0.0;
            autoExportedThisRun_ = false;
            error_.clear();
            status_ = u8"闭环仿真正在运行";
        } catch (const std::exception& exception) {
            error_ = exception.what();
        }
    }

    void renderWorkspace() {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const float maximumMapHeight =
            std::max(180.0f, available.y - 340.0f);
        const float mapHeight = std::clamp(
            telemetryFocus_ ? 190.0f : available.y * 0.48f,
            180.0f,
            maximumMapHeight);
        renderMap(ImVec2(available.x, mapHeight));
        ImGui::Spacing();
        telemetryPanel_.render(
            session_, darkTheme_, ImGui::GetContentRegionAvail());
    }

    ImVec2 worldToScreen(
        double x,
        double y,
        const ImVec2& minimum,
        const ImVec2& size) const {
        return {
            minimum.x + size.x * 0.5f +
                static_cast<float>((x - view_.centerX) * view_.pixelsPerMeter),
            minimum.y + size.y * 0.5f -
                static_cast<float>((y - view_.centerY) * view_.pixelsPerMeter)};
    }

    truck_model::Point2d screenToWorld(
        const ImVec2& point,
        const ImVec2& minimum,
        const ImVec2& size) const {
        return {
            view_.centerX +
                (point.x - minimum.x - size.x * 0.5f) /
                    view_.pixelsPerMeter,
            view_.centerY -
                (point.y - minimum.y - size.y * 0.5f) /
                    view_.pixelsPerMeter};
    }

    void renderMap(const ImVec2& requestedSize) {
        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 10.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::BeginChild("##TrajectoryMap", requestedSize, true);
        const ImVec2 minimum = ImGui::GetCursorScreenPos();
        const ImVec2 size = ImGui::GetContentRegionAvail();
        ImGui::InvisibleButton(
            "##MapInteraction",
            size,
            ImGuiButtonFlags_MouseButtonLeft |
                ImGuiButtonFlags_MouseButtonRight);
        const bool hovered = ImGui::IsItemHovered();
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const ImVec2 maximum{minimum.x + size.x, minimum.y + size.y};
        draw->PushClipRect(minimum, maximum, true);
        draw->AddRectFilled(
            minimum,
            maximum,
            darkTheme_ ? color(16, 22, 32) : color(247, 249, 252),
            9.0f);
        drawGrid(draw, minimum, size);

        if (fitRequested_ && size.x > 20.0f && size.y > 20.0f) {
            fitPath(size);
            fitRequested_ = false;
        }
        handleMapInput(hovered, minimum, size);
        drawPath(draw, minimum, size);
        drawHistory(draw, minimum, size);
        drawPrediction(draw, minimum, size);
        if (!pathEditor_.rawPoints().empty()) {
            drawPolyline(
                draw,
                pathEditor_.rawPoints(),
                minimum,
                size,
                color(255, 100, 82, 150),
                2.0f);
        }
        if (!pathEditor_.smoothedPoints().empty()) {
            drawPolyline(
                draw,
                pathEditor_.smoothedPoints(),
                minimum,
                size,
                darkTheme_ ? color(46, 220, 184) : color(0, 138, 110),
                3.5f);
        }
        if (!session_.path().empty()) {
            drawVehicle(draw, session_.currentVehicle(), minimum, size);
        }

        const bool drawing =
            pathEditor_.state() == truck_demo::PathEditorState::drawing;
        const bool reviewing =
            pathEditor_.state() == truck_demo::PathEditorState::review;
        draw->AddText(
            {minimum.x + 14.0f, minimum.y + 12.0f},
            drawing   ? color(255, 135, 105)
            : reviewing ? (darkTheme_ ? color(46, 220, 184)
                                      : color(0, 118, 92))
                        : (darkTheme_ ? color(169, 184, 207)
                                      : color(66, 79, 98)),
            drawing
                ? u8"绘制模式：按住左键绘制目标轨迹"
                : reviewing
                      ? u8"绿色为 Pure Pursuit 平滑预览，请点击顶部“确认轨迹”"
                      : u8"参考路廊 · 实际轨迹 · MPC 预测");
        draw->AddText(
            {minimum.x + 14.0f, maximum.y - 27.0f},
            darkTheme_ ? color(112, 130, 158) : color(92, 107, 128),
            u8"右键拖动平移  |  滚轮缩放  |  双击右键适应轨迹");
        draw->PopClipRect();
        ImGui::EndChild();
        ImGui::PopStyleVar(2);
    }

    void handleMapInput(
        bool hovered,
        const ImVec2& minimum,
        const ImVec2& size) {
        ImGuiIO& io = ImGui::GetIO();
        if (hovered && io.MouseWheel != 0.0f) {
            const auto before =
                screenToWorld(io.MousePos, minimum, size);
            view_.pixelsPerMeter = std::clamp(
                view_.pixelsPerMeter *
                    (io.MouseWheel > 0.0f ? 1.15 : 1.0 / 1.15),
                0.8,
                60.0);
            const auto after =
                screenToWorld(io.MousePos, minimum, size);
            view_.centerX += before.x - after.x;
            view_.centerY += before.y - after.y;
        }
        if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            view_.centerX -= io.MouseDelta.x / view_.pixelsPerMeter;
            view_.centerY += io.MouseDelta.y / view_.pixelsPerMeter;
        }
        if (hovered &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Right)) {
            fitRequested_ = true;
        }
        if (pathEditor_.state() !=
            truck_demo::PathEditorState::drawing) {
            return;
        }
        if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            pathEditor_.begin();
            pathEditor_.addPoint(
                screenToWorld(io.MousePos, minimum, size));
            drawingStroke_ = true;
        }
        if (drawingStroke_ &&
            ImGui::IsMouseDown(ImGuiMouseButton_Left) && hovered) {
            const auto point =
                screenToWorld(io.MousePos, minimum, size);
            pathEditor_.addPoint(point);
        }
        if (drawingStroke_ &&
            ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
            drawingStroke_ = false;
            finishDrawnPath();
        }
    }

    void finishDrawnPath() {
        try {
            pathEditor_.finish();
            error_.clear();
            status_ =
                u8"Pure Pursuit 平滑完成，请检查绿色预览并确认轨迹";
        } catch (const std::exception& exception) {
            error_ = exception.what();
            status_ = u8"轨迹无效，请绘制更长且更平缓的路径";
            pathEditor_.begin();
        }
    }

    void confirmSmoothedPath() {
        try {
            pathEditor_.confirm(
                [this](const truck_model::ReferencePath& path) {
                    session_.setPath(path);
                });
            fitRequested_ = true;
            error_.clear();
            status_ = u8"平滑轨迹已确认，可开始闭环仿真";
        } catch (const std::exception& exception) {
            error_ = exception.what();
            confirmationError_ = exception.what();
            openConfirmationError_ = true;
            status_ =
                u8"轨迹尚未确认：请降低车速或重新绘制更平缓的轨迹";
        }
    }

    void renderNotifications() {
        if (openConfirmationError_) {
            ImGui::OpenPopup(u8"轨迹无法确认");
            openConfirmationError_ = false;
        }
        if (ImGui::BeginPopupModal(
                u8"轨迹无法确认",
                nullptr,
                ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextWrapped(
                u8"当前轨迹在所设车速和执行器限制下不可安全跟踪：");
            ImGui::Spacing();
            ImGui::TextWrapped("%s", confirmationError_.c_str());
            ImGui::Spacing();
            ImGui::TextWrapped(
                u8"平滑预览会保留。可关闭提示后降低运行车速并应用参数，"
                u8"然后再次点击“确认轨迹”；也可以选择重新绘制。");
            ImGui::Spacing();
            if (accentButton(
                    u8"返回调整车速", ImVec2(130.0f, 34.0f), true)) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button(u8"重新绘制", ImVec2(100.0f, 34.0f))) {
                pathEditor_.begin();
                drawingStroke_ = false;
                confirmationError_.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    void fitPath(const ImVec2& size) {
        const bool usePreview =
            pathEditor_.state() == truck_demo::PathEditorState::review &&
            !pathEditor_.smoothedPoints().empty();
        if (!usePreview && session_.path().empty()) {
            return;
        }
        double minX = 0.0;
        double maxX = 0.0;
        double minY = 0.0;
        double maxY = 0.0;
        if (usePreview) {
            const auto& points = pathEditor_.smoothedPoints();
            minX = maxX = points.front().x;
            minY = maxY = points.front().y;
            for (const auto& point : points) {
                minX = std::min(minX, point.x);
                maxX = std::max(maxX, point.x);
                minY = std::min(minY, point.y);
                maxY = std::max(maxY, point.y);
            }
        } else {
            const auto& points = session_.path().points();
            minX = maxX = points.front().x;
            minY = maxY = points.front().y;
            for (const auto& point : points) {
                minX = std::min(minX, point.x);
                maxX = std::max(maxX, point.x);
                minY = std::min(minY, point.y);
                maxY = std::max(maxY, point.y);
            }
        }
        view_.centerX = 0.5 * (minX + maxX);
        view_.centerY = 0.5 * (minY + maxY);
        const double width = std::max(20.0, maxX - minX + 16.0);
        const double height = std::max(12.0, maxY - minY + 16.0);
        view_.pixelsPerMeter = std::clamp(
            std::min(size.x / width, size.y / height), 0.8, 40.0);
    }

    void drawGrid(
        ImDrawList* draw,
        const ImVec2& minimum,
        const ImVec2& size) const {
        const double spacing =
            view_.pixelsPerMeter > 14.0 ? 2.0
            : view_.pixelsPerMeter > 5.0 ? 5.0
                                         : 10.0;
        const auto topLeft = screenToWorld(minimum, minimum, size);
        const auto bottomRight = screenToWorld(
            {minimum.x + size.x, minimum.y + size.y}, minimum, size);
        for (double x = std::floor(topLeft.x / spacing) * spacing;
             x <= bottomRight.x;
             x += spacing) {
            const ImVec2 pixel = worldToScreen(x, 0.0, minimum, size);
            draw->AddLine(
                {pixel.x, minimum.y},
                {pixel.x, minimum.y + size.y},
                darkTheme_ ? color(31, 42, 58) : color(198, 206, 217),
                1.0f);
        }
        for (double y = std::floor(bottomRight.y / spacing) * spacing;
             y <= topLeft.y;
             y += spacing) {
            const ImVec2 pixel = worldToScreen(0.0, y, minimum, size);
            draw->AddLine(
                {minimum.x, pixel.y},
                {minimum.x + size.x, pixel.y},
                darkTheme_ ? color(31, 42, 58) : color(198, 206, 217),
                1.0f);
        }
    }

    template <typename Point>
    void drawPolyline(
        ImDrawList* draw,
        const std::vector<Point>& points,
        const ImVec2& minimum,
        const ImVec2& size,
        ImU32 lineColor,
        float thickness) const {
        if (points.size() < 2) {
            return;
        }
        std::vector<ImVec2> screen;
        screen.reserve(points.size());
        for (const auto& point : points) {
            screen.push_back(
                worldToScreen(point.x, point.y, minimum, size));
        }
        draw->AddPolyline(
            screen.data(),
            static_cast<int>(screen.size()),
            lineColor,
            ImDrawFlags_None,
            thickness);
    }

    void drawPath(
        ImDrawList* draw,
        const ImVec2& minimum,
        const ImVec2& size) const {
        if (session_.path().empty()) {
            return;
        }
        const auto& points = session_.path().points();
        drawPolyline(
            draw,
            points,
            minimum,
            size,
            darkTheme_ ? color(52, 65, 82) : color(178, 188, 201),
            std::clamp(
                static_cast<float>(3.6 * view_.pixelsPerMeter),
                7.0f,
                54.0f));
        drawPolyline(
            draw,
            points,
            minimum,
            size,
            darkTheme_ ? color(130, 150, 178) : color(105, 118, 137),
            1.5f);
    }

    void drawHistory(
        ImDrawList* draw,
        const ImVec2& minimum,
        const ImVec2& size) const {
        std::vector<truck_model::Point2d> points;
        points.reserve(session_.history().size());
        for (const auto& sample : session_.history()) {
            points.push_back(
                {sample.vehicle.truckX, sample.vehicle.truckY});
        }
        drawPolyline(
            draw, points, minimum, size, color(55, 160, 255), 3.0f);
    }

    void drawPrediction(
        ImDrawList* draw,
        const ImVec2& minimum,
        const ImVec2& size) const {
        const auto points = session_.predictedWorldPositions();
        drawPolyline(
            draw, points, minimum, size, color(255, 157, 72), 2.0f);
    }

    std::array<ImVec2, 4> rectangle(
        double centerX,
        double centerY,
        double heading,
        double length,
        double width,
        const ImVec2& minimum,
        const ImVec2& size) const {
        std::array<ImVec2, 4> result{};
        const std::array<std::array<double, 2>, 4> local{{
            {0.5 * length, 0.5 * width},
            {0.5 * length, -0.5 * width},
            {-0.5 * length, -0.5 * width},
            {-0.5 * length, 0.5 * width}}};
        for (std::size_t i = 0; i < result.size(); ++i) {
            result[i] = worldToScreen(
                centerX + local[i][0] * std::cos(heading) -
                    local[i][1] * std::sin(heading),
                centerY + local[i][0] * std::sin(heading) +
                    local[i][1] * std::cos(heading),
                minimum,
                size);
        }
        return result;
    }

    void body(
        ImDrawList* draw,
        double x,
        double y,
        double heading,
        double length,
        double width,
        ImU32 fill,
        const ImVec2& minimum,
        const ImVec2& size) const {
        const auto corners =
            rectangle(x, y, heading, length, width, minimum, size);
        draw->AddConvexPolyFilled(corners.data(), 4, fill);
        draw->AddPolyline(
            corners.data(),
            4,
            darkTheme_ ? color(205, 215, 228) : color(55, 65, 78),
            ImDrawFlags_Closed,
            1.5f);
    }

    void axle(
        ImDrawList* draw,
        double x,
        double y,
        double bodyHeading,
        double wheelHeading,
        double width,
        const ImVec2& minimum,
        const ImVec2& size) const {
        for (double side : {-1.0, 1.0}) {
            const double wheelX =
                x - side * 0.5 * width * std::sin(bodyHeading);
            const double wheelY =
                y + side * 0.5 * width * std::cos(bodyHeading);
            body(
                draw,
                wheelX,
                wheelY,
                wheelHeading,
                0.78,
                0.3,
                color(14, 17, 22),
                minimum,
                size);
        }
    }

    void drawVehicle(
        ImDrawList* draw,
        const truck_demo::VehicleSnapshot& pose,
        const ImVec2& minimum,
        const ImVec2& size) const {
        const auto& p = session_.settings().vehicle;

        const double trailerLength = p.a2 + p.b2 + 1.0;
        const double trailerCenterOffset = 0.5 * (p.b2 - 0.6);
        body(
            draw,
            pose.trailerX -
                trailerCenterOffset * std::cos(pose.trailerHeading),
            pose.trailerY -
                trailerCenterOffset * std::sin(pose.trailerHeading),
            pose.trailerHeading,
            trailerLength,
            2.55,
            color(48, 177, 158),
            minimum,
            size);
        const double trailerAxleDistance = p.a2 + p.b2;
        axle(
            draw,
            pose.hitchX -
                trailerAxleDistance * std::cos(pose.trailerHeading),
            pose.hitchY -
                trailerAxleDistance * std::sin(pose.trailerHeading),
            pose.trailerHeading,
            pose.trailerHeading,
            2.3,
            minimum,
            size);

        const double rear = -p.b1 - 0.55;
        const double front = p.a1 + 1.0;
        const double offset = 0.5 * (front + rear);
        body(
            draw,
            pose.truckX + offset * std::cos(pose.truckHeading),
            pose.truckY + offset * std::sin(pose.truckHeading),
            pose.truckHeading,
            front - rear,
            2.5,
            color(246, 151, 52),
            minimum,
            size);
        body(
            draw,
            pose.truckX + (p.a1 + 0.35) * std::cos(pose.truckHeading),
            pose.truckY + (p.a1 + 0.35) * std::sin(pose.truckHeading),
            pose.truckHeading,
            1.15,
            2.32,
            color(214, 84, 44),
            minimum,
            size);
        axle(
            draw,
            pose.truckX + p.a1 * std::cos(pose.truckHeading),
            pose.truckY + p.a1 * std::sin(pose.truckHeading),
            pose.truckHeading,
            pose.truckHeading + session_.steering(),
            2.25,
            minimum,
            size);
        axle(
            draw,
            pose.truckX - p.b1 * std::cos(pose.truckHeading),
            pose.truckY - p.b1 * std::sin(pose.truckHeading),
            pose.truckHeading,
            pose.truckHeading,
            2.25,
            minimum,
            size);
        const ImVec2 hitch =
            worldToScreen(pose.hitchX, pose.hitchY, minimum, size);
        const float hitchRadius = std::clamp(
            static_cast<float>(0.32 * view_.pixelsPerMeter), 3.0f, 9.0f);
        draw->AddCircleFilled(
            hitch,
            hitchRadius,
            darkTheme_ ? color(235, 239, 246) : color(214, 84, 44));
        draw->AddCircle(
            hitch,
            hitchRadius + 0.6f,
            darkTheme_ ? color(16, 21, 29) : color(82, 43, 30),
            0,
            std::clamp(hitchRadius * 0.28f, 1.2f, 2.2f));
    }

    truck_demo::DemoSession session_;
    truck_demo::DemoSettings draft_{};
    View view_{};
    truck_demo::PathEditor pathEditor_;
    truck_demo::TelemetryPanel telemetryPanel_;
    bool drawingStroke_{};
    bool fitRequested_{true};
    bool darkTheme_{true};
    bool themeChangeRequested_{};
    bool showSettings_{true};
    bool telemetryFocus_{};
    double simulationAccumulator_{};
    double maximumSteeringDegrees_{};
    double maximumSteeringRateDegrees_{};
    double initialHeadingDegrees_{};
    double initialHeadingRateDegrees_{};
    double initialArticulationDegrees_{};
    double initialArticulationRateDegrees_{};
    double initialSteeringDegrees_{};
    double amplitudeDegrees_{};
    double offsetDegrees_{};
    double phaseDegrees_{};
    double ekfMeasurementStdDegrees_{1.0};
    double ekfPhiProcessStdDegrees_{3.0};
    double ekfTrailerBiasStdDegrees_{0.81};
    double ekfLidarBiasStdDegrees_{0.0018};
    bool drawingReference_{};
    std::vector<truck_demo::TimeArticulationPoint> drawnRawPoints_;
    std::vector<double> previewTimes_;
    std::vector<double> previewDegrees_;
    truck_demo::ArticulationReference drawnReference_{};
    std::string status_{u8"默认场景已就绪"};
    std::string error_;
    std::string confirmationError_;
    bool openConfirmationError_{};
    bool autoExportedThisRun_{};
};

LRESULT WINAPI windowProcedure(
    HWND window,
    UINT message,
    WPARAM wParam,
    LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(window, message, wParam, lParam)) {
        return true;
    }
    switch (message) {
        case WM_SIZE:
            if (wParam != SIZE_MINIMIZED) {
                gResizeWidth = static_cast<UINT>(LOWORD(lParam));
                gResizeHeight = static_cast<UINT>(HIWORD(lParam));
            }
            return 0;
        case WM_DPICHANGED: {
            const auto* suggested = reinterpret_cast<RECT*>(lParam);
            SetWindowPos(
                window,
                nullptr,
                suggested->left,
                suggested->top,
                suggested->right - suggested->left,
                suggested->bottom - suggested->top,
                SWP_NOZORDER | SWP_NOACTIVATE);
            return 0;
        }
        case WM_GETMINMAXINFO: {
            auto* information = reinterpret_cast<MINMAXINFO*>(lParam);
            information->ptMinTrackSize = {1200, 760};
            return 0;
        }
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU) {
                return 0;
            }
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
        default:
            break;
    }
    return DefWindowProcW(window, message, wParam, lParam);
}

void configureThemeColors(bool dark) {
    if (dark) {
        ImGui::StyleColorsDark();
        ImPlot::StyleColorsDark();
    } else {
        ImGui::StyleColorsLight();
        ImPlot::StyleColorsLight();
    }
    ImGuiStyle& style = ImGui::GetStyle();
    ImPlotStyle& plotStyle = ImPlot::GetStyle();
    if (dark) {
        style.Colors[ImGuiCol_WindowBg] =
            ImVec4(0.035f, 0.048f, 0.072f, 1.0f);
        style.Colors[ImGuiCol_ChildBg] =
            ImVec4(0.055f, 0.072f, 0.105f, 1.0f);
        style.Colors[ImGuiCol_Border] =
            ImVec4(0.13f, 0.18f, 0.25f, 1.0f);
        style.Colors[ImGuiCol_FrameBg] =
            ImVec4(0.075f, 0.10f, 0.145f, 1.0f);
        style.Colors[ImGuiCol_FrameBgHovered] =
            ImVec4(0.10f, 0.16f, 0.23f, 1.0f);
        style.Colors[ImGuiCol_Button] =
            ImVec4(0.10f, 0.15f, 0.22f, 1.0f);
        style.Colors[ImGuiCol_ButtonHovered] =
            ImVec4(0.14f, 0.25f, 0.36f, 1.0f);
        style.Colors[ImGuiCol_Header] =
            ImVec4(0.09f, 0.16f, 0.23f, 1.0f);
        style.Colors[ImGuiCol_HeaderHovered] =
            ImVec4(0.12f, 0.25f, 0.36f, 1.0f);
        plotStyle.Colors[ImPlotCol_AxisGrid] =
            ImVec4(0.24f, 0.30f, 0.39f, 1.0f);
        plotStyle.Colors[ImPlotCol_PlotBg] =
            ImVec4(0.035f, 0.048f, 0.072f, 1.0f);
        plotStyle.Colors[ImPlotCol_PlotBorder] =
            ImVec4(0.20f, 0.27f, 0.36f, 1.0f);
        plotStyle.Colors[ImPlotCol_FrameBg] =
            ImVec4(0.055f, 0.072f, 0.105f, 1.0f);
    } else {
        style.Colors[ImGuiCol_WindowBg] =
            ImVec4(0.94f, 0.96f, 0.98f, 1.0f);
        style.Colors[ImGuiCol_ChildBg] =
            ImVec4(0.985f, 0.99f, 1.0f, 1.0f);
        style.Colors[ImGuiCol_Border] =
            ImVec4(0.76f, 0.80f, 0.86f, 1.0f);
        style.Colors[ImGuiCol_FrameBg] =
            ImVec4(0.90f, 0.93f, 0.97f, 1.0f);
        style.Colors[ImGuiCol_FrameBgHovered] =
            ImVec4(0.82f, 0.89f, 0.96f, 1.0f);
        style.Colors[ImGuiCol_Button] =
            ImVec4(0.86f, 0.90f, 0.95f, 1.0f);
        style.Colors[ImGuiCol_ButtonHovered] =
            ImVec4(0.75f, 0.85f, 0.95f, 1.0f);
        style.Colors[ImGuiCol_Header] =
            ImVec4(0.82f, 0.89f, 0.96f, 1.0f);
        style.Colors[ImGuiCol_HeaderHovered] =
            ImVec4(0.70f, 0.82f, 0.94f, 1.0f);
        plotStyle.Colors[ImPlotCol_AxisGrid] =
            ImVec4(0.72f, 0.76f, 0.82f, 1.0f);
        plotStyle.Colors[ImPlotCol_PlotBg] =
            ImVec4(0.975f, 0.982f, 0.992f, 1.0f);
        plotStyle.Colors[ImPlotCol_PlotBorder] =
            ImVec4(0.72f, 0.77f, 0.84f, 1.0f);
        plotStyle.Colors[ImPlotCol_FrameBg] =
            ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    }
}

void configureStyle(float scale) {
    configureThemeColors(true);
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.ChildRounding = 9.0f;
    style.FrameRounding = 6.0f;
    style.GrabRounding = 5.0f;
    style.PopupRounding = 7.0f;
    style.ScrollbarRounding = 7.0f;
    style.FramePadding = ImVec2(10.0f, 7.0f);
    style.ItemSpacing = ImVec2(9.0f, 8.0f);
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize = 1.0f;
    style.ScaleAllSizes(scale);
}

}  // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR, int) {
    ImGui_ImplWin32_EnableDpiAwareness();
    const float scale = ImGui_ImplWin32_GetDpiScaleForMonitor(
        MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));
    WNDCLASSEXW windowClass{
        sizeof(windowClass),
        CS_CLASSDC,
        windowProcedure,
        0L,
        0L,
        instance,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        L"TruckMpcStudioDx11",
        nullptr};
    RegisterClassExW(&windowClass);
    HWND window = CreateWindowW(
        windowClass.lpszClassName,
        L"TruckModel MPC Studio 2.8.1",
        WS_OVERLAPPEDWINDOW,
        80,
        60,
        static_cast<int>(1560 * scale),
        static_cast<int>(940 * scale),
        nullptr,
        nullptr,
        instance,
        nullptr);
    if (!createDevice(window)) {
        cleanupDevice();
        UnregisterClassW(windowClass.lpszClassName, instance);
        return EXIT_FAILURE;
    }
    ShowWindow(window, SW_SHOWDEFAULT);
    UpdateWindow(window);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    configureStyle(scale);
    ImFont* chineseFont = io.Fonts->AddFontFromFileTTF(
        "C:\\Windows\\Fonts\\msyh.ttc",
        17.0f * scale,
        nullptr,
        io.Fonts->GetGlyphRangesChineseSimplifiedCommon());
    if (!chineseFont) {
        io.Fonts->AddFontDefault();
    }
    ImGui_ImplWin32_Init(window);
    ImGui_ImplDX11_Init(gDevice, gDeviceContext);

    StudioApp application;
    bool done = false;
    while (!done) {
        MSG message{};
        while (PeekMessageW(&message, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&message);
            DispatchMessageW(&message);
            if (message.message == WM_QUIT) {
                done = true;
            }
        }
        if (done) {
            break;
        }
        if (gResizeWidth != 0 && gResizeHeight != 0) {
            cleanupRenderTarget();
            gSwapChain->ResizeBuffers(
                0,
                gResizeWidth,
                gResizeHeight,
                DXGI_FORMAT_UNKNOWN,
                0);
            gResizeWidth = gResizeHeight = 0;
            createRenderTarget();
        }

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();
        application.frame(ImGui::GetIO().DeltaTime);
        ImGui::Render();

        const ImVec4 background =
            ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
        const float clearColor[4] = {
            background.x, background.y, background.z, background.w};
        gDeviceContext->OMSetRenderTargets(1, &gRenderTarget, nullptr);
        gDeviceContext->ClearRenderTargetView(gRenderTarget, clearColor);
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        gSwapChain->Present(1, 0);
    }

    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    cleanupDevice();
    DestroyWindow(window);
    UnregisterClassW(windowClass.lpszClassName, instance);
    return 0;
}
