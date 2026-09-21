#pragma once

#include "truck_model/articulated_vehicle.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace truck_demo {

inline constexpr double kMaximumArticulationReference = 0.7;

enum class ArticulationReferenceKind {
    none,
    sine,
    periodicStep,
    drawn
};

struct ArticulationReferenceConfig {
    ArticulationReferenceKind kind{ArticulationReferenceKind::none};
    double duration{20.0};
    double amplitude{0.15};
    double frequency{0.15};
    double phase{0.0};
    double offset{0.0};
    double period{6.0};
    double dutyCycle{0.5};
    double sampleSpacing{0.05};

    [[nodiscard]] std::string validationError() const;
};

struct ArticulationSample {
    double value{};
    double rate{};
};

struct TimeArticulationPoint {
    double time{};
    double articulation{};
};

class ArticulationReference {
public:
    [[nodiscard]] static ArticulationReference fromConfig(
        ArticulationReferenceConfig config);
    [[nodiscard]] static ArticulationReference fromDrawn(
        const std::vector<TimeArticulationPoint>& points,
        ArticulationReferenceConfig config = {});

    [[nodiscard]] ArticulationSample sample(double time) const;
    [[nodiscard]] std::vector<truck_model::Vector<6>> preview(
        double startTime,
        double sampleTime,
        std::size_t horizon) const;
    [[nodiscard]] std::vector<TimeArticulationPoint> curve(
        double sampleTime = 0.05) const;

    [[nodiscard]] const ArticulationReferenceConfig& config() const noexcept;
    [[nodiscard]] bool empty() const noexcept;

private:
    ArticulationSample evaluateAnalytic(double time) const;
    ArticulationSample evaluateDrawn(double time) const;

    ArticulationReferenceConfig config_{};
    std::vector<ArticulationSample> drawnValues_;
    std::vector<double> drawnTimes_;
};

}  // namespace truck_demo
