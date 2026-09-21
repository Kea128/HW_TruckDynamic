#include "articulation_reference.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace truck_demo {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRampDuration = 0.15;

bool isFinite(double value) {
    return std::isfinite(value);
}

void requirePositive(std::ostringstream& errors, double value, const char* name) {
    if (!(value > 0.0) || !isFinite(value)) {
        errors << name << " must be finite and > 0; ";
    }
}

void smoothRates(std::vector<ArticulationSample>& samples, int passes) {
    if (samples.size() < 3) {
        return;
    }
    std::vector<double> rates(samples.size());
    for (int pass = 0; pass < passes; ++pass) {
        for (std::size_t i = 0; i < samples.size(); ++i) {
            rates[i] = samples[i].rate;
        }
        for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
            samples[i].rate = 0.25 * rates[i - 1] + 0.5 * rates[i] +
                              0.25 * rates[i + 1];
        }
        for (std::size_t i = samples.size() - 2; i > 0; --i) {
            samples[i].rate = 0.25 * samples[i - 1].rate + 0.5 * samples[i].rate +
                              0.25 * samples[i + 1].rate;
        }
    }
}

}  // namespace

std::string ArticulationReferenceConfig::validationError() const {
    std::ostringstream errors;
    requirePositive(errors, duration, "duration");
    requirePositive(errors, sampleSpacing, "sampleSpacing");
    if (!isFinite(amplitude) || amplitude < 0.0) {
        errors << "amplitude must be finite and >= 0; ";
    }
    if (!isFinite(offset)) {
        errors << "offset must be finite; ";
    }
    if (kind == ArticulationReferenceKind::sine) {
        requirePositive(errors, frequency, "frequency");
        if (!isFinite(phase)) {
            errors << "phase must be finite; ";
        }
    }
    if (kind == ArticulationReferenceKind::periodicStep) {
        requirePositive(errors, period, "period");
        if (!isFinite(dutyCycle) || dutyCycle < 0.05 || dutyCycle > 0.95) {
            errors << "dutyCycle must be in [0.05, 0.95]; ";
        }
    }
    if (std::abs(amplitude) + std::abs(offset) > kMaximumArticulationReference) {
        errors << "articulation reference peak must be within +/-0.7 rad; ";
    }
    return errors.str();
}

ArticulationReference ArticulationReference::fromConfig(
    ArticulationReferenceConfig config) {
    const auto error = config.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
    ArticulationReference reference;
    reference.config_ = config;
    return reference;
}

ArticulationReference ArticulationReference::fromDrawn(
    const std::vector<TimeArticulationPoint>& points,
    ArticulationReferenceConfig config) {
    config.kind = ArticulationReferenceKind::drawn;
    const auto error = config.validationError();
    if (!error.empty()) {
        throw std::invalid_argument(error);
    }
    if (points.size() < 8) {
        throw std::invalid_argument(
            "drawn articulation reference needs at least 8 points");
    }
    std::vector<TimeArticulationPoint> sorted = points;
    std::sort(
        sorted.begin(),
        sorted.end(),
        [](const TimeArticulationPoint& lhs, const TimeArticulationPoint& rhs) {
            return lhs.time < rhs.time;
        });
    std::vector<TimeArticulationPoint> unique;
    unique.reserve(sorted.size());
    for (const auto& point : sorted) {
        if (!isFinite(point.time) || !isFinite(point.articulation)) {
            throw std::invalid_argument(
                "drawn articulation points must be finite");
        }
        if (std::abs(point.articulation) > kMaximumArticulationReference) {
            throw std::invalid_argument(
                "drawn articulation exceeds +/-0.7 rad");
        }
        if (unique.empty() || point.time - unique.back().time >= 1.0e-4) {
            unique.push_back(point);
        } else {
            unique.back() = point;
        }
    }
    if (unique.size() < 8 || unique.back().time - unique.front().time < 2.0) {
        throw std::invalid_argument(
            "drawn articulation reference must cover at least 2 s");
    }

    config.duration = unique.back().time - unique.front().time;
    const double start = unique.front().time;
    ArticulationReference reference;
    reference.config_ = config;
    for (double time = unique.front().time;
         time <= unique.back().time + 0.5 * config.sampleSpacing;
         time += config.sampleSpacing) {
        const double query = std::min(time, unique.back().time);
        auto upper = std::lower_bound(
            unique.begin(),
            unique.end(),
            query,
            [](const TimeArticulationPoint& point, double value) {
                return point.time < value;
            });
        ArticulationSample sample;
        if (upper == unique.begin()) {
            sample.value = unique.front().articulation;
        } else if (upper == unique.end()) {
            sample.value = unique.back().articulation;
        } else {
            const auto& before = *(upper - 1);
            const double span = upper->time - before.time;
            const double ratio =
                span > 0.0 ? (query - before.time) / span : 0.0;
            sample.value = before.articulation +
                           ratio * (upper->articulation - before.articulation);
        }
        reference.drawnTimes_.push_back(query - start);
        reference.drawnValues_.push_back(sample);
    }
    for (std::size_t i = 0; i < reference.drawnValues_.size(); ++i) {
        if (i == 0) {
            const double dt =
                reference.drawnTimes_[1] - reference.drawnTimes_[0];
            reference.drawnValues_[i].rate =
                (reference.drawnValues_[1].value -
                 reference.drawnValues_[0].value) /
                std::max(dt, 1.0e-9);
        } else if (i + 1 == reference.drawnValues_.size()) {
            const double dt = reference.drawnTimes_[i] -
                              reference.drawnTimes_[i - 1];
            reference.drawnValues_[i].rate =
                (reference.drawnValues_[i].value -
                 reference.drawnValues_[i - 1].value) /
                std::max(dt, 1.0e-9);
        } else {
            const double dt = reference.drawnTimes_[i + 1] -
                              reference.drawnTimes_[i - 1];
            reference.drawnValues_[i].rate =
                (reference.drawnValues_[i + 1].value -
                 reference.drawnValues_[i - 1].value) /
                std::max(dt, 1.0e-9);
        }
    }
    smoothRates(reference.drawnValues_, 3);
    return reference;
}

ArticulationSample ArticulationReference::sample(double time) const {
    if (!isFinite(time)) {
        throw std::invalid_argument("reference time must be finite");
    }
    if (config_.kind == ArticulationReferenceKind::none) {
        return {};
    }
    if (config_.kind == ArticulationReferenceKind::drawn) {
        return evaluateDrawn(time);
    }
    return evaluateAnalytic(time);
}

std::vector<truck_model::Vector<6>> ArticulationReference::preview(
    double startTime,
    double sampleTime,
    std::size_t horizon) const {
    std::vector<truck_model::Vector<6>> result(horizon + 1);
    for (std::size_t i = 0; i <= horizon; ++i) {
        const auto sample =
            this->sample(startTime + static_cast<double>(i) * sampleTime);
        result[i][4] = sample.value;
        result[i][5] = sample.rate;
    }
    return result;
}

std::vector<TimeArticulationPoint> ArticulationReference::curve(
    double sampleTime) const {
    std::vector<TimeArticulationPoint> result;
    if (!(sampleTime > 0.0) || config_.kind == ArticulationReferenceKind::none) {
        return result;
    }
    const double end = std::max(config_.duration, sampleTime);
    for (double time = 0.0; time <= end + 0.5 * sampleTime; time += sampleTime) {
        result.push_back({time, sample(time).value});
    }
    return result;
}

const ArticulationReferenceConfig&
ArticulationReference::config() const noexcept {
    return config_;
}

bool ArticulationReference::empty() const noexcept {
    return config_.kind == ArticulationReferenceKind::none;
}

ArticulationSample ArticulationReference::evaluateAnalytic(double time) const {
    const double clamped = std::clamp(time, 0.0, config_.duration);
    ArticulationSample sample;
    if (config_.kind == ArticulationReferenceKind::sine) {
        const double omega = 2.0 * kPi * config_.frequency;
        const double argument = omega * clamped + config_.phase;
        sample.value =
            config_.amplitude * std::sin(argument) + config_.offset;
        sample.rate = config_.amplitude * omega * std::cos(argument);
        if (time > config_.duration) {
            sample.rate = 0.0;
        }
        return sample;
    }

    const double period = config_.period;
    const double highDuration = config_.dutyCycle * period;
    const double lowDuration = period - highDuration;
    const double ramp = std::min(
        {kRampDuration, 0.25 * highDuration, 0.25 * lowDuration});
    const double high = config_.offset + config_.amplitude;
    const double low = config_.offset - config_.amplitude;
    const double phaseTime = std::fmod(clamped, period);
    if (phaseTime < ramp) {
        const double ratio = ramp > 0.0 ? phaseTime / ramp : 1.0;
        sample.value = low + ratio * (high - low);
        sample.rate = (high - low) / std::max(ramp, 1.0e-9);
    } else if (phaseTime < highDuration - ramp) {
        sample.value = high;
    } else if (phaseTime < highDuration) {
        const double ratio =
            ramp > 0.0 ? (phaseTime - (highDuration - ramp)) / ramp : 1.0;
        sample.value = high + ratio * (low - high);
        sample.rate = (low - high) / std::max(ramp, 1.0e-9);
    } else if (phaseTime < period - ramp) {
        sample.value = low;
    } else {
        const double ratio =
            ramp > 0.0 ? (phaseTime - (period - ramp)) / ramp : 1.0;
        sample.value = low + ratio * (high - low);
        sample.rate = (high - low) / std::max(ramp, 1.0e-9);
    }
    if (time > config_.duration) {
        sample = evaluateAnalytic(config_.duration);
        sample.rate = 0.0;
    }
    return sample;
}

ArticulationSample ArticulationReference::evaluateDrawn(double time) const {
    if (drawnTimes_.empty()) {
        return {};
    }
    const double clamped =
        std::clamp(time, drawnTimes_.front(), drawnTimes_.back());
    auto upper = std::lower_bound(
        drawnTimes_.begin(), drawnTimes_.end(), clamped);
    if (upper == drawnTimes_.begin()) {
        return drawnValues_.front();
    }
    if (upper == drawnTimes_.end()) {
        auto last = drawnValues_.back();
        if (time > drawnTimes_.back()) {
            last.rate = 0.0;
        }
        return last;
    }
    const std::size_t index =
        static_cast<std::size_t>(upper - drawnTimes_.begin());
    const auto& before = drawnValues_[index - 1];
    const auto& after = drawnValues_[index];
    const double span = drawnTimes_[index] - drawnTimes_[index - 1];
    const double ratio =
        span > 0.0 ? (clamped - drawnTimes_[index - 1]) / span : 0.0;
    ArticulationSample sample;
    sample.value = before.value + ratio * (after.value - before.value);
    sample.rate = before.rate + ratio * (after.rate - before.rate);
    if (time > drawnTimes_.back()) {
        sample.rate = 0.0;
    }
    return sample;
}

}  // namespace truck_demo
