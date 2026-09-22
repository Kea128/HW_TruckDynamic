#pragma once

#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/matrix_exponential.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

namespace truck_model {

// Why a measurement did not reach the Joseph update.
enum class MeasurementOutcome {
    accepted,
    gated,             // Mahalanobis test rejected the innovation
    duplicate,         // same identifier already in the timeline
    staleBeyondWindow, // scan stamp older than the retained history
    aheadOfInputs,     // scan stamp newer than the newest input event
    notInitialized,
    nonFinite
};

[[nodiscard]] inline const char* measurementOutcomeName(
    MeasurementOutcome outcome) {
    switch (outcome) {
        case MeasurementOutcome::accepted:
            return "accepted";
        case MeasurementOutcome::gated:
            return "gated";
        case MeasurementOutcome::duplicate:
            return "duplicate";
        case MeasurementOutcome::staleBeyondWindow:
            return "staleBeyondWindow";
        case MeasurementOutcome::aheadOfInputs:
            return "aheadOfInputs";
        case MeasurementOutcome::nonFinite:
            return "nonFinite";
        case MeasurementOutcome::notInitialized:
        default:
            return "notInitialized";
    }
}

struct DelayedEkfLimits {
    // Must exceed the largest expected scan latency plus one control period,
    // otherwise late packets fall outside the replay window and are discarded.
    double historyHorizon{0.55};
    double mahalanobisGate{9.0};
    int consecutiveRejectLimit{3};
    // Open-loop tolerance measured on scan stamps, i.e. how stale the newest
    // fused information may be before the process noise is inflated.
    double lostTimeout{0.6};
    double coastingProcessNoiseScale{4.0};
    std::size_t maximumTimelineEntries{4096};

    [[nodiscard]] std::string validationError() const {
        std::string errors;
        const auto requirePositive = [&errors](double value, const char* name) {
            if (!(value > 0.0) || !std::isfinite(value)) {
                errors += name;
                errors += " must be finite and > 0; ";
            }
        };
        requirePositive(historyHorizon, "historyHorizon");
        requirePositive(mahalanobisGate, "mahalanobisGate");
        requirePositive(lostTimeout, "lostTimeout");
        if (!(coastingProcessNoiseScale >= 1.0) ||
            !std::isfinite(coastingProcessNoiseScale)) {
            errors += "coastingProcessNoiseScale must be finite and >= 1; ";
        }
        if (consecutiveRejectLimit < 1) {
            errors += "consecutiveRejectLimit must be >= 1; ";
        }
        if (maximumTimelineEntries < 2) {
            errors += "maximumTimelineEntries must be >= 2; ";
        }
        return errors;
    }
};

// Out-of-sequence-measurement EKF shell.
//
// The filter keeps an ordered timeline of events (input samples and lidar
// scans) together with the posterior snapshot that follows each event. A late
// scan is inserted at its own stamp, the state is rewound to the entry just
// before it, and every later event is replayed in time order. Because the
// replay re-applies stored measurements as well as inputs, a packet that
// arrives out of order can no longer erase corrections that were already fused
// at a later stamp.
//
// The propagation interval that contains a scan stamp is split exactly at that
// stamp; both halves keep the input that governed the original interval, so
// alignment error is zero rather than half an input period.
//
// Process-noise inflation is derived during the forward walk from the stamps of
// the measurements the timeline holds, never from arrival order. Together with
// the full replay this makes the posterior a function of the event set alone:
// delivering the same packets in any order yields the same state.
//
// Model requirements:
//   static constexpr std::size_t kStateSize
//   using Inputs = ...
//   void propagate(Vector<kStateSize>& state, Matrix<kStateSize, kStateSize>&
//                  covariance, const Inputs& inputs, double dt,
//                  double processNoiseScale) const
//   double predictMeasurement(const Vector<kStateSize>& state) const
//   Vector<kStateSize> measurementJacobian(
//                  const Vector<kStateSize>& state) const
//   double measurementVariance() const
//   double residual(double measurement, double predicted) const
//   void normalize(Vector<kStateSize>& state) const
template <typename Model>
class DelayedEkf {
public:
    static constexpr std::size_t kStateSize = Model::kStateSize;
    using Inputs = typename Model::Inputs;
    using State = Vector<kStateSize>;
    using Covariance = Matrix<kStateSize, kStateSize>;

    struct MeasurementReport {
        MeasurementOutcome outcome{MeasurementOutcome::notInitialized};
        double innovation{};
        double innovationCovariance{};
        double mahalanobis{};
        State gain{};
        double alignedStamp{};
        double stamp{};
        std::size_t replayedEntries{};
    };

    void configure(Model model, DelayedEkfLimits limits) {
        const auto error = limits.validationError();
        if (!error.empty()) {
            throw std::invalid_argument(error);
        }
        model_ = std::move(model);
        limits_ = limits;
        configured_ = true;
        initialized_ = false;
        entries_.clear();
    }

    void reset(
        double time,
        const State& state,
        const Covariance& covariance,
        const Inputs& inputs) {
        requireConfigured();
        if (!std::isfinite(time)) {
            throw std::invalid_argument("estimator reset time must be finite");
        }
        for (const double value : state) {
            if (!std::isfinite(value)) {
                throw std::invalid_argument(
                    "estimator reset state must be finite");
            }
        }
        Entry anchor;
        anchor.time = time;
        anchor.governingInputs = inputs;
        anchor.state = state;
        anchor.covariance = covariance;
        model_.normalize(anchor.state);
        anchor.lastAcceptedStamp = time;
        anchor.everAccepted = false;
        anchor.consecutiveRejects = 0;
        entries_.clear();
        entries_.push_back(anchor);
        initialized_ = true;
    }

    // Advances the filter to inputs.time. Samples that are not newer than the
    // newest entry only refresh the governing input; they never rewind time.
    void predict(const Inputs& inputs, double time) {
        requireInitialized();
        if (!std::isfinite(time)) {
            throw std::invalid_argument("estimator input time must be finite");
        }
        Entry& newest = entries_.back();
        if (time <= newest.time + 1.0e-15) {
            newest.governingInputs = inputs;
            return;
        }

        Entry next;
        next.time = time;
        next.governingInputs = inputs;
        next.state = newest.state;
        next.covariance = newest.covariance;
        next.lastAcceptedStamp = newest.lastAcceptedStamp;
        next.everAccepted = newest.everAccepted;
        next.consecutiveRejects = newest.consecutiveRejects;
        model_.propagate(
            next.state,
            next.covariance,
            inputs,
            time - newest.time,
            noiseScaleAt(newest));
        entries_.push_back(next);
        trim();
    }

    [[nodiscard]] MeasurementReport update(
        double stamp,
        double value,
        std::uint64_t identifier) {
        MeasurementReport report;
        report.stamp = stamp;
        if (!configured_ || !initialized_ || entries_.empty()) {
            report.outcome = MeasurementOutcome::notInitialized;
            return report;
        }
        if (!std::isfinite(stamp) || !std::isfinite(value)) {
            report.outcome = MeasurementOutcome::nonFinite;
            return report;
        }
        if (stamp < entries_.front().time - 1.0e-9) {
            report.outcome = MeasurementOutcome::staleBeyondWindow;
            registerRejection();
            return report;
        }
        if (stamp > entries_.back().time + 1.0e-9) {
            // No input covers this stamp yet. Call predict() first; extending
            // the newest input past its sample would fabricate information.
            report.outcome = MeasurementOutcome::aheadOfInputs;
            return report;
        }
        for (const Entry& entry : entries_) {
            if (entry.hasMeasurement && entry.measurementId == identifier) {
                report.outcome = MeasurementOutcome::duplicate;
                return report;
            }
        }

        const std::size_t insertion = insertMeasurement(stamp, value, identifier);
        replayFrom(insertion - 1, identifier, &report);
        return report;
    }

    [[nodiscard]] const State& state() const { return entries_.back().state; }
    [[nodiscard]] const Covariance& covariance() const {
        return entries_.back().covariance;
    }
    [[nodiscard]] double time() const { return entries_.back().time; }
    [[nodiscard]] const Inputs& latestInputs() const {
        return entries_.back().governingInputs;
    }
    [[nodiscard]] double lastAcceptedStamp() const {
        return entries_.back().lastAcceptedStamp;
    }
    [[nodiscard]] bool everAccepted() const {
        return entries_.back().everAccepted;
    }
    [[nodiscard]] int consecutiveRejects() const {
        return entries_.back().consecutiveRejects;
    }
    // Age of the newest fused scan. This is how stale the information behind
    // the current estimate is, which is not the same as link health. Before the
    // first accepted scan it measures back to the reset instant; that reference
    // is carried along the timeline so trimming cannot shorten it.
    [[nodiscard]] double informationAge() const {
        const Entry& newest = entries_.back();
        return newest.time - newest.lastAcceptedStamp;
    }
    [[nodiscard]] bool coasting() const {
        return noiseScaleAt(entries_.back()) > 1.0;
    }
    [[nodiscard]] std::size_t timelineSize() const { return entries_.size(); }
    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] const Model& model() const { return model_; }
    [[nodiscard]] Model& model() { return model_; }
    [[nodiscard]] const DelayedEkfLimits& limits() const { return limits_; }

private:
    struct Entry {
        double time{};
        Inputs governingInputs{};
        bool hasMeasurement{};
        double measurement{};
        std::uint64_t measurementId{};
        bool measurementAccepted{};
        State state{};
        Covariance covariance{};
        double lastAcceptedStamp{};
        bool everAccepted{};
        int consecutiveRejects{};
    };

    void requireConfigured() const {
        if (!configured_) {
            throw std::logic_error("DelayedEkf is not configured");
        }
    }

    void requireInitialized() const {
        requireConfigured();
        if (!initialized_ || entries_.empty()) {
            throw std::logic_error("DelayedEkf is not initialized");
        }
    }

    [[nodiscard]] double noiseScaleAt(const Entry& entry) const {
        const bool stale =
            entry.time - entry.lastAcceptedStamp > limits_.lostTimeout;
        const bool rejecting =
            entry.consecutiveRejects >= limits_.consecutiveRejectLimit;
        return (stale || rejecting) ? limits_.coastingProcessNoiseScale : 1.0;
    }

    void registerRejection() {
        Entry& newest = entries_.back();
        ++newest.consecutiveRejects;
    }

    // Inserts the scan in stamp order and splits the containing interval. Both
    // halves inherit the input that governed the original interval, so the
    // split is exact rather than snapped to a neighbouring frame.
    [[nodiscard]] std::size_t insertMeasurement(
        double stamp,
        double value,
        std::uint64_t identifier) {
        std::size_t position = entries_.size();
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            if (entries_[i].time > stamp) {
                position = i;
                break;
            }
        }
        if (position == 0) {
            position = 1;
        }

        Entry inserted;
        inserted.time = stamp;
        inserted.hasMeasurement = true;
        inserted.measurement = value;
        inserted.measurementId = identifier;
        inserted.governingInputs = position < entries_.size()
                                       ? entries_[position].governingInputs
                                       : entries_.back().governingInputs;
        entries_.insert(
            entries_.begin() + static_cast<std::ptrdiff_t>(position), inserted);
        return position;
    }

    void replayFrom(
        std::size_t baseIndex,
        std::uint64_t reportId,
        MeasurementReport* report) {
        std::size_t replayed = 0;
        for (std::size_t i = baseIndex + 1; i < entries_.size(); ++i) {
            const Entry& previous = entries_[i - 1];
            Entry& current = entries_[i];
            current.state = previous.state;
            current.covariance = previous.covariance;
            current.lastAcceptedStamp = previous.lastAcceptedStamp;
            current.everAccepted = previous.everAccepted;
            current.consecutiveRejects = previous.consecutiveRejects;

            const double dt = current.time - previous.time;
            if (dt > 0.0) {
                model_.propagate(
                    current.state,
                    current.covariance,
                    current.governingInputs,
                    dt,
                    noiseScaleAt(previous));
            }

            if (current.hasMeasurement) {
                MeasurementReport local;
                local.stamp = current.time;
                local.alignedStamp = current.time;
                applyMeasurement(current, local);
                if (current.measurementId == reportId && report != nullptr) {
                    *report = local;
                    report->replayedEntries = 0;
                }
            }
            ++replayed;
        }
        if (report != nullptr) {
            report->replayedEntries = replayed;
        }
    }

    void applyMeasurement(Entry& entry, MeasurementReport& report) {
        const State jacobian = model_.measurementJacobian(entry.state);
        const double variance = model_.measurementVariance();
        const double predicted = model_.predictMeasurement(entry.state);
        const double innovation = model_.residual(entry.measurement, predicted);

        State covarianceTimesH{};
        for (std::size_t row = 0; row < kStateSize; ++row) {
            double accumulated = 0.0;
            for (std::size_t column = 0; column < kStateSize; ++column) {
                accumulated += entry.covariance[row][column] * jacobian[column];
            }
            covarianceTimesH[row] = accumulated;
        }
        double innovationCovariance = variance;
        for (std::size_t i = 0; i < kStateSize; ++i) {
            innovationCovariance += jacobian[i] * covarianceTimesH[i];
        }
        if (!(innovationCovariance > 0.0) ||
            !std::isfinite(innovationCovariance)) {
            throw std::runtime_error(
                "articulation innovation covariance is invalid");
        }
        const double mahalanobis =
            innovation * innovation / innovationCovariance;

        report.innovation = innovation;
        report.innovationCovariance = innovationCovariance;
        report.mahalanobis = mahalanobis;

        if (!(mahalanobis <= limits_.mahalanobisGate)) {
            entry.measurementAccepted = false;
            ++entry.consecutiveRejects;
            report.outcome = MeasurementOutcome::gated;
            return;
        }

        State gain{};
        for (std::size_t i = 0; i < kStateSize; ++i) {
            gain[i] = covarianceTimesH[i] / innovationCovariance;
            entry.state[i] += gain[i] * innovation;
        }
        model_.normalize(entry.state);

        // Joseph form keeps the posterior symmetric positive semi-definite even
        // when the gain and the prior are slightly inconsistent numerically.
        Matrix<kStateSize, kStateSize> factor = identityMatrix<kStateSize>();
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                factor[row][column] -= gain[row] * jacobian[column];
            }
        }
        const auto left = matrixProduct(factor, entry.covariance);
        auto updated = matrixProduct(left, transposed(factor));
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                updated[row][column] += gain[row] * variance * gain[column];
            }
        }
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = row + 1; column < kStateSize; ++column) {
                const double mean =
                    0.5 * (updated[row][column] + updated[column][row]);
                updated[row][column] = mean;
                updated[column][row] = mean;
            }
        }
        entry.covariance = updated;

        entry.measurementAccepted = true;
        entry.lastAcceptedStamp = entry.time;
        entry.everAccepted = true;
        entry.consecutiveRejects = 0;
        report.gain = gain;
        report.outcome = MeasurementOutcome::accepted;
    }

    void trim() {
        while (entries_.size() > 2 &&
               entries_.back().time - entries_.front().time >
                   limits_.historyHorizon + 1.0e-12) {
            entries_.pop_front();
        }
        while (entries_.size() > limits_.maximumTimelineEntries) {
            entries_.pop_front();
        }
    }

    Model model_{};
    DelayedEkfLimits limits_{};
    bool configured_{};
    bool initialized_{};
    std::deque<Entry> entries_;
};

}  // namespace truck_model
