#pragma once

#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/matrix_exponential.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace truck_model {

// Why a measurement did not reach the Joseph update.
enum class MeasurementOutcome {
    accepted,
    gated,             // Mahalanobis test rejected the innovation
    duplicate,         // same scan stamp as the previous measurement
    outOfOrder,        // scan stamp older than one already fused
    staleBeyondWindow, // scan stamp older than the retained history
    aheadOfInputs,     // scan stamp newer than the newest input sample
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
        case MeasurementOutcome::outOfOrder:
            return "outOfOrder";
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
    // At least the largest scan age the filter can actually see, plus the
    // maximum service wait, plus the largest adjacent-frame gap trim() may
    // cross. On the demo's fixed grid the latter two are one control period
    // each, hence visible age + 2 periods. Note that visible age equals true
    // latency only when the stamp is trustworthy.
    double historyHorizon{0.55};
    double mahalanobisGate{9.0};
    int consecutiveRejectLimit{3};
    // Open-loop tolerance measured on scan stamps, i.e. how stale the newest
    // fused information may be before the process noise is inflated.
    double lostTimeout{0.6};
    double coastingProcessNoiseScale{4.0};
    std::size_t maximumFrames{4096};
    // Legacy behaviour: fuse the scan at the closest stored frame instead of
    // splitting the interval at its stamp. Window rejection still happens
    // first, so an out-of-window packet is dropped rather than snapped.
    bool snapToNearestFrame{false};

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
        if (maximumFrames < 2) {
            errors += "maximumFrames must be >= 2; ";
        }
        return errors;
    }
};

// Delayed-measurement EKF shell.
//
// The filter keeps a short history of frames, each holding the posterior that
// followed an input sample. A late scan is fused at its own stamp by rewinding
// to that point in the history and re-propagating the cached inputs forward.
//
// REQUIRED ASSUMPTION: scan stamps arrive in non-decreasing order.
//
// The assumption is what keeps this simple. When a scan at t_s arrives, every
// previously fused scan has a stamp at or before t_s, so re-propagating inputs
// alone cannot discard a correction: there is none after t_s to discard. If
// scans could arrive out of order, the forward pass would have to re-apply the
// later measurements too, which needs a full event log.
//
// Rather than assume silently, the shell checks. A stamp at or before the last
// fused one is rejected and reported as `outOfOrder` or `duplicate`. A pipeline
// that violates the assumption therefore shows up as a visible counter instead
// of a quietly corrupted state.
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
        std::size_t repropagatedFrames{};
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
        frames_.clear();
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
        Frame anchor;
        anchor.time = time;
        anchor.inputs = inputs;
        anchor.state = state;
        anchor.covariance = covariance;
        model_.normalize(anchor.state);
        frames_.clear();
        frames_.push_back(anchor);
        lastAcceptedStamp_ = time;
        lastMeasurementStamp_ = -std::numeric_limits<double>::infinity();
        everAccepted_ = false;
        consecutiveRejects_ = 0;
        initialized_ = true;
    }

    // Advances the filter to `time`. Samples that are not newer than the newest
    // frame only refresh the governing input; they never rewind time.
    void predict(const Inputs& inputs, double time) {
        requireInitialized();
        if (!std::isfinite(time)) {
            throw std::invalid_argument("estimator input time must be finite");
        }
        Frame& newest = frames_.back();
        if (time <= newest.time + 1.0e-15) {
            newest.inputs = inputs;
            return;
        }

        Frame next;
        next.time = time;
        next.inputs = inputs;
        next.state = newest.state;
        next.covariance = newest.covariance;
        model_.propagate(
            next.state,
            next.covariance,
            inputs,
            time - newest.time,
            noiseScale(newest.time));
        frames_.push_back(next);
        trim();
    }

    [[nodiscard]] MeasurementReport update(double stamp, double value) {
        MeasurementReport report;
        report.stamp = stamp;
        if (!configured_ || !initialized_ || frames_.empty()) {
            report.outcome = MeasurementOutcome::notInitialized;
            return report;
        }
        if (!std::isfinite(stamp) || !std::isfinite(value)) {
            report.outcome = MeasurementOutcome::nonFinite;
            return report;
        }
        // Enforce the monotonic-stamp assumption this shell relies on.
        if (stamp == lastMeasurementStamp_) {
            report.outcome = MeasurementOutcome::duplicate;
            return report;
        }
        if (stamp < lastMeasurementStamp_) {
            report.outcome = MeasurementOutcome::outOfOrder;
            registerRejection();
            return report;
        }
        if (stamp < frames_.front().time - 1.0e-9) {
            report.outcome = MeasurementOutcome::staleBeyondWindow;
            registerRejection();
            return report;
        }
        if (stamp > frames_.back().time + 1.0e-9) {
            // No input covers this stamp yet. Call predict() first; extending
            // the newest input past its sample would fabricate information.
            report.outcome = MeasurementOutcome::aheadOfInputs;
            return report;
        }

        lastMeasurementStamp_ = stamp;

        const std::size_t index = locateFrame(stamp);
        report.alignedStamp = frames_[index].time;
        applyMeasurement(frames_[index], value, report);
        if (report.outcome == MeasurementOutcome::accepted) {
            report.repropagatedFrames = repropagateFrom(index);
        }
        return report;
    }

    [[nodiscard]] const State& state() const { return frames_.back().state; }
    [[nodiscard]] const Covariance& covariance() const {
        return frames_.back().covariance;
    }
    [[nodiscard]] double time() const { return frames_.back().time; }
    [[nodiscard]] const Inputs& latestInputs() const {
        return frames_.back().inputs;
    }
    [[nodiscard]] double lastAcceptedStamp() const { return lastAcceptedStamp_; }
    [[nodiscard]] bool everAccepted() const { return everAccepted_; }
    [[nodiscard]] int consecutiveRejects() const { return consecutiveRejects_; }
    // Age of the newest fused scan. This is how stale the information behind
    // the current estimate is, which is not the same as link health.
    [[nodiscard]] double informationAge() const {
        return frames_.back().time - lastAcceptedStamp_;
    }
    [[nodiscard]] bool coasting() const {
        return noiseScale(frames_.back().time) > 1.0;
    }
    [[nodiscard]] std::size_t frameCount() const { return frames_.size(); }
    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] const Model& model() const { return model_; }
    [[nodiscard]] Model& model() { return model_; }
    [[nodiscard]] const DelayedEkfLimits& limits() const { return limits_; }

private:
    struct Frame {
        double time{};
        Inputs inputs{};
        State state{};
        Covariance covariance{};
    };

    void requireConfigured() const {
        if (!configured_) {
            throw std::logic_error("DelayedEkf is not configured");
        }
    }

    void requireInitialized() const {
        requireConfigured();
        if (!initialized_ || frames_.empty()) {
            throw std::logic_error("DelayedEkf is not initialized");
        }
    }

    [[nodiscard]] double noiseScale(double time) const {
        const bool stale = time - lastAcceptedStamp_ > limits_.lostTimeout;
        const bool rejecting =
            consecutiveRejects_ >= limits_.consecutiveRejectLimit;
        return (stale || rejecting) ? limits_.coastingProcessNoiseScale : 1.0;
    }

    void registerRejection() { ++consecutiveRejects_; }

    // Returns the index of the frame the update is applied to. By default the
    // interval containing the stamp is split so the update lands exactly on it;
    // the legacy mode snaps to the closest existing frame instead.
    [[nodiscard]] std::size_t locateFrame(double stamp) {
        if (limits_.snapToNearestFrame) {
            std::size_t best = 0;
            double bestGap = std::abs(frames_.front().time - stamp);
            for (std::size_t i = 1; i < frames_.size(); ++i) {
                const double gap = std::abs(frames_[i].time - stamp);
                if (gap < bestGap) {
                    bestGap = gap;
                    best = i;
                }
            }
            return best;
        }

        std::size_t after = frames_.size();
        for (std::size_t i = 0; i < frames_.size(); ++i) {
            if (frames_[i].time > stamp) {
                after = i;
                break;
            }
        }
        if (after == 0) {
            return 0;
        }
        if (after == frames_.size()) {
            // Stamp is at or after the newest frame; the window check already
            // bounded how far past it can be.
            return frames_.size() - 1;
        }
        if (std::abs(frames_[after - 1].time - stamp) <= 1.0e-12) {
            return after - 1;
        }

        // Split the interval. Both halves keep the input that governed the
        // original interval, so the split does not change input semantics.
        Frame inserted;
        inserted.time = stamp;
        inserted.inputs = frames_[after].inputs;
        inserted.state = frames_[after - 1].state;
        inserted.covariance = frames_[after - 1].covariance;
        model_.propagate(
            inserted.state,
            inserted.covariance,
            inserted.inputs,
            stamp - frames_[after - 1].time,
            noiseScale(frames_[after - 1].time));
        frames_.insert(
            frames_.begin() + static_cast<std::ptrdiff_t>(after), inserted);
        return after;
    }

    std::size_t repropagateFrom(std::size_t index) {
        std::size_t touched = 0;
        for (std::size_t i = index; i + 1 < frames_.size(); ++i) {
            const double dt = frames_[i + 1].time - frames_[i].time;
            frames_[i + 1].state = frames_[i].state;
            frames_[i + 1].covariance = frames_[i].covariance;
            if (dt > 0.0) {
                model_.propagate(
                    frames_[i + 1].state,
                    frames_[i + 1].covariance,
                    frames_[i + 1].inputs,
                    dt,
                    // The span from the corrected frame to now carries no
                    // further measurement, so it is still open loop. The
                    // update has already moved lastAcceptedStamp_ to the scan
                    // instant, so this measures age since the correction,
                    // which is exactly the right clock for the replay.
                    noiseScale(frames_[i].time));
            }
            ++touched;
        }
        return touched;
    }

    void applyMeasurement(
        Frame& frame,
        double measurement,
        MeasurementReport& report) {
        const State jacobian = model_.measurementJacobian(frame.state);
        const double variance = model_.measurementVariance();
        const double predicted = model_.predictMeasurement(frame.state);
        const double innovation = model_.residual(measurement, predicted);

        State covarianceTimesH{};
        for (std::size_t row = 0; row < kStateSize; ++row) {
            double accumulated = 0.0;
            for (std::size_t column = 0; column < kStateSize; ++column) {
                accumulated += frame.covariance[row][column] * jacobian[column];
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
            registerRejection();
            report.outcome = MeasurementOutcome::gated;
            return;
        }

        State gain{};
        for (std::size_t i = 0; i < kStateSize; ++i) {
            gain[i] = covarianceTimesH[i] / innovationCovariance;
            frame.state[i] += gain[i] * innovation;
        }
        model_.normalize(frame.state);

        // Joseph form keeps the posterior symmetric positive semi-definite even
        // when the gain and the prior are slightly inconsistent numerically.
        Matrix<kStateSize, kStateSize> factor = identityMatrix<kStateSize>();
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                factor[row][column] -= gain[row] * jacobian[column];
            }
        }
        const auto left = matrixProduct(factor, frame.covariance);
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
        frame.covariance = updated;

        lastAcceptedStamp_ = frame.time;
        everAccepted_ = true;
        consecutiveRejects_ = 0;
        report.gain = gain;
        report.outcome = MeasurementOutcome::accepted;
    }

    void trim() {
        while (frames_.size() > 2 &&
               frames_.back().time - frames_.front().time >
                   limits_.historyHorizon + 1.0e-12) {
            frames_.pop_front();
        }
        while (frames_.size() > limits_.maximumFrames) {
            frames_.pop_front();
        }
    }

    Model model_{};
    DelayedEkfLimits limits_{};
    bool configured_{};
    bool initialized_{};
    std::deque<Frame> frames_;
    double lastAcceptedStamp_{};
    double lastMeasurementStamp_{-std::numeric_limits<double>::infinity()};
    bool everAccepted_{};
    int consecutiveRejects_{};
};

}  // namespace truck_model
