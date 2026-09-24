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
    duplicate,         // same stamp as the last processed scan
    outOfOrder,        // stamp older than the last processed scan
    staleBeyondWindow, // stamp older than the history or historyHorizon
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
// one that passed the order and window checks, a gated scan included, is
// rejected and reported as `outOfOrder` or `duplicate`. A pipeline that
// violates the assumption therefore shows up as a visible counter instead of a
// quietly corrupted state.
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
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = 0; column < kStateSize; ++column) {
                const double value = covariance[row][column];
                const double mirror = covariance[column][row];
                if (!std::isfinite(value)) {
                    throw std::invalid_argument(
                        "estimator reset covariance must be finite");
                }
                if (std::abs(value - mirror) >
                    kSymmetryTolerance *
                        std::max({1.0, std::abs(value), std::abs(mirror)})) {
                    throw std::invalid_argument(
                        "estimator reset covariance must be symmetric");
                }
            }
            if (covariance[row][row] < 0.0) {
                throw std::invalid_argument(
                    "estimator reset covariance diagonal must be >= 0");
            }
        }
        Frame anchor;
        anchor.time = time;
        anchor.inputs = inputs;
        anchor.state = state;
        anchor.covariance = covariance;
        symmetrize(anchor.covariance);
        model_.normalize(anchor.state);
        frames_.clear();
        frames_.push_back(anchor);
        lastAcceptedStamp_ = time;
        lastMeasurementStamp_ = -std::numeric_limits<double>::infinity();
        everAccepted_ = false;
        consecutiveRejects_ = 0;
        initialized_ = true;
    }

    // Advances the filter to `time`. Input time may not run backwards. A sample
    // at the newest frame's time changes nothing: that frame was already
    // propagated with the input it stores, and rewriting the input would make
    // a later replay diverge from the forward pass.
    void predict(const Inputs& inputs, double time) {
        requireInitialized();
        if (!std::isfinite(time)) {
            throw std::invalid_argument("estimator input time must be finite");
        }
        const Frame& newest = frames_.back();
        if (time < newest.time - kSameTimeTolerance) {
            throw std::invalid_argument("estimator input time went backwards");
        }
        if (time <= newest.time + kSameTimeTolerance) {
            return;
        }

        Frame next;
        next.time = time;
        next.inputs = inputs;
        next.state = newest.state;
        next.covariance = newest.covariance;
        propagateFrom(next, newest.time);
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
        // The age bound matters after a time jump: trim() always keeps two
        // frames, so the oldest frame alone would admit arbitrarily old stamps.
        if (stamp < frames_.front().time - kWindowTolerance ||
            stamp < frames_.back().time - limits_.historyHorizon -
                        kWindowTolerance) {
            report.outcome = MeasurementOutcome::staleBeyondWindow;
            registerRejection();
            return report;
        }
        if (stamp > frames_.back().time + kWindowTolerance) {
            // No input covers this stamp yet. Call predict() first; extending
            // the newest input past its sample would fabricate information.
            report.outcome = MeasurementOutcome::aheadOfInputs;
            return report;
        }

        lastMeasurementStamp_ = stamp;

        const Target target = locateTarget(stamp);
        if (!target.split) {
            Frame& frame = frames_[target.index];
            report.alignedStamp = frame.time;
            applyMeasurement(frame, value, report);
            if (report.outcome == MeasurementOutcome::accepted) {
                report.repropagatedFrames = repropagateFrom(target.index);
            }
            return report;
        }

        // Gate on a detached split frame, so that a rejected scan leaves the
        // history exactly as it was.
        Frame inserted = splitFrame(target.index, stamp);
        report.alignedStamp = inserted.time;
        applyMeasurement(inserted, value, report);
        if (report.outcome != MeasurementOutcome::accepted) {
            return report;
        }
        frames_.insert(
            frames_.begin() + static_cast<std::ptrdiff_t>(target.index),
            inserted);
        report.repropagatedFrames = repropagateFrom(target.index);
        enforceFrameLimit();
        return report;
    }

    [[nodiscard]] const State& state() const { return frames_.back().state; }
    [[nodiscard]] const Covariance& covariance() const {
        return frames_.back().covariance;
    }
    [[nodiscard]] double time() const { return frames_.back().time; }
    [[nodiscard]] double lastAcceptedStamp() const { return lastAcceptedStamp_; }
    [[nodiscard]] bool everAccepted() const { return everAccepted_; }
    [[nodiscard]] int consecutiveRejects() const { return consecutiveRejects_; }
    // Age of the newest fused scan. This is how stale the information behind
    // the current estimate is, which is not the same as link health.
    [[nodiscard]] double informationAge() const {
        return frames_.back().time - lastAcceptedStamp_;
    }
    [[nodiscard]] bool coasting() const {
        return openLoop(frames_.back().time);
    }
    [[nodiscard]] std::size_t frameCount() const { return frames_.size(); }
    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] const Model& model() const { return model_; }
    [[nodiscard]] Model& model() { return model_; }
    [[nodiscard]] const DelayedEkfLimits& limits() const { return limits_; }

private:
    // An input sample within this of the newest frame is the same instant.
    static constexpr double kSameTimeTolerance = 1.0e-15;
    // Window slack at both ends. A stamp inside the slack but outside the
    // stored span is fused at the end frame, at most this far from the stamp.
    static constexpr double kWindowTolerance = 1.0e-9;
    // A stamp this close to a stored frame is fused there instead of splitting.
    static constexpr double kFrameMatchTolerance = 1.0e-12;
    static constexpr double kSpanTolerance = 1.0e-12;
    static constexpr double kSymmetryTolerance = 1.0e-9;

    struct Frame {
        double time{};
        Inputs inputs{};
        State state{};
        Covariance covariance{};
    };

    struct Target {
        // Frame to update, or the insertion position when the interval before
        // it has to be split at the stamp.
        std::size_t index{};
        bool split{};
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

    // Evaluated at the left end of every propagated interval. The reject count
    // takes effect when a rejection is processed, not at the rejected stamp.
    [[nodiscard]] bool openLoop(double time) const {
        return time - lastAcceptedStamp_ > limits_.lostTimeout ||
               consecutiveRejects_ >= limits_.consecutiveRejectLimit;
    }

    [[nodiscard]] double noiseScale(double time) const {
        return openLoop(time) ? limits_.coastingProcessNoiseScale : 1.0;
    }

    void registerRejection() { ++consecutiveRejects_; }

    static void symmetrize(Covariance& covariance) {
        for (std::size_t row = 0; row < kStateSize; ++row) {
            for (std::size_t column = row + 1; column < kStateSize; ++column) {
                const double mean =
                    0.5 * (covariance[row][column] + covariance[column][row]);
                covariance[row][column] = mean;
                covariance[column][row] = mean;
            }
        }
    }

    // `frame` holds the posterior at `fromTime` on entry and the prior at
    // frame.time on exit, propagated with the input the frame stores.
    void propagateFrom(Frame& frame, double fromTime) const {
        model_.propagate(
            frame.state,
            frame.covariance,
            frame.inputs,
            frame.time - fromTime,
            noiseScale(fromTime));
        symmetrize(frame.covariance);
    }

    // By default the interval containing the stamp is split so the update
    // lands exactly on it; the legacy mode snaps to the closest frame instead.
    [[nodiscard]] Target locateTarget(double stamp) const {
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
            return {best, false};
        }

        std::size_t after = frames_.size();
        for (std::size_t i = 0; i < frames_.size(); ++i) {
            if (frames_[i].time > stamp) {
                after = i;
                break;
            }
        }
        if (after == 0) {
            return {0, false};
        }
        if (after == frames_.size()) {
            // Stamp is at or after the newest frame; the window check already
            // bounded how far past it can be.
            return {frames_.size() - 1, false};
        }
        if (std::abs(frames_[after - 1].time - stamp) <= kFrameMatchTolerance) {
            return {after - 1, false};
        }
        if (std::abs(frames_[after].time - stamp) <= kFrameMatchTolerance) {
            return {after, false};
        }
        return {after, true};
    }

    // Both halves keep the input that governed the original interval, so the
    // split does not change input semantics.
    [[nodiscard]] Frame splitFrame(std::size_t after, double stamp) const {
        Frame inserted;
        inserted.time = stamp;
        inserted.inputs = frames_[after].inputs;
        inserted.state = frames_[after - 1].state;
        inserted.covariance = frames_[after - 1].covariance;
        propagateFrom(inserted, frames_[after - 1].time);
        return inserted;
    }

    std::size_t repropagateFrom(std::size_t index) {
        std::size_t touched = 0;
        for (std::size_t i = index; i + 1 < frames_.size(); ++i) {
            frames_[i + 1].state = frames_[i].state;
            frames_[i + 1].covariance = frames_[i].covariance;
            if (frames_[i + 1].time - frames_[i].time > 0.0) {
                // The span from the corrected frame to now carries no further
                // measurement, so it is still open loop. The update has already
                // moved lastAcceptedStamp_ to the scan instant, so the noise
                // scale measures age since the correction, which is exactly
                // the right clock for the replay.
                propagateFrom(frames_[i + 1], frames_[i].time);
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
        symmetrize(updated);
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
                   limits_.historyHorizon + kSpanTolerance) {
            frames_.pop_front();
        }
        enforceFrameLimit();
    }

    void enforceFrameLimit() {
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
