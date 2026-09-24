// Case B probe: lidar packets carry no scan stamp, only an exact arrival
// instant and the [0.1, 0.4] s latency bound. Runs the case-B timing
// strategies on both delayed shells against a K27 plant and prints the
// accuracy numbers. Not part of the build.
//
//   g++ -std=c++17 -O2 -I include tools/case_b_probe.cpp src/*.cpp
//       -o build-caseb/case_b_probe.exe
//   build-caseb/case_b_probe.exe [results.txt] [seeds]
#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/delayed_ekf.hpp"
#include "truck_model/linear_discretization.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using truck_model::ArticulationEstimatorConfig;
using truck_model::ArticulationInputs;
using truck_model::DynamicArticulationModel;
using truck_model::KinematicArticulationModel;
using truck_model::MeasurementOutcome;
using truck_model::Parameters;

constexpr double kDeg = 57.29577951308232;
constexpr double kPi = 3.14159265358979323846;

constexpr double kPlantStep = 1.0e-3;
constexpr std::size_t kPlantStepsPerTick = 50;
constexpr double kControlPeriod = 0.05;
constexpr double kDuration = 80.0;
constexpr double kScoreFrom = 5.0;

constexpr double kScanPeriod = 0.1;
constexpr double kScanPhase = 0.013;
constexpr double kScanClockError = 200.0e-6;
constexpr double kMinDelay = 0.1;
constexpr double kMaxDelay = 0.4;
constexpr double kNominalDelay = 0.25;
constexpr double kTimingStd = 0.08660254037844387;  // 0.3 / sqrt(12)
constexpr double kPriorError = 0.02;
constexpr double kRateFloor = 2.5 / kDeg;
// Covers the oldest stamp any strategy can hand over: 0.4 s latency, one
// control period of service wait, one of trim slack, and the profile offsets.
constexpr double kHistoryHorizon = 0.7;

constexpr double kYawRateNoise = 0.2 / kDeg;
constexpr double kSpeedNoise = 0.05;
constexpr double kSteeringNoise = 0.02 / kDeg;

// Scan-clock envelope: window in scans, an a-priori +-1000 ppm period bound,
// and the slack before an empty interval at the selected slope raises the
// residual alarm (a slope error of a few us per scan widens the detrended
// spread). The strict test searches every admissible slope instead and only
// allows for rounding.
constexpr int kClockWindow = 300;
constexpr double kClockSlopeBound = kScanPeriod * 1.0e-3;
constexpr double kClockSpanFloor = 1.0e-3;
constexpr double kClockConsistencySlack = 5.0e-3;
constexpr double kClockFeasibilityTolerance = 1.0e-9;

// Profile likelihood: offsets around the stamp source (for B4ML none of them
// is the true +20 ms), and the per-scan forgetting of the log-likelihood.
constexpr int kCandidates = 13;
constexpr double kCandidateStep = 0.0125;
constexpr double kForgetting = 0.995;
constexpr double kSlowForgetting = 0.999;

constexpr std::array<double, 2> kNoiseLevels{0.5, 4.0};

Parameters vehicle() {
    Parameters p;
    p.m1 = 8000.0;
    p.iz1 = 25000.0;
    p.a1 = 1.5;
    p.b1 = 2.5;
    p.c1f = 220000.0;
    p.c1r = 300000.0;
    p.m2 = 18000.0;
    p.iz2 = 140000.0;
    p.a2 = 4.0;
    p.b2 = 3.0;
    p.c2r = 500000.0;
    p.d1 = p.b1;
    p.vx = 15.0;
    return p;
}

double smoothRamp(double t, double begin, double end) {
    const double x = std::clamp((t - begin) / (end - begin), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

struct Drive {
    double steering{};
    double speed{};
};

// A common latency offset is observable only while the truck yaw acceleration
// varies: the S-curves and the lane change excite it, the straights and the
// steady turns do not.
Drive drive(double t) {
    Drive d;
    d.speed = 13.0 + 2.0 * smoothRamp(t, 16.0, 20.0) -
              smoothRamp(t, 34.0, 38.0) - 2.0 * smoothRamp(t, 42.0, 46.0) +
              smoothRamp(t, 64.0, 67.0);
    double degrees = 0.0;
    if (t >= 6.0 && t < 16.0) {
        degrees = 4.0 * std::sin(2.0 * kPi * 0.2 * (t - 6.0));
    } else if (t >= 24.0 && t < 28.0) {
        degrees = 4.5 * std::sin(2.0 * kPi * 0.25 * (t - 24.0));
    } else if (t >= 30.0 && t < 42.0) {
        degrees = 1.1 * (smoothRamp(t, 30.0, 32.0) - smoothRamp(t, 40.0, 42.0));
    } else if (t >= 48.0 && t < 58.0) {
        degrees = 9.0 * std::sin(2.0 * kPi * 0.4 * (t - 48.0));
    } else if (t >= 60.0 && t < 70.0) {
        degrees =
            -1.2 * (smoothRamp(t, 60.0, 62.0) - smoothRamp(t, 68.0, 70.0));
    }
    d.steering = degrees / kDeg;
    return d;
}

struct Truth {
    std::vector<double> articulation;
    std::vector<double> articulationRate;
    std::vector<double> truckYawRate;
    std::vector<double> speed;
    std::vector<double> steering;
    double rateRms{};

    [[nodiscard]] double articulationAt(double t) const {
        const double x = std::clamp(
            t / kPlantStep, 0.0, static_cast<double>(articulation.size() - 1));
        const auto i = std::min(static_cast<std::size_t>(x),
                                articulation.size() - 2);
        const double w = x - static_cast<double>(i);
        return (1.0 - w) * articulation[i] + w * articulation[i + 1];
    }
};

// K27 plant, z = [vy1, r1, r2, theta12], exact ZOH per 1 ms step with the
// matrices rebuilt whenever the speed changes.
Truth simulatePlant(Parameters p) {
    const auto steps =
        static_cast<std::size_t>(std::llround(kDuration / kPlantStep));
    Truth truth;
    truth.articulation.resize(steps + 1);
    truth.articulationRate.resize(steps + 1);
    truth.truckYawRate.resize(steps + 1);
    truth.speed.resize(steps + 1);
    truth.steering.resize(steps + 1);
    truck_model::Vector<4> x{};
    truck_model::DiscreteDynamicModel zoh;
    double builtFor = -1.0;
    for (std::size_t i = 0;; ++i) {
        const double t = static_cast<double>(i) * kPlantStep;
        const Drive now = drive(t);
        truth.articulation[i] = x[3];
        truth.articulationRate[i] = x[1] - x[2];
        truth.truckYawRate[i] = x[1];
        truth.speed[i] = now.speed;
        truth.steering[i] = now.steering;
        if (i == steps) {
            break;
        }
        const Drive mid = drive(t + 0.5 * kPlantStep);
        if (mid.speed != builtFor) {
            p.vx = mid.speed;
            zoh = truck_model::discretizeZeroOrderHold(
                truck_model::buildDynamicModel(p), kPlantStep);
            builtFor = mid.speed;
        }
        truck_model::Vector<4> next{};
        for (std::size_t row = 0; row < 4; ++row) {
            next[row] = zoh.b[row] * mid.steering;
            for (std::size_t column = 0; column < 4; ++column) {
                next[row] += zoh.a[row][column] * x[column];
            }
        }
        x = next;
    }
    double sum = 0.0;
    std::size_t count = 0;
    for (std::size_t i = 0; i <= steps; i += kPlantStepsPerTick) {
        if (static_cast<double>(i) * kPlantStep >= kScoreFrom) {
            sum += truth.articulationRate[i] * truth.articulationRate[i];
            ++count;
        }
    }
    truth.rateRms = std::sqrt(sum / static_cast<double>(count));
    return truth;
}

enum class Latency { iid, fifo };

struct Packet {
    int index{};
    double scan{};
    double arrival{};
    double value{};
};

std::mt19937_64 stream(std::uint64_t seed, std::uint64_t purpose) {
    std::seed_seq sequence{seed, purpose, std::uint64_t{0x5eed}};
    return std::mt19937_64(sequence);
}

// D1: i.i.d. uniform, so arrivals can overtake. D2: single-server FIFO
// pipeline (Lindley recursion) behind an 80 ms transport delay; 30 % of the
// frames are heavy, and service is shortened under backlog so every latency
// stays inside [kMinDelay, kMaxDelay]. D2 preserves order and is correlated.
std::vector<double> latencies(Latency latency,
                              const std::vector<double>& scans,
                              std::mt19937_64& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<double> result(scans.size());
    if (latency == Latency::iid) {
        for (double& value : result) {
            value = kMinDelay + (kMaxDelay - kMinDelay) * unit(rng);
        }
        return result;
    }
    constexpr double transport = 0.08;
    double inSystem = 0.0;
    for (std::size_t k = 0; k < scans.size(); ++k) {
        const double gap = k == 0 ? kScanPeriod : scans[k] - scans[k - 1];
        const double backlog = std::max(0.0, inSystem - gap);
        double service = unit(rng) < 0.3 ? 0.10 + 0.15 * unit(rng)
                                         : 0.02 + 0.07 * unit(rng);
        service = std::min(service, kMaxDelay - transport - backlog);
        inSystem = backlog + service;
        result[k] = transport + inSystem;
    }
    return result;
}

// Sorted by arrival. The noise stream is indexed by scan, so both noise
// levels see the same draws scaled.
std::vector<Packet> makePackets(const Truth& truth,
                                Latency latency,
                                double noiseStd,
                                std::uint64_t seed) {
    std::vector<double> scans;
    for (int k = 0;; ++k) {
        const double scan =
            kScanPhase + k * kScanPeriod * (1.0 + kScanClockError);
        if (scan > kDuration) {
            break;
        }
        scans.push_back(scan);
    }
    auto delayRng = stream(seed, 1 + static_cast<std::uint64_t>(latency));
    auto noiseRng = stream(seed, 3);
    std::normal_distribution<double> gauss(0.0, 1.0);
    const auto delays = latencies(latency, scans, delayRng);
    std::vector<Packet> packets;
    for (std::size_t k = 0; k < scans.size(); ++k) {
        Packet packet;
        packet.index = static_cast<int>(k);
        packet.scan = scans[k];
        packet.arrival = scans[k] + delays[k];
        packet.value = truth.articulationAt(scans[k]) + noiseStd * gauss(noiseRng);
        if (packet.arrival <= kDuration) {
            packets.push_back(packet);
        }
    }
    std::stable_sort(packets.begin(), packets.end(),
                     [](const Packet& a, const Packet& b) {
                         return a.arrival < b.arrival;
                     });
    return packets;
}

std::vector<ArticulationInputs> makeInputs(const Truth& truth,
                                           std::uint64_t seed) {
    auto rng = stream(seed, 4);
    std::normal_distribution<double> gauss(0.0, 1.0);
    const std::size_t ticks =
        (truth.articulation.size() - 1) / kPlantStepsPerTick + 1;
    std::vector<ArticulationInputs> inputs(ticks);
    for (std::size_t n = 0; n < ticks; ++n) {
        const std::size_t i = n * kPlantStepsPerTick;
        auto& sample = inputs[n];
        sample.time = static_cast<double>(n) * kControlPeriod;
        sample.truckYawRate = truth.truckYawRate[i] + kYawRateNoise * gauss(rng);
        sample.speed = truth.speed[i] + kSpeedNoise * gauss(rng);
        sample.steering = truth.steering[i] + kSteeringNoise * gauss(rng);
    }
    return inputs;
}

double articulation(const truck_model::Vector<2>& state) { return state[0]; }
double articulation(const truck_model::Vector<3>& state) { return state[2]; }

// phiDot = r1 - r2 from a state and the input that governs its interval.
double articulationRate(const KinematicArticulationModel& model,
                        const truck_model::Vector<2>& state,
                        const ArticulationInputs& inputs) {
    return inputs.truckYawRate -
           truck_model::kinematicTrailerYawRate(
               model.parameters(), inputs.speed, inputs.truckYawRate,
               state[0]) -
           state[1];
}

double articulationRate(const DynamicArticulationModel&,
                        const truck_model::Vector<3>& state,
                        const ArticulationInputs& inputs) {
    return inputs.truckYawRate - state[1];
}

void initialPrior(const KinematicArticulationModel&,
                  const ArticulationEstimatorConfig& config,
                  truck_model::Vector<2>& state,
                  truck_model::Matrix<2, 2>& covariance) {
    state = {0.0, 0.0};
    covariance = {};
    covariance[0][0] = config.initialArticulationVariance;
    covariance[1][1] = config.initialTrailerYawBiasVariance;
}

void initialPrior(const DynamicArticulationModel&,
                  const ArticulationEstimatorConfig& config,
                  truck_model::Vector<3>& state,
                  truck_model::Matrix<3, 3>& covariance) {
    state = {0.0, 0.0, 0.0};
    covariance = {};
    covariance[0][0] = config.initialTruckLateralVelocityVariance;
    covariance[1][1] = config.initialTrailerYawRateVariance;
    covariance[2][2] = config.initialArticulationVariance;
}

// PROTOTYPE WORKAROUND, not a pattern for production code. The shell's Model
// interface has a parameterless measurementVariance(), so a state-dependent
// R_eff = R + sigma_t^2 (rate^2 + floor^2) cannot be passed in. It relies on
// DelayedEkf::applyMeasurement calling measurementJacobian() on the anchor
// frame's pre-update prior before it reads measurementVariance(); the
// Jacobian call freezes R_eff. A production shell needs a per-measurement
// variance argument on update() instead.
template <typename Base>
class TimedModel {
public:
    static constexpr std::size_t kStateSize = Base::kStateSize;
    using Inputs = typename Base::Inputs;
    using State = truck_model::Vector<kStateSize>;
    using Covariance = truck_model::Matrix<kStateSize, kStateSize>;

    TimedModel() = default;
    TimedModel(Base base,
               const std::vector<Inputs>* inputLog,
               double sensorVariance)
        : base_(std::move(base)),
          inputLog_(inputLog),
          sensorVariance_(sensorVariance),
          variance_(sensorVariance) {}

    // A negative fixedRateSq selects the anchor-frame rate estimate.
    void arm(double stamp, double timingVariance, double fixedRateSq) {
        stamp_ = stamp;
        timingVariance_ = timingVariance;
        fixedRateSq_ = fixedRateSq;
    }

    void propagate(State& state,
                   Covariance& covariance,
                   const Inputs& inputs,
                   double dt,
                   double processNoiseScale) const {
        base_.propagate(state, covariance, inputs, dt, processNoiseScale);
    }
    [[nodiscard]] double predictMeasurement(const State& state) const {
        return base_.predictMeasurement(state);
    }
    [[nodiscard]] State measurementJacobian(const State& state) const {
        const double rate = articulationRate(base_, state, governingInputs());
        const double rateSq = fixedRateSq_ >= 0.0
                                  ? fixedRateSq_
                                  : rate * rate + kRateFloor * kRateFloor;
        variance_ = sensorVariance_ + timingVariance_ * rateSq;
        return base_.measurementJacobian(state);
    }
    [[nodiscard]] double measurementVariance() const { return variance_; }
    [[nodiscard]] double residual(double measurement, double predicted) const {
        return base_.residual(measurement, predicted);
    }
    void normalize(State& state) const { base_.normalize(state); }
    [[nodiscard]] const Base& base() const { return base_; }

private:
    // Right-endpoint hold, as the shell stores it: the sample at t_n governs
    // (t_{n-1}, t_n]. The shell never anchors past its newest input.
    [[nodiscard]] const Inputs& governingInputs() const {
        const double position = std::ceil(stamp_ / kControlPeriod - 1.0e-9);
        const auto last = static_cast<double>(inputLog_->size() - 1);
        return (*inputLog_)[static_cast<std::size_t>(
            std::clamp(position, 0.0, last))];
    }

    Base base_{};
    const std::vector<Inputs>* inputLog_{};
    double sensorVariance_{};
    double stamp_{};
    double timingVariance_{};
    double fixedRateSq_{-1.0};
    mutable double variance_{};
};

// Scan-grid reconstruction. With s_k = theta + k (T + dT) and a_k = s_k +
// tau_k, the offsets c_k = a_k - k T sit on or above the line theta + tau_min
// + k dT. The line is the lower-envelope LP solution over the window (the
// lower-hull edge under the mean index, slope clamped to the period bound),
// and the tau_min prior turns it into absolute scan times. At that slope the
// interval [max(c - dT k) - tau_max, min(c - dT k) - tau_min] brackets theta.
// B4 hands over its upper end: under a uniform prior on the interval the
// one-sided error has mean width/2 and second moment width^2/3, and B4 puts
// the whole second moment into R. At the selected slope the interval can be
// empty while other admissible slopes still fit, and B4 then hands over the
// envelope value anyway. B4mid instead takes the range of the scan time over
// every admissible (phase, period) pair and hands over its midpoint with the
// zero-mean variance width^2/12.
class ScanClock {
public:
    struct Fit {
        double stamp{};
        double variance{};
    };

    ScanClock(double minDelay, double maxDelay)
        : minDelay_(minDelay), maxDelay_(maxDelay) {}

    void add(int index, double arrival) {
        const Point point{index, arrival - index * kScanPeriod};
        const auto position = std::lower_bound(
            points_.begin(), points_.end(), index,
            [](const Point& p, int k) { return p.index < k; });
        points_.insert(position, point);
        refit();
    }

    [[nodiscard]] std::pair<double, double> reconstruct(int index) const {
        const double line = intercept_ + slope_ * index;
        return {index * kScanPeriod + line - minDelay_, variance_};
    }
    // Range of scan `index`'s time over the joint feasible set. For a fixed
    // slope b the phase at `index` spans [lo(b), hi(b)] with lo convex and
    // hi concave; the admissible slopes {|b| <= bound, spread(b) <= W} form
    // an interval, so the range is [min lo, max hi] over that interval. If
    // no slope is admissible the window contradicts the prior; B4mid then
    // falls back to the midpoint at the selected slope.
    [[nodiscard]] Fit reconstructJoint(int index) const {
        const double limit = maxDelay_ - minDelay_ + kClockFeasibilityTolerance;
        const auto spread = [&](double slope) { return spreadAt(slope); };
        const auto best = minimizeConvex(spread, -kClockSlopeBound,
                                         kClockSlopeBound);
        if (best.second > limit) {
            const double line = intercept_ + slope_ * index;
            const double span = std::max(width_, kClockSpanFloor);
            return {index * kScanPeriod + line - minDelay_ - 0.5 * width_,
                    span * span / 12.0};
        }
        const auto boundary = [&](double outside, double inside) {
            if (spreadAt(outside) <= limit) {
                return outside;
            }
            for (int iteration = 0; iteration < 80; ++iteration) {
                const double middle = 0.5 * (outside + inside);
                (spreadAt(middle) <= limit ? inside : outside) = middle;
            }
            return inside;
        };
        const double lowSlope = boundary(-kClockSlopeBound, best.first);
        const double highSlope = boundary(kClockSlopeBound, best.first);
        const auto lower = [&](double slope) {
            return bandAt(slope, index).first - maxDelay_;
        };
        const auto negativeUpper = [&](double slope) {
            return minDelay_ - bandAt(slope, index).second;
        };
        const double earliest = minimizeConvex(lower, lowSlope, highSlope).second;
        const double latest =
            -minimizeConvex(negativeUpper, lowSlope, highSlope).second;
        const double width = std::max(latest - earliest, 0.0);
        const double span = std::max(width, kClockSpanFloor);
        return {index * kScanPeriod + 0.5 * (earliest + latest),
                span * span / 12.0};
    }
    // Residual alarm: the interval at the selected slope is empty. It can fire
    // while some other admissible slope still fits every arrival.
    [[nodiscard]] bool inconsistent() const { return empty_; }
    // The interval at the selected slope has negative width.
    [[nodiscard]] bool negativeWidth() const { return width_ < 0.0; }
    // Strict test: no (phase, period) pair with |period error| within the
    // bound fits every offset in the window into the delay prior. The
    // detrended spread is convex in the slope.
    [[nodiscard]] bool infeasible() const {
        const auto spread = [&](double slope) { return spreadAt(slope); };
        return minimizeConvex(spread, -kClockSlopeBound, kClockSlopeBound)
                   .second >
               maxDelay_ - minDelay_ + kClockFeasibilityTolerance;
    }

private:
    struct Point {
        int index{};
        double offset{};
    };

    // {max, min} over the window of c_k - slope (k - reference).
    [[nodiscard]] std::pair<double, double> bandAt(double slope,
                                                   int reference) const {
        double lowest = std::numeric_limits<double>::infinity();
        double highest = -std::numeric_limits<double>::infinity();
        for (std::size_t i = windowStart_; i < points_.size(); ++i) {
            const double detrended =
                points_[i].offset - slope * (points_[i].index - reference);
            lowest = std::min(lowest, detrended);
            highest = std::max(highest, detrended);
        }
        return {highest, lowest};
    }
    [[nodiscard]] double spreadAt(double slope) const {
        const auto band = bandAt(slope, points_.back().index);
        return band.first - band.second;
    }

    // Golden-section search for the minimum of a convex function on
    // [low, high]; returns {argument, value}, endpoints included.
    template <typename Function>
    static std::pair<double, double> minimizeConvex(const Function& f,
                                                    double low,
                                                    double high) {
        std::pair<double, double> best{low, f(low)};
        const double atHigh = f(high);
        if (atHigh < best.second) {
            best = {high, atHigh};
        }
        const double ratio = 0.5 * (std::sqrt(5.0) - 1.0);
        double left = high - ratio * (high - low);
        double right = low + ratio * (high - low);
        double leftValue = f(left);
        double rightValue = f(right);
        for (int iteration = 0; iteration < 80; ++iteration) {
            if (leftValue <= rightValue) {
                high = right;
                right = left;
                rightValue = leftValue;
                left = high - ratio * (high - low);
                leftValue = f(left);
            } else {
                low = left;
                left = right;
                leftValue = rightValue;
                right = low + ratio * (high - low);
                rightValue = f(right);
            }
        }
        if (leftValue < best.second) {
            best = {left, leftValue};
        }
        if (rightValue < best.second) {
            best = {right, rightValue};
        }
        return best;
    }

    void refit() {
        const int newest = points_.back().index;
        const auto first = std::lower_bound(
            points_.begin(), points_.end(), newest - kClockWindow + 1,
            [](const Point& p, int k) { return p.index < k; });
        hull_.clear();
        double indexSum = 0.0;
        for (auto it = first; it != points_.end(); ++it) {
            while (hull_.size() >= 2) {
                const Point& o = hull_[hull_.size() - 2];
                const Point& a = hull_.back();
                const double cross =
                    (a.index - o.index) * (it->offset - o.offset) -
                    (a.offset - o.offset) * (it->index - o.index);
                if (cross > 0.0) {
                    break;
                }
                hull_.pop_back();
            }
            hull_.push_back(*it);
            indexSum += it->index;
        }
        const double meanIndex =
            indexSum / static_cast<double>(points_.end() - first);
        double slope = 0.0;
        for (std::size_t i = 0; i + 1 < hull_.size(); ++i) {
            if (hull_[i + 1].index >= meanIndex) {
                slope = (hull_[i + 1].offset - hull_[i].offset) /
                        (hull_[i + 1].index - hull_[i].index);
                break;
            }
        }
        slope_ = std::clamp(slope, -kClockSlopeBound, kClockSlopeBound);
        double lowest = std::numeric_limits<double>::infinity();
        double highest = -std::numeric_limits<double>::infinity();
        for (auto it = first; it != points_.end(); ++it) {
            const double detrended = it->offset - slope_ * it->index;
            lowest = std::min(lowest, detrended);
            highest = std::max(highest, detrended);
        }
        intercept_ = lowest;
        windowStart_ = static_cast<std::size_t>(first - points_.begin());
        width_ = (lowest - minDelay_) - (highest - maxDelay_);
        empty_ = width_ < -kClockConsistencySlack;
        const double span = std::max(width_, kClockSpanFloor);
        variance_ = span * span / 3.0;
    }

    double minDelay_{};
    double maxDelay_{};
    std::vector<Point> points_;
    std::vector<Point> hull_;
    std::size_t windowStart_{};
    double intercept_{};
    double slope_{};
    double width_{};
    double variance_{};
    bool empty_{};
};

// The last five are diagnostics: B4MLs is B4ML with a near whole-run
// memory, Arb and B4rb release scans in sequence order (reorder buffer), AML
// runs the profile likelihood on true stamps to show which offset each model
// prefers by itself, B4mid hands over the interval midpoint instead of its
// upper end.
enum class Strategy {
    scanStamp,
    arrivalStamp,
    nominal,
    nominalFixedR,
    grid,
    gridBiased,
    gridProfile,
    gridProfileSlow,
    scanStampOrdered,
    gridOrdered,
    scanStampProfile,
    gridMidpoint
};

constexpr std::array<Strategy, 12> kStrategies{
    Strategy::scanStamp,        Strategy::arrivalStamp,
    Strategy::nominal,          Strategy::nominalFixedR,
    Strategy::grid,             Strategy::gridBiased,
    Strategy::gridProfile,      Strategy::gridProfileSlow,
    Strategy::scanStampOrdered, Strategy::gridOrdered,
    Strategy::scanStampProfile, Strategy::gridMidpoint};
constexpr std::size_t kFirstDiagnostic = 7;

const char* strategyName(Strategy strategy) {
    switch (strategy) {
        case Strategy::scanStamp:
            return "A";
        case Strategy::arrivalStamp:
            return "B0";
        case Strategy::nominal:
            return "B1";
        case Strategy::nominalFixedR:
            return "B1c";
        case Strategy::grid:
            return "B4";
        case Strategy::gridBiased:
            return "B4err";
        case Strategy::gridProfile:
            return "B4ML";
        case Strategy::gridProfileSlow:
            return "B4MLs";
        case Strategy::scanStampOrdered:
            return "Arb";
        case Strategy::gridOrdered:
            return "B4rb";
        case Strategy::gridMidpoint:
            return "B4mid";
        case Strategy::scanStampProfile:
        default:
            return "AML";
    }
}

bool usesGrid(Strategy strategy) {
    return strategy == Strategy::grid || strategy == Strategy::gridBiased ||
           strategy == Strategy::gridProfile ||
           strategy == Strategy::gridProfileSlow ||
           strategy == Strategy::gridOrdered ||
           strategy == Strategy::gridMidpoint;
}

bool isProfile(Strategy strategy) {
    return strategy == Strategy::gridProfile ||
           strategy == Strategy::gridProfileSlow ||
           strategy == Strategy::scanStampProfile;
}

bool isOrdered(Strategy strategy) {
    return strategy == Strategy::scanStampOrdered ||
           strategy == Strategy::gridOrdered;
}

struct Timing {
    double stamp{};
    double variance{};
    double fixedRateSq{-1.0};
};

struct Served {
    MeasurementOutcome outcome{};
    double mahalanobis{};
    double innovation{};
    double innovationCovariance{};
    double stampError{};
};

struct Tally {
    std::array<double, 5> phiSq{};
    std::array<double, 5> rateSq{};
    std::array<long, 5> ticks{};
    long packets{};
    long accepted{};
    long gated{};
    long outOfOrder{};
    long stale{};
    long other{};
    double nisSum{};
    long nisCount{};
    double stampErrorSum{};
    double stampErrorSq{};
    long stampCount{};
    long deferred{};
    long clamped{};
    long held{};
    long clockChecks{};
    long clockNegative{};
    long clockEmpty{};
    long clockInfeasible{};
    double offsetSum{};
    long offsetCount{};
    double finalOffset{};
    // Whole-run (t >= kScoreFrom) innovation log-likelihood per profile lane.
    std::array<double, kCandidates> profile{};
    double seconds{};

    void record(const Served& served) {
        ++packets;
        switch (served.outcome) {
            case MeasurementOutcome::accepted:
                ++accepted;
                break;
            case MeasurementOutcome::gated:
                ++gated;
                break;
            case MeasurementOutcome::outOfOrder:
                ++outOfOrder;
                break;
            case MeasurementOutcome::staleBeyondWindow:
                ++stale;
                break;
            default:
                ++other;
                break;
        }
        if (served.innovationCovariance > 0.0) {
            nisSum += served.mahalanobis;
            ++nisCount;
        }
        stampErrorSum += served.stampError;
        stampErrorSq += served.stampError * served.stampError;
        ++stampCount;
    }

    void score(double phi, double rate, const Truth& truth, std::size_t tick) {
        const std::size_t i = tick * kPlantStepsPerTick;
        const double phiError =
            std::remainder(phi - truth.articulation[i], 2.0 * kPi);
        const double rateError = rate - truth.articulationRate[i];
        const double magnitude = std::abs(truth.articulationRate[i]) * kDeg;
        const std::size_t bin = magnitude < 5.0    ? 1
                                : magnitude < 10.0 ? 2
                                : magnitude < 20.0 ? 3
                                                   : 4;
        for (const std::size_t b : {std::size_t{0}, bin}) {
            phiSq[b] += phiError * phiError;
            rateSq[b] += rateError * rateError;
            ++ticks[b];
        }
    }
};

struct Pending {
    const Packet* packet{};
    int index{};
    bool deferred{};
    bool held{};
};

template <typename Base>
struct Lane {
    using Filter = truck_model::DelayedEkf<TimedModel<Base>>;

    Filter filter;
    std::deque<Pending> pending;
    bool inOrder{};
    int nextIndex{};
    long deferred{};
    long clamped{};
    long held{};
    double lastStamp{-std::numeric_limits<double>::infinity()};
    int newestIndex{-1};
    double offset{};
    double logLikelihood{};

    void receive(const Packet* packet, int index) {
        const Pending entry{packet, index, false, false};
        if (!inOrder) {
            pending.push_back(entry);
            return;
        }
        const auto position = std::upper_bound(
            pending.begin(), pending.end(), index,
            [](int k, const Pending& p) { return k < p.index; });
        pending.insert(position, entry);
    }

    // Serves pending packets in arrival order, or in scan order with the
    // reorder buffer. A stamp past the newest input would come back
    // aheadOfInputs, so that packet and everything behind it wait for the
    // next tick. A newer scan whose reconstructed stamp does not advance,
    // which only a grid re-fit can cause, is nudged 1 ms past the last one;
    // an older scan is left to the shell's outOfOrder check, as with true
    // stamps.
    template <typename TimingOf>
    void serve(double now, const TimingOf& timingOf, std::vector<Served>& out) {
        while (!pending.empty()) {
            Pending& entry = pending.front();
            const Packet& packet = *entry.packet;
            Timing timing = timingOf(packet, entry.index);
            timing.stamp += offset;
            if (inOrder && entry.index > nextIndex) {
                // A predecessor is in flight. Give its slot up only once it
                // can no longer arrive.
                const double deadline =
                    timing.stamp - (entry.index - nextIndex) * kScanPeriod +
                    kMaxDelay + kControlPeriod;
                if (now < deadline) {
                    for (auto& p : pending) {
                        p.held = true;
                    }
                    return;
                }
                nextIndex = entry.index;
            }
            if (timing.stamp > now + 1.0e-9) {
                entry.deferred = true;
                return;
            }
            if (entry.index > newestIndex && timing.stamp <= lastStamp) {
                timing.stamp = lastStamp + 1.0e-3;
                ++clamped;
            }
            filter.model().arm(timing.stamp, timing.variance, timing.fixedRateSq);
            const auto report = filter.update(timing.stamp, packet.value);
            if (report.outcome == MeasurementOutcome::accepted ||
                report.outcome == MeasurementOutcome::gated) {
                lastStamp = std::max(lastStamp, timing.stamp);
            }
            newestIndex = std::max(newestIndex, entry.index);
            nextIndex = std::max(nextIndex, entry.index + 1);
            deferred += entry.deferred ? 1 : 0;
            held += entry.held ? 1 : 0;
            Served served;
            served.outcome = report.outcome;
            served.mahalanobis = report.mahalanobis;
            served.innovation = report.innovation;
            served.innovationCovariance = report.innovationCovariance;
            served.stampError = timing.stamp - packet.scan;
            out.push_back(served);
            pending.pop_front();
        }
    }
};

struct Case {
    const Truth* truth{};
    Latency latency{};
    std::vector<ArticulationInputs> inputs;
    std::vector<Packet> packets;
};

template <typename Base>
Tally runStrategy(const Case& data,
                  Strategy strategy,
                  const Base& base,
                  const ArticulationEstimatorConfig& config) {
    const auto started = std::chrono::steady_clock::now();
    const bool profile = isProfile(strategy);
    const bool grid = usesGrid(strategy);
    const int laneCount = profile ? kCandidates : 1;

    truck_model::DelayedEkfLimits limits;
    limits.historyHorizon = config.historyHorizon;
    limits.mahalanobisGate = config.mahalanobisGate;
    limits.consecutiveRejectLimit = config.consecutiveRejectLimit;
    limits.lostTimeout = config.lostTimeout;
    limits.coastingProcessNoiseScale = config.coastingProcessNoiseScale;
    limits.maximumFrames = config.maximumFrames;

    truck_model::Vector<Base::kStateSize> state{};
    truck_model::Matrix<Base::kStateSize, Base::kStateSize> covariance{};
    initialPrior(base, config, state, covariance);

    std::vector<Lane<Base>> lanes(static_cast<std::size_t>(laneCount));
    for (int j = 0; j < laneCount; ++j) {
        auto& lane = lanes[static_cast<std::size_t>(j)];
        lane.filter.configure(
            TimedModel<Base>(base, &data.inputs, config.measurementVariance),
            limits);
        lane.filter.reset(data.inputs.front().time, state, covariance,
                          data.inputs.front());
        lane.offset = profile ? (j - kCandidates / 2) * kCandidateStep : 0.0;
        lane.inOrder = isOrdered(strategy);
    }
    std::size_t selected = profile ? kCandidates / 2 : 0;

    const bool biasedPrior = strategy == Strategy::gridBiased ||
                             strategy == Strategy::gridProfile ||
                             strategy == Strategy::gridProfileSlow;
    ScanClock clock(biasedPrior ? kMinDelay + kPriorError : kMinDelay,
                    kMaxDelay);
    const double forgetting =
        strategy == Strategy::gridProfileSlow ? kSlowForgetting : kForgetting;
    const double fixedRateSq = data.truth->rateRms * data.truth->rateRms +
                               kRateFloor * kRateFloor;
    const auto timingOf = [&](const Packet& packet, int index) {
        Timing timing;
        switch (strategy) {
            case Strategy::scanStamp:
            case Strategy::scanStampOrdered:
            case Strategy::scanStampProfile:
                timing.stamp = packet.scan;
                break;
            case Strategy::arrivalStamp:
                timing.stamp = packet.arrival;
                break;
            case Strategy::nominal:
            case Strategy::nominalFixedR:
                timing.stamp = packet.arrival - kNominalDelay;
                timing.variance = kTimingStd * kTimingStd;
                if (strategy == Strategy::nominalFixedR) {
                    timing.fixedRateSq = fixedRateSq;
                }
                break;
            case Strategy::gridMidpoint: {
                const auto fit = clock.reconstructJoint(index);
                timing.stamp = fit.stamp;
                timing.variance = fit.variance;
                break;
            }
            default: {
                const auto fit = clock.reconstruct(index);
                timing.stamp = fit.first;
                timing.variance = fit.second;
                break;
            }
        }
        return timing;
    };

    Tally tally;
    std::vector<std::vector<Served>> served(lanes.size());
    std::size_t next = 0;
    int arrivals = 0;
    for (std::size_t n = 1; n < data.inputs.size(); ++n) {
        const auto& inputs = data.inputs[n];
        const double now = inputs.time;
        const bool scored = now >= kScoreFrom - 1.0e-9;
        for (auto& lane : lanes) {
            lane.filter.predict(inputs, now);
        }
        while (next < data.packets.size() &&
               data.packets[next].arrival <= now) {
            const Packet& packet = data.packets[next++];
            // FIFO: the arrival count is the scan index. D1 assumes a frame
            // counter in the packet.
            const int index =
                data.latency == Latency::fifo ? arrivals : packet.index;
            ++arrivals;
            if (grid) {
                clock.add(index, packet.arrival);
                if (scored) {
                    ++tally.clockChecks;
                    tally.clockNegative += clock.negativeWidth() ? 1 : 0;
                    tally.clockEmpty += clock.inconsistent() ? 1 : 0;
                    tally.clockInfeasible += clock.infeasible() ? 1 : 0;
                }
            }
            for (auto& lane : lanes) {
                lane.receive(&packet, index);
            }
        }
        for (std::size_t j = 0; j < lanes.size(); ++j) {
            served[j].clear();
            lanes[j].serve(now, timingOf, served[j]);
            for (const auto& s : served[j]) {
                if (s.innovationCovariance > 0.0) {
                    const double logLikelihood =
                        -0.5 * (s.innovation * s.innovation /
                                    s.innovationCovariance +
                                std::log(s.innovationCovariance));
                    lanes[j].logLikelihood =
                        forgetting * lanes[j].logLikelihood + logLikelihood;
                    if (profile && scored) {
                        tally.profile[j] += logLikelihood;
                    }
                }
            }
        }
        if (profile) {
            for (std::size_t j = 0; j < lanes.size(); ++j) {
                if (lanes[j].logLikelihood > lanes[selected].logLikelihood) {
                    selected = j;
                }
            }
        }
        if (!scored) {
            continue;
        }
        for (const auto& s : served[selected]) {
            tally.record(s);
        }
        const auto& lane = lanes[selected];
        const auto& estimate = lane.filter.state();
        tally.score(articulation(estimate),
                    articulationRate(base, estimate, inputs), *data.truth, n);
        if (profile) {
            tally.offsetSum += lane.offset;
            ++tally.offsetCount;
        }
    }
    tally.deferred = lanes[selected].deferred;
    tally.clamped = lanes[selected].clamped;
    tally.held = lanes[selected].held;
    tally.finalOffset = lanes[selected].offset;
    tally.seconds = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - started)
                        .count();
    return tally;
}

struct Job {
    std::uint64_t seed{};
    Latency latency{};
    std::size_t noise{};
    std::size_t model{};  // 0 = K5, 1 = K27r
};

using Results = std::array<Tally, kStrategies.size()>;

Results runJob(const Job& job, const std::array<Truth, 2>& truths) {
    const Truth& truth = truths[job.model];
    const double noiseStd = kNoiseLevels[job.noise] / kDeg;
    Case data;
    data.truth = &truth;
    data.latency = job.latency;
    data.inputs = makeInputs(truth, job.seed);
    data.packets = makePackets(truth, job.latency, noiseStd, job.seed);

    ArticulationEstimatorConfig config;
    config.processModel = job.model == 0
                              ? truck_model::ArticulationProcessModel::kinematic
                              : truck_model::ArticulationProcessModel::dynamic;
    config.measurementVariance = noiseStd * noiseStd;
    config.historyHorizon = kHistoryHorizon;
    // Both filters carry the nominal vehicle; the K27r one runs against the
    // c2r * 0.8 plant.
    const Parameters nominal = vehicle();
    Results results;
    for (std::size_t s = 0; s < kStrategies.size(); ++s) {
        results[s] =
            job.model == 0
                ? runStrategy(data, kStrategies[s],
                              KinematicArticulationModel(nominal, config), config)
                : runStrategy(data, kStrategies[s],
                              DynamicArticulationModel(nominal, config), config);
    }
    return results;
}

std::string output;

void emit(const char* format, ...) {
    char buffer[1024];
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(buffer, sizeof buffer, format, arguments);
    va_end(arguments);
    output += buffer;
}

struct Summary {
    double phi{};
    double phiSd{};
    double rate{};
    double rateSd{};
    std::array<double, 4> phiBin{};
    std::array<double, 4> rateBin{};
    double stampBias{};
    double stampRmse{};
    double nis{};
    double acceptedPercent{};
    double gated{};
    double outOfOrder{};
    double stale{};
    double other{};
    double served{};
    double deferred{};
    double clamped{};
    double held{};
    double negativePercent{};
    double emptyPercent{};
    double infeasiblePercent{};
    double offset{};
    std::array<double, kCandidates> profile{};
    // Per-seed parabolic peak of the whole-run profile [ms], and how many
    // seeds peaked on the outermost lane.
    double peak{};
    double peakSd{};
    int peakAtEdge{};
    // Lane selected online at the end of the run [ms].
    double finalOffset{};
    double finalOffsetSd{};
    double seconds{};
};

double sampleSd(double sum, double sumSq, double count) {
    if (count < 2.0) {
        return 0.0;
    }
    return std::sqrt(std::max(0.0, (sumSq - sum * sum / count) / (count - 1.0)));
}

Summary summarize(const std::vector<const Tally*>& runs) {
    Summary s;
    const double count = static_cast<double>(runs.size());
    double phiSq = 0.0;
    double rateSq = 0.0;
    double packets = 0.0;
    double accepted = 0.0;
    double nisSum = 0.0;
    double nisCount = 0.0;
    double stampSum = 0.0;
    double stampSq = 0.0;
    double stampCount = 0.0;
    double checks = 0.0;
    double negative = 0.0;
    double empty = 0.0;
    double infeasible = 0.0;
    double offsetSum = 0.0;
    double offsetCount = 0.0;
    double peakSum = 0.0;
    double peakSq = 0.0;
    double finalSum = 0.0;
    double finalSq = 0.0;
    for (const Tally* t : runs) {
        const double phi =
            std::sqrt(t->phiSq[0] / static_cast<double>(t->ticks[0])) * kDeg;
        const double rate =
            std::sqrt(t->rateSq[0] / static_cast<double>(t->ticks[0])) * kDeg;
        s.phi += phi;
        phiSq += phi * phi;
        s.rate += rate;
        rateSq += rate * rate;
        for (std::size_t b = 0; b < 4; ++b) {
            const auto ticks = static_cast<double>(t->ticks[b + 1]);
            if (ticks > 0.0) {
                s.phiBin[b] += std::sqrt(t->phiSq[b + 1] / ticks) * kDeg;
                s.rateBin[b] += std::sqrt(t->rateSq[b + 1] / ticks) * kDeg;
            }
        }
        packets += static_cast<double>(t->packets);
        accepted += static_cast<double>(t->accepted);
        s.gated += static_cast<double>(t->gated);
        s.outOfOrder += static_cast<double>(t->outOfOrder);
        s.stale += static_cast<double>(t->stale);
        s.other += static_cast<double>(t->other);
        s.deferred += static_cast<double>(t->deferred);
        s.clamped += static_cast<double>(t->clamped);
        s.held += static_cast<double>(t->held);
        nisSum += t->nisSum;
        nisCount += static_cast<double>(t->nisCount);
        stampSum += t->stampErrorSum;
        stampSq += t->stampErrorSq;
        stampCount += static_cast<double>(t->stampCount);
        checks += static_cast<double>(t->clockChecks);
        negative += static_cast<double>(t->clockNegative);
        empty += static_cast<double>(t->clockEmpty);
        infeasible += static_cast<double>(t->clockInfeasible);
        offsetSum += t->offsetSum;
        offsetCount += static_cast<double>(t->offsetCount);
        const auto bestAt =
            std::max_element(t->profile.begin(), t->profile.end());
        for (std::size_t j = 0; j < kCandidates; ++j) {
            s.profile[j] += (t->profile[j] - *bestAt) / count;
        }
        const auto j = static_cast<int>(bestAt - t->profile.begin());
        double peak = (j - kCandidates / 2) * kCandidateStep;
        if (j == 0 || j == kCandidates - 1) {
            ++s.peakAtEdge;
        } else {
            const double left = t->profile[static_cast<std::size_t>(j - 1)];
            const double right = t->profile[static_cast<std::size_t>(j + 1)];
            const double curvature = left - 2.0 * *bestAt + right;
            if (curvature < 0.0) {
                peak += 0.5 * kCandidateStep * (left - right) / curvature;
            }
        }
        peakSum += 1000.0 * peak;
        peakSq += 1.0e6 * peak * peak;
        finalSum += 1000.0 * t->finalOffset;
        finalSq += 1.0e6 * t->finalOffset * t->finalOffset;
        s.seconds += t->seconds;
    }
    s.peak = peakSum / count;
    s.peakSd = sampleSd(peakSum, peakSq, count);
    s.finalOffset = finalSum / count;
    s.finalOffsetSd = sampleSd(finalSum, finalSq, count);
    s.phiSd = sampleSd(s.phi, phiSq, count);
    s.rateSd = sampleSd(s.rate, rateSq, count);
    s.phi /= count;
    s.rate /= count;
    for (std::size_t b = 0; b < 4; ++b) {
        s.phiBin[b] /= count;
        s.rateBin[b] /= count;
    }
    s.gated /= count;
    s.outOfOrder /= count;
    s.stale /= count;
    s.other /= count;
    s.deferred /= count;
    s.clamped /= count;
    s.held /= count;
    s.served = packets / count;
    s.acceptedPercent = packets > 0.0 ? 100.0 * accepted / packets : 0.0;
    s.nis = nisCount > 0.0 ? nisSum / nisCount : 0.0;
    s.stampBias = stampCount > 0.0 ? 1000.0 * stampSum / stampCount : 0.0;
    s.stampRmse =
        stampCount > 0.0 ? 1000.0 * std::sqrt(stampSq / stampCount) : 0.0;
    s.negativePercent = checks > 0.0 ? 100.0 * negative / checks : -1.0;
    s.emptyPercent = checks > 0.0 ? 100.0 * empty / checks : -1.0;
    s.infeasiblePercent = checks > 0.0 ? 100.0 * infeasible / checks : -1.0;
    s.offset = offsetCount > 0.0 ? 1000.0 * offsetSum / offsetCount : 0.0;
    return s;
}

double runRmse(const Tally& tally, int metric) {
    const double sq = metric == 0 ? tally.phiSq[0] : tally.rateSq[0];
    return std::sqrt(sq / static_cast<double>(tally.ticks[0])) * kDeg;
}

std::size_t strategyIndex(Strategy strategy) {
    return static_cast<std::size_t>(
        std::find(kStrategies.begin(), kStrategies.end(), strategy) -
        kStrategies.begin());
}

const char* latencyName(Latency latency) {
    return latency == Latency::iid ? "D1" : "D2";
}

const char* modelName(std::size_t model) { return model == 0 ? "K5" : "K27r"; }

void describeScenario(const std::array<Truth, 2>& truths) {
    for (std::size_t m = 0; m < truths.size(); ++m) {
        const Truth& truth = truths[m];
        std::array<long, 4> bins{};
        long total = 0;
        double maxRate = 0.0;
        double maxPhi = 0.0;
        for (std::size_t i = 0; i < truth.articulation.size();
             i += kPlantStepsPerTick) {
            if (static_cast<double>(i) * kPlantStep < kScoreFrom) {
                continue;
            }
            const double rate = std::abs(truth.articulationRate[i]) * kDeg;
            ++bins[rate < 5.0 ? 0 : rate < 10.0 ? 1 : rate < 20.0 ? 2 : 3];
            ++total;
            maxRate = std::max(maxRate, rate);
            maxPhi = std::max(maxPhi, std::abs(truth.articulation[i]) * kDeg);
        }
        emit("  %-4s plant (%s): |phidot| occupancy 0-5 %.1f%%  5-10 %.1f%%  "
             "10-20 %.1f%%  >20 %.1f%%;  rms %.2f deg/s, max %.1f deg/s, "
             "max |phi| %.1f deg\n",
             modelName(m), m == 0 ? "nominal" : "c2r x 0.8",
             100.0 * bins[0] / total, 100.0 * bins[1] / total,
             100.0 * bins[2] / total, 100.0 * bins[3] / total,
             truth.rateRms * kDeg, maxRate, maxPhi);
    }
}

void describeLatency(const Truth& truth, int seeds) {
    for (const Latency latency : {Latency::iid, Latency::fifo}) {
        double sum = 0.0;
        double sumSq = 0.0;
        double lag = 0.0;
        double low = 1.0;
        double high = 0.0;
        long count = 0;
        long lagCount = 0;
        long overtaken = 0;
        long nearFloor = 0;
        long atCap = 0;
        std::vector<double> byIndex;
        for (int seed = 0; seed < seeds; ++seed) {
            auto packets = makePackets(truth, latency, 0.0,
                                       static_cast<std::uint64_t>(seed));
            int newest = -1;
            for (const auto& p : packets) {
                overtaken += p.index < newest ? 1 : 0;
                newest = std::max(newest, p.index);
            }
            byIndex.assign(packets.size(), 0.0);
            for (const auto& p : packets) {
                const double tau = p.arrival - p.scan;
                if (static_cast<std::size_t>(p.index) < byIndex.size()) {
                    byIndex[static_cast<std::size_t>(p.index)] = tau;
                }
                sum += tau;
                sumSq += tau * tau;
                low = std::min(low, tau);
                high = std::max(high, tau);
                nearFloor += tau < kMinDelay + 0.005 ? 1 : 0;
                atCap += tau > kMaxDelay - 0.005 ? 1 : 0;
                ++count;
            }
            for (std::size_t k = 1; k < byIndex.size(); ++k) {
                lag += byIndex[k] * byIndex[k - 1];
                ++lagCount;
            }
        }
        const double mean = sum / static_cast<double>(count);
        const double variance = sumSq / static_cast<double>(count) - mean * mean;
        const double lag1 =
            (lag / static_cast<double>(lagCount) - mean * mean) / variance;
        emit("  %s %-24s mean %.3f s  sd %.3f s  range [%.3f, %.3f]  lag-1 "
             "corr %.2f  overtaken %.1f%%  <0.105 s %.1f%%  >0.395 s %.1f%%\n",
             latencyName(latency),
             latency == Latency::iid ? "iid U[0.1,0.4]:" : "FIFO Lindley queue:",
             mean, std::sqrt(variance), low, high, lag1,
             100.0 * overtaken / count, 100.0 * nearFloor / count,
             100.0 * atCap / count);
    }
}

}  // namespace

int main(int argc, char** argv) {
    const char* outputPath = argc > 1 ? argv[1] : nullptr;
    const int seeds = argc > 2 ? std::max(1, std::atoi(argv[2])) : 20;
    const auto started = std::chrono::steady_clock::now();

    Parameters mismatched = vehicle();
    mismatched.c2r *= 0.8;
    const std::array<Truth, 2> truths{simulatePlant(vehicle()),
                                      simulatePlant(mismatched)};

    std::vector<Job> jobs;
    for (int seed = 0; seed < seeds; ++seed) {
        for (const Latency latency : {Latency::iid, Latency::fifo}) {
            for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
                for (std::size_t model = 0; model < 2; ++model) {
                    jobs.push_back({static_cast<std::uint64_t>(seed), latency,
                                    noise, model});
                }
            }
        }
    }
    std::vector<Results> results(jobs.size());
    std::atomic<std::size_t> nextJob{0};
    std::mutex failureMutex;
    std::string failure;
    const unsigned workers =
        std::max(1u, std::min(8u, std::thread::hardware_concurrency()));
    std::vector<std::thread> pool;
    for (unsigned w = 0; w < workers; ++w) {
        pool.emplace_back([&] {
            for (;;) {
                const std::size_t j = nextJob++;
                if (j >= jobs.size()) {
                    return;
                }
                try {
                    results[j] = runJob(jobs[j], truths);
                } catch (const std::exception& error) {
                    const std::lock_guard<std::mutex> lock(failureMutex);
                    failure += error.what();
                    failure += "\n";
                }
            }
        });
    }
    for (auto& thread : pool) {
        thread.join();
    }
    if (!failure.empty()) {
        std::fprintf(stderr, "run failed:\n%s", failure.c_str());
        return 1;
    }
    const double wall = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - started)
                            .count();

    emit("Case B probe: no scan stamp, exact arrival instant, latency bound "
         "[%.1f, %.1f] s\n",
         kMinDelay, kMaxDelay);
    emit("  %d seeds, %.0f s drive, scored t >= %.0f s; inputs 20 Hz "
         "(r1 +-%.1f deg/s, U +-%.2f m/s); scans 10 Hz, phase %.0f ms, "
         "period %+.0f ppm; served at the first tick after arrival\n",
         seeds, kDuration, kScoreFrom, kYawRateNoise * kDeg, kSpeedNoise,
         kScanPhase * 1000.0, kScanClockError * 1.0e6);
    emit("  K5 filter vs nominal K27 plant; K27r filter (nominal c2r) vs c2r "
         "x 0.8 plant; historyHorizon %.2f s, gate %.0f\n",
         kHistoryHorizon, ArticulationEstimatorConfig{}.mahalanobisGate);
    describeScenario(truths);
    describeLatency(truths[0], seeds);

    const auto collect = [&](Latency latency, std::size_t noise,
                             std::size_t model, std::size_t strategy) {
        std::vector<const Tally*> runs;
        for (std::size_t j = 0; j < jobs.size(); ++j) {
            if (jobs[j].latency == latency && jobs[j].noise == noise &&
                jobs[j].model == model) {
                runs.push_back(&results[j][strategy]);
            }
        }
        return summarize(runs);
    };

    for (int metric = 0; metric < 2; ++metric) {
        emit("\n%s RMSE, mean over seeds [%s]\n",
             metric == 0 ? "Current phi" : "Current phidot",
             metric == 0 ? "deg" : "deg/s");
        emit("strat ");
        for (const Latency latency : {Latency::iid, Latency::fifo}) {
            for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
                for (std::size_t model = 0; model < 2; ++model) {
                    char label[32];
                    std::snprintf(label, sizeof label, "%s/%.1f/%s",
                                  latencyName(latency), kNoiseLevels[noise],
                                  modelName(model));
                    emit(" %12s", label);
                }
            }
        }
        emit("\n");
        for (std::size_t s = 0; s < kStrategies.size(); ++s) {
            if (s == kFirstDiagnostic) {
                emit("-- diagnostics --\n");
            }
            emit("%-6s", strategyName(kStrategies[s]));
            for (const Latency latency : {Latency::iid, Latency::fifo}) {
                for (std::size_t noise = 0; noise < kNoiseLevels.size();
                     ++noise) {
                    for (std::size_t model = 0; model < 2; ++model) {
                        const auto summary = collect(latency, noise, model, s);
                        emit(" %12.3f", metric == 0 ? summary.phi : summary.rate);
                    }
                }
            }
            emit("\n");
        }
    }

    // Every strategy of a job sees the same packets and inputs, so per-seed
    // differences cancel most of the seed-to-seed spread.
    const std::array<std::pair<Strategy, Strategy>, 12> pairs{{
        {Strategy::nominal, Strategy::arrivalStamp},
        {Strategy::nominal, Strategy::nominalFixedR},
        {Strategy::grid, Strategy::nominal},
        {Strategy::grid, Strategy::scanStamp},
        {Strategy::nominal, Strategy::scanStamp},
        {Strategy::gridBiased, Strategy::grid},
        {Strategy::gridProfile, Strategy::gridBiased},
        {Strategy::gridProfile, Strategy::grid},
        {Strategy::gridProfileSlow, Strategy::gridBiased},
        {Strategy::scanStampOrdered, Strategy::scanStamp},
        {Strategy::gridOrdered, Strategy::grid},
        {Strategy::gridMidpoint, Strategy::grid},
    }};
    for (int metric = 0; metric < 2; ++metric) {
        emit("\nPaired %s RMSE difference, mean(standard error) over seeds "
             "[%s], negative = first is better\n",
             metric == 0 ? "phi" : "phidot", metric == 0 ? "deg" : "deg/s");
        emit("%-11s", "pair");
        for (const Latency latency : {Latency::iid, Latency::fifo}) {
            for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
                for (std::size_t model = 0; model < 2; ++model) {
                    char label[32];
                    std::snprintf(label, sizeof label, "%s/%.1f/%s",
                                  latencyName(latency), kNoiseLevels[noise],
                                  modelName(model));
                    emit(" %14s", label);
                }
            }
        }
        emit("\n");
        for (const auto& pair : pairs) {
            char label[32];
            std::snprintf(label, sizeof label, "%s-%s",
                          strategyName(pair.first), strategyName(pair.second));
            emit("%-11s", label);
            const std::size_t first = strategyIndex(pair.first);
            const std::size_t second = strategyIndex(pair.second);
            for (const Latency latency : {Latency::iid, Latency::fifo}) {
                for (std::size_t noise = 0; noise < kNoiseLevels.size();
                     ++noise) {
                    for (std::size_t model = 0; model < 2; ++model) {
                        double sum = 0.0;
                        double sumSq = 0.0;
                        double count = 0.0;
                        for (std::size_t j = 0; j < jobs.size(); ++j) {
                            if (jobs[j].latency == latency &&
                                jobs[j].noise == noise &&
                                jobs[j].model == model) {
                                const double d =
                                    runRmse(results[j][first], metric) -
                                    runRmse(results[j][second], metric);
                                sum += d;
                                sumSq += d * d;
                                count += 1.0;
                            }
                        }
                        const double se =
                            sampleSd(sum, sumSq, count) / std::sqrt(count);
                        emit(metric == 0 ? "  %+7.3f(%5.3f)" : "  %+7.2f(%5.2f)",
                             sum / count, se);
                    }
                }
            }
            emit("\n");
        }
    }

    emit("\nWhole-run profile log-likelihood (t >= %.0f s), max-normalised "
         "per seed, mean over seeds; columns are lane offsets [ms]\n",
         kScoreFrom);
    emit("%-18s", "strat config");
    for (int j = 0; j < kCandidates; ++j) {
        emit(" %6.1f", (j - kCandidates / 2) * kCandidateStep * 1000.0);
    }
    emit(" | %12s %6s %4s\n", "peak(sd)", "net", "edge");
    for (const Strategy strategy :
         {Strategy::scanStampProfile, Strategy::gridProfile}) {
        for (const Latency latency : {Latency::iid, Latency::fifo}) {
            for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
                for (std::size_t model = 0; model < 2; ++model) {
                    const auto summary = collect(latency, noise, model,
                                                 strategyIndex(strategy));
                    // B4ML lanes offset the B4err grid, AML lanes true stamps.
                    const double base =
                        strategy == Strategy::gridProfile
                            ? collect(latency, noise, model,
                                      strategyIndex(Strategy::gridBiased))
                                  .stampBias
                            : 0.0;
                    char label[40];
                    std::snprintf(label, sizeof label, "%s %s/%.1f/%s",
                                  strategyName(strategy), latencyName(latency),
                                  kNoiseLevels[noise], modelName(model));
                    emit("%-18s", label);
                    for (const double value : summary.profile) {
                        emit(" %6.1f", value);
                    }
                    char peak[32];
                    std::snprintf(peak, sizeof peak, "%+.1f(%.1f)",
                                  summary.peak, summary.peakSd);
                    emit(" | %12s %+6.1f %4d\n", peak, summary.peak + base,
                         summary.peakAtEdge);
                }
            }
        }
    }
    emit("  peak: per-seed parabolic maximum of the whole-run profile, "
         "mean(sd) over seeds [ms]; net = peak + stamp bias of the lanes' "
         "source (B4err grid for B4ML, 0 for AML) = implied stamp error of "
         "an offline offset fit; edge: seeds peaking on the outermost lane. "
         "B4MLs shares B4ML's lanes, so its profile is identical.\n");

    emit("\nOnline selection at t = %.0f s as net stamp error [ms] (lane "
         "offset + source bias), mean(sd) over seeds\n",
         kDuration);
    emit("%-6s", "strat");
    for (const Latency latency : {Latency::iid, Latency::fifo}) {
        for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
            for (std::size_t model = 0; model < 2; ++model) {
                char label[32];
                std::snprintf(label, sizeof label, "%s/%.1f/%s",
                              latencyName(latency), kNoiseLevels[noise],
                              modelName(model));
                emit(" %13s", label);
            }
        }
    }
    emit("\n");
    for (const Strategy strategy :
         {Strategy::scanStampProfile, Strategy::gridProfile,
          Strategy::gridProfileSlow}) {
        emit("%-6s", strategyName(strategy));
        for (const Latency latency : {Latency::iid, Latency::fifo}) {
            for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
                for (std::size_t model = 0; model < 2; ++model) {
                    const auto summary = collect(latency, noise, model,
                                                 strategyIndex(strategy));
                    const double base =
                        strategy == Strategy::scanStampProfile
                            ? 0.0
                            : collect(latency, noise, model,
                                      strategyIndex(Strategy::gridBiased))
                                  .stampBias;
                    char cell[32];
                    std::snprintf(cell, sizeof cell, "%+.1f(%.1f)",
                                  summary.finalOffset + base,
                                  summary.finalOffsetSd);
                    emit(" %13s", cell);
                }
            }
        }
        emit("\n");
    }

    for (const Latency latency : {Latency::iid, Latency::fifo}) {
        for (std::size_t noise = 0; noise < kNoiseLevels.size(); ++noise) {
            for (std::size_t model = 0; model < 2; ++model) {
                emit("\n== %s %s | lidar %.1f deg | %s ==\n",
                     latencyName(latency),
                     latency == Latency::iid ? "iid" : "FIFO",
                     kNoiseLevels[noise], modelName(model));
                emit("%-6s %-14s %-12s | %-23s | %s\n", "strat", "phi[deg]",
                     "phidot[deg/s]", "phi by |phidot| [deg]",
                     "phidot by |phidot| [deg/s]");
                emit("%-6s %-14s %-12s | %5s %5s %5s %5s | %5s %5s %5s %5s\n",
                     "", "mean(sd)", "mean(sd)", "0-5", "5-10", "10-20", ">20",
                     "0-5", "5-10", "10-20", ">20");
                std::array<Summary, kStrategies.size()> summaries;
                for (std::size_t s = 0; s < kStrategies.size(); ++s) {
                    summaries[s] = collect(latency, noise, model, s);
                    const auto& m = summaries[s];
                    if (s == kFirstDiagnostic) {
                        emit("-- diagnostics --\n");
                    }
                    emit("%-6s %6.3f(%5.3f) %6.2f(%4.2f) | %5.3f %5.3f %5.3f "
                         "%5.3f | %5.2f %5.2f %5.2f %5.2f\n",
                         strategyName(kStrategies[s]), m.phi, m.phiSd, m.rate,
                         m.rateSd, m.phiBin[0], m.phiBin[1], m.phiBin[2],
                         m.phiBin[3], m.rateBin[0], m.rateBin[1],
                         m.rateBin[2], m.rateBin[3]);
                }
                emit("%-6s %6s %6s | %5s %5s | %5s %5s %5s %5s %5s | %5s %5s "
                     "%5s %5s %6s %7s | %6s | %6s\n",
                     "strat", "bias", "rmse", "NIS", "acc%", "served", "gated",
                     "ooo", "stale", "other", "defer", "clamp", "hold",
                     "w<0%", "empty%", "infeas%", "offset", "cpu[s]");
                for (std::size_t s = 0; s < kStrategies.size(); ++s) {
                    const auto& m = summaries[s];
                    char empty[40];
                    char offset[16];
                    if (usesGrid(kStrategies[s])) {
                        std::snprintf(empty, sizeof empty, "%5.1f %6.1f %7.1f",
                                      m.negativePercent, m.emptyPercent,
                                      m.infeasiblePercent);
                    } else {
                        std::snprintf(empty, sizeof empty, "%5s %6s %7s", "-",
                                      "-", "-");
                    }
                    if (isProfile(kStrategies[s])) {
                        std::snprintf(offset, sizeof offset, "%+6.1f", m.offset);
                    } else {
                        std::snprintf(offset, sizeof offset, "%6s", "-");
                    }
                    if (s == kFirstDiagnostic) {
                        emit("-- diagnostics --\n");
                    }
                    emit("%-6s %6.1f %6.1f | %5.2f %5.1f | %5.0f %5.1f %5.1f "
                         "%5.1f %5.1f | %5.1f %5.1f %5.1f %s | %s | %6.1f\n",
                         strategyName(kStrategies[s]), m.stampBias, m.stampRmse,
                         m.nis, m.acceptedPercent, m.served, m.gated,
                         m.outOfOrder, m.stale, m.other, m.deferred, m.clamped,
                         m.held, empty, offset, m.seconds);
                }
            }
        }
    }

    std::array<double, kStrategies.size()> cpu{};
    double cpuTotal = 0.0;
    for (const auto& run : results) {
        for (std::size_t s = 0; s < kStrategies.size(); ++s) {
            cpu[s] += run[s].seconds;
            cpuTotal += run[s].seconds;
        }
    }
    emit("\nLegend:\n");
    emit("  A     true scan stamp, served in arrival order (achievable bound)\n");
    emit("  B0    arrival instant as stamp, R = sensor variance\n");
    emit("  B1    stamp a - %.2f s, R_eff = R + sigma_e^2 (rate_anchor^2 + "
         "(%.1f deg/s)^2), sigma_e = %.1f ms, rate at the anchor-frame prior\n",
         kNominalDelay, kRateFloor * kDeg, kTimingStd * 1000.0);
    emit("  B1c   B1 stamp, constant R_eff with the plant's rms phidot in place "
         "of rate_anchor\n");
    emit("  B4    scan grid: affine lower envelope of a_k - k T over %d scans, "
         "tau_min = %.2f s; index = frame counter (D1) or arrival count (D2); "
         "stamp at the interval's upper end, R_eff with sigma_t^2 = interval "
         "width^2 / 3 (second moment of its one-sided error, no mean "
         "correction)\n",
         kClockWindow, kMinDelay);
    emit("  B4err B4 with the tau_min prior %+.0f ms off\n",
         kPriorError * 1000.0);
    emit("  B4ML  B4err grid + %d lanes offset by %.1f ms steps, the lane with "
         "the largest forgetting (%.3f/scan) pre-update innovation "
         "log-likelihood is the output\n",
         kCandidates, kCandidateStep * 1000.0, kForgetting);
    emit("  B4MLs B4ML with forgetting %.3f/scan (memory ~ the whole run)\n",
         kSlowForgetting);
    emit("  Arb, B4rb  A and B4 released in scan order (reorder buffer on the "
         "sequence number); AML the B4ML profile on true stamps\n");
    emit("  B4mid B4 window, but the scan time's range over every admissible "
         "(phase, period) pair; stamp at its midpoint, R_eff with sigma_t^2 = "
         "max(width, %.0f ms)^2 / 12; falls back to the selected slope when "
         "no pair is admissible\n",
         kClockSpanFloor * 1000.0);
    emit("  bias/rmse: used stamp - true scan time [ms] (profiles: selected "
         "lane) | NIS over accepted + gated | served..other: per run, t >= "
         "%.0f s | defer (aheadOfInputs waits), clamp (monotone nudges), hold "
         "(reorder waits): per run, whole run | w<0%%: scans whose interval at "
         "the selected slope had negative width | empty%%: residual alarm, "
         "scans whose delay-prior interval at the selected slope was empty "
         "beyond %.0f ms | infeas%%: strict test, scans for which no slope within "
         "+-%.0f ppm fits the window into the delay prior | offset: mean "
         "selected profile offset [ms] | cpu: summed over seeds\n",
         kScoreFrom, kClockConsistencySlack * 1000.0,
         kClockSlopeBound / kScanPeriod * 1.0e6);
    emit("CPU per strategy [s]:");
    for (std::size_t s = 0; s < kStrategies.size(); ++s) {
        emit(" %s %.1f", strategyName(kStrategies[s]), cpu[s]);
    }
    emit(" | total %.1f s on %u threads, wall %.1f s\n", cpuTotal, workers,
         wall);

    std::fputs(output.c_str(), stdout);
    if (outputPath != nullptr) {
        if (std::FILE* file = std::fopen(outputPath, "w")) {
            std::fputs(output.c_str(), file);
            std::fclose(file);
        } else {
            std::fprintf(stderr, "cannot write %s\n", outputPath);
            return 1;
        }
    }
    return 0;
}
