// Scratch: is historyHorizon >= tau_max + dt actually sufficient? Sweeps the
// stamp phase against a window sized by that rule and reports any rejection.
#include "truck_model/articulation_estimator.hpp"

#include <cstdio>

int main() {
    truck_model::Parameters p;
    p.m1 = 8000; p.iz1 = 25000; p.a1 = 1.5; p.b1 = 2.5;
    p.c1f = 220000; p.c1r = 300000;
    p.m2 = 18000; p.iz2 = 140000; p.a2 = 4; p.b2 = 3; p.c2r = 500000;
    p.d1 = p.b1; p.vx = 15;

    const double dt = 0.05;
    // Deliberately off the control grid, as a real latency bound would be.
    const double tauMax = 0.401;

    for (const double extra : {1.0, 1.5, 2.0}) {
        const double window = tauMax + extra * dt;
        int rejected = 0;
        double worstAge = 0.0;
        // The lidar is not locked to the control grid, so the scan instant
        // falls anywhere inside a control period. Delivery is then quantised
        // up to the next tick.
        for (int k = 1; k < 200; ++k) {
            const double scanTime = 1.0 + dt * static_cast<double>(k) / 200.0;
            const double arrival = scanTime + tauMax;
            // First control tick at or after the arrival.
            const double served =
                dt * std::ceil(arrival / dt - 1.0e-12);

            truck_model::ArticulationEstimatorConfig cfg;
            cfg.historyHorizon = window;
            truck_model::ArticulationEstimator ekf(p, cfg);
            ekf.reset(0.0, 0.0);
            for (int step = 1; dt * step <= served + 1.0e-12; ++step) {
                truck_model::ArticulationInputs u;
                u.time = dt * step;
                u.truckYawRate = 0.05;
                u.speed = 12.0;
                ekf.predict(u);
            }
            truck_model::ArticulationLidarMeasurement z;
            z.stamp = scanTime;
            z.articulation = 0.03;
            const auto r = ekf.updateLidar(z);
            if (r.outcome != truck_model::MeasurementOutcome::accepted) {
                ++rejected;
                if (served - scanTime > worstAge) {
                    worstAge = served - scanTime;
                }
            }
        }
        std::printf("window = tau_max + %.1f*dt = %.3f s -> %d rejected",
                    extra, window, rejected);
        if (rejected) {
            std::printf("  (worst rejected age %.4f s)", worstAge);
        }
        std::printf("\n");
    }
    return 0;
}
