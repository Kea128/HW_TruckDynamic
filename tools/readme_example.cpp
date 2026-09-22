// Compiles the two code samples from README.md verbatim, so a stale API in the
// front page fails the build instead of wasting a porting engineer's morning.
#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/articulation_estimator.hpp"
#include "truck_model/lateral_mpc.hpp"
#include "truck_model/linear_discretization.hpp"

#include <cstdio>

namespace {

truck_model::Parameters demoParameters() {
    truck_model::Parameters p;
    p.m1 = 8000; p.iz1 = 25000; p.a1 = 1.5; p.b1 = 2.5;
    p.c1f = 220000; p.c1r = 300000;
    p.m2 = 18000; p.iz2 = 140000; p.a2 = 4; p.b2 = 3; p.c2r = 500000;
    p.d1 = p.b1;   // 铰接点在后轴，(K5)->(K6)
    p.vx = 15;
    return p;
}

// README section "动力学与 MPC"
void dynamicsAndMpc() {
    const auto p = demoParameters();
    const auto plant = truck_model::buildDynamicModel(p);   // 4 状态 K27
    const auto error = truck_model::buildErrorModel(p);     // 6 状态 MPC
    truck_model::LateralMpc mpc(error);

    truck_model::Vector<6> xc{};
    std::vector<truck_model::CurvatureSample> curvaturePreview(
        mpc.config().horizon);
    const auto step = mpc.update(xc, curvaturePreview);     // 转角 rad
    std::printf("mpc steering = %.6f rad\n", step.steering);
    (void)plant;
}

// README section "Delayed EKF（实车对接）"
void delayedEkf() {
    const auto p = demoParameters();
    const double sigma = 0.0087;

    truck_model::ArticulationEstimatorConfig cfg;
    // >= 可见最大扫描年龄 + 2 个控制周期（融合文档 9.6）
    cfg.historyHorizon = 0.55;
    cfg.measurementVariance = sigma * sigma;   // 雷达噪声 rad^2
    cfg.processModel = truck_model::ArticulationProcessModel::kinematic;
    truck_model::ArticulationEstimator ekf(p, cfg);

    const double t0 = 0.0;
    const double phi0 = 0.0;
    ekf.reset(t0, phi0);

    // 每个 IMU/控制拍（>=20 Hz）
    for (int k = 1; k <= 20; ++k) {
        truck_model::ArticulationInputs u;
        u.time = 0.05 * k;
        u.truckYawRate = 0.05;
        u.speed = 15.0;
        u.steering = 0.02;
        ekf.predict(u);

        // 雷达到达时：stamp 必须是扫描时刻，不是到达时刻
        if (k % 2 == 0) {
            truck_model::ArticulationLidarMeasurement z;
            z.stamp = u.time - 0.1;
            z.articulation = 0.01;  // rad
            ekf.updateLidar(z);
        }
    }

    const auto& e = ekf.estimate();
    // e.articulation, e.articulationRate -> 写入 MPC 的 xc[4], xc[5]
    std::printf("phi = %.6f rad, phiDot = %.6f rad/s\n",
                e.articulation, e.articulationRate);
}

}  // namespace

int main() {
    dynamicsAndMpc();
    delayedEkf();
    std::printf("README examples compile and run\n");
    return 0;
}
