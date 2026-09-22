# TruckModel

卡车—半挂车铰接单轨模型与横向 MPC Studio。核心库是无第三方依赖的 C++17；
Windows 下额外构建实时可视化 `truck_mpc_demo`。

内部一律 SI：长度 m、质量 kg、角度 **rad**、角速度 rad/s、力 N、刚度 N/rad。
面板上的度只是显示换算。

| 模块 | 状态 | 作用 |
|---|---|---|
| 非线性运动学 (K5/K6) | 库 | 铰接点几何、\(\dot\phi\) |
| 线性 Plant (K27) | 库 / Demo 真值 | \(z=[v_{y1},r_1,r_2,\phi]^\top\) |
| 误差 MPC (K38) | 库 / Demo 控制 | \(x_c=[e_y,\dot e_y,e_\psi,\dot e_\psi,\phi,\dot\phi]^\top\) |
| Delayed EKF | 库 / Demo 可选 | 10 Hz、100–400 ms 时延雷达铰接角融合 |

## 文档

按这个顺序读即可上手；公式不要改符号。

| 文件 | 内容 |
|---|---|
| [`docs/1_truck_model_key_formula.md`](docs/1_truck_model_key_formula.md) | 几何、K5/K27/MPC 矩阵（实现对照） |
| [`docs/2_truck_model_derivation.md`](docs/2_truck_model_derivation.md) | 逐式推导 |
| [`docs/3_articulation_fusion_filter.md`](docs/3_articulation_fusion_filter.md) | **冻结的 Delayed EKF 方案、接口、调参、验收** |
| [`docs/articulated_vehicle_model_zh-CN.md`](docs/articulated_vehicle_model_zh-CN.md) | 模型长文 |
| [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) | Dear ImGui / ImPlot |

把融合滤波器接到其他工作区时，以第 3 份文档为准：在**已有 EKF 类上扩充**铰接角模型与时延重传播，不要另起滤波核或另起状态维。

## 环境

- Windows 10/11 或 Server，CMake ≥ 3.16，MSVC（C++17）
- 配置 Studio 时 CMake 会下载 ImGui v1.92.9、ImPlot v0.17（需能访问 GitHub）
- 只编核心库、不编界面：`-DTRUCK_MODEL_BUILD_DEMO=OFF`
- 编核心库与 Demo 会话、但跳过 ImGui 界面（无需 DirectX 或联网，可在 GCC/Clang 下跑全部测试）：
  `-DTRUCK_MODEL_BUILD_STUDIO=OFF`

## 构建与测试

在仓库根目录：

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

通过后应有 `truck_model_tests` 与 `truck_mpc_demo_tests`。可执行文件一般在：

```
build\Release\truck_mpc_demo.exe
```

单配置生成器则可能是 `build\truck_mpc_demo.exe`。

## 五分钟跑通 Studio

```powershell
cmake --build build --config Release --target truck_mpc_demo
.\build\Release\truck_mpc_demo.exe
```

| 操作 | 作用 |
|---|---|
| 运行 / 空格 | 开始或暂停 |
| 重置 / `R` | 回到初始状态（含 EKF 与雷达队列） |
| `Esc` | 退出 |
| 画布右键拖、滚轮 | 平移、缩放 |
| 绘制轨迹 | 按住左键画线 → 看绿色预览 → **确认轨迹** |

默认是 **S 形路径跟踪**，巡航 15 m/s，曲率自适应降速。过弯看左侧车体和右侧
\(e_y,e_\psi,\phi\) 曲线即可。

路径过短、曲率超过约 \(0.08\,\mathrm{m}^{-1}\)，或当前车速下转不过去，会被拒绝；
按提示降低巡航车速后再确认，不必重画。横向误差很大时只警告，不会停机；出现
NaN 才会安全停止。

### 打开铰接角雷达融合

左侧「铰接角雷达融合」：

1. 勾选 **启用 Delayed EKF**
2. 雷达周期 `0.1` s（10 Hz）
3. 时延：工程值 **最小 0.1 / 最大 0.4** s（面板允许最大 0.5 s）
4. 雷达噪声按感知标定填（仿真常用 3°–4°）
5. 勾选 **R 跟随雷达噪声**
6. **历史窗必须不小于最大时延**：默认已是 **0.55 s**，面板可调。
   设小于 `lidarDelayMax` 时会在应用参数时直接报错，不会静默丢包
7. 需要更好的 \(\dot\phi\) 时把「过程模型」切到 **动力学 K27r**；
   不确定时先开「影子估计器」并跑对比（见融合文档第 7、9 节）
8. 应用参数后 **重置**，再运行

MPC 吃的是当前 \(\hat\phi,\hat{\dot\phi}\)，不是迟到雷达。遥测里应同时看到
Plant、雷达、估计；估计应贴近 Plant，雷达明显滞后。

点 **导出本次记录**，或运行结束/故障时，写入 `runs/YYYYMMDD_HHMMSS/`：

- `settings.txt` 车型、MPC、雷达、EKF
- `timeseries.csv` 逐步 Plant / 控制 / 雷达 / EKF
- `mpc_horizon.csv`、`path.csv`

复盘列名与指标见融合文档第 10 节。接受率要用 `lidar_*_count` 逐包计数列，
不要用布尔列。该目录是运行产物，不要提交 git。

### 铰接角参考（可选）

「铰接角参考」可设正弦 / 周期阶跃 / 手绘 \(\phi_{\mathrm{ref}}(t)\)。
**铰接角跟踪实验**会把路径权重置零，让转角去跟 \(\phi_{\mathrm{ref}}\)。
这条闭环会放大 EKF 的幅值偏瘦（约 0.74），路径跟踪不受影响。不要用它当融合验收。

## 库用法

头文件在 `include/truck_model/`。核心库不依赖 ImGui。

### 动力学与 MPC

```cpp
#include "truck_model/articulated_vehicle.hpp"
#include "truck_model/linear_discretization.hpp"
#include "truck_model/lateral_mpc.hpp"

truck_model::Parameters p;
p.m1 = 8000; p.iz1 = 25000; p.a1 = 1.5; p.b1 = 2.5;
p.c1f = 220000; p.c1r = 300000;
p.m2 = 18000; p.iz2 = 140000; p.a2 = 4; p.b2 = 3; p.c2r = 500000;
p.d1 = p.b1;   // 铰接点在后轴，(K5)→(K6)
p.vx = 15;

const auto plant = truck_model::buildDynamicModel(p);   // 4 状态 K27
const auto error = truck_model::buildErrorModel(p);     // 6 状态 MPC
truck_model::LateralMpc mpc(error);
const auto step = mpc.update(xc, curvaturePreview);     // 转角 rad
```

`discretizeZeroOrderHold()` 可单独做 ZOH；`LateralMpc` 内部已使用同一实现。

### Delayed EKF（实车对接）

```cpp
#include "truck_model/articulation_estimator.hpp"

truck_model::ArticulationEstimatorConfig cfg;
cfg.historyHorizon = 0.55;                 // > 最大雷达时延
cfg.measurementVariance = sigma * sigma;   // 雷达噪声 rad²
truck_model::ArticulationEstimator ekf(p, cfg);
ekf.reset(t0, phi0);

// 每个 IMU/控制拍（≥20 Hz）
truck_model::ArticulationInputs u;
u.time = t; u.truckYawRate = r1_imu; u.speed = U; u.steering = delta;
ekf.predict(u);

// 雷达到达时：stamp 必须是扫描时刻，不是到达时刻
truck_model::ArticulationLidarMeasurement z;
z.stamp = t_scan;
z.articulation = phi_lidar;  // rad
ekf.updateLidar(z);

const auto& e = ekf.estimate();
// e.articulation, e.articulationRate  → 写入 MPC 的 xc[4], xc[5]
```

几何 \(a_2,b_2,b_1,d_1\) 必须与实车一致。\(R\) 必须与雷达噪声同量级。
完整公式、门限、coasting、移植清单：[`docs/3_articulation_fusion_filter.md`](docs/3_articulation_fusion_filter.md)。

## 默认演示车型

| 量 | 值 |
|---|---|
| \(m_1,I_1,a_1,b_1\) | 8000 kg，25000 kg·m²，1.5 m，2.5 m |
| \(C_{1f},C_{1r}\) | 2.2×10⁵，3.0×10⁵ N/rad |
| \(m_2,I_2,a_2,b_2\) | 18000 kg，1.4×10⁵ kg·m²，4 m，3 m |
| \(C_{2r},d_1,U\) | 5.0×10⁵ N/rad，\(d_1=b_1\)，15 m/s |
| MPC | \(\Delta t=0.05\) s，\(N=40\)，\(\lvert\delta\rvert\le 0.35\) rad |

## 源码地图

```
include/truck_model/     对外头文件（模型、EKF、MPC、参考线、离散化）
src/                     对应实现
examples/mpc_demo/       Studio：会话、雷达时延仿真、日志、ImGui
tests/                   模型单测 + Demo/EKF 单测
docs/                    公式、推导、融合交接
runs/                    本地导出，不入库
```

Studio 相关实现：`demo_session.*`（仿真与雷达队列）、`session_log.*`（导出）、
`demo_imgui.cpp` / `telemetry_panel.*`（界面）、`articulation_reference.*`
（\(\phi_{\mathrm{ref}}\)）。
