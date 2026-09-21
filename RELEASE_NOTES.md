# TruckModel MPC Studio 2.8.1

- 聚焦图表后，2×2 曲线铺满变高的单元格，当前时刻圆点随图高缩放。
- 切换聚焦/平衡布局时自动适应轨迹，铰接点圆随地图比例缩放。

# TruckModel MPC Studio 2.8.0

横向 MPC Studio、路径/铰接角参考，以及 10 Hz、100–400 ms 时延雷达铰接角 Delayed EKF。

## Delayed EKF

- 过程模型为 (K5)，状态 \(x=[\phi,b_{r2},b_\phi]^\top\)；量测为带时延的雷达 \(\phi\)。
- 历史重传播补偿可变时延；马氏门限拒野值；路径跟踪下 100–400 ms 时延 RMSE 约 1.7°–1.8°。
- **接到其他工作区时，必须扩充目标仓已有 EKF 类**（`predict` / `update`），不要拷贝第二套卡尔曼核。方案、接口与验收见 `docs/3_articulation_fusion_filter.md`。

## Studio

- 六状态误差 MPC，Plant 仍为 K27；曲率自适应车速、手绘参考线。
- 可设 \(\phi_{\mathrm{ref}}(t)\)（正弦 / 阶跃 / 手绘）及铰接角跟踪实验。
- 可注入 10 Hz、100–500 ms 雷达时延与噪声，遥测叠加 Plant、雷达、估计。
- 运行可导出 `runs/<timestamp>/`（settings、时序、预测域、路径）；结束或故障时自动导出。

## 库

- C++17，核心无第三方依赖；Windows 下可选构建 `truck_mpc_demo`（ImGui / ImPlot）。
