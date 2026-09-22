# TruckModel MPC Studio 2.9.0

铰接角融合重做为 **Delayed EKF v2**，并补齐车辆模型文档的适用边界。
移植规范仍以 `docs/3_articulation_fusion_filter.md` 为准。

## Delayed EKF v2

- **事件化 OOSM**：迟到扫描在其自身时间戳处切分传播区间，回退后重放输入**与量测**。
  后验只取决于事件集合，与到达顺序无关。v1 只重放输入，乱序旧包会抹掉其后已融合的校正。
- **精确对齐**：不再吸附到最近历史帧，去掉最多半个输入周期的对齐误差。
- **噪声契约**：过程噪声统一为连续功率谱密度，用 Van Loan 精确离散，
  补回 v1 丢失的 `phi`/`b_r2` 交叉项与 `T^3` 项。v1 的 `rad^2/s^2` 单位标注是错的。
- **可观状态**：三状态在定常工况可观秩仅 2。雷达偏置默认冻结为离线标定常数，
  在线自由度退化为结构可观的 `[phi, b_r2]`；`estimateLidarBias` 可恢复旧行为。
- **可选动态过程模型**：以实测 `r1` 为输入的降阶 (K27)，四状态
  `[v_y1, r2, phi, b_phi]`，针对 `phi_dot` 幅值偏瘦。默认仍是运动学 (K5)。
- **边界处理**：拒绝未来时间戳与重复包；`historyHorizon` 默认提到 0.55 s，
  且配置期强制不小于 `lidarDelayMax`。
- **诊断拆分**：`informationAge`（驱动过程噪声倍率）与 `arrivalGap`（链路健康）分开，
  修掉 v1 在长时延下的 coasting 误报。

## Studio 与记录

- 主/影子估计器并行，影子不进控制回路，可在同一轨迹上无风险对比两种过程模型。
- 雷达按**到达时间**投递而非队列顺序，真实触发乱序；v1 的队头阻塞掩盖了这条路径。
- 可注入横摆率/车速噪声与偏置、雷达安装偏差；默认不再用 Plant 真值初始化滤波器。
- `timeseries.csv` 新增逐包计数、影子列、传感器读数列与时序诊断列。
  **接受率必须用 `lidar_*_count`**：v1 每拍只保留最后一个包的诊断。

## 文档与构建

- `docs/1`、`docs/2` 新增适用边界：`e_x` 不是不变流形、纵向铰接力的阶次条件、
  变速工况的遗漏项、曲率率是时间导数（`dRho/dt = U * dKappa/ds`）。
- `docs/3` 修正 K5 投影推导的符号错误（v1 的中间式与其结论不自洽）。
- 新增 `TRUCK_MODEL_BUILD_STUDIO=OFF`，可在无 DirectX、无网络的工具链上构建并运行全部测试。

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
