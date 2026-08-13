# TruckModel

卡车—半挂车铰接单轨模型的无第三方依赖 C++17 实现。

模型包括：

- 铰接点/后轴参考的精确非线性运动学；
- 线性轮胎横向动力学与铰接点速度约束；
- 通过 Schur 补消去铰接力；
- 最小物理状态 \(z=[v_{y1},r_1,r_2,\theta_{12}]^T\)；
- 控制误差状态
  \(x_c=[e_y,\dot e_y,e_\Phi,\dot e_\Phi,\theta_{12},
  \dot\theta_{12}]^T\)；
- 路径曲率 \(\kappa\) 和曲率变化率 \(\dot\kappa\) 扰动输入。

完整推导见
[`docs/articulated_vehicle_model_zh-CN.md`](docs/articulated_vehicle_model_zh-CN.md)。

## 构建

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## 最小用法

```cpp
#include "truck_model/articulated_vehicle.hpp"

truck_model::Parameters p;
// 设置 m1, iz1, a1, b1, c1f, c1r, m2, iz2, a2, b2, c2r, d1, vx
p.d1 = p.b1;

const auto physical = truck_model::buildDynamicModel(p);
const auto control = truck_model::buildErrorModel(p);
```

矩阵均按连续时间返回。调用者应根据控制周期采用零阶保持离散化。
