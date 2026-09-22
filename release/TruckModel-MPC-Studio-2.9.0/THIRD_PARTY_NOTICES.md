# Third-party notices

The optional Windows MPC Studio target uses:

- Dear ImGui v1.92.9 — Copyright (c) 2014–2026 Omar Cornut and contributors,
  MIT License, https://github.com/ocornut/imgui
- ImPlot v0.17 — Copyright (c) 2020–2026 Evan Pezent and contributors,
  MIT License, https://github.com/epezent/implot

These dependencies are fetched only when `TRUCK_MODEL_BUILD_DEMO=ON`. The
`truck_model` core library remains independent of UI libraries.
