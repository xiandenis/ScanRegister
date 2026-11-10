# small_gicp（vendored）

高效、可并行的点云配准库，源自 [koide3/small_gicp](https://github.com/koide3/small_gicp)（MIT，Copyright 2024 Kenji Koide）。

本仓库仅保留算法核心：头文件 + `registration_helper` 实现；已去掉上游 CI、Python、ROS、benchmark、example、test 与文档站点。

## 用法

```cpp
#include <small_gicp/registration/registration_helper.hpp>
```

链接 CMake 目标 `small_gicp`（Eigen + OpenMP）。可选头：`pcl/`（需 PCL）、`*_tbb.hpp`（需 TBB）。

冒烟测试：`-DBUILD_SMALL_GICP_COMPILE_TEST=ON`。
