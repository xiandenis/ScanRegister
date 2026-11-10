# 分辨率估计_Dictionary

覆盖模块：`resolution_estimate`（径向深度直方图体素、分位数场景尺度体素、曲率有界调制）。  
**产物约定**：`resolution_estimate` 为 **SHARED**（默认动态库）。

深度按点到站心（原点）的欧氏范数；站心系 LAS 经 `FramedCloud` 读入后，估计前须 `local + origin` 折回文件站心坐标。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| resolution_estimate | logging | LOG_INFO | 仓库内 | 直方图 / 分位数结果日志 |
| resolution_estimate | PCL / Eigen3 | PointCloud / Vector3f | 外部 | 点云与径向深度 |
| resolution_estimate_test | point_cloud_io | 【FramedCloud】(load) | 仓库内 | 读盘局部化后折回站心再估 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层初始化 / 粗配准） | resolution_estimate | 【分辨率估计】/【分位数分辨率估计】 | 仓库内 | 业务接入后回填 |

---

## 模块：resolution_estimate

依赖摘要：`logging`、`pcl_common`、`Eigen3`、OpenMP（可选）。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 深度直方图 | Histogram | histogram.hpp/.cpp | 公共 | 按径向深度分箱统计频次 |
| 2 | 直方图分辨率估计 | ResolutionEstimate | resolution_estimate.hpp/.cpp | 公共 | 众数箱 → 离散体素档位 |
| 3 | 分位数分辨率估计 | PercentileResolutionEstimate | percentile_resolution_estimate.hpp/.cpp | 公共 | P50/P75/P90 场景尺度 + 曲率调制 |

### 文件内小算法条目

#### 1、深度直方图（Histogram）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 4 | 添加深度分箱 | addBinsRanges / addCloud | 公共 | 配置距离区间并对点云累加频次 |

#### 2、分位数分辨率估计（PercentileResolutionEstimate）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 分位数分辨率参数 | PercentileResolutionParams | 公共 | 权重、截断、量化、曲率调制开关 |
| 6 | 分位数分辨率结果 | PercentileResolutionResult | 公共 | r50/r75/r90、体素与场景类 |
| 7 | 体素场景类 | VoxelSceneClass | 公共 | Unknown / IndoorShrink / OutdoorExcess / Neutral |
| 8 | 室内分曲率调制 | kIndoorScore | 公共 | 平面占比 + log-κ 标准差有界缩放 |
| 9 | 非平面超额曲率调制 | kNonplanarExcess | 公共 | 超额放大 / 高平面缩小 / 中间带不动 |
