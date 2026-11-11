# FastReg滤波_Dictionary

覆盖模块：`fastreg_filter`（深度图差分滤波：异常率 / 重叠率 / 深度有效率）。  
**产物约定**：`fastreg_filter` 为 **SHARED**（仓库默认）。

与点云体素下采样、全景深度图估计（`DepthMapEstimator`）、以及 `depth_map_tool`（深度生成/格式转换）无关：本模块只对已对齐的深度图做邻域差分统计。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| fastreg_filter | logging | 【日志管理器】(LogManager) | 仓库内 | verbose 差分统计日志 |
| fastreg_filter | OpenCV | opencv_world / cv::Mat | 外部 | 深度图与形态学 |
| fastreg_filter | Eigen3 | Eigen::Matrix | 外部 | 邻域核偏移 |
| fastreg_filter | OpenMP | OpenMP::OpenMP_CXX | 外部 | 像素并行差分 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 FastReg / Session） | fastreg_filter | 【全景深度差分滤波】/【空间透视差分滤波】 | 仓库内 | 业务接入后回填 |

---

## 模块：fastreg_filter

依赖摘要：`logging`、`opencv_world`、`Eigen3`、OpenMP。输入为同尺寸深度 `cv::Mat`；输出异常率 / 重叠率 / 深度有效率，可选写出二值差分图。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 全景深度差分滤波 | PanoramaFilter | panorama_filter.hpp/.cpp | 公共 | `CV_16UC1` 全景深度邻域差分，估异常/重叠率 |
| 2 | 空间透视差分滤波 | SpatialPerspectiveFilter | spatial_perspective_filter.hpp/.cpp | 公共 | `CV_32FC1` 透视深度邻域差分，相对+绝对阈值 |

### 文件内小算法条目

#### 1、全景深度差分滤波（PanoramaFilter）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 3 | 全景深度差分 | PanoramaFilter::difference | 公共 | 3×3 邻域最小差 + 角向容差；形态学开运算后统计 |
| 4 | 全景输入尺寸校验 | PanoramaFilter::checkSize | 私有 | 要求同尺寸 `CV_16UC1` |

#### 2、空间透视差分滤波（SpatialPerspectiveFilter）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 空间透视深度差分 | SpatialPerspectiveFilter::difference | 公共 | 5×5 邻域；`max(threshold, depth*tolerance)`；形态学开运算 |
| 6 | 透视输入尺寸校验 | SpatialPerspectiveFilter::checkSize | 私有 | 要求同尺寸 `CV_32FC1` |
