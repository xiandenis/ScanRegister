# 深度图工具_Dictionary

覆盖模块：`depth_map_tool`（全景深度图生成与格式转换）。  
**产物约定**：`depth_map_tool` 为 **STATIC**（模块 CMake 约定）。

与 `fastreg_filter`（深度差分统计）、计划中的 `DepthMapEstimator`（估计/缓存）区分：本模块提供点云→全景深度投影，以及 float/uchar 深度图格式转换与水平环带扩展。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| depth_map_tool | common | 【全景坐标工具】(PanoramaUtil::math_pixel_from_xyz) | 仓库内 | XYZ→全景像素 |
| depth_map_tool | OpenCV | opencv_world / cv::Mat | 外部 | 深度图缓冲与类型 |
| depth_map_tool | PCL | pcl::PointCloud\<PointXYZ\> | 外部 | 输入点云 |
| depth_map_tool | TBB / Eigen3 | tbb::parallel_for / Matrix4d | 外部 | 并行投影与刚体变换 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| registration_initializer | depth_map_tool | 【深度图工具】(DepthMapTool) | 仓库内 | Root 单线程深度图路径 |
| （待上层 / FastReg 流水线） | depth_map_tool | 【深度图工具】(DepthMapTool) | 仓库内 | 可与 fastreg_filter 衔接，无硬链接 |

---

## 模块：depth_map_tool

依赖摘要：`common`、`opencv_world`、`pcl_common`、`TBB`、`Eigen3`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 深度图工具 | DepthMapTool | depth_map_tool.hpp/.cpp | 公共 | 全景深度生成与深度图格式转换 |

### 文件内小算法条目

#### 1、深度图工具（DepthMapTool）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 2 | float↔4uchar 深度编码 | convert_float_to_4uchar / convert_4uchar_to_float | 公共 | `CV_32FC1` 与 `CV_8UC4` 按位型转换（PNG 落盘） |
| 3 | 32F 转 16U 深度量化 | convert_32FC1_to_16UC1 | 公共 | 按 scale 量化，截断上限 |
| 4 | 深度图水平环带扩展 | image_float_extend / image_float_extend_restore | 公共 | 全景水平周期扩展与还原 |
| 5 | 深度分段伪彩 uchar | image_value_segmentd_to_uchar | 公共 | `[min,max]` 线性映射到 `CV_8UC1` |
| 6 | 点云投影全景深度（串行） | depth_create | 公共 | 最近深度填像素；可选刚体变换 |
| 7 | 点云投影全景深度（TBB） | depth_create_tbb | 公共 | 线程局部深度图再归约取最小 |
