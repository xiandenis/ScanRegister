# 配准初始化_Dictionary

覆盖模块：`registration_initializer`（测站帧初始化：Root 角栅格筛选、Medoid 子体素、法向、分辨率估计）。  
**产物约定**：`registration_initializer` 为 **SHARED**。

读盘走 `FramedCloud`；站心系处理前须 `local + origin` 折回文件坐标（径向深度 / 角度栅格）。  
`AngleDownsample` 暂内置于本模块；完整 `cloud_sample` 迁入后可改回复用。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| registration_initializer | registration_type | LidarFrame / VoxelSceneClass | 仓库内 | 测站帧与场景类写回 |
| registration_initializer | point_cloud_io | 【FramedCloud】(load) | 仓库内 | 读盘局部化后折回站心 |
| registration_initializer | octree | 【体素真 Medoid 索引下采样】 | 仓库内 | 子体素索引下采样 |
| registration_initializer | resolution_estimate | 【分位数分辨率估计】 | 仓库内 | 主配准体素边长 |
| registration_initializer | depth_map_tool / common | DepthMapTool / PanoramaUtil | 仓库内 | Root 单线程深度图路径 |
| registration_initializer | small_gicp | KdTree TBB 法向 | 仓库内 | 默认法向后端 |
| registration_initializer | logging / file_system | TicToc / MemPeak / stlplus | 仓库内 | 耗时内存与路径检查 |
| registration_initializer | PCL / OpenCV / TBB / Eigen | PCD IO / Mat / parallel | 外部 | 点云与并行 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 Session / StationInitialize） | registration_initializer | 【配准初始化器】 | 仓库内 | 业务接入后回填 |

---

## 模块：registration_initializer

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 配准初始化器 | RegistrationInitializer | registration_initializer.hpp/.cpp | 公共 | initalState / initializd / check 测站缓存管线 |
| 2 | KNN 法向估计 | estimateNormalsKnn | normal_estimate.hpp/.cpp | 公共 | SmallGicpTbb / PclOmp |
| 3 | 角度栅格赢家下采样 | AngleDownsample | angle_downsample.hpp/.cpp | 公共 | 全景像素最近点赢家（站心径向） |

### 文件内小算法条目

#### 1、配准初始化器（RegistrationInitializer）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 4 | 初始化状态骨架 | initalState | 公共 | 写 Root/子体素元数据 JSON |
| 5 | 执行初始化计算 | initializd | 公共 | 读盘→Root→Medoid+法向→分辨率 |
| 6 | Root 角深度筛选 | _calculateRoot | 私有 | AngleDownsample 或深度图路径 |
| 7 | 子体素 Medoid+法向 | _cacalVoxel | 私有 | 索引下采样同步抽法向 |
| 8 | 分辨率写回 | _resolution | 私有 | 压缩比早退或分位数+曲率调制 |
