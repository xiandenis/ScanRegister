# 鲁棒精配_Dictionary

大模块目录：`robust_refine/`（ICP 集合）。另有仓库根 **`trim_icp/`**（不并入 `robust_refine/`，根 CMake 单独 `add_subdirectory`）。  
覆盖目标：`basic_icp`、`fast_gicp`、`small_gicp`、`genz_icp`、`trim_icp`。

**产物约定（用户明确）**：`robust_refine/` 子目标全部按 **STATIC** 管理；其中 `fast_gicp` 无编译单元，以 **INTERFACE** 交付头文件。仓库根 `trim_icp` 按仓库默认 **SHARED**（依赖 `basic_icp`）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| basic_icp | Eigen3 | Eigen | 外部 | ICP / FRICP 数值 |
| fast_gicp | PCL / Eigen / OpenMP | pcl_registration 等 | 外部 | Fast-GICP / NDT 头实现 |
| small_gicp | Eigen3 / OpenMP | Eigen | 外部 | 轻量 GICP 辅助与核心头 |
| genz_icp | matcher_tsl | tsl::robin_map | 仓库内 | 体素哈希等 |
| genz_icp | Eigen3 / TBB / PCL | — | 外部 | GenZ-ICP 流水线 |
| trim_icp | basic_icp | 【基础 ICP 集合】(BasicICP) | 仓库内 | TrimIcpSolver 底层求解 |
| trim_icp | octree | 【哈希体素首点下采样】等 | 仓库内 | 求解前索引下采样 |
| trim_icp | PCL / Eigen / OpenMP | — | 外部 | 点云 / KdTree / 数值 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| registration_initializer | small_gicp | KdTree TBB 法向 | 仓库内 | 默认 NormalEstimateMethod::SmallGicpTbb |
| （待精配准上层） | basic_icp / small_gicp / fast_gicp / genz_icp / trim_icp | — | 仓库内 | Fine / GEO / GLC 等迁入后回填 |

---

## 模块：basic_icp

依赖摘要：STATIC；Eigen；OpenMP。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 基础 ICP 集合 | BasicICP / ICP / FRICP | basic_icp.h/.cpp、ICP.h、FRICP.h | 公共 | Point-to-Point / 鲁棒 / FRICP 等基础 ICP |
| 2 | Anderson 加速 | AndersonAcceleration | AndersonAcceleration.h | 私有 | 迭代加速 |
| 3 | 中值工具 | median | median.h | 私有 | 鲁棒统计辅助 |

### 文件内小算法条目

#### 1、基础 ICP 集合（BasicICP）

待补充（需结合业务确认）

#### 2、Anderson 加速（AndersonAcceleration）

待补充（需结合业务确认）

#### 3、中值工具（median）

待补充（需结合业务确认）

---

## 模块：fast_gicp

依赖摘要：INTERFACE（头文件）；PCL + Eigen + OpenMP。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 4 | Fast-GICP | fast_gicp::FastGICP 等 | gicp/*.hpp | 公共 | 多线程 GICP |
| 5 | Fast-VGICP / NDT | fast_gicp NDT/VGICP | ndt/*.hpp、gicp/* | 公共 | 体素 GICP / NDT 变体 |
| 6 | SO(3) 辅助 | so3 | so3/*.hpp | 私有 | 旋转李代数辅助 |

### 文件内小算法条目

#### 4、Fast-GICP（FastGICP）

待补充（需结合业务确认）

#### 5、Fast-VGICP / NDT

待补充（需结合业务确认）

#### 6、SO(3) 辅助（so3）

待补充（需结合业务确认）

---

## 模块：small_gicp

依赖摘要：STATIC；Eigen；OpenMP；大量头文件核心。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 7 | Small-GICP 配准辅助 | registration_helper | registration/registration_helper.cpp | 公共 | 轻量并行 GICP 入口辅助 |
| 8 | Small-GICP 配准核心 | registration | registration/registration.cpp | 公共 | 配准迭代实现 |
| 9 | Small-GICP 因子/ANN | factors / ann | factors/*、ann/* | 私有 | ICP/GICP 因子与近邻结构 |

### 文件内小算法条目

#### 7、Small-GICP 配准辅助（registration_helper）

待补充（需结合业务确认）

#### 8、Small-GICP 配准核心（registration）

待补充（需结合业务确认）

#### 9、Small-GICP 因子/ANN

待补充（需结合业务确认）

---

## 模块：genz_icp

依赖摘要：STATIC；matcher_tsl、Eigen、TBB、PCL；内嵌 Sophus。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 10 | GenZ-ICP 封装 | genz_icp API | genz_icp.hpp/.cpp | 公共 | 对本库的 GenZ-ICP 适配入口 |
| 11 | GenZ-ICP 流水线 | pipeline::GenZICP | pipeline/GenZICP.* | 公共 | 端到端配准流水线 |
| 12 | GenZ 配准核心 | core::Registration | core/Registration.* | 私有 | 核心配准迭代 |
| 13 | GenZ 体素哈希图 | VoxelHashMap | core/VoxelHashMap.* | 私有 | 局部地图 |
| 14 | GenZ 预处理/去畸变 | Preprocessing / Deskew | core/Preprocessing.*、Deskew.* | 私有 | 输入处理 |

### 文件内小算法条目

#### 10、GenZ-ICP 封装（genz_icp）

待补充（需结合业务确认）

#### 11、GenZ-ICP 流水线（GenZICP）

待补充（需结合业务确认）

#### 12、GenZ 配准核心（Registration）

待补充（需结合业务确认）

#### 13、GenZ 体素哈希图（VoxelHashMap）

待补充（需结合业务确认）

#### 14、GenZ 预处理/去畸变

待补充（需结合业务确认）

---

## 模块：trim_icp

目录：仓库根 `trim_icp/`（**不**在 `robust_refine/` 下）。  
依赖摘要：SHARED；`basic_icp`、`octree`、PCL、Eigen、OpenMP。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 15 | TrimICP 编排 | TrimICP | trim_icp.hpp/.cpp | 公共 | 重叠裁剪 → 可选 NC 预筛 → 下采样 → 求解 |
| 16 | TrimICP 求解器 | TrimIcpSolver | trim_icp_solver.hpp/.cpp | 公共 | 已裁工作子集上的 BasicICP 求解 |
| 17 | 哈希体素重叠互裁 | overlap_voxel_crop | overlap_voxel_crop.hpp/.cpp | 公共 | 源/目标互裁重叠体素邻域 |
| 18 | KNN 法向重叠预筛 | knn_normal_overlap_crop | knn_normal_overlap_crop.hpp/.cpp | 公共 | 双向 KNN + 法向点积过滤 |
| 19 | 重叠估计 / 配准报告 | OverlapEstimator / RegistrationReport | overlap_estimator.*、*surface_report.*、registration_report.* | 公共/私有 | 重叠度量与显式/隐式表面报告 |

### 文件内小算法条目

#### 15、TrimICP 编排（TrimICP）

流水线默认：体素互裁开、NC 预筛关、求解下采样 0.05 m、自动角色对调；Method 默认 RICP（仅 XYZ）。

#### 16、TrimICP 求解器（TrimIcpSolver）

假定输入已是工作子集；不做重叠裁剪 / 法向预筛。

#### 17–19

待补充（对照 bench / GEO 挂点时回填参数默认）。
