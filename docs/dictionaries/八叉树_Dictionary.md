# 八叉树_Dictionary

覆盖模块：`octree`（哈希体素 Medoid / FastOctree / 中心近邻八叉树）。  
**产物约定**：`octree` 为 **INTERFACE**（纯头文件）。

生产默认下采样走【体素真 Medoid 索引下采样】（`downsampleMedoidIndices`）；与点云+法向缓存配合时用索引同步抽取，见规则 `cached-normal-index-downsample`。  
与 `robust_rough` 内 matcher 私有体素下采样、`genz_icp` 局部体素哈希图无关。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| octree | PCL | pcl::PointCloud / copyPointCloud | 外部 | 点云与索引抽取 |
| octree | Eigen3 / TBB | Eigen / tbb::parallel_sort | 外部 | 几何与并行排序 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层初始化 / 配准） | octree | 【体素真 Medoid 索引下采样】等 | 仓库内 | 业务接入后回填 |

---

## 模块：octree

依赖摘要：`pcl_common`、`Eigen3`、`TBB`；无仓库内链接依赖。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 八叉树下采样入口 | Ddx::octree::*（octree_downsample） | octree_downsample.hpp | 公共 | Medoid/首点/OctreeMedoid 统一入口 |
| 2 | 哈希体素真 Medoid | downsampleHashMedoidIndices | voxel_hash_medoid.hpp | 公共 | 原点对齐整数体素格，每格真 Medoid |
| 3 | 快速八叉树 | FastOctree | fast_octree.h/.hpp | 公共 | 包围盒对齐八叉树；叶内 Medoid |
| 4 | 中心近邻八叉树 | CenterNearestOctree | center_nearest_octree.hpp | 公共 | 叶内保留距中心最近代表点；可扩展根 |

### 文件内小算法条目

#### 1、八叉树下采样入口（octree_downsample）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 体素真 Medoid 索引下采样 | downsampleMedoidIndices | 公共 | 生产默认；返回原点云索引 |
| 6 | 体素首点索引下采样 | downsampleFirstIndices | 公共 | 每格保留首点；求解前抽稀 |
| 7 | FastOctree 叶 Medoid 索引 | downsampleOctreeMedoidIndices | 公共 | 包围盒对齐叶；语义≠哈希格 |
| 8 | Medoid 点云抽取 | downsampleMedoid | 公共 | 索引 + copyPointCloud |

#### 2、哈希体素真 Medoid（downsampleHashMedoidIndices）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 9 | 哈希体素首点索引 | downsampleHashFirstIndices | 公共 | 单遍哈希，每格首点 |

#### 3、快速八叉树（FastOctree）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 10 | 构建 FastOctree | FastOctree::SetMatrixData | 公共 | 按 OctreeParams 建树 |
| 11 | 叶节点真 Medoid | FastOctree::getMedoidCenters | 公共 | 可多线程 |

#### 4、中心近邻八叉树（CenterNearestOctree）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 12 | 带属性采样点 | Sample | 公共 | 位置/法向/颜色/强度/时间戳 |
| 13 | 插入并保留中心近邻 | CenterNearestOctree::insert | 公共 | 叶内更近中心则替换；可扩根 |
