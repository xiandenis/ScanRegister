# 鲁棒粗匹配_Dictionary

大模块目录：`robust_rough/`。  
覆盖目标：`robust_matcher`、`robin`、`pmc`、`matcher_*`、`xenium`。

**产物约定（用户明确）**：`matcher_hnswlib`、`matcher_kdtree`、`matcher_points`、`matcher_tsl`、`pmc`、`robin`、`robust_matcher` 为 **STATIC**（其中 `matcher_*` / `xenium` 为 header-only **INTERFACE**）。`xenium` 为 `robin` 的 vendored 头依赖。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_matcher | robin | 【ROBIN 内点结构】(robin::FindInlierStructure) | 仓库内 | 图论外点剔除 |
| robust_matcher | matcher_points / matcher_kdtree / matcher_hnswlib / matcher_tsl | 点云/KD/HNSW/robin_map | 仓库内 | 特征与近邻检索 |
| robust_matcher | Eigen3 / TBB / FLANN | — | 外部 | 线性代数、并行、FLANN knn |
| robin | pmc | 【最大团】(pmc) | 仓库内 | 团求解 |
| robin | xenium | ramalhete_queue | 仓库内 | 无锁队列 |
| matcher_points | Eigen3 / TBB | — | 外部 | 点云与并行下采样 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待粗配准上层） | robust_matcher | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 粗配准迁入后回填 |

---

## 模块：robust_matcher

依赖摘要：STATIC；链 robin + matcher_* + Eigen/TBB/FLANN。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 鲁棒匹配器 | robust_matcher::RobustMatcher | RobustMatcher.hpp/.cpp | 公共 | FPFH + ROBIN + GNC 的鲁棒对应与位姿估计 |
| 2 | 加速 PFH 特征 | FasterPFH | FasterPFH.hpp/.cpp | 公共 | 体素/法向/FPFH 特征提取 |
| 3 | GNC 求解器 | GncSolver | GncSolver.hpp/.cpp | 公共 | Graduated Non-Convexity 位姿求解 |
| 4 | ROBIN 匹配（FLANN） | ROBINMatching | ROBINMatching.hpp/.cpp | 公共 | 特征 knn + ROBIN 图剔除 |
| 5 | ROBIN 匹配（HNSW） | ROBINMatchingHnsw | ROBINMatchingHnsw.hpp/.cpp | 公共 | hnswlib knn 路径 |
| 6 | 特征空间预算 | feature_spatial_budget | feature_spatial_budget.hpp | 公共 | 特征数软上限与空间分箱 |

### 文件内小算法条目

#### 1、鲁棒匹配器（RobustMatcher）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 7 | 鲁棒匹配配置 | RobustMatcherConfig | 公共 | 体素、半径、HNSW/ratio-test 等开关 |

#### 2、加速 PFH 特征（FasterPFH）

待补充（需结合业务确认）

#### 3、GNC 求解器（GncSolver）

待补充（需结合业务确认）

#### 4、ROBIN 匹配（FLANN）（ROBINMatching）

待补充（需结合业务确认）

#### 5、ROBIN 匹配（HNSW）（ROBINMatchingHnsw）

待补充（需结合业务确认）

#### 6、特征空间预算（feature_spatial_budget）

待补充（需结合业务确认）

---

## 模块：robin

依赖摘要：STATIC；链 pmc、xenium、Eigen。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 8 | ROBIN 内点选择 | robin::FindInlierStructure 等 | robin.hpp/.cpp | 公共 | 最大核/最大团等内点图结构 |
| 9 | ROBIN 图核心 | graph / graph_core / graph_solvers | graph*.hpp/.cpp | 私有 | 图构建与求解 |
| 10 | ROBIN 问题封装 | problems | problems.hpp/.cpp | 私有 | 向量/旋转平均等问题接口 |

### 文件内小算法条目

#### 8、ROBIN 内点选择（robin API）

待补充（需结合业务确认）

#### 9、ROBIN 图核心（graph）

待补充（需结合业务确认）

#### 10、ROBIN 问题封装（problems）

待补充（需结合业务确认）

---

## 模块：pmc

依赖摘要：STATIC；OpenMP；Windows 链 psapi。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 11 | PMC 最大团库 | pmc / pmc_lib | pmc_*.cpp/.h | 公共 | Parallel Maximum Clique 求解 |

### 文件内小算法条目

#### 11、PMC 最大团库（pmc）

待补充（需结合业务确认）

---

## 模块：matcher_points

依赖摘要：INTERFACE（头文件）；Eigen + TBB。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 12 | 匹配点云容器 | matcher_points::point_cloud | point_cloud.hpp | 公共 | 点云视图与特征缓冲 |
| 13 | 体素下采样 | downsampling | downsampling.hpp | 公共 | 匹配前点云抽稀 |

### 文件内小算法条目

#### 12、匹配点云容器（point_cloud）

待补充（需结合业务确认）

#### 13、体素下采样（downsampling）

待补充（需结合业务确认）

---

## 模块：matcher_kdtree

依赖摘要：INTERFACE；nanoflann / TBB 包装。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 14 | KD 树近邻 | kdtree / kdtree_tbb | kdtree.hpp / kdtree_tbb.hpp | 公共 | nanoflann 近邻查询 |

### 文件内小算法条目

#### 14、KD 树近邻（kdtree）

待补充（需结合业务确认）

---

## 模块：matcher_hnswlib

依赖摘要：INTERFACE；vendored hnswlib。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 15 | HNSW 近邻库 | hnswlib | hnswlib/hnswlib.h | 公共 | 近似近邻索引 |

### 文件内小算法条目

#### 15、HNSW 近邻库（hnswlib）

待补充（需结合业务确认）

---

## 模块：matcher_tsl

依赖摘要：INTERFACE；robin_map/set。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 16 | Robin 哈希表 | tsl::robin_map / robin_set | robin_map.h / robin_set.h | 公共 | 开放寻址哈希容器 |

### 文件内小算法条目

#### 16、Robin 哈希表（tsl）

待补充（需结合业务确认）

---

## 模块：xenium

依赖摘要：INTERFACE；vendored；被 robin 使用。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 17 | 无锁并发结构 | xenium::ramalhete_queue 等 | xenium/*.hpp | 忽略 | 无锁队列与回收器（对业务文档可忽略） |

### 文件内小算法条目

#### 17、无锁并发结构（xenium）

待补充（需结合业务确认）
