# 位姿图_Dictionary

覆盖模块：`pose_graph_opt`（Ceres 位姿图优化）、`pose_graph_tool`（连通分组与 MST 全局系传播）。  
路径：`pose_graph/pose_graph_opt/`、`pose_graph/pose_graph_tool/`。  
**产物约定**：二者均为 **SHARED**（仓库默认动态库）；互不链接，由上层按需分别消费。

与通用图算法 `graph`（MST/连通分量）互补：`pose_graph_tool` 消费 `graph`；`pose_graph_opt` 做 SE(3) 非线性位姿图优化（PGO）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| pose_graph_opt | logging | 【日志管理器】(LogManager) | 仓库内 | 校验失败 / 求解失败日志 |
| pose_graph_opt | registration_type | 【配准边结果】(AlignResult::information_) | 仓库内 | 从边表装入 Ω / T_rel |
| pose_graph_opt | Eigen3 / Ceres | Matrix4d / Ceres::Problem | 外部 | SE(3) 与非线性最小二乘 |
| pose_graph_tool | common | 【站名自然序比较】(naturalNameLess) | 仓库内 | 参考原点站选择 |
| pose_graph_tool | logging | 【日志管理器】(LogManager) | 仓库内 | MST / 连通失败日志 |
| pose_graph_tool | registration_type | 【测站帧】/【配准边结果】 | 仓库内 | 构图输入 |
| pose_graph_tool | graph | 【最小生成树】(MinimumSpanningTree) | 仓库内 | Kruskal MST 与连通分组 |
| pose_graph_tool | Eigen3 | Matrix4d | 外部 | 全局位姿容器 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 Session） | pose_graph_opt | 【位姿图优化器】(PoseGraphOptimizer) | 仓库内 | 业务挂接后回填 |
| （待上层 Session） | pose_graph_tool | 【全局系传播】(PoseGlobalSystem) / 【位姿图分组】(PoseGraphGrouper) | 仓库内 | 业务挂接后回填 |

---

## 模块：pose_graph_opt

依赖摘要：`logging`、`registration_type`、`Eigen3::Eigen`、`Ceres::ceres`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 位姿图类型 | PoseGraphNode / PoseGraphEdge / PoseGraphSummary / PoseGraphEdgeReport | pose_graph_types.hpp | 公共 | 节点/边/求解摘要/边残差报告 |
| 2 | SE(3) 李代数工具 | se3Log / se3Exp / matrixToQt / qtToMatrix / infoCholL | se3_math.hpp/.cpp | 公共 | 右扰动 twist、四元数布局、Ω Cholesky |
| 3 | 位姿图求解器接口 | IPoseGraphSolver | pose_graph_solver.hpp | 公共 | 可替换后端抽象 |
| 4 | Ceres 位姿图求解器 | CeresPoseGraphSolver | ceres_pose_graph_solver.hpp/.cpp | 公共 | 默认 Ceres QuaternionManifold 后端 |
| 5 | 位姿图优化器 | PoseGraphOptimizer | pose_graph_optimizer.hpp/.cpp | 公共 | 建图、连通/规范校验、调用求解器 |
| 6 | 配准边装入位姿图 | loadFromAlignResults / isotropicInformationFromAlign | align_result_loader.hpp/.cpp | 公共 | AlignResult → 节点/边；Ω 取 information_ |

### 文件内小算法条目

#### 5、位姿图优化器（PoseGraphOptimizer）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 7 | 添加节点/边 | addNode / addEdge | 公共 | 插入初值与相对边（含 Ω 校验） |
| 8 | 规范固定站 | setFixedNode | 公共 | 钉死 gauge；未设则自然序最小站名 |
| 9 | Huber 损失开关 | enableHuberLoss / disableLoss | 公共 | 鲁棒核开关 |
| 10 | 优化求解 | optimize | 公共 | 校验连通+gauge 后求解；失败保留初值 |
| 11 | 边残差报告 | getEdgeReports | 公共 | 当前位姿相对 T_rel 的 ρ/ϕ/χ² |

#### 6、配准边装入位姿图

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 12 | 从配准边装载 | loadFromAlignResults | 公共 | 仅 VALID 边；Ω=AlignResult::information_ |
| 13 | 各向同性信息矩阵 | isotropicInformationFromAlign | 公共 | λI 辅助（保留；装载路径不用） |

---

## 模块：pose_graph_tool

依赖摘要：`common`、`logging`、`registration_type`、`graph`、`Eigen3::Eigen`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 14 | 位姿图基类 | PoseBase | pose_base.hpp/.cpp | 公共 | 帧/边装入与无向构图 |
| 15 | 全局系传播 | PoseGlobalSystem | pose_global_system.hpp/.cpp | 公共 | MST + DFS 将相对边合成全局阵 |
| 16 | 位姿图分组 | PoseGraphGrouper | pose_graph_grouper.hpp/.cpp | 公共 | 按连通分量拆分子图 |

### 文件内小算法条目

#### 14、位姿图基类（PoseBase）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 17 | 初始化索引 | PoseBase::initialization | 公共 | 建站名/边键映射 |
| 18 | 构建位姿图 | PoseBase::_buildPoseGraph | 私有 | LidarFrame+AlignResult → GraphNode/Edge |

#### 15、全局系传播（PoseGlobalSystem）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 19 | 最小生成树传播 | PoseGlobalSystem::process | 公共 | Kruskal MST 后以自然序首站为原点 DFS |

#### 16、位姿图分组（PoseGraphGrouper）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 20 | 连通分量分组 | PoseGraphGrouper::process | 公共 | 无向连通分量 → 站名分组 |
| 21 | 提取分组帧边 | PoseGraphGrouper::getGroupInfos | 公共 | 按组导出 LidarFrame / AlignResult 子集 |
