# 公共基础_Dictionary

覆盖模块：`common`、`file_system`、`logging`、`graph`。  
`common` / `file_system` / `logging` 默认 **SHARED**；`graph` 为 **INTERFACE**（纯头文件，无翻译单元）。  
`graph` 与 `robust_rough/robin` 的【ROBIN 图核心】无关（后者是配准外点兼容图）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| common | logging | 【日志管理器】(LogManager) | 仓库内 | 调度/工具写日志 |
| common | Eigen3 | Eigen | 外部 | 位姿与坐标 |
| logging | spdlog | spdlog | 外部 | 日志实现 |
| logging | Threads | Threads::Threads | 外部 | 线程与采样 |
| file_system | （无） | — | — | 无仓库内依赖 |
| graph | （无） | — | — | 纯标准库；无 Boost / 无仓库内依赖 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_global_reg | common / logging / file_system | Common / LogManager | 仓库内 | 粗配准内存入口 |
| point_cloud_io | logging | LogManager | 仓库内 | 公共点云 IO（见点云 IO 字典） |
| fastreg_filter | logging | LogManager | 仓库内 | 深度差分 verbose 日志（见 FastReg 滤波字典） |
| global_matrix_file | common / logging | Common 串编解码 / LogManager | 仓库内 | 全局位姿 JSON（见全局位姿文件字典） |
| depth_map_tool | common | 【全景坐标工具】(PanoramaUtil) | 仓库内 | 点云投影全景深度（见深度图工具字典） |
| scan_register（伞目标） | graph | 【最小生成树】/【连通分量】/【最短路径】 | 仓库内 | 基础图算法随伞目标导出 |
| pose_graph_tool | graph | 【最小生成树】(MinimumSpanningTree) | 仓库内 | MST 全局系传播 / 连通分组（见位姿图字典） |
| pose_graph_tool | common | 【站名自然序比较】(naturalNameLess) | 仓库内 | 参考原点站选择 |
| pose_graph_opt / pose_graph_tool | logging | 【日志管理器】(LogManager) | 仓库内 | PGO / 构图诊断日志 |
| registration_type | common / logging | Common 串编解码 / LogManager | 仓库内 | 配准类型与 JSON IO（见配准类型字典） |
| （待上层业务） | common / logging / file_system / graph | — | 仓库内 | 业务模块迁入后回填 |

---

## 模块：common

依赖摘要：链接 `logging`、`Eigen3::Eigen`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 公共工具集 | Common | common.hpp/.cpp | 公共 | 位姿串、边键、采样与格式化等通用工具 |
| 2 | 数学常量定义 | defines | defines.hpp/.cpp | 公共 | π/WGS84 等数学与地理常量 |
| 3 | 三维刚体变换工具 | Transformations3D | transformations_3d.hpp/.cpp | 公共 | 插值、SLERP、大地/ECEF/ENU 互转 |
| 4 | 全景坐标工具 | PanoramaUtil | panorama_util.hpp/.cpp | 公共 | XYZ↔球面↔纹理↔像素 |
| 5 | E57 扫面几何 | E57ScanGeometry | e57_scan_geometry.hpp/.cpp | 公共 | 码盘角↔E57↔XYZ↔全景像素 |
| 6 | 三维随机工具 | Ddx::random | random_3d.hpp/.cpp | 公共 | 全局随机引擎与采样辅助 |
| 7 | 内存代价模型 | MemCostModel | mem_cost_model.hpp/.cpp | 公共 | 峰值内存回归估计与磁盘持久化 |
| 8 | 内存感知调度器 | MemAwareScheduler | mem_aware_scheduler.hpp/.cpp | 公共 | 按内存水位与代价模型调度任务 |
| 9 | 局部工作坐标系 | WorkingFrame | working_frame.hpp | 公共 | 作业级原点 O 换系工具；IO 产出的可恢复原点在 FramedCloud.origin（见点云 IO 字典） |
| 10 | 站名自然序比较 | naturalNameLess | natural_name.hpp | 公共 | 数字段按数值、其余按字节序比较站名 |

### 文件内小算法条目

#### 1、公共工具集（Common）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 11 | 位姿矩阵转七元组 | convert_matrix_to_vec7 | 公共 | 4×4 → 平移+四元数向量 |
| 12 | 七元组转位姿矩阵 | convert_vec7_to_matrix | 公共 | 七元组 → 4×4 |
| 13 | 南向位姿字符串编解码 | matrix_to_south_string / south_string_to_matrix | 公共 | 与南向文本格式互转 |
| 14 | 无向边键编码 | encode_edge_key | 公共 | 两端自然序排成 min\|max |

#### 5、E57 扫面几何（E57ScanGeometry）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 15 | 码盘角转 E57 | encoder_to_e57 | 公共 | θ₁/φ₁ → θ/φ |
| 16 | E57 转码盘（正常站） | e57_to_encoder_normal | 公共 | 正常站唯一解 |
| 17 | E57 与测站 XYZ 互转 | e57_to_xyz / xyz_to_e57 | 公共 | 球坐标与笛卡尔互转 |

#### 8、内存感知调度器（MemAwareScheduler）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 18 | 内存感知调度选项 | MemAwareSchedulerOptions | 公共 | 并发、水位、安全系数与持久化路径 |
| 19 | 调度任务描述 | MemAwareTask | 公共 | size + run 可调用对象 |

#### 9、局部工作坐标系（WorkingFrame）

作业级共用 `WorkingFrame` 与 IO 每云 `FramedCloud.origin` 分层：IO 必须产出可恢复 origin；作业 frame 由上层合成/设置（可 `WorkingFrame::fromOrigin(framed.origin)`）。**禁止**再依赖「裸 PointCloud + 减 LAS header offset」当工作系（已废弃缺陷）。

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 20 | 世界转局部 | WorkingFrame::toLocal / toLocalF | 公共 | p' = p − O |
| 21 | 局部转世界 | WorkingFrame::toWorld / toWorldFromF | 公共 | p = p' + O |

#### 2、数学常量定义（defines）

待补充（需结合业务确认）

#### 3、三维刚体变换工具（Transformations3D）

待补充（需结合业务确认）

#### 4、全景坐标工具（PanoramaUtil）

待补充（需结合业务确认）

#### 6、三维随机工具（Ddx::random）

待补充（需结合业务确认）

#### 7、内存代价模型（MemCostModel）

待补充（需结合业务确认）

#### 10、站名自然序比较（naturalNameLess）

待补充（需结合业务确认）

---

## 模块：file_system

依赖摘要：无仓库内链接依赖；可移植路径与通配。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 22 | 文件系统访问 | stlplus::file_system | file_system.hpp/.cpp | 公共 | 路径判定、读写、拷贝/移动、目录遍历 |
| 23 | 可移植性修补 | portability_fixes | portability_fixes.hpp/.cpp | 公共 | 跨平台文件系统相关修补 |
| 24 | 字符串工具 | util::strings | strings.hpp/.cpp | 公共 | 数值格式化与字符串辅助 |
| 25 | 分词器 | util::Tokenizer | tokenizer.hpp/.cpp | 公共 | 按分隔符切分字符串 |
| 26 | 通配符匹配 | stlplus::wildcard | wildcard.hpp/.cpp | 公共 | `*`/`?`/`[]` 风格匹配 |

### 文件内小算法条目

#### 22、文件系统访问（stlplus::file_system）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 27 | 路径存在判定 | is_present / is_folder / is_file | 公共 | 判定路径类型 |
| 28 | 文件读写属性 | file_exists / file_readable / file_writable | 公共 | 可读可写与存在性 |
| 29 | 路径拼接 | create_filespec / create_filename | 公共 | 目录与文件名组合 |

#### 23、可移植性修补（portability_fixes）

待补充（需结合业务确认）

#### 24、字符串工具（util::strings）

待补充（需结合业务确认）

#### 25、分词器（util::Tokenizer）

待补充（需结合业务确认）

#### 26、通配符匹配（stlplus::wildcard）

待补充（需结合业务确认）

---

## 模块：logging

依赖摘要：`scan_register_spdlog`、Threads；Windows 另链 Psapi。进程内单例，必须 SHARED。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 30 | 日志管理器 | LogManager | logging.hpp/.cpp | 公共 | 初始化/重置/按日滚动与级别控制 |
| 31 | 日志便捷初始化 | initiLog / resetLog | logging_initi.hpp/.cpp | 公共 | 对 LogManager 的薄封装入口 |
| 32 | 峰值内存采样作用域 | MemPeakScope | mem_peak.hpp/.cpp | 公共 | RAII 采样进程峰值内存并打日志 |
| 33 | 计时记录器 | TimeRecorder / TicToc | tic_toc.hpp | 公共 | 作用域计时与累计耗时 |

### 文件内小算法条目

#### 30、日志管理器（LogManager）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 34 | 普通文件日志初始化 | LogManager::init | 公共 | 指定路径与级别初始化 |
| 35 | 按日滚动日志初始化 | LogManager::init_daily | 公共 | 按日切割日志文件 |
| 36 | 获取日志器 | LogManager::get_logger | 公共 | 取 spdlog logger 单例引用 |

#### 32、峰值内存采样作用域（MemPeakScope）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 37 | 当前进程内存占用 | MemPeakScope::currentUsageMb | 公共 | 读 RSS/私有工作集 MB |

#### 31、日志便捷初始化（initiLog）

待补充（需结合业务确认）

#### 33、计时记录器（TimeRecorder / TicToc）

待补充（需结合业务确认）

---

## 模块：graph

依赖摘要：纯 C++17 标准库头文件；CMake 目标 `graph`（INTERFACE）。与 `robin::graph` / `teaser::Graph` 命名空间与用途均不同。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 38 | 图节点与边 | GraphNode / GraphEdge / NodeID | graph.hpp | 公共 | 带权无向图基本元素 |
| 39 | 最小生成树 | MinimumSpanningTree | graph.hpp | 公共 | Kruskal / Prim 求 MST |
| 40 | 连通分量子图 | ConnectedComponentsSubgraph | graph.hpp | 公共 | 按连通分量分组边或节点 |
| 41 | 最短路径 | ShortestPaths | graph.hpp | 公共 | Dijkstra 单源最短路径 |
| 42 | 图算法 std 别名（弃用） | Ddx::graph_std::* | graph_std.hpp | 忽略 | 兼容旧 `#include`；请改用 `graph.hpp` |

### 文件内小算法条目

#### 38、图节点与边（GraphNode / GraphEdge）

待补充（需结合业务确认）

#### 39、最小生成树（MinimumSpanningTree）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 43 | Kruskal 最小生成树 | kruskalRun / kruskalRun2 / kruskal_mst | 公共 | 按边权排序 + 并查集合并 |
| 44 | Prim 最小生成树 | primRun / prim_mst | 公共 | 优先队列扩展生成树（含多连通分量） |
| 45 | 并查集 | graph_detail::UnionFind | 私有 | 路径压缩 + 按秩合并 |

#### 40、连通分量子图（ConnectedComponentsSubgraph）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 46 | 连通分量边分组 | ConnectedComponentsSubgraph::run | 公共 | 同分量内边列表 |
| 47 | 连通分量节点分组 | ConnectedComponentsSubgraph::run2 | 公共 | 同分量内节点列表 |
| 48 | 连通分量标记 | connected_component_labels | 私有 | DFS/栈式分量标号 |

#### 41、最短路径（ShortestPaths）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 49 | Dijkstra 最短路径 | ShortestPaths::run / dijkstra_path | 公共 | 非负权最短节点序列（dst→src 逆序） |

#### 42、图算法 std 别名（弃用）（Ddx::graph_std）

待补充（需结合业务确认）
