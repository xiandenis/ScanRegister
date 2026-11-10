# 点云算法总览（Hub）

ScanRegister（CMake 包 `scan_register`）跨模块互用速查。细则见 [`rules.md`](../rules.md)。

## 跨模块依赖（互用速查）

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| common | logging | 【日志管理器】(LogManager) | 仓库内 | 公共工具与调度输出日志 |
| logging | spdlog | spdlog::spdlog | 外部 | 日志后端 |
| common | Eigen3 | Eigen::Matrix / Vector | 外部 | 矩阵与坐标变换 |
| file_system | （无仓库内） | — | — | 独立文件系统工具 |
| graph | （无） | 【最小生成树】/【连通分量】/【最短路径】 | — | 纯 std 通用图算法（INTERFACE；≠ robin/teaser 图） |
| scan_register（伞） | graph | graph.hpp | 仓库内 | 基础图算法随伞目标导出 |
| scan_register（伞） | registration_type | 【测站帧】/【配准边结果】 | 仓库内 | party 面类型与 JSON IO |
| registration_type | common / logging / RapidJSON | ScanInfo / LidarFrameIO | 仓库内/外部 | 配准会话数据结构（SHARED） |
| resolution_estimate | registration_type | 【体素场景类】(VoxelSceneClass) | 仓库内 | 曲率调制场景提示权威定义 |
| point_cloud_io | logging / PCL / Eigen | 【FramedCloud / 点云读写】 | 仓库内/外部 | 公共 IO；局部云+可恢复 origin（无 PDAL） |
| 测试 / Session / 精配工具等 | point_cloud_io | 【FramedCloud】(load/save) | 仓库内 | 读盘局部化；配准前后对 M / M_init 做 origin 系变换 |
| robust_matcher | robin / matcher_* | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 鲁棒粗匹配主入口 |
| robin | pmc / xenium | 【ROBIN 内点结构】 | 仓库内 | 图论内点与最大团 |
| teaser | pmc / PCL / FLANN | 【TEASER 对齐】(TeaserAlign) | 仓库内/外部 | TEASER++ 鲁棒刚体配准 |
| genz_icp | matcher_tsl / PCL / TBB | 【GenZ-ICP】(genz_icp) | 仓库内/外部 | 精配准后端之一 |
| small_gicp / basic_icp / fast_gicp | Eigen / PCL / OpenMP | 精配 ICP 集合 | 仓库内/外部 | `robust_refine/` |
| robust_global_reg | robust_matcher | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 站间粗全局配准（内存点云；I/O 由调用方） |
| rough_global_reg | robust_global_reg | 【粗全局配准门面】(RoughGlobalReg) | 仓库内 | ScanInfo→FramedCloud→RobustGlobalReg；位姿折回 |
| rough_global_reg | point_cloud_io | 【FramedCloud】(load) | 仓库内 | 门面读 `ScanInfo.lasFn_` |
| rough_global_reg | registration_type | 【扫描信息】/【配准边结果】 | 仓库内 | 输入 ScanInfo，输出 AlignResult |
| fastreg_filter | logging / OpenCV / Eigen / OpenMP | 【全景深度差分滤波】/【空间透视差分滤波】 | 仓库内/外部 | 深度图邻域差分统计异常/重叠率（STATIC） |
| global_matrix_file | common / logging / RapidJSON | 【全局位姿文件读写】(GlobalMatrixFileIO) | 仓库内/外部 | 多站全局位姿 JSON loading/saving（STATIC） |
| depth_map_tool | common / OpenCV / PCL / TBB | 【深度图工具】(DepthMapTool) | 仓库内/外部 | 点云→全景深度与格式转换（STATIC） |
| octree | PCL / Eigen / TBB | 【体素真 Medoid 索引下采样】等 | 外部 | 哈希体素 Medoid / FastOctree / 中心近邻（INTERFACE） |
| overlap_estimation | PCL / Eigen / OpenMP | 【体素重叠估计】/【配准重叠子集提取】 | 外部 | 点云重叠度量与子集抽取（INTERFACE） |
| resolution_estimate | logging / registration_type / PCL / Eigen | 【分辨率估计】/【分位数分辨率估计】 | 仓库内/外部 | 站心径向深度 → 推荐体素（SHARED） |
| resolution_estimate_test | point_cloud_io | 【FramedCloud】+ origin 折回站心 | 仓库内 | 站心 LAS 估分辨率前还原文件系 |
| registration_initializer | registration_type / point_cloud_io / octree / resolution_estimate / small_gicp | 【配准初始化器】 | 仓库内 | 测站 Root+Medoid+法向+体素分辨率（SHARED） |
| WorkingFrame（common） | FramedCloud.origin | 【作业系 / 可恢复原点】 | 仓库内 | 作业级 O 与每云 origin 分层；见位姿折回规则 |

## 已登记模块

| 模块 | 领域字典 | 模块说明 |
|------|----------|----------|
| common | [`dictionaries/公共基础_Dictionary.md`](dictionaries/公共基础_Dictionary.md) | [`modules/公共基础.md`](modules/公共基础.md) |
| file_system | 同上 | 同上 |
| logging | 同上 | 同上 |
| **graph** | 同上 | 同上 |
| **registration_type** | [`dictionaries/配准类型_Dictionary.md`](dictionaries/配准类型_Dictionary.md) | [`modules/配准类型.md`](modules/配准类型.md) |
| **registration_initializer** | [`dictionaries/配准初始化_Dictionary.md`](dictionaries/配准初始化_Dictionary.md) | [`modules/配准初始化.md`](modules/配准初始化.md) |
| **point_cloud_io** | [`dictionaries/点云IO_Dictionary.md`](dictionaries/点云IO_Dictionary.md) | [`modules/点云IO.md`](modules/点云IO.md) |
| robust_matcher 等（`robust_rough/`） | [`dictionaries/鲁棒粗匹配_Dictionary.md`](dictionaries/鲁棒粗匹配_Dictionary.md) | [`modules/鲁棒粗匹配.md`](modules/鲁棒粗匹配.md) |
| basic_icp 等（`robust_refine/`） | [`dictionaries/鲁棒精配_Dictionary.md`](dictionaries/鲁棒精配_Dictionary.md) | [`modules/鲁棒精配.md`](modules/鲁棒精配.md) |
| robust_global_reg | [`dictionaries/粗全局配准_Dictionary.md`](dictionaries/粗全局配准_Dictionary.md) | [`modules/粗全局配准.md`](modules/粗全局配准.md) |
| **rough_global_reg** | 同上 | 同上 |
| **fastreg_filter** | [`dictionaries/FastReg滤波_Dictionary.md`](dictionaries/FastReg滤波_Dictionary.md) | [`modules/FastReg滤波.md`](modules/FastReg滤波.md) |
| **global_matrix_file** | [`dictionaries/全局位姿文件_Dictionary.md`](dictionaries/全局位姿文件_Dictionary.md) | [`modules/全局位姿文件.md`](modules/全局位姿文件.md) |
| **depth_map_tool** | [`dictionaries/深度图工具_Dictionary.md`](dictionaries/深度图工具_Dictionary.md) | [`modules/深度图工具.md`](modules/深度图工具.md) |
| **octree** | [`dictionaries/八叉树_Dictionary.md`](dictionaries/八叉树_Dictionary.md) | [`modules/八叉树.md`](modules/八叉树.md) |
| **overlap_estimation** | [`dictionaries/重叠估计_Dictionary.md`](dictionaries/重叠估计_Dictionary.md) | [`modules/重叠估计.md`](modules/重叠估计.md) |
| **resolution_estimate** | [`dictionaries/分辨率估计_Dictionary.md`](dictionaries/分辨率估计_Dictionary.md) | [`modules/分辨率估计.md`](modules/分辨率估计.md) |
