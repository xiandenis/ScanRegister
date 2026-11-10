# 点云算法总览（Hub）

ScanRegister（CMake 包 `scan_register`）跨模块互用速查。细则见 [`rules.md`](../rules.md)。

## 跨模块依赖（互用速查）

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| common | logging | 【日志管理器】(LogManager) | 仓库内 | 公共工具与调度输出日志 |
| logging | spdlog | spdlog::spdlog | 外部 | 日志后端 |
| common | Eigen3 | Eigen::Matrix / Vector | 外部 | 矩阵与坐标变换 |
| file_system | （无仓库内） | — | — | 独立文件系统工具 |
| point_cloud_io | logging / PCL / Eigen | 【FramedCloud / 点云读写】 | 仓库内/外部 | 公共 IO；局部云+可恢复 origin（无 PDAL） |
| 测试 / Session / 精配工具等 | point_cloud_io | 【FramedCloud】(load/save) | 仓库内 | 读盘局部化；配准前后对 M / M_init 做 origin 系变换 |
| robust_matcher | robin / matcher_* | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 鲁棒粗匹配主入口 |
| robin | pmc / xenium | 【ROBIN 内点结构】 | 仓库内 | 图论内点与最大团 |
| teaser | pmc / PCL / FLANN | 【TEASER 对齐】(TeaserAlign) | 仓库内/外部 | TEASER++ 鲁棒刚体配准 |
| genz_icp | matcher_tsl / PCL / TBB | 【GenZ-ICP】(genz_icp) | 仓库内/外部 | 精配准后端之一 |
| small_gicp / basic_icp / fast_gicp | Eigen / PCL / OpenMP | 精配 ICP 集合 | 仓库内/外部 | `robust_refine/` |
| robust_global_reg | robust_matcher | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 站间粗全局配准（内存点云；I/O 由调用方） |
| WorkingFrame（common） | FramedCloud.origin | 【作业系 / 可恢复原点】 | 仓库内 | 作业级 O 与每云 origin 分层；见位姿折回规则 |

## 已登记模块

| 模块 | 领域字典 | 模块说明 |
|------|----------|----------|
| common | [`dictionaries/公共基础_Dictionary.md`](dictionaries/公共基础_Dictionary.md) | [`modules/公共基础.md`](modules/公共基础.md) |
| file_system | 同上 | 同上 |
| logging | 同上 | 同上 |
| **point_cloud_io** | [`dictionaries/点云IO_Dictionary.md`](dictionaries/点云IO_Dictionary.md) | [`modules/点云IO.md`](modules/点云IO.md) |
| robust_matcher 等（`robust_rough/`） | [`dictionaries/鲁棒粗匹配_Dictionary.md`](dictionaries/鲁棒粗匹配_Dictionary.md) | [`modules/鲁棒粗匹配.md`](modules/鲁棒粗匹配.md) |
| basic_icp 等（`robust_refine/`） | [`dictionaries/鲁棒精配_Dictionary.md`](dictionaries/鲁棒精配_Dictionary.md) | [`modules/鲁棒精配.md`](modules/鲁棒精配.md) |
| robust_global_reg | [`dictionaries/粗全局配准_Dictionary.md`](dictionaries/粗全局配准_Dictionary.md) | [`modules/粗全局配准.md`](modules/粗全局配准.md) |
