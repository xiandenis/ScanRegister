# 点云算法总览（Hub）

ScanRegister（CMake 包 `scan_register`）跨模块互用速查。细则见 [`rules.md`](../rules.md)。

## 跨模块依赖（互用速查）

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| common | logging | 【日志管理器】(LogManager) | 仓库内 | 公共工具与调度输出日志 |
| logging | spdlog | spdlog::spdlog | 外部 | 日志后端 |
| common | Eigen3 | Eigen::Matrix / Vector | 外部 | 矩阵与坐标变换 |
| file_system | （无仓库内） | — | — | 独立文件系统工具 |
| robust_matcher | robin / matcher_* | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 鲁棒粗匹配主入口 |
| robin | pmc / xenium | 【ROBIN 内点结构】 | 仓库内 | 图论内点与最大团 |
| teaser | pmc / PCL / FLANN | 【TEASER 对齐】(TeaserAlign) | 仓库内/外部 | TEASER++ 鲁棒刚体配准 |
| genz_icp | matcher_tsl / PCL / TBB | 【GenZ-ICP】(genz_icp) | 仓库内/外部 | 精配准后端之一 |
| small_gicp / basic_icp / fast_gicp | Eigen / PCL / OpenMP | 精配 ICP 集合 | 仓库内/外部 | `robust_refine/` |
| robust_global_reg | robust_matcher | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 站间粗全局配准（内存点云；I/O 由调用方） |
| point_cloud_io | logging / PCL / PDAL | 【点云读写】 | 仓库内/外部 | 多格式点云 I/O |

## 已登记模块

| 模块 | 领域字典 | 模块说明 |
|------|----------|----------|
| common | [`dictionaries/公共基础_Dictionary.md`](dictionaries/公共基础_Dictionary.md) | [`modules/公共基础.md`](modules/公共基础.md) |
| file_system | 同上 | 同上 |
| logging | 同上 | 同上 |
| robust_matcher 等（`robust_rough/`） | [`dictionaries/鲁棒粗匹配_Dictionary.md`](dictionaries/鲁棒粗匹配_Dictionary.md) | [`modules/鲁棒粗匹配.md`](modules/鲁棒粗匹配.md) |
| basic_icp 等（`robust_refine/`） | [`dictionaries/鲁棒精配_Dictionary.md`](dictionaries/鲁棒精配_Dictionary.md) | [`modules/鲁棒精配.md`](modules/鲁棒精配.md) |
| robust_global_reg / point_cloud_io | [`dictionaries/粗全局配准_Dictionary.md`](dictionaries/粗全局配准_Dictionary.md) | [`modules/粗全局配准.md`](modules/粗全局配准.md) |
