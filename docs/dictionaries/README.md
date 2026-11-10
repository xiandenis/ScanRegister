# 领域字典（Spoke）

按 [`rules.md`](../../rules.md) 维护。每份 `*_Dictionary.md` 含：

1. `## 模块相互依赖关系`
2. 按 `## 模块：<名>` 分段的文件级 / 文件内小算法条目

| 字典 | 覆盖模块 |
|------|----------|
| [`公共基础_Dictionary.md`](公共基础_Dictionary.md) | `common`、`file_system`、`logging`、`graph` |
| [`配准类型_Dictionary.md`](配准类型_Dictionary.md) | `registration_type`（ScanInfo / LidarFrame / AlignResult） |
| [`配准初始化_Dictionary.md`](配准初始化_Dictionary.md) | `registration_initializer`（Root / 子体素 / 法向 / 分辨率） |
| [`点云IO_Dictionary.md`](点云IO_Dictionary.md) | `point_cloud_io`（公共 FramedCloud 读写） |
| [`鲁棒粗匹配_Dictionary.md`](鲁棒粗匹配_Dictionary.md) | `robust_rough/` 下 `robust_matcher`、`robin`、`pmc`、`teaser`、`matcher_*`、`xenium` |
| [`鲁棒精配_Dictionary.md`](鲁棒精配_Dictionary.md) | `robust_refine/` 下 `basic_icp`、`fast_gicp`、`small_gicp`、`genz_icp` |
| [`粗全局配准_Dictionary.md`](粗全局配准_Dictionary.md) | `robust_global_reg`（IO 见点云 IO） |
| [`FastReg滤波_Dictionary.md`](FastReg滤波_Dictionary.md) | `fastreg_filter`（深度图差分滤波） |
| [`全局位姿文件_Dictionary.md`](全局位姿文件_Dictionary.md) | `global_matrix_file`（多站全局位姿 JSON） |
| [`深度图工具_Dictionary.md`](深度图工具_Dictionary.md) | `depth_map_tool`（全景深度生成与格式转换） |
| [`八叉树_Dictionary.md`](八叉树_Dictionary.md) | `octree`（Medoid / FastOctree / 中心近邻） |
| [`重叠估计_Dictionary.md`](重叠估计_Dictionary.md) | `overlap_estimation`（体素重叠 / 配准子集） |
| [`分辨率估计_Dictionary.md`](分辨率估计_Dictionary.md) | `resolution_estimate`（直方图 / 分位数体素） |

Hub 总览：[`../点云算法总览_Dictionary.md`](../点云算法总览_Dictionary.md)。
