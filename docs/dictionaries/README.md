# 领域字典（Spoke）

按 [`rules.md`](../../rules.md) 维护。每份 `*_Dictionary.md` 含：

1. `## 模块相互依赖关系`
2. 按 `## 模块：<名>` 分段的文件级 / 文件内小算法条目

| 字典 | 覆盖模块 |
|------|----------|
| [`公共基础_Dictionary.md`](公共基础_Dictionary.md) | `common`、`file_system`、`logging` |
| [`点云IO_Dictionary.md`](点云IO_Dictionary.md) | `point_cloud_io`（公共 FramedCloud 读写） |
| [`鲁棒粗匹配_Dictionary.md`](鲁棒粗匹配_Dictionary.md) | `robust_rough/` 下 `robust_matcher`、`robin`、`pmc`、`teaser`、`matcher_*`、`xenium` |
| [`鲁棒精配_Dictionary.md`](鲁棒精配_Dictionary.md) | `robust_refine/` 下 `basic_icp`、`fast_gicp`、`small_gicp`、`genz_icp` |
| [`粗全局配准_Dictionary.md`](粗全局配准_Dictionary.md) | `robust_global_reg`（IO 见点云 IO） |

Hub 总览：[`../点云算法总览_Dictionary.md`](../点云算法总览_Dictionary.md)。
