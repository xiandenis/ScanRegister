# FramedCloud 点云 IO

**状态**：已实施  
**规格**：[`../superpowers/specs/2026-11-10-framed-cloud-io-design.md`](../superpowers/specs/2026-11-10-framed-cloud-io-design.md)

## 定案摘要

- IO 契约硬切为 `FramedCloud{ cloud, origin }`（局部 float + double 可恢复原点）。
- 原点 = 首个有效世界点；LAS 写回用 origin 作 header offset。
- 删除裸 `pcl::PointCloud` 公共 load/save 与 `subtract_las_header_offset`。
- 作业级 `WorkingFrame` 本轮仅文档预留。
- **后续修订**：PDAL 已从 `point_cloud_io` 硬切移除（读盘过慢）；LAS 仅 `las_fast`（未压缩 0–3/6–8）。`.laz` / 波形格式暂不支持，需要时再挂回可选后端。

## 落点

- `point_cloud_io/framed_cloud.hpp`
- `point_cloud_io/point_cloud_io.hpp` 及 las_fast / pcse / stream_convert
- 消费者：`robust_global_reg` 测试、`genz_icp_pair`

## 相关

- 字典：[`../dictionaries/点云IO_Dictionary.md`](../dictionaries/点云IO_Dictionary.md)（公共 IO 领域；粗配准字典仅交叉引用）
- 模块说明：[`../modules/点云IO.md`](../modules/点云IO.md)
- WorkingFrame：[`../dictionaries/公共基础_Dictionary.md`](../dictionaries/公共基础_Dictionary.md)
