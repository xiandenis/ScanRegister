# 点云 IO（公共库）

| 目标 | 类型 | 说明 |
|------|------|------|
| `point_cloud_io` | **SHARED** | 多格式点云读/写；公共契约 **`FramedCloud`**（局部云 + 可恢复 `origin`） |

**不属于**粗全局配准库硬依赖；任意算法 / 测试 / 产品侧按需链接。

## 链接关系

```text
point_cloud_io
  ├── logging
  ├── pcl_io / pcl_common
  ├── Eigen3
  └── RapidJSON（PCSE）
```

消费：`target_link_libraries(foo PRIVATE point_cloud_io)`。不在默认伞目标 `scan_register` 内时仍可作为独立目标链接。

## API 契约（摘要）

- `load*(path, FramedCloud&)` / `save*(path, FramedCloud const&)`
- LAS：仅 `las_fast`（未压缩 0–3 / 6–8）；**无 PDAL**；`.laz` 暂不支持
- 配准用局部点后：结果 `M` 折回世界系；世界系 `M_init` 先变局部系（见规则 `framed-cloud-pose-origin`）

## 规格 / 计划

- 规格：[`../superpowers/specs/2026-11-10-framed-cloud-io-design.md`](../superpowers/specs/2026-11-10-framed-cloud-io-design.md)
- 计划：[`../plan/FramedCloud点云IO.md`](../plan/FramedCloud点云IO.md)

## 字典

[`../dictionaries/点云IO_Dictionary.md`](../dictionaries/点云IO_Dictionary.md)
