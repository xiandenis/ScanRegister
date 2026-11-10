# FramedCloud IO 设计（2026-11-10）

**状态**：已实施  
**治理**：R2 / 验证 L1  
**模块**：`point_cloud_io`

## 1. 问题（旧契约缺陷）

旧设计「`WorkingFrame` 在 `common`、IO 仍返回裸 `pcl::PointCloud`，LAS 可选减 header offset」有下列缺陷，**标为已废弃，禁止再当基线**：

| 缺陷 | 说明 |
|------|------|
| 责任断裂 | 原点可恢复性不在 IO 契约内，调用方易漏减/漏记 |
| 偏移语义错位 | LAS header offset 是量化锚点，不是作业工作原点；offset=0 的大坐标仍会炸 |
| 溯源丢失 | 仅 float XYZ 无法写回世界系 / 对接 LidarStudio |
| 大/小坐标分支 | `subtract_las_header_offset` 暗示按量级分支，违反「始终局部化」 |

## 2. 目标

- IO 公共契约只进出 **`FramedCloud`**：局部 float 点云 + **可恢复** `origin`（double）。
- 读：世界系 double 还原后，用**首个有效世界点**定 `origin`，再存 `float(p - origin)`。
- 写 LAS/SLAS：局部点入库，`header offset = origin`。
- 硬切删除裸 `pcl::PointCloud` 公共 load/save 与 `subtract_las_header_offset`。

## 3. 数据契约

```cpp
namespace Ddx::point_cloud_io {
template <typename PointT>
struct FramedCloud {
  pcl::PointCloud<PointT> cloud;     // 局部系 float XYZ
  Eigen::Vector3d origin{0, 0, 0}; // world ≈ local + origin
};
}
```

- **不**在 `FramedCloud` 内嵌 `WorkingFrame`（避免与未落地的作业级 frame 混淆）。
- 需要换系工具时：`WorkingFrame::fromOrigin(framed.origin)`。
- 不变量：`world ≈ local + origin`（float 精度内）。

### 原点候选

流式/顺序扫描世界坐标，取**第一个**满足：

- 有限（非 NaN/Inf）
- `squaredNorm >= 1e-12`

若无有效点 → `load` 失败。

### LAS header 角色

- 读：`world = scale * i + header_offset`（仅还原世界系）。
- 写：`header_offset = FramedCloud.origin`；点为局部坐标。
- `read_las_header_offset` 可保留作诊断，**不等于** `FramedCloud.origin`（除非碰巧相同）。

## 4. API

- `load` / `load_las` / `load_slas` / `load_ply` / `load_pcd` / `load_pcse` → `FramedCloud<PointT>&`
- `save` / `save_las` / `save_slas` / `save_pcd` / `save_ply` → `const FramedCloud<PointT>&`
- PCD/PLY：写局部坐标；origin **不**写入文件体（本轮无 sidecar）。
- 流式转换与上述原点规则一致；删除 `subtract_las_header_offset` / `preserve_source_las_offset`。

## 5. 作业级 WorkingFrame（非本轮）

每份 `FramedCloud` 自带 `origin`。作业级共用一个 `WorkingFrame`（由各云 origin 合成或显式设置）**本轮不做 API**，仅预留。

多云配准须先 rebase 到同一作业原点，或把局部 `M_loc` 折回：

\[
M_{\mathrm{world}} = T(O_t)\,M_{\mathrm{loc}}\,T(-O_s)
\]

使世界系下 \(\tilde{O}_t \approx M_{\mathrm{world}}\,\tilde{O}_s\)。

带世界系初始位姿时（精配准：`source`/`target`/`M_init`），点已局部化后：

\[
M_{\mathrm{init,loc}} = T(-O_t)\,M_{\mathrm{init,world}}\,T(O_s)
\]

（已 rebase 到作业 \(O\) 时用 \(T(-O)M_{\mathrm{init}}T(O)\)。）Agent 规则：`.cursor/rules/framed-cloud-pose-origin.mdc`。

## 6. 退化与往返

- 空 / 全无效 → 失败。
- 单有效点 → origin=该点，局部可为 (0,0,0)。
- LAS `save→load`：相对几何一致；`origin` 可为新文件首个有效世界点，不要求与旧 origin 逐位相等。

## 7. 非目标

- 作业级 WorkingFrame / rebase / 多云合成 O
- PCD/PLY origin sidecar
- 自研非 PCL SoA 点缓冲

## 9. 修订：移除 PDAL（2026-11-10）

`point_cloud_io` **不再链接 PDAL**。LAS 仅 `las_fast`（未压缩 format 0–3 / 6–8）。`.laz` 与波形格式明确失败；需要时再挂可选后端。

## 8. 验证（L1）

- 大坐标世界点 → 局部量级小、`local+origin≈world`
- 无效点前缀跳过，原点落到首有效点
- LAS 往返相对几何
- 仓内调用点编译通过（`robust_global_reg` 测试、`genz_icp_pair`）
