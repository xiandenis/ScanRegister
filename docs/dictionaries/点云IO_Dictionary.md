# 点云 IO_Dictionary

覆盖模块：`point_cloud_io`（**公共点云读写库**，跨粗配准 / 精配准 / 测试 / 产品联调共用；**不属于**粗全局配准领域）。

**产物约定**：`point_cloud_io` 为 **SHARED**（仓库默认）。

权威规格：[`../superpowers/specs/2026-11-10-framed-cloud-io-design.md`](../superpowers/specs/2026-11-10-framed-cloud-io-design.md)。  
配准位姿折回 / 初始阵入局部系：`.cursor/rules/framed-cloud-pose-origin.mdc`。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| point_cloud_io | logging | LogManager | 仓库内 | 读盘失败与诊断日志 |
| point_cloud_io | PCL | pcl_io / pcl_common | 外部 | PCD/PLY 与点类型 |
| point_cloud_io | Eigen | Vector3d | 外部 | FramedCloud.origin |
| point_cloud_io | RapidJSON | — | 外部 | PCSE manifest |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| 测试 / Session / 精配工具等 | point_cloud_io | 【FramedCloud / 点云读写】 | 仓库内 | `load`→局部云+origin；算法用 `.cloud` |
| robust_global_reg（仅测试侧） | point_cloud_io | 同上 | 仓库内 | 库本身**不**链 IO；测试读盘后 `run` |
| genz_icp_pair 等 | point_cloud_io | 同上 | 仓库内 | 读盘后对 init/结果做 origin 系变换 |
| resolution_estimate_test | point_cloud_io | 同上 | 仓库内 | 站心 LAS：load 后 `local+origin` 再估径向深度 |
| registration_initializer | point_cloud_io | 同上 | 仓库内 | load 后 local+origin−ScanInfo.station_origin_ → 站心系再 Root/分辨率 |
| rough_global_reg | point_cloud_io | 同上 | 仓库内 | 门面自 ScanInfo.lasFn_ 加载；世界系 guess/结果折回 |

---

## IO 契约（摘要）

- 公共契约只进出 **`FramedCloud`**（局部 float XYZ + 可恢复 `Eigen::Vector3d origin`）。
- 读：世界系还原后，原点 = 首个有效世界点；点存 `float(p - origin)`。
- 写 LAS/SLAS：局部点 + `header offset = origin`；PCD/PLY 写局部（无 sidecar）。
- **无 PDAL**；LAS 仅 `las_fast`（未压缩 format 0–3 / 6–8）；`.laz` 暂不支持。
- 禁止裸 `pcl::PointCloud` 当 IO 出口；禁止用 LAS header offset 冒充工作原点。
- 多云配准：rebase 或折回 `M_world`；世界系 `M_init` 须先变到局部系（见位姿规则）。

---

## 模块：point_cloud_io

依赖摘要：SHARED；logging、PCL、Eigen、RapidJSON。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 点云读写 | Ddx::point_cloud_io | point_cloud_io.hpp/.cpp | 公共 | 多格式读/写；契约为 FramedCloud |
| 2 | 带原点点云载体 | FramedCloud | framed_cloud.hpp | 公共 | 局部 float 云 + double origin |
| 3 | LAS 快读 | las_fast_reader | las_fast_reader.hpp/.cpp | 公共 | 未压缩 LAS 快读并局部化 |
| 4 | PCSE 读取 | pcse_reader | pcse_reader.hpp/.cpp | 公共 | PCSE 数据集读取并局部化 |
| 5 | 点云流式转换 | point_cloud_stream_convert | point_cloud_stream_convert.hpp/.cpp | 公共 | 流式转换；原点规则同 FramedCloud |

### 文件内小算法条目

#### 1、点云读写（point_cloud_io）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 10 | 加载带原点点云 | load / load_las / load_slas / load_ply / load_pcd / load_pcse | 公共 | 世界系定 origin，输出局部 FramedCloud |
| 11 | 保存带原点点云 | save / save_las / save_slas / save_pcd / save_ply | 公共 | LAS 写 header offset=origin；PCD/PLY 写局部 |
| 12 | 格式探测 | detect_format / file_extension | 公共 | 扩展名与 PCSE 目录探测 |
| 13 | LAS 头偏移诊断 | read_las_header_offset | 公共 | 读量化锚点；≠ FramedCloud.origin |

#### 2、带原点点云载体（FramedCloud）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 20 | 原点候选判定 | is_valid_origin_candidate | 公共 | 有限且 squaredNorm≥1e-12 |
| 21 | 世界云就地局部化 | localize_world_cloud | 公共 | 首有效点为 O，点减 O |

#### 3、LAS 快读（las_fast_reader）

世界坐标 = scale×i+header；再减 FramedCloud.origin（首有效世界点）。mmap/分块；无 PDAL。

#### 4、PCSE 读取（pcse_reader）

解码得数据集坐标后，按 FramedCloud 规则定 origin 并局部化。

#### 5、点云流式转换（point_cloud_stream_convert）

先发现工作 origin，再流式写局部；LAS 目的文件 header offset=该 origin。
