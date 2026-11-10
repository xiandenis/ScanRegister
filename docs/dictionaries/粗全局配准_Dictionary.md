# 粗全局配准_Dictionary

覆盖模块：`robust_global_reg`、`point_cloud_io`（同领域 I/O；粗配准库本身**不再**链接 I/O）。

**产物约定**：`robust_global_reg` 为 **STATIC**（模块 CMake 约定）。`point_cloud_io` 为 **SHARED**（仓库默认）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_global_reg | robust_matcher | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 粗匹配核心 |
| robust_global_reg | common / logging / file_system | Common / LogManager | 仓库内 | 工具、日志、路径 |
| robust_global_reg | Eigen / RapidJSON | — | 外部 | 位姿与参数 JSON |
| point_cloud_io | logging / PCL | — | 仓库内/外部 | 点云读写后端（las_fast；PDAL 已移除，后续可再挂） |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 Session / 测试） | robust_global_reg | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 调用方先加载并局部化点云再 `run(src,tgt)` |
| （测试 / 业务加载侧） | point_cloud_io | 【点云读写 / FramedCloud】(point_cloud_io) | 仓库内 | 消费者侧读盘得 `FramedCloud`，非 robust_global_reg 库依赖 |

---

## 消费者输入契约

调用方全权管理输入点云的坐标系与语义；模块不做加载、不做站心推断、不校验站心是否在原点。

**IO 侧**：`point_cloud_io` 公共契约为 **`FramedCloud`**（局部 float 云 + 可恢复 `origin`）；禁止再把裸 `pcl::PointCloud` 当 IO 出口。旧「减 LAS header offset 当工作系」为**已废弃缺陷**。作业级共用 `WorkingFrame` 由上层从各 `FramedCloud.origin` 合成或设置（本轮 IO 不提供）。

**配准位姿**：在局部云上求出的 `M_loc` 须折回世界系（推荐先 rebase 到同一作业原点，或 \(M_{\mathrm{world}}=T(O_t)M_{\mathrm{loc}}T(-O_s)\)），使 \(\tilde{O}_t \approx M_{\mathrm{world}}\tilde{O}_s\)。若输入世界系初始阵 `M_init`（精配准等），进求解前须 \(M_{\mathrm{init,loc}}=T(-O_t)M_{\mathrm{init}}T(O_s)\)（或作业原点形式）。规则：`.cursor/rules/framed-cloud-pose-origin.mdc`。

| 场景 | 调用方必须保证 |
|------|----------------|
| 通用粗全局配准 | src/tgt 已在同一约定工作系（通常取自 `FramedCloud` 局部点，或再 rebase 到作业 `WorkingFrame`） |
| `use_compass == true`（罗盘约束） | **必须是站心坐标系数据**：测站原点（站心）在 `(0, 0, 0)`；点坐标相对该站心。非站心 / 大坐标 / 仅减 LAS header offset 而未对齐站心，罗盘语义无效 |

---

## 模块：robust_global_reg

依赖摘要：STATIC；链 robust_matcher、common、logging、file_system。**不**链 `point_cloud_io`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 鲁棒全局配准 | Ddx::RobustGlobalReg | robust_global_reg.hpp/.cpp | 公共 | 站间粗全局配准入口（内存点云 + RobustMatcher） |
| 2 | 粗全局配准参数 | RobGlobRegParameter | robGlob_reg_parameter.hpp/.cpp | 公共 | 配置读写与配准参数（无路径加载字段） |
| 3 | 粗配平移检查 | t_coarse_check | t_coarse_check.h/.cpp | 私有 | 粗配平移合理性检查 |
| 4 | 管线重叠辅助 | pipeline_overlap | pipeline_overlap.hpp | 忽略 | 联调测试用重叠辅助头 |

### 文件内小算法条目

#### 1、鲁棒全局配准（RobustGlobalReg）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 运行粗配准 | RobustGlobalReg::run(src,tgt) | 公共 | 对内存 Eigen Vector3f 向量执行配准 |
| 6 | 从文件设参 | setParameterFromFile | 公共 | JSON 算法配置加载（不含点云路径） |

#### 2、粗全局配准参数（RobGlobRegParameter）

可选 `scan_name`（日志/MemPeak）；`result_file` / `log_file`；算法开关含 `use_compass`（见上契约）。

#### 3、粗配平移检查（t_coarse_check）

待补充（需结合业务确认）

#### 4、管线重叠辅助（pipeline_overlap）

待补充（需结合业务确认）

---

## 模块：point_cloud_io

依赖摘要：SHARED；logging、PCL、Eigen、RapidJSON（**无 PDAL**；LAS 走 las_fast，`.laz` 暂不支持）。由**调用方/测试**链接，供加载后再交给 `RobustGlobalReg::run`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 7 | 点云读写 | Ddx::point_cloud_io | point_cloud_io.hpp/.cpp | 公共 | 多格式读/写；契约为 FramedCloud |
| 8 | 带原点点云载体 | FramedCloud | framed_cloud.hpp | 公共 | 局部 float 云 + double origin |
| 9 | LAS 快读 | las_fast_reader | las_fast_reader.hpp/.cpp | 公共 | LAS 快读并局部化到 FramedCloud |
| 10 | PCSE 读取 | pcse_reader | pcse_reader.hpp/.cpp | 公共 | PCSE 数据集读取并局部化 |
| 11 | 点云流式转换 | point_cloud_stream_convert | point_cloud_stream_convert.hpp/.cpp | 公共 | 流式转换；原点规则同 FramedCloud |

### 文件内小算法条目

#### 7、点云读写（point_cloud_io）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 30 | 加载带原点点云 | load / load_las / … | 公共 | 世界系定 origin，输出局部 FramedCloud |
| 31 | 保存带原点点云 | save / save_las / … | 公共 | LAS 写 header offset=origin；PCD/PLY 写局部 |

#### 8、带原点点云载体（FramedCloud）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 32 | 原点候选判定 | is_valid_origin_candidate | 公共 | 有限且 squaredNorm≥1e-12 |
| 33 | 世界云就地局部化 | localize_world_cloud | 公共 | 首有效点为 O，点减 O |

#### 9、LAS 快读（las_fast_reader）

世界坐标 = scale×i+header；再减 FramedCloud.origin（首有效世界点）。

#### 10、PCSE 读取（pcse_reader）

解码得数据集坐标后，按 FramedCloud 规则定 origin 并局部化。

#### 11、点云流式转换（point_cloud_stream_convert）

先发现工作 origin，再流式写局部；LAS 目的文件 header offset=该 origin。
