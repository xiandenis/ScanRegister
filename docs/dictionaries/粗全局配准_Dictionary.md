# 粗全局配准_Dictionary

覆盖模块：`robust_global_reg`（站间粗全局配准内核）、`rough_global_reg`（业务门面）。  
公共点云 IO 见独立领域：[`点云IO_Dictionary.md`](点云IO_Dictionary.md)（`point_cloud_io` / `FramedCloud`）。

**产物约定**：`robust_global_reg`、`rough_global_reg` 均为 **SHARED**（仓库默认）。内核**不**链接 `point_cloud_io`；门面链接 `point_cloud_io` 做读盘与局部化。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_global_reg | robust_matcher | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 粗匹配核心 |
| robust_global_reg | common / logging / file_system | Common / LogManager | 仓库内 | 工具、日志、路径 |
| robust_global_reg | Eigen / RapidJSON | — | 外部 | 位姿与参数 JSON |
| rough_global_reg | robust_global_reg | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 门面组参后 `run(src,tgt)` |
| rough_global_reg | point_cloud_io | 【FramedCloud】(load) | 仓库内 | 自 `ScanInfo.lasFn_` 读盘局部化 |
| rough_global_reg | registration_type | 【扫描信息】/【配准边结果】 | 仓库内 | `ScanInfo` 入、`AlignResult` 出 |
| rough_global_reg | logging / file_system | LogManager / stlplus | 仓库内 | 路径检查与诊断日志 |
| rough_global_reg | Eigen3 / PCL | Matrix4d / PointXYZ | 外部 | 位姿折回与点缓冲 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| rough_global_reg | robust_global_reg | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 会话/业务路径经门面调用内核 |
| （待上层 Session / 测试） | rough_global_reg | 【粗全局配准门面】(RoughGlobalReg) | 仓库内 | 链式粗配 / 联调入口 |

### 与公共 IO 的衔接（跨领域）

| 调用方 | 被调用方 | 关键衔接点 | 说明 |
|--------|----------|------------|------|
| 测试 / 直接调内核 | point_cloud_io | 【FramedCloud】 | 读盘得局部云+origin，rebase 后 `run(src,tgt)` |
| rough_global_reg | point_cloud_io | 【FramedCloud】 | 门面统一从 `ScanInfo.lasFn_` 加载；**无**路径字符串 API |

---

## 消费者输入契约

### robust_global_reg（内核）

调用方全权管理输入点云的坐标系与语义；模块不做加载、不做站心推断、不校验站心是否在原点。

| 场景 | 调用方必须保证 |
|------|----------------|
| 通用粗全局配准 | src/tgt 已在同一约定工作系（通常取自 `FramedCloud` 局部点，或再 rebase 到作业 `WorkingFrame`） |
| `use_compass == true`（罗盘约束） | **必须是站心坐标系数据**：测站原点（站心）在 `(0, 0, 0)` |

### rough_global_reg（门面）

统一入口：`ScanInfo`（`LidarFrame` 因继承亦可绑定，但只用 `ScanInfo` 字段，云来自 `lasFn_`）。**已删除**路径字符串 `run`。

| 入口 | 语义 |
|------|------|
| `ScanInfo` + `resolution` + 可选世界系 `guess` | `load(lasFn_)` → FramedCloud；无罗盘时 rebase 到源 `origin`；有罗盘时折到站心系（须双方 `SENSOR_STAND`） |
| `LidarFrame` + `resolution` | **SENSOR_STAND 专用入口**（非架站禁止）：须双方 `SENSOR_STAND`、帧/`voxel_3` 均 `HAS_CALCU`、缓存文件在；直读站心系 PCD（禁止 FramedCloud 再局部化）；折回用 `station_origin_` |
| 初始阵 / 结果阵 | 调用方传入的 `guess` 为**文件/世界系**；进内核前变局部/站心系；`AlignResult.matrix_` 折回**文件/世界系**（\(M=T(O_t)M_{\mathrm{loc}}T(-O_s)\)） |
| 分辨率策略 | `resolution≥0.20` → HNSW + 特征分箱软预算；否则 FLANN。无初始阵时按 resolution 估迭代次数 |
| 罗盘 | **仅站心系可用**：须双方 `SENSOR_STAND`；`ScanInfo` 路径先折站心再开；`LidarFrame` 缓存已是站心系。无猜测：双方 `SENSOR_COMPASS` **且** `SENSOR_STAND` 才自动开；显式开缺站心则失败 |

---

## 模块：robust_global_reg

依赖摘要：SHARED；链 robust_matcher、common、logging、file_system。**不**链 `point_cloud_io`。

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

可选 `scan_name`（日志/MemPeak）；`result_file` / `log_file`；算法开关含 `use_compass`、`use_hnsw_knn`、`use_feat_spatial_budget`（业务路径由 RoughGlobalReg 按 resolution 设）。

#### 3、粗配平移检查（t_coarse_check）

待补充（需结合业务确认）

#### 4、管线重叠辅助（pipeline_overlap）

待补充（需结合业务确认）

---

## 模块：rough_global_reg

依赖摘要：SHARED；PUBLIC 链 `robust_global_reg`、`point_cloud_io`、`registration_type`、Eigen、`file_system`、`logging`。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 7 | 粗全局配准门面 | Ddx::RoughGlobalReg | rough_global_reg.hpp/.cpp | 公共 | ScanInfo→FramedCloud→内核；位姿世界↔局部折回 |
| 8 | 粗配准门面联调 | rough_global_reg_test | rough_global_reg_test.cpp | 忽略 | ScanInfo + las 路径联调 exe |

### 文件内小算法条目

#### 7、粗全局配准门面（RoughGlobalReg）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 9 | ScanInfo 有初始阵粗配 | RoughGlobalReg::run(ScanInfo..., guess, ...) | 公共 | lasFn_→FramedCloud；世界系初值；罗盘显式 |
| 10 | ScanInfo 无初始阵粗配 | RoughGlobalReg::run(ScanInfo...) | 公共 | 自动估迭代；canUseCompass 时开罗盘 |
| 11 | LidarFrame 有初始阵粗配 | RoughGlobalReg::run(LidarFrame..., guess, ...) | 公共 | SENSOR_STAND 专用；voxel_3 站心缓存；世界系初值 |
| 12 | LidarFrame 无初始阵粗配 | RoughGlobalReg::run(LidarFrame...) | 公共 | SENSOR_STAND 专用；会话主路径；canUseCompass 时开罗盘 |
| 13 | 解析迭代次数 | resolveIterationCount | 私有 | resolution 分段映射迭代（或采用显式值） |
| 14 | 罗盘可用判定 | canUseCompass | 私有 | 双方 SENSOR_COMPASS 且均已声明站心 |
| 15 | LidarFrame 就绪判定 | isReadyLidarFrame | 私有 | SENSOR_STAND + HAS_CALCU + voxel_3 缓存可用 |
| 16 | 解析 HNSW 近邻 | resolveUseHnswKnn | 私有 | resolution≥0.20 → HNSW，否则 FLANN |
| 17 | 解析特征分箱预算 | resolveUseFeatSpatialBudget | 私有 | 与 HNSW 同阈值，仅室外开软预算 |
| 18 | ScanInfo 装载执行 | runFromScanInfo | 私有 | FramedCloud 加载/折系后进内核 |
| 19 | LidarFrame 装载执行 | runFromLidarFrame | 私有 | 站心 PCD 直读（无 IO 再局部化） |
| 20 | 内核组参执行 | runKernel | 私有 | 组参/`run(src,tgt)`/折回世界系 |

#### 8、粗配准门面联调（rough_global_reg_test）

待补充（需结合业务确认）
