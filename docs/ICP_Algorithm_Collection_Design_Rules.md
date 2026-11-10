# ICP算法集合统一设计规则（ICP Algorithm Collection Design Rules）

> 状态：已按本仓库 **分阶段落地** 修订（`fine_registration` ICP 集合层）。  
> 对照实现与字典：[精配准_Dictionary.md](dictionaries/精配准_Dictionary.md)、[docs/plan/ICP算法集合归入精配准.md](plan/ICP算法集合归入精配准.md)。

## 1. 目标

本规则用于指导 Agent 整理、扩展和维护三维点云精配准算法集合。

设计目标：

- 多种 ICP / GICP 变体统一管理，接口一致。
- 参数分公共 / 专属 / 自适应三层，尺度相关，禁止无意义万能默认。
- 业务侧经 Manager + Factory 选型，禁止 if-else 点名具体算法类。
- 可扩展新算法；当前仓库已落地 FastGICP、BasicICP、FastVGICP、SmallGICP 四条路径。

------------------------------------------------------------------------

# 2. 总体架构（本仓库现状）

命名空间：`Ddx::registration`。模块根目录：**`fine_registration/`**（不用顶层 `registration/`）。

```
real_time_manager
        |
        |  （仍只调 FineRegistration 入口）
        v
FineRegistration
        |
        |  （已改经 Manager；对齐后估报告）
        v
RegistrationManager
        |
        v
RegistrationFactory::create(config)
        |
        v
RegistrationAlgorithm
   +-------------------+--------------------+------------------+
   |                   |                    |                  |
FastGicpAlgorithm  BasicIcpAlgorithm  FastVgicpAlgorithm  SmallGicpAlgorithm
   |                   |                    |                  |
fast_gicp/         basic_icp/           fast_gicp/         small_gicp/
（顶层保留）        （顶层保留）        FastVGICP          helper align
```

规则：

- **集合层**：上层只依赖 `RegistrationManager` / `RegistrationAlgorithm`，由 Factory 创建具体适配器。
- **业务层**：`FineRegistration::runInternal` 已改经 Manager（默认 `kSmallGicp`）；对齐成功后调用配准报告填评估字段；`real_time_manager` 调用方式未改。

------------------------------------------------------------------------

# 3. 统一接口规则

头文件：`fine_registration/interface/RegistrationAlgorithm.hpp`

点云类型：

```cpp
using SourceCloud = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;
using TargetCloud = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;
using InitialPose = Eigen::Matrix4d;
```

接口：

```cpp
class RegistrationAlgorithm
{
public:
    virtual ~RegistrationAlgorithm() = default;

    virtual bool align(SourceCloud source,
                       TargetCloud target,
                       InitialPose const& initial,
                       AlignResult& result) = 0;

    virtual void setConfig(RegistrationConfig const& config) = 0;

    virtual std::string name() const = 0;
};
```

禁止：

- 新集合业务代码直接 `new` / 调用具体 ICP 类（适配器内部除外）。
- 在 Manager 或业务中用 if-else 按算法名分支实现路径。
- 为每种算法复制一套孤立业务封装。

必须：经 `RegistrationFactory::create(config)` 创建。

------------------------------------------------------------------------

# 4. 参数设计规则

头文件：`fine_registration/config/RegistrationConfig.hpp`

```
RegistrationConfig
    ├── AlgorithmType type
    ├── CommonParams common
    └── std::variant<GICPParams, BasicIcpParams, VGICPParams> algorithm_params
```

自适应推荐不在 `RegistrationConfig` 内嵌死值，而由 `ParameterManager` 根据 `CloudFeature` 生成 `CommonParams` 建议值。

------------------------------------------------------------------------

# 5. 公共参数（CommonParams）

```cpp
struct CommonParams
{
    int max_iterations = 64;
    double transformation_epsilon = 1e-4;
    double fitness_epsilon = 1e-6;
    double max_correspondence_distance = 2.0;
    bool use_voxel_filter = false;
    double voxel_size = 0.1;
};
```

| 参数 | 作用 |
|------|------|
| max_iterations | 最大优化次数（底层有 setter 时落地） |
| transformation_epsilon | 位姿收敛阈值 |
| fitness_epsilon | 误差收敛阈值 |
| max_correspondence_distance | 最大匹配距离（算法对应关系阈值） |
| use_voxel_filter / voxel_size | 降采样开关与尺度（当前适配器可不强制执行滤波） |

说明：`BasicICP` 已透出 `max_iterations` → `max_icp`、`transformation_epsilon` → `stop`（与 SLAM 一致：缩放空间阈值，默认 `1e-5`，不再 `eps/scale`）。  
`max_correspondence_distance` / `fitness_epsilon` / 体素滤波在 FRICP/SICP **无对应项**，适配器校验后**不伪造**行为（Debug 日志标明 max_corr 被忽略）。

------------------------------------------------------------------------

# 6. 算法专属参数与类型

## 6.1 AlgorithmType（已落地）

```cpp
enum class AlgorithmType
{
    kFastGicp = 0,
    kBasicIcp = 1,
    kFastVgicp = 2,
    kSmallGicp = 3
    // 后续可扩：kNdt 等
};
```

## 6.2 专属参数（已落地）

```cpp
struct GICPParams
{
    int k_correspondence = 20;
    int num_threads = 0;  // <=0 → OpenMP max / 1
};

struct BasicIcpParams
{
    BasicICP::Method method = BasicICP::Method::RICP;
};

struct VGICPParams
{
    int k_correspondence = 20;
    int num_threads = 0;  // <=0 → OpenMP max / 1
    double voxel_resolution = 1.0;
    // 暂不开放：NeighborSearchMethod / VoxelAccumulationMode（内核默认 DIRECT1 / ADDITIVE）
};

struct SmallGicpParams
{
    enum class Method { kIcp, kPlaneIcp, kGicp, kVgicp };
    Method method = Method::kGicp;
    int num_threads = 0;                   // <=0 → OpenMP max / 1
    double voxel_resolution = 1.0;         // VGICP
    double downsampling_resolution = 0.25; // Eigen align 路径；use_voxel_filter 时可用 common.voxel_size
    // 暂不开放：rotation_eps / verbose
};
```

`RegistrationConfig::algorithm_params` 为：

```cpp
std::variant<GICPParams, BasicIcpParams, VGICPParams, SmallGicpParams>
```

禁止把 GICP / BasicICP / VGICP / SmallGICP 专属字段塞进 `CommonParams`（`voxel_size` 仍只表示降采样尺度；VGICP 体素分辨率用各自 Params 的 `voxel_resolution`）。

## 6.3 适配器行为摘要

| 适配器 | 底层 | 说明 |
|--------|------|------|
| `FastGicpAlgorithm` | `fast_gicp::FastGICP` | 落地 max_corr、k、threads、迭代/epsilon；成功后填 `AlignResult` |
| `BasicIcpAlgorithm` | `Ddx::registration::BasicICP` | PCL→Eigen；`setMethod` + `max_iterations`/`transformation_epsilon` + `align`；点到面缺法向失败；max_corr 不落地 |
| `FastVgicpAlgorithm` | `fast_gicp::FastVGICP` | 落地 voxel_resolution；邻域/体素累积暂用内核默认 |
| `SmallGicpAlgorithm` | `small_gicp::align` | PCL→Eigen；经 RegistrationSetting 注入 type/threads/体素与下采样；收敛后填 `AlignResult` |

CUDA / experimental FastGICP 变体不进 Factory。

------------------------------------------------------------------------

# 7. ICP 算法分类与推荐范围

## 7.1 Point-to-Point / 鲁棒 ICP（本仓库：BasicICP）

- 特点：基础或鲁棒点到点；对初值敏感；较快。
- 落地：`AlgorithmType::kBasicIcp`，默认 `Method::RICP`。
- 推荐：`max_iterations` 20–50；`max_correspondence_distance` ≈ 3–5 × 点间距。

## 7.2 Point-to-Plane

- 特点：用法向，收敛快；需法向估计或输入。
- 现状：`BasicICP` 的 PPL/RPPL/SICPPPL 在集合适配器中**缺法向则失败**。站扫定向 `alignOrientedRppl` 已于 2026-10-19 回退至 `SLAMRegistration/basic_icp`；历史见 [`docs/notes/2026-10-19-basic_icp站扫定向RPPL回退前快照.md`](notes/2026-10-19-basic_icp站扫定向RPPL回退前快照.md)。

## 7.3 GICP / Fast-GICP（本仓库：FastGicpAlgorithm）

- 特点：协方差马氏距离；Scan-to-Map / 激光精配准常用。
- 落地：`AlgorithmType::kFastGicp`；`k_correspondence` 默认 20。
- 旧 `FineRegistration` 默认 max_corr = 2 m（业务侧另配置）。

## 7.4 VGICP（本仓库：FastVgicpAlgorithm）

- 特点：目标高斯体素图 + OpenMP；适合较大点云 / Scan-to-Map。
- 落地：`AlgorithmType::kFastVgicp`；`voxel_resolution` 默认 1.0 m；邻域/体素累积暂不开放配置，沿用内核 `DIRECT1` / `ADDITIVE`。
- CUDA VGICP（`FastVGICPCuda`）不进 Factory。

## 7.5 Small-GICP（本仓库：SmallGicpAlgorithm）

- 特点：轻量并行 ICP/PlaneICP/GICP/VGICP（koide3/small_gicp）；Eigen helper 路径自带下采样与协方差估计。
- 落地：`AlgorithmType::kSmallGicp`；默认 `Method::kGicp`；`downsampling_resolution` 默认 0.25 m；VGICP 用 `voxel_resolution` 默认 1.0 m。
- 不走可选 `RegistrationPCL` 头（库目标未强制链 PCL）。

## 7.6 NDT（未进 Factory）

- 仍可作为后续扩展；参数经验：NDT resolution 0.5–2.0 m 等。
- 顶层 `fast_gicp` 中 CUDA NDT 头文件存在，但集合层未注册。

------------------------------------------------------------------------

# 8. 参数自动生成（ParameterManager）

头文件：`fine_registration/adaptive/ParameterManager.hpp`

输入：

```cpp
struct CloudFeature
{
    double resolution = 0.05;
    double density = 0.0;
    double curvature = 0.0;
    bool indoor = false;
    bool sparse = false;
};
```

输出：`ParameterManager::recommendCommon(feature)` → `CommonParams`。

约定（当前实现）：

- `voxel_size ≈ 3 * resolution`
- `max_correspondence_distance ≈ 4 * resolution`（稀疏时 ×5）
- `max_iterations` 钳制在 20–50
- `resolution` 非法时回退 0.05

禁止在所有场景写死同一套与尺度无关的公共参数而不经推荐或说明。

------------------------------------------------------------------------

# 9. Voxel 参数规则

```
voxel_size ≈ point_resolution * 3
```

经验参考：

| 场景 | Voxel |
|------|-------|
| 室内 RGBD | 0.01–0.05 m |
| Livox | 0.05–0.15 m |
| Velodyne | 0.1–0.3 m |

------------------------------------------------------------------------

# 10. Correspondence 距离规则

禁止无场景依据地固定 `max_correspondence_distance = 1 m`。

应：

```
max_correspondence_distance ≈ 3~5 * point_resolution
```

该距离用于算法匹配阈值（如 FastGICP `setMaxCorrespondenceDistance`）。

旧 `FineRegistration` 业务默认 2 m，与 `registration_report` 重叠阈（默认 0.2 m）分离。评估指标不由 ICP 适配器填写。

------------------------------------------------------------------------

# 11. Factory 规则

头文件：`fine_registration/factory/RegistrationFactory.hpp`

```cpp
auto algo = RegistrationFactory::create(config);
```

- Factory 内按 `AlgorithmType` 创建适配器并 `setConfig`。
- Manager / 业务禁止再写算法选型 if-else。

------------------------------------------------------------------------

# 12. 返回结果统一规则（AlignResult）

集合层**不**使用独立 `RegistrationResult`。统一使用 `registration_type` 中的：

```cpp
struct AlignResult
{
    StateType state_;
    std::string src_name_;   // 可空
    std::string tgt_name_;   // 可空
    Eigen::Matrix4d matrix_; // 源 → 目标
    double similarity_;
    double rms_;
    double final_rms_;
    double weight_;
    int feature_num_;
    int auto_reg_;
};
```

## 12.1 ICP 适配器填写范围

| 字段 | ICP 适配器 |
|------|------------|
| `matrix_` | 写入最终变换 |
| `state_` | 成功：`VALID \| HAS_CALCU`；失败：`INVALID` |
| `src_name_` / `tgt_name_` | 清空（可空；上层按需填） |
| `similarity_` / `rms_` / `final_rms_` / `weight_` / `feature_num_` | **不计算**；保持 `reset()` 默认（0 / -1 / 1 / 0） |

实现辅助：`align_result_utils::fillSuccess` / `markFailed`。

## 12.2 统一评估接口

内点/重叠、相似度、RMS、`feature_num_`、`weight_` 等由 **`RegistrationReportEstimator`**（`registration_report`）统一评估后写入 `AlignResult`。

- `FineRegistration::runInternal` 在 Manager `align` 成功后调用报告估计，回填 `similarity_`/`rms_`/`final_rms_`/`weight_`/`feature_num_` 与站名。
- `RegistrationManager` **不**自动调用评估；由上层在 `align` 成功后再评估并回填。
- 报告重叠距离阈 API（`setReportDistanceThresh`，默认 0.2 m）与求解对应距离分离。
- RMS 内点阈 API（`setReportRmsInlierThresh`，默认 0.1 m）与重叠阈分离：仅 `|点到面残差| < 内点阈` 计入 RMSE。

------------------------------------------------------------------------

# 13. 扩展新算法规则

新增算法必须完成：

1. 实现 `RegistrationAlgorithm` 适配器（`algorithms/`）。
2. 增加 `AlgorithmType`。
3. 增加专属 Params，并入 `algorithm_params` 的 `std::variant`。
4. 在 `RegistrationFactory` 注册。
5. 用 `AlignResult` 只填 `matrix_`/`state_`；评估字段留给 `RegistrationReportEstimator`。
6. 增加 / 扩展 `registration_collection_compile_test`（或等价单测）。
7. 更新 [精配准_Dictionary.md](dictionaries/精配准_Dictionary.md) 与本文件。

------------------------------------------------------------------------

# 14. 场景推荐策略

## Scan Matching / 站间精配准（本仓库主路径）

推荐：`kFastGicp`（与现网 `FineRegistration` 一致）。

备选：`kBasicIcp` + `RICP`（小初值误差、实验）。

## Scan-to-Map

推荐：`kFastGicp` / `kFastVgicp`；轻量备选 `kSmallGicp`（含 VGICP）。

## Loop Closure

推荐：粗匹配（如 NDT，待接入）+ GICP / Fast-GICP 精匹配。

------------------------------------------------------------------------

# 15. Agent 执行规则

整理或扩展本集合时必须：

1. 判断算法类别与是否已有适配器。
2. 公共参数进 `CommonParams`，专属进 variant。
3. 只经 Factory / Manager 暴露给业务。
4. 结果用 `AlignResult`，遵守 §12（ICP 不填评估字段）。
5. 尺度相关参数经 `ParameterManager` 或明确文档化默认值。
6. 同步字典与 `docs/plan` 归档（若范围变更）。

禁止：

- 再复制孤立精配准业务模块。
- 混合专属参数进公共结构。
- 忽略点云尺度写死匹配距离。
- 未经说明改断 `FineRegistration` / `real_time_manager`（除非任务明确要求接线）。

------------------------------------------------------------------------

# 16. 目录规范（本仓库落地）

```
fine_registration/
├── interface/
│   └── RegistrationAlgorithm.hpp
├── config/
│   └── RegistrationConfig.hpp
├── factory/
│   └── RegistrationFactory.hpp|.cpp
├── algorithms/
│   ├── FastGicpAlgorithm.hpp|.cpp
│   ├── BasicIcpAlgorithm.hpp|.cpp
│   ├── FastVgicpAlgorithm.hpp|.cpp
│   ├── SmallGicpAlgorithm.hpp|.cpp
│   └── align_result_utils.hpp
├── adaptive/
│   └── ParameterManager.hpp|.cpp
├── registration_manager.hpp|.cpp
├── fine_registration.hpp|.cpp          # 业务封装（经 Manager，默认 FastGICP）
├── registration_collection_compile_test.cpp
└── fine_registration_compile_test.cpp

仓库顶层（暂不搬家，由适配器链接）：
  fast_gicp/
  basic_icp/
  small_gicp/
```

CMake：扩现有目标 `fine_registration`，`PUBLIC` 链接 `fast_gicp`、`basic_icp`、`small_gicp`、`registration_type` 等。

------------------------------------------------------------------------

# 17. 分阶段状态与后续

**已完成**

- Manager / Factory / FastGICP·BasicICP·FastVGICP·SmallGICP 四适配器 / ParameterManager / AlignResult 约定
- 顶层 `fast_gicp`、`basic_icp`、`small_gicp` 逻辑归并（不物理搬家）
- 集合 compile smoke：`registration_collection_compile_test`
- `FineRegistration::runInternal` 改经 Manager（默认 SmallGICP）；对齐后调用 `RegistrationReportEstimator` 填评估字段

**明确未做（后续）**

- 物理迁入 `algorithms/vendor/{fast_gicp,basic_icp}`
- Factory 注册 NDT / CUDA VGICP / 点到面带法向路径

以上规则以本仓库当前代码为准；扩展时先改实现与字典，再回写本文件。
