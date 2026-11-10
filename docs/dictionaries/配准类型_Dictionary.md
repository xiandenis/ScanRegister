# 配准类型_Dictionary

覆盖模块：`registration_type`（测站/边/状态等配准会话数据结构与 JSON IO）。  
**产物约定**：`registration_type` 为 **SHARED**（跨静态/动态边界避免类型/IO 重复）。

Session / party 面基础类型库；`VoxelSceneClass` 以本模块为权威（`resolution_estimate` 复用，禁止平行再定义）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| registration_type | common | Common 串编解码 | 仓库内 | 位姿/矩阵文本编解码 |
| registration_type | logging | LogManager | 仓库内 | IO 失败与诊断日志 |
| registration_type | Eigen3 / PCL | Matrix4d / PointXYZ | 外部 | 位姿与点类型别名 |
| registration_type | RapidJSON | document / writer | 外部 | LidarFrame / AlignResult / StationStatus JSON |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| scan_register（伞） | registration_type | ScanInfo / LidarFrame / AlignResult | 仓库内 | party 面随伞目标导出 |
| resolution_estimate | registration_type | 【体素场景类】(VoxelSceneClass) | 仓库内 | 分位数曲率调制场景提示 |
| （待上层 Session / 位姿图） | registration_type | 【测站帧】/【配准边结果】IO | 仓库内 | 业务接入后回填 |

---

## 模块：registration_type

依赖摘要：`common`、`logging`、`Eigen3`、`pcl_common`、`pcl_search`、RapidJSON。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 扫描信息 | ScanInfo | registration_type.hpp/.cpp | 公共 | 站名、路径、传感器位标志、全局矩阵 |
| 2 | 测站帧 | LidarFrame | registration_type.hpp/.cpp | 公共 | 体素/Root/子体素与缓存目录 |
| 3 | 配准边结果 | AlignResult | registration_type.hpp/.cpp | 公共 | source→target 刚体、RMSE[m]、边权与信息阵 |
| 4 | 测站图 | StationGraph | registration_type.hpp | 公共 | 顶点全局位姿 + 边 RMSE/相似度 |
| 5 | 测站状态 | StationStatus | registration_type.hpp/.cpp | 公共 | 网页进度 phase / 耗时 / rms |
| 6 | 体素场景类 | VoxelSceneClass | registration_type.hpp/.cpp | 公共 | 曲率调制场景提示枚举 |
| 7 | 测站帧 IO | LidarFrameIO | lidar_frame_io.hpp/.cpp | 公共 | LidarFrame JSON 读写 |
| 8 | 配准边结果 IO | AlignResultIO | align_result_io.hpp/.cpp | 公共 | AlignResult JSON 读写 |
| 9 | 测站状态 IO | StationStatusIO | station_status_io.hpp/.cpp | 公共 | registration_status.json 读写/upsert |
| 10 | PCL 点类型别名 | pcl_type | pcl_type.hpp | 公共 | PointXYZ / Normal / KdTree 别名 |

### 文件内小算法条目

#### 1、测站帧（LidarFrame）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 11 | Root 体素描述 | VoxelRoot | 公共 | 角度深度筛选层路径与状态 |
| 12 | 子体素描述 | SubVoxel | 公共 | 如 voxel_3 边长与点数 |
| 13 | 状态位标志 | StateType / stateHas | 公共 | VALID/HAS_CALCU 等位标志，禁止裸 == VALID |

#### 2、配准边结果（AlignResult）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 14 | RMSE 米制约定 | rms_ / final_rms_ | 公共 | 已开方 RMSE [m]；禁止未开方旧尺度 |
| 15 | SE(3) 信息矩阵 | information_ | 公共 | 6×6 Ω，布局 [t\|r] |
