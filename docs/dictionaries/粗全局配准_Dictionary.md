# 粗全局配准_Dictionary

覆盖模块：`robust_global_reg`（依赖 `point_cloud_io`、`robust_matcher` 等）。

**产物约定**：`robust_global_reg` 为 **STATIC**（模块 CMake 约定）。`point_cloud_io` 为 **SHARED**（仓库默认）。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_global_reg | robust_matcher | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 粗匹配核心 |
| robust_global_reg | point_cloud_io | 【点云读写】(point_cloud_io) | 仓库内 | LAS/PCD/PCSE 等加载 |
| robust_global_reg | common / logging / file_system | Common / LogManager | 仓库内 | 工具、日志、路径 |
| robust_global_reg | Eigen / RapidJSON | — | 外部 | 位姿与参数 JSON |
| point_cloud_io | logging / PCL / PDAL | pdalcpp | 仓库内/外部 | 点云读写后端 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 Session） | robust_global_reg | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 粗全局配准编排迁入后回填 |

---

## 模块：robust_global_reg

依赖摘要：STATIC；链 robust_matcher、point_cloud_io、common、logging、file_system。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 鲁棒全局配准 | Ddx::RobustGlobalReg | robust_global_reg.hpp/.cpp | 公共 | 站间粗全局配准入口（加载点云 + RobustMatcher） |
| 2 | 粗全局配准参数 | RobGlobRegParameter | robGlob_reg_parameter.hpp/.cpp | 公共 | 配置读写与配准参数 |
| 3 | 粗配平移检查 | t_coarse_check | t_coarse_check.h/.cpp | 私有 | 粗配平移合理性检查 |
| 4 | 管线重叠辅助 | pipeline_overlap | pipeline_overlap.hpp | 忽略 | 联调测试用重叠辅助头 |

### 文件内小算法条目

#### 1、鲁棒全局配准（RobustGlobalReg）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 运行粗配准 | RobustGlobalReg::run | 公共 | 执行配准主流程 |
| 6 | 从文件设参 | setParameterFromFile | 公共 | JSON 配置加载 |

#### 2、粗全局配准参数（RobGlobRegParameter）

待补充（需结合业务确认）

#### 3、粗配平移检查（t_coarse_check）

待补充（需结合业务确认）

#### 4、管线重叠辅助（pipeline_overlap）

待补充（需结合业务确认）

---

## 模块：point_cloud_io

依赖摘要：SHARED；logging、PCL、PDAL（pdalcpp）、Eigen、RapidJSON。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 7 | 点云读写 | Ddx::point_cloud_io | point_cloud_io.hpp/.cpp | 公共 | 多格式点云读/写与格式探测 |
| 8 | LAS 快读 | las_fast_reader | las_fast_reader.hpp/.cpp | 公共 | LAS/SLAS 快速读取路径 |
| 9 | PCSE 读取 | pcse_reader | pcse_reader.hpp/.cpp | 公共 | PCSE 数据集读取 |
| 10 | 点云流式转换 | point_cloud_stream_convert | point_cloud_stream_convert.hpp/.cpp | 公共 | 流式格式转换 |

### 文件内小算法条目

#### 7、点云读写（point_cloud_io）

待补充（需结合业务确认）

#### 8、LAS 快读（las_fast_reader）

待补充（需结合业务确认）

#### 9、PCSE 读取（pcse_reader）

待补充（需结合业务确认）

#### 10、点云流式转换（point_cloud_stream_convert）

待补充（需结合业务确认）
