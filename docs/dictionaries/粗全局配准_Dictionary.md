# 粗全局配准_Dictionary

覆盖模块：`robust_global_reg`（站间粗全局配准）。  
公共点云 IO 见独立领域：[`点云IO_Dictionary.md`](点云IO_Dictionary.md)（`point_cloud_io` / `FramedCloud`）。

**产物约定**：`robust_global_reg` 为 **STATIC**（模块 CMake 约定）。库本身**不**链接 `point_cloud_io`。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| robust_global_reg | robust_matcher | 【鲁棒匹配器】(RobustMatcher) | 仓库内 | 粗匹配核心 |
| robust_global_reg | common / logging / file_system | Common / LogManager | 仓库内 | 工具、日志、路径 |
| robust_global_reg | Eigen / RapidJSON | — | 外部 | 位姿与参数 JSON |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| （待上层 Session / 测试） | robust_global_reg | 【鲁棒全局配准】(RobustGlobalReg) | 仓库内 | 调用方先加载并局部化点云再 `run(src,tgt)` |

### 与公共 IO 的衔接（跨领域）

| 调用方 | 被调用方 | 关键衔接点 | 说明 |
|--------|----------|------------|------|
| 测试 / 业务加载侧 | point_cloud_io | 【FramedCloud】 | 读盘得局部云+origin，再交给本模块 `run`；详见点云 IO 字典 |

---

## 消费者输入契约

调用方全权管理输入点云的坐标系与语义；模块不做加载、不做站心推断、不校验站心是否在原点。

推荐经公共库 [`point_cloud_io`](点云IO_Dictionary.md) 得 `FramedCloud`，rebase 到同一作业系后取 `.cloud`（或等价局部点）再 `run`。配准位姿折回 / 初始阵入局部系见 `.cursor/rules/framed-cloud-pose-origin.mdc`。

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
