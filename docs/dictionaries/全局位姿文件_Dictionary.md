# 全局位姿文件_Dictionary

覆盖模块：`global_matrix_file`（多站全局位姿 JSON 读写）。  
**产物约定**：`global_matrix_file` 为 **STATIC**（模块 CMake 约定）。

与 `Common::matrix_to_south_string`（单阵南向串）互补：本模块按工程 JSON（`Transformation[]`：name / rotation / translation）批量读写站名→`Matrix4d`。

## 模块相互依赖关系

### 本领域对外依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| global_matrix_file | common | 【公共工具集】(string_to_matrix33 等) | 仓库内 | 旋转/平移字符串编解码 |
| global_matrix_file | logging | 【日志管理器】(LogManager) | 仓库内 | 打开/解析失败日志 |
| global_matrix_file | RapidJSON / Eigen3 | rapidjson::Document / Matrix4d | 外部 | JSON 与位姿容器 |

### 本领域被谁依赖

| 调用方 | 被调用方 | 关键衔接点（标准中文命名 / 代码名） | 依赖类型 (仓库内/外部) | 一句话说明 |
|--------|----------|--------------------------------------|------------------------|------------|
| pose_graph_tool（上游产出） | global_matrix_file | 【全局位姿文件读写】(GlobalMatrixFileIO) | 仓库内 | PoseGlobalSystem 的 map 可经本模块落盘（调用方接线） |
| （待上层 Session） | global_matrix_file | 【全局位姿文件读写】(GlobalMatrixFileIO) | 仓库内 | Session 挂接后回填 |

---

## 模块：global_matrix_file

依赖摘要：`common`、`logging`、RapidJSON、Eigen3。

### 文件级算法条目

| 序号 | 标准中文命名 | 核心代码函数/类名 | 来源 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|------|---------------------------|------------|
| 1 | 全局位姿文件读写 | GlobalMatrixFileIO | global_matrix_file_io.hpp/.cpp | 公共 | 站名→全局 `Matrix4d` 的 JSON loading/saving |
| 2 | 位姿 JSON 基类读写 | BaseMatrixFileIO | global_matrix_file_io.cpp | 私有 | Transformation 数组与单阵南向串辅助 |

### 文件内小算法条目

#### 1、全局位姿文件读写（GlobalMatrixFileIO）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 3 | 加载全局位姿表 | GlobalMatrixFileIO::loading | 公共 | 读 JSON → `map<name, Matrix4d>` |
| 4 | 保存全局位姿表 | GlobalMatrixFileIO::saving | 公共 | `map` → Pretty JSON（含 version） |

#### 2、位姿 JSON 基类读写（BaseMatrixFileIO）

| 序号 | 标准中文命名 | 核心代码函数/类名 | 所属模块 (私有/公共/忽略) | 一句话简述 |
|------|--------------|-------------------|---------------------------|------------|
| 5 | 解析单站位姿对象 | BaseMatrixFileIO::loadMatrix(Value&, name, T) | 私有 | name + rotation + translation |
| 6 | 写出单站位姿对象 | BaseMatrixFileIO::saveMatrix(Value&, …) | 私有 | 写入 name/rotation/translation |
| 7 | 单阵南向串读写 | writeMatrix / readMatrix | 私有 | `transformation` 字段用南向位姿串 |
