# robust_global_reg 点云 I/O 外置重构

**状态**：已实施

## 定案

- 纯内存入口（打破旧契约）：公共 `run` 只接收已准备好的点，库内删除 `loadCloud` 与对 `point_cloud_io` 的链接。
- 输入类型：`const std::vector<Eigen::Vector3f>&`（与现有 `_run` 一致）。
- 坐标系契约：调用方完成局部化后再传入；推荐经 `point_cloud_io` 得 `FramedCloud` 后用 `.cloud`（或再 rebase 到作业 WorkingFrame）。模块不接收原点、不减偏移。旧「仅减 LAS header offset」不当工作系（见 FramedCloud IO 规格）。

## 消费者输入契约

**调用方（消费者）全权管理输入点云的坐标系与语义；模块不做加载、不做站心推断、不校验站心是否真在原点。**

| 场景 | 调用方必须保证 |
|------|----------------|
| 通用粗全局配准 | src/tgt 已在同一约定工作系（通常已做 WorkingFrame：`p' = p - O`） |
| `use_compass == true`（罗盘约束） | **必须是站心坐标系数据**：测站原点（站心）在 `(0, 0, 0)`；点坐标相对该站心。非站心 / 大坐标 / 仅减 LAS header offset 而未对齐站心，罗盘语义无效 |

落点：头文件 `run` 注释、`RobGlobRegParameter::use_compass` 注释、字典与本归档。

## API

- `bool run(const std::vector<Eigen::Vector3f>& src, const std::vector<Eigen::Vector3f>& tgt)`
- 参数：删除 `source_file` / `target_file`；可选 `scan_name`；保留 `result_file` / `log_file` 与算法开关。
- 库 CMake：去掉 `point_cloud_io`；测试自行链接并读盘。

## 非目标

- 不改 RobustMatcher 算法。
- 不在本模块实现 WorkingFrame / 站心推断。
- 不保留文件路径 `run()` 兼容包装。

## 相关

- 字典：[`../dictionaries/粗全局配准_Dictionary.md`](../dictionaries/粗全局配准_Dictionary.md)
- 模块：[`../modules/粗全局配准.md`](../modules/粗全局配准.md)
- WorkingFrame：[`局部工作坐标系.md`](局部工作坐标系.md)
