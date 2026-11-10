# 点云 / 配准验证协议（一期）

配合 Skill：`algo-verification`。规格：`docs/superpowers/specs/2026-10-26-algo-verification-gates-design.md`。

## 位姿误差

- **平移误差**：\(\|t_{\mathrm{est}} - t_{\mathrm{gt}}\|_2\)（米或工程单位，须声明）。
- **旋转误差**：SO(3) **测地距离**（角度）；不要用未处理 wrap 的欧拉差当主指标。
- **成功率**：旋转与平移同时低于阈值才计成功；失败单独计数，禁止并入 RMSE 均值。

## 数值稳定性

- 大坐标点云：metric 在相对质心（或等价稳定形式）下计算，避免手搓全局质心 RMSE 漂移。
- 矩阵/四元数：归一化与正交性在关键路径有 assert 或明确失败。

## Ground truth

- 有可靠 GT：报位姿误差 + 成功率；注明 GT 来源与已知不确定度上限。
- **无可靠 GT**：用本库相对质量指标，并在证据写明「无绝对 GT」：
  - 工程 `align_result.txt`：similarity / RMS / weight / feature_num
  - 必要时配准报告 Canvas / 报告网页（用户要求时）

## 鲁棒性应力轴（方案须声明工作区）

| 轴 | 说明 |
|----|------|
| 重叠率 | 过低重叠失败率上升；声明最小可用重叠假设 |
| 初值偏差 | 局部法对初值敏感；粗→精链路须说明 |
| 密度 / 噪声 / 稀疏 | 下采样与特征数变化 |
| 异常输入 | 空云、NaN/Inf、极少重叠 |

L2+ 至少覆盖其中 **2** 轴的验证或明确「未测」风险。

## 本库实跑挂钩

| 本机 OS | 命令入口 |
|---------|----------|
| Windows | `build\bin\Release\real_time_manager_test.exe <数据> <工程>` |
| Ubuntu/Linux | `./ros2/test_ros2.sh --data-dir … --project-dir …` |
| 开发板 | `./remote_dev.sh test-ros2 --data-dir … --project-dir …` |

性能证据优先：`EFFICIENCY REPORT`（`ros2/report_ros2_efficiency.py` 或测试脚本打印）。  
专用回归（例：`voxel-size-estimate-regression` 三组数据）**优先于**本文件通用轴。

## 复现卡片字段

- git commit / 算法版本号
- 数据目录、工程目录
- 关键参数或 Options 摘要
- OS、构建类型（Release/Debug）
- 原始日志或 node.log 路径
