---
name: algo-verification
description: Use when planning or reviewing point-cloud algorithm validation, regression baselines, reproducibility evidence, or algorithm completion claims.
---

# 算法验证门禁（V&V）

先读 [统一治理规范](../../../agent-rnd/GOVERNANCE.md) 和当前任务的批准设计/冻结协议。R 决定审批，L 决定验证覆盖，不能按数字映射；R2/R3 必需证据不得以低 L 豁免。原六维内容继续作为本库验证规范源。

## 协议执行与证据有效性

- R2/R3 先核验独立评审及设计/协议的人工批准；作者自审和 Agent 建议不构成人工批准。
- 数据、基线、参数/选参、指标/单位/阈值、环境/硬件/线程、重复/种子、失败策略须冻结。结果后修改须保留原结果、协议升版并重新批准。
- 每项记录实际命令、退出码、时间、源码提交及 dirty 差异/未跟踪输入哈希、数据版本、参数、环境、原始日志和指标。代码/配置变更后旧证据标记 STALE 并重跑。
- 状态分为 NOT_RUN/PASS/FAIL/BLOCKED/INCONCLUSIVE；未运行就是 NOT_RUN，缺必需数据/环境为 BLOCKED。旧 exe 或历史日志不能证明当前版本通过；区分环境失败与算法失败。
- 执行范围须获得任务授权；需要改实现交回主 Agent，不改阈值、测试预期或隐藏失败。结果交独立 evidence-auditor 核验，再由指定负责人验收。

权威规格：[`docs/superpowers/specs/2026-10-26-algo-verification-gates-design.md`](../../../docs/superpowers/specs/2026-10-26-algo-verification-gates-design.md)  
流程闸口 Rule：[`.cursor/rules/algo-dev-gates.mdc`](../../rules/algo-dev-gates.mdc)  
命令新鲜度：先完成本 Skill 证据，再遵守 `verification-before-completion`。

## When to use

- Plan / Mini-PRD / 全闸口 PM·PMO：写 R 治理风险、L 验证强度与验证计划
- 算法 Task 收尾或声称「配准/点云改动完成」
- 用户说「算法验证门禁」「方案审」「六维」

## 验证强度（先判 R 治理风险，再独立选择 L）

| 级 | 典型 | 必过 |
|----|------|------|
| L0 | 不改变算法语义的日志/注释/文档 | 正确性子集 + 已有冒烟 |
| L1 | 局部实现/默认阈值/后端对照 | 正确性 + 精度 + 回归 + 可复现 |
| L2 | 新模块/数据流/坐标系/挂点/跨 API | 六维；鲁棒性 ≥2 应力轴 |
| L3 | 板端默认路径/发版前/改回归集约定 | 六维 + 性能实跑 + 固定集趋势 |

不确定 → **上调一级**。专用回归 Rule 优先于下表。

## 六维与证据（禁止「看起来 OK」）

| 维度 | 方案审 | 收尾验 | 证据 |
|------|--------|--------|------|
| 正确性 | 坐标系/变换；数值稳定；退化策略 | 空云/NaN/Inf/秩亏/稀疏；明确失败 | 测试命令 + assert/log |
| 精度 | 基线或参考指标；GT 不确定度 | 位姿误差或 similarity/RMS/inlier；成功率 | 对照表 |
| 性能 | 耗时/内存预算 | 墙钟、线程简述、峰值内存（L2+；L3 实跑） | EFFICIENCY / 内存日志 |
| 鲁棒性 | 应力轴声明 | 覆盖声明子集；失败可检出 | 工况矩阵 |
| 回归性 | 固定集/参数/基线位置 | 同配置复跑与趋势 | 路径 + 基线 + 摘要 |
| 可复现性 | 如何记录版本环境 | git、输入、参数、OS/构建、日志 | 复现卡片 |

点云/配准细节：读 [`references/point-cloud-registration-vv.md`](references/point-cloud-registration-vv.md)。  
SLAM（二期）：[`references/slam-vv.md`](references/slam-vv.md)。

## 收尾顺序

1. 宣布 L 级（或用户指定）
2. 按级别列出已有/缺失证据；缺则补跑（平台见 `dual-platform-git` / `ros2-build-test`）
3. 再执行 `verification-before-completion`（新鲜命令）
4. 无对应证据 → **不得**声称完成

## Plan / Mini-PRD 必填（L1+）

- R0–R3 治理风险及理由；L0–L3 验证强度及理由
- 度量（成功阈值或本库质量指标）
- 退化策略
- 验证数据集 / 应力轴
- 有无绝对 GT
