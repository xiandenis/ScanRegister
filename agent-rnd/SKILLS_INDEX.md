# 治理流程索引

先读 [GOVERNANCE.md](GOVERNANCE.md)。实际任务记录放在 docs/plan/，使用 [模板](task-template/ALGORITHM_TASK.md)，可链接已有计划而不重复复制。

| 场景 | 规范源 | Cursor 入口 |
|---|---|---|
| 算法研究、行为变化与实验方案 | [研究设计](skills/algorithm-research-and-design/SKILL.md) | algorithm-research-and-design |
| R2/R3 独立方案评审 | [独立评审](skills/independent-algorithm-review/SKILL.md) | independent-algorithm-review |
| 六维验证、协议执行及证据 | [现有 algo-verification](../.cursor/skills/algo-verification/SKILL.md) | algo-verification |

首批复用 algo-verification，不另建平行的 algorithm-experiment-and-verification 流程。现有 .agents/skills 通用技能继续可用，旧 AGENTS 索引不是排他白名单；直接读取已存在的 SKILL.md，不为读文件自动运行 npx、安装或联网。

## 角色路由

- domain-researcher：只读研究，输出来源、假设、反例及未知项。
- algorithm-reviewer：只读独立评审，输出具体发现和建议；不能代替人工批准。
- test-runner：只运行授权且已核实的命令，保存真实日志，不能修实现或阈值。
- evidence-auditor：只读核验实际 diff、协议、批准与证据一致性。

项目角色定义位于 .cursor/agents/。缺少该运行环境的委派能力时明确记录，不伪装已经委派；R2/R3 不得用作者自审替代独立评审。首批不扩展 SLAM/机器人/AI 知识包，不要求读取尚不存在的空骨架。
领域材料沿用 docs/dictionaries/、专项 .cursor/rules/ 及 algo-verification/references/；逐项记录来源、适用性、版本、证据与审核状态。
