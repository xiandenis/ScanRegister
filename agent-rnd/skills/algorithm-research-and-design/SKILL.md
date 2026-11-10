---
name: algorithm-research-and-design
description: Use when researching algorithms, planning algorithm behavior changes, or defining experiment protocols.
---

# 算法研究与设计

先读 [治理规范](../../GOVERNANCE.md)、[索引](../../SKILLS_INDEX.md) 和当前任务记录。遵循 G0–G4，R2/R3 未获可核验批准只研究和准备方案。

1. 根据实际影响判 R，另定 L；检查仓库实现、现有测试、字典、数据和历史结果，分清存在与已验证。
2. 定义 I/O、单位/坐标系、目标、约束、失败处理与非目标；记录基线源码和观察到的问题。
3. 从现有领域材料和必要的原始来源比较候选，给出假设、复杂度、数值/内存/并行影响、退化与反例；来源标版本与适用范围。
4. 区分 ESTABLISHED/SUPPORTED/HYPOTHESIS/DISPUTED/UNKNOWN，不能把推测写成实验事实。
5. 提出数据/GT、指标、阈值及不能退化项、参数/选参、重复策略和环境的协议草案；不把猜测阈值视为产品批准。
6. R2/R3 交独立评审，处理发现，再取得设计与协议的人工批准并冻结；缺材料保留明确阻断项，不进入实现。

输出到任务记录或返回主 Agent：问题、R/L、来源、基线/替代方案、假设/反例、实验草案、未解决项及批准状态。不得自行批准、改代码或用本批治理配置授权替代算法任务批准。
