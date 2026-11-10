---
name: independent-algorithm-review
description: Use when independently reviewing an R2 or R3 algorithm design and its experiment protocol.
---

# 独立算法评审

先读 [治理规范](../../GOVERNANCE.md)、实际设计及协议版本、当前任务记录和相关源码/证据。作者不能担任本次独立评审者，材料不足即说明；不得只读作者总结。

检查问题定义、推导/假设、单位/坐标系、反例和退化、数值稳定性、复杂度/内存/线程、公平基线、实验区分力、指标与产品目标、不可接受回归、可复现性及证据边界。
每个发现给出 BLOCKER/HIGH/MEDIUM/LOW、实际证据、影响和修正建议；未解决阻断项优先列出。
最终建议只能是 APPROVED、CONDITIONALLY_APPROVED、REJECTED、NEEDS_MORE_EVIDENCE。建议不是人工批准，不修改设计、源码、阈值或批准记录；转给指定负责人决定。
结果审计另由 evidence-auditor 检查实际版本、diff、日志、退出码及协议，不以设计评审代替结果审计。
