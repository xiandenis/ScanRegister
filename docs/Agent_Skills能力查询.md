# Agent Skills 能力查询手册

面向本仓库已安装技能的**场景检索 + 能力速查**。  
权威清单：根目录 [`skills-lock.json`](../skills-lock.json)；Agent 调用名见 [`AGENTS.md`](../AGENTS.md)。

| 项 | 说明 |
|----|------|
| 技能正文 | `.claude/skills/<调用名>/SKILL.md`（与 `.agents/skills/` 同内容） |
| 锁文件显示名 | `skills-lock.json` 里的英文 Title |
| 调用名 | `AGENTS.md` / 文件夹名（kebab-case） |
| 如何启用 | 对 Agent 说需求，或明确「用 `<调用名>` skill」；亦可 `npx openskills read <调用名>` |

> 本仓库另有 Cursor 项目技能 [`board-ros2-debug`](../.cursor/skills/board-ros2-debug/SKILL.md)（开发板 ROS2 联调），**不在** `skills-lock.json` 内，见文末。

---

## 1. 按你想做的事查（场景索引）

| 你想… | 优先用这些技能 |
|--------|----------------|
| **不知道用哪个技能 / 刚上手 Skills** | `getting-started-with-skills` → 卡住时 `when-stuck-problem-solving-dispatch` |
| **把模糊想法谈清楚再动手** | `brainstorming-ideas-into-designs` |
| **写可执行的分步实现计划** | `writing-plans` |
| **按计划一批批落地，带检查点** | `executing-plans` |
| **每个任务派子代理，任务间做评审** | `subagent-driven-development` |
| **多个独立问题并行查/修** | `dispatching-parallel-agents` |
| **合并前先做代码评审** | `requesting-code-review` |
| **处理别人给的 review 意见（别瞎改）** | `code-review-reception` |
| **功能做完：合并 / PR / 清理分支** | `finishing-a-development-branch` |
| **隔离实验分支（worktree）** | `using-git-worktrees` |
| **出 bug：先找根因再改** | `systematic-debugging` → `root-cause-tracing` |
| **声称「做完了」前必须跑验证** | `verification-before-completion` |
| **多层校验，让非法数据进不来** | `defense-in-depth-validation` |
| **先写测试再写实现（TDD）** | `test-driven-development-tdd` |
| **异步测试别用瞎 sleep** | `condition-based-waiting` |
| **测错了：别测 mock、别加测试专用生产 API** | `testing-anti-patterns` |
| **思路卡死，要换解题角度** | `when-stuck-problem-solving-dispatch`，再按需：`inversion-exercise` / `collision-zone-thinking` / `scale-game` / `simplification-cascades` / `meta-pattern-recognition` |
| **架构上两种方案都有道理，别过早二选一** | `preserving-productive-tensions` |
| **查历史对话里的决策/事实** | `remembering-conversations` |
| **追一个想法/方案是怎么演变来的** | `tracing-knowledge-lineages` |
| **写 SRS / PRD / OpenAPI / 手册 / Runbook / 图** | `documentation-specialist` |
| **从 git 历史生成用户可读 changelog** | `changelog-generator` |
| **做海报 / 静态视觉稿（png/pdf）** | `canvas-design` |
| **做复杂多页 React + Tailwind + shadcn 产物** | `artifacts-builder` |
| **产品 UI / 前端观感要有辨识度** | `frontend-design` |
| **搭 MCP Server（Python/TS）** | `mcp-builder` |
| **新建/改进一个 Skill** | `skill-creator` 或更严格的 `writing-skills` |
| **用子代理压测 Skill 是否好用** | `testing-skills-with-subagents` |
| **把技能贡献回上游 / 维护技能库** | `sharing-skills` / `gardening-skills-wiki` / `pulling-updates-from-skills-repository` |

---

## 2. 按类别浏览

### 2.1 开发流程（计划 → 实现 → 收尾）

| 调用名 | 能干什么 | 典型说法 |
|--------|----------|----------|
| `brainstorming-ideas-into-designs` | 用追问把想法收敛成完整设计，避免一上来就写代码 | 「帮我把这个功能 brainstorm 清楚」 |
| `writing-plans` | 写出零上下文也能跟做的咬口级实现计划 | 「写一份实现计划」 |
| `executing-plans` | 按计划分批执行，批次间设审查点 | 「按这份计划执行」 |
| `subagent-driven-development` | 每任务新开子代理实现，任务间做 code review | 「用子代理按任务推进」 |
| `dispatching-parallel-agents` | 互不依赖的问题并行派多个 Agent | 「这几个问题并行查」 |
| `requesting-code-review` | 对照计划/需求派 reviewer，过关再继续 | 「实现完先做 code review」 |
| `code-review-reception` | 严谨消化 review：验证、反驳或正确改，禁止表演式同意 | 「按 review 意见处理」 |
| `finishing-a-development-branch` | 功能完成后结构化选：合并、开 PR 或清理 | 「这个功能收尾」 |
| `using-git-worktrees` | 安全建隔离 worktree，避免弄脏主工作区 | 「单独开个 worktree 试」 |
| `verification-before-completion` | 完成前必须跑验证命令并看真实输出，禁止空口说成功 | 「确认真的做完了」 |

### 2.2 调试与质量

| 调用名 | 能干什么 | 典型说法 |
|--------|----------|----------|
| `systematic-debugging` | 四阶段调试：先根因调查，禁止上来就猜补丁 | 「系统排查这个 bug」 |
| `root-cause-tracing` | 从症状沿调用栈/数据流往回追到最初触发点 | 「追根因，别只治表象」 |
| `defense-in-depth-validation` | 数据经过的每一层都校验，让坏数据无法穿透 | 「加纵深校验」 |
| `test-driven-development-tdd` | 先失败测试 → 最小实现 → 再整理 | 「用 TDD 做」 |
| `condition-based-waiting` | 异步等待改为条件轮询，替代固定 timeout | 「别用 sleep，等条件成立」 |
| `testing-anti-patterns` | 避免：测 mock、给生产类加仅测试用方法、盲目 mock | 「检查测试是不是反模式」 |

### 2.3 解题与架构思维

| 调用名 | 能干什么 | 典型说法 |
|--------|----------|----------|
| `when-stuck-problem-solving-dispatch` | 根据「卡法」分流到合适的解题技巧 | 「我卡住了，帮我选方法」 |
| `inversion-exercise` | 把核心假设反过来，暴露隐藏约束与新路径 | 「反过来想会怎样」 |
| `collision-zone-thinking` | 强行把无关概念碰撞：若把 X 当成 Y 会怎样 | 「用碰撞思维想一想」 |
| `scale-game` | 拉到极端尺度（×1000 / 瞬时 / 一年）暴露真约束 | 「放到极端规模看看」 |
| `simplification-cascades` | 找一个洞察能消掉多个组件/步骤 | 「能不能一刀砍掉一堆复杂度」 |
| `meta-pattern-recognition` | 跨 ≥3 个领域找同构模式与通用原则 | 「这像不像别的领域的模式」 |
| `preserving-productive-tensions` | 保留多种合理方案与张力，避免过早唯一解 | 「两种方案都先保留」 |
| `tracing-knowledge-lineages` | 追想法/方案的知识谱系，复用旧解、少重复踩坑 | 「这招以前怎么演化来的」 |

### 2.4 文档、发布与产品呈现

| 调用名 | 能干什么 | 典型说法 |
|--------|----------|----------|
| `documentation-specialist` | 专业文档全流程：SRS/PRD/OpenAPI、手册、教程、Runbook；从代码反推文档；审计；MD↔DOCX/PDF；C4/时序等图 | 「写一份 SRS / 用户手册 / OpenAPI」 |
| `changelog-generator` | 从 commit 归类并写成用户可读发布说明 | 「根据提交生成 changelog」 |
| `frontend-design` | 有辨识度的前端视觉与版式指导（非套模板感） | 「把这个页面 UI 做好看」 |
| `artifacts-builder` | 复杂多组件 HTML 产物：React + Tailwind + shadcn（非单文件小玩具） | 「做一个带路由/状态的交互 artifact」 |
| `canvas-design` | 按设计哲学做原创海报/静态视觉（png/pdf），不抄袭既有画风 | 「做一张海报 / 视觉稿」 |

### 2.5 MCP、Skills 生态与元技能

| 调用名 | 能干什么 | 典型说法 |
|--------|----------|----------|
| `mcp-builder` | 设计高质量 MCP Server（FastMCP / TS MCP SDK）与工具面 | 「搭一个 MCP 接某某 API」 |
| `skill-creator` | 创建或更新 Skill（知识、流程、工具集成） | 「帮我新建一个 skill」 |
| `writing-skills` | 用 TDD 思路写流程类 Skill：先压测再写到闭环 | 「把这个流程写成可靠 skill」 |
| `testing-skills-with-subagents` | 用子代理对 Skill 做 RED-GREEN-REFACTOR 压测 | 「压测一下这个 skill」 |
| `getting-started-with-skills` | Skills 入门：强制工作流、检索、brainstorm 触发点 | 「Skills 怎么用」 |
| `gardening-skills-wiki` | 维护技能 wiki：链接、命名、交叉引用、覆盖度 | 「整理一下 skills wiki」 |
| `pulling-updates-from-skills-repository` | 从上游 `obra/superpowers-skills` 同步更新 | 「拉取技能库更新」 |
| `sharing-skills` | 把本地技能贡献回上游（分支 + PR） | 「把这个 skill 贡献上去」 |
| `remembering-conversations` | 语义/文本检索历史 Claude Code 对话中的事实与决策 | 「以前我们是怎么定的」 |

---

## 3. 全量对照表（锁文件 ↔ 调用名 ↔ 来源）

| skills-lock 显示名 | 调用名（文件夹） | 来源仓库 |
|--------------------|------------------|----------|
| Brainstorming Ideas Into Designs | `brainstorming-ideas-into-designs` | obra/superpowers-skills |
| Code Review Reception | `code-review-reception` | obra/superpowers-skills |
| Collision-Zone Thinking | `collision-zone-thinking` | obra/superpowers-skills |
| Condition-Based Waiting | `condition-based-waiting` | obra/superpowers-skills |
| Defense-in-Depth Validation | `defense-in-depth-validation` | obra/superpowers-skills |
| Dispatching Parallel Agents | `dispatching-parallel-agents` | obra/superpowers-skills |
| Executing Plans | `executing-plans` | obra/superpowers-skills |
| Finishing a Development Branch | `finishing-a-development-branch` | obra/superpowers-skills |
| Gardening Skills Wiki | `gardening-skills-wiki` | obra/superpowers-skills |
| Getting Started with Skills | `getting-started-with-skills` | obra/superpowers-skills |
| Inversion Exercise | `inversion-exercise` | obra/superpowers-skills |
| Meta-Pattern Recognition | `meta-pattern-recognition` | obra/superpowers-skills |
| Preserving Productive Tensions | `preserving-productive-tensions` | obra/superpowers-skills |
| Pulling Updates from Skills Repository | `pulling-updates-from-skills-repository` | obra/superpowers-skills |
| Remembering Conversations | `remembering-conversations` | obra/superpowers-skills |
| Requesting Code Review | `requesting-code-review` | obra/superpowers-skills |
| Root Cause Tracing | `root-cause-tracing` | obra/superpowers-skills |
| Scale Game | `scale-game` | obra/superpowers-skills |
| Sharing Skills | `sharing-skills` | obra/superpowers-skills |
| Simplification Cascades | `simplification-cascades` | obra/superpowers-skills |
| Subagent-Driven Development | `subagent-driven-development` | obra/superpowers-skills |
| Systematic Debugging | `systematic-debugging` | obra/superpowers-skills |
| Test-Driven Development (TDD) | `test-driven-development-tdd` | obra/superpowers-skills |
| Testing Anti-Patterns | `testing-anti-patterns` | obra/superpowers-skills |
| Testing Skills With Subagents | `testing-skills-with-subagents` | obra/superpowers-skills |
| Tracing Knowledge Lineages | `tracing-knowledge-lineages` | obra/superpowers-skills |
| Using Git Worktrees | `using-git-worktrees` | obra/superpowers-skills |
| Verification Before Completion | `verification-before-completion` | obra/superpowers-skills |
| When Stuck - Problem-Solving Dispatch | `when-stuck-problem-solving-dispatch` | obra/superpowers-skills |
| Writing Plans | `writing-plans` | obra/superpowers-skills |
| Writing Skills | `writing-skills` | obra/superpowers-skills |
| artifacts-builder | `artifacts-builder` | ComposioHQ/awesome-claude-skills |
| canvas-design | `canvas-design` | ComposioHQ/awesome-claude-skills |
| changelog-generator | `changelog-generator` | ComposioHQ/awesome-claude-skills |
| documentation-specialist | `documentation-specialist` | SpillwaveSolutions/document-specialist-skill |
| frontend-design | `frontend-design` | anthropics/skills |
| mcp-builder | `mcp-builder` | ComposioHQ/awesome-claude-skills |
| skill-creator | `skill-creator` | ComposioHQ/awesome-claude-skills |

共 **38** 条（与当前 `skills-lock.json` 一致）。

---

## 4. 推荐使用姿势（本仓库日常）

### 新功能 / 较大改动

```text
brainstorming-ideas-into-designs
  → writing-plans
  → executing-plans 或 subagent-driven-development
  → requesting-code-review
  → verification-before-completion
  → finishing-a-development-branch
```

### 线上/联调 Bug

```text
systematic-debugging
  →（需要时）root-cause-tracing
  → 修复后 verification-before-completion
```

### 卡住或方案发散

```text
when-stuck-problem-solving-dispatch
  → 按分流选用 inversion / collision-zone / scale-game / simplification-cascades …
```

### 文档与发布

```text
documentation-specialist   # 规格/手册/图
changelog-generator        # 发版说明
```

### 对本仓库算法/ROS2 联调（非 lock 内技能）

| 技能 | 路径 | 何时用 |
|------|------|--------|
| `board-ros2-debug` | `.cursor/skills/board-ros2-debug/SKILL.md` | 开发板编译 ROS2、远程全量 `register_scan` 测试（`remote_dev.sh`） |

Windows 本机测试仍走 `real_time_manager_test.exe`；Ubuntu/Linux 本机走 `ros2/test_ros2.sh`（见 `.cursor/rules`）。

---

## 5. 对 Agent 怎么说（示例句）

- 「用 `writing-plans` 给小体素分辨率估计写实现计划」
- 「这个配准失败按 `systematic-debugging` 查，不要直接改参数蒙」
- 「两个独立问题用 `dispatching-parallel-agents` 并行查」
- 「实现完用 `requesting-code-review`，再 `verification-before-completion`」
- 「从最近提交用 `changelog-generator` 写一版用户向 release notes」
- 「用 `documentation-specialist` 给某某模块补一份使用说明」

详情与步骤以对应 `SKILL.md` 为准；本页只做**能力范围与选型**，不替代技能正文。

---

## 6. 维护说明

- 安装/更新技能后：同步改 `skills-lock.json`、`AGENTS.md`，并更新本页第 1–3 节。
- 上游同步：`pulling-updates-from-skills-repository`。
- 技能正文变更后，以 `SKILL.md` frontmatter 的 `description` 为准复核本页摘要。
