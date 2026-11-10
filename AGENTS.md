# AGENTS

## 项目治理入口（首批配置）

非平凡任务先读 [统一治理规范](agent-rnd/GOVERNANCE.md)、[治理 Skills 索引](agent-rnd/SKILLS_INDEX.md)、当前 docs/plan/ 任务记录及相关实现/测试。项目流程冲突以统一规范为准；领域专项约束可加强，不能降低审批要求。运行环境权限与当前用户授权仍须遵守。

- R0–R3 是治理风险，L0–L3 是验证强度，分开记录。bug、单行、调参和“继续”不自动豁免 R2/R3 审批。
- R2/R3 未核验独立评审及设计/冻结协议的人工批准，只能研究或准备方案，标记 BLOCKED_FOR_IMPLEMENTATION。
- 作者自审、Agent 的 APPROVED、技术偏好二次确认均不能代替人工批准；协议修订须升版、保留原结果并重新批准。
- 结果仅用 NOT_RUN/PASS/FAIL/BLOCKED/INCONCLUSIVE；旧 exe 和历史日志不证明当前版本通过。
- 角色入口见 .cursor/agents/；实际加载、CI/分支保护、Hooks 与平台权限均未在首批验证，不得声称已经生效。
- 以下旧 Skills 表保留兼容；不是排他白名单，新增治理索引和现有 .cursor/skills 同样可用。直接读取本地文件，不为读取自动安装工具或联网。


<skills_system priority="1">

## Available Skills

<!-- SKILLS_TABLE_START -->
<usage>
When users ask you to perform tasks, check if any of the available skills below can help complete the task more effectively. Skills provide specialized capabilities and domain knowledge.

How to use skills:
- Invoke: read the existing local SKILL.md directly; use agent-rnd/SKILLS_INDEX.md first.
  - For multiple: read each relevant local entry once; no installation or network access is needed.
- The skill content will load with detailed instructions on how to complete the task
- Base directory provided in output for resolving bundled resources (references/, scripts/, assets/)

Usage notes:
- The legacy <available_skills> list below is not exclusive; also use agent-rnd/SKILLS_INDEX.md and existing project skills.
- Do not invoke a skill that is already loaded in your context
- Each skill invocation is stateless
</usage>

<available_skills>

<skill>
<name>artifacts-builder</name>
<description>Suite of tools for creating elaborate, multi-component claude.ai HTML artifacts using modern frontend web technologies (React, Tailwind CSS, shadcn/ui). Use for complex artifacts requiring state management, routing, or shadcn/ui components - not for simple single-file HTML/JSX artifacts.</description>
<location>project</location>
</skill>

<skill>
<name>brainstorming-ideas-into-designs</name>
<description>Interactive idea refinement using Socratic method to develop fully-formed designs</description>
<location>project</location>
</skill>

<skill>
<name>canvas-design</name>
<description>Create beautiful visual art in .png and .pdf documents using design philosophy. You should use this skill when the user asks to create a poster, piece of art, design, or other static piece. Create original visual designs, never copying existing artists' work to avoid copyright violations.</description>
<location>project</location>
</skill>

<skill>
<name>changelog-generator</name>
<description>Automatically creates user-facing changelogs from git commits by analyzing commit history, categorizing changes, and transforming technical commits into clear, customer-friendly release notes. Turns hours of manual changelog writing into minutes of automated generation.</description>
<location>project</location>
</skill>

<skill>
<name>code-review-reception</name>
<description>Receive and act on code review feedback with technical rigor, not performative agreement or blind implementation</description>
<location>project</location>
</skill>

<skill>
<name>collision-zone-thinking</name>
<description>Force unrelated concepts together to discover emergent properties - "What if we treated X like Y?"</description>
<location>project</location>
</skill>

<skill>
<name>condition-based-waiting</name>
<description>Replace arbitrary timeouts with condition polling for reliable async tests</description>
<location>project</location>
</skill>

<skill>
<name>defense-in-depth-validation</name>
<description>Validate at every layer data passes through to make bugs impossible</description>
<location>project</location>
</skill>

<skill>
<name>dispatching-parallel-agents</name>
<description>Use multiple Claude agents to investigate and fix independent problems concurrently</description>
<location>project</location>
</skill>

<skill>
<name>documentation-specialist</name>
<description>|</description>
<location>project</location>
</skill>

<skill>
<name>executing-plans</name>
<description>Execute detailed plans in batches with review checkpoints</description>
<location>project</location>
</skill>

<skill>
<name>finishing-a-development-branch</name>
<description>Complete feature development with structured options for merge, PR, or cleanup</description>
<location>project</location>
</skill>

<skill>
<name>frontend-design</name>
<description>Guidance for distinctive, intentional visual design when building new UI or reshaping an existing one. Helps with aesthetic direction, typography, and making choices that don't read as templated defaults.</description>
<location>project</location>
</skill>

<skill>
<name>gardening-skills-wiki</name>
<description>Maintain skills wiki health - check links, naming, cross-references, and coverage</description>
<location>project</location>
</skill>

<skill>
<name>getting-started-with-skills</name>
<description>Skills wiki intro - mandatory workflows, search tool, brainstorming triggers</description>
<location>project</location>
</skill>

<skill>
<name>inversion-exercise</name>
<description>Flip core assumptions to reveal hidden constraints and alternative approaches - "what if the opposite were true?"</description>
<location>project</location>
</skill>

<skill>
<name>mcp-builder</name>
<description>Guide for creating high-quality MCP (Model Context Protocol) servers that enable LLMs to interact with external services through well-designed tools. Use when building MCP servers to integrate external APIs or services, whether in Python (FastMCP) or Node/TypeScript (MCP SDK).</description>
<location>project</location>
</skill>

<skill>
<name>meta-pattern-recognition</name>
<description>Spot patterns appearing in 3+ domains to find universal principles</description>
<location>project</location>
</skill>

<skill>
<name>preserving-productive-tensions</name>
<description>Recognize when disagreements reveal valuable context, preserve multiple valid approaches instead of forcing premature resolution</description>
<location>project</location>
</skill>

<skill>
<name>pulling-updates-from-skills-repository</name>
<description>Sync local skills repository with upstream changes from obra/superpowers-skills</description>
<location>project</location>
</skill>

<skill>
<name>remembering-conversations</name>
<description>Search previous Claude Code conversations for facts, patterns, decisions, and context using semantic or text search</description>
<location>project</location>
</skill>

<skill>
<name>requesting-code-review</name>
<description>Dispatch code-reviewer subagent to review implementation against plan or requirements before proceeding</description>
<location>project</location>
</skill>

<skill>
<name>root-cause-tracing</name>
<description>Systematically trace bugs backward through call stack to find original trigger</description>
<location>project</location>
</skill>

<skill>
<name>scale-game</name>
<description>Test at extremes (1000x bigger/smaller, instant/year-long) to expose fundamental truths hidden at normal scales</description>
<location>project</location>
</skill>

<skill>
<name>sharing-skills</name>
<description>Contribute skills back to upstream via branch and PR</description>
<location>project</location>
</skill>

<skill>
<name>simplification-cascades</name>
<description>Find one insight that eliminates multiple components - "if this is true, we don't need X, Y, or Z"</description>
<location>project</location>
</skill>

<skill>
<name>skill-creator</name>
<description>Guide for creating effective skills. This skill should be used when users want to create a new skill (or update an existing skill) that extends Claude's capabilities with specialized knowledge, workflows, or tool integrations.</description>
<location>project</location>
</skill>

<skill>
<name>subagent-driven-development</name>
<description>Execute implementation plan by dispatching fresh subagent for each task, with code review between tasks</description>
<location>project</location>
</skill>

<skill>
<name>systematic-debugging</name>
<description>Four-phase debugging framework that ensures root cause investigation before attempting fixes. Never jump to solutions.</description>
<location>project</location>
</skill>

<skill>
<name>test-driven-development-tdd</name>
<description>Write the test first, watch it fail, write minimal code to pass</description>
<location>project</location>
</skill>

<skill>
<name>testing-anti-patterns</name>
<description>Never test mock behavior. Never add test-only methods to production classes. Understand dependencies before mocking.</description>
<location>project</location>
</skill>

<skill>
<name>testing-skills-with-subagents</name>
<description>RED-GREEN-REFACTOR for process documentation - baseline without skill, write addressing failures, iterate closing loopholes</description>
<location>project</location>
</skill>

<skill>
<name>tracing-knowledge-lineages</name>
<description>Understand how ideas evolved over time to find old solutions for new problems and avoid repeating past failures</description>
<location>project</location>
</skill>

<skill>
<name>using-git-worktrees</name>
<description>Create isolated git worktrees with smart directory selection and safety verification</description>
<location>project</location>
</skill>

<skill>
<name>verification-before-completion</name>
<description>Run verification commands and confirm output before claiming success</description>
<location>project</location>
</skill>

<skill>
<name>when-stuck-problem-solving-dispatch</name>
<description>Dispatch to the right problem-solving technique based on how you're stuck</description>
<location>project</location>
</skill>

<skill>
<name>writing-plans</name>
<description>Create detailed implementation plans with bite-sized tasks for engineers with zero codebase context</description>
<location>project</location>
</skill>

<skill>
<name>writing-skills</name>
<description>TDD for process documentation - test with subagents before writing, iterate until bulletproof</description>
<location>project</location>
</skill>

</available_skills>
<!-- SKILLS_TABLE_END -->

</skills_system>
