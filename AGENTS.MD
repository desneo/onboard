## 文档资料地址

- `docs/requirement.md` 原始项目背景信息和输入
- `docs/SPEC.md` 产品规格文档
- `docs/ARCHITECTURE.md` 架构设计文档
- `docs/DEV_PLAN.md` 开发总路线图（M0–M10）、全局约束、依赖基线、风险与 DoD
- `docs/plans/<change-id>-design.md` 各变更的 Change Design
- `docs/plans/<change-id>-tasks.md` 各变更的任务级计划与执行记录

## 禁止修改 requirement.md，除非得到我明确允许

## 信息来源优先级

当多个文档存在冲突时，按照以下优先级处理：

1. 用户明确提出的最新要求；
2. `docs/SPEC.md`；
3. `docs/ARCHITECTURE.md`；
4. 当前变更（`docs/plans/<change-id>-design.md`）的 Design；
5. 当前变更（`docs/plans/<change-id>-tasks.md`）的 tasks；
6. 现有代码实现；
7. Agent 自己的推测。

发现冲突时：

- 不得静默选择一个方案；
- 不得擅自修改上层需求；
- 必须明确指出冲突；
- 涉及需求或架构变化时，应暂停并请求确认。

## 开发前必须

1. 阅读 `docs/SPEC.md`；
2. 阅读 `docs/ARCHITECTURE.md`；
3. 找到当前正在实施的变更（`docs/plans/<change-id>-design.md` 与 `-tasks.md`），阅读 Design 与 tasks；
4. 检查与当前任务相关的现有代码；
5. 确定本次变更涉及的 SPEC 要求与架构模块；
6. 检查实现方案是否违反 `ARCHITECTURE.md`，确认与需求和架构一致；
7. 生成 Change Design，Design 确认后才能生成 `tasks.md`。

不得在不了解上述上下文的情况下直接开始大规模修改代码。
不得跳过任务，也不得在未明确说明的情况下重新定义任务。

## 检索与外部目录约束

- 工程根为 `onboard/`；公共头文件位于工程外的 `../common/common_include`（由 `cmake/Dependencies.cmake` 的 `PLANE_COMMON_INCLUDE_DIR` 提供），三方库位于 `../../../3rdparty/common`。需要确认定义时直接读这两个相对路径。
- 禁止使用 `find /`、`rg`、`grep` 等扫描工程根之外的目录（含全盘搜索）；检索只在工程根内进行。
- 若检索命中工程外的意外路径，必须停下并说明，不得继续读取。
- 子代理（subagent）同样受本约束；派发任务时应在提示中给出所需外部文件的准确路径。

## 编码过程中要遵守的原则

优先：

- 简单实现，便于人理解；
- 清晰职责；
- 明确所有权；
- 明确生命周期；
- 可测试；
- 最小必要修改；
- 复用已有机制。

避免：

- 复杂语法；
- 过度抽象、过度设计；
- 猜测性功能；
- 无关重构；
- 重复实现；
- 为未来需求提前设计复杂框架。

## 文档更新

仅在确实发生变化时更新文档，这时候要提醒我确认后再进行文档更新。

如果修改了：

- 产品需求 → 更新 `docs/SPEC.md`；
- 系统架构 → 更新 `docs/ARCHITECTURE.md`；
- 当前 Change 的实现方案 → 更新 `design.md`；
- 当前 Change 的执行任务 → 更新 `tasks.md`。

不得为了形式而创建额外的文档。

## 架构约束

- 生成或修改任何代码前，必须先阅读 `docs/ARCHITECTURE.md`，遵循其中的分层、模块边界与接口约定。
- 若实现与架构文档冲突，先停下来指出冲突，不要擅自偏离。
