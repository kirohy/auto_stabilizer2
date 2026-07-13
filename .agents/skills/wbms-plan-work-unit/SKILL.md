---
name: wbms-plan-work-unit
description: Read-only planning for exactly one WBMS external whole-body teleoperation Work Unit. Use for branch archaeology, current-code inspection, interface design, safety-invariant analysis, acceptance criteria, and a frozen Work Unit Contract before implementation. Do not use to modify source, review a completed diff, commit, push, merge, or open a PR.
---

# WBMS Work Unit計画

## 目的

一回の呼出しで、外部whole-body操縦projectの一つのWork Unitだけを計画する。

成果物はreview可能な`Work Unit Contract`とする。このSkillではsource codeを変更しない。

## 必須入力

Contract作成前に次を特定する。

- Work Unit IDとtitle。
- 対象repository。
- 各repositoryのtarget branchとcurrent HEAD。
- 正式Implementation PlanとRevision。
- 最新Progress entry。
- 目的と既知の制約。

Work Unitが曖昧な場合は曖昧点を報告して停止する。Work Unitを発明したり、scopeを暗黙に広げたりしない。

## 読む順序

`auto_stabilizer/`以下では次を順に読む。

1. repository rootの`AGENTS.md`。
2. `auto_stabilizer/AGENTS.md`。
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`。
6. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
7. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
8. 現在のWork Unit Contractがあれば読む。
9. walking preparation、COM高さ、READY、歩行API ownershipが関係する場合は`auto_stabilizer/docs/WBMSWalkingPreparationDesignRevisionPlan.md`。
10. 古い文書は履歴と不採用理由の確認にだけ使用する。

他repositoryではnearest `AGENTS.md`と同じ正式計画、Progressを先に読む。

## Repository状態確認

全repositoryで次の出力または同等情報を記録する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

必要な場合、`auto_stabilizer2`、`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`のdependency branchとSHAも記録する。

user変更を無断でreset、stash、checkout、clean、削除、上書きしない。

## Read-only調査

次を確定できるまで現行コードと履歴を読む。

1. 実際のcall orderとownership境界。
2. 関係するclass、function、port、message、IDL、parameter。
3. Revision/正式計画と現行コードの差異。
4. 保持すべき安全不変条件。
5. failure、stale、invalid data、mode transition。
6. cross-repository schemaとcompatibility影響。
7. 変更してよいfileと変更してはいけないfile。
8. acceptanceに必要なbuild、static check、simulation、log、実機確認。
9. 現在環境では確認できない項目。

500 Hz変更では、blocking I/O、ROS callback、network wait、mutex wait、condition variable、毎周期thread生成、無制限queue、不要clone、全探索、無制限allocationの可能性を明示的に調査する。

制御変更では、joint limit、self collision、feet/contact、COM/ZMP/`refdz`/`omega`/`l`、walking preparation ownership、hidden goal、final validationを追跡する。

## Contractルール

- 一つのWork Unitに一つの主要責務だけを置く。
- protocol、producer、bridge、consumer、安全処理、diagnostics、cleanupは独立review可能なら分割する。
- unrelated repository変更を一つのcommitへ混ぜない。
- 関連する複数repository変更は同じWork Unit IDのsub-unitにしてよい。
- 古い文書に記載されているだけの理由で実装を選ばない。
- source inspectionだけでsimulationまたは実機挙動をVERIFIEDにしない。
- 計画変更が必要なら、矛盾を示してdesignへ戻る。scopeを静かに変更しない。

## 必須出力

次の構造でContractを一つだけ返す。

```markdown
# Work Unit Contract: <ID> <title>

## Goal

## Scope
- repository
- branch
- base SHA
- files allowed

## Out of scope

## Current behavior

## Required behavior

## Safety invariants

## Interface/schema impact

## Implementation steps

## Acceptance criteria

## Verification commands

## Review focus

## Known unverified items

## Dependency compatibility

## Open decisions blocking implementation
```

Contractには具体的で検証可能なacceptance criteriaと、判明しているexact commandを含める。

## 完了条件

次を全て満たした場合だけplanning完了とする。

- branchとbase SHAが明示されている。
- scopeとout-of-scopeが明示されている。
- safety invariantとfailure behaviorが明示されている。
- interface変更がproducerからconsumerまでmappingされている。
- verificationとunverifiedが分離されている。
- implementationをblockする未決定事項がない。

このSkill使用中は実装、stage、commit、push、merge、PR作成を行わない。
