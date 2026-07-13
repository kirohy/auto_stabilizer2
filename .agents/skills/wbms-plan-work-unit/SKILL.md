---
name: wbms-plan-work-unit
description: WBMS外部whole-body操縦プロジェクトのWork Unitを1つだけread-onlyで計画する。単一repository sub-unitまたは複数repositoryのParent Work Unitについて、branch履歴、現行コード、interface、安全不変条件、repository access、受入条件、package build手順を調査し、実装前のWork Unit Contractを作成する。source変更、完成diffのreview、commit、push、merge、PR作成には使用しない。
---

# WBMS Work Unit計画

## 目的

一回の呼出しで、外部whole-body操縦projectの一つのWork Unitだけを計画する。

成果物はreview可能な`Work Unit Contract`とする。このSkillではsource codeを変更しない。

## Workspace path

次の定義を使用する。

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

## Task種別と起動directory

### Cross-repository Parent Work Unit

- Codex起動directory: `${CATKIN_SOURCE_ROOT}`。
- 全repositoryは原則`READ`。
- branch archaeology、protocol設計、compatible-set review、package build計画を扱う。
- Parent Contractとrepository sub-unit一覧を作る。

### Repository Sub-unit

- Codex起動directory: 対象repository root。
- `WRITE` repositoryは原則一つ。
- sibling repositoryは必要な範囲だけ`READ`。
- 対象repositoryのatomic commitへ収まるContractを作る。

## 必須入力

Contract作成前に次を特定する。

- Work Unit IDとtitle。
- Parent Work Unit ID、該当する場合。
- task種別: Parent / repository sub-unit。
- `${CATKIN_WORKSPACE}`と`${CATKIN_SOURCE_ROOT}`。
- Codex launch directory。
- 対象repositoryのpath、branch、current HEAD、dirty state。
- repository access matrix: `READ` / `WRITE` / `NONE`。
- 正式Implementation Plan Revision 2、MultiRepositoryOperations、Revision 1、元計画。
- 最新Progress entry。
- repository Project Context、存在する場合。
- 目的と既知の制約。

Work Unitまたはrepository accessが曖昧な場合は曖昧点を報告して停止する。Work Unitを発明したり、scopeを暗黙に広げたりしない。

## 読む順序

`auto_stabilizer/`以下では次を順に読む。

1. repository rootの`AGENTS.md`。
2. `auto_stabilizer/AGENTS.md`。
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
6. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
7. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`。
8. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
9. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`。
10. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
11. 現在のWork Unit Contract、存在する場合。
12. walking preparation、COM高さ、READY、歩行API ownershipが関係する場合は`WBMSWalkingPreparationDesignRevisionPlan.md`。
13. 古い文書は履歴と不採用理由の確認にだけ使用する。

他repositoryでは次を読む。

1. source-root `AGENTS.md`、cross-repository taskの場合。
2. 対象repository rootの`AGENTS.md`。
3. nearest package/module `AGENTS.md`。
4. `docs/WBMSExternalTeleopProjectContext.md`、存在する場合。
5. 中央のRevision 2、MultiRepositoryOperations、Revision 1、元計画、Progress。
6. Parent Contractとdependency sub-unit Contract。

cross-repository taskでは、子repositoryの`AGENTS.md`が自動適用されたと仮定せず、明示的に読む。

## Repository状態確認

全対象repositoryで次の出力または同等情報を記録する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

必要な場合、`auto_stabilizer2`、`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`のdependency branchとSHAを記録する。

user変更を無断でreset、stash、checkout、clean、削除、上書きしない。

## Read-only調査

次を確定できるまで現行コードと履歴を読む。

1. 実際のcall orderとownership境界。
2. 関係するclass、function、port、message、IDL、parameter。
3. Revision/正式計画と現行コードの差異。
4. 保持すべき安全不変条件。
5. failure、stale、invalid data、mode transition。
6. cross-repository schemaとcompatibility影響。
7. Parent/Sub-unit dependency順。
8. 各repositoryの`READ`/`WRITE`範囲。
9. 変更してよいfileと変更してはいけないfile。
10. acceptanceに必要なstatic check、package build、simulation、log、実機確認。
11. 現在環境では確認できない項目。

500 Hz変更では、blocking I/O、ROS callback、network wait、mutex wait、condition variable、毎周期thread生成、無制限queue、不要clone、全探索、無制限allocationの可能性を明示的に調査する。

制御変更では、joint limit、self collision、feet/contact、COM/ZMP/`refdz`/`omega`/`l`、walking preparation ownership、hidden goal、final validationを追跡する。

## Build計画

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存関係まで確認する必要がある場合だけ:

```sh
catkin build <package-name>
```

IDL/CMake再生成が必要な場合はContractへ`--force-cmake`を明示する。

`catkin build`の実行directoryは固定しないが、Contractに記録する。

## Contractルール

- 一つのWork Unitに一つの主要責務だけを置く。
- implementation Contractの`WRITE` repositoryは原則一つ。
- 複数repositoryへ影響する機能はParent Contractとrepository sub-unitへ分割する。
- protocol、producer、bridge、consumer、安全処理、diagnostics、cleanupは独立review可能なら分割する。
- unrelated repository変更を一つのcommitへ混ぜない。
- schema未確定のproducer、bridge、consumerを並行実装しない。
- 古い文書に記載されているだけの理由で実装を選ばない。
- source inspectionだけでsimulationまたは実機挙動を`VERIFIED`にしない。
- 計画変更が必要なら、矛盾を示してdesignへ戻る。scopeを静かに変更しない。

## 必須出力

次の構造でContractを一つだけ返す。

```markdown
# Work Unit Contract: <ID> <title>

## Goal

## Workspace context
- catkin workspace root
- source root
- Codex launch directory

## Repository access
| repository | path | branch | base SHA | current SHA | access |

## Active instructions
- source-root AGENTS.md
- target repository AGENTS.md
- package/module AGENTS.md
- explicitly read sibling AGENTS.md

## Parent Work Unit
- Parent ID
- schema/protocol version
- dependency sub-units

## Scope
- files allowed

## Out of scope

## Current behavior

## Required behavior

## Safety invariants

## Interface/schema impact

## Compatible input set
| repository | branch | SHA | status |

## Expected repository output
- target repository
- expected commit subject
- central Progress sync requirement

## Implementation steps

## Acceptance criteria

## Package verification
| package | command | execution directory | dependency scope |

## Cross-repository acceptance

## Review focus

## Known unverified items

## Open decisions blocking implementation
```

Contractには具体的で検証可能なacceptance criteriaと、判明しているexact commandを含める。

## 完了条件

次を全て満たした場合だけplanning完了とする。

- `${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`、launch directoryが明示されている。
- branchとbase SHAが明示されている。
- repository accessが明示され、implementationでは`WRITE` repositoryが原則一つである。
- Parent/Sub-unit関係が明示されている。
- scopeとout-of-scopeが明示されている。
- safety invariantとfailure behaviorが明示されている。
- interface変更がproducerからconsumerまでmappingされている。
- package-specific buildとdependency scopeが明示されている。
- verificationとunverifiedが分離されている。
- implementationをblockする未決定事項がない。

このSkill使用中は実装、stage、commit、push、merge、PR作成を行わない。
