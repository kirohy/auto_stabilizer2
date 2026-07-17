---
name: wbms-plan-work-unit
description: WBMS外部whole-body操縦プロジェクトのParent Work Packageまたはrepository sub-unitをread-onlyで計画する。risk level、repository access、stop condition、review boundary、Progress checkpoint、package verificationを固定する。source変更、review、commit、push、merge、PR作成には使用しない。
---

# WBMS Work Package計画

## 目的

一回の呼出しで、一つのParent Work Packageまたは一つのrepository sub-unitを計画する。
source、branch、index、working treeを変更しない。

最初に読む。

1. nearest `AGENTS.md` chain。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
3. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`。
4. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`。
5. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
6. 制御仕様のImplementation Plan / Revision。
7. 必要なProgress履歴entry。
8. Parent Contract、存在する場合。

## Workspace

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

- cross-repository Parent planningは`${CATKIN_SOURCE_ROOT}`から行う。
- repository sub-unit planningは対象repository rootから行う。
- 子repositoryの`AGENTS.md`が自動適用されたと仮定せず、必要なものを明示的に読む。

## Repository状態

対象repositoryで次を確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

user変更を無断でreset、stash、checkout、clean、削除しない。

## Risk判定

最高riskを一つ選ぶ。

- `R0`: 文書、Progress、AGENTS、Skill、bootstrap、package skeleton。
- `R1`: message、IDL、enum、task mask、bridge mapping、scaffolding。
- `R2`: external generator、mapping、external IK、非RT state machine。
- `R3`: 500 Hz、COM/ZMP、walking preparation、final IK、安全constraint、最終出力。

riskを過小評価しない。複数分類に該当する場合は最高riskを採用する。

## Planning粒度

### R0 Work Brief

R0は次の短い形式でよい。

```markdown
# Work Brief: <ID>

- Parent Work Package:
- Goal:
- Risk: R0
- Launch directory:
- WRITE repository/path:
- READ repositories/SHAs:
- Allowed paths:
- Verification:
- Review boundary:
- Progress checkpoint:
- Commit authorization:
- Stop conditions:
- Next eligible sub-units:
```

### R1〜R3 Contract

```markdown
# Work Unit Contract: <ID>

## Parent Work Package
## Goal
## Risk level
## Workspace context
## Repository access
## Current behavior
## Required behavior
## Safety invariants
## Interface/schema impact
## Scope / files allowed
## Out of scope
## Implementation steps
## Package verification
## Review policy
## Progress checkpoint policy
## Commit authorization
## Stop conditions
## Acceptance criteria
## Known unverified items
## Dependency compatibility
## Next eligible sub-units
```

## Parent Work Package

複数repositoryへ影響する成果では、Parentで次を固定する。

- outcome。
- repository sub-unitとdependency順。
- risk。
- compatible input set。
- review boundary。
- package build。
- Progress checkpoint。
- standing authorization。
- simulation/hardware gate。
- stop conditions。

repository commitごとの中央Progress syncを既定にしない。
exact SHAはParent working stateと次promptへ渡し、integration/checkpoint前に中央Progressへ記録する。

## Review policy

- R0: SELF。Work Package末尾にfocused reviewを一回。
- R1: repository SELF + compatible set完成時にCOMPATIBLE_SET review。
- R2: REPOSITORY_FULLを一回。非本質修正はTARGETED。
- R3: SAFETY_FULL。material修正後にfull fresh review。

full fresh review条件はWorkflow Revision 1に従う。

## Commit authorization

計画結果に次を明記する。

- `none`: commitしない。
- `exact`: 一つのexact commitだけ人間承認が必要。
- `standing`: Parent scope内のR0〜R2 local commitを事前許可できる。

standing authorizationでも次では停止する。

- riskがR3へ上がった。
- schema/safety invariant変更。
- user変更との衝突。
- sibling WRITEが必要。
- destructive Git操作。
- simulation/hardware。

## 完了条件

- riskとreview policyが明示されている。
- WRITE repositoryが原則一つ。
- branch、HEAD、dirty stateが記録されている。
- stop conditionsが具体的。
- package verificationが対象変更に対応。
- Progress checkpointがmicro-step単位になっていない。
- implementationをblockする未決定事項がない。

## 必須出力末尾

```markdown
## Workflow state
- risk level:
- Parent Work Package:
- repository access:
- review policy:
- Progress checkpoint:
- commit authorization:
- stop conditions:

## Next mandatory action
- next Work Package / sub-unit:
- Codex launch directory:
- Skill or review type:
- human gate:
- copy-paste prompt:
```

このSkill使用中は実装、stage、commit、push、merge、PR作成を行わない。
