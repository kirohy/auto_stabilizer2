---
name: wbms-implement-work-unit
description: 承認済みWBMS Parent Work Packageまたはrepository sub-unitをrisk-based workflowで実装する。一度に一つのWRITE repositoryだけを扱い、R0〜R2ではstanding authorizationに基づく連続実行とlocal commitを許可し、R3では厳格なreview・人間gateを維持する。計画、独立review、push、merge、PR作成、無許可simulation・実機実行には使用しない。
---

# WBMS Work Package実装

## 目的

承認済みParent Work Packageまたはrepository sub-unitを実装する。
一つのimplementation実行がWRITEするrepositoryは一つだけとする。

最初に読む。

1. nearest `AGENTS.md` chain。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
3. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`。
4. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`。
5. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
6. Parent Work Package / sub-unit Contract。
7. 必要な制御仕様。
8. 必要なProgress履歴entry。

## 開始条件

必須:

- Work Package / sub-unit ID。
- risk level。
- target repository、branch、base/current SHA。
- WRITE repositoryが一つ。
- READ sibling repositoryとSHA。
- allowed paths。
- verification command。
- review policy。
- Progress checkpoint policy。
- commit authorization。
- stop conditions。

以下の場合はsourceを変更せず停止する。

- Contract/Work Briefがない。
- branch/base SHAが一致しない。
- userの既存変更とscopeが衝突する。
- WRITE repositoryが複数。
- riskが未定義。
- safety仕様を推測する必要がある。

## Repository状態

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- user変更を無断でreset、stash、checkout、clean、削除しない。
- sibling repositoryを変更しない。
- destructive Git操作が必要なら停止する。

## Risk別実装

### R0

対象例:

- 文書、Current Checkpoint、AGENTS、Skill。
- bootstrap、Project Context、manifest。
- package skeleton。
- build/runbook記録。

実行:

- allowed pathだけ変更。
- 必要な構文確認。
- package skeletonならpackage discovery。
- `git diff --check`。
- SELF review。

Parentにstanding authorizationがある場合、必要check後にatomic local commitまで進めてよい。
R0 sub-unitごとのdetached reviewと中央Progress syncを要求しない。

### R1

対象例:

- ROS message、RTM IDL。
- enum、task mask、schema。
- bridge mapping、scaffolding。

実行:

- Parent protocol contractに従う。
- package build。
- mapping SELF review。
- repository commitを作成してよいのはstanding/exact authorizationがある場合だけ。
- compatible set完成時にCOMPATIBLE_SET reviewへ進む。

各repository sub-unitで同じfull reviewを反復しない。

### R2

対象例:

- external generator。
- hand/HMD mapping。
- external IK。
- 非RT state machine。

実行:

- package build。
- failure、stale、frame、hidden goalを自己点検。
- REPOSITORY_FULL review前で一度停止する。
- review後の非本質修正はTARGETED follow-up。
- material変更ならfull fresh review。

commitはreview policyを満たし、standing/exact authorizationがある場合だけ行う。

### R3

対象例:

- 500 Hz `auto_stabilizer`。
- COM/ZMP、walking preparation、final IK。
- safety constraint、stale最終出力。
- non-IK joint override、projection削除。

実行:

- full Contractを厳守。
- package build。
- SAFETY_FULL review前に停止。
- P0/P1/P2を解消。
- material修正後にfull fresh review。
- simulation gateで停止。
- commitごとのexact人間承認を要求する。

R3はstanding authorizationでcommitしない。

## 500 Hz禁止事項

`auto_stabilizer::onExecute()`と同期呼出し経路へ追加しない。

- ROS callback。
- network I/O。
- blocking I/O。
- 外部process待ち。
- condition variable待ち。
- 無制限mutex待ち。
- 毎周期thread生成。
- 無制限queue。
- 不要clone、全探索。
- 無制限反復。
- 不要な動的確保。

## 安全不変条件

該当する場合は保持する。

- joint position/velocity limit。
- self collision。
- 足拘束、接触、歩行安定化。
- COM、ZMP、`genCog`、`sbpOffset`、`refdz`、`omega`、`l`。
- walking preparation、READY、walking API gate、COM高さ保持。
- static両足支持だけoperator CHEST/COM。
- walking preparation/歩行中のCHEST/COM無効、腕/頭継続。
- stale/invalid/failure時のhold。
- hidden goalなし。
- `q_nominal`をhardwareへ直接出力しない。

## Interface変更

Parent Contractで固定された次を照合する。

- field/type。
- unit/frame。
- quaternion順序。
- task mask/enum。
- schema version。
- timestamp/session/epoch/sequence。
- joint name mapping。
- producer/consumer SHA。

意味変更が必要なら現在sub-unitで暗黙変更せずParent planへ戻る。

## Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

dependency確認時だけ:

```sh
catkin build <package-name>
```

IDL変更後の`auto_stabilizer`初回:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

execution directory、exact command、resultを記録する。

## Verification evidence

実装後に対象を識別する。

commit前の候補:

```sh
git diff --binary --no-ext-diff | sha256sum
```

最低限:

```sh
git diff --check
git status --short
git diff --stat
git diff -- <changed files>
```

記録する。

- source diff hashまたはcommit SHA。
- build command/result。
- review type/result。
- affected files/packages。

ProgressやMarkdownだけが後から変わってもsource evidenceを無効化しない。

## Commit

### Authorization

- `none`: commitしない。
- `exact`: 指定された一commitだけ。
- `standing`: Parent scope内のR0〜R2 local commitを順次作成可能。

commit前:

```sh
git branch --show-current
git status --short
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
```

- explicit pathだけstageする。
- `git add -A`を既定にしない。
- sibling repositoryをcommitしない。
- push、merge、PR作成を行わない。

standing authorization中でもstop conditionが発生したらcommitせず停止する。

## Parent内の連続実行

以下を満たす場合、commit後に次eligible sub-unitへ進んでよい。

- 同じ承認済みParent Work Package内。
- dependencyを満たす。
- exact predecessor SHAを次sub-unitへ渡す。
- next sub-unitのWRITE repositoryが一つ。
- risk/review policyに違反しない。
- Current Checkpoint/Parent working stateを更新できる。
- stop conditionなし。

micro-stepごとの中央Progress commitは不要。
integration、simulation、milestone、引き継ぎ前にcheckpointを作る。

## Stop conditions

- Parent scope外。
- riskがR3へ上昇。
- schema/safety invariant変更。
- sibling WRITEが必要。
- user変更と衝突。
- destructive Git操作。
- build failureの修正先が別repository。
- simulation/hardware。
- compatible SHA不明。
- Contractと現行コードの重大な矛盾。

## 必須出力

```markdown
# Work Package implementation result

## Workflow state
- risk level:
- Parent Work Package:
- completed sub-units:
- repository commit SHAs:
- verification evidence:
- pending review/build:
- blockers:

## Changes
## Commands and results
## Acceptance
## Unverified
## Contract deviations
## Current Checkpoint update
## Next mandatory action
- next Work Package / sub-unit:
- launch directory:
- WRITE/READ repositories:
- Skill/review type:
- human gate:
- copy-paste prompt:
```

push、merge、PR作成、無許可simulation・実機実行は行わない。
