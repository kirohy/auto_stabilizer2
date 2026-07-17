---
name: wbms-review-work-unit
description: WBMS外部whole-body操縦の変更をriskに応じてread-only reviewする。TARGETED、REPOSITORY_FULL、COMPATIBLE_SET、SAFETY_FULLを使い分け、R0のProgress-only変更へ不要なfull reviewを要求しない。file修正、stage、commit、push、merge、PR作成、simulation、実機実行には使用しない。
---

# WBMS Risk-based Review

## 目的

実装taskから独立して、指定されたdiff、commit、compatible setをread-onlyで検査する。
review強度をriskに合わせる。

最初に読む。

1. nearest `AGENTS.md` chain。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
3. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`。
4. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`。
5. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
6. Parent Work Package / Contract。
7. 必要な制御仕様。
8. review対象diffと周辺source。

## Read-only制約

行わない。

- source、文書、設定fileの変更。
- findingの自動修正。
- stage、commit、push、merge、PR。
- branch切替、reset、stash、clean。
- simulation、実機。
- compatible setのSHA変更。

## Review type

### SELF

implementation taskが行う最低限の自己点検。
本Skillを別taskで呼ぶ必要はない。

対象:

- R0。
- typo、Progress、文書。
- package skeletonの構文、path、scope。

### TARGETED

既知findingの修正箇所と影響範囲を確認する。

適する:

- compile fix。
- include漏れ。
- P3。
- diagnostic名。
- 文書path。
- behaviorを変えないguard。

前回reviewの前提を変える修正には使わない。

### REPOSITORY_FULL

一つのrepositoryのfeature diff全体をreviewする。

適する:

- R2。
- 初回repository review。
- state/failure pathを含む非RT機能。

### COMPATIBLE_SET

`${CATKIN_SOURCE_ROOT}`から複数repositoryをread-onlyで照合する。

重点:

- message / IDL / bridge mapping。
- field、type、unit、frame、quaternion順序。
- enum、task mask、schema version。
- session、epoch、sequence。
- producer/consumer SHA。
- package build結果。
- Project Context / Current Checkpoint。

### SAFETY_FULL

R3の最新source diff全体と周辺call pathをreviewする。

重点:

- 500 Hz blocking。
- joint limit、collision、足拘束。
- COM/ZMP、`refdz`、`omega`、`l`。
- walking preparation、READY、walking API gate。
- stale、hold、fallback。
- final IK、non-IK override。
- simulation前条件。

## Risk別既定

| Risk | Review |
|---|---|
| R0 | SELF。Parent末尾でfocused review一回 |
| R1 | COMPATIBLE_SET一回 |
| R2 | REPOSITORY_FULL一回。修正後は原則TARGETED |
| R3 | SAFETY_FULL。material修正後はfull fresh review |

## Full fresh review条件

以下の場合だけ、最新source diff全体を最初からreviewする。

1. P0/P1修正。
2. materialなP2修正。
3. schema field/type/unit/frame/enum変更。
4. state machine/session/stale/mode gate変更。
5. joint/collision/foot/COM/ZMP変更。
6. threading/blocking/allocation変更。
7. solver variable/constraint/priority変更。
8. simulation/実機前。
9. reviewerが前提崩壊を明示。

Progress、Current Checkpoint、Markdownだけの追記はsource full reviewを無効化しない。

## Repository状態

対象repositoryで確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

review対象を明示する。

- uncommitted diff。
- exact commit。
- base..head。
- compatible set。
- diff hash。

scopeが不明ならfindingを作る前に停止する。

## Severity

### P0 Critical

- 危険な不連続指令。
- joint limit、collision、足拘束、接触安全違反。
- stale後のcommand再生。
- NaN、未初期化、data corruption。
- schema誤解釈で別関節/taskへ指令。
- 使用不能なbuild break。

### P1 High

- 主要仕様不達。
- walking preparation/READY/ownership破壊。
- 500 Hz blocking。
- session/staleで誤task有効化。
- COM/ZMP等の不連続。
- 重大なlegacy回帰。
- repository間schema/frame/unit非互換。

### P2 Medium

- failure診断不足。
- validation不足。
- 次工程を危険にする責務混在。
- one-write-repository違反。
- 必要なbuild/check欠落。
- 計画/Contract/実装の矛盾。
- compatible SHA不明。

### P3 Low

- 命名、局所可読性。
- optional cleanup。
- 小さなperformance改善。
- future extension。

P3はcommitをblockしない。

## Review重点

### Multi-repository

- WRITE repositoryが一つ。
- sibling変更なし。
- Parent dependency順。
- exact SHA。
- Current Checkpointと実物の一致。
- 中央Progress checkpointが適切な境界か。

### Interface

- field/type/sequence長。
- unit/frame/quaternion順序。
- mask/enum/schema。
- timestamp/session/epoch/sequence。
- unknown/duplicate joint。
- silent dropがない。

### State / control

- accepted generated state基準。
- hidden goalなし。
- stale hold。
- reverse input response。
- frame変換。
- mode/support gate。

### Safety

- joint limit/collision/feet。
- COM/ZMP。
- walking preparation。
- final validation。
- 500 Hz performance。

## 問題として扱わない事項

単独ではfindingにしない。

- clang-format未適用。
- auto_stabilizerの新規unit testなし。
- コメント/Markdownが日本語。
- legacy portを初回に残す。
- single-QP final WBC未実装。
- walking中CHEST/COM未実装。
- workspace一括build未実施。
- package buildが`--no-deps`。
- simulation/実機未実施。`UNVERIFIED`とする。
- Progress-only diffにdetached full reviewがない。
- repository commitごとの中央Progress commitがない。
- R0 sub-unitごとのfresh reviewがない。
- reviewed sourceに影響しない記録追記。

## Finding形式

```markdown
## Findings

### [P1] <title>
- Repository:
- Location:
- Execution path:
- Broken specification/invariant:
- Reproduction condition:
- Impact:
- Minimal correction direction:
- Assigned sub-unit:
- Evidence:
```

findingなし:

```markdown
## Findings
P0/P1/P2 findingはありません。

## Residual risks / unverified
## Review scope
## Reviewed evidence identity
- commit SHA / diff hash:
## Required follow-up review
- NONE / TARGETED / REPOSITORY_FULL / COMPATIBLE_SET / SAFETY_FULL
```

## 必須出力末尾

```markdown
## Workflow state
- risk:
- review type:
- reviewed SHA/diff hash:
- material change detected:
- full fresh review required:

## Next mandatory action
- finding修正 / commit / compatible-set review / simulation gate:
- launch directory:
- human gate:
- copy-paste prompt:
```

このSkill使用中はfileを変更しない。
