---
name: wbms-review-work-unit
description: 実装済みのWBMS外部whole-body操縦Work Unitを1つだけ、実装taskから独立してread-only reviewする。repository sub-unitのdiffまたは複数repositoryのcompatible setを対象に、根拠付きのP0-P3 findingを返す。file修正、findingの自動修正、複数repository同時変更、stage、commit、push、merge、PR作成には使用しない。
---

# WBMS Work Unit専用レビュー

## 目的

実装taskから独立したreviewerとして、一つのWork Unitの最新diffまたは一つのParent Work Unitのcompatible setをread-onlyで検査する。

findingは、実行経路、安全不変条件、interface contract、受入条件に基づいて報告する。styleだけの一般論や、今回scope外の将来機能をblocking findingにしない。

## Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

## Review種別

### Repository review

- Codex起動directory: 対象repository root。
- 対象: uncommitted diff、exact commit、base branchとの差分。
- 全repositoryはread-only。
- 対象repositoryのsub-unit Contractを中心にreviewする。

### Cross-repository compatible-set review

- Codex起動directory: `${CATKIN_SOURCE_ROOT}`。
- 対象: Parent Work Unit、repository SHA表、message/IDL/bridge/producer/consumer、package build結果。
- 全repositoryはread-only。
- 対象repositoryのroot/nearest `AGENTS.md`を明示的に読む。
- findingを対象repository sub-unitへ割り当てる。

## 必須入力

- Work Unit IDとtitle。
- Parent Work Unit ID、該当する場合。
- review種別。
- `${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`。
- Codex launch directory。
- 対象repository、branch、base SHA、current SHA。
- review scope。
  - uncommitted changes。
  - exact commit。
  - base branchとの差分。
  - compatible set。
- repository access matrix、全て`READ`。
- 承認済みParent/Sub-unit Contract。
- 正式Implementation Plan Revision 2、MultiRepositoryOperations、Revision 1、元計画。
- 最新中央Progress entry。
- repository Project Context、存在する場合。

review対象またはcompatible setが曖昧な場合は、対象を特定するまでfindingを作らない。

## 読む順序

1. source-root `AGENTS.md`、cross-repository reviewの場合。
2. 対象repository rootの`AGENTS.md`。
3. nearest package/module `AGENTS.md`。
4. repository Project Context、存在する場合。
5. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
6. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
7. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
8. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
9. `WBMSExternalWholeBodyTeleoperationProgress.md`。
10. Parent/Sub-unit Contract。
11. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
12. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`。
13. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
14. 該当する既存正式仕様。
15. review対象diffと周辺source。

古いprojection IK文書を新計画より優先しない。

## Read-only制約

このSkillでは次を行わない。

- source、文書、設定fileの変更。
- findingの自動修正。
- stage、commit、push、merge、PR作成。
- branch切替、reset、stash、clean。
- simulationまたは実機実行。
- compatible setのrepositoryを別SHAへ勝手に切り替えること。

buildや静的checkは、review環境と権限が明示されている場合だけ実行してよい。実行した場合もworking treeを変更するgenerator出力をcommit対象にしてはならない。

## Repository状態確認

全対象repositoryで次を確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

compatible setと異なる場合、findingを作る前にscope mismatchとして報告する。

## Review severity

### P0 Critical

- 実機へ危険な不連続指令を出す。
- joint limit、collision、足拘束、接触安全を破る。
- stale後に過去commandを再生する。
- NaN、未初期化、data corruption。
- schema解釈違いで別関節や別taskへ指令する。
- build不能でWork Unitが使用不能。

### P1 High

- Work Unitの主要仕様・受入条件を満たさない。
- walking preparation、READY、歩行API gate、ownershipを壊す。
- 500 Hz経路へblocking処理を追加する。
- session、epoch、sequence、staleで誤taskを有効化する。
- COM/ZMP/`refdz`/`omega`/`l`が不連続または不整合になる。
- legacy経路へ重大な回帰を入れる。
- repository間schema、enum、frame、unitが非互換である。

### P2 Medium

- failure原因を切り分けられないdiagnostics不足。
- interface validation不足。
- 次Work Unitを危険にする責務混在。
- one-write-repository rule違反。
- 必要なpackage build/check/Progress記録が欠落。
- 計画・Contract・Project Context・実装が矛盾する。
- compatible setまたはdependency SHAが不明。

### P3 Low / Suggestion

- 命名、局所的可読性、optional cleanup。
- 小さなperformance改善。
- 今回scope外の将来拡張。

P3はcommitを止めない。実害を説明できないstyle findingはP3以下にする。

## 重点確認

### Multi-repository operation

- Codex launch directoryがtask種別と一致するか。
- repository access matrixがあるか。
- implementationのWRITE repositoryが原則一つか。
- Parent/Sub-unit dependencyが明示されているか。
- sibling repositoryへ意図しない変更がないか。
- repository root/nearest `AGENTS.md`が参照されているか。
- Project Contextと中央計画/Progressの参照SHAが一致するか。
- central Progress syncが必要なcommitで記録されているか。

### Protocol / bridge

- ROS message、RTM IDL、bridge mappingの全field。
- 単位、frame、quaternion順序。
- task mask、enum、schema version。
- timestamp、session、epoch、sequence。
- out-of-order、duplicate、unknown joint。
- command/stateのatomicity。
- source timestampをbridge時刻で上書きしていないか。
- producer、bridge、consumerのcompatible SHA。

### State machine / stale

- task group別enabled、valid、stale、hold。
- generator/bridge heartbeat。
- zero velocity freshとstaleの区別。
- same-epoch rebase。
- epoch変更時の再enable。
- stale中の未実現commandを復帰後に回収していないか。

### Frames / control

- device world、foot-mid、CHEST相対、generate worldの取り違え。
- hand baseline、position scale、rotation delta。
- CHEST/COM速度のfoot-mid解釈。
- accepted generated state基準の局所target。
- hidden goalの有無。

### Safety / walking

- joint position/velocity limit。
- self collision。
- 両足pose、support phase、static gate。
- walking preparation ownership。
- walking中CHEST/COM無効、腕/頭部継続。
- COM、`genCog`、`sbpOffset`、ZMP、`refdz`、`omega`、`l`。
- final validationとreject/hold。

### Non-IK direct joint

- qRef読込後、FK/COM前にoverrideしているか。
- upstream q/dqを毎周期復元しているか。
- allowlistと`jointControllable=false`確認。
- finite、position、velocity、acceleration limit。
- stale hold、WBMS start/stop blend。
- final IK後だけ上書きしてmodelと出力を不一致にしていないか。

### Performance

- 500 Hz経路のROS callback、network I/O、blocking I/O。
- condition variable、mutex待ち。
- 毎周期thread生成。
- 無制限queue、clone、全探索、allocation。
- external process停止時に500 Hzを待たないか。

### Build / workflow

- workspace一括buildを暗黙に要求していないか。
- 対象packageの`catkin build <package> --no-deps`が指定されているか。
- dependency確認が必要な場合だけ`--no-deps`を外しているか。
- exact commandと実行directoryが記録されているか。
- IDL/CMake変更時の`--force-cmake`。
- Contract外のfileやcleanupが混在していないか。
- simulation/実機未確認をPASS扱いしていないか。
- Progress、計画、実装の一致。

## 問題として扱わない事項

次を単独理由でfindingにしない。

- `clang-format`未適用。
- `auto_stabilizer`で新規unit testを追加していないこと。
- コメントとMarkdownが日本語であること。
- legacy portを初回実装で残していること。
- pre-M5 final IKと`maxIteration=1`を初期baselineにしていること。
- single-QP final WBCを今回実装していないこと。
- walking中CHEST/COM操作を未実装であること。
- workspace全体の引数なし`catkin build`を実行していないこと。
- package buildを`--no-deps`で実行していること。
- simulationや実機を現在の環境で実行できないこと自体。これは`UNVERIFIED`であり、虚偽のPASS表記がある場合だけfindingにする。
- 古い文書に不採用案が残っていること。
- future extensionを実装していないこと。

## Finding形式

findingがある場合、重要度順に並べる。

```markdown
## Findings

### [P1] <短いtitle>
- Repository: <repository>
- Location: `path/file.cpp:123`
- Execution path:
- Broken specification or invariant:
- Reproduction condition:
- Impact:
- Minimal correction direction:
- Assigned sub-unit:
- Evidence:
```

同じ根本原因による複数箇所は一つのfindingにまとめる。

曖昧な可能性だけでfindingを作らず、具体的な実行経路と影響を示す。

## Findingがない場合

```markdown
## Findings

P0/P1/P2 findingはありません。

## Residual risks / unverified
- ...

## Review scope
- repositories / SHAs
- diff or compatible set
```

P3提案がある場合はblocking findingと分ける。

## 完了条件

- review対象、launch directory、repository SHAが明示されている。
- 最新diffまたはcompatible set全体を読んでいる。
- 対象repositoryのAGENTSと周辺call pathを確認している。
- P0-P3分類が一貫している。
- findingにrepository、file:line、実行経路、破られる仕様、最小修正方針、assigned sub-unitがある。
- 未確認事項がfindingと分離されている。

このSkill使用中はsourceを変更しない。
