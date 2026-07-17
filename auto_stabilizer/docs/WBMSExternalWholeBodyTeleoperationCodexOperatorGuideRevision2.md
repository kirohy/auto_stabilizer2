# WBMS外部Whole-Body操縦 Codex運用ガイド Revision 2

## 1. 位置づけ

本書は`WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md`を、
risk-based workflowへ対応させる。

優先:

1. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`
2. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`
3. 本書
4. 旧Operator Guide / Workflow

この文書のpromptでは次を置換する。

```text
<CATKIN_WORKSPACE>
<CATKIN_SOURCE_ROOT>
<repository SHA>
<Contract path>
```

現在の既知workspace:

```text
CATKIN_WORKSPACE=/home/kirohy/catkin_ws/teleop_ws
CATKIN_SOURCE_ROOT=/home/kirohy/catkin_ws/teleop_ws/src
```

---

## 2. M0途中からの再開

## 2.1 M0-REMAINDER Parent planning

`${CATKIN_SOURCE_ROOT}`から開始する。

```text
$wbms-plan-work-unit を使用してください。

Parent Work Package:
M0-REMAINDER Complete remaining workspace bootstrap and baseline verification

Task type:
cross-repository Parent Work Package

CATKIN_WORKSPACE:
/home/kirohy/catkin_ws/teleop_ws

CATKIN_SOURCE_ROOT:
/home/kirohy/catkin_ws/teleop_ws/src

Codex launch directory:
/home/kirohy/catkin_ws/teleop_ws/src

次を最初に読んでください。
1. auto_stabilizer2/auto_stabilizer/AGENTS.md
2. WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md
3. WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md
4. WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md
5. WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md
6. 最新Parent/Sub-unit Contract
7. 必要なProgress履歴
8. tools/codex_workspaceの最新template
9. 対象repositoryのroot/nearest AGENTS.md

このtaskはread-only planningです。
source、branch、index、working treeを変更しないでください。

現在までに完了済みとして再利用するもの:
- M0-A baseline selection
- M0-B1
- M0-B2
- M0-B3 whole_body_teleop repository bootstrap
- M0-B3 repository review/static verification
- auto_stabilizer2 HEAD上のWORKFLOW-R1 migration

次を一つのParent Work Packageへまとめてください。

R0 bootstrap remainder:
- M0-B4 rtmros_msg_bridge branch/instruction bootstrap
- M0-B5 source-root AGENTS.mdとcommon Skill installation/recognition
- M0-B6 Project Context、workspace manifest、defer状態
- whole_body_teleopの実配置AGENTS.mdをWORKFLOW-R1へ同期
- R0全体のfocused bootstrap review一回

R1 baseline verification:
- M0-B7 exact compatible-set materialization
- package-specific build
- workspace一括buildは行わない

M0 integration:
- M0-B8 cross-repository review
- M0 compatible set freeze
- M0 completion Current Checkpoint / Progress一件

Risk:
- M0-B4〜B6: R0
- M0-B7: R1
- M0-B8: COMPATIBLE_SET review

必須条件:
- 一つのimplementation実行がWRITEするrepositoryは一つ
- 各repository commitはatomic
- B4〜B6の各commit間で中央Progress commitを要求しない
- exact predecessor SHAを次sub-unitへ渡す
- source root、user-level Skill、repository AGENTSの既存状態を無断上書きしない
- control sourceを変更しない
- simulation、実機を実行しない

停止条件:
- control source変更が必要
- schema/safety invariant変更が必要
- user変更との衝突
- destructive Git操作
- selected baseline SHA不一致
- build failureの修正が別repositoryに必要
- simulation/実機が必要

R0ではfull ContractではなくWork Briefを使用してよいです。
M0-REMAINDER Parent Work Package、sub-unit順、standing commit authorization案、
verification、review boundary、Progress checkpointを出力してください。

完了時に次を出力してください。

## Next mandatory action
- 最初のeligible sub-unit
- launch directory
- WRITE/READ repositories
- copy-paste implementation prompt
```

## 2.2 M0-REMAINDER承認・自走prompt

Parent Work Packageを確認後、次を入力する。

```text
M0-REMAINDER Parent Work Packageを承認します。

standing authorization:
- Parent scope内のR0 local commitを許可
- 一回に一つのrepositoryだけstage/commit
- explicit pathだけstage
- push、merge、PR作成なし
- control source、schema、安全仕様を変更しない
- user変更と衝突した場合は停止
- destructive Git操作が必要なら停止
- riskがR2/R3へ上がった場合は停止
- simulation、実機前で停止

$wbms-implement-work-unit を使用してください。

Current Checkpointと承認済みParent Work Packageに従い、
M0-B4、M0-B5、M0-B6のeligible sub-unitをdependency順に進めてください。

各sub-unitで:
- 一つのWRITE repositoryだけを扱う
- 必要な構文/package discoveryを実行
- git diff --check
- SELF review
- standing authorizationの条件を満たす場合だけatomic local commit
- commit SHAを次sub-unitへ直接渡す

各commit間の中央Progress sync、detached review、fresh full reviewは行わないでください。

M0-B4〜B6完了後に一旦停止し、次を報告してください。
- repository commit SHA一覧
- verification evidence
- Current Checkpoint更新案
- R0 focused bootstrap review用prompt
- M0-B7用prompt
```

## 2.3 R0 focused bootstrap review

```text
$wbms-review-work-unit を使用してください。

Review type:
TARGETED focused bootstrap review

Parent Work Package:
M0-REMAINDER

Risk:
R0

対象:
- M0-B4〜B6のcommitted repository SHAs
- source-root AGENTS.md
- common Skill symlink/recognition evidence
- repository AGENTS.md
- Project Context
- workspace manifest
- Current Checkpoint

全repositoryはread-onlyです。
control sourceのfull reviewは行わないでください。

確認:
- one-write-repository rule
- template/targetの意味整合
- repository role
- Skill認識
- exact SHA/path
- user file非上書き
- control source非変更
- M0-B7へ必要な情報が揃っているか

Progress-only fresh reviewがないことをfindingにしないでください。
P0/P1/P2 findingと、M0-B7へ進めるかを報告してください。
```

## 2.4 M0-B7 build prompt

```text
$wbms-implement-work-unit を使用してください。

Work Unit:
M0-B7 exact compatible-set package verification

Risk:
R1

このtaskはsource/repositoryを変更しないbuild verificationです。

Selected set:
- auto_stabilizer2:
  c06b63c8e12dbf85bda4c8391a37544c2469731c
- ik_solvers2:
  b5de6cd99a6bf89ddb9baadd2a77b63a52319add
- prioritized_qp:
  624bc1e3e26d4a16f7765baf64fc5941865f2d64
- rtmros_msg_bridge:
  10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214

承認済みM0-REMAINDER Contractのisolated workspace/build spaceを使用してください。
current development checkoutをselected baselineとして扱わないでください。

workspace一括buildは行わず、Contractに指定されたpackage-specific commandを順に実行してください。
通常は--no-depsを使用し、dependency確認用commandだけContractどおり実行してください。

記録:
- execution directory
- exact command
- package
- dependency scope
- result
- relevant error
- selected repository SHA

build failureでsource修正が必要な場合は修正せず停止し、
対象repositoryと最小fix sub-unitを報告してください。

simulation、実機は実行しないでください。
```

## 2.5 M0-B8 review prompt

```text
$wbms-review-work-unit を使用してください。

Review type:
COMPATIBLE_SET

Parent Work Package:
M0-REMAINDER

Risk:
R1 integration

Codex launch directory:
/home/kirohy/catkin_ws/teleop_ws/src

全repositoryはread-onlyです。

対象:
- M0-A selected baseline
- M0-B2 isolated auto_stabilizer branch
- M0-B3 whole_body_teleop bootstrap
- M0-B4〜B6 repository commits
- source-root AGENTS/common Skill evidence
- Project Context/workspace manifest
- M0-B7 package build evidence
- Current Checkpoint
- repository branch/HEAD/dirty state

確認:
- exact compatible SHA
- one-write-repository履歴
- control source非変更
- repository instruction整合
- Skill recognition
- manifest/Project Context
- package-specific build結果
- workspace一括buildを要求していないこと
- M1-P開始に必要な情報

sourceは変更しないでください。
P0/P1/P2 finding、residual unverified、M0 completion可否を報告してください。
```

## 2.6 M0 completion prompt

```text
$wbms-close-work-unit を使用してください。

Closure type:
Parent Work Package / milestone checkpoint

Parent Work Package:
M0-REMAINDER

Risk:
R1 checkpoint

M0-B8 review結果と全repository commit SHA、M0-B7 build結果を読み、
次を実行してください。

- M0-compatible-setをfreeze
- Current CheckpointをM0 completed / M1-P nextへ更新
- 中央ProgressへM0 completion entryを一件だけappend
- exact commit authorizationがある場合だけauto_stabilizer2でworkflow/document checkpoint commit
- control sourceを変更しない
- push、merge、PR、simulation、実機を行わない

過去M0-B1〜B3 entryは変更しないでください。
```

---

## 3. M1以降の標準prompt

## 3.1 Parent planning

```text
$wbms-plan-work-unit を使用してください。

Parent Work Package:
<Milestone-ID and outcome>

Task type:
cross-repository Parent Work Package

CATKIN_WORKSPACE:
<CATKIN_WORKSPACE>

CATKIN_SOURCE_ROOT:
<CATKIN_SOURCE_ROOT>

Codex launch directory:
<CATKIN_SOURCE_ROOT>

次を読んでください。
- AGENTS.md chain
- Implementation Plan Revision 2
- Codex Workflow Revision 1
- Current Checkpoint
- relevant control specification
- current compatible set
- relevant repository Project Context

このtaskはread-only planningです。

固定してください。
- outcome
- risk level
- repository sub-units and dependency order
- WRITE/READ repository
- interface/schema
- package verification
- review boundary
- Progress checkpoint
- standing/exact commit authorization
- simulation/hardware gate
- stop conditions

R0はWork Brief、R1〜R3は必要なContractを使用してください。
micro-stepごとの中央Progress entryを要求しないでください。

最後に最初のeligible sub-unit用copy-paste promptを出してください。
```

## 3.2 R0/R1 implementation with standing authorization

```text
Parent Work Package <ID> を承認します。

standing authorization:
- Parent scope内のR0/R1 local commitを許可
- 一回に一つのWRITE repository
- explicit pathだけstage
- required static check/package buildを実行
- commit SHAを次sub-unitへ渡す
- 各commit間の中央Progress commitは不要
- compatible set完成時に停止
- push、merge、PRなし

停止条件:
- schema意味変更
- riskがR2/R3へ上昇
- safety invariant変更
- sibling WRITEが必要
- user変更との衝突
- destructive Git操作
- simulation/hardware

$wbms-implement-work-unit を使用し、
承認済みParent Work Package内のeligible sub-unitを順に進めてください。
```

## 3.3 R1 compatible-set review

```text
$wbms-review-work-unit を使用してください。

Review type:
COMPATIBLE_SET

Parent Work Package:
<ID>

Compatible set:
<repository SHA table>

全repositoryはread-onlyです。

Parent Contractに従い、message、IDL、bridge、producer、consumer、
field/type/unit/frame/quaternion、enum/mask/schema、
session/epoch/sequence、package build、Project Contextを照合してください。

各repository sub-unitでfull reviewを実施していないことをfindingにしないでください。
materialなinterface不整合だけをP0/P1/P2として報告してください。
```

## 3.4 R2 feature implementation

```text
$wbms-implement-work-unit を使用してください。

Work Unit:
<ID and feature>

Risk:
R2

WRITE repository:
<one repository>

READ repositories:
<repository SHA table>

承認済みContractの範囲だけを実装してください。
package build、failure/stale/frame/hidden-goal自己点検を実行してください。

REPOSITORY_FULL review前で停止してください。
review findingの非本質修正はTARGETED follow-upとし、
state machine、frame、priority、constraint、fallbackが変わる場合だけfull fresh reviewを要求してください。

simulation、実機、push、merge、PRは行わないでください。
```

## 3.5 R3 safety implementation

```text
$wbms-implement-work-unit を使用してください。

Work Unit:
<ID and safety-critical outcome>

Risk:
R3

WRITE repository:
auto_stabilizer2

承認済みfull Contractに従って実装してください。

必須:
- 500 Hz禁止事項
- safety invariants
- package build
- verification evidence identity
- SAFETY_FULL review前に停止
- P0/P1/P2解消
- material修正後full fresh review
- simulation gateで停止
- commitごとのexact人間承認

standing authorizationでcommitしないでください。
simulation、実機、push、merge、PRを行わないでください。
```

## 3.6 Parent checkpoint

```text
$wbms-close-work-unit を使用してください。

Closure type:
Parent Work Package checkpoint

Parent Work Package:
<ID>

次を確認してください。
- completed sub-units
- repository commit SHAs
- verification/review evidence
- compatible set
- pending simulation/hardware
- Current Checkpoint
- 中央Progress update要否
- next Parent Work Package

中央Progressはこのcheckpointで一件だけ追加してください。
micro-stepの履歴を重複記録しないでください。
```

---

## 4. 次工程判定prompt

```text
このprojectの次工程をread-onlyで一つだけ決定してください。

読むもの:
1. Codex Workflow Revision 1
2. Current Checkpoint
3. active Parent Work Package / Contracts
4. latest repository commits
5. review/build evidence
6. 必要なProgress履歴

判定:
- Parent未承認 -> approval
- eligible sub-unitあり -> implementation
- compatible set完成 -> COMPATIBLE_SET review
- R2 implementation済み -> REPOSITORY_FULL review
- R3 implementation済み -> SAFETY_FULL review
- material finding修正済み -> full fresh review
- non-material finding修正済み -> TARGETED review
- Parent完了 -> checkpoint closure
- simulation/hardware -> 人間gate
- blocker -> blockerだけ提示

source、文書、branch、indexを変更しないでください。

出力:
- current workflow state
- evidence
- next mandatory action
- launch directory
- WRITE/READ repositories
- human gate
- copy-paste prompt
```
