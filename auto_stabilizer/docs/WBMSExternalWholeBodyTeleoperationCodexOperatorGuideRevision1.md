# WBMS外部Whole-Body操縦 Codex運用ガイド Revision 1

## 1. 文書の位置づけ

本書は`WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`に対し、複数repository運用、起動directory、one-write-repository rule、package build、central Progress syncを追加・修正する。

本書の内容は元Operator Guideより優先する。

参照順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
3. 本書
4. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`
5. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`
6. `WBMSExternalWholeBodyTeleoperationProgress.md`

### Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

---

## 2. Taskごとの起動directory

### Cross-repository planning / review

```sh
codex -C ${CATKIN_SOURCE_ROOT}
```

用途:

- M0 branch archaeology。
- Parent Work Unit Contract。
- protocol全体設計。
- compatible-set review。
- package build結果の統合確認。
- simulation計画とlog解析。

原則read-onlyで行う。

### Repository implementation / review / commit

```sh
codex -C ${CATKIN_SOURCE_ROOT}/auto_stabilizer2
codex -C ${CATKIN_SOURCE_ROOT}/whole_body_teleop
codex -C ${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge
codex -C ${CATKIN_SOURCE_ROOT}/ik_solvers2
codex -C ${CATKIN_SOURCE_ROOT}/prioritized_qp
```

一つのimplementation taskがWRITEするrepositoryは原則一つだけとする。

---

## 3. 最初のWorkspace確認

M0-Bでsource-root instructionと共通Skillを配置した後、`${CATKIN_SOURCE_ROOT}`から次を入力する。

```text
現在のcatkin workspaceについて次をread-onlyで確認してください。

- CATKIN_WORKSPACE
- CATKIN_SOURCE_ROOT
- source-root AGENTS.md
- 対象repository一覧
- 各repositoryのbranch、HEAD、dirty state
- 各repository root/nearest AGENTS.mdのpath
- 利用可能なwbms-* Skillのname、description、path

子repositoryのAGENTS.mdを自動適用済みと仮定せず、明示的に読んだものを区別してください。
file、branch、index、working treeを変更しないでください。
```

期待するSkill:

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

---

## 4. Parent Work Unit planning

`${CATKIN_SOURCE_ROOT}`から開始する。

```text
$wbms-plan-work-unit を使用してください。

Work Unit: <Parent IDとtitle>
Task type: cross-repository Parent Work Unit
CATKIN_WORKSPACE: <absolute path to catkin_ws/<workspace_name>>
CATKIN_SOURCE_ROOT: <CATKIN_WORKSPACE>/src
Codex launch directory: <CATKIN_SOURCE_ROOT>

このtaskはread-only planningです。
対象repositoryのbranch、HEAD、dirty stateを確認し、各repositoryのroot/nearest AGENTS.mdとProject Contextを明示的に読んでください。

目的:
- 論理interfaceとdependency順を固定する。
- repository sub-unitへ分割する。
- 各sub-unitのWRITE repositoryを一つにする。
- compatible input setとpackage buildを定義する。

Parent Work Unit Contractだけを出力してください。
source、branch、index、working treeを変更しないでください。
```

人間が確認する。

- repository分割。
- schema/frame/unit/enum。
- dependency順。
- WRITE/READ access。
- package build。
- compatible-set review。
- simulation gate。

---

## 5. Repository Sub-unit planning

対象repository rootから開始する。

```text
$wbms-plan-work-unit を使用してください。

Work Unit: <Sub-unit IDとtitle>
Parent Work Unit: <Parent ID>
CATKIN_WORKSPACE: <...>
CATKIN_SOURCE_ROOT: <...>
Codex launch directory: <CATKIN_SOURCE_ROOT>/<target repository>

WRITE repository:
- <target repository>

READ-only repositories:
- <repository and SHA>

Parent Contractと中央計画、対象repositoryのAGENTS.md、Project Contextを読んでください。
このtaskはread-only planningです。
Sub-unit Contractだけを出力してください。
```

---

## 6. Repository implementation

Contract承認後、対象repository rootから新taskを開始する。

```text
$wbms-implement-work-unit を使用してください。

Work Unit: <Sub-unit IDとtitle>
Parent Work Unit: <Parent ID>
Contract: <path>
CATKIN_WORKSPACE: <...>
CATKIN_SOURCE_ROOT: <...>
Codex launch directory: <target repository root>

WRITE repository:
- <target repository>

READ-only repositories:
- <repository and SHA>

承認済みContractの範囲だけを実装してください。
他repositoryを変更しないでください。
指定package buildを実行してください。
commit、push、merge、PR作成、simulation、実機実行は行わないでください。
```

実装taskは次まで自走できる。

- source実装。
- Contract内compile error修正。
- static check。
- package-specific build。
- diff自己点検。
- implementation report。

次の場合は停止する。

- branch/base SHA不一致。
- user dirty changeとの衝突。
- schema変更が必要。
- sibling repository変更が必要。
- safety invariant変更が必要。
- simulation/実機判断が必要。

---

## 7. Package build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存関係まで確認する場合だけ:

```sh
catkin build <package-name>
```

IDL変更後の`auto_stabilizer`初回:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

実行directoryは固定しない。

実装結果とProgressへ次を記録する。

```text
execution directory
exact command
package
--no-depsの有無
--force-cmakeの有無
result
```

workspace全体の引数なし`catkin build`を実行しないこと自体はfindingにしない。

---

## 8. Repository review

対象repository rootで別taskを開始する。

```text
$wbms-review-work-unit を使用してください。

Review type: repository review
Work Unit: <Sub-unit IDとtitle>
Parent Work Unit: <Parent ID>
Scope: uncommitted changes / exact commit / base diff
CATKIN_WORKSPACE: <...>
CATKIN_SOURCE_ROOT: <...>
Codex launch directory: <target repository root>
Contract: <path>

対象repositoryとREAD-only dependency SHAを確認し、sourceを変更せずP0/P1/P2/P3 findingを返してください。
```

finding修正後は最新diff全体をfresh reviewする。

---

## 9. Cross-repository compatible-set review

repository sub-unitが揃った後、`${CATKIN_SOURCE_ROOT}`から開始する。

```text
$wbms-review-work-unit を使用してください。

Review type: cross-repository compatible set
Parent Work Unit: <ID>
CATKIN_WORKSPACE: <...>
CATKIN_SOURCE_ROOT: <...>
Codex launch directory: <CATKIN_SOURCE_ROOT>

Compatible set:
- auto_stabilizer2: <sha>
- whole_body_teleop: <sha>
- rtmros_msg_bridge: <sha>
- ik_solvers2: <sha>
- prioritized_qp: <sha>

全repositoryはread-onlyです。
各repositoryのAGENTS.mdとProject Contextを明示的に読んでください。

重点:
- ROS message / RTM IDL / bridge mapping
- field、型、単位、frame、quaternion順序
- task mask、enum、schema version
- session、epoch、sequence、timestamp
- package build結果
- compatible SHA
- central ProgressとProject Context

sourceを変更しないでください。
```

findingは対象repositoryのfix sub-unitへ割り当てる。

---

## 10. Closureとcommit

対象repository rootで行う。

```text
$wbms-close-work-unit を使用してください。

Work Unit: <Sub-unit IDとtitle>
Parent Work Unit: <Parent ID>
Commit repository: <repository>
Contract: <path>
Latest review: <result>

Project Contextと中央Progress同期要否を確認し、commit readinessを判定してください。
この時点ではcommitしないでください。
```

`READY FOR COMMIT`後、人間が明示する。

```text
Work Unit <ID>について、repository <repository>へのcommitを許可します。

今回Work Unitのfileだけをstageし、checklistを再確認してください。
全条件を満たす場合だけ次のsubjectでatomic commitしてください。

<subject>

sibling repositoryを変更・stage・commitしないでください。
push、merge、PR作成、amend、rebaseは行わないでください。
```

---

## 11. 他repository commit後の中央Progress同期

`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`のcommit後、依存する次sub-unit前に`auto_stabilizer2`でdocument-only syncを行う。

```text
$wbms-plan-work-unit を使用してください。

Work Unit: <Parent/Sub-unit>-PROGRESS central Progress sync
WRITE repository: auto_stabilizer2
READ-only repository commit:
- <repository>: <sha>

中央Progressへcommit SHA、build/review結果、compatible set、next entry pointをappend-onlyで記録するContractを作成してください。
```

その後、`auto_stabilizer2` rootから実装、review、closure、commitを行う。

---

## 12. Simulation gate

simulationを必要とするWork Unitでは、package buildとsource review後に一旦停止する。

Codexは次を提示する。

```markdown
# Simulation verification request

## Work Unit
## Compatible repository set
## Package build evidence
## Scenario
## Proposed commands
## Logs to collect
## Metrics
## Acceptance thresholds
## Abort conditions
```

ユーザーの明示許可なしにsimulationを起動しない。

初期は人間が既存runbookで実行する。反復手順が安定した後、simulation自動化を独立Work Unitとする。

---

## 13. M0-Bで行うこと

1. `${CATKIN_SOURCE_ROOT}/AGENTS.md`をtemplateから配置する。
2. 共通4 Skillを`${HOME}/.agents/skills`へsymlinkする。
3. `whole_body_teleop`を作成し、root `AGENTS.md`とProject Contextを追加する。
4. `rtmros_msg_bridge`の実装branchへ`AGENTS.md`とProject Contextを追加する。
5. solver repositoryは変更が必要になった時点でinstructionを追加する。
6. workspace manifestを確定する。
7. 各repository rootからSkill認識を確認する。
8. package-specific baseline buildを行う。
9. cross-repository instruction/Skill reviewを行う。

---

## 14. 次のtask

planning branchのformal review後、M0-Aは`${CATKIN_SOURCE_ROOT}`からread-onlyで開始する。

```text
$wbms-plan-work-unit を使用してください。

Work Unit: M0-A branch archaeology and exact baseline selection
Task type: cross-repository Parent Work Unit
CATKIN_WORKSPACE: <absolute path>
CATKIN_SOURCE_ROOT: <CATKIN_WORKSPACE>/src
Codex launch directory: <CATKIN_SOURCE_ROOT>

Revision 2、MultiRepositoryOperations、Revision 1、Implementation Plan、最新Progress、walking preparation正式文書、git history、対象sourceを読んでください。

source、branch、index、working treeを変更せず、Parent Contractだけを作成してください。
```
