# WBMS外部Whole-Body操縦 複数リポジトリ運用仕様

## 1. 文書の位置づけ

本書は、WBMS外部whole-body操縦プロジェクトを、同一catkin workspace内の複数Git repositoryに分割して実装・review・検証・commitするための正式運用仕様である。

### 1.1 Workspace path

本書では次を用いる。

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

例:

```text
/home/user/catkin_ws/jaxon_ws
/home/user/catkin_ws/jaxon_ws/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

対象repositoryは次を基本とする。

```text
${CATKIN_SOURCE_ROOT}/
  auto_stabilizer2/
  whole_body_teleop/
  rtmros_msg_bridge/
  ik_solvers2/
  prioritized_qp/
```

新規ROS nodeは`whole_body_teleop`で管理する。ROS/RTM変換は`rtmros_msg_bridge`、500 Hzのconsumerと安全制御は`auto_stabilizer2`、IK/QP基盤は`ik_solvers2`と`prioritized_qp`で管理する。

参照順は次とする。

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. 本書
3. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
4. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
5. `WBMSExternalWholeBodyTeleoperationProgress.md`
6. 対象Work Unit Contract
7. repository固有`AGENTS.md`
8. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`
9. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`
10. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`

本書と既存Workflow、Operator Guideの複数repository運用記述が矛盾する場合、本書を優先する。

---

## 2. Repository責務

| repository | role | final safety responsibility |
|---|---|---|
| `auto_stabilizer2` | external reference consumer、500 Hz制御、walking preparation、final IK、中央計画・Progress | あり |
| `whole_body_teleop` | ROS message、入力baseline、手差分、HMD、external IK、reference generator | なし |
| `rtmros_msg_bridge` | ROS/RTM message変換、latest-only transport | なし |
| `ik_solvers2` | generic IK library | library contractのみ |
| `prioritized_qp` | QP backend | solver contractのみ |

重いexternal IK、device mapping、HMD処理を`auto_stabilizer2`へ置かない。bridgeへIK、mode判断、stale policyを置かない。

---

## 3. 基本原則

### 3.1 One-write-repository rule

```text
一つのimplementation task
  = 一つのWRITE repository
  = 一つのrepository sub-unit
  = 原則一つのatomic commit
```

同一implementation taskで複数repositoryを同時に書き換えない。

理由:

- repositoryごとのinstruction、build、review、commitを明確にできる。
- schema、producer、bridge、consumerの責務を分離できる。
- compatible SHAを追跡できる。
- finding修正範囲を限定できる。
- userの未commit変更を別repositoryまで巻き込まない。

複数repositoryを同時に変更する例外は、repository移動自体が目的である場合等に限定し、Parent Contractとユーザーの明示承認を必要とする。

### 3.2 Cross-repository taskは原則read-only

次をcross-repository taskとする。

- branch archaeology
- Parent Work Unit planning
- protocol全体設計
- compatible-set review
- interface整合review
- package build結果の統合確認
- simulation計画
- 複数repository log解析

source修正が必要になった場合、対象repositoryのsub-unitへ分離する。

### 3.3 中央正本

当面の正本:

```text
auto_stabilizer2/auto_stabilizer/docs/
```

含むもの:

- Implementation PlanとRevision。
- MultiRepositoryOperations。
- Codex Workflow、Operator Guide、Official Guidance。
- 中央Progress。

各repositoryへ計画全文を複製しない。

---

## 4. Codex起動directory

### 4.1 Cross-repository task

起動directory:

```text
${CATKIN_SOURCE_ROOT}
```

例:

```sh
codex -C ${CATKIN_SOURCE_ROOT}
```

対象:

- cross-repository planning。
- branch archaeology。
- Parent Contract。
- protocol全体review。
- compatible-set review。
- package build結果の統合確認。
- simulation計画、log解析。

原則read-onlyとする。

### 4.2 Repository task

対象repository rootから起動する。

```sh
codex -C ${CATKIN_SOURCE_ROOT}/auto_stabilizer2
codex -C ${CATKIN_SOURCE_ROOT}/whole_body_teleop
codex -C ${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge
codex -C ${CATKIN_SOURCE_ROOT}/ik_solvers2
codex -C ${CATKIN_SOURCE_ROOT}/prioritized_qp
```

対象:

- implementation。
- repository review。
- finding修正。
- closure。
- commit。

### 4.3 起動directory表

| task | Codex launch directory | write access |
|---|---|---|
| cross-repo planning | `${CATKIN_SOURCE_ROOT}` | なし |
| branch archaeology | `${CATKIN_SOURCE_ROOT}` | なし |
| Parent Contract | `${CATKIN_SOURCE_ROOT}` | 原則なし |
| compatible-set review | `${CATKIN_SOURCE_ROOT}` | なし |
| package build integration check | `${CATKIN_SOURCE_ROOT}`または任意のworkspace内directory | source変更なし |
| simulation計画・log解析 | `${CATKIN_SOURCE_ROOT}` | 明示文書・logだけ |
| repository implementation | 対象repository root | 対象repositoryのみ |
| repository review | 対象repository root | なし |
| finding修正 | 対象repository root | 対象repositoryのみ |
| commit | 対象repository root | 対象repositoryのみ |

`${CATKIN_SOURCE_ROOT}`から特定repositoryを実装することは可能だが、instruction、write範囲、commit対象を誤りやすいため既定にしない。

---

## 5. `AGENTS.md`の三層構成

### 5.1 Source-root `AGENTS.md`

配置:

```text
${CATKIN_SOURCE_ROOT}/AGENTS.md
```

通常はGit repository外のlocal fileである。`auto_stabilizer2`内のtemplateからbootstrapする。

責務:

- 対象repository一覧とpath。
- cross-repository taskは原則read-only。
- implementationのWRITE repositoryは一つ。
- repositoryを扱う前にrootとnearest `AGENTS.md`を明示的に読む。
- 全repositoryのbranch、HEAD、dirty stateを確認する。
- user変更をreset、stash、cleanしない。
- `build`、`devel`、`install`、`logs`をsource変更として扱わない。
- package-specific buildを用いる。
- simulationと実機は明示許可を必要とする。

`${CATKIN_SOURCE_ROOT}`から起動した場合、子repositoryの`AGENTS.md`が自動で全て適用されたと仮定しない。

### 5.2 Repository-root `AGENTS.md`

各repositoryへ配置する。

```text
auto_stabilizer2/AGENTS.md
whole_body_teleop/AGENTS.md
rtmros_msg_bridge/AGENTS.md
ik_solvers2/AGENTS.md
prioritized_qp/AGENTS.md
```

責務はrepository固有の恒久規約に限定する。

#### `auto_stabilizer2`

- 500 Hz経路の禁止事項。
- walking preparation、COM/ZMP、安全validation。
- external reference consumer。
- non-IK joint override。
- legacy互換。

#### `whole_body_teleop`

- 標準ROS入力、session baseline、手差分scale。
- CHEST/COM局所target。
- HMDとhead recenter。
- external whole-body IK。
- `q_nominal`とcustom bundle。
- RobotHardwareへ直接出力しない。
- external IKを最終安全保証とみなさない。
- 重いIKをsubscriber callbackで直接解かない。

#### `rtmros_msg_bridge`

- ROS/RTM変換だけを担当する。
- IK、limit、stale policy、mode gate、walking preparationを持たない。
- source timestampを上書きしない。
- latest-only、queue backlogを作らない。
- `hrpEC`へ参加させない。
- message/IDL fieldの一対一対応。

#### `ik_solvers2`

- generic IK libraryの責務を維持する。
- robot-specific policyを入れない。
- 既存API semanticsを暗黙に変更しない。
- external generator用変更は明示APIまたはfeatureとして分離する。
- dependent package buildとbenchmarkを記録する。

#### `prioritized_qp`

- QP backend互換性。
- task semantics、priority、warm start、solver status。
- 既存consumerの数値挙動を暗黙に変更しない。
- performance regressionとdependent build。

### 5.3 Package/module-level `AGENTS.md`

必要な場合だけ追加する。

```text
auto_stabilizer2/auto_stabilizer/AGENTS.md
whole_body_teleop/whole_body_teleop_msgs/AGENTS.md
whole_body_teleop/whole_body_teleop_reference_generator/AGENTS.md
rtmros_msg_bridge/<bridge-package>/AGENTS.md
```

---

## 6. 共通Skill

### 6.1 対象Skill

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

各repositoryへcopyして別version化しない。

### 6.2 正本

```text
${CATKIN_SOURCE_ROOT}/auto_stabilizer2/.agents/skills/
```

Skillの意味は`auto_stabilizer2`専用ではなく、本プロジェクト共通である。

### 6.3 User-level installation

bootstrap時に次へsymlinkする。

```text
${HOME}/.agents/skills/wbms-plan-work-unit
  -> ${CATKIN_SOURCE_ROOT}/auto_stabilizer2/.agents/skills/wbms-plan-work-unit
```

同様に4 Skillをlinkする。

目的:

- 各repository rootから同じSkillを利用する。
- Skill正本を一つにする。
- copy driftを防ぐ。

同名Skillをsource-rootとuser-levelへ重複配置することは既定にしない。

### 6.4 Repository固有Skill

repository固有の反復作業だけを各repositoryへ置いてよい。

```text
whole_body_teleop/.agents/skills/wbms-build-teleop-package/
rtmros_msg_bridge/.agents/skills/wbms-check-bridge-roundtrip/
auto_stabilizer2/.agents/skills/wbms-analyze-stabilizer-log/
```

共通Work Unit lifecycleを複製しない。

---

## 7. Workspace bootstrap

M0-Bで次を用意する。

1. `${CATKIN_SOURCE_ROOT}/AGENTS.md`。
2. user-level共通Skill symlink。
3. 実装対象repositoryのroot `AGENTS.md`。
4. repository Project Context。
5. repository path/role manifest。
6. Skillとinstruction chainの認識確認。

後続Work Unitで作成可能なscript:

```text
auto_stabilizer2/tools/codex_workspace/bootstrap_codex_workspace.sh
auto_stabilizer2/tools/codex_workspace/verify_codex_workspace.sh
```

scriptは次を守る。

- `${CATKIN_WORKSPACE}`を引数で受ける。
- `${CATKIN_SOURCE_ROOT}`を導出する。
- 既存fileを無断上書きしない。
- symlink先を表示する。
- repositoryの存在、branch、HEAD、dirty stateを確認する。
- 不一致を修正せず報告する。
- control sourceを変更しない。

---

## 8. Project Context

各repositoryに短い文書を置く。

```text
docs/WBMSExternalTeleopProjectContext.md
```

内容:

- project ID。
- repository role。
- 正式計画、Revision、中央Progressのrepository/branch/SHA/path。
- 関連repository一覧。
- current Parent Work Unitとsub-unit。
- current compatible set。
- package build command。
- repository固有invariant。

正式Implementation Plan全文を複製しない。

---

## 9. Parent Work UnitとRepository Sub-unit

### 9.1 Parent Work Unit

複数repositoryへ影響する論理機能は、`${CATKIN_SOURCE_ROOT}`からread-only Parent Contractを作る。

例:

```text
M1-P: WBMS external teleoperation protocol
```

固定する項目:

- schema version。
- 論理field。
- frame、単位、quaternion順序。
- task mask、enum。
- session、epoch、sequence、timestamp。
- compatibility rule。
- repository sub-unit一覧。
- dependency順。
- cross-repository acceptance。

### 9.2 Repository Sub-unit

```text
M1-A: whole_body_teleop ROS message
M1-B: auto_stabilizer RTM IDL
M1-C: rtmros_msg_bridge mapping
M1-D: compatible-set review
M1-E: package build integration check
```

M1-A、M1-B、M1-Cは各repository rootから実装する。

M1-D、M1-Eはsource rootからread-onlyで行う。

### 9.3 Dependency

後続sub-unitは依存sub-unitのcommit SHAをContractへ記録する。

schema未確定のproducer、bridge、consumerを並行実装しない。

---

## 10. Work Unit Contract拡張

複数repositoryに関係するContractへ次を追加する。

```markdown
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

## Compatible input set
| repository | branch | SHA | status |

## Expected repository output
- target repository
- expected commit subject
- central Progress sync requirement

## Package verification
| package | command | execution directory | dependency scope |

## Cross-repository acceptance
```

`access`は`READ`、`WRITE`、`NONE`を使う。implementation Contractの`WRITE` repositoryは原則一つ。

---

## 11. Build方針

### 11.1 Workspace一括buildは行わない

次を通常の受入条件にしない。

```sh
catkin build
```

### 11.2 対象package build

通常:

```sh
catkin build <package-name> --no-deps
```

例:

```sh
catkin build auto_stabilizer --no-deps
catkin build whole_body_teleop_msgs --no-deps
catkin build whole_body_teleop_reference_generator --no-deps
catkin build whole_body_teleop_rtmros_bridge --no-deps
```

### 11.3 Dependency確認

依存関係のsource/API互換性まで確認する場合だけ`--no-deps`を外す。

```sh
catkin build <package-name>
```

目的と対象dependencyをContractとProgressへ記録する。

### 11.4 IDL/CMake変更

`auto_stabilizer`のIDL変更後の初回:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

他packageでも必要ならContractで`--force-cmake`を指定する。

### 11.5 実行directory

`catkin build`の実行directoryは固定しない。catkin workspace内でworkspaceを解決できる場所から実行してよい。

Progressへ記録する。

- exact command。
- execution directory。
- package名。
- `--no-deps`の有無。
- `--force-cmake`の有無。
- result。

### 11.6 Integration build

integration buildは、関連packageのpackage-specific buildを順に実行することを意味する。workspace一括buildを意味しない。

build failureをcross-repository task内で複数repository同時修正しない。

```text
build failure
  -> root cause候補と対象repositoryを記録
  -> repository-specific fix sub-unit
  -> 対象repository rootから修正
  -> package build再実行
```

---

## 12. Review

### 12.1 Repository review

対象repository rootから行う。

- 当該repositoryのdiff。
- repositoryのAGENTS。
- Parent/Sub-unit Contract。
- sibling repositoryはcompatible inputとしてread-only参照。

### 12.2 Cross-repository review

`${CATKIN_SOURCE_ROOT}`からread-onlyで行う。

重点:

- message、IDL、bridge mapping。
- field、型、単位、frame、quaternion順序。
- enum、task mask、schema version。
- session、epoch、sequence。
- producer/consumer compatible SHA。
- package build結果。
- Project Contextと中央Progress。

reviewerはsourceを修正しない。findingは対象repository sub-unitへ割り当てる。

---

## 13. Commitと中央Progress

### 13.1 Repository commit

対象repository rootで行う。

- 一つの主要目的。
- 対象repositoryだけstage。
- `git add -A`を既定にしない。
- push、merge、PRは別許可。

### 13.2 中央Progress

正本:

```text
auto_stabilizer2/auto_stabilizer/docs/
  WBMSExternalWholeBodyTeleoperationProgress.md
```

### 13.3 他repository commit後

`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`でcommitした後、依存する次sub-unit前に中央ProgressへSHAを記録する。

```text
repository sub-unit commit
  -> central Progress sync sub-unit
  -> compatible set更新
  -> dependent sub-unit開始
```

central Progress syncはdocument-onlyの独立commitとしてよい。

### 13.4 Compatible set

Milestoneごとに名前を付ける。

```text
M1-compatible-set
M3-compatible-set
M8-compatible-set
M11-release-candidate-set
```

```yaml
compatible_set:
  auto_stabilizer2: <sha>
  whole_body_teleop: <sha>
  rtmros_msg_bridge: <sha>
  ik_solvers2: <sha>
  prioritized_qp: <sha>
```

未変更repositoryも使用中SHAを記録する。

---

## 14. Worktree

`${CATKIN_SOURCE_ROOT}`内へ同一packageを持つ複数worktreeを配置しない。

悪い例:

```text
${CATKIN_SOURCE_ROOT}/auto_stabilizer2
${CATKIN_SOURCE_ROOT}/auto_stabilizer2-review
```

read-only review worktreeはworkspace外へ置く。

```text
${HOME}/codex_worktrees/auto_stabilizer2-review
```

buildが必要な別branchは、専用catkin workspaceまたはpackage重複がない構成を使う。

---

## 15. Simulationと実機

simulationはsource rootから計画できるが、起動、command送信、log取得はユーザーの明示許可を必要とする。

初期は人間が既存runbookで実行し、Codexへ次を渡す。

- exact command。
- branch/SHA。
- dependency SHA。
- parameter。
- scenario。
- log path。
- 目視所見。

同じscenarioを反復する場合、別Work Unitでrunbook、script、timeout、abort、log収集を実装する。

実機は常に別の明示許可を必要とする。

---

## 16. Repository bootstrap

### 16.1 `whole_body_teleop`

新規repository作成時に追加する。

- root `AGENTS.md`。
- `docs/WBMSExternalTeleopProjectContext.md`。
- `whole_body_teleop_msgs`。
- `whole_body_teleop_reference_generator`。
- package-specific build説明。
- central plan/Progressの固定参照。

### 16.2 `rtmros_msg_bridge`

実装branch作成時に追加する。

- rootまたは対象package `AGENTS.md`。
- Project Context。
- bridge責務と非責務。
- message/IDL mapping review規約。

### 16.3 `ik_solvers2`、`prioritized_qp`

変更が必要になった時点で、実装前に追加する。

- repository `AGENTS.md`。
- Project Context。
- backward compatibilityとdependent build規約。

変更不要の場合、bootstrapだけを目的に既存repositoryへcommitする必要はない。cross-repository taskではbranch/SHAを記録する。

---

## 17. M0-B再構成

```text
M0-B1: source-root workspace operation contract
M0-B2: auto_stabilizer implementation branch bootstrap
M0-B3: whole_body_teleop repository bootstrap
M0-B4: rtmros_msg_bridge branch and instruction bootstrap
M0-B5: common Skill user-level installation verification
M0-B6: repository Project Context and manifest
M0-B7: package-specific baseline builds
M0-B8: cross-repository instruction/Skill review
```

Acceptance:

- `${CATKIN_SOURCE_ROOT}/AGENTS.md`が存在する。
- repository pathとroleが記録される。
- 実装対象repositoryにroot/nearest `AGENTS.md`がある。
- 4 Skillが各repository rootから認識される。
- one-write-repository ruleがContractへ反映される。
- Parent/Sub-unit形式が利用できる。
- package-specific build commandが記録される。
- workspace一括buildを要求していない。
- central ProgressとProject Contextの同期方法が確認される。

---

## 18. 標準prompt

### Cross-repository planning

```text
$wbms-plan-work-unit を使用してください。

Work Unit: <Parent IDとtitle>
Codex起動directory: ${CATKIN_SOURCE_ROOT}

このtaskはcross-repository read-only planningです。
全対象repositoryのbranch、HEAD、dirty stateを確認し、各repositoryのAGENTS.mdを明示的に読んでください。
source、branch、index、working treeを変更しないでください。

Parent Contractとrepository sub-unit一覧を作成してください。
各sub-unitのWRITE repositoryは一つにしてください。
```

### Repository implementation

```text
$wbms-implement-work-unit を使用してください。

Work Unit: <Sub-unit IDとtitle>
Codex起動directory: ${CATKIN_SOURCE_ROOT}/<repository>
WRITE repository: <repository>
READ-ONLY sibling repositories: <list>

承認済みContractの範囲だけを実装してください。
他repositoryを変更しないでください。
指定package buildを実行してください。
commitは行わないでください。
```

### Compatible-set review

```text
$wbms-review-work-unit を使用してください。

Review type: cross-repository compatible set
Codex起動directory: ${CATKIN_SOURCE_ROOT}
Parent Work Unit: <ID>
Compatible set: <repository SHA table>

全repositoryはread-onlyです。
message、IDL、bridge、producer、consumer、frame、unit、enum、schema、package build結果、Project Context、中央Progressを照合してください。
sourceは変更しないでください。
```

---

## 19. 禁止事項

- `catkin_ws/src`を固定layoutとして扱わない。
- cross-repository implementationを一つの巨大taskで行わない。
- `${CATKIN_SOURCE_ROOT}`から複数repositoryを同時編集しない。
- repository固有`AGENTS.md`を読まずにcross-repo判断しない。
- 共通Skillを各repositoryへcopyして別version化しない。
- schema未確定でproducer、bridge、consumerを並行実装しない。
- workspace一括buildを暗黙の受入条件にしない。
- build failureをcross-repo task内で複数repository同時修正しない。
- 他repository commit SHAを中央Progressへ記録せず依存sub-unitへ進まない。
- source root内へ同一packageの複数worktreeを置かない。
- simulationまたは実機を無許可で開始しない。

---

## 20. 変更管理

本書を変更する場合:

1. 変更理由を中央Progressへ追記する。
2. Revision 2との整合を確認する。
3. package `AGENTS.md`と4 Skillを確認する。
4. workspace templateとProject Context templateを確認する。
5. workflow変更だけの独立commitとする。
6. control source変更と混ぜない。
