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

M0-Bでのownerを次へ固定する。

- source-root `AGENTS.md`、user-level共通Skill symlink、Skill/instruction認識確認: M0-B5。
- `whole_body_teleop`および`rtmros_msg_bridge`のroot/nearest `AGENTS.md`: M0-B3、M0-B4。
- 全repository Project Contextの配置または正式defer判断とworkspace manifest: M0-B6。

M0-B3/M0-B4はProject Contextを作成・変更しない。M0-B6はrepository別sub-unitへ分割し、各sub-unitのWRITE repositoryを最大一つにする。

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

M0-Bでは、Project Contextの配置または正式defer判断をM0-B6が単独所有する。M0-B3/M0-B4のrepository/branch bootstrapへProject Contextを混在させない。

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

M0では次を特例ではなくformal gateとして適用する。

```text
M0-A-R1 fresh review
  -> 人間によるbaseline・diff・commit承認
  -> M0-A-R1 document-only commit
  -> M0-A5 central Progress sync
  -> M0-A complete
  -> M0-B1
```

M0-A5はapproved baseline、M0-A-R1 exact commit SHA、build・simulation・実機の`UNVERIFIED`状態をappend-onlyで記録する。M0-A5のreview、承認、commitが完了する前にM0-B1を開始しない。

### 13.4 Compatible set

Milestoneごとに名前を付ける。

```text
M0-pre-M5-baseline
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

M0 baselineではcurrent dirty workspaceをbaseline sourceまたはbuild sourceとして使わない。M0-B1で、current `${CATKIN_SOURCE_ROOT}`外のworktree配置、dedicated catkin workspaceのexact path、underlay、同名package重複回避、current generated artifact非混入を人間承認付きで固定する。M0-B1承認前にbranch、worktree、dedicated workspaceを作成しない。

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
- `whole_body_teleop_msgs`。
- `whole_body_teleop_reference_generator`。
- package-specific build説明。
- central plan/Progress参照の受け口。

Project Contextの配置とcentral plan/Progressのexact SHA固定はM0-B3では行わず、M0-B6の`whole_body_teleop` repository-specific sub-unitへdeferする。

### 16.2 `rtmros_msg_bridge`

実装branch作成時に追加する。

- rootまたは対象package `AGENTS.md`。
- bridge責務と非責務。
- message/IDL mapping review規約。

Project Contextの配置とexact SHA固定はM0-B4では行わず、M0-B6の`rtmros_msg_bridge` repository-specific sub-unitへdeferする。

### 16.3 `ik_solvers2`、`prioritized_qp`

変更が必要になった時点で、実装前に追加する。

- repository `AGENTS.md`。
- Project Context。
- backward compatibilityとdependent build規約。

変更不要の場合、bootstrapだけを目的に既存repositoryへcommitする必要はない。cross-repository taskではbranch/SHAを記録する。

Project Contextの配置または正式defer判断はM0-B6が単独所有する。変更不要でProject Contextをまだ配置しない場合、M0-B6がworkspace manifestへdefer状態と理由を記録し、M0-B3/M0-B4または別のbootstrap unitへownerを移さない。

---

## 17. M0 dependencyとM0-B再構成

### 17.1 M0 compatible set

named compatible set `M0-pre-M5-baseline`を次で固定する。

| repository | ref | SHA | status |
|---|---|---|---|
| `auto_stabilizer2` | historical `wbms-dev` ancestor | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `2.0` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `master` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | 未作成 | — | NOT APPLICABLE |

current `ik_solvers2/teleop-dev`および`prioritized_qp/teleop-dev`は観測中のdevelopment branchであり、M0 baselineの期待branchまたは期待SHAとして使用しない。

### 17.2 Formal dependency order

```text
M0-A1 branch/source archaeology
  -> M0-A2 dependency SHA selection
  -> M0-A4 historical walking evidence確認
  -> M0-A-R1 planning asset correction
  -> M0-A-R1 fresh review from CATKIN_SOURCE_ROOT
  -> 人間によるbaseline・diff・commit承認
  -> M0-A-R1 document-only commit
  -> M0-A5 central Progress sync
  -> M0-A complete
  -> M0-B1
  -> M0-B2
  -> M0-B3
  -> M0-B4
  -> M0-B5
  -> M0-B6
  -> M0-B7
  -> M0-B8
```

M0-A3はexecutable sub-unitとして廃止し、build commandの計画だけをM0-Aへ残す。M0-A5はcentral Progress syncだけを担当する。全package buildの唯一のownerはM0-B7とする。

### 17.3 M0-B責務

| unit | sole responsibility | WRITE boundary |
|---|---|---|
| `M0-B1` | current dirty workspaceを使用しない隔離方式、exact path、source-root外worktree配置、dedicated catkin workspace、underlay条件を人間承認付きで確定する | source/repository WRITEなし |
| `M0-B2` | `c06b63c`起点のisolated `auto_stabilizer2` implementation branchを作り、承認済みplanning assetをexact rangeと個別SHAで移植する | `auto_stabilizer2`だけ |
| `M0-B3` | `whole_body_teleop` repository/package bootstrapとroot/nearest `AGENTS.md`配置を行う。Project Contextは作成・変更せずM0-B6へdeferする | `whole_body_teleop`だけ |
| `M0-B4` | isolated `rtmros_msg_bridge` branchとroot/nearest `AGENTS.md`をbootstrapする。Project Contextは作成・変更せずM0-B6へdeferする | `rtmros_msg_bridge`だけ |
| `M0-B5` | `${CATKIN_SOURCE_ROOT}/AGENTS.md`配置、4 common Skillのuser-level symlink installation、各repository rootからのinstruction/Skill認識確認を所有する | Git repository WRITEなし。承認済みworkspace-level targetだけ |
| `M0-B6` | 全repository Project Contextの配置または正式defer状態と、workspace manifestの配置・exact SHA固定を単独所有する | repository別sub-unitへ分割し、各sub-unitは最大一つのWRITE repository。source-root manifest sub-unitはGit repository WRITEなし |
| `M0-B7` | exact compatible setのmaterializationと全package-specific baseline buildを単独所有し、exact command、execution directory、resultを記録する | source/repository WRITEなし。failure修正は別repository sub-unit |
| `M0-B8` | source rootからcentral Progress、instructions、Skill、Project Context、manifest、M0-B7 build resultをcross-repository fresh reviewする | 全repository READのみ |

M0-B1より先にM0-B2のbranch作成、worktree作成、dedicated workspace作成、buildを行わない。M0-A3、M0-A-R1、M0-A5はbuildを実行しない。

M0-B6を一つのmulti-repository implementation taskとして実行しない。Project Contextを実際に配置する場合は対象repositoryごとのsub-unitへ分割し、各sub-unitのWRITE repositoryを一つだけにする。`ik_solvers2`と`prioritized_qp`を変更しない場合はbootstrapだけを目的とするcommitを要求せず、M0-B6がworkspace manifestへ明示的なdefer状態と理由を記録する。

M0-B8がreviewするassetのproducerを次で固定する。

| M0-B8 review asset | sole producer | consumer condition |
|---|---|---|
| M0-A approved baselineとM0-A-R1 SHA | `M0-A5` | append-only central Progress sync済み |
| isolated auto_stabilizer branch/planning asset | `M0-B2` | exact rangeとM0-A-R1/M0-A5個別SHAを順番に移植済み |
| `whole_body_teleop` root/nearest instruction | `M0-B3` | Project Contextは未所有 |
| `rtmros_msg_bridge` root/nearest instruction | `M0-B4` | Project Contextは未所有 |
| source-root `AGENTS.md` | `M0-B5` | template/targetと認識結果を記録済み |
| 4 common Skill installation/recognition | `M0-B5` | symlink先と各rootの認識結果を記録済み |
| 全Project Contextおよび明示defer状態 | `M0-B6` | repository別one-write sub-unitとcompatible SHAを記録済み |
| workspace manifest | `M0-B6` | `M0-pre-M5-baseline`のexact 4 SHAとrepository statusが一致 |
| package build result | `M0-B7` | 全exact command、execution directory、resultを記録済み |
| cross-repository fresh review | `M0-B8` | 上記producer完了後、全repository READのみ |

### 17.4 Planning asset migration

既存planning assetの移植はcommit件数へ依存させず、次のexact rangeで記述する。

```text
range boundary parent:
  5c21cc0cb3c6ef6c906642836ddadf279c8266fd

first planning asset commit:
  8261114e9fe7bcd61fde14e12a6f5eda50b767d4

last existing planning asset commit:
  6e530edacecf663f2eedbdcfd5a787468dad70ce

exact revision range:
  5c21cc0cb3c6ef6c906642836ddadf279c8266fd..6e530edacecf663f2eedbdcfd5a787468dad70ce

equivalent inclusive cherry-pick range:
  8261114e9fe7bcd61fde14e12a6f5eda50b767d4^..6e530edacecf663f2eedbdcfd5a787468dad70ce
```

M0-A-R1 document correction commitとM0-A5 central Progress sync commitは上記rangeへ含めない。作成後のexact SHAを記録し、M0-B2で既存rangeの後に個別SHAとして順番に移植する。可変rangeを使用しない。

### 17.5 Acceptance

- `${CATKIN_SOURCE_ROOT}/AGENTS.md`が存在する。
- `${CATKIN_SOURCE_ROOT}/AGENTS.md`のsole ownerがM0-B5である。
- repository pathとroleが記録される。
- 実装対象repositoryにroot/nearest `AGENTS.md`がある。
- 4 Skillが各repository rootから認識される。
- M0-B3/M0-B4がProject Contextを作成・変更せず、M0-B6へdeferする。
- 全Project Contextの配置または正式defer判断とworkspace manifestのsole ownerがM0-B6である。
- M0-B6の実書込みがrepository別sub-unitへ分かれ、各sub-unitが最大一つのWRITE repositoryを持つ。
- one-write-repository ruleがContractへ反映される。
- Parent/Sub-unit形式が利用できる。
- workspace manifestの`M0-pre-M5-baseline`がexact 4 SHAと一致する。
- current dirty checkoutをbaseline sourceまたはbuild sourceに使用していない。
- M0-B7の全package-specific build command、execution directory、resultが記録される。
- workspace一括buildを要求していない。
- M0-B8がcentral Progress、Project Context、manifest、instructions、Skill、build resultをcross-repository reviewする。
- M0-B8の全review assetに一意のproducerがある。
- M0-B7完了前のbuild、simulation、実機は`UNVERIFIED`として維持する。

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
