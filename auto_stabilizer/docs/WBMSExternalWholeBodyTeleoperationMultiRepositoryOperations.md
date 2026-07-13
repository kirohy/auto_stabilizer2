# WBMS外部Whole-Body操縦 複数リポジトリ運用仕様

## 1. 文書の位置づけ

本書は、WBMS外部whole-body操縦プロジェクトを、同一catkin workspace内の複数Git repositoryに分割して実装・review・検証・commitするための正式運用仕様である。

対象repositoryは次を基本とする。

```text
${CATKIN_WS}/src/
  auto_stabilizer2/
  whole_body_teleop/
  rtmros_msg_bridge/
  ik_solvers2/
  prioritized_qp/
```

新規ROS nodeは`whole_body_teleop`で管理する。ROS/RTM変換は`rtmros_msg_bridge`、500 Hzのconsumerと安全制御は`auto_stabilizer2`、IK/QP基盤は`ik_solvers2`と`prioritized_qp`で管理する。

本書は、複数repositoryの起動directory、`AGENTS.md`、Skill、Work Unit、build、Progress、compatible SHA、commit、worktree、simulation gateを定義する。

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

## 2. 基本原則

### 2.1 一つのimplementation taskが書き換えるrepositoryは一つ

原則は次とする。

```text
一つのimplementation task
  = 一つのwrite repository
  = 一つのrepository sub-unit
  = 原則一つのatomic commit
```

同一taskで複数repositoryを同時に書き換えない。

理由:

- repositoryごとの`AGENTS.md`、build、review、commitを明確にできる。
- schema、producer、bridge、consumerの責務を分離できる。
- compatible SHAを追跡できる。
- review findingの修正範囲を限定できる。
- userの未commit変更を別repositoryまで巻き込まない。

複数repositoryを同時に書き換える例外は、repository移動自体が目的である場合などに限定し、Parent Contractとユーザーの明示承認を必要とする。

### 2.2 Cross-repository taskは原則read-only

次は`${CATKIN_WS}/src`から行う。

- cross-repository planning
- branch archaeology
- Parent Work Unit Contract作成
- protocol全体設計
- compatible-set review
- cross-repository interface review
- package build結果の統合確認
- simulation計画とlog解析

これらのtaskは原則read-onlyであり、source修正が必要になった場合は対象repositoryのsub-unitへ分離する。

### 2.3 Repository taskは対象repository rootから行う

次は対象repository rootから行う。

- repository sub-unitの実装
- repository単位のreview
- finding修正
- repository単位のclosure
- atomic commit

例:

```sh
codex -C ${CATKIN_WS}/src/auto_stabilizer2
codex -C ${CATKIN_WS}/src/whole_body_teleop
codex -C ${CATKIN_WS}/src/rtmros_msg_bridge
```

対象repository rootから起動することで、そのrepositoryのroot `AGENTS.md`とpackage/module固有`AGENTS.md`を適用しやすくする。

---

## 3. Codex起動directory

### 3.1 Source root

cross-repository taskの共通起点は次とする。

```text
${CATKIN_WS}/src
```

catkin workspace rootではなくsource rootを使用する理由:

- 対象Git repositoryが全て直下にある。
- `build`、`devel`、`install`、`logs`等を通常のsource探索対象から外せる。
- cross-repositoryのpathを一定にできる。
- package buildは任意のworkspace内directoryから実行可能であり、workspace rootを起点にする必要がない。

### 3.2 起動directory表

| task | Codex起動directory | write可能範囲 |
|---|---|---|
| cross-repo planning | `${CATKIN_WS}/src` | なし |
| branch archaeology | `${CATKIN_WS}/src` | なし |
| Parent Contract | `${CATKIN_WS}/src` | 原則なし |
| protocol全体review | `${CATKIN_WS}/src` | なし |
| compatible-set review | `${CATKIN_WS}/src` | なし |
| package build統合確認 | `${CATKIN_WS}/src`または任意のworkspace内directory | source変更なし |
| simulation計画・log解析 | `${CATKIN_WS}/src` | 明示されたlog/文書だけ |
| repository implementation | 対象repository root | 対象repositoryのみ |
| repository review | 対象repository root | なし |
| repository finding修正 | 対象repository root | 対象repositoryのみ |
| repository commit | 対象repository root | 対象repositoryのみ |

### 3.3 `src`からのimplementationは禁止しないが既定にしない

`${CATKIN_WS}/src`から起動したCodexに特定repositoryだけを書かせることは技術的には可能である。しかし、instruction chain、write範囲、dirty state、commit対象を誤りやすいため、通常のimplementationはrepository rootから起動する。

---

## 4. `AGENTS.md`の三層構成

### 4.1 Source-root `AGENTS.md`

配置:

```text
${CATKIN_WS}/src/AGENTS.md
```

これはlocal workspace用であり、通常は単独のGit repositoryに属さない。`auto_stabilizer2`内のtemplateからbootstrapする。

責務:

- 対象repository一覧とpath。
- cross-repository taskは原則read-only。
- implementation taskのwrite repositoryは一つ。
- 各repositoryを扱う前に、そのrootとnearest `AGENTS.md`を明示的に読む。
- 全repositoryのbranch、HEAD、dirty stateを確認する。
- user変更をreset、stash、cleanしない。
- `build`、`devel`、`install`、`logs`をsource変更として扱わない。
- package単位buildを用いる。
- simulationと実機は明示許可を必要とする。

Codexを`${CATKIN_WS}/src`から起動しても、子repository内の`AGENTS.md`が自動的に全て適用されるとは仮定しない。source-root `AGENTS.md`は、対象repositoryのinstructionを明示的に読むよう指示する。

### 4.2 Repository-root `AGENTS.md`

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
- HMD、head recenter。
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

### 4.3 Package/module-level `AGENTS.md`

必要な場合だけ追加する。

例:

```text
auto_stabilizer2/auto_stabilizer/AGENTS.md
whole_body_teleop/whole_body_teleop_msgs/AGENTS.md
whole_body_teleop/whole_body_teleop_reference_generator/AGENTS.md
rtmros_msg_bridge/<bridge-package>/AGENTS.md
```

message packageと制御nodeの規約が異なる場合は分離する。

---

## 5. 共通Skillの配置

### 5.1 共通Skill

プロジェクト共通Skillは次である。

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

これらを各repositoryへコピーして複製管理しない。

### 5.2 正本

当面の正本は次とする。

```text
${CATKIN_WS}/src/auto_stabilizer2/.agents/skills/
```

Skillの意味は`auto_stabilizer2`専用ではなく、本プロジェクトの全repository共通とする。

### 5.3 user-level installation

repository rootから起動したtaskでも共通Skillを利用できるよう、bootstrap時にuser-levelへsymlinkする。

```text
${HOME}/.agents/skills/wbms-plan-work-unit
  -> ${CATKIN_WS}/src/auto_stabilizer2/.agents/skills/wbms-plan-work-unit
```

同様に4 Skillをlinkする。

理由:

- `whole_body_teleop`や`rtmros_msg_bridge`から起動しても認識できる。
- Skill本文の正本を一つにできる。
- repositoryごとのcopy driftを防げる。

新PCではbootstrapが必要である。Skill認識はM0-Bで確認する。

### 5.4 Source-root Skill

`${CATKIN_WS}/src/.agents/skills`へ同名Skillを重複配置することは既定にしない。user-levelで認識できない環境のみ、同じ正本へのsymlinkを使用する。

同名Skillが複数scopeに存在する構成は、どのSkillが選択されたか分かりにくくなるため避ける。

### 5.5 Repository固有Skill

repository固有の反復作業だけを各repositoryへ置いてよい。

例:

```text
whole_body_teleop/.agents/skills/wbms-build-teleop-package/
rtmros_msg_bridge/.agents/skills/wbms-check-bridge-roundtrip/
auto_stabilizer2/.agents/skills/wbms-analyze-stabilizer-log/
```

共通Work Unit lifecycleをrepo固有Skillへ複製しない。

---

## 6. Workspace bootstrap

### 6.1 Bootstrapの目的

M0-Bで、source rootと各repositoryのCodex運用を初期化する。

必須成果物:

1. `${CATKIN_WS}/src/AGENTS.md`
2. user-level共通Skill symlink
3. 各repositoryのroot `AGENTS.md`
4. 各repositoryのProject Context
5. repository path/role manifest
6. Skillとinstruction chainの認識確認

### 6.2 Bootstrap script

後続の独立Work Unitで、次のscriptを作成してよい。

```text
auto_stabilizer2/tools/codex_workspace/bootstrap_codex_workspace.sh
auto_stabilizer2/tools/codex_workspace/verify_codex_workspace.sh
```

bootstrapは次を守る。

- workspace pathを引数で受ける。
- 既存fileを無断で上書きしない。
- symlink先を表示する。
- repositoryの存在を確認する。
- branch、HEAD、dirty stateを表示する。
- 不一致を修正せず報告する。
- source codeを変更しない。

script作成は本書追加と同じWork Unitへ混ぜない。

---

## 7. Project Context

各repositoryへ短い文書を置く。

推奨path:

```text
docs/WBMSExternalTeleopProjectContext.md
```

内容:

- project ID。
- 当該repositoryのrole。
- 正式計画のrepository、branch、commit、path。
- 中央Progressのrepository、branch、commit、path。
- 関連repository一覧。
- 現在のParent Work Unitとsub-unit。
- current compatible set。
- repository固有build command。
- repository固有invariant。

正式Implementation Plan全文を複製しない。参照先と固定SHAを記録する。

---

## 8. Parent Work UnitとRepository Sub-Unit

### 8.1 Parent Work Unit

複数repositoryへ影響する論理機能は、source rootからread-onlyでParent Contractを作る。

例:

```text
M1-P: WBMS external teleoperation protocol
```

Parent Contractで固定する。

- schema version。
- 論理field。
- frame、単位、quaternion順序。
- task mask、enum。
- session、epoch、sequence、timestamp。
- compatibility rule。
- repository sub-unit一覧。
- dependency順。
- cross-repository acceptance。

### 8.2 Repository Sub-Unit

例:

```text
M1-A: whole_body_teleop ROS message
M1-B: auto_stabilizer RTM IDL
M1-C: rtmros_msg_bridge mapping
M1-D: compatible-set review
M1-E: package build integration check
```

M1-A、M1-B、M1-Cはそれぞれ対象repository rootから実装する。

M1-DとM1-Eはsource rootからread-onlyで行う。

### 8.3 Sub-unitのdependency

後続sub-unitは、依存sub-unitのcommit SHAまたは明示的なuncommitted compatible stateをContractへ記録する。

schema未確定のままproducer、bridge、consumerを並行実装しない。

---

## 9. Work Unit Contract拡張

複数repositoryが関係するContractには次を追加する。

```markdown
## Workspace context
- catkin workspace root
- source root
- Codex launch directory

## Repository access
| repository | path | branch | base SHA | access |

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
| package | command | dependency scope |

## Cross-repository acceptance
```

`access`は`READ`、`WRITE`、`NONE`を明示する。

implementation Contractでは`WRITE` repositoryを原則一つだけにする。

---

## 10. Build方針

### 10.1 Workspace一括buildは行わない

本プロジェクトでは、引数なしのworkspace全体buildを標準手順にしない。

```sh
catkin build
```

を受入条件として要求しない。

### 10.2 対象packageだけをbuildする

通常は次を使う。

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

### 10.3 依存関係まで確認する場合

依存関係のsource/API互換性までbuildで確認する必要がある場合だけ`--no-deps`を外す。

```sh
catkin build <package-name>
```

どのdependencyを確認する目的かContractへ記載する。

### 10.4 IDL/CMake生成変更

`auto_stabilizer`のIDL変更後の初回buildは次を使う。

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

他packageでもmessage/IDL/CMake生成の再構成が必要な場合は、そのpackageのContractで`--force-cmake`を指定する。

### 10.5 実行directory

`catkin build`の実行directoryは固定しない。catkin workspace内でworkspaceを解決できる場所から実行してよい。

ただしProgressには次を記録する。

- exact command。
- commandを実行したdirectory。
- package名。
- `--no-deps`の有無。
- `--force-cmake`の有無。
- result。

### 10.6 Cross-repository integration build

integration buildは、必要なpackage-specific commandを順に実行することを意味する。workspace一括buildを意味しない。

例:

```text
1. catkin build whole_body_teleop_msgs --no-deps
2. catkin build whole_body_teleop_reference_generator --no-deps
3. catkin build whole_body_teleop_rtmros_bridge --no-deps
4. catkin build auto_stabilizer --no-deps --force-cmake
```

dependency compatibilityを確認する場合だけ、対象packageについて`--no-deps`を外した追加buildを行う。

### 10.7 Build failureの扱い

cross-repository build taskは、その場で複数repositoryを修正しない。

```text
build failure
  -> errorとroot cause候補を記録
  -> 対象repositoryを特定
  -> repository-specific fix sub-unitを作成
  -> 対象repository rootから修正
  -> package buildを再実行
```

---

## 11. Review構成

### 11.1 Repository review

対象repository rootから行う。

- 当該repositoryのdiff。
- 当該repositoryのAGENTS。
- Parent/Sub-unit Contract。
- sibling repositoryはcompatible inputとしてread-only参照。

### 11.2 Cross-repository review

`${CATKIN_WS}/src`からread-onlyで行う。

重点:

- message、IDL、bridge mapping。
- field、型、単位、frame、quaternion順序。
- enum/task mask/schema version。
- session、epoch、sequence。
- producer/consumer compatible SHA。
- package build順。
- central ProgressとProject Contextの一致。

cross-repository reviewerはsourceを修正しない。findingは対象repository sub-unitへ割り当てる。

---

## 12. Commitと中央Progress

### 12.1 Repository単位commit

commitは対象repository rootで行う。

- 一つの主要目的。
- 対象repositoryだけstage。
- `git add -A`を既定にしない。
- push、merge、PRは別許可。

### 12.2 中央Progress

正式な全体Progressの正本は当面次とする。

```text
auto_stabilizer2/auto_stabilizer/docs/
  WBMSExternalWholeBodyTeleoperationProgress.md
```

### 12.3 他repository commit後の同期

`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`でcommitした後、依存する次sub-unitへ進む前に中央Progressへcommit SHAを記録する。

流れ:

```text
repository sub-unit実装・review・commit
  -> repository commit SHA取得
  -> central Progress sync sub-unit
  -> compatible set更新
  -> dependent sub-unit開始
```

central Progress syncはdocument-onlyの独立commitとしてよい。

### 12.4 Repository内の一時記録

他repositoryのcommit前は、Project ContextまたはWork Unit reportへ次を記録する。

- Work Unit ID。
- branch/base/current SHA。
- changed files。
- build/check。
- review。
- unverified。
- expected commit subject。
- central Progress sync pending。

### 12.5 Compatible set

Milestoneごとに名前付きcompatible setを記録する。

```text
M1-compatible-set
M3-compatible-set
M8-compatible-set
M11-release-candidate-set
```

形式:

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

## 13. Worktree

### 13.1 catkin source root内へ同一packageを重複配置しない

次を避ける。

```text
${CATKIN_WS}/src/auto_stabilizer2
${CATKIN_WS}/src/auto_stabilizer2-review
```

同名packageが複数発見される可能性がある。

### 13.2 Review worktree

read-only review用worktreeはworkspace外へ置く。

```text
${HOME}/codex_worktrees/auto_stabilizer2-review
```

### 13.3 Buildが必要な別branch

buildが必要な比較branchは、専用catkin workspaceまたはpackage重複が起きない構成を使う。

`1.0`、pre-M5、新branchを比較する場合、同一source rootへ同名packageを複数置かない。

---

## 14. Simulationと実機

### 14.1 Simulation gate

simulationはsource rootから計画できるが、起動・command送信・log取得はユーザーの明示許可を必要とする。

初期は人間が既存runbookで実行し、Codexへ次を渡す。

- exact command。
- branch/SHA。
- dependency SHA。
- parameter。
- scenario。
- log path。
- 目視所見。

### 14.2 自動化

同じscenarioを反復する場合、別Work Unitでrunbook、script、timeout、abort、log収集を実装する。

simulation Skillと実機Skillは分離する。

### 14.3 実機

実機実行は常に別の明示許可を必要とする。build成功やsimulation成功から自動的に実機実行へ進まない。

---

## 15. Repository bootstrap要件

### 15.1 `whole_body_teleop`

新規repository作成時に最低限追加する。

- root `AGENTS.md`。
- `docs/WBMSExternalTeleopProjectContext.md`。
- `whole_body_teleop_msgs`。
- `whole_body_teleop_reference_generator`。
- package-specific build説明。
- central plan/Progressの固定参照。

### 15.2 `rtmros_msg_bridge`

実装branch作成時に追加する。

- rootまたは対象package `AGENTS.md`。
- Project Context。
- bridge責務と非責務。
- message/IDL mapping review規約。

### 15.3 `ik_solvers2`、`prioritized_qp`

変更が必要になった時点で、実装前に追加する。

- repository `AGENTS.md`。
- Project Context。
- backward compatibilityとdependent build規約。

変更不要の場合、bootstrapのためだけに既存repositoryへcommitする必要はない。ただしcross-repository taskでは現在のbranch/SHAを記録する。

---

## 16. M0-Bへ追加する作業

M0-Bを次のsub-unitへ分割する。

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

M0-B acceptance:

- source-root `AGENTS.md`が存在する。
- 全対象repository pathが記録されている。
- 実装対象repositoryにroot/nearest `AGENTS.md`がある。
- 共通4 Skillが各repository rootから認識される。
- one-write-repository ruleがContractへ反映される。
- Parent/Sub-unit形式が使用できる。
- package-specific build commandが記録されている。
- workspace一括buildを要求していない。
- central ProgressとProject Contextの同期方法が確認される。

---

## 17. 標準prompt

### 17.1 Cross-repository planning

```text
$wbms-plan-work-unit を使用してください。

Work Unit: <Parent IDとtitle>
Codex起動directory: ${CATKIN_WS}/src

このtaskはcross-repository read-only planningです。
全対象repositoryのbranch、HEAD、dirty stateを確認し、各repositoryのAGENTS.mdを明示的に読んでください。
source、branch、index、working treeを変更しないでください。

Parent Contractとrepository sub-unit一覧を作成してください。
各sub-unitのWRITE repositoryは一つにしてください。
```

### 17.2 Repository implementation

```text
$wbms-implement-work-unit を使用してください。

Work Unit: <Sub-unit IDとtitle>
Codex起動directory: ${CATKIN_WS}/src/<repository>
WRITE repository: <repository>
READ-ONLY sibling repositories: <list>

承認済みContractの範囲だけを実装してください。
他repositoryを変更しないでください。
指定されたpackage-specific buildを実行してください。
commitは行わないでください。
```

### 17.3 Cross-repository compatible-set review

```text
$wbms-review-work-unit を使用してください。

Review type: cross-repository compatible set
Codex起動directory: ${CATKIN_WS}/src
Parent Work Unit: <ID>
Compatible set: <repository SHA table>

全repositoryはread-onlyです。
message、IDL、bridge、producer、consumer、frame、unit、enum、schema、build結果、Project Context、中央Progressを照合してください。
sourceは変更しないでください。
```

---

## 18. 禁止事項

- cross-repository implementationを一つの巨大taskで行わない。
- source rootから複数repositoryを同時編集しない。
- repository固有`AGENTS.md`を読まずにcross-repo判断しない。
- 共通Skillを各repositoryへ手動コピーして別version化しない。
- schema未確定でproducer、bridge、consumerを並行実装しない。
- workspace一括buildを暗黙の受入条件にしない。
- build failureをcross-repo task内で複数repository同時修正しない。
- 他repository commit SHAを中央Progressへ記録せず依存sub-unitへ進まない。
- catkin source root内へ同一packageの複数worktreeを置かない。
- simulationまたは実機を無許可で開始しない。

---

## 19. 変更管理

本書を変更する場合:

1. 変更理由を中央Progressへ追記する。
2. `ImplementationPlanRevision2`との整合を確認する。
3. package `AGENTS.md`と4 Skillを確認する。
4. workspace templateとProject Context templateを確認する。
5. workflow変更だけの独立commitとする。
6. control source変更と混ぜない。
