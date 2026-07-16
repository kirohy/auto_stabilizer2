# WBMS外部Whole-Body操縦Reference Generator 実装計画 Revision 2

## 1. 文書の位置づけ

本書は、`WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`および`WBMSExternalWholeBodyTeleoperationImplementationPlan.md`に対し、複数repository運用、Codex起動directory、`AGENTS.md`、Skill、Parent/Sub-unit、package build、Progress同期を追加・修正する。

本書で扱う項目は本書を最優先する。それ以外はRevision 1および元計画を維持する。

参照順:

1. 本書。
2. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
3. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
4. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
5. `WBMSExternalWholeBodyTeleoperationProgress.md`。
6. 対象Work Unit Contract。
7. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md`。
8. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
9. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`。
10. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。

### 1.1 Workspace pathの定義

本プロジェクトでは次の変数を用いる。

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

`catkin_ws/src`を直接workspace layoutとして仮定しない。

---

## 2. Repository分割の正式仕様

### 2.1 Repository責務

| repository | role |
|---|---|
| `auto_stabilizer2` | 500 Hz consumer、安全制御、walking preparation、final IK、中央計画・Progress |
| `whole_body_teleop` | ROS message、入力差分、HMD、external whole-body IK、reference generator |
| `rtmros_msg_bridge` | ROS/RTM変換、latest-only transport |
| `ik_solvers2` | generic IK library |
| `prioritized_qp` | QP backend |

新規ROS node sourceを`auto_stabilizer2`へ置かない。`whole_body_teleop`を新規独立repositoryとして作成する。

### 2.2 One-write-repository rule

一つのimplementation taskが変更するGit repositoryは原則一つとする。

```text
one implementation task
  = one WRITE repository
  = one repository sub-unit
  = one atomic commit
```

複数repositoryへ影響する論理機能は、Parent Work Unitとrepository sub-unitへ分割する。

---

## 3. Codex起動directory

### 3.1 Cross-repository task

次から起動する。

```text
${CATKIN_SOURCE_ROOT}
```

対象:

- cross-repository planning。
- branch archaeology。
- Parent Contract。
- protocol全体review。
- compatible-set review。
- package build結果の統合確認。
- simulation計画、log解析。

これらは原則read-onlyとする。

### 3.2 Repository task

対象repository rootから起動する。

```text
${CATKIN_SOURCE_ROOT}/auto_stabilizer2
${CATKIN_SOURCE_ROOT}/whole_body_teleop
${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge
${CATKIN_SOURCE_ROOT}/ik_solvers2
${CATKIN_SOURCE_ROOT}/prioritized_qp
```

対象:

- implementation。
- repository review。
- finding修正。
- closure。
- commit。

---

## 4. `AGENTS.md`とSkill

### 4.1 三層構成

次を使用する。

1. `${CATKIN_SOURCE_ROOT}/AGENTS.md`: cross-repository workspace規約。
2. 各repository rootの`AGENTS.md`: repository責務とbuild規約。
3. 必要なpackage/moduleの`AGENTS.md`: 局所的な安全・実装規約。

`${CATKIN_SOURCE_ROOT}`から起動した場合、子repositoryの`AGENTS.md`が自動で全て適用されるとは仮定しない。source-root `AGENTS.md`から対象repositoryのinstructionを明示的に読むよう指示する。

### 4.2 共通Skill

次の4 Skillはproject共通とする。

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

正本は当面`auto_stabilizer2/.agents/skills`とする。各repositoryへcopyせず、M0-Bで`${HOME}/.agents/skills`へsymlinkし、全repository rootから認識できるようにする。

repository固有Skillは必要な場合だけ各repositoryへ置く。

### 4.3 Project Context

各repositoryに`docs/WBMSExternalTeleopProjectContext.md`を置く。

正式計画全文を複製せず、次を記録する。

- repository role。
- 正式計画、Revision、中央Progressのrepository/branch/SHA/path。
- current Parent/Sub-unit。
- compatible set。
- package build command。
- repository固有invariant。

M0-Bでは、全Project Contextの配置または正式defer判断とworkspace manifestをM0-B6が単独所有する。M0-B3/M0-B4はrepository/branchとroot/nearest instructionのbootstrapだけを担当し、Project Contextを作成・変更しない。

---

## 5. Parent Work UnitとSub-unit

複数repositoryへ影響する機能は、`${CATKIN_SOURCE_ROOT}`からread-only Parent Contractを作る。

例:

```text
M1-P: WBMS external teleoperation protocol
M1-A: whole_body_teleop ROS message
M1-B: auto_stabilizer RTM IDL
M1-C: rtmros_msg_bridge mapping
M1-D: compatible-set review
M1-E: package build integration check
```

M1-A、M1-B、M1-Cはそれぞれ一つのWRITE repositoryだけを持つ。

M1-D、M1-Eはcross-repository read-only taskとする。

schema未確定のproducer、bridge、consumerを並行実装しない。

---

## 6. Work Unit Contract拡張

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

implementation Contractでは`WRITE` repositoryを原則一つだけにする。

---

## 7. Build方針の修正

### 7.1 Workspace一括buildを標準にしない

引数なしの次のcommandを通常の受入条件にしない。

```sh
catkin build
```

### 7.2 対象package build

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

### 7.3 Dependency確認

依存関係のsource/API互換性まで確認する場合だけ`--no-deps`を外す。

```sh
catkin build <package-name>
```

目的と対象dependencyをContractとProgressへ記録する。

### 7.4 IDL/CMake変更

`auto_stabilizer`のIDL変更後の初回buildは次を維持する。

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

他packageでmessage/IDL/CMake生成の再構成が必要な場合は、Contractで`--force-cmake`を指定する。

### 7.5 実行directory

`catkin build`の実行directoryは固定しない。catkin workspace内でworkspaceを解決できる場所から実行してよい。

Progressには次を記録する。

- exact command。
- 実行directory。
- package名。
- `--no-deps`の有無。
- `--force-cmake`の有無。
- result。

### 7.6 Integration build

integration buildとは、関連packageのpackage-specific buildを順に実行することを意味し、workspace一括buildを意味しない。

build failureをcross-repository task内で複数repository同時修正しない。対象repositoryのfix sub-unitへ戻す。

---

## 8. 中央ProgressとCompatible Set

中央Progressの正本は当面次とする。

```text
auto_stabilizer2/auto_stabilizer/docs/
  WBMSExternalWholeBodyTeleoperationProgress.md
```

他repositoryでcommitした後、依存する次sub-unitへ進む前に中央ProgressへSHAを同期する。

```text
repository sub-unit commit
  -> central Progress sync
  -> compatible set更新
  -> dependent sub-unit開始
```

Milestoneごとに名前付きcompatible setを記録する。

```text
M0-pre-M5-baseline
M1-compatible-set
M3-compatible-set
M8-compatible-set
M11-release-candidate-set
```

未変更repositoryも使用中SHAを記録する。

M0では、M0-A-R1 document correctionのfresh review、人間承認、commit後に、M0-A5を独立したcentral Progress sync sub-unitとして行う。M0-A5はapproved baseline、M0-A-R1 exact commit SHA、build・simulation・実機の`UNVERIFIED`状態をappend-onlyで記録する。M0-A5のreview、承認、commitが完了する前にM0-B1を開始しない。

---

## 9. Worktree

`${CATKIN_SOURCE_ROOT}`内へ同一packageを持つ複数worktreeを配置しない。

read-only review worktreeはworkspace外へ置く。

```text
${HOME}/codex_worktrees/<repository>-review
```

buildが必要な比較branchは、専用catkin workspaceまたはpackage重複がない構成を使用する。

M0 baselineはcurrent dirty workspaceをbuild sourceとして使用しない。M0-B1で、current `${CATKIN_SOURCE_ROOT}`外のworktree配置、dedicated catkin workspaceのexact path、underlay、同名package重複回避、current generated artifact非混入を人間承認付きで固定する。M0-B1承認前にbranch、worktree、dedicated workspaceを作成しない。

---

## 10. M0 baselineとM0-Bの再構成

### 10.1 M0 compatible set

named compatible set `M0-pre-M5-baseline`を次で固定する。

| repository | ref | SHA | status |
|---|---|---|---|
| `auto_stabilizer2` | historical `wbms-dev` ancestor | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `2.0` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `master` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | 未作成 | — | NOT APPLICABLE |

current `ik_solvers2/teleop-dev`および`prioritized_qp/teleop-dev`は観測中のdevelopment branchであり、M0 baselineの期待branchまたは期待SHAとして使用しない。

### 10.2 Final IK差分

| 項目 | `1.0` `641f4c8f88a2da8413a8e067c841260b0584a601` | baseline `c06b63c8e12dbf85bda4c8391a37544c2469731c` | `wbms-dev` `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` |
|---|---|---|---|
| priority構造 | 3層 | 足、通常task、reference angleを分離した5層 | 5層を維持 |
| 上半身EE | world target | WBMS active時はCHEST相対 | CHEST相対を維持 |
| WBMS CHEST/root | CHEST taskなし、root `PositionConstraint` | 並進weightを0にした`PositionConstraint`で姿勢を扱い、walking preparation root targetを保持 | `OrientationConstraint`へ変更 |
| iteration | `maxIteration=1` | `maxIteration=1` | `maxIteration=1` |
| final-state再評価 | 既存solver挙動 | 既存solver挙動、明示的skipなし | `checkFinalState=false`で省略 |
| QP workspace | なし | なし | persistent QP workspaceあり |
| profiling/reference診断 | なし | walking preparationに必要なfinal IK診断のみ | 旧M5 profiling、workspace counter、reference-angle統計を追加 |
| baseline判定 | WBMS walking preparation前 | 採用 | 旧M5差分を含むため不採用 |

`7df08e7f275127102f20e9d3f3fea37b6fec655f^`はexactに`c06b63c`である。CHEST相対腕拘束とM4.2.2の必要な全安全修正は`c06b63c`のancestorであり、旧M5はその子commitから始まるため、synthetic baselineは不要である。

### 10.3 Formal dependency order

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

### 10.4 M0-B責務

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

M0-B1より先にM0-B2のbranch作成、worktree作成、dedicated workspace作成、buildを行わない。

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

### 10.5 Planning asset migration

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

### 10.6 M0-B acceptance

- `${CATKIN_SOURCE_ROOT}/AGENTS.md`がある。
- `${CATKIN_SOURCE_ROOT}/AGENTS.md`のsole ownerがM0-B5である。
- repository pathとroleが記録されている。
- 実装対象repositoryにroot/nearest `AGENTS.md`がある。
- 4つの共通Skillが各repository rootから認識される。
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

## 11. 既存文書・Skillへの適用

### 11.1 直接更新したもの

- `auto_stabilizer/AGENTS.md`。
- 4つのWork Unit Skill。
- `WBMSExternalWholeBodyTeleoperationProgress.md`。

### 11.2 Revisionで上書きするもの

- `WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md`を追加し、元Operator Guideの複数repository部分を上書きする。
- `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`と`WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`の一般原則は維持する。
- Workflow/Official Guidanceに残る起動directory、Skill配置、workspace一括buildに関する古い例は、本書とMultiRepositoryOperationsへ読み替える。

既存文書に残る`${CATKIN_WS}/src`またはworkspace一括buildの例は、本Revisionの`${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`、package-specific buildへ読み替える。

---

## 12. Bootstrap template

M0-B用の正本templateを`auto_stabilizer2/tools/codex_workspace/`へ置く。

含むもの:

- source-root `AGENTS.md` template。
- `whole_body_teleop` `AGENTS.md` template。
- `rtmros_msg_bridge` `AGENTS.md` template。
- `ik_solvers2` `AGENTS.md` template。
- `prioritized_qp` `AGENTS.md` template。
- repository Project Context template。
- workspace manifest template。

実際の他repository配置とbootstrap script実装はM0-B sub-unitで行う。

---

## 13. 禁止事項

- `catkin_ws/src`を固定layoutとして扱わない。
- cross-repository implementationを一つの巨大taskで行わない。
- `${CATKIN_SOURCE_ROOT}`から複数repositoryを同時編集しない。
- repository固有`AGENTS.md`を読まずにcross-repo判断しない。
- 共通Skillを各repositoryへcopyして別version化しない。
- schema未確定でproducer、bridge、consumerを並行実装しない。
- workspace一括buildを暗黙の受入条件にしない。
- 他repository commit SHAを中央Progressへ記録せず依存sub-unitへ進まない。
- source root内へ同一packageの複数worktreeを置かない。
- simulationまたは実機を無許可で開始しない。
