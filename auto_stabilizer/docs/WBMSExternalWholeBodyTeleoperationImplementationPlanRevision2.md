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
M1-compatible-set
M3-compatible-set
M8-compatible-set
M11-release-candidate-set
```

未変更repositoryも使用中SHAを記録する。

---

## 9. Worktree

`${CATKIN_SOURCE_ROOT}`内へ同一packageを持つ複数worktreeを配置しない。

read-only review worktreeはworkspace外へ置く。

```text
${HOME}/codex_worktrees/<repository>-review
```

buildが必要な比較branchは、専用catkin workspaceまたはpackage重複がない構成を使用する。

---

## 10. M0-Bの再構成

M0-Bを次へ分割する。

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

M0-B acceptanceへ追加する。

- `${CATKIN_SOURCE_ROOT}/AGENTS.md`がある。
- repository pathとroleが記録されている。
- 実装対象repositoryにroot/nearest `AGENTS.md`がある。
- 4つの共通Skillが各repository rootから認識される。
- one-write-repository ruleがContractへ反映される。
- Parent/Sub-unit形式が利用できる。
- package-specific build commandが記録される。
- workspace一括buildを要求していない。
- central ProgressとProject Contextの同期方法が確認される。

---

## 11. 既存文書への適用

次の文書・Skillは本RevisionとMultiRepositoryOperationsを読むよう更新する。

- `auto_stabilizer/AGENTS.md`。
- `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
- `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`。
- `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
- 4つのWork Unit Skill。
- `WBMSExternalWholeBodyTeleoperationProgress.md`。

既存文書に残る`${CATKIN_WS}/src`またはworkspace一括buildの例は、本Revisionの`${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`、package-specific buildへ読み替える。

---

## 12. 禁止事項

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
