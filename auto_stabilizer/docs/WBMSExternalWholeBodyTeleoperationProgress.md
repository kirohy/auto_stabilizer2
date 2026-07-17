# WBMS外部Whole-Body操縦 作業進捗記録

## 1. 文書の位置づけ

本書は、WBMS外部whole-body操縦プロジェクトの中央Progressである。

別task、別担当、context圧縮後でも作業を再開できる粒度で、実施済み作業、判断、変更、検証、review、compatible SHA、next entry pointを時系列に記録する。

参照順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`: 複数repository運用を含む最新計画修正。
2. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`: 起動directory、AGENTS、Skill、Parent/Sub-unit、build、Progress同期。
3. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`: branch基点、heartbeat等の初回修正。
4. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`: 制御仕様とMilestone。
5. 本書: 実施済み作業、検証結果、compatible SHA。
6. 対象Work Unit Contract。
7. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md`: 複数repositoryの実操作。
8. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`: Work Unit lifecycle。
9. `WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`: 初期operator guide。
10. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`: OpenAI公式Codex情報への対応。
11. `WBMSWalkingPreparationDesignRevisionPlan.md`: walking preparationの既存正式仕様。
12. `WBMSFeasibleVelocityPostureControlProgress.md`: 旧projection IK構成の履歴。

過去entryの誤りを静かに書き換えない。訂正は新しいentryとして追記する。

### 1.1 2026-07-13 planning branch内再構成

PLAN-0Cで、まだformal detached review前のplanning branchにあったPLAN-0とPLAN-0Bを、複数repository仕様と新しい参照順へ合わせて本書へ再構成した。

- 過去の主要判断、unverified、next entry point、commit SHAを保持した。
- control sourceは変更していない。
- PLAN-0C以後はappend-onlyとする。

---

## 2. Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

---

## 3. Status一覧

| Status | 意味 |
|---|---|
| PLANNED | Contract作成前または承認待ち |
| IN_PROGRESS | 実装・調査中 |
| BLOCKED | 外部判断、環境、依存変更待ち |
| IMPLEMENTED | source変更完了、review前 |
| REVIEWED | P0/P1/P2 finding解消済み |
| VERIFIED | 指定build/simulation/受入確認済み |
| COMMITTED | atomic commit作成済み |

---

## 4. Work Unit entry template

```markdown
## YYYY-MM-DD <Work Unit ID> <title>

### Status

### Workspace context
- catkin workspace root
- source root
- Codex launch directory

### Repository state
| repository | branch | base SHA | current SHA | access | dirty |

### Goal

### Scope and out-of-scope

### Code investigation

### Decisions
- adopted
- rejected and reason

### Changes
| repository | file | change | reason |

### Commands and results
| execution directory | command | result | evidence |

### Simulation / log evidence

### Review
| round | reviewer/task | findings | resolution |

### Acceptance
| criterion | result | evidence |

### Unverified

### Open issues

### Compatible dependency set
| repository | SHA |

### Central Progress sync
- required / completed / pending

### Next entry point

### Commit
- repository
- SHA
- subject
- cherry-pick notes
```

---

## 2026-07-13 PLAN-0 外部Whole-Body操縦方針・実装計画策定

### Status

COMMITTED

### Workspace context

未確定。GitHub connector上でplanning branchを作成した。

### Repository state

| repository | branch | base SHA | current SHA | access | dirty |
|---|---|---|---|---|---|
| `kirohy/auto_stabilizer2` | `wbms-external-teleop-plan` | `wbms-dev` HEAD `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | 文書commit群 | WRITE | local未確認 |
| `kirohy/rtmros_msg_bridge` | 未作成 | `master`想定 | 未変更 | NONE | 未確認 |
| `kirohy/ik_solvers2` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |
| `kirohy/prioritized_qp` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |
| `whole_body_teleop` | 未作成 | - | - | NONE | - |

### Goal

500 Hzの`auto_stabilizer`クリティカルパスからprojection IKを除去し、独立ROS processでwhole-body referenceを生成する新アーキテクチャの正式仕様とCodex運用手順を確定する。

### Scope and out-of-scope

含む:

- 外部ROS generator、bridge、auto_stabilizer統合設計。
- 手、CHEST、COM、頭部、`q_nominal`。
- non-IK joint reference override。
- stale、session、heartbeat。
- walking preparation維持。
- Milestone、acceptance、Codex workflow。

含まない:

- source code実装。
- exact pre-M5 base SHAの確定。
- new repository作成。
- build、simulation、実機確認。

### Code investigation

- `jointControllable=false`関節はfinal IK変数から外れ、出力時に`refRobotRaw`が用いられる。
- `qRef`は`refRobotRaw`へ読み込まれ、その後FK/COM計算とframe変換が行われる。
- 首・将来指関節は、qRef読込後・FK前の許可付きoverrideでfinal IK変数を増やさずモデルと出力を整合できる。
- 現行腕差分操縦はmaster/slave開始poseを保存し、並進差分だけをscaleし、CHEST相対hand targetを生成する。
- M4.2.2 walking preparationはCOM高さ保持、RETURNING/HANDOFF、READY、歩行API gate、歩行中腕継続を担当するためauto_stabilizer内部へ残す。
- 既存`rtmros_msg_bridge`はROS callbackとRTM port変換を行う。新bridgeも変換責務に限定し、独立processとする。

### Decisions

採用:

- external whole-body IKは同一PCの独立ROS process、初期100 Hz候補。
- bridgeは`hrpEC`外の独立process。
- auto_stabilizerはpre-M5 final IKを初期baselineとする。
- projection IKを削除する。
- M4.2.2 walking preparationを維持する。
- 左右手はsession開始差分、並進scale parameter、CHEST相対target。
- CHEST/COM速度はfoot-mid基準。
- HMD差分から首yaw/pitch、roll/position無視。
- `recenter_head` service。
- 首はnon-IK reference override。
- `q_nominal`は低優先度soft reference。
- static両足支持だけCHEST/COMを許可。
- 歩行中は腕と頭部のみexternal操作。
- group別stale、現在姿勢hold。
- legacy interfaceを初回は残す。
- external IK self collisionは段階追加し、床到達受入前に必須化。

後続:

- single-QP final WBC。
- walking中CHEST/COM。
- rigid bimanual coupling。
- finger実運用。
- payload/把持。
- legacy cleanup。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `WBMSExternalWholeBodyTeleoperationImplementationPlan.md` | 正式実装計画を追加 | 新アーキテクチャ仕様 |
| `auto_stabilizer2` | `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md` | Codex作業標準を追加 | Work Unit反復 |
| `auto_stabilizer2` | 本Progress | 進捗文書を追加 | 引き継ぎ |

### Commands and results

| execution directory | command | result | evidence |
|---|---|---|---|
| GitHub connector | create branch `wbms-external-teleop-plan` from `wbms-dev` | PASS | branch作成 |
| GitHub connector | create implementation plan / workflow / Progress | PASS | commit SHA取得 |
| local | `git diff --check` | NOT RUN | local checkoutなし |
| local | `catkin build auto_stabilizer --no-deps` | NOT RUN | source変更なし、local環境未使用 |

### Simulation / log evidence

なし。旧構成の挙動根拠は既存ProgressとM4.2.2/M5ログを参照した。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| 0 | 仕様対話 | interface、stale、head、branch、priorityの未確定点 | ユーザー回答により解消 |
| 1 | 文書自己点検 | 実装branchと計画branchの分離 | plan branchを作成し、pre-M5 branchへ文書だけ移植する方針 |

formal detached reviewは未実施。

### Acceptance

| criterion | result | evidence |
|---|---|---|
| 主要仕様のユーザー判断が確定 | PASS | 手、CHEST、COM、head、stale、branch、legacy、priority |
| 新アーキテクチャ計画 | PASS | Implementation Plan |
| Codex反復workflow | PASS | Codex Workflow |
| control source未変更 | PASS | 文書のみ |
| exact pre-M5 base SHA | PENDING | M0-A |
| performance threshold | PENDING | baseline後 |

### Unverified

- M4.2.2全review修正を含むpre-M5またはsynthetic baseline SHA。
- dependency repository HEAD。
- pre-M5 baseline build。
- external generator solve時間。
- actual hand tracking error。

### Open issues

- M0-A branch archaeology。
- external node用repository作成。
- planning文書の実装branch移植。

### Compatible dependency set

未確定。

### Central Progress sync

本repository内で完了。

### Next entry point

PLAN-0B: Codex運用文書、Skill、公式情報照合。

### Commit

| repository | SHA | subject |
|---|---|---|
| `auto_stabilizer2` | `8261114e9fe7bcd61fde14e12a6f5eda50b767d4` | `Add external whole-body teleoperation implementation plan` |
| `auto_stabilizer2` | `e568de33d9e74bea686433c9a0fa8f38daecbc75` | `Add Codex workflow for external teleoperation project` |
| `auto_stabilizer2` | `f8108e8d65586754bf5754f5746f04b50e0ba899` | `Add progress log for external teleoperation project` |

---

## 2026-07-13 PLAN-0B Codex運用文書・Skill完成と初回文書review

### Status

COMMITTED

### Workspace context

GitHub connector上で実施。local workspace path未確定。

### Repository state

| repository | branch | base SHA | current SHA | access | dirty |
|---|---|---|---|---|---|
| `kirohy/auto_stabilizer2` | `wbms-external-teleop-plan` | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | PLAN-0B文書・Skill群 | WRITE | local未確認 |
| `rtmros_msg_bridge` | 未作成 | `master`想定 | 未変更 | READ | 未確認 |
| `ik_solvers2` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |
| `prioritized_qp` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |
| `whole_body_teleop` | 未作成 | - | - | NONE | - |

### Goal

長期・複数repository・安全重要projectをCodexで小さく実装、review、引き継ぎ、commitできる初期運用文書とrepository-local Skillを用意し、初回計画のbranch/heartbeat曖昧点を修正する。

### Scope and out-of-scope

含む:

- package-level `AGENTS.md`。
- Plan、Implement、Review、Closeの4 Skill。
- OpenAI公式Codex情報への対応指針。
- branch基点とheartbeat条件のRevision 1。
- review、Progress、commit template。

含まない:

- control source実装。
- exact pre-M5 base調査。
- local build、simulation、実機確認。
- planning branch publish。

### Code investigation

- Codex instructionはproject rootからcurrent directoryへ`AGENTS.md` chainを構成する。
- Skillは`.agents/skills/<name>/SKILL.md`を用い、明示呼出しできる。
- dedicated reviewはworking treeを変更せずfindingを返す。
- long-running workにはclear outcome、constraints、definition of doneが必要。
- external未使用時にもheartbeat必須と読める初回仕様がlegacy-onlyと矛盾した。
- M4.2.2 review修正とM5が交錯する場合、単一pre-M5 commitが存在しない可能性がある。

### Decisions

採用:

- Work Unitをdistinct outcome単位とする。
- read-only plan、Contract承認、implementation、detached review、fresh review、closureの順。
- commitは明示許可時だけ。
- safety-critical/branch archaeologyは強いreasoning modelを使用する。
- schema固定後の通常実装と機械的変換を分ける。
- external sourceを選択した場合だけgenerator/bridge heartbeatを開始条件にする。
- legacy-onlyではexternal heartbeatを要求しない。
- 単一pre-M5 baseが無ければsynthetic baselineを構築する。

不採用:

- project全体を一つの巨大Goalで実装する。
- implementation task自身だけのreviewで完了する。
- finding修正後にincremental部分だけreviewする。
- same sourceへ複数taskのwrite accessを与える。
- current `wbms-dev` HEADから大量削除してbaseとする。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `auto_stabilizer/AGENTS.md` | package安全規約 | 恒久instruction |
| `auto_stabilizer2` | 4 Work Unit Skill | plan/implement/review/close分離 | 定型workflow |
| `auto_stabilizer2` | Codex Official Guidance | 公式情報対応 | UIと原則の分離 |
| `auto_stabilizer2` | Implementation Plan Revision 1 | branch/heartbeat修正 | 初回review finding |
| `auto_stabilizer2` | Operator Guide | 実用prompt | 人間向け運用 |

### Commands and results

| execution directory | command | result | evidence |
|---|---|---|---|
| GitHub connector | create AGENTS / 4 Skills / Official Guidance / Revision 1 / Operator Guide | PASS | commit SHA取得 |
| local | `git status --short` | NOT RUN | local checkoutなし |
| local | `git diff --check` | NOT RUN | local checkoutなし |
| local | Skill/AGENTS discovery | NOT RUN | M0-Bで確認 |
| local | `catkin build auto_stabilizer --no-deps` | NOT RUN | control source未変更 |

### Simulation / log evidence

なし。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| 2 | 文書自己review | external未使用でもheartbeat必須 | Revision 1でexternal source選択時だけ必須化 |
| 2 | 文書自己review | 単一pre-M5 base暗黙仮定 | synthetic baseline手順追加 |
| 2 | 公式情報照合 | 特定UI名への依存 | read-only planning、Goal、Reviewへ整理 |
| 2 | Skill review | Plan Skillしかない | Implement、Review、Close追加 |
| 2 | instruction hierarchy | Revisionが優先されない | AGENTS参照順修正 |

formal detached reviewは未実施。

### Acceptance

| criterion | result | evidence |
|---|---|---|
| Implementation Plan + Revision 1 | PASS | 文書存在 |
| M0-M12 Milestone | PASS | Implementation Plan |
| Work Unit workflow | PASS | Workflow |
| package safety instruction | PASS | AGENTS |
| 4 Skill | PASS | `.agents/skills` |
| review重点と非finding指定 | PASS | Workflow/Review Skill |
| commit checklist | PASS | Workflow/Close Skill |
| control source未変更 | PASS | 文書のみ |
| local `git diff --check` | UNVERIFIED | local checkoutなし |
| Skill/AGENTS discovery | UNVERIFIED | M0-B |
| formal detached document review | UNVERIFIED | planning review待ち |

### Unverified

- exact pre-M5/synthetic baseline。
- local diff check。
- Skill/AGENTS discovery。
- formal detached review。
- dependency branch/HEAD。

### Open issues

- M0-Aで履歴・コード・正式文書を照合する。
- active AGENTS chainとSkillをlocal確認する。
- planning文書の実装branch移植単位を決定する。

### Compatible dependency set

| repository | SHA |
|---|---|
| `auto_stabilizer2` planning source base | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` |
| `rtmros_msg_bridge` | 未記録 |
| `ik_solvers2` | 未記録 |
| `prioritized_qp` | 未記録 |
| `whole_body_teleop` | 未作成 |

### Central Progress sync

本repository内で完了。

### Next entry point

PLAN-0C: 複数repository運用仕様の正式化。

### Commit

| repository | SHA | subject |
|---|---|---|
| `auto_stabilizer2` | `eaa45c961f5b15e1fdcedf9ac37fdb9e4880cd7d` | `Add package instructions for external teleoperation work` |
| `auto_stabilizer2` | `3413fc82c5de5488024212da26577e747a8121d0` | `Add WBMS work unit planning skill` |
| `auto_stabilizer2` | `a39ed803f2baeda9b1db3326fabd6a8f0442e9f9` | `Add WBMS work unit implementation skill` |
| `auto_stabilizer2` | `ce106f15e3547eb9ee7e5fae6b3a02a650ea48fe` | `Add WBMS work unit review skill` |
| `auto_stabilizer2` | `9b345225584ce672445f6dea9bfdee7ffef65c04` | `Add WBMS work unit closure skill` |
| `auto_stabilizer2` | `5bb65793c603d06a9c0b481ad4adc8173ddc43de` | `Add official Codex guidance for WBMS workflow` |
| `auto_stabilizer2` | `2458371b1c6adff4f6f5d358ecfd8eabfc19b531` | `Clarify external teleoperation implementation plan` |
| `auto_stabilizer2` | `1491662239a2aedbd482e46afbd193e9674a468e` | `Prioritize external teleoperation plan revision` |
| `auto_stabilizer2` | `6fb17c38a4c7c37a208514260b4580f1957d85ee` | `Align WBMS planning skill with revised plan` |
| `auto_stabilizer2` | `0331f23b385b32e44d49966e54b162ac7856e1bd` | `Add Codex operator guide for external teleoperation` |

---

## 2026-07-13 PLAN-0C 複数repository運用仕様策定

### Status

COMMITTED

### Workspace context

```text
CATKIN_WORKSPACE = catkin_ws/<workspace_name> の絶対パス
CATKIN_SOURCE_ROOT = CATKIN_WORKSPACE/src
```

実際のlocal absolute pathはM0-Bで記録する。

### Repository state

| repository | branch | base SHA | current SHA | access | dirty |
|---|---|---|---|---|---|
| `kirohy/auto_stabilizer2` | `wbms-external-teleop-plan` | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | 本entry直前HEAD `a630e4c9cc8b0674ff5a78103a6c5945eb94a8ca` | WRITE | GitHub connector、local未確認 |
| `whole_body_teleop` | 未作成 | - | - | NONE | - |
| `kirohy/rtmros_msg_bridge` | 未作成 | `master`想定 | 未変更 | READ | 未確認 |
| `kirohy/ik_solvers2` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |
| `kirohy/prioritized_qp` | `teleop-dev`想定 | 未記録 | 未変更 | READ | 未確認 |

### Goal

複数repositoryにまたがる本プロジェクトについて、Codex起動directory、AGENTS三層構成、共通Skill、one-write-repository rule、Parent/Sub-unit、package build、中央Progress同期、worktree、bootstrapを正式化する。

### Scope and out-of-scope

含む:

- Revision 2。
- MultiRepositoryOperations。
- package `AGENTS.md`更新。
- 4 Work Unit Skill更新。
- source-root/repository AGENTS template。
- Project Context template。
- workspace manifest template。
- multi-repository Operator Guide Revision 1。
- 中央Progress再構成とPLAN-0C記録。

含まない:

- 他repositoryへの実配置。
- `whole_body_teleop` repository作成。
- bootstrap script実装。
- source code実装。
- local build、simulation、実機確認。

### Code investigation

- 実際のworkspace layoutは`catkin_ws/<workspace_name>/src`であり、`catkin_ws/src`固定ではない。
- 複数repositoryをsource rootから見渡す必要がある。
- 子repositoryの`AGENTS.md`がsource-root taskへ自動適用されるとは仮定できないため、明示readが必要。
- repository-local Skillだけでは他repository rootから利用できないため、共通Skillのuser-level symlinkが必要。
- repository実装とcross-repository planning/reviewを分ける必要がある。
- package buildは通常`catkin build <package> --no-deps`で十分であり、workspace一括buildは不要。

### Decisions

採用:

- `${CATKIN_WORKSPACE}`と`${CATKIN_SOURCE_ROOT}`を正式変数とする。
- cross-repository taskは`${CATKIN_SOURCE_ROOT}`から原則read-only。
- implementation/repository review/commitは対象repository rootから行う。
- 一つのimplementation taskがWRITEするrepositoryは原則一つ。
- 複数repository機能はParent Work Unitとrepository sub-unitへ分割。
- source-root、repository-root、package/moduleの三層`AGENTS.md`。
- 共通4 Skillの正本は`auto_stabilizer2/.agents/skills`、user-levelへsymlink。
- 各repositoryへProject Contextを置き、中央計画全文は複製しない。
- 他repository commit後、依存sub-unit前に中央ProgressへSHAを同期。
- workspace一括buildを標準にしない。
- 通常は対象packageの`--no-deps` build。
- dependency確認時だけ`--no-deps`を外す。
- `catkin build`の実行directoryは固定しない。
- source root内へ同一packageの複数worktreeを置かない。

不採用:

- 全repositoryを一つのimplementation taskで変更する。
- 共通Skillを各repositoryへcopyする。
- `catkin_ws/src`固定表記。
- workspace一括buildをacceptanceへ入れる。
- integration build failure時にcross-repo task内で複数repositoryを修正する。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md` | 複数repo修正を追加 | 最新計画 |
| `auto_stabilizer2` | `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md` | 正式運用仕様を追加・修正 | 起動、AGENTS、Skill、build、Progress |
| `auto_stabilizer2` | `WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md` | 実操作promptを追加 | 人間向け |
| `auto_stabilizer2` | `auto_stabilizer/AGENTS.md` | Revision 2とmulti-repo規約 | package instruction |
| `auto_stabilizer2` | 4 Work Unit Skill | workspace/access/Parent/Sub-unit/buildを追加 | 全repo共通workflow |
| `auto_stabilizer2` | `tools/codex_workspace/templates/*` | source-root/repository/Project Context/manifest template | M0-B bootstrap |
| `auto_stabilizer2` | 本Progress | PLAN-0Cと参照順を記録 | 中央引き継ぎ |

### Commands and results

| execution directory | command | result | evidence |
|---|---|---|---|
| GitHub connector | create/update Revision 2、MultiRepositoryOperations、Operator Guide Revision 1、AGENTS、Skills、templates | PASS | commit SHA取得 |
| GitHub connector | compare `wbms-dev...wbms-external-teleop-plan` | PASS | 文書・AGENTS・Skill・templateのみ |
| local | `git status --short` | NOT RUN | local checkoutなし |
| local | `git diff --check` | NOT RUN | local checkoutなし |
| local | Skill/AGENTS discovery | NOT RUN | M0-B |
| local | package-specific baseline build | NOT RUN | M0-B |

### Simulation / log evidence

なし。control source未変更。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| 3 | 仕様対話 | 新ROS nodeだけでなく全repositoryのinstruction/Skill配置が未定義 | MultiRepositoryOperationsとRevision 2で定義 |
| 3 | 仕様対話 | Codex起動directoryが不明 | source rootとrepository rootへ分離 |
| 3 | 仕様対話 | workspace layoutを`catkin_ws/src`と誤認 | `${CATKIN_WORKSPACE}=catkin_ws/<workspace_name>`へ修正 |
| 3 | 仕様対話 | workspace一括buildを想定 | package-specific buildへ修正 |
| 3 | 自己review | common Skillのcopy drift | user-level symlinkを採用 |
| 3 | 自己review | 他repo commitと中央Progressの順序 | central Progress sync sub-unitを追加 |

formal detached reviewは未実施。

### Acceptance

| criterion | result | evidence |
|---|---|---|
| workspace pathが実配置を表す | PASS | Revision 2 / MultiRepositoryOperations |
| task別起動directory | PASS | source root / repository root |
| one-write-repository rule | PASS | formal spec、Skills |
| Parent/Sub-unit | PASS | formal spec、Plan Skill |
| AGENTS三層構成 | PASS | formal spec、templates |
| 共通Skill配布 | PASS | user-level symlink仕様 |
| repository Project Context | PASS | template |
| package-specific build | PASS | `catkin build <package> --no-deps` |
| dependency build | PASS | 必要時だけ`--no-deps`除外 |
| central Progress sync | PASS | MultiRepositoryOperations / Close Skill |
| source code未変更 | PASS | planning assetsのみ |
| local diff check | UNVERIFIED | local checkoutなし |
| formal detached review | UNVERIFIED | local task待ち |
| Skill/AGENTS認識 | UNVERIFIED | M0-B |

### Unverified

- `${CATKIN_WORKSPACE}`の実際のabsolute path。
- source-root `AGENTS.md`の配置。
- user-level Skill symlink。
- 各repository rootから4 Skillが認識されること。
- repository Project Contextの実配置。
- `whole_body_teleop` repository作成。
- `rtmros_msg_bridge`等へのAGENTS配置。
- package-specific baseline build。
- formal detached document review。

### Open issues

- planning branchをlocalへ取得し`git diff --check`。
- planning文書群のformal detached review。
- M0-A branch archaeology。
- M0-Bでworkspace/repository bootstrap。
- bootstrap/verify scriptは別Work Unitで実装。

### Compatible dependency set

| repository | SHA |
|---|---|
| `auto_stabilizer2` planning source base | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` |
| `auto_stabilizer2` planning docs before this entry | `a630e4c9cc8b0674ff5a78103a6c5945eb94a8ca` plus PLAN-0C commits |
| `whole_body_teleop` | 未作成 |
| `rtmros_msg_bridge` | 未記録 |
| `ik_solvers2` | 未記録 |
| `prioritized_qp` | 未記録 |

### Central Progress sync

本entryで完了。

### Next entry point

1. planning branchをlocalへ取得。
2. `git diff --check`。
3. active AGENTS chainと4 Skillの認識確認。
4. planning文書群のformal detached review。
5. P0/P1/P2解消。
6. `${CATKIN_SOURCE_ROOT}`からM0-A Parent planning。

開始prompt:

```text
$wbms-plan-work-unit を使用してください。

Work Unit: M0-A branch archaeology and exact baseline selection
Task type: cross-repository Parent Work Unit
CATKIN_WORKSPACE: <absolute path to catkin_ws/<workspace_name>>
CATKIN_SOURCE_ROOT: <CATKIN_WORKSPACE>/src
Codex launch directory: <CATKIN_SOURCE_ROOT>

Revision 2、MultiRepositoryOperations、Revision 1、Implementation Plan、最新Progress、walking preparation正式文書、旧Progress後方、git history、対象sourceをread-onlyで調査してください。

source、branch、index、working treeを変更せず、Parent Contractだけを作成してください。
```

### Commit

本entryを追加したcommit SHAは次entryで記録する。

PLAN-0Cで作成・更新した主要commit:

| repository | SHA | subject |
|---|---|---|
| `auto_stabilizer2` | `a434d81ab9dc1d3ec731a15a6d149517b91e6540` | `Add multi-repository operations specification` |
| `auto_stabilizer2` | `6beaca0e50f1c8baf4218c5f2e5088174a519a27` | `Define multi-repository implementation operations` |
| `auto_stabilizer2` | `e8f68a8892905233113bfd445a593b3ad44d734d` | `Correct and consolidate multi-repository operations` |
| `auto_stabilizer2` | `a630e4c9cc8b0674ff5a78103a6c5945eb94a8ca` | `Reference multi-repository operator guide` |
| `auto_stabilizer2` | `6f3b4dc3b8d86c3f9cdb2cbe54c513cb6e37752a` | `Extend WBMS planning skill for multiple repositories` |
| `auto_stabilizer2` | `33ad35d123ec9c02a057ff37cde24eb745a6b602` | `Restrict WBMS implementation skill to one repository` |
| `auto_stabilizer2` | `38c4d38ccc206844ac58fb5606b3c9beb17d606d` | `Extend WBMS review skill across repositories` |
| `auto_stabilizer2` | `f104b9011f5fc620d044b82212b94f4a2eb721a5` | `Extend WBMS closure skill for repository sub-units` |
| `auto_stabilizer2` | `ff2ba7c06151c27eb4e71bbbe196189557d29523` | `Add catkin source root AGENTS template` |
| `auto_stabilizer2` | `2ffc9a7952a8f7a02a2b65df1de5a7db92b7f01c` | `Add whole body teleop AGENTS template` |
| `auto_stabilizer2` | `1b287e49b83c4bc159997ba371cc44dd873058ef` | `Add RTM ROS bridge AGENTS template` |
| `auto_stabilizer2` | `1848001530a9d451ec87596a51ebf7bc5e5d4ecd` | `Add IK solver AGENTS template` |
| `auto_stabilizer2` | `fd383cbb76c5bcb93bc41a471ca4440d5db7a517` | `Add prioritized QP AGENTS template` |
| `auto_stabilizer2` | `422821cc2b900cd18a9490ce6ca0637d0e0823e4` | `Add repository project context template` |
| `auto_stabilizer2` | `ac96f923176110af9ad5e96f4ca65ec478efd576` | `Add workspace manifest template` |
| `auto_stabilizer2` | `e9ea7dd1fb11ade46265d1718c36f1893711ae27` | `Document Codex workspace bootstrap assets` |
| `auto_stabilizer2` | `e7795f7b2d05d6d131c7cc230ffa408a3d3e4266` | `Add multi-repository Codex operator guide revision` |

---

## 2026-07-16 M0-A5 central Progress sync

### Status

- M0-A-R1 repository commit: `COMMITTED`。
- M0-A5 central Progress sync: `IMPLEMENTED`、fresh read-only review前。
- M0全体: `IN_PROGRESS`。M0-A5のfresh review、人間によるexact diffとcommitの明示承認、commitが完了するまでは完了扱いにしない。

### Workspace context

- CATKIN_WORKSPACE: `/home/kirohy/catkin_ws/cnoid2`
- CATKIN_SOURCE_ROOT: `/home/kirohy/catkin_ws/cnoid2/src`
- Codex launch directory: `/home/kirohy/catkin_ws/cnoid2/src/auto_stabilizer2`

### Repository state

| repository | branch | base SHA | current SHA | access | dirty |
|---|---|---|---|---|---|
| `auto_stabilizer2` | `wbms-external-teleop-plan` | `15369f77665311381e27eef464edbfc69660b8a4` | `15369f77665311381e27eef464edbfc69660b8a4` | WRITE | 既存untrackedあり、保持 |
| `ik_solvers2` | observed `teleop-dev` | READ観測のみ | `47576209a01a35177ac0d594e586abfa90927dd7` | READ | 既存untrackedあり、保持 |
| `prioritized_qp` | observed `teleop-dev` | READ観測のみ | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | READ | clean |
| `rtmros_msg_bridge` | `jaxon-minimal` | READ観測のみ | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | READ | 既存untrackedあり、保持 |
| `whole_body_teleop` | 未作成 | — | — | NONE | — |

既存のdirty/untracked状態は全repositoryで保持した。reset、stash、checkout、clean、削除、移動、上書き、stageは行っていない。

### Goal

M0-A-R1で人間承認されcommit済みとなったbaseline compatible set、exact commit evidence、未検証状態、M0-B1へのgateを、中央Progressへappend-onlyで同期する。

### Scope and out-of-scope

含む:

- 本entryのappend-only追加。
- approved compatible set `M0-pre-M5-baseline`の記録。
- M0-A-R1 commitとfresh review evidenceの記録。
- M0-A5 review、承認、commit gateとnext entry pointの記録。

含まない:

- 既存Progress entryの修正、並べ替え、削除。
- M0-A-R1 Contract/input、M0-A、Revision 2、MultiRepositoryOperations、manifest、AGENTS、4 Work Unit Skill、control sourceの変更。
- sibling repositoryの変更。
- stage、commit、push、merge、PR作成、branch/worktree/dedicated workspace作成。
- build、simulation、実機実行。
- M0-B1以降の開始。

### Approved compatible dependency set

name: `M0-pre-M5-baseline`

| repository | SHA | status |
|---|---|---|
| `auto_stabilizer2` | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | — | NOT APPLICABLE、repository未作成 |

current dirty workspaceおよびcurrent development branchのHEADは、このbaseline sourceまたはbuild sourceに使用しない。

### M0-A-R1 completion evidence

- repository: `auto_stabilizer2`
- commit SHA: `15369f77665311381e27eef464edbfc69660b8a4`
- subject: `Correct M0 baseline planning assets`
- parent SHA: `6e530edacecf663f2eedbdcfd5a787468dad70ce`
- committed paths、exact 4 files:
  - `auto_stabilizer/docs/M0-A.md`
  - `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
  - `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
  - `tools/codex_workspace/templates/WBMSExternalTeleopWorkspaceManifest.template.yaml`
- diff stat: `4 files changed, 751 insertions(+), 35 deletions(-)`
- latest fresh cross-repository read-only review: P0/P1/P2 findingなし、P3 findingなし。
- `git diff --check`: PASS。
- build、simulation、実機: `UNVERIFIED`。

### Decisions

- CHEST相対腕拘束とM4.2.2の必要な安全修正は最初の旧M5 commitより前の直列履歴に含まれるため、synthetic baselineは不要とする。
- M0-A3はexecutable sub-unitとして廃止する。build commandの計画だけをM0-Aへ残す。
- 全package-specific baseline buildのsole ownerはM0-B7とする。
- M0-A5はcentral Progress syncだけを所有する。
- current dirty workspaceをbaseline sourceまたはbuild sourceに使用しない。
- M0-B1で隔離方式、exact path、source-root外worktree、dedicated catkin workspace、underlay、同名package重複回避、current generated artifact非混入を人間承認する。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | 本M0-A5 entryを末尾へappend-only追加 | approved baselineとM0-A-R1 SHAの中央同期 |

### Verification / Unverified

| item | result | evidence |
|---|---|---|
| package build | UNVERIFIED | M0-B7だけが実行owner |
| dedicated baseline workspace | UNVERIFIED | M0-B1の人間承認前 |
| planning asset migration | UNVERIFIED | M0-B2未開始 |
| simulation | UNVERIFIED | 今回未実行 |
| 実機 | UNVERIFIED | 今回未実行 |

historical logはbaseline選定根拠としてのみ扱い、今回のruntime verificationとして扱わない。未実行項目をPASS表記しない。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| pending | M0-A5最新diff全体のfresh read-only review | PENDING | review後に記録する |

### Central Progress sync

- M0-A-R1 exact SHAとapproved `M0-pre-M5-baseline`の記録: 本entryで実装。
- M0-A5 fresh review: PENDING。
- 人間によるM0-A5 exact diffとcommitの明示承認: PENDING。
- M0-A5 commit: PENDING。
- expected M0-A5 commit subject: `Record approved M0 baseline compatible set`
- 上記3 gateの完了前にM0-AおよびM0全体を完了扱いにしない。

### Next entry point

1. 最新M0-A5 diff全体をfresh read-only reviewする。
2. P0/P1/P2 findingを解消し、修正後は最新diff全体を再reviewする。
3. 人間がM0-A5のexact diffとcommitを明示承認する。
4. expected subject `Record approved M0 baseline compatible set`でM0-A5をcommitする。
5. M0-A5 commit完了後だけM0-Aを完了扱いにする。
6. M0-A5のreview、承認、commit完了前にM0-B1を開始しない。

### Commit

- M0-A5 commit SHA: PENDING。
- expected subject: `Record approved M0 baseline compatible set`

---

## 2026-07-17 M0-B1 isolated workspace closure

### Status

- M0-A5 source commit: `c6a1084edea1f6e916fa699adb01e4641149bf57`。
- M0-A: `COMPLETE`。
- M0-B1の隔離方式とclosure: 人間承認済み。
- 本durable record: `IMPLEMENTED`、fresh read-only review、人間によるexact diffとcommitの明示承認、commitは`PENDING`。

### Workspace context

- CATKIN_WORKSPACE: `/home/kirohy/catkin_ws/teleop_ws`
- CATKIN_SOURCE_ROOT: `/home/kirohy/catkin_ws/teleop_ws/src`
- isolated `auto_stabilizer2` checkout: `/home/kirohy/catkin_ws/teleop_ws/src/auto_stabilizer2`
- Codex launch directory: `/home/kirohy/catkin_ws/teleop_ws/src/auto_stabilizer2`

### Repository state at record start

| repository | branch | current SHA | access | dirty |
|---|---|---|---|---|
| `auto_stabilizer2` | `wbms-external-teleop` | `2497f521c17b522d09f190faf85b3d94fd742f0f` | WRITE、中央Progressだけ | clean |
| `ik_solvers2` | `teleop-dev` | `47576209a01a35177ac0d594e586abfa90927dd7` | READ | clean |
| `prioritized_qp` | `teleop-dev` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | READ | clean |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | READ | clean |
| `whole_body_teleop` | — | — | NONE | 未参照 |

### Goal

M0-B1で人間承認された隔離方式、exact path、environment、generated space非混入条件、およびclosure evidenceを中央Progressへappend-onlyで固定する。

### Scope and out-of-scope

含む:

- M0-B1の人間承認済みclosure evidence。
- isolated checkout、workspace、underlay setup、package重複回避の記録。
- default spaceとM0-B7専用spaceの所有権分離。
- 未検証項目と後続gateの記録。

含まない:

- 過去Progress entryの修正、削除、並べ替え。
- M0-B2の移植・review evidence。
- control source、IDL、CMake、package metadata、AGENTS、Skill、template、Project Context、workspace manifestの変更。
- sibling repositoryの変更。
- generated spaceの作成、削除、整理。
- build、simulation、実機実行。
- branch切替、worktree操作、reset、stash、clean、stage、commit、push、merge、PR作成。

### Approved isolation and closure evidence

- historical dirty checkout `/home/kirohy/catkin_ws/cnoid2/src/auto_stabilizer2`はbaseline sourceまたはbuild sourceに使用しない。本Work Unitでも参照元・実行元・変更先に使用せず、変更していない。
- `git worktree list --porcelain`で確認できる`auto_stabilizer2` worktreeは、isolated checkout `/home/kirohy/catkin_ws/teleop_ws/src/auto_stabilizer2`の1件だけである。additional worktreeはない。
- 承認済みworkspace layoutに同名package重複はない。本記録直前の全READ repositoryの`package.xml` name照合でも重複はない。
- source-root内へ同一packageを持つ別worktreeを置かない。
- historical checkoutのgenerated artifact、cache、compile database、logを本workspaceのbaseline evidenceへ混入させない。

### Approved environment

M0-B7はfresh shellで次のexact sequenceを使用する。

```zsh
source /opt/ros/noetic/setup.zsh
source /home/kirohy/catkin_ws/teleop_ws/devel/setup.zsh
cd /home/kirohy/catkin_ws/teleop_ws
```

- `/opt/ros/noetic`をunderlayとし、`teleop_ws/devel`を承認済みprebuilt environmentとして重ねる。
- 現在のCodex processが継承したenvironmentは、M0-B7のfresh build evidenceに使用しない。
- `.catkin_tools/profiles/default/config.yaml`では`install: false`である。

### Generated space ownership

既存default space:

- `/home/kirohy/catkin_ws/teleop_ws/build`
- `/home/kirohy/catkin_ws/teleop_ws/devel`
- `/home/kirohy/catkin_ws/teleop_ws/logs`

これらはfeasibility buildまたはprebuilt environmentであり、fresh M0-B7 evidenceとして使用しない。

M0-B7専用space:

- `/home/kirohy/catkin_ws/teleop_ws/build_m0_baseline`
- `/home/kirohy/catkin_ws/teleop_ws/devel_m0_baseline`
- `/home/kirohy/catkin_ws/teleop_ws/logs_m0_baseline`
- install disabled

上記専用spaceは本記録時点で未作成である。M0-B7だけが作成とpackage-specific baseline buildを所有し、本Work Unitでは作成しない。

### Deferred ownership

- source-root `AGENTS.md`配置とcommon Skill installation/recognition: M0-B5へdefer。
- Project Contextの配置または正式defer状態とworkspace manifest: M0-B6へdefer。
- selected compatible setのmaterializationとpackage-specific baseline build: M0-B7だけが所有。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | 本M0-B1 entryをfile末尾へappend-only追加 | 人間承認済み隔離方式とclosure evidenceのdurable record |

### Commands and results

| execution directory | command | result | evidence |
|---|---|---|---|
| 各repository root | `git status --short` | PASS | 4 repositoryとも出力なし |
| 各repository root | `git branch --show-current` | PASS | 上記repository stateと一致 |
| 各repository root | `git rev-parse HEAD` | PASS | 上記repository stateと一致 |
| `auto_stabilizer2` root | `git worktree list --porcelain` | PASS | current checkout 1件だけ |
| READ repository群 | `package.xml`の`<name>`重複照合 | PASS | duplicate nameなし |
| `/home/kirohy/catkin_ws/teleop_ws` | default/M0-B7専用space存在確認 | PASS | default 3 spaceは存在、専用3 spaceは未作成 |
| `/home/kirohy/catkin_ws/teleop_ws` | catkin profileと`devel/_setup_util.py`のread-only確認 | PASS | install disabled、承認済みprefixは`teleop_ws/devel`と`/opt/ros/noetic` |
| — | package build | NOT RUN | M0-B7へdefer |
| — | simulation | NOT RUN | `UNVERIFIED` |
| — | hardware | NOT RUN | `UNVERIFIED` |

### Verification / Unverified

| item | result | evidence |
|---|---|---|
| M0-B1 isolation/closure human approval | PASS | 承認済みContractと本実装指示 |
| package build | UNVERIFIED | 今回未実行、M0-B7だけが実行owner |
| selected compatible set materialization | UNVERIFIED | M0-B7へdefer |
| simulation | UNVERIFIED | 今回未実行 |
| hardware | UNVERIFIED | 今回未実行 |
| source-root AGENTS/common Skill installation | UNVERIFIED | M0-B5へdefer |
| Project Context/workspace manifest | UNVERIFIED | M0-B6へdefer |

未実行のbuild、simulation、hardwareをPASS扱いしない。既存default spaceとhistorical artifactもfresh verification evidenceとして扱わない。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| pending | 本M0-B1最新diff全体のfresh repository review | PENDING | review後に記録する |

### Central Progress sync and gates

- M0-B1 isolation/closure evidenceのappend-only記録: 本entryで実装。
- M0-B1 fresh review: PENDING。
- 人間によるM0-B1 exact diffとcommitの明示承認: PENDING。
- M0-B1 record commit: PENDING。
- expected commit subject: `Record M0-B1 isolated workspace closure`
- M0-B2 durable recordは別Work Unit・別commitとする。本entryまたは同じcommitへ混在させない。
- M0-B1 record commit完了前にM0-B2 durable recordへ進まない。
- M0-B2 closure完了前にM0-B3へ進まない。

### Next entry point

1. 本M0-B1最新diff全体をfresh read-only repository reviewする。
2. P0/P1/P2 findingを解消し、修正後は最新diff全体を再reviewする。
3. 人間がM0-B1のexact diffとcommitを明示承認する。
4. expected subject `Record M0-B1 isolated workspace closure`でM0-B1 recordをcommitする。
5. M0-B1 record commit後だけ、M0-B2 durable recordを別Work Unit・別commitとして開始する。
6. M0-B2 closure完了前にM0-B3を開始しない。

### Commit

- expected subject: `Record M0-B1 isolated workspace closure`
- commit SHA: PENDING

---

## 2026-07-17 M0-B2-record M0-B2 isolated branch materialization durable record

### Status

- M0-B2 isolated branch materialization履歴のread-only verification: `PASS`。
- 本durable record: `IMPLEMENTED`、fresh read-only review前。
- M0-B2全体: `IN_PROGRESS`。fresh review、人間によるexact diffとcommitの明示承認、commitは`PENDING`。
- M0-B3: `BLOCKED`。M0-B2 closure完了前に開始しない。

### Workspace context

- CATKIN_WORKSPACE: `/home/kirohy/catkin_ws/teleop_ws`
- CATKIN_SOURCE_ROOT: `/home/kirohy/catkin_ws/teleop_ws/src`
- Codex launch directory: `/home/kirohy/catkin_ws/teleop_ws/src/auto_stabilizer2`

### Repository state at record start

| repository | branch | current SHA | access | dirty |
|---|---|---|---|---|
| `auto_stabilizer2` | `wbms-external-teleop` | `91c0f23d2596c6cec4501b9a0f551974098e4f7a` | WRITE、中央Progressだけ | clean |
| `ik_solvers2` | `teleop-dev` | `47576209a01a35177ac0d594e586abfa90927dd7` | READ | clean |
| `prioritized_qp` | `teleop-dev` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | READ | clean |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | READ | clean |
| `whole_body_teleop` | — | — | NONE | 未参照 |

### Goal

`c06b63c8e12dbf85bda4c8391a37544c2469731c`をcontrol baselineとするisolated `auto_stabilizer2` implementation branchについて、planning asset rangeとM0-A-R1/M0-A5の個別移植、commit順序、内容同等性、非制御差分、未検証状態を中央Progressへappend-onlyで固定する。

### Scope and out-of-scope

含む:

- 既にmaterialize済みのbranch履歴に対するread-only verification evidence。
- planning assetのexact source/destination rangeと個別commit mapping。
- changed-path、`range-diff`、stable patch-id、parent関係の記録。
- control source、IDL、CMake、package metadataの非変更証拠。
- sibling repository非変更、未検証項目、review/commit/M0-B3 gateの記録。

含まない:

- branch、worktree、repository、dedicated workspaceの作成または再materialization。
- cherry-pick、checkout、reset、rebase、merge、stash、clean。
- 過去Progress entryの修正、削除、並べ替え。
- 中央Progress以外の文書、AGENTS、Skill、template、manifest、Project Contextの変更。
- control source、IDL、CMake、package metadata、sibling repositoryの変更。
- stage、commit、push、merge、PR作成。
- package build、simulation、hardware execution。
- M0-B3以降の開始。

### Baseline and materialization mapping

control baseline:

```text
c06b63c8e12dbf85bda4c8391a37544c2469731c
```

Planning asset range:

| role | source planning branch | destination implementation branch |
|---|---|---|
| range boundary/base | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | `c06b63c8e12dbf85bda4c8391a37544c2469731c` |
| first commit | `8261114e9fe7bcd61fde14e12a6f5eda50b767d4` | `61a75b9f52aba6b04b12d4892309999c2ab363da` |
| last commit | `6e530edacecf663f2eedbdcfd5a787468dad70ce` | `5f6fa92a1e6eec1d6e79f8278052dfbeb4d7d48c` |

```text
source exact range:
  5c21cc0cb3c6ef6c906642836ddadf279c8266fd..6e530edacecf663f2eedbdcfd5a787468dad70ce

source inclusive range:
  8261114e9fe7bcd61fde14e12a6f5eda50b767d4^..6e530edacecf663f2eedbdcfd5a787468dad70ce

destination exact range:
  c06b63c8e12dbf85bda4c8391a37544c2469731c..5f6fa92a1e6eec1d6e79f8278052dfbeb4d7d48c

destination inclusive range:
  61a75b9f52aba6b04b12d4892309999c2ab363da^..5f6fa92a1e6eec1d6e79f8278052dfbeb4d7d48c
```

個別commit mapping:

| role | source commit | destination commit | verification |
|---|---|---|---|
| M0-A-R1 document correction | `15369f77665311381e27eef464edbfc69660b8a4` | `ad7aa5bfc8a58f160b2c4c3bf89f506c15d51f7a` | `range-diff =`、stable patch-id一致 |
| M0-A5 central Progress sync | `c6a1084edea1f6e916fa699adb01e4641149bf57` | `2497f521c17b522d09f190faf85b3d94fd742f0f` | `range-diff =`、stable patch-id一致 |

stable patch-id:

| mapping | patch-id |
|---|---|
| M0-A-R1 source/destination | `5bb6fc827acca30bca7ef9b5b54d98fab329b052` |
| M0-A5 source/destination | `93834743a6f4a1812ca57045e0f1c4cf30510b7d` |

### Commit order and parent evidence

```text
c06b63c8e12dbf85bda4c8391a37544c2469731c
  -> 61a75b9f52aba6b04b12d4892309999c2ab363da
  -> planning asset destination range、41 commits
  -> 5f6fa92a1e6eec1d6e79f8278052dfbeb4d7d48c
  -> ad7aa5bfc8a58f160b2c4c3bf89f506c15d51f7a
  -> 2497f521c17b522d09f190faf85b3d94fd742f0f
  -> 91c0f23d2596c6cec4501b9a0f551974098e4f7a
  -> expected M0-B2 durable record commit
```

- source planning range: 41 commits。
- destination planning range: 41 commits。
- baselineからM0-B1 record commitまで: 44 commits、merge commitなし。
- destination first commit `61a75b9...`のparentはbaseline `c06b63c...`。
- M0-A-R1 destination `ad7aa5b...`のparentはplanning range末尾`5f6fa92...`。
- M0-A5 destination `2497f52...`のparentは`ad7aa5b...`。
- M0-B1 durable record `91c0f23...`のparentは`2497f52...`。
- 本M0-B2 durable recordは`91c0f23...`の子commitとして別commitにする。

materialized planning commitsがM0-B1 record commitのancestorにあるのは、isolated destination branch上でM0-B1のdurable recordを作成したためである。formal gateとしては、M0-B1 record commit完了後に本M0-B2 durable recordを作成し、そのclosure完了後だけM0-B3へ進む。

### Content-equivalence evidence

- planning source/destination rangeの`git range-diff`: 41/41 commitsすべて`=`。
- M0-A-R1/M0-A5の追加`git range-diff`: 2/2 commitsすべて`=`。
- M0-A-R1のsource/destination stable patch-id: 双方`5bb6fc827acca30bca7ef9b5b54d98fab329b052`。
- M0-A5のsource/destination stable patch-id: 双方`93834743a6f4a1812ca57045e0f1c4cf30510b7d`。
- conflict resolutionまたは内容変更を示す`range-diff`差分はない。

### Changed-path evidence

Planning destination rangeのnet changed-pathは、次のplanning/instruction/Skill/template assetだけである。

```text
.agents/skills/wbms-close-work-unit/SKILL.md
.agents/skills/wbms-implement-work-unit/SKILL.md
.agents/skills/wbms-plan-work-unit/SKILL.md
.agents/skills/wbms-review-work-unit/SKILL.md
auto_stabilizer/AGENTS.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md
auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md
tools/codex_workspace/README.md
tools/codex_workspace/templates/AGENTS.ik_solvers2.md
tools/codex_workspace/templates/AGENTS.prioritized_qp.md
tools/codex_workspace/templates/AGENTS.rtmros_msg_bridge.md
tools/codex_workspace/templates/AGENTS.source-root.md
tools/codex_workspace/templates/AGENTS.whole_body_teleop.md
tools/codex_workspace/templates/WBMSExternalTeleopProjectContext.template.md
tools/codex_workspace/templates/WBMSExternalTeleopWorkspaceManifest.template.yaml
```

M0-A-R1 destinationのexact changed paths:

```text
A auto_stabilizer/docs/M0-A.md
M auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md
M auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md
M tools/codex_workspace/templates/WBMSExternalTeleopWorkspaceManifest.template.yaml
```

M0-A5 destinationとM0-B1 durable recordは、それぞれ`auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`だけを変更している。

### Baseline non-control evidence

`c06b63c8e12dbf85bda4c8391a37544c2469731c..91c0f23d2596c6cec4501b9a0f551974098e4f7a`について、次のpathを指定した`git diff --name-only`は全て出力なしだった。

| category | checked paths | result |
|---|---|---|
| control source | `auto_stabilizer/rtc`、`auto_stabilizer/euslisp` | PASS、差分なし |
| RTM IDL | `auto_stabilizer/idl` | PASS、差分なし |
| CMake | 全`CMakeLists.txt`、`*.cmake` | PASS、差分なし |
| package metadata | 全`package.xml`、`manifest.xml` | PASS、差分なし |

したがって、現HEADのruntime/control treeはbaseline `c06b63c...`から変更されていない。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | 本M0-B2 entryをfile末尾へappend-only追加 | isolated branch materialization evidenceのdurable record |

### Commands and results

| execution directory | command | result | evidence |
|---|---|---|---|
| 各repository root | `git status --short` | PASS | record開始時、4 repositoryとも出力なし |
| 各repository root | `git branch --show-current` | PASS | repository state tableと一致 |
| 各repository root | `git rev-parse HEAD` | PASS | repository state tableと一致 |
| `auto_stabilizer2` root | planning source/destinationの`git range-diff --no-color` | PASS | 41/41 commitsすべて`=` |
| `auto_stabilizer2` root | M0-A-R1/M0-A5の`git range-diff --no-color` | PASS | 2/2 commitsすべて`=` |
| `auto_stabilizer2` root | M0-A-R1 source/destinationの`git patch-id --stable` | PASS | 双方`5bb6fc827acca30bca7ef9b5b54d98fab329b052` |
| `auto_stabilizer2` root | M0-A5 source/destinationの`git patch-id --stable` | PASS | 双方`93834743a6f4a1812ca57045e0f1c4cf30510b7d` |
| `auto_stabilizer2` root | source/destination planning rangeの`git rev-list --count` | PASS | 双方41 commits |
| `auto_stabilizer2` root | baselineからM0-B1 recordまでの`git rev-list --count` / `--merges` | PASS | 44 commits、merge commitなし |
| `auto_stabilizer2` root | planning rangeと個別commitのchanged-path確認 | PASS | planning assetおよび上記exact pathだけ |
| `auto_stabilizer2` root | baselineから現HEADのcontrol/IDL/CMake/package metadata差分確認 | PASS | 全category出力なし |
| 全対象repository root | implementation後の`git status --short`、`git branch --show-current`、`git rev-parse HEAD` | PASS | sibling 3 repositoryは開始時branch/HEAD/clean stateを維持。WRITE repositoryは指定Progress 1 fileだけmodified |
| — | package build | NOT RUN | M0-B7だけがowner、`UNVERIFIED` |
| — | simulation | NOT RUN | `UNVERIFIED` |
| — | hardware | NOT RUN | `UNVERIFIED` |

### Verification / Unverified

| item | result | evidence |
|---|---|---|
| isolated branch commit graph | PASS | baselineから44 commits、mergeなし、parent chain確認 |
| planning range content equivalence | PASS | `range-diff` 41/41 `=` |
| M0-A-R1 content equivalence | PASS | `range-diff =`、stable patch-id一致 |
| M0-A5 content equivalence | PASS | `range-diff =`、stable patch-id一致 |
| changed-path / baseline non-control inspection | PASS | 許可assetだけ、control/IDL/CMake/package metadata差分なし |
| sibling repository非変更 | PASS | record開始時とimplementation後のbranch/HEAD/clean stateが一致 |
| package build | UNVERIFIED | 今回未実行、M0-B7だけがowner |
| exact compatible set materialization/build | UNVERIFIED | M0-B7へdefer |
| simulation | UNVERIFIED | 今回未実行 |
| hardware | UNVERIFIED | 今回未実行 |
| source-root AGENTS/common Skill installation | UNVERIFIED | M0-B5へdefer |
| Project Context/workspace manifest | UNVERIFIED | M0-B6へdefer |

materialization履歴のPASSはcommit graph、patch同等性、changed-pathに対するread-only verificationである。package build、simulation、hardware verificationを代替しない。

### Compatible dependency set

name: `M0-pre-M5-baseline`

| repository | selected ref/SHA | status |
|---|---|---|
| `auto_stabilizer2` | historical `wbms-dev` ancestor `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED、control baseline preserved |
| `ik_solvers2` | `2.0` at `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED、exact materialization/buildはM0-B7で確認 |
| `prioritized_qp` | `master` at `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED、exact materialization/buildはM0-B7で確認 |
| `rtmros_msg_bridge` | `jaxon-minimal` at `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | — | NOT APPLICABLE、repository未作成 |

current `ik_solvers2/teleop-dev`および`prioritized_qp/teleop-dev`は観測中のdevelopment HEADであり、selected baseline SHAとして扱わない。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| pending | 本M0-B2最新diff全体のfresh read-only repository review | PENDING | implementation taskとは別taskでreviewする |

### Central Progress sync and gates

- M0-B2 materialization evidenceのappend-only durable record: 本entryで実装。
- M0-B2 fresh review: PENDING。
- 人間によるM0-B2 exact diffとcommitの明示承認: PENDING。
- M0-B2 record commit: PENDING。
- expected commit subject: `Record M0-B2 isolated branch materialization`
- 本M0-B2 recordはM0-B1 record commit `91c0f23d2596c6cec4501b9a0f551974098e4f7a`とは別commitにする。
- M0-B2 closure完了前にM0-B3を開始しない。

### Next entry point

1. 本M0-B2最新diff全体を別taskでfresh read-only repository reviewする。
2. P0/P1/P2 findingを解消し、修正後は最新diff全体を再reviewする。
3. 人間がM0-B2のexact diffとcommitを明示承認する。
4. expected subject `Record M0-B2 isolated branch materialization`でM0-B2 recordをcommitする。
5. M0-B2 record commit後にclosureを確認する。
6. M0-B2 closure完了後だけM0-B3を開始する。

### Commit

- repository: `auto_stabilizer2`
- expected parent: `91c0f23d2596c6cec4501b9a0f551974098e4f7a`
- expected subject: `Record M0-B2 isolated branch materialization`
- commit SHA: PENDING

## 2026-07-17 M0-B3-PROGRESS M0-B3 whole_body_teleop bootstrap central Progress sync

### Status

- M0-B2 record commit: COMMITTED as `e0c311ecb59e3d9ae9bea6b551c03ffd5d58efa3` (`Record M0-B2 isolated branch materialization`)。
- 直前のM0-B2 entryはcommit前snapshotであり、append-only履歴としてそのまま保存する。そこに残る`PENDING`を直接修正しない。
- M0-B3 `whole_body_teleop` repository/package bootstrap: COMPLETED and COMMITTED。
- M0-B3-PROGRESS implementation: 本entryをappend-only追加するcurrent task。
- M0-B3-PROGRESS fresh review: PENDING。
- M0-B3-PROGRESS human approval/commit: PENDING。
- M0-B4: BLOCKED。M0-B3-PROGRESSのfresh review、人間承認、commit完了前に開始しない。

### Workspace context

- `CATKIN_WORKSPACE=/home/kirohy/catkin_ws/teleop_ws`
- `CATKIN_SOURCE_ROOT=/home/kirohy/catkin_ws/teleop_ws/src`
- implementation launch directory: `/home/kirohy/catkin_ws/teleop_ws/src/auto_stabilizer2`
- WRITE repository: `auto_stabilizer2`だけ
- WRITE file: `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`だけ
- READ repositories: `whole_body_teleop`、`ik_solvers2`、`prioritized_qp`、`rtmros_msg_bridge`

### Goal

preceding repository sub-unit M0-B3のcommitted result、review、static verification、defer、repository setおよび次gateを、中央Progress正本へappend-onlyで同期する。

### Scope and out-of-scope

本sub-unitの変更scopeは、この中央Progress 1 file末尾への本entry追加だけである。既存entryの修正、削除、並べ替えは行わない。

control source、IDL、CMake、package metadata、AGENTS、Skill、template、Project Context、workspace manifest、sibling repositoryは変更しない。stage、commit、push、merge、PR作成、package build、simulation、hardware executionおよびM0-B4開始も本sub-unitのimplementation scope外とする。

### Repository state at sync start

| repository | access | branch | observed HEAD | working tree/index |
|---|---|---|---|---|
| `auto_stabilizer2` | WRITE、中央Progress 1 fileだけ | `wbms-external-teleop` | `e0c311ecb59e3d9ae9bea6b551c03ffd5d58efa3` | clean |
| `whole_body_teleop` | READ | `wbms-external-teleop` | `fd464f574eb26afc25b4e88e67e12b2296d0e420` | clean |
| `ik_solvers2` | READ | `teleop-dev` | `47576209a01a35177ac0d594e586abfa90927dd7` | clean |
| `prioritized_qp` | READ | `teleop-dev` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | clean |
| `rtmros_msg_bridge` | READ | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | clean |

`whole_body_teleop`はremoteなし。HEADはparentのないroot commitである。

### M0-B3 repository commit

| item | recorded result |
|---|---|
| repository | `whole_body_teleop` |
| branch | `wbms-external-teleop` |
| commit SHA | `fd464f574eb26afc25b4e88e67e12b2296d0e420` |
| commit type | root commit |
| subject | `Bootstrap whole_body_teleop repository and packages` |
| parent | none |
| result | 7 files、208 insertions、全file mode `100644` |
| post-commit state | working tree/index clean、remoteなし |

committed exact 7 paths:

```text
AGENTS.md
whole_body_teleop_msgs/AGENTS.md
whole_body_teleop_msgs/CMakeLists.txt
whole_body_teleop_msgs/package.xml
whole_body_teleop_reference_generator/AGENTS.md
whole_body_teleop_reference_generator/CMakeLists.txt
whole_body_teleop_reference_generator/package.xml
```

### Review

| round | target | result | findings/source state |
|---|---|---|---|
| M0-B3 fresh read-only repository review | exact 7 untracked files、repository state、sibling state | PASS | P0/P1/P2/P3 findingなし |
| review後からcommitまで | reviewed source content | PASS | source変更なし。stageとatomic root commitだけを実施 |
| M0-B3-PROGRESS fresh read-only review | 本append-only entryを含むlatest diff全体 | PENDING | implementation taskとは別taskでreviewする |

### M0-B3 static verification and commit evidence

| item | result | evidence |
|---|---|---|
| root `AGENTS.md` template comparison | PASS、byte-identical | template/target SHA-256ともに`2814eb8cfc24293296a15efb6e3b352eb67332b6227660d33666e06aedb2021d` |
| package XML syntax | PASS | 2 packageの`package.xml`に`xmllint --noout` |
| catkin package discovery | PASS | `whole_body_teleop_msgs`、`whole_body_teleop_reference_generator`のexpected 2 packagesだけを認識 |
| repository directory layout | PASS | `.git/`とcommitted exact 7 pathsだけ |
| pre-commit staged scope | PASS | exact 7 pathsだけ |
| `git diff --cached --check` | PASS | error出力なし |
| atomic root commit | PASS | commit parentなし、subject一致 |
| committed paths/stat/modes | PASS | exact 7 paths、208 insertions、全mode `100644` |
| post-commit `whole_body_teleop` state | PASS | branch/HEAD一致、working tree/index clean、remoteなし |
| post-commit sibling repository state | PASS | `auto_stabilizer2`、`ik_solvers2`、`prioritized_qp`、`rtmros_msg_bridge`はexpected branch/HEAD/clean stateを維持 |

### Selected M0-pre-M5-baseline

このtableは正式にselectedされたbaseline dependency setであり、下記のobserved development checkoutをbaseline SHAとして扱わない。

| repository | selected ref/SHA | status |
|---|---|---|
| `auto_stabilizer2` | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | — | NOT APPLICABLE、M0 baseline dependencyではない |

exact compatible-set materialization/buildはM0-B7へdeferする。

### Observed repository checkout set

このtableはM0-B3-PROGRESS開始時に観測したdevelopment checkoutであり、selected `M0-pre-M5-baseline`とは別の記録である。

| repository | observed branch | observed SHA |
|---|---|---|
| `auto_stabilizer2` | `wbms-external-teleop` | `e0c311ecb59e3d9ae9bea6b551c03ffd5d58efa3` |
| `whole_body_teleop` | `wbms-external-teleop` | `fd464f574eb26afc25b4e88e67e12b2296d0e420` |
| `ik_solvers2` | `teleop-dev` | `47576209a01a35177ac0d594e586abfa90927dd7` |
| `prioritized_qp` | `teleop-dev` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` |

### Verification / Unverified

| item | result | owner/evidence |
|---|---|---|
| M0-B3 package skeleton review/static verification | PASS | 上記reviewおよびstatic evidence |
| package build | UNVERIFIED | M0-B7へdefer |
| exact compatible-set materialization/build | UNVERIFIED | M0-B7へdefer |
| simulation | UNVERIFIED | 本sub-unitでは実行しない |
| hardware | UNVERIFIED | 本sub-unitでは実行しない |
| source-root AGENTS/common Skill installation | UNVERIFIED | M0-B5へdefer |
| Project Context/workspace manifest | UNVERIFIED | M0-B6へdefer |

static verification PASSはpackage build、exact compatible-set build、simulationまたはhardware verificationを代替しない。

### Changes

| repository | file | change | reason |
|---|---|---|---|
| `auto_stabilizer2` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | 既存1256行を変更せず、本M0-B3-PROGRESS entryを末尾へappend-only追加 | M0-B3 evidenceとgateの中央Progress同期 |

### Central Progress sync and gates

- M0-B3 repository commit: COMPLETED。
- M0-B3-PROGRESS implementation: 本entryで実施。
- M0-B3-PROGRESS fresh read-only review: PENDING。
- 人間によるM0-B3-PROGRESS exact diffとcommitの明示承認: PENDING。
- M0-B3-PROGRESS commit: PENDING。
- M0-B4: BLOCKED。
- M0-B3-PROGRESSのfresh review、人間承認、commit完了前にM0-B4へ進まない。

### Next entry point

1. 本M0-B3-PROGRESS latest diff全体を別taskでfresh read-only repository reviewする。
2. findingがあれば修正し、修正後のlatest diff全体を再reviewする。
3. review PASS後、別taskでcommit readinessを判定する。
4. 人間がexact diffとcommitを明示承認する。
5. expected subject `Record M0-B3 whole_body_teleop bootstrap`で中央Progress 1 fileだけをcommitする。
6. M0-B3-PROGRESS commit完了後だけM0-B4へ進む。

### Commit

- repository: `auto_stabilizer2`
- expected parent: `e0c311ecb59e3d9ae9bea6b551c03ffd5d58efa3`
- expected subject: `Record M0-B3 whole_body_teleop bootstrap`
- commit SHA: PENDING
