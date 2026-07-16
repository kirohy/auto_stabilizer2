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
