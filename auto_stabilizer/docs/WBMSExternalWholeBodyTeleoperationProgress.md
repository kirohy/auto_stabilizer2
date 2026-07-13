# WBMS外部Whole-Body操縦 作業進捗記録

## 1. 文書の位置づけ

本書は、`WBMSExternalWholeBodyTeleoperationImplementationPlan.md`に基づく作業を時系列で記録するappend-onlyの進捗文書である。

別task、別担当、context圧縮後でも作業を再開できることを目的とする。

参照順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`: 初回計画reviewで確定した修正。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`: 正式仕様とMilestone。
3. 本書: 実施済み作業、検証結果、未確認事項、compatible SHA。
4. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`: Codex作業手順。
5. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`: 現行OpenAI公式Codex情報への対応。
6. `WBMSWalkingPreparationDesignRevisionPlan.md`: walking preparationの既存正式仕様。
7. `WBMSFeasibleVelocityPostureControlProgress.md`: 旧構成の履歴。

過去entryの誤りを静かに書き換えない。訂正は新しいentryとして追記する。

---

## 2. Status一覧

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

## 3. Work Unit entry template

```markdown
## YYYY-MM-DD <Work Unit ID> <title>

### Status

### Repository state
| repository | branch | base SHA | current SHA | dirty |

### Goal

### Scope and out-of-scope

### Code investigation

### Decisions

### Changes
| file | change | reason |

### Commands and results
```sh
<exact command>
```

### Simulation / log evidence

### Review
| round | reviewer/task | findings | resolution |

### Acceptance
| criterion | result | evidence |

### Unverified

### Open issues

### Compatible dependency set
| repository | SHA |

### Next entry point

### Commit
```

---

## 2026-07-13 PLAN-0 外部Whole-Body操縦方針・実装計画策定

### Status

COMMITTED

### Repository state

| repository | branch | base SHA | current SHA | dirty |
|---|---|---|---|---|
| `kirohy/auto_stabilizer2` | `wbms-external-teleop-plan` | `wbms-dev` HEAD `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | 文書commit群 | GitHub API上で作成 |
| `kirohy/rtmros_msg_bridge` | 未作成 | default `master` | 未変更 | 未確認 |
| `kirohy/ik_solvers2` | 想定`teleop-dev` | 未記録 | 未変更 | 未確認 |
| `kirohy/prioritized_qp` | 想定`teleop-dev` | 未記録 | 未変更 | 未確認 |
| `whole_body_teleop` | 未作成 | - | - | - |

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

確認した現行要点:

- `auto_stabilizer`は`jointControllable=false`の関節をfinal IK変数から外し、出力時には`refRobotRaw`の値を用いる。
- `qRef`は`refRobotRaw`へ読み込まれ、その後FK/COM計算とframe変換が行われる。
- このため首・将来の指関節は、`qRef`読込後・FK前に許可付きoverrideを適用することで、final IK変数を増やさずモデルと出力を整合できる。
- 現行WBMS腕差分操縦はmaster/slave開始poseを保存し、並進差分だけをscaleし、CHEST相対hand targetを生成している。
- M4.2.2 walking preparationは、COM高さ保持、RETURNING/HANDOFF、READY、歩行API gate、歩行中腕継続を担当するためauto_stabilizer内部へ残す。
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

| file | change | reason |
|---|---|---|
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md` | 正式実装計画を追加 | 新アーキテクチャの仕様・Milestoneを固定 |
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md` | Codex作業標準を追加 | 長期・複数repo作業を小さくreview可能に進める |
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | 本進捗文書を追加 | 別taskへの引き継ぎ |
| `.agents/skills/*` | Work Unit用Skillを追加予定 | 定型反復の標準化 |
| `AGENTS.md` | 恒久workflowルールを追加予定 | repository全体の安全運用 |

### Commands and results

GitHub connectorで以下を実施した。

```text
create branch wbms-external-teleop-plan from wbms-dev
```

PASS。

```text
create implementation plan
create Codex workflow
create Progress
```

PASS。

Local checkoutでの以下は未実行。

```sh
git diff --check
catkin build auto_stabilizer --no-deps
```

NOT RUN。本entryは文書作成のみであり、Local checkoutとbuild環境を使用していない。

### Simulation / log evidence

なし。旧構成の挙動根拠は既存ProgressとM4.2.2/M5ログを参照した。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| 0 | 仕様対話 | interface、stale、head、branch、priorityに多数の未確定点 | ユーザー回答により解消 |
| 1 | 文書自己点検 | 実装branchと計画branchを分ける必要 | plan branchを作成し、pre-M5 branchへ文書だけcherry-pickする方針を採用 |

正式なdetached `/review`は文書群作成後に実施する。

### Acceptance

| criterion | result | evidence |
|---|---|---|
| 主要仕様のユーザー判断が確定 | PASS | 手、CHEST、COM、head、stale、branch、legacy、priorityを確定 |
| 新アーキテクチャ計画がある | PASS | ImplementationPlan |
| Codex反復workflowがある | PASS | CodexWorkflow |
| source codeを変更していない | PASS | 文書・Skill・AGENTSのみのplanning branch |
| exact pre-M5 base SHA | PENDING | M0-Aで確定 |
| performance threshold | PENDING | M0 baseline後に追記 |

### Unverified

- M4.2.2全review修正を含む最後のpre-M5 commit SHA。
- `ik_solvers2`、`prioritized_qp`の実装開始時HEAD SHA。
- pre-M5 baselineのcurrent build状態。
- new message/IDLで使用する具体的package名と生成手順。
- external generator 100 Hzでのsolve時間。
- 実機のactual hand tracking error。

### Open issues

- M0でbranch archaeologyを行う。
- performance acceptanceの数値はM0 baselineに基づいて追記する。
- external node用新repositoryを作成する。
- planning文書をpre-M5実装branchへcherry-pickする手順を確定する。

### Compatible dependency set

未確定。M0で記録する。

### Next entry point

次Work Unit:

```text
M0-A: branch archaeology and exact baseline selection
```

最初に読むもの:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
2. 本ProgressのPLAN-0 entry
3. `WBMSWalkingPreparationDesignRevisionPlan.md`
4. `WBMSFeasibleVelocityPostureControlProgress.md`後方のM4.2.2完了記録と最初のM5記録
5. Git historyの該当commit群

注意:

- 現在の`wbms-dev` HEADを実装baseにしない。
- M5コードを無条件cherry-pickしない。
- walking preparation review修正を落とさない。

### Commit

| SHA | subject |
|---|---|
| `8261114e9fe7bcd61fde14e12a6f5eda50b767d4` | `Add external whole-body teleoperation implementation plan` |
| `e568de33d9e74bea686433c9a0fa8f38daecbc75` | `Add Codex workflow for external teleoperation project` |

本Progress、Skill、AGENTS変更のSHAは後続entryへ追記する。

---

## 2026-07-13 PLAN-0B Codex運用文書・Skill完成と文書review

### Status

COMMITTED

### Repository state

| repository | branch | base SHA | current SHA | dirty |
|---|---|---|---|---|
| `kirohy/auto_stabilizer2` | `wbms-external-teleop-plan` | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` | `6fb17c38a4c7c37a208514260b4580f1957d85ee`時点までの文書・Skill群 | GitHub connector上で変更、local status未確認 |
| `kirohy/rtmros_msg_bridge` | 未作成 | `master`想定 | 未変更 | 未確認 |
| `kirohy/ik_solvers2` | `teleop-dev`想定 | 未記録 | 未変更 | 未確認 |
| `kirohy/prioritized_qp` | `teleop-dev`想定 | 未記録 | 未変更 | 未確認 |
| `whole_body_teleop` | 未作成 | - | - | - |

### Goal

長期・複数repository・安全重要projectをCodexで小さく実装、review、引き継ぎ、commitできる運用文書とrepository-local Skillを完成させ、初回計画の文書矛盾を修正する。

### Scope and out-of-scope

含む:

- package-level `AGENTS.md`。
- Plan、Implement、Review、Closeの4 Skill。
- OpenAI公式Codex情報への対応指針。
- 初回Implementation Planのbranch/heartbeat修正。
- review、Progress、commit templateと禁止事項。

含まない:

- source code実装。
- exact pre-M5 base SHA調査。
- local build、simulation、実機確認。
- planning branchのpush/merge/PR。

### Code investigation

- OpenAI公式AGENTS.md資料では、Codexがproject rootからcurrent directoryへinstructionを連結し、近いdirectoryのinstructionを後段で適用することを確認した。
- OpenAI公式Skill資料では、`.agents/skills/<name>/SKILL.md`、`name`/`description`、progressive disclosure、明示呼出しを確認した。
- OpenAI公式code review資料では、`/review`がworking treeを変更せずprioritized findingを返し、detached reviewを選択できることを確認した。
- OpenAI公式long-running work資料では、clear outcome、constraints、definition of done、CLIの`/goal`、independent taskの別chatを確認した。
- OpenAI公式model資料では、GPT-5.6 Sol/Terra/Luna、reasoning level、Max/Ultra、Ultraのsubagent利用を確認した。
- 元計画19.1はexternal source未使用時にもgenerator/bridge heartbeatを必須と読め、legacy-only運用と矛盾していた。
- 「全M4.2.2修正完了かつM5前」の単一commitが履歴上存在することを暗黙に仮定していた。

### Decisions

採用:

- Work Unitをdistinct outcomeの単位とする。
- read-only planning、承認済みContract implementation、detached review、fresh review、closureの順で進める。
- implementation/review/closureを別Skillへ分離する。
- source変更とcommitを分離し、commitは明示許可時だけ行う。
- planning phaseは特定UI名へ固定せず、clientにplan capabilityがあれば使用する。
- long-running implementationはGoalとdefinition of doneを明示する。
- safety-critical/branch archaeologyはGPT-5.6 Sol High/Extra Highを基本とする。
- schema固定後の通常実装はTerra、明確な反復変換はLunaを候補とする。
- Ultraは分割可能な独立taskだけに使用する。
- external sourceを一つでも選択した場合だけgenerator/bridge heartbeatをWBMS開始条件にする。
- legacy-onlyではexternal heartbeatを要求しない。
- 単一pre-M5 baseが無い場合は、pre-M5 commitへ必要なM4.2.2 review修正だけを選択的にcherry-pickしたsynthetic baselineを作る。

不採用:

- 全projectを一つの巨大Goalで実装する。
- implementation task自身だけのreviewで完了とする。
- review finding修正後にincremental部分だけを見る。
- same sourceへ複数taskのwrite accessを与える。
- model名やplan mode名を恒久仕様として固定する。
- current `wbms-dev` HEADから大量削除するだけで実装baseを作る。

### Changes

| file | change | reason |
|---|---|---|
| `auto_stabilizer/AGENTS.md` | 500 Hz、安全、interface、review、commitのpackage規約を追加 | 各taskで安定した制約を自動適用 |
| `.agents/skills/wbms-plan-work-unit/SKILL.md` | read-only調査とContract作成 | 実装前のscope freeze |
| `.agents/skills/wbms-implement-work-unit/SKILL.md` | Contract内実装とverification、commit禁止 | 実装scopeと安全制約の固定 |
| `.agents/skills/wbms-review-work-unit/SKILL.md` | P0-P3 read-only review | 実装者から独立したfinding |
| `.agents/skills/wbms-close-work-unit/SKILL.md` | Progress、checklist、commit readiness | 未確認事項を隠さないatomic closure |
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md` | 現行公式Codex機能、model、mode、environment適用 | UI更新と安定原則を分離 |
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md` | branch基点、heartbeat、Codex mode、Skill完成条件を修正 | 初回計画review findingの解消 |
| `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` | PLAN-0B entryを追記 | 別taskへの引き継ぎ |

### Commands and results

GitHub connector:

```text
create auto_stabilizer/AGENTS.md
create four .agents/skills/*/SKILL.md
create CodexOfficialGuidance
create ImplementationPlanRevision1
compare wbms-dev...wbms-external-teleop-plan
```

PASS。compare時点ではplanning branchは`wbms-dev`より13 commit先行し、source code変更はなく、文書・AGENTS・Skillの10 fileだけが追加されていた。

OpenAI公式web資料:

```text
open AGENTS.md / Skills / Code review / Long-running work / Projects / Environments / Worktrees / Models
```

PASS。2026-07-13時点の現行説明を確認した。

Local checkoutでの以下は未実行。

```sh
git status --short
git diff --check
codex --ask-for-approval never "Summarize the current instructions."
catkin build auto_stabilizer --no-deps
```

NOT RUN。containerからGitHubへのcloneはDNS制約で失敗し、GitHub connectorで文書を作成した。

### Simulation / log evidence

なし。control sourceは変更していない。

### Review

| round | reviewer/task | findings | resolution |
|---|---|---|---|
| 2 | 文書・workflow自己review | external未使用でもheartbeat必須と読める | Revision 1でexternal source選択時だけ必須へ修正 |
| 2 | 文書・workflow自己review | 必要なM4.2.2修正とM5が交錯した場合のbase構築が未定義 | synthetic baseline手順をRevision 1へ追加 |
| 2 | 公式情報照合 | workflowが特定のplan mode名と旧URLへ依存し得る | CodexOfficialGuidanceでread-only planning phase、`/goal`、現行公式URLへ整理 |
| 2 | Skill構成review | Plan Skillしか存在せずworkflow記載と不一致 | Implement、Review、Close Skillを追加 |
| 2 | instruction hierarchy review | main planだけを最優先にするとRevisionが読まれない | package `AGENTS.md`の参照順をRevision 1優先へ更新 |

正式なCodex detached `/review`はlocal project/worktreeで未実行。

### Acceptance

| criterion | result | evidence |
|---|---|---|
| 正式Implementation Planがある | PASS | ImplementationPlan + Revision 1 |
| M0-M12の段階的Milestoneがある | PASS | ImplementationPlan |
| Codex Work Unit workflowがある | PASS | CodexWorkflow |
| 現行OpenAI公式情報への対応がある | PASS | CodexOfficialGuidance |
| package-level safety instructionがある | PASS | `auto_stabilizer/AGENTS.md` |
| Plan/Implement/Review/Close Skillがある | PASS | `.agents/skills/`の4 Skill |
| review重点・非finding指定がある | PASS | CodexWorkflowとReview Skill |
| append-only Progress templateがある | PASS | 本書 |
| commit checklistと明示許可条件がある | PASS | CodexWorkflowとClose Skill |
| source codeを変更していない | PASS | branch compareは文書、AGENTS、Skillのみ |
| local `git diff --check` | UNVERIFIED | local checkoutなし |
| CodexによるSkill/AGENTS discovery | UNVERIFIED | M0でlocal確認 |
| formal detached document review | UNVERIFIED | M0開始前またはplanning PRで実施 |

### Unverified

- exact pre-M5またはsynthetic baseline SHA。
- planning branchのlocal `git diff --check`。
- Codexがroot/package `AGENTS.md`と4 Skillを正しく検出すること。
- formal detached `/review`のfinding。
- package build。source変更はないがlocal build環境未使用。
- dependency branch/HEAD SHA。

### Open issues

- M0-Aでcommit履歴、コード、正式文書を照合する。
- M0でactive AGENTS chainとSkill一覧をCodexに出力させる。
- planning branch文書を実装baseへcherry-pickする単位を決定する。
- formal document reviewのP0/P1/P2を解消してからM0-Bへ進む。

### Compatible dependency set

| repository | SHA |
|---|---|
| `auto_stabilizer2` planning source base | `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` |
| `auto_stabilizer2` planning docs | `6fb17c38a4c7c37a208514260b4580f1957d85ee`以降、本entry commitまで |
| `rtmros_msg_bridge` | 未記録 |
| `ik_solvers2` | 未記録 |
| `prioritized_qp` | 未記録 |
| `whole_body_teleop` | 未作成 |

### Next entry point

次Work Unit:

```text
M0-A: branch archaeology and exact baseline selection
```

開始task:

```text
$wbms-plan-work-unit M0-A
```

最初に読む順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
3. 本ProgressのPLAN-0B。
4. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
5. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
6. `WBMSWalkingPreparationDesignRevisionPlan.md`。
7. 旧Progress後方のM4.2.2/M5記録。
8. git history。

注意:

- 単一base commitの存在を仮定しない。
- synthetic baselineが必要なら、walking preparation修正とM5変更をcommit単位・diff単位で分類する。
- current `wbms-dev` HEADを実装baseにしない。
- source変更を行わずContractを作成する。

### Commit

| SHA | subject |
|---|---|
| `f8108e8d65586754bf5754f5746f04b50e0ba899` | `Add progress log for external teleoperation project` |
| `eaa45c961f5b15e1fdcedf9ac37fdb9e4880cd7d` | `Add package instructions for external teleoperation work` |
| `3413fc82c5de5488024212da26577e747a8121d0` | `Add WBMS work unit planning skill` |
| `a39ed803f2baeda9b1db3326fabd6a8f0442e9f9` | `Add WBMS work unit implementation skill` |
| `ce106f15e3547eb9ee7e5fae6b3a02a650ea48fe` | `Add WBMS work unit review skill` |
| `9b345225584ce672445f6dea9bfdee7ffef65c04` | `Add WBMS work unit closure skill` |
| `5bb65793c603d06a9c0b481ad4adc8173ddc43de` | `Add official Codex guidance for WBMS workflow` |
| `59d5f26fa516cd3caf39e35a0e8360049dc61bcb` | `Reference official Codex guidance in package instructions` |
| `2458371b1c6adff4f6f5d358ecfd8eabfc19b531` | `Clarify external teleoperation implementation plan` |
| `1491662239a2aedbd482e46afbd193e9674a468e` | `Prioritize external teleoperation plan revision` |
| `6fb17c38a4c7c37a208514260b4580f1957d85ee` | `Align WBMS planning skill with revised plan` |

本PLAN-0B entryを追加したcommit SHAは、次entryのRepository stateで記録する。
