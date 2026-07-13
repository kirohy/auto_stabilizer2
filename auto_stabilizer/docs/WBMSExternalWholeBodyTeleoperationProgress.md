# WBMS外部Whole-Body操縦 作業進捗記録

## 1. 文書の位置づけ

本書は、`WBMSExternalWholeBodyTeleoperationImplementationPlan.md`に基づく作業を時系列で記録するappend-onlyの進捗文書である。

別task、別担当、context圧縮後でも作業を再開できることを目的とする。

参照順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`: 正式仕様とMilestone。
2. 本書: 実施済み作業、検証結果、未確認事項、compatible SHA。
3. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`: Codex作業手順。
4. `WBMSWalkingPreparationDesignRevisionPlan.md`: walking preparationの既存正式仕様。
5. `WBMSFeasibleVelocityPostureControlProgress.md`: 旧構成の履歴。

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
