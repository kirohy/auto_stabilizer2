# WBMS外部Whole-Body操縦 Current Checkpoint

## 1. 位置づけ

本書は現在の作業状態を短く示すmutable checkpointである。
履歴の正本は`WBMSExternalWholeBodyTeleoperationProgress.md`に残す。

workflowについては次を優先する。

1. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`
2. 本書
3. 旧Workflow、MultiRepositoryOperations、Progress内の旧gate

本書以前のProgress entryは履歴snapshotとして変更しない。
旧entryに残る`PENDING`や`BLOCKED`は、最新workflowでsupersedeされた場合に限り
現在のblockerとして扱わない。

---

## 2. Workflow

- workflow version: `WORKFLOW-R1`
- risk model: `R0 / R1 / R2 / R3`
- one-write-repository rule: 維持
- simulation gate: 人間の明示許可
- hardware gate: 人間の別明示許可
- control safety invariant: 変更なし

---

## 3. 現在地

### 完了済み

- M0-A exact baseline selection。
- M0-B1 isolated workspace方針。
- M0-B2 `auto_stabilizer2` isolated implementation branch materialization。
- M0-B3 `whole_body_teleop` repository/package bootstrap。
- M0-B3 repository reviewとstatic verification。
- M0-B3中央Progress記録commit。

### 現在の主要repository

| repository | branch/ref | recorded SHA | status |
|---|---|---|---|
| `auto_stabilizer2` | `wbms-external-teleop` | workflow migration直前: `e5495f4e5af9e017e73b21543d483b196a421044` | active |
| `whole_body_teleop` | `wbms-external-teleop` | `fd464f574eb26afc25b4e88e67e12b2296d0e420` | bootstrap committed |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | M0-B4待ち |
| `ik_solvers2` | observed `teleop-dev` | `47576209a01a35177ac0d594e586abfa90927dd7` | observed checkout |
| `prioritized_qp` | observed `teleop-dev` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | observed checkout |

### Selected M0-pre-M5-baseline

| repository | selected SHA |
|---|---|
| `auto_stabilizer2` | `c06b63c8e12dbf85bda4c8391a37544c2469731c` |
| `ik_solvers2` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` |
| `prioritized_qp` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` |
| `rtmros_msg_bridge` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` |

observed development checkoutとselected baselineを混同しない。

---

## 4. Workflow migration

M0-B3のrepository sourceはreview済みであり、再reviewしない。

次の旧gateはsuperseded:

- M0-B3 Progress-only diffのfresh detached review。
- Progress-only closureのための全file再走査。
- M0-B4を上記review完了までblockする規則。
- M0-B4以降の各micro-stepごとの中央Progress commit。

workflow migration commit後、M0-B4へ進んでよい。

---

## 5. Next Parent Work Package

```text
M0-REMAINDER
Risk: R0 -> R1
```

### R0 bootstrap remainder

- M0-B4: `rtmros_msg_bridge` branch/instruction bootstrap。
- M0-B5: `${CATKIN_SOURCE_ROOT}/AGENTS.md`、common Skill installation/recognition。
- M0-B6: Project Context、workspace manifest、明示defer状態。
- `whole_body_teleop`実配置`AGENTS.md`のWORKFLOW-R1同期。
- R0全体にfocused bootstrap reviewを一回。

各repository commitは分離する。
各commit間の中央Progress syncは不要。
commit SHAはParent working stateと次promptへ直接渡す。

### R1 baseline verification

- M0-B7: exact compatible set materialization。
- package-specific build。
- workspace一括buildは行わない。

### M0 integration

- M0-B8: cross-repository compatible/instruction/build review。
- named compatible setをfreeze。
- M0完了Progress entryを一件追加。
- M1-Pへ進む。

---

## 6. Pending verification

- M0 exact compatible-set package build: `M0-B7`。
- source-root `AGENTS.md`とcommon Skill認識: `M0-B5`。
- Project Context / manifest: `M0-B6`。
- M0 cross-repository review: `M0-B8`。
- simulation: `UNVERIFIED`。M0では通常実行しない。
- hardware: `UNVERIFIED`。

---

## 7. Stop conditions

以下が発生した場合だけ停止する。

- control source変更が必要。
- schemaまたはsafety invariant変更が必要。
- userの未commit変更と衝突。
- destructive Git操作が必要。
- selected baseline SHAと現物が一致しない。
- build failureの修正が別repositoryに必要。
- simulationまたは実機実行が必要。

---

## 8. 次のeligible action

`M0-REMAINDER` Parent Work Packageを一度計画・承認し、
M0-B4〜B6をR0として順次実行する。

具体的promptは`WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision2.md`を使用する。
