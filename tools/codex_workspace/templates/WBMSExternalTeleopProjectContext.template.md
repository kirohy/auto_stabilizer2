# WBMS外部Whole-Body操縦 Project Context

## 1. 目的

このrepositoryの責務と、中央計画、workflow、Current Checkpoint、compatible setへの参照を記録する。

正式計画全文を複製しない。
中央正本のrepository、branch、commit SHA、pathを固定して参照する。

## 2. Project identity

```text
project_id: WBMS_EXTERNAL_WHOLE_BODY_TELEOP
repository: <owner/repository>
repository_role: <role>
```

## 3. Workspace

```text
CATKIN_WORKSPACE: <absolute path to catkin_ws/<workspace_name>>
CATKIN_SOURCE_ROOT: <CATKIN_WORKSPACE>/src
repository_path: <CATKIN_SOURCE_ROOT>/<repository>
```

Git管理する場合、環境固有absolute pathはplaceholderまたは相対pathにしてよい。

## 4. Authoritative documents

| type | repository | branch | commit SHA | path |
|---|---|---|---|---|
| Implementation Plan Revision 2 | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md` |
| Codex Workflow Revision 1 | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md` |
| Current Checkpoint | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md` |
| Multi-repository operations | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md` |
| Control Implementation Plan | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md` |
| Central Progress | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` |

workflow gateが矛盾する場合、Codex Workflow Revision 1を優先する。
control behaviorはImplementation Plan/Revisionを優先する。

## 5. Repository responsibility

### In scope

- <responsibility>

### Out of scope

- <responsibility>

### Safety / compatibility invariants

- <invariant>

## 6. Related repositories

| repository | path | role | access |
|---|---|---|---|
| `auto_stabilizer2` | `${CATKIN_SOURCE_ROOT}/auto_stabilizer2` | realtime consumer / central docs | READ / WRITE / NONE |
| `whole_body_teleop` | `${CATKIN_SOURCE_ROOT}/whole_body_teleop` | ROS reference generator | READ / WRITE / NONE |
| `rtmros_msg_bridge` | `${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge` | ROS/RTM bridge | READ / WRITE / NONE |
| `ik_solvers2` | `${CATKIN_SOURCE_ROOT}/ik_solvers2` | IK library | READ / WRITE / NONE |
| `prioritized_qp` | `${CATKIN_SOURCE_ROOT}/prioritized_qp` | QP backend | READ / WRITE / NONE |

## 7. Current Work Package

```text
parent_work_package: <ID>
repository_sub_unit: <ID or N/A>
risk_level: R0 / R1 / R2 / R3
contract_or_brief_path: <path>
schema_version: <version or N/A>
review_policy: <SELF / TARGETED / REPOSITORY_FULL / COMPATIBLE_SET / SAFETY_FULL>
commit_authorization: none / exact / standing
```

## 8. Compatible dependency set

```yaml
compatible_set_name: <name>
repositories:
  auto_stabilizer2: <sha>
  whole_body_teleop: <sha>
  rtmros_msg_bridge: <sha>
  ik_solvers2: <sha>
  prioritized_qp: <sha>
```

未変更repositoryも使用中SHAを記録する。

## 9. Build

workspace一括buildを標準にしない。

| package | normal command | dependency check | force-cmake condition |
|---|---|---|---|
| `<package>` | `catkin build <package> --no-deps` | `catkin build <package>` | `<condition>` |

execution directory、exact command、resultをverification evidenceへ記録する。

## 10. Review

- repository-specific review focus。
- riskに応じたreview type。
- reviewed commit SHAまたはdiff hash。
- material changeの場合だけfull fresh review。
- Progress/Markdownだけの追記はsource reviewを無効化しない。

## 11. Checkpoint / Progress

```text
current_checkpoint_required: true / false
central_progress_checkpoint: Parent completion / compatible set / simulation / milestone / handoff / N/A
last_recorded_repository_commit: <sha or NONE>
last_central_progress_checkpoint_commit: <sha or NONE>
```

repository commitごとの中央Progress syncを既定にしない。
exact SHAはParent working stateと次sub-unitへ渡し、
integration、simulation、milestone、引き継ぎ前に中央Progressへ記録する。

## 12. Stop conditions

- Parent scope外。
- riskがR3へ上昇。
- schema/safety invariant変更。
- sibling WRITEが必要。
- user変更との衝突。
- destructive Git操作。
- simulation/hardware。

## 13. Open issues

- <issue>

## 14. Next entry point

- next Work Package / sub-unit
- launch directory
- first files/functions
- required review/build
- warnings
