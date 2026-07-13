# WBMS外部Whole-Body操縦 Project Context

## 1. 文書の目的

本書は、このrepositoryがWBMS外部whole-body操縦プロジェクトで担う責務と、中央計画・Progress・compatible setへの参照を記録する。

正式計画全文を本repositoryへ複製しない。中央正本のrepository、branch、commit SHA、pathを固定して参照する。

---

## 2. Project identity

```text
project_id: WBMS_EXTERNAL_WHOLE_BODY_TELEOP
repository: <owner/repository>
repository_role: <role>
```

---

## 3. Workspace path

```text
CATKIN_WORKSPACE: <absolute path to catkin_ws/<workspace_name>>
CATKIN_SOURCE_ROOT: <CATKIN_WORKSPACE>/src
repository_path: <CATKIN_SOURCE_ROOT>/<repository>
```

local absolute pathは環境固有である。Git管理する場合、placeholderまたは相対pathを使用し、実環境値はProgress/Work Unit Contractへ記録してよい。

---

## 4. Authoritative documents

| type | repository | branch | commit SHA | path |
|---|---|---|---|---|
| Implementation Plan Revision 2 | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md` |
| Multi-repository operations | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md` |
| Implementation Plan Revision 1 | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md` |
| Implementation Plan | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md` |
| Central Progress | `kirohy/auto_stabilizer2` | `<branch>` | `<sha>` | `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md` |

参照SHAを変更する場合、理由とcompatible setを記録する。

---

## 5. Repository responsibility

### In scope

- <responsibility>

### Out of scope

- <responsibility>

### Safety / compatibility invariants

- <invariant>

---

## 6. Related repositories

| repository | path | role | access for current Work Unit |
|---|---|---|---|
| `auto_stabilizer2` | `${CATKIN_SOURCE_ROOT}/auto_stabilizer2` | realtime consumer / central docs | READ / WRITE / NONE |
| `whole_body_teleop` | `${CATKIN_SOURCE_ROOT}/whole_body_teleop` | ROS reference generator | READ / WRITE / NONE |
| `rtmros_msg_bridge` | `${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge` | ROS/RTM bridge | READ / WRITE / NONE |
| `ik_solvers2` | `${CATKIN_SOURCE_ROOT}/ik_solvers2` | IK library | READ / WRITE / NONE |
| `prioritized_qp` | `${CATKIN_SOURCE_ROOT}/prioritized_qp` | QP backend | READ / WRITE / NONE |

---

## 7. Current Work Unit

```text
parent_work_unit: <ID>
repository_sub_unit: <ID>
contract_path: <path>
schema_version: <version or N/A>
```

---

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

---

## 9. Build

workspace一括buildを標準にしない。

| package | normal command | dependency check command | force-cmake condition |
|---|---|---|---|
| `<package>` | `catkin build <package> --no-deps` | `catkin build <package>` | `<condition>` |

`catkin build`の実行directoryは固定しない。Work Unit reportと中央Progressへexact commandとexecution directoryを記録する。

---

## 10. Review focus

- <repository-specific review item>

---

## 11. Central Progress sync

```text
required_after_commit: true / false
sync_sub_unit: <ID or N/A>
last_synced_repository_commit: <sha or NONE>
last_central_progress_commit: <sha or NONE>
```

他repositoryのcommit後は、依存する次sub-unit前に中央ProgressへSHAを同期する。

---

## 12. Open issues

- <issue>

---

## 13. Next entry point

- first file/function to read
- next Work Unit
- warnings
