# Codex workspace bootstrap assets

## 1. 目的

このdirectoryは、WBMS外部whole-body操縦projectを
`catkin_ws/<workspace_name>/src`配下の複数repositoryで運用するためのtemplateを保持する。

templateを自動適用しない。
承認済みM0 Work Packageまたは後続bootstrap Work Packageに従って配置する。

## 2. 正式workflow

配置前に読む。

1. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision2.md`
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`

旧workflowと矛盾する場合、Workflow Revision 1を優先する。

## 3. Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

## 4. Templates

| file | target |
|---|---|
| `templates/AGENTS.source-root.md` | `${CATKIN_SOURCE_ROOT}/AGENTS.md` |
| `templates/AGENTS.whole_body_teleop.md` | `whole_body_teleop/AGENTS.md` |
| `templates/AGENTS.rtmros_msg_bridge.md` | `rtmros_msg_bridge/AGENTS.md`または対象package |
| `templates/AGENTS.ik_solvers2.md` | `ik_solvers2/AGENTS.md`、変更が必要な場合 |
| `templates/AGENTS.prioritized_qp.md` | `prioritized_qp/AGENTS.md`、変更が必要な場合 |
| `templates/WBMSExternalTeleopProjectContext.template.md` | 各repositoryのProject Context |
| `templates/WBMSExternalTeleopWorkspaceManifest.template.yaml` | workspace manifest |

実配置済みfileが古いworkflowを含む場合、R0 bootstrap remainderでtemplateと同期する。
control sourceと同じcommitへ混ぜない。

## 5. Common Skills

正本:

```text
auto_stabilizer2/.agents/skills/
```

M0-B5相当で`${HOME}/.agents/skills`へsymlinkし、各repository rootから認識させる。

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

各repositoryへcopyして別version化しない。

## 6. Risk-based bootstrap

bootstrapは原則R0。

- sub-unitごとのdetached reviewを要求しない。
- 必要な構文/package discoveryを実行する。
- 一つのimplementation実行がWRITEするrepositoryは一つ。
- repository commitは分離する。
- Parent Work Package末尾にfocused bootstrap reviewを一回行う。
- 中央ProgressはM0 completion checkpointでまとめて更新する。

## 7. Bootstrap script

後続Work Packageで実装してよい。

```text
bootstrap_codex_workspace.sh
verify_codex_workspace.sh
```

要件:

- `${CATKIN_WORKSPACE}`を引数で受ける。
- `${CATKIN_SOURCE_ROOT}`を導出。
- 既存fileを無断上書きしない。
- template/target diffを表示。
- Skill symlink先を表示。
- repository path、branch、HEAD、dirty stateを確認。
- 不一致を自動修正せず報告。
- control sourceを変更しない。

## 8. Build

workspace一括buildを標準にしない。

```sh
catkin build <package-name> --no-deps
```

dependency確認時だけ`--no-deps`を外す。
execution directory、exact command、resultを記録する。

## 9. 配置前check

- Parent Work Package / Work Briefを承認済み。
- target repositoryの既存`AGENTS.md`を確認。
- userのlocal fileを上書きしない。
- one-write-repository rule。
- Current Checkpointを確認。
- source-root/各repositoryからSkill認識を確認。
- simulation/実機を開始しない。
