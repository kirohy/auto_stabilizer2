# Codex workspace bootstrap assets

## 1. 目的

このdirectoryは、WBMS外部whole-body操縦プロジェクトを`catkin_ws/<workspace_name>/src`配下の複数repositoryで運用するためのbootstrap素材を保持する。

本directoryのfileを自動的にworkspaceへ適用してはならない。M0-Bの承認済みWork Unit Contractに従って配置する。

## 2. Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

## 3. Templates

| file | target |
|---|---|
| `templates/AGENTS.source-root.md` | `${CATKIN_SOURCE_ROOT}/AGENTS.md` |
| `templates/AGENTS.whole_body_teleop.md` | `whole_body_teleop/AGENTS.md` |
| `templates/AGENTS.rtmros_msg_bridge.md` | `rtmros_msg_bridge/AGENTS.md`または対象package |
| `templates/AGENTS.ik_solvers2.md` | `ik_solvers2/AGENTS.md`、変更が必要な場合 |
| `templates/AGENTS.prioritized_qp.md` | `prioritized_qp/AGENTS.md`、変更が必要な場合 |
| `templates/WBMSExternalTeleopProjectContext.template.md` | 各repositoryの`docs/WBMSExternalTeleopProjectContext.md` |
| `templates/WBMSExternalTeleopWorkspaceManifest.template.yaml` | M0-Bで確定するworkspace manifest |

## 4. Common Skills

共通Skillの正本:

```text
auto_stabilizer2/.agents/skills/
```

M0-Bでは、各repository rootから認識できるよう`${HOME}/.agents/skills`へsymlinkする。

```text
wbms-plan-work-unit
wbms-implement-work-unit
wbms-review-work-unit
wbms-close-work-unit
```

同じSkillを各repositoryへcopyして別version化しない。

## 5. Bootstrap script

後続Work Unitで次を実装してよい。

```text
bootstrap_codex_workspace.sh
verify_codex_workspace.sh
```

scriptの要件:

- `${CATKIN_WORKSPACE}`を引数で受ける。
- `${CATKIN_SOURCE_ROOT}`を導出する。
- 既存fileを無断で上書きしない。
- templateとtargetのdiffを表示する。
- Skill symlink先を表示する。
- repository path、branch、HEAD、dirty stateを確認する。
- 不一致を自動修正せず報告する。
- control sourceを変更しない。

本planning Work Unitではscript自体を作成しない。

## 6. Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存関係まで確認する場合だけ:

```sh
catkin build <package-name>
```

`catkin build`の実行directoryは固定しない。exact commandとexecution directoryをProgressへ記録する。

## 7. 配置前check

- MultiRepositoryOperationsとRevision 2を読む。
- M0-B Contractを承認する。
- target repositoryの既存`AGENTS.md`を確認する。
- userのlocal workspace fileを上書きしない。
- central documentsのbranch/SHAをProject Contextへ固定する。
- Codexにactive `AGENTS.md` chainとSkill一覧を列挙させる。
