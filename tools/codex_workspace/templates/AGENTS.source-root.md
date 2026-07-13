# WBMS外部Whole-Body操縦 source-root instructions

この文書は`${CATKIN_SOURCE_ROOT}/AGENTS.md`として配置するtemplateである。

## Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして扱わない。

## 対象repository

```text
${CATKIN_SOURCE_ROOT}/auto_stabilizer2
${CATKIN_SOURCE_ROOT}/whole_body_teleop
${CATKIN_SOURCE_ROOT}/rtmros_msg_bridge
${CATKIN_SOURCE_ROOT}/ik_solvers2
${CATKIN_SOURCE_ROOT}/prioritized_qp
```

## 正式仕様

中央正本は`auto_stabilizer2/auto_stabilizer/docs/`にある。

次を優先して読む。

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
3. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
4. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
5. `WBMSExternalWholeBodyTeleoperationProgress.md`

## Cross-repository task

- `${CATKIN_SOURCE_ROOT}`から行うtaskは原則read-onlyとする。
- branch archaeology、Parent Contract、protocol review、compatible-set review、package build統合確認、simulation計画、log解析を扱う。
- source修正が必要な場合は対象repositoryのsub-unitへ分離する。
- 一つのimplementation taskがWRITEするrepositoryは原則一つとする。
- source rootから複数repositoryを同時編集しない。

## Instruction読込

repositoryを調査する前に、次を明示的に読む。

1. 対象repository rootの`AGENTS.md`。
2. nearest package/module `AGENTS.md`。
3. `docs/WBMSExternalTeleopProjectContext.md`、存在する場合。

子repositoryの`AGENTS.md`が自動適用されたと仮定しない。

## Repository状態

全対象repositoryで次を確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- user変更を無断でreset、stash、checkout、clean、削除しない。
- branch、HEAD、dirty stateをWork Unit ContractとProgressへ記録する。
- compatible setと異なる場合、勝手に切り替えず報告する。

## Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存関係まで確認する場合だけ:

```sh
catkin build <package-name>
```

`catkin build`の実行directoryは固定しない。exact commandと実行directoryを記録する。

`build`、`devel`、`install`、`logs`をsource変更として扱わない。

## Reviewとcommit

- cross-repository reviewはread-onlyで行う。
- findingは対象repository sub-unitへ割り当てる。
- commitは対象repository rootから行う。
- 一度に一つのrepositoryだけstage/commitする。
- 他repository commit後、依存する次sub-unit前に中央ProgressへSHAを同期する。
- push、merge、PR作成は別の明示許可を必要とする。

## Simulationと実機

- simulation起動、command送信、log取得はユーザーの明示許可を必要とする。
- 実機実行は常に別の明示許可を必要とする。
- simulation/実機未確認をPASS扱いしない。

## 禁止事項

- cross-repository implementationを一つの巨大taskで行わない。
- schema未確定でproducer、bridge、consumerを並行実装しない。
- 同じsource fileを複数taskで同時編集しない。
- source root内へ同一packageの複数worktreeを置かない。
- workspace一括buildを暗黙の受入条件にしない。
