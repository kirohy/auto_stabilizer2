# WBMS外部Whole-Body操縦 source-root instructions

この文書は`${CATKIN_SOURCE_ROOT}/AGENTS.md`として配置するtemplateである。

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

優先:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`
3. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`
4. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
5. 制御仕様のImplementation Plan / Revision
6. Parent Work Package / Contract
7. 必要なProgress履歴

workflowの旧gateと矛盾する場合、Workflow Revision 1を優先する。

## Cross-repository task

- `${CATKIN_SOURCE_ROOT}`からのtaskは原則read-only。
- Parent planning、branch archaeology、compatible-set review、build結果統合、simulation計画、log解析を扱う。
- source修正は対象repositoryのsub-unitへ分離する。
- 一つのimplementation実行がWRITEするrepositoryは一つ。
- 子repositoryの`AGENTS.md`が自動適用されたと仮定せず明示的に読む。

## Risk-based workflow

- `R0`: 文書、Progress、AGENTS、Skill、bootstrap、package skeleton。
- `R1`: message、IDL、enum、bridge mapping、scaffolding。
- `R2`: external generator、mapping、external IK。
- `R3`: 500 Hz、COM/ZMP、walking preparation、final IK、安全。

既定review:

- R0: SELF、Parent末尾にfocused review一回。
- R1: compatible set完成時にcross-repository review一回。
- R2: repository full review一回、修正後はtargeted review。
- R3: safety full review、material修正後full fresh review。

Progress-only変更へfull source reviewを要求しない。
repository commitごとの中央Progress commitを既定にしない。

## Repository状態

対象repositoryで確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- user変更を無断でreset、stash、checkout、clean、削除しない。
- compatible setと異なる場合、勝手に切り替えない。
- exact predecessor SHAを次sub-unitへ渡す。

## Build

workspace一括buildを標準にしない。

```sh
catkin build <package-name> --no-deps
```

dependency確認時だけ`--no-deps`を外す。
execution directory、exact command、resultを記録する。

`build`、`devel`、`install`、`logs`をsource変更として扱わない。

## Commit

- 一回に一repositoryだけstage/commitする。
- explicit pathだけstageする。
- R0〜R2はParentのstanding authorizationを使用できる。
- R3はcommitごとのexact人間承認。
- push、merge、PR作成は別許可。

## Progress

- 現在地は`CurrentCheckpoint`で管理する。
- 中央ProgressはParent完了、compatible set、simulation、milestone、引き継ぎ等のcheckpointで更新する。
- micro-stepごとのProgress-only Work Unitを作らない。

## Simulationと実機

- simulation起動、command送信、log取得はユーザーの明示許可。
- 実機は別の明示許可。
- 未実施をPASS扱いしない。

## 禁止事項

- source rootから複数repositoryを同時編集する。
- schema未確定でproducer、bridge、consumerを並行実装する。
- 同じsource fileを複数taskで編集する。
- source root内へ同一packageの複数worktreeを置く。
- R0/R1へR3 lifecycleを一律適用する。
