# auto_stabilizer package instructions

この文書は`auto_stabilizer/`以下で作業するCodexへ適用する。
repository rootの`AGENTS.md`も併せて守ること。

## 正式文書の参照順

WBMS外部whole-body操縦に関するtaskでは、作業開始前に次を読む。

1. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `docs/WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`
3. `docs/WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`
4. `docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision2.md`
5. `docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
6. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
7. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
8. 対象Parent Work Package / Work Unit Contract
9. `docs/WBMSExternalWholeBodyTeleoperationProgress.md`
10. 必要な既存正式文書

workflow、review頻度、Progress頻度、commit gateについて旧文書と矛盾する場合、
`CodexWorkflowRevision1`を優先する。

古いprojection IK文書を新しい正式計画より優先しない。

## Workspaceとrepository

- `${CATKIN_WORKSPACE}`は`catkin_ws/<workspace_name>`の絶対パス。
- `${CATKIN_SOURCE_ROOT}`は`${CATKIN_WORKSPACE}/src`。
- cross-repository planning/reviewは`${CATKIN_SOURCE_ROOT}`から原則read-onlyで行う。
- implementation、repository review、commitは対象repository rootから行う。
- 一つのimplementation実行がWRITEするrepositoryは一つだけとする。
- sibling repositoryは明示されたSHAをREADするだけとする。
- 複数repositoryへ影響する機能はParent Work Packageとrepository sub-unitへ分割する。
- user変更を無断でreset、stash、checkout、clean、削除しない。

## Risk level

作業開始時に最高riskを選ぶ。

- `R0`: 文書、Progress、AGENTS、Skill、bootstrap、package skeleton。
- `R1`: message、IDL、enum、task mask、bridge mapping、scaffolding。
- `R2`: 非リアルタイムexternal generator、mapping、external IK。
- `R3`: 500 Hz、COM/ZMP、walking preparation、final IK、安全constraint、最終出力。

R0/R1へR3のfull lifecycleを一律適用しない。

- R0はSELF確認し、Parent Work Package末尾でfocused reviewを一回。
- R1はcompatible set完成時にcross-repository reviewを一回。
- R2はrepository full reviewを一回。非本質修正はtargeted follow-up。
- R3はsafety full reviewとmaterial修正後のfull fresh review。

Progress-only変更はcontrol source reviewを無効化しない。

## Work Package

- R0では短いWork Briefを使用してよい。
- R1〜R3は影響に応じたContractを作る。
- Parent Work Packageの承認範囲内なら、複数のeligible sub-unitを順次進めてよい。
- 各sub-unitのWRITE repositoryは一つ。
- micro-stepごとに中央Progress entryを作らない。
- compatible set、simulation、milestone、引き継ぎ前にcheckpointを作る。

## 500 Hz経路の不変条件

- `auto_stabilizer::onExecute()`へROS callback、network I/O、blocking I/O、外部process待ちを追加しない。
- 500 Hz経路で毎周期threadを生成しない。
- 無制限queue、不要clone、全探索、無制限反復、不要な動的確保を追加しない。
- external generatorまたはbridge停止時にも500 Hz周期を待たせない。
- projection IKを復活させる場合は計画変更として扱う。

## 安全とmode ownership

- joint position/velocity limit、self collision、足拘束、接触、歩行安定化、ZMP/COM安全をoperator taskより優先する。
- static両足支持時だけoperator CHEST/COMを有効にする。
- walking preparationおよび歩行中はoperator CHEST/COMを無効化し、腕と頭部を継続する。
- M4.2.2のwalking preparation、READY、walking API gate、COM高さ保持、timeout/failureを維持する。
- stale、invalid、solver failure時はcurrent accepted/generated stateをholdする。
- hidden goalを作らない。
- `q_nominal`をhardwareへ直接出力しない。

## non-IK関節reference

- 首と将来の指関節はallowlistに含まれる`jointControllable=false`関節だけを対象にする。
- direct referenceは`qRef`読込後、`refRobotRaw`のFK/COM計算前に適用する。
- 上流q/dqを各周期に復元してからoverrideする。
- finite、position、velocity、acceleration、stale、start/stop blendを適用する。
- final IK後だけ上書きして内部modelと出力を不一致にしない。

## Interface

- ROS message、RTM IDL、bridge mappingのfield、type、unit、frame、quaternion順序、task mask、enum、schema versionをParent Contractで固定する。
- source選択は明示し、last-writer-winsを使わない。
- session、epoch、sequence、out-of-order、stale、schema mismatchを処理する。
- schema意味変更時はParent planへ戻る。

## Build

- workspace一括buildを標準にしない。
- 通常は`catkin build <package> --no-deps`。
- dependency確認時だけ`--no-deps`を外し、目的を記録する。
- IDL変更後の`auto_stabilizer`初回は`catkin build auto_stabilizer --no-deps --force-cmake`。
- 実行directory、exact command、resultをverification evidenceへ記録する。
- simulation/実機未実施を`PASS`扱いしない。

## Review evidence

review/build対象をSHAまたはdiff hashで識別する。
対象source diffが変わっていなければ、Progress追記後にsource reviewやbuildをやり直さない。

full fresh reviewが必要なのは、P0/P1、materialなP2、schema意味、state machine、
safety constraint、threading、solver semantics、simulation/実機前など、
`CodexWorkflowRevision1`に定義された場合だけとする。

## Progressとcommit

- 現在地は`CurrentCheckpoint`で管理する。
- 中央ProgressはParent Work Package、compatible set、simulation、milestone、引き継ぎ等のcheckpointで更新する。
- repository commitごとのProgress-only sub-unitを通常作らない。
- R0〜R2はユーザーのstanding authorizationがあればParent scope内のlocal commitを順次作成してよい。
- R3はcommitごとのexact人間承認を維持する。
- commit前はstage対象、`git diff --cached --check`、stat、name-onlyを確認する。
- push、merge、PR作成、simulation、実機実行は別の明示許可を必要とする。

## Code style

- `clang-format`を実行しない。
- 既存styleに合わせる。
- コメントとMarkdownは日本語。
- class、function、variable、topic、message fieldは英語。
- 不要に広範囲なrewriteを行わない。
