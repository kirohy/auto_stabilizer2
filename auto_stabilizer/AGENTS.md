# auto_stabilizer package instructions

この文書は`auto_stabilizer/`以下で作業するCodexへ適用する。repository rootの`AGENTS.md`も併せて守ること。

## 外部Whole-Body操縦作業の参照順

WBMS外部whole-body操縦に関するtaskでは、作業開始前に次をこの順で読む。

1. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. `docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
3. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
4. `docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
5. `docs/WBMSExternalWholeBodyTeleoperationProgress.md`
6. 対象Work Unit Contract
7. `docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuideRevision1.md`
8. `docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`
9. `docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`
10. `docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`
11. `docs/WBMSWalkingPreparationDesignRevisionPlan.md`
12. 必要な過去文書

古い文書や旧projection IKの実験記録を、新しい正式計画より優先してはならない。

## 複数repositoryとCodex起動directory

- `${CATKIN_WORKSPACE}`は`catkin_ws/<workspace_name>`の絶対パスを表す。
- `${CATKIN_SOURCE_ROOT}`は`${CATKIN_WORKSPACE}/src`を表す。
- cross-repository planning、branch archaeology、compatible-set reviewは`${CATKIN_SOURCE_ROOT}`からread-onlyで行う。
- implementation、repository review、finding修正、commitは対象repository rootから行う。
- 一つのimplementation taskが`WRITE`するrepositoryは原則一つだけとする。
- sibling repositoryはContractで`READ`と明記し、変更しない。
- 複数repositoryへ影響する機能はParent Work Unitとrepository sub-unitへ分割する。
- cross-repository taskでは、対象repositoryのrootとnearest `AGENTS.md`を明示的に読む。
- 他repositoryのcommit SHAを中央Progressへ同期してから、依存する次sub-unitへ進む。

## Work Unit

- source変更前に、Work Unit ID、Parent ID、Codex起動directory、branch、base SHA、repository access、scope、out-of-scope、acceptance criteria、verification commandを固定する。
- 承認済みWork Unit Contractが無い場合は実装せず、read-only調査とContract案の作成までに留める。
- 一つのWork Unitには一つの主要責務だけを含める。
- protocol、producer、bridge、consumer、安全処理、diagnostics、cleanupを不用意に一つのcommitへ混在させない。
- 複数repositoryを変更する場合、repositoryごとにatomic commitを作り、compatible SHAをProgressへ記録する。

## 500 Hz経路の不変条件

- `auto_stabilizer::onExecute()`へROS callback、network I/O、blocking I/O、condition variable待ち、外部process待ちを追加しない。
- 500 Hz経路で毎周期threadを生成しない。
- 500 Hz経路へ無制限なqueue、不要なclone、全探索、反復回数の無制限増加、意図しない動的確保を追加しない。
- external generatorまたはbridgeが停止しても500 Hz周期を待たせない。
- 新アーキテクチャの実装branchへprojection IKを復活させない。projectionを必要と判断した場合は計画変更として扱う。

## 安全とmode ownership

- joint position/velocity limit、self collision、足拘束、接触、歩行安定化、ZMP/COM安全をoperator taskより優先する。
- static両足支持時だけoperator CHEST/COMを有効にする。
- walking preparationおよび歩行中はoperator CHEST/COMを無効化し、腕と頭部だけを継続する。
- M4.2.2のwalking preparation、READY、歩行API gate、COM高さ保持、timeout/failureを維持する。
- stale、invalid、solver failure時は現在のaccepted/generated stateをholdし、未実現commandを後で再生しない。
- hidden goalを作らない。次の局所targetは最新accepted generated stateを基準にする。
- `q_nominal`は低優先度の非権威的referenceであり、hardwareへ直接出力しない。

## non-IK関節reference

- 首と将来の指関節は、明示的allowlistに含まれる`jointControllable=false`関節だけを対象にする。
- direct referenceは`qRef`読込後、`refRobotRaw`のFK/COM計算前に適用する。
- 上流`qRef`を`refRobotRaw`への上書きで失わないこと。最新上流q/dqを専用bufferへ保持し、各周期にその値を復元してからdirect overrideを適用する。
- direct referenceにはfinite、joint position、velocity、acceleration、stale、start/stop blendを適用する。
- final IK後やOutPort書込み直前だけで関節角を上書きし、内部robot modelと出力を不一致にしてはならない。

## interface

- ROS message、RTM IDL、bridge mappingについて、field、単位、frame、quaternion順序、task mask、enum、schema versionをParent Contractとrepository sub-unitで照合する。
- source選択は明示的に行い、last-writer-winsを使わない。
- external sourceを一つでも選択している場合だけgenerator/bridge heartbeatをWBMS開始条件にする。全taskがlegacy/upstream sourceの場合はexternal heartbeatを要求しない。
- session、epoch、sequence、out-of-order、stale、schema mismatchを明示的に処理する。

## 実装と検証

- `clang-format`を実行しない。既存styleに合わせる。
- コメントとMarkdownは日本語で記述する。
- このrepositoryでは新規unit test作成を必須にしない。計画書で指定されたbuild、static check、simulation、log確認を実行する。
- workspace一括buildを標準にしない。
- 通常は対象packageだけを`catkin build <package> --no-deps`でbuildする。
- 依存関係まで確認する場合だけ`--no-deps`を外し、その目的をContractとProgressへ記録する。
- IDL変更後の`auto_stabilizer`初回buildは`catkin build auto_stabilizer --no-deps --force-cmake`を使用する。
- それ以外は`catkin build auto_stabilizer --no-deps`を使用する。
- `catkin build`の実行directoryは固定しないが、exact commandと実行directoryをProgressへ記録する。
- 実行できなかったsimulation、実機確認をPASS扱いせず`UNVERIFIED`と記録する。

## review、Progress、commit

- 実装taskとは別のread-only reviewを行う。
- P0/P1/P2 findingがなくなるまで、修正後の最新diff全体をfresh reviewする。
- styleだけの指摘、legacy portを残すこと、clang-format未適用、新規unit test未追加、初回にsingle-QP final WBCを実装しないことを、それ自体ではblocking findingにしない。
- commit前に`docs/WBMSExternalWholeBodyTeleoperationProgress.md`へ、repository/branch/SHA、判断、変更、exact command、実行directory、結果、review、未確認事項、compatible dependency set、next entry pointを追記する。
- 他repository commit後は、依存する次sub-unit前に中央ProgressへSHAを同期する。
- commitはユーザーが明示的に指示した場合だけ行う。push、merge、PR作成、実機実行は別の明示指示が必要である。
