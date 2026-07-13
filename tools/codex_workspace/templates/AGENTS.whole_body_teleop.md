# whole_body_teleop repository instructions

この文書は新規`whole_body_teleop` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

本repositoryは、WBMS外部whole-body操縦のROS側を担当する。

含む責務:

- `whole_body_teleop_msgs`。
- 標準ROS inputの受信。
- session開始時baseline。
- 左右手差分とposition scale。
- CHEST/COM速度から局所target生成。
- HMD姿勢差分とhead recenter。
- external whole-body IK。
- realized task-space reference。
- `q_nominal`とdirect joint reference。
- command/state custom bundle。
- heartbeat、diagnostics。

含まない責務:

- RobotHardwareへの直接関節指令。
- 500 Hz final safety guarantee。
- walking preparation。
- final joint/collision/foot validation。
- ZMP、`genCog`、`sbpOffset`、`refdz`、`omega`、`l`の最終更新。
- ROS/RTM bridge。

## 正式仕様

`docs/WBMSExternalTeleopProjectContext.md`に記録された固定SHAの中央文書を読む。

優先:

1. Implementation Plan Revision 2。
2. MultiRepositoryOperations。
3. Implementation Plan Revision 1。
4. Implementation Plan。
5. 中央Progress。
6. Parent/Sub-unit Contract。

正式計画全文を本repositoryへ複製しない。

## Work Unit

- Codexは本repository rootから起動する。
- 一つのimplementation taskがWRITEするrepositoryは本repositoryだけとする。
- sibling repositoryはContractでREADとされた範囲だけ参照する。
- Parent Contractでschema、frame、unit、enum、session semanticsを固定してから実装する。
- schema変更が必要になった場合、bridgeやconsumerへ暗黙に合わせずParent planへ戻る。

## ROS callbackとsolver loop

- 重いwhole-body IKをsubscriber callback内で直接解かない。
- callbackはlatest input snapshotの更新に限定する。
- explicit main loopまたはtimer loopがatomic snapshotを読み、solverを実行する。
- subscriber queueは原則1。
- 古いqueueを順番に処理し続けない。
- input group別timestamp、enabled、valid、staleを明示する。

## Reference生成

- 次の局所targetは最新accepted generated stateを基準にする。
- 未実現operator commandをhidden goalとして蓄積しない。
- stale中のcommandを復帰後に再生しない。
- raw operator targetではなく、solverで実現されたhand/CHEST/COMと`q_nominal`をpublishする。
- solver failureまたはunsafe candidate時は前回safe/accepted stateをholdする。

## Coordinate frame

- hand inputはdevice worldのmaster poseを受け、session baselineとの差分を用いる。
- hand outputはCHEST相対。
- CHEST angular velocityはcurrent foot-mid frame。
- COM velocityとpositionはcurrent foot-mid frameのrobot raw COM。
- HMD positionは無視し、orientation差分からyaw/pitchを生成する。
- frame、単位、quaternion順序をParent Contractとmessage commentへ明記する。

## External IK

- `ik_solvers2`、`prioritized_qp`の明示APIを使用する。
- joint position/velocity、feet、support、task priorityを扱う。
- self collisionは計画の段階に従って追加する。
- external IKは最終安全保証ではない。
- task priorityは安全系 > hand position > hand orientation > CHEST > operator COM > postureを維持する。
- head/finger等のdirect non-IK jointをexternal IK変数へ入れない。

## Message/schema

- message、IDL、bridge mappingはParent Contractと一致させる。
- `schema_version`、task mask、status、rejection reason、session、epoch、sequenceを明示する。
- `JointState.name`のunknown、duplicate、session途中変更をreject可能にする。
- source timestampをpublish時刻で置換しない。

## Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build whole_body_teleop_msgs --no-deps
catkin build whole_body_teleop_reference_generator --no-deps
```

依存関係まで確認する場合だけ`--no-deps`を外す。

message/CMake再生成が必要な場合はContractで`--force-cmake`を指定する。

exact commandと実行directoryをProgress/Work Unit reportへ記録する。

## Code style

- C++で実装する。
- 小さなclassに責務を分離する。
- state machineを明示する。
- RAIIと値所有を優先する。
- magic numberをparameterまたは名前付き定数にする。
- `clang-format`を自動適用せず、repositoryで確定したstyleに従う。
- コメントとMarkdownは日本語。
- class、function、variable、topic、message fieldは英語。

## Review、Progress、commit

- 実装taskとは別のread-only reviewを行う。
- P0/P1/P2がなくなるまでfresh reviewする。
- package build結果とcompatible input SHAを記録する。
- commit後、依存する次sub-unit前に中央ProgressへSHAを同期する。
- commitはユーザーの明示許可時だけ。
- push、merge、PR作成、simulation、実機実行は別許可。
