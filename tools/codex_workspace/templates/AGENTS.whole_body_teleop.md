# whole_body_teleop repository instructions

この文書は`whole_body_teleop` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

含む:

- `whole_body_teleop_msgs`。
- 標準ROS input、session baseline。
- 左右手差分、position scale。
- CHEST/COM局所target。
- HMD、head recenter。
- external whole-body IK。
- realized reference、`q_nominal`、direct joint reference。
- command/state bundle、heartbeat、diagnostics。

含まない:

- RobotHardwareへの直接指令。
- 500 Hz final safety guarantee。
- walking preparation。
- final joint/collision/foot validation。
- ZMP、`genCog`、`sbpOffset`、`refdz`、`omega`、`l`の最終更新。
- ROS/RTM bridge。

## 正式仕様

Project Contextに固定された中央文書を読む。

1. Implementation Plan Revision 2。
2. Codex Workflow Revision 1。
3. Current Checkpoint。
4. MultiRepositoryOperations。
5. 制御仕様のImplementation Plan / Revision。
6. Parent Work Package / Contract。
7. 必要なProgress履歴。

workflowの旧gateと矛盾する場合、Workflow Revision 1を優先する。

## Repository境界

- Codexはrepository rootから起動する。
- 一つのimplementation実行がWRITEするrepositoryは本repositoryだけ。
- sibling repositoryは明示SHAをREADする。
- schema意味変更時はParent planへ戻る。
- unrelated cleanupを混ぜない。
- user変更を無断でreset、stash、checkout、cleanしない。

## Risk

本repositoryの典型:

- R0: package skeleton、AGENTS、Project Context。
- R1: ROS message、enum、schema、launch skeleton。
- R2: reference generator、mapping、external IK、state machine。
- R3: 原則なし。最終安全責務を本repositoryへ移さない。

Review:

- R0: SELF、bootstrap package末尾にfocused review。
- R1: compatible set完成時にcross-repository review。
- R2: repository full review一回。非本質修正はtargeted follow-up。

Progress-only reviewを要求しない。
repository commitごとの中央Progress commitを既定にしない。

## ROS callbackとsolver loop

- 重いIKをsubscriber callback内で解かない。
- callbackはlatest snapshot更新に限定。
- main/timer loopがatomic snapshotを読みsolverを実行。
- subscriber queueは原則1。
- backlogを処理し続けない。
- group別timestamp、enabled、valid、staleを明示する。

## Reference生成

- 次の局所targetは最新accepted generated state基準。
- hidden goalを作らない。
- stale中commandを復帰後に再生しない。
- raw targetではなくrealized hand/CHEST/COMと`q_nominal`をpublish。
- failure/unsafe時は前回safe/accepted stateをhold。

## Coordinate frame

- hand input: device world master pose、session差分。
- hand output: CHEST相対。
- CHEST angular velocity: current foot-mid。
- COM: current foot-midのrobot raw COM。
- HMD position無視、orientation差分からyaw/pitch。
- frame、unit、quaternion順序をParent Contractへ明記。

## External IK

- `ik_solvers2`、`prioritized_qp`の明示API。
- joint position/velocity、feet、support、task priority。
- self collisionは計画段階に従う。
- external IKを最終安全保証とみなさない。
- safety > hand position > hand orientation > CHEST > operator COM > posture。
- direct non-IK jointをIK変数へ入れない。

## Message/schema

- Parent Contractと一致。
- schema version、mask、status、rejection reason、session、epoch、sequence。
- unknown/duplicate joint、session中name変更をreject可能にする。
- source timestampをpublish時刻で置換しない。

## Build

```sh
catkin build whole_body_teleop_msgs --no-deps
catkin build whole_body_teleop_reference_generator --no-deps
```

dependency確認時だけ`--no-deps`を外す。
exact command、execution directory、resultを記録する。

## Commitとcheckpoint

- R0〜R2はParentのstanding authorizationを使用できる。
- 一回に本repositoryだけstage/commitする。
- explicit pathだけstage。
- compatible set review前にcommit SHAをParent working stateへ渡す。
- 中央ProgressはParent/compatible-set checkpointで更新する。
- push、merge、PR、simulation、実機は別許可。

## Code style

- C++。
- 小さなclass、明示state machine。
- RAII/値所有を優先。
- magic numberをparameter/定数へ。
- `clang-format`を自動適用しない。
- コメント/Markdownは日本語。
- identifiersは英語。
