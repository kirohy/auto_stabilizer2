# WBMS外部Whole-Body操縦Reference Generator 実装計画

## 1. 文書の位置づけ

本書は、`auto_stabilizer2`におけるWBMS操縦機能を、500 Hzの`hrpEC`クリティカルパス内で毎周期projection IKを解く構成から、以下の責務分離へ移行するための正式な実装計画である。

```text
外部ROS node、50〜100 Hz
  task-space whole-body reference生成
  腕、CHEST、COM、頭部、q_nominal
        ↓
独立rtmros bridge process
        ↓
auto_stabilizer、500 Hz
  reference検証・補間・mode gate
  COM/ZMP統合
  walking preparation
  final whole-body IK
  安全validation
```

本書は新アーキテクチャに関する最新の仕様判断として扱う。

参照順は次のとおりとする。

1. 本書: 外部whole-body操縦reference generator、bridge、auto_stabilizer統合の正式仕様。
2. `WBMSExternalWholeBodyTeleoperationProgress.md`: 本計画の実施履歴、ログ、受入結果。
3. `WBMSWalkingPreparationDesignRevisionPlan.md`: walking preparation、READY、歩行API gate、COM高さ保持の既存正式仕様。
4. `WBMSFeasibleVelocityPostureControlProgress.md`: 旧projection IK構成の作業履歴と実測結果。
5. `WBMSComputationReductionImplementationPlan.md`: 旧projection IK構成に対するM5計算量削減の履歴。
6. その他の過去文書: 経緯と不採用理由の確認にのみ使用する。

本書とM5計画が矛盾する場合、新アーキテクチャについては本書を優先する。M5計画の「projection IKを間引かない」という判断は、旧二段IK構成を維持する場合の判断であり、本書ではprojection IK自体を500 Hz経路から削除して安全責務を再定義する。

---

## 2. 確定した設計判断

### 2.1 主要方針

- 重いwhole-body reference生成は、実機制御PC上の独立ROS processで実行する。
- 外部node、bridgeは500 Hzの`hrpEC`へ参加させない。
- `auto_stabilizer`内のprojection IKは削除する。
- 初回はpre-M5のfinal IK構造を維持する。
- `task scaling`を内包したsingle-QP final WBCは初回範囲外とし、必要になった場合の後続候補とする。
- M4.2.2のwalking preparation、歩行API gate、COM高さ保持、歩行中腕継続は維持する。
- 操作者がCHESTとCOMを明示的に速度操作する。
- 左右手は外骨格等からの絶対`PoseStamped`入力を、操縦開始時差分へ変換して使用する。
- 頭部はHMDの姿勢差分から首yaw/pitchを生成する。
- `q_nominal`はハードな関節指令ではなく、final IKの非権威的な低優先度referenceとして使用する。
- 首、将来の指関節はfinal IK変数にせず、安全な非IK関節reference override経路で扱う。
- legacyの個別EE portと`refTorsoVelIn`は初回実装では残す。
- legacy整理は機能完成後の独立commitで行う。

### 2.2 操作可能条件

- CHEST操作: staticかつ両足支持時のみ有効。
- COM操作: staticかつ両足支持時のみ有効。
- 腕操作: static、walking preparation、歩行中に有効。
- 頭部操作: static、walking preparation、歩行中に有効。
- 歩行中は歩行安定性を最優先し、CHESTとCOMのoperator taskを無効化する。
- 将来の歩行中CHEST/COM操作を追加できるようtask maskとmode policyを分離するが、初回受入要件にはしない。

### 2.3 優先順位

安全系を含む全体優先順位は次とする。

```text
joint position / velocity limit
self collision
足拘束、接触、歩行安定化、ZMP・COM安全
    >
左右手位置
    >
左右手姿勢
    >
CHEST姿勢
    >
操作者指定COM
    >
q_nominal
```

左右腕は初回は独立taskとする。rigid bimanual couplingは後続範囲とする。

---

## 3. 目的

### 3.1 必須目的

1. 500 Hzの`auto_stabilizer::onExecute()`からprojection IKのQP solveを除去する。
2. 左右手、CHEST、COMを外部ROS nodeで整合させたreferenceとして生成する。
3. static両足支持中に、体幹前傾、COM移動、腕操作を同時に行えるようにする。
4. 床付近へ手先を到達させられる大きな前傾を維持する。
5. 大きく前傾した姿勢からwalking preparationを経て歩行可能姿勢へ安全に復帰する。
6. 歩行中も腕と頭部の操縦を継続する。
7. 外部node、bridgeの停止、通信途絶、stale時に残留動作を作らない。
8. 未実現operator commandをhidden goalとして蓄積しない。
9. legacy interfaceを初回は維持する。
10. 新しい処理を追加しても、実機で安定していたpre-M5 final IKの500 Hz実行性を大きく損なわない。

### 3.2 非目的

初回実装では以下を扱わない。

- single-QP final WBCへの統合。
- 歩行中のCHEST・COM操作。
- rigid bimanual coupling。
- 物体の把持、持上げ、payloadモデル。
- 学習ベース制御。
- 指関節の実運用。
- legacy port削除。
- 外部nodeを別PCへ配置する正式対応。
- external IKによる最終安全保証。

---

## 4. 対象リポジトリとbranch

### 4.1 `auto_stabilizer2`

- repository: `kirohy/auto_stabilizer2`
- 実装branchの基点: M4.2.2の全修正完了後、最初のM5変更前のcommit。
- exact SHAはMilestone 0で履歴、コード、文書を照合して決定する。
- 初回はpre-M5 final IK構造を維持する。
- `1.0`との計算時間比較は統合完了後に行う。

### 4.2 `whole_body_teleop`

新規repositoryとして作成する。

想定package:

```text
whole_body_teleop_msgs
whole_body_teleop_reference_generator
```

責務:

- 標準ROS message入力。
- generated state feedback受信。
- operator inputのbaseline、差分、scale処理。
- 低周期whole-body IK。
- custom atomic command bundle生成。
- diagnostics、heartbeat、recenter service。

### 4.3 `rtmros_msg_bridge`

- repository: `kirohy/rtmros_msg_bridge`
- 新しい操縦関連bridge packageを追加する。
- bridgeは独立processで動かし、`hrpEC`に参加させない。
- IK、limit、interpolation、mode判断をbridgeへ入れない。

### 4.4 `ik_solvers2`、`prioritized_qp`

- external whole-body IKで既存solverを再利用する。
- 初期は`teleop-dev` branchを基準とする。
- solver側変更が不要なら新規変更を行わない。
- 変更が必要な場合は、external generator専用APIを追加し、auto_stabilizer既存挙動を暗黙に変更しない。

### 4.5 branch運用

計画文書は`wbms-external-teleop-plan`に置く。

実装開始時は次を行う。

1. Milestone 0でpre-M5 base SHAを確定する。
2. そのSHAから実装branchを作る。
3. 本計画、Codex workflow、Progress、Skill、必要な`AGENTS.md`変更だけをcherry-pickする。
4. 現在のM5実装コードを実装branchへ混入させない。
5. 後続の有用な診断・軽量化変更は、一件ずつ意図を確認して選択的に移植する。

---

## 5. システム構成

```text
入力device node群、今回の主実装範囲外
  right hand PoseStamped
  left hand PoseStamped
  CHEST TwistStamped
  COM velocity
  HMD PoseStamped
  task enable / heartbeat
        |
        v
whole_body_teleop_reference_generator、50〜100 Hz
  - session同期
  - hand/head baseline
  - hand差分scale
  - CHEST/COM速度の局所target生成
  - whole-body IK
  - realized task-space reference
  - q_nominal
  - direct joint reference
        |
        v
WholeBodyTeleopReference、ROS custom message
        |
        v
WholeBodyTeleopROSBridge、独立process
        |
        v
TimedWholeBodyTeleopReference、RTM IDL
        |
        v
AutoStabilizer、500 Hz
  -受信検証
  - source選択
  - task別stale
  - 500 Hz補間・limit
  - non-IK joint override
  - COM/ZMP/refdz統合
  - walking preparation
  - stabilizer
  - final IK
  - final validation
        |
        v
RobotHardware

AutoStabilizer generated state
        |
        v
TimedWholeBodyTeleopState
        |
        v
bridge
        |
        v
WholeBodyTeleopState
        |
        v
external generator
```

---

## 6. 用語とtask group

### 6.1 task group

```text
RIGHT_HAND
LEFT_HAND
CHEST
COM
HEAD
```

各groupは独立したsource timestamp、enabled、valid、stale状態を持つ。

### 6.2 heartbeat

- generator heartbeat: external generator processが正常に動作していることを示す。
- bridge heartbeat: ROS/RTM変換processが正常に動作していることを示す。
- 個別task heartbeat: 値が変化しなくても入力nodeがfreshであることを示す。

### 6.3 session

`startWholeBodyMasterSlave()`による一回の操縦開始から停止までを一つのsessionとする。

各sessionは次を持つ。

- `session_id`
- `epoch`
- command `sequence`
- state `sequence`
- last received / accepted sequence

external node再起動時はepochを変更する。同一sessionの一時通信断とprocess再起動を区別する。

### 6.4 accepted state

external generatorが次の局所targetを作る基準は、raw operator targetではなく、auto_stabilizerから返された最新のaccepted generated stateとする。

---

## 7. 実行周期とthread境界

### 7.1 external generator

- 初期目標周期: 100 Hz。
- 50 Hzまでparameterで下げられるようにする。
- HMD入力は約60 Hzを想定する。
- subscriber queueは原則1。
- 重いIKはROS callback内で直接解かず、明示的なmain loopまたはtimer callbackで最新snapshotを用いて解く。
- 同一周期中のcommandとstateはatomic snapshotとして扱う。

### 7.2 bridge

- 独立process。
- `hrpEC`へ参加しない。
- ROS callbackとRTM onExecute間はlatest-value mailboxで分離する。
- bridge側は古いqueueを処理し続けず、常に最新値を転送する。

### 7.3 auto_stabilizer

- 500 Hzの既存直列実行を維持する。
- external generatorまたはbridgeを待たない。
- mutex待ち、condition variable待ち、network I/O、ROS callbackを500 Hz経路へ入れない。
- 受信済みの最新RTM dataだけを読む。
- 500 Hz経路で毎周期threadを生成しない。

---

## 8. 標準ROS入力interface

### 8.1 左右手

```text
/right_hand_input  geometry_msgs/PoseStamped
/left_hand_input   geometry_msgs/PoseStamped
```

入力はdevice world frameにおけるmaster poseでよい。external generatorがWBMS session開始時、または最初のfresh sample時にbaselineを保存する。

### 8.2 CHEST

```text
/chest_velocity_input  geometry_msgs/TwistStamped
```

使用成分:

- `angular.x`: foot-mid roll軸角速度。
- `angular.y`: foot-mid pitch軸角速度。
- `angular.z`: foot-mid yaw軸角速度。
- linear成分は無視する。

### 8.3 COM

初期候補:

```text
/com_velocity_input  geometry_msgs/Vector3Stamped
```

- x/y/zはcurrent foot-mid座標系のCOM速度。
- 対象はrobot raw COM。
- `genCog`との変換はauto_stabilizer側で`sbpOffset`を用いて行う。

### 8.4 HMD

```text
/head_pose_input  geometry_msgs/PoseStamped
```

- positionは無視。
- orientationのみ使用。
- source PCのworld frame基準。
- session開始時のorientationとの差分を使う。

### 8.5 enableとheartbeat

各task groupは定周期publishを必須とする。値が変化しなくてもpublishする。

追加で以下に相当する入力を定義する。

```text
/task_enable
/input_heartbeat
```

型はMilestone 1で、標準messageを優先して確定する。

---

## 9. 左右手差分操縦

### 9.1 baseline

session開始時、または各手の最初のfresh input時に以下を保存する。

```text
masterStartPose[hand]
slaveStartPoseInChest[hand]
```

### 9.2 差分

```text
masterDelta = masterStartPose.inverse() * masterCurrentPose
```

### 9.3 scale

並進差分だけをscaleする。

```text
scaledDelta.translation = handPositionScale.cwiseProduct(masterDelta.translation)
scaledDelta.rotation = masterDelta.rotation
```

parameter:

```yaml
right_hand_position_scale: [1.0, 1.0, 1.0]
left_hand_position_scale:  [1.0, 1.0, 1.0]
```

初期運用では各軸同値を使用してもよい。

回転差分gainは初回1.0固定とする。必要になった場合だけparameter化する。

### 9.4 target

```text
handTargetInChest = slaveStartPoseInChest * scaledDelta
```

出力referenceはCHEST相対とする。

### 9.5 stale

片手だけstaleの場合、その手の現在generated CHEST相対poseをholdし、他taskは継続する。

### 9.6 recenter

初回はsession開始時自動baselineのみとする。明示的なhand recenter serviceは後続候補とする。

---

## 10. CHEST速度操作

### 10.1 座標系

current foot-mid座標系のroll/pitch/yaw軸を使用する。

### 10.2 target生成

external generatorはaccepted generated CHEST姿勢から一周期先の局所targetを作る。

```text
acceptedChestInFootMid
  + limitedAngularVelocity * generatorDt
  -> localChestTarget
```

### 10.3 limit

- angular velocity limit。
- angular acceleration limit。
- WBMS開始baselineからのroll/pitch/yaw範囲limit。
- parameterはexternal側とauto_stabilizer側で意味を揃える。

### 10.4 anti-windup

final IKで実現できなかった姿勢差を次周期へ積み上げない。

次周期はauto_stabilizerから返ったaccepted generated CHEST姿勢から再計算する。

### 10.5 mode gate

- static両足支持時のみoperator velocityを受け付ける。
- walking preparation開始時はvelocityをゼロへ減速する。
- walking preparationと歩行中はexternal CHEST taskを無効化する。

---

## 11. COM速度操作

### 11.1 座標系

current foot-mid座標系のx/y/z軸を使用する。

### 11.2 target生成

```text
acceptedRobotComInFootMid
  + limitedComVelocity * generatorDt
  -> localRobotComTarget
```

### 11.3 limit

- COM velocity limit。
- COM acceleration limit。
- WBMS開始baselineからのoffset limit。
- shrunken support hull。

### 11.4 safety ownership

external generatorでもCOM targetを支持領域へ制限する。

最終責務はauto_stabilizerに残す。

- support hull clamp。
- `genCog`、`genCogVel`、`genCogAcc`。
- `refdz`。
- `l`。
- `omega`。
- ZMP導出と`refZmpTraj`更新。

### 11.5 mode gate

- static両足支持時のみoperator COM velocityを有効にする。
- walking preparationでは既存RETURNING/HANDOFF controllerがownershipを持つ。
- walking中はoperator COM taskを無効化する。
- walking preparation開始時のCOM Z snapshotと歩行中height holdを維持する。

---

## 12. 頭部操縦

### 12.1 baseline

session開始時、または最初のfresh HMD sample時に保存する。

```text
hmdStartRotation
neckYawStart
neckPitchStart
```

### 12.2 相対回転

```text
hmdDelta = hmdStartRotation.transpose() * hmdCurrentRotation
robotDelta = deviceToRobotRotation * hmdDelta * deviceToRobotRotation.transpose()
```

### 12.3 2軸への変換

- rollは無視。
- yawを首yawへ反映。
- pitchを首pitchへ反映。
- yaw wrapを連続化する。
- gain、sign、固定軸変換はparameter化する。

### 12.4 direct non-IK reference

首関節はfinal IK変数にしない。

外部bundleでは名前付きdirect joint referenceとして送る。

`auto_stabilizer`では次の順序で適用する。

```text
qRef InPort read
  -> refRobotRawへ上流qRefを設定
  -> NonIkJointReferenceControllerが許可された首角をoverride
  -> refRobotRaw FK / COM
  -> frame conversion
  -> final IK
  -> 既存output path
```

これにより`refRobotRaw`、`refRobot`、`genRobot`、出力関節角のモデル整合を維持する。

### 12.5 limit

- joint position limit。
- joint velocity limit。
- joint acceleration limit。
- 500 Hz補間。

### 12.6 有効phase

- static WBMS: 有効。
- walking preparation: 有効。
- walking: 有効。
- WBMS停止: 無効。
- HMD stale: 現在首角hold。

### 12.7 recenter

external nodeへ以下を追加する。

```text
std_srvs/Trigger recenter_head
```

recenter時は現在HMD姿勢と現在generated首角を新baselineとして保存する。首をneutralへ急に戻さない。

### 12.8 WBMS停止

`stopWholeBodyMasterSlave()`時は、その時点のdirect首角から上流`qRef`の首角へ滑らかに復帰する。

### 12.9 将来の指関節

同じnon-IK reference override機構を使用できるよう、許可関節名をparameter化する。

初回は首yaw/pitchだけを許可する。

---

## 13. external whole-body IK

### 13.1 基本方式

`ik_solvers2`、`prioritized_qp`を用いたconstrained differential whole-body IK / hierarchical QPを採用する。

学習ベース手法は使用しない。

### 13.2 初期状態

最新のaccepted generated stateを使用する。

actual stateは追従誤差と安全診断に使用するが、operator reference生成の主初期状態にはしない。

### 13.3 変数

- root free joint。
- external whole-body IK対象関節。
- 首・指等のdirect non-IK関節は除外する。

### 13.4 constraintとtask

初期構成:

```text
priority 0:
  joint velocity
  joint position / joint limit table

priority 1:
  self collision
  Milestone 10で有効化

priority 2:
  両足pose

priority 3:
  右手位置
  左手位置

priority 4:
  右手姿勢
  左手姿勢

priority 5:
  CHEST姿勢

priority 6:
  robot COM

priority 7:
  posture regularization
```

実際のpriority層数は計算量とsolver仕様に合わせて統合してよいが、意味上の優先関係は維持する。

### 13.5 precision初期値

```text
hand position: 0.01 m
hand orientation: 0.10 rad
```

診断warning候補:

```text
hand position: 0.03 m
hand orientation: 0.26 rad
```

actual robotの統計的1 cm保証は初期hard acceptanceにしない。

### 13.6 iteration

- warm startを使用する。
- 初期は1 iterationまたは少数iterationをparameter化する。
- 外部nodeは500 Hz deadline外だが、60〜100 Hzの入力応答を損なう長時間solveは避ける。
- solve時間、iteration、statusを出力する。

### 13.7 safe candidate

external solverの「全task satisfied」だけでpublish可否を決めない。

少なくとも以下を検証する。

- finite。
- joint limit。
- joint one-cycle step。
- root one-cycle step。
- foot pose error。
- support hull。
- collision、Milestone 10以降。

unsafeなら新referenceをpublishせず、直前accepted state基準のholdを維持する。

### 13.8 出力

raw operator targetではなく、solver結果から再計算した以下を出力する。

- realized right/left hand pose in CHEST。
- realized CHEST pose in foot-mid。
- realized robot COM in foot-mid。
- realized task velocity。
- `q_nominal`。
- solver statusとvalidation metrics。

---

## 14. custom ROS command bundle

Milestone 1で正式な`.msg`を確定する。概念構造は次とする。

```text
WholeBodyTeleopReference
  std_msgs/Header header
  uint32 schema_version
  uint64 session_id
  uint64 epoch
  uint64 sequence
  duration valid_duration

  uint32 enabled_task_mask
  uint32 valid_task_mask

  time right_hand_source_stamp
  time left_hand_source_stamp
  time chest_source_stamp
  time com_source_stamp
  time head_source_stamp

  geometry_msgs/Pose right_hand_pose_in_chest
  geometry_msgs/Pose left_hand_pose_in_chest

  geometry_msgs/Pose chest_pose_in_foot_mid
  geometry_msgs/Twist chest_twist_in_foot_mid

  geometry_msgs/Point com_position_in_foot_mid
  geometry_msgs/Vector3 com_velocity_in_foot_mid

  sensor_msgs/JointState q_nominal
  sensor_msgs/JointState direct_joint_reference

  uint8 solver_status
  diagnostics fields
```

### 14.1 atomicity

手、CHEST、COM、`q_nominal`は一つのexternal IK solveに由来するため、同じsequenceとしてatomicに送る。

頭部は独立taskだが、同bundle内に独立valid bitとsource stampを持たせる。

### 14.2 joint names

- 周期messageでは`JointState.name`を使用できる。
- bridgeとauto_stabilizerは初回受信時またはsession開始時に名前をjoint IDへmappingする。
- 同一session中の名前配列変更はrejectする。
- unknown、duplicate、direct許可外jointはrejectする。

---

## 15. custom ROS state bundle

概念構造:

```text
WholeBodyTeleopState
  std_msgs/Header header
  uint32 schema_version
  uint64 session_id
  uint64 epoch
  uint64 state_sequence
  uint64 last_received_reference_sequence
  uint64 last_accepted_reference_sequence

  sensor_msgs/JointState generated_joint_state
  sensor_msgs/JointState actual_joint_state

  geometry_msgs/Pose root_pose
  geometry_msgs/Pose chest_pose_in_foot_mid
  geometry_msgs/Point robot_com_in_foot_mid
  geometry_msgs/Pose right_hand_pose_in_chest
  geometry_msgs/Pose left_hand_pose_in_chest

  uint32 allowed_task_mask
  uint32 active_task_mask
  uint8 support_state
  uint8 walking_preparation_phase
  uint8 external_reference_status
  uint8 rejection_reason

  bool is_static
  bool is_double_support
  bool wbms_active
  bool walking

  diagnostics fields
```

external generatorはこのstateを次周期の基準にする。

---

## 16. RTM IDLとbridge

### 16.1 IDL

custom ROS messageと意味を一致させたRTM IDLを追加する。

- schema versionを持つ。
- quaternion順序を明記する。
- task mask bitを固定する。
- coordinate frameを文字列TFへ依存させず、schema内の固定定義またはenumで表す。
- sequence、session、epochを保持する。

### 16.2 bridge component

一つの操縦bridge componentでcommandとstateの両方向を扱う。

責務:

- ROS custom messageとRTM IDLの変換。
- finite check。
- quaternion norm check。
- schema version check。
- latest-only mailbox。
- drop、out-of-order、invalid counter。

非責務:

- IK。
- limit。
- interpolation。
- stale policy。
- mode gate。
- walking preparation。

### 16.3 timestamp

- source stampを`ros::Time::now()`で上書きしない。
- bridge受信時刻は別diagnosticとして保持する。
- 別PC HMDのclock同期を安全性の必須前提にしない。
- staleはrobot PCの受信時刻でも判定可能にする。

---

## 17. auto_stabilizer側構成

### 17.1 class分離

`WbmsPostureControl`のprojection solver責務を削除し、次の責務へ再編する。

候補class:

```text
WbmsExternalReferenceControl
NonIkJointReferenceController
```

`WbmsExternalReferenceControl`:

- external bundle受信状態。
- source selector。
- task別stale。
- 500 Hz interpolation。
- CHEST/COM limit。
- COM/ZMP統合。
- walking preparation。
- final IK target設定。
- final validation。
- debug。

`NonIkJointReferenceController`:

- 名前mapping。
- allowlist。
- qRef override。
- position/velocity/acceleration limit。
- stale hold。
- start/stop blend。

### 17.2 projection削除

削除対象:

- projection用robot。
- projection variables。
- projection constraints/tasks。
- projection QP workspace。
- `solveProjection()`。
- projection candidate status。
- projector profiling counter。
- projection専用parameter。

保持・置換対象:

- velocity command limit。
- operation mode。
- static COM/ZMP integration。
- walking preparation。
- COM height hold。
- final IK diagnostics。
- CHEST相対腕拘束。

### 17.3 final IK入力

static時:

- right/left hand: external CHEST相対reference。
- CHEST: external foot-mid reference。
- COM: external robot COMから導出した`genCog + sbpOffset`。
- `q_nominal`: external全身reference。

walking preparation / walking:

- hands: external。
- head: direct external。
- CHEST: internal walking preparationまたはstabilizer。
- COM: internal walking / height hold。
- `q_nominal`: 腕関節だけexternal、脚・腰・体幹は無効。

### 17.4 source selector

初回はtask groupごとに選択できるようにする。

```text
arm_reference_source:
  LEGACY
  EXTERNAL_BUNDLE

posture_reference_source:
  LEGACY
  EXTERNAL_BUNDLE

head_reference_source:
  UPSTREAM_QREF
  EXTERNAL_BUNDLE
```

source変更はWBMS停止中または安全な明示的遷移でのみ許可する。

last-writer-winsは使用しない。

---

## 18. stale、通信断、復帰

### 18.1 個別task stale

| stale group | 動作 |
|---|---|
| RIGHT_HAND | 現在generated右手CHEST相対poseをhold |
| LEFT_HAND | 現在generated左手CHEST相対poseをhold |
| CHEST | 現在generated CHEST姿勢をhold |
| COM | 現在generated robot COMをhold |
| HEAD | 現在首角をhold |

ゼロ速度がfreshに届いている状態はstaleではない。

### 18.2 generator / bridge stale

全external taskを現在generated stateへholdする。

- 自動直立復帰はしない。
- walking preparation未実行のまま歩行APIを許可しない。
- staleだけでWBMSを停止しない。

### 18.3 同一session復帰

短時間通信断でepochが変わらない場合:

1. current generated stateへexternal integratorをrebaseする。
2. 速度・加速度limit付きで自動再開する。
3. stale中の未実現commandを回収しない。

### 18.4 process再起動

epoch変更時:

- external referenceを即時acceptしない。
- 明示的な再enableまたはWBMS再startを要求する。
- 現在姿勢holdを維持する。

---

## 19. WBMS開始・停止

### 19.1 開始条件

`startWholeBodyMasterSlave()`を再利用する。

必須:

- generator heartbeat fresh。
- bridge heartbeat fresh。
- schema version compatible。

個別task inputはoptional。

- 手inputなし: 現在手pose hold。
- CHEST inputなし: 速度ゼロ。
- COM inputなし: 速度ゼロ。
- HMD inputなし: 現在首角hold。

### 19.2 baseline

session開始時に以下をsnapshotする。

- generated robot state。
- hand slave baseline in CHEST。
- external master baselineは最初のfresh sample時。
- CHEST baseline in foot-mid。
- robot COM baseline in foot-mid。
- head baseline。
- source selector。

### 19.3 停止

- CHEST/COM/hand referenceを現在姿勢へhold。
- legacyのWBMS停止transitionを維持する。
- direct首角を上流qRefへ滑らかに戻す。
- sessionを無効化する。
- stale commandをclearする。

---

## 20. walking preparation

### 20.1 維持する仕様

- 歩行APIはREADY前にreject。
- pending commandをhidden goalとして保存しない正式仕様を維持する。
- 専用walking preparation serviceを使用する。
- CHEST、COM XY、rootを速度・加速度limit付きで復帰する。
- COM Zはwalking preparation開始時高さを保持する。
- READY成立とwalking command投入を分離する。
- timeout時は歩行を強行しない。
- walking中は腕と頭部を継続する。

### 20.2 ownership

walking preparation開始時:

- external CHEST/COM operator commandをゼロへ減速。
- external generatorへCHEST/COM `allowed=false`をfeedback。
- external generatorはcurrent accepted stateへrebaseし、未実現commandを捨てる。
- walking preparation controllerがCHEST/COM targetのownershipを持つ。

### 20.3 READY判定

projection candidate safe条件を、final IK後のexternal reference validationへ置換する。

READY条件には少なくとも以下を含める。

- applied CHEST/COM速度が小さい。
- walking preparation return速度が小さい。
- CHEST error。
- COM XY/Z error。
- root error。
- walking stability mode。
- final IK max joint step。
- final foot error。
- dynamics finite。
- external reference statusが安全またはhold。

---

## 21. final IK後validation

projection削除後の安全監視として、final IKの結果を毎周期検証する。

必須項目:

- root、joint、CHEST、COMがfinite。
- joint limit table内。
- max one-cycle joint delta。
- root one-cycle translation/rotation。
- 左右足位置・姿勢誤差。
- CHEST角速度。
- COM速度。
- self collision residualまたは最新distance。
- reference age。
- session、epoch、sequence。
- support/mode version一致。

invalid時:

1. その周期のexternal task更新を採用しない。
2. last safe generated postureまたはcurrent safe postureをholdする。
3. rejection reasonをstate feedbackへ出す。
4. raw targetを後で回収しない。
5. walking preparation中ならFAILED/UNSAFEへ遷移する条件を明記する。

joint limitを後段clampするだけで安全とみなさない。clamp後に足・手・COM等が変化し得るため、QP constraintとnonlinear validationの双方を使う。

---

## 22. parameter

初期parameter群を次の責務へ分ける。

### 22.1 input mapping

- hand position scale、左右別3軸。
- HMD device-to-robot rotation。
- head yaw/pitch gain、sign。
- source timeout。
- heartbeat timeout。

### 22.2 external IK

- frequency。
- max iteration。
- damping。
- task weights / priorities。
- hand position precision。
- hand orientation precision。
- support margin。
- collision activation distance。
- posture regularization weight。

### 22.3 auto_stabilizer

- 500 Hz interpolation duration。
- CHEST velocity/acceleration/offset limit。
- COM velocity/acceleration/offset limit。
- direct joint velocity/acceleration limit。
- final validation thresholds。
- source selector。

parameter名と単位はIDL、ROS param、文書で一致させる。

---

## 23. diagnosticsとログ

### 23.1 external generator

- input age、group別。
- heartbeat age。
- session/epoch/sequence。
- solver status。
- solver time mean/p99/maxはlogger側で集計可能な毎sample値。
- iteration数。
- candidate validation status。
- realized hand/CHEST/COM error。
- min joint margin。
- support margin。
- collision active count。

### 23.2 bridge

- command receive/write count。
- state receive/publish count。
- invalid message count。
- out-of-order count。
- dropped/overwritten count。
- schema mismatch。

### 23.3 auto_stabilizer

- active source、group別。
- allowed/active/valid/stale task mask。
- reference age。
- last received/accepted sequence。
- rejection reason。
- interpolation state。
- final IK time。
- onExecute time。
- final joint step。
- foot error。
- hand generated error。
- CHEST/COM realized velocity。
- walking preparation phase/READY/FAILED。
- direct head target/current/error。

固定indexの巨大なdebug配列を無計画に増やさず、schemaとindex表を同じcommitで更新する。

---

## 24. Milestoneと作業単位

各Milestoneはさらにrepository単位のwork unitへ分ける。各work unitは原則として一つのreview可能なcommitにする。

### M0: baseline、branch、依存関係確定

#### M0-A: commit履歴調査

- M4.2.2全修正完了commitを特定。
- 最初のM5 commitを特定。
- CHEST相対腕拘束導入commitを特定。
- walking preparation review修正がpre-M5範囲に全て含まれるか確認。
- exact base SHAを本書とProgressへ記録。

#### M0-B: branch作成

- auto_stabilizer実装branch。
- rtmros bridge branch。
- dependency branch確認。
- new repository作成。
- 計画文書とSkillをcherry-pick。

#### M0 acceptance

- 各repository、branch、HEAD SHAが記録されている。
- `1.0`、pre-M5、`wbms-dev`のfinal IK差分表がある。
- pre-M5 baselineがbuildできる。
- 既存実機またはシミュレータtimeover基準ログの場所が記録されている。

### M1: protocolとmessage contract

#### M1-A: ROS message

- `whole_body_teleop_msgs`作成。
- command/state bundle定義。
- task mask、status、rejection enum。
- schema version。

#### M1-B: RTM IDL

- bridge用IDL定義。
- ROS messageとのmapping表。

#### M1-C: protocol documentation

- frame、単位、quaternion順序。
- timestamp、session、epoch、sequence。
- compatibility rule。

#### M1 acceptance

- message/IDLが生成・buildできる。
- round-tripで全fieldが保持される。
- schema mismatchがrejectされる。
- mapping表に未定義fieldがない。

### M2: bridge skeleton

- command ROS->RTM。
- state RTM->ROS。
- latest-only mailbox。
- independent process launch。
- heartbeatとdiagnostics。

#### M2 acceptance

- bridgeが`hrpEC`外で動作する。
- queue backlogを作らない。
- source timestampを保持する。
- out-of-orderとinvalid quaternionをrejectする。

### M3: auto_stabilizer receiverとstate feedback skeleton

- new ports。
- schema/session検証。
- state bundle出力。
- source selector。
- 既存control behaviorは変更しない。

#### M3 acceptance

- external command未接続でpre-M5挙動が変わらない。
- state feedbackが50〜100 Hzで外部nodeへ届く。
- legacy sourceで既存腕操作が維持される。

### M4: task lifecycle、stale、session

- task group state machine。
- heartbeat。
- stale hold。
- same-epoch rebase。
- epoch変更時再enable。
- WBMS start readiness。

#### M4 acceptance

- group単独staleで他groupが停止しない。
- generator停止で全external taskが現在姿勢hold。
- node再起動後に古いtargetをacceptしない。
- zero velocity freshはstaleにならない。

### M5: non-IK head reference

#### M5-A: auto_stabilizer controller

- qRef read後、FK前override。
- allowlist。
- position/velocity/acceleration limit。
- WBMS stop復帰。

#### M5-B: external HMD mapping

- HMD baseline。
- yaw/pitch抽出。
- recenter service。
- stale。

#### M5 acceptance

- 首関節はfinal IK変数に増えない。
- static、walking preparation、walkingで頭部操作継続。
- roll、positionを無視。
- stale時不連続なくhold。
- WBMS停止で上流qRefへ滑らかに戻る。

### M6: hand differential mapping

-左右master baseline。
- slave baseline in CHEST。
- position scale。
- rotation delta。
- independent stale。
- raw target diagnostics。

#### M6 acceptance

- 現行差分操縦と同じ方向・scale意味。
- CHESTが動いても手targetがCHEST相対で追従。
- 左右別scaleが機能する。
- 手input未接続で現在pose hold。

### M7: external whole-body IK、arms first

- generated state初期化。
- joint safety、feet、hands、posture。
- realized hand、`q_nominal`出力。
- initial candidate validation。
- self collisionはまだfinal IKのみでもよい。

#### M7 acceptance

- static両足支持で左右手を独立操作できる。
- hand position precision 0.01 m設定が機能する。
- hand orientation precision 0.10 rad設定が機能する。
- unsafe/solver failure時に前回safe referenceをhold。
- q_nominalを直接hardwareへ送らない。

### M8: CHEST/COM external IKとauto_stabilizer統合

- CHEST/COM velocity入力。
- accepted state基準の局所target。
- static gate。
- external IK task追加。
- auto_stabilizer COM/ZMP integration。
- external CHEST final IK target。

#### M8 acceptance

- static両足支持でCHEST、COM、両腕を同時操作。
- hidden goalなし。
- limit到達後の逆方向入力へ反応。
- COM、ZMP、`refdz`、`omega`、`l`がfiniteかつ連続。

### M9: projection削除とwalking preparation再接続

- projection solver、robot、workspace削除。
- external reference controllerへ置換。
- final validationへREADY条件を接続。
- walking preparation ownership。
- walking中腕・頭、CHEST/COM無効。

#### M9 acceptance

- projector QP solveが500 Hz経路に存在しない。
- 前傾姿勢からREADYへ到達。
- READY前walking API reject。
- READY後walking開始。
- COM高さ保持。
- walking中腕・頭操作継続。
- FAILED時にhidden pending commandなし。

### M10: external IK self collision

- collision state feedback。
- external constraint。
- active set diagnostics。
- externalとfinal IKのthreshold整合。

#### M10 acceptance

- 床接近姿勢で腕-脚、腕-胴、左右腕collisionを回避。
- external q_nominalが恒常的にfinal collision constraintへ拒否されない。
- collision入力stale時は保守的holdまたは外部task縮小。

### M11: integrated floor reach

- 大きな前傾。
- COM下降・移動。
- 片手、両手の床付近到達。
- 頭部操作。
- walking preparation。
- 歩行開始。

#### M11 acceptance

- mode遷移なしで腕+CHEST+COMにより床付近へ到達。
- 足踏み、yaw振動、急激なjoint stepなし。
- 床付近姿勢からwalking preparation READY。
- READY後歩行開始。
- walking中腕・頭継続。

### M12: performance、比較、cleanup

- pre-M5 baselineと比較。
- `1.0` final IK構造とのoffline/branch比較。
- onExecute mean/p95/p99/max、timeover。
- external solver latency。
- bridge latency。
- legacy未使用項目整理を別commitで検討。

#### M12 acceptance

- projection追加前に近い500 Hz余裕を回復。
- onExecute p99とtimeoverがbaselineから許容範囲。
- external solver停止が500 Hz deadlineへ影響しない。
- cleanup commitが機能変更と混在しない。

---

## 25. buildと静的check

### 25.1 共通

```sh
git diff --check
```

- `clang-format`は適用しない。
- コメント、Markdownは日本語。
- 不要な全体rewriteを行わない。

### 25.2 auto_stabilizer

IDL変更後初回:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

以後:

```sh
catkin build auto_stabilizer --no-deps
```

### 25.3 external packages

package名確定後、依存順に個別buildする。

```text
whole_body_teleop_msgs
whole_body_teleop_reference_generator
whole_body_teleop_rtmros_bridge
```

### 25.4 dependency repositories

変更した場合のみ個別build、diff checkを行う。

---

## 26. 検証マトリクス

### 26.1 no-op / interface

- external未接続。
- heartbeatのみ。
- task全無効。
- schema mismatch。
- out-of-order。
- duplicate sequence。

### 26.2 腕

- 右手のみ。
- 左手のみ。
- 両手独立。
- position scale。
- orientation。
- stale片手のみ。

### 26.3 CHEST / COM

- roll/pitch/yaw単独。
- COM x/y/z単独。
- CHEST+COM。
- CHEST+COM+両腕。
- limit到達と逆方向。

### 26.4 頭部

- yaw。
- pitch。
- roll入力無視。
- recenter。
- stale。
- walking preparation。
- walking。

### 26.5 walking preparation

- 前傾なし。
- 大前傾。
- COM低姿勢。
- 腕操作中。
- 頭部操作中。
- timeout。
- external node停止。

### 26.6 collision

- 腕-胴。
- 腕-脚。
- 左右腕。
- collision input stale。

### 26.7 performance

- external 100 Hz。
- external 50 Hz。
- HMD 60 Hz。
- bridge停止。
- external solver遅延。
- onExecute timeover。

---

## 27. acceptance全体条件

初期リリース候補は次を全て満たすこと。

1. projection IKのQP solveが500 Hz経路から削除されている。
2. external nodeが独立processである。
3. static両足支持で左右手、CHEST、COMを同時操作できる。
4. 手位置IK precision 0.01 m、姿勢precision 0.10 radを設定できる。
5. 頭部yaw/pitch操作がstatic、walking preparation、walkingで継続する。
6. stale時に現在姿勢holdし、残留commandを再生しない。
7. pre-M5 walking preparationの安全要件を維持する。
8. 床付近へ手先到達後、歩行可能姿勢へ復帰し、歩行開始できる。
9. joint limit、collision、足拘束を弱めない。
10. legacy interfaceを初回は保持する。
11. 全変更がreview済みで、Progressへ実施内容、build、未確認事項が記録されている。
12. 500 Hz performanceがM0で取得したbaselineの許容範囲内である。

performanceの数値thresholdはM0 baseline取得後に本書へ追記する。

---

## 28. 後続候補

- task scalingを含むsingle-QP final WBC。
- walking中CHEST/COM操作。
- rigid bimanual coupling。
- finger direct reference実運用。
- hand recenter service。
- separate-PC external generator。
- payload、wrench、impedance統合。
- external trajectory optimization / MPC。
- legacy port削除。

---

## 29. Codex実装運用

実際のCodex work unitは`WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`に従う。

- work unitごとに`/plan`でread-only調査。
- 承認済みwork unitを`/goal`で実装。
- dedicated `/review`を実行。
- findingを修正し、fresh reviewを繰り返す。
- build、diff check、Progress更新。
- commit readiness確認後に一つのatomic commitを作る。

各work unitは新しいtaskで開始できるよう、Progressへ十分な引き継ぎ情報を残す。
