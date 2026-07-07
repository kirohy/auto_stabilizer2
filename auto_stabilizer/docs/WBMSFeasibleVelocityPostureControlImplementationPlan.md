# WBMS実現可能速度投影型・体幹／重心操縦 実装仕様書

## 現行優先度に関する注記

本書はWBMS実現可能速度投影型・体幹/COM操縦の最初期に作成された実装仕様書であり、M1からM3の基礎実装方針を理解するための文書である。

後続作業により、本書の一部仕様・計画は更新されている。後続スレッドで本書を読む場合は、以下の優先関係を守る。

- 最新の作業状態、実装済み内容、ログ解釈は `WBMSFeasibleVelocityPostureControlProgress.md` を優先する。
- projector候補採用判定、safe candidate、`solveIKLoop()` 戻り値の扱いは `WBMSProjectionAcceptanceFixImplementationPlan.md` を優先する。
- 歩行準備遷移、READY判定、歩行API受付可否、pre-walk姿勢生成は `WBMSWalkingPreparationDesignRevisionPlan.md` を優先する。
- `WBMSWalkingPreparationTransitionImplementationPlan.md` は、歩行準備遷移の前提・経緯として参照する。
- 500 Hz計算量削減、IK parameter、`checkFinalState`、prioritized IK軽量化は `WBMSComputationReductionImplementationPlan.md` を優先する。

本書と後続文書が矛盾する場合、本書を根拠に後続文書の仕様を上書きしてはならない。

## 1. 文書の目的

本書は、`kirohy/auto_stabilizer2` の `wbms-dev` ブランチにおいて、`startWholeBodyMasterSlave()` 起動後の操縦モードへ、以下の機能を追加するための実装仕様・作業計画である。

- 腕操縦と既存歩行指令の両立を維持する。
- ユーザー入力に応じて、体幹を直感的な速度関係で傾けられるようにする。
- ユーザー入力に応じて、重心位置を前後・左右・上下へ移動できるようにする。
- 到達不能な姿勢目標を内部へ蓄積せず、制約へ達したときは滑らかに停止し、逆方向入力には直ちに反応する。
- 500 Hz周期実行を維持する。
- 既存の歩行安定化、足拘束、関節限界、自己干渉回避を壊さない。

本書はCodex等の実装エージェントへ直接読み込ませることを想定する。単なる検討案ではなく、原則として本書の必須要件をそのまま実装すること。

---

## 2. 作業上の前提

- 対象リポジトリ: `kirohy/auto_stabilizer2`
- 対象ブランチ: `wbms-dev`
- 主な対象ディレクトリ: `auto_stabilizer/rtc/AutoStabilizer`
- IDL変更先: `auto_stabilizer/idl/AutoStabilizerService.idl`
- コメントおよびMarkdownは日本語で記述する。
- `clang-format`は適用しない。
- 本リポジトリではテストコードを新規作成しない。
- 既存の公開API、歩行API、腕のWBMS入力形式は、必要がない限り変更しない。
- IDL変更後の初回ビルドは以下を使う。

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

以後は以下でよい。

```sh
catkin build auto_stabilizer --no-deps
```

実装前に必ず現行コードを読み、ブランチ上のコードが本書記載と異なる場合は、意図を維持したうえで現行構造へ適合させること。安易に過去の実験コードを復活させないこと。

---

## 3. 背景と維持すべき現行仕様

### 3.1 既に成功している部分

現行実装では、WBMS中の直接操縦対象を上半身エンドエフェクタへ限定し、脚は以下の既存歩行APIへ任せている。

- `goVelocity()`
- `goPos()`
- `setFootSteps()`

また、WBMS中の上半身エンドエフェクタ拘束は `CHEST_JOINT2` 相対になっている。これにより、歩行時に体幹が動いても、手先がworld固定のように体幹運動を妨げず、腕姿勢が体幹へ自然に追従する。

この設計は維持すること。上半身エンドエフェクタをroot/world基準へ戻してはならない。

### 3.2 歩行開始遅延

現行実装には、WBMS静止状態から歩行を開始する際に、以下を分離する仕組みがある。

1. 歩行安定化用のroot姿勢・COM Z重みを復帰させる。
2. その後にfuture footstepを生成する。

`WbmsWalkingCommandDelay`および以下の状態は維持すること。

- `isWbmsWalkingStartDelay`
- `wbmsWalkingStartDelayRemainTime`
- `wbmsWalkingStabilityStartTime`
- `wbmsWalkingStabilityStopTime`

今回追加する体幹・COM操縦は、歩行開始遅延へ入った時点から滑らかに無効化し、歩行安定化を優先すること。

### 3.3 現在最も安定している方式

現行の体幹操縦実験では、最終IKへroot/CHEST姿勢を直接必達に近い形で加える方式よりも、reference用IKで姿勢候補を生成し、最終IKではreference angleとして弱く追従する方式が最も安定していた。

ただし現在のreference IKには以下の問題がある。

- ユーザー角速度を姿勢目標へ積分してからIKへ渡すため、到達不能目標が残留する。
- reference IKが主に「足固定＋root姿勢」であり、COM位置や姿勢配分を明示的に扱わない。
- 体幹姿勢という名称でも、実際にはroot姿勢を目標にしている。
- 前傾時に腰を大きく落とす局所解へ流れやすい。
- COM Z拘束を後付けすると、root姿勢、足、腕、COMが競合する。

今回の実装では、既存reference IKを単なる姿勢目標追従器ではなく、**現在姿勢から次の1周期で実現可能な速度を投影する局所QP/IK**へ変更する。

---

## 4. 実装方針の要点

### 4.1 目標姿勢を積分しない

ユーザー入力から独立した体幹姿勢目標・COM位置目標を無制限に積分してはならない。

各周期で行う処理は以下とする。

1. 現在の生成姿勢 `genRobot` をreference投影用ロボットへコピーする。
2. 現在姿勢から `dt` 後の小さな目標を作る。
3. 足拘束、関節限界、自己干渉等を考慮した1回の優先度付きIK/QPで、実現可能な次姿勢へ投影する。
4. 投影後の姿勢・COMを最終IKの整合済み目標として使う。
5. 次周期は再び、その時点の実際の `genRobot` から計算する。

したがって、達成できなかった差分は次周期へ持ち越さない。

### 4.2 体幹はrootではなくCHEST姿勢を操作する

ユーザーが操作する姿勢は、`gaitParam.chestLinkName`で指定されたリンク、現運用では `CHEST_JOINT2` の世界姿勢とする。

- 位置3自由度は体幹タスクでは拘束しない。
- 姿勢3自由度のみを扱う。
- root、股関節、足首、CHEST関節への姿勢分配はIKへ任せる。
- root姿勢をユーザー指令として直接拘束しない。

### 4.3 COM入力は既存の `refTorsoVelIn` を再利用する

新しい入力ポートは作らず、既存の `RTC::TimedVelocity3D` 型 `refTorsoVelIn` を以下の意味で使う。

| フィールド | 意味 | 座標系 | 単位 |
|---|---|---|---|
| `vx` | COM前後速度 | `footMidCoords` | m/s |
| `vy` | COM左右速度 | `footMidCoords` | m/s |
| `vz` | COM上下速度 | `footMidCoords` | m/s |
| `vr` | 体幹roll速度 | `footMidCoords`軸 | rad/s |
| `vp` | 体幹pitch速度 | `footMidCoords`軸 | rad/s |
| `va` | 体幹yaw速度 | `footMidCoords`軸 | rad/s |

`vx/vy/vz`はロボットモデルの生のcenter of massを操作する。内部の`sbpOffset`を使う箇所では、既存定義に合わせて `genCog = projectedRobotCOM - sbpOffset` とする。

### 4.4 静止両足支持中のみ全機能を有効にする

体幹・COM速度投影を有効にする条件は原則として以下すべてを満たす場合とする。

- AutoBalancer実行中。
- WBMSが有効、または有効へ遷移中。
- `gaitParam.isStatic()` がtrue。
- 右脚・左脚ともsupport phase。
- `isWbmsWalkingStartDelay`がfalse。
- 左右いずれの脚もmanual control対象ではない。

上記を満たさない場合、入力目標はゼロへ加速度制限付きで戻す。歩行中のCOM操縦は今回の実装範囲外とする。

歩行中の腕操縦は従来どおり維持する。

---

## 5. 現行入力処理の不具合修正

### 5.1 問題

現行 `readInPortData()` は、`refTorsoVelIn`が新着でない周期に角速度目標をゼロへ戻している。

500 Hz制御に対して50 Hzで入力した場合、非ゼロ指令が1周期、ゼロ指令が約9周期という入力になる。これは「一定入力なら一定速度」という要求を満たさず、通信周期へ依存した応答になる。

### 5.2 必須修正

`refTorsoVelIn`はsample-and-hold方式に変更する。

新着データがないだけではゼロへ戻さない。以下の状態を保持する。

```cpp
cnoid::Vector3 wbmsRawComVelocityCommand;
cnoid::Vector3 wbmsRawTorsoAngularVelocityCommand;
cnoid::Vector3 wbmsAppliedComVelocityCommand;
cnoid::Vector3 wbmsAppliedTorsoAngularVelocityCommand;
double wbmsVelocityCommandAge;
bool wbmsVelocityCommandValid;
```

新着かつfiniteな入力を受信したときのみ以下を行う。

```text
raw commandを更新
command age = 0
valid = true
```

新着がない周期ではraw commandを変更しない。

毎周期、command ageを`dt`だけ増加させる。以下の場合は目標速度をゼロとする。

- command ageがtimeoutを超えた。
- WBMS操作可能条件を満たさない。
- 入力がinvalid。

ゼロへの遷移、および新しい入力への遷移は、補間時間ではなく成分ごとの加速度limitで行う。

```cpp
applied += clamp(desired - applied, accelerationLimit * dt);
```

`wbms_interpolate_duration`は引き続きエンドエフェクタ姿勢入力の補間に使用し、速度入力のsample-and-holdには流用しない。

### 5.3 stale commandの再開禁止

歩行開始、WBMS停止、AutoBalancer停止、RTC再初期化時にはraw commandをクリアする。

歩行終了後に静止へ戻っても、歩行前の入力を自動再開してはならない。操作者から新しい`refTorsoVelIn`を受信するまでゼロを維持する。

### 5.4 クラス責務

現行 `WbmsTorsoControl` は姿勢目標の積分を行っているが、この責務は廃止する。

**`WbmsPostureControl`へ改名して責務を整理する方式を採用すること。**

- `WbmsTorsoControl.h/.cpp`を削除または置換する。
- `WbmsPostureControl.h/.cpp`を追加する。
- CMakeListsと`AutoStabilizer`のメンバを更新する。

`WbmsPostureControl`は以下を担当する。

- 速度指令のhold、timeout、加速度limit。
- WBMS操作可否の判定。
- 歩行安定化モードの補間。
- 実現可能速度投影用IK。
- 投影結果によるCOM参照、体幹参照、ZMP参照の更新。

最終全身IKそのものは引き続き`FullbodyIKSolver`が担当する。

---

## 6. 状態とパラメータ

### 6.1 `GaitParam`へ追加する状態

名称は周辺コードの命名へ合わせて多少変更してよいが、意味は維持すること。

```cpp
// 生入力・適用入力
cnoid::Vector3 wbmsRawComVelocityCommand;
cnoid::Vector3 wbmsRawTorsoAngularVelocityCommand;
cnoid::Vector3 wbmsAppliedComVelocityCommand;
cnoid::Vector3 wbmsAppliedTorsoAngularVelocityCommand;
double wbmsVelocityCommandAge;
bool wbmsVelocityCommandValid;

// WBMS開始時基準。footMidCoords座標系
cnoid::Matrix3 wbmsStartChestRInFootMid;
cnoid::Vector3 wbmsStartComInFootMid;
bool wbmsPostureBaselineValid;

// 投影結果
std::vector<double> wbmsPostureReferenceQ;
cnoid::Matrix3 wbmsProjectedChestR;
cnoid::Vector3 wbmsProjectedRobotCom;
cnoid::Vector3 wbmsRealizedComVelocity;
cnoid::Vector3 wbmsRealizedTorsoAngularVelocity;
bool wbmsPostureReferenceValid;

// モード値
// 0～1。歩行中および歩行開始遅延中に1へ近づく。
double wbmsWalkingStabilityModeValue;
// wbmsMode * (1 - walkingStabilityMode)
double wbmsOperationModeValue;
```

旧状態 `wbmsTorsoTargetRpy` は削除する。互換上残す必要がある場合でも制御に使用してはならない。

`refTorsoAnglVel`の`TwoPointInterpolator`も、新方式では不要である。新しいraw/applied commandへ置き換える。

### 6.2 IDLへ追加するパラメータ

既存パラメータのうち以下は意味を更新して継続利用する。

```idl
sequence<double,3> wbms_torso_angular_velocity_limit;
sequence<double,3> wbms_torso_rpy_lower_limit;
sequence<double,3> wbms_torso_rpy_upper_limit;
sequence<double,3> wbms_torso_orientation_weight;
sequence<double,3> wbms_torso_orientation_max_error;
```

意味は以下とする。

- `wbms_torso_angular_velocity_limit`: 入力角速度limit。
- `wbms_torso_rpy_lower_limit/upper_limit`: WBMS開始時のCHEST姿勢からの実現姿勢差分limit。旧方式の「積分目標limit」ではない。
- `wbms_torso_orientation_weight`: 投影IKおよび最終IKのCHEST姿勢weight。
- `wbms_torso_orientation_max_error`: 投影IKおよび最終IKで許容する1秒あたりの最大姿勢補正量。実際の`maxError`には`dt`を掛ける。

以下を追加する。

```idl
/// refTorsoVelInを最後に受信してから指令を無効にするまでの時間[s]
double wbms_velocity_command_timeout;

/// 体幹角速度指令の加速度limit[rad/s^2]
sequence<double,3> wbms_torso_angular_acceleration_limit;

/// COM速度limit[m/s]
sequence<double,3> wbms_com_velocity_limit;

/// COM速度指令の加速度limit[m/s^2]
sequence<double,3> wbms_com_acceleration_limit;

/// WBMS開始時COMからの位置差分下限[m]。footMidCoords座標系
sequence<double,3> wbms_com_offset_lower_limit;

/// WBMS開始時COMからの位置差分上限[m]。footMidCoords座標系
sequence<double,3> wbms_com_offset_upper_limit;

/// 投影IKおよび最終IKのCOM位置weight
sequence<double,3> wbms_com_position_weight;

/// 支持多角形境界から内側へ確保するCOM/ZMP XY余裕[m]
double wbms_com_xy_support_margin;
```

### 6.3 推奨初期値

初期値は保守的にする。

```text
wbms_velocity_command_timeout = 0.2

wbms_torso_angular_velocity_limit = [0.15, 0.15, 0.30]
wbms_torso_angular_acceleration_limit = [0.50, 0.50, 1.00]

wbms_torso_rpy_lower_limit = [-0.10, -0.02, -0.30]
wbms_torso_rpy_upper_limit = [ 0.10,  0.25,  0.30]
wbms_torso_orientation_weight = [0.3, 0.3, 0.3]
wbms_torso_orientation_max_error = [0.15, 0.15, 0.30]

wbms_com_velocity_limit = [0.05, 0.05, 0.05]
wbms_com_acceleration_limit = [0.20, 0.20, 0.20]
wbms_com_offset_lower_limit = [-0.10, -0.08, -0.20]
wbms_com_offset_upper_limit = [ 0.10,  0.08,  0.05]
wbms_com_position_weight = [3.0, 3.0, 1.0]
wbms_com_xy_support_margin = 0.03
```

値は実機・シミュレータ調整対象だが、初回実装では上記を使用する。

`setAutoStabilizerParam()`ではfinite確認を前提に、負値を許さないパラメータをclampする。lower/upperは大小を正規化する。現在姿勢が新しいlimit外にある場合でも姿勢を瞬時に変更せず、以後limit内へ戻る方向だけを許可する。

`getAutoStabilizerParam()`へ全項目を反映する。

---

## 7. WBMS開始・停止時の初期化

### 7.1 `startWholeBodyMasterSlave()`

従来の腕用offset保存後に、`WbmsPostureControl::start()`相当を呼ぶ。

開始時に以下を行う。

1. `genRobot`のforward kinematicsとcenter of massを最新化する。
2. `footMidCoords`を取得する。
3. CHEST姿勢をfoot-mid座標へ変換して保存する。
4. robot COMをfoot-mid座標へ変換して保存する。
5. raw/applied velocityをゼロにする。
6. command ageをtimeoutより大きい値へ設定し、validをfalseにする。
7. 投影用ロボットを現在の`genRobot`へ同期する。
8. `wbmsPostureReferenceValid`をfalseで初期化する。

姿勢基準は次の定義とする。

```cpp
R_FC0 = footMidR.transpose() * chestR;
p_FCOM0 = footMidT.inverse() * robotCOM;
```

### 7.2 停止・リセット

以下で同じclear処理を呼ぶ。

- `stopWholeBodyMasterSlave()`
- `stopAutoBalancer()`
- `MODE_SYNC_TO_ABC`初期化
- RTC activate/deactivateに必要な箇所
- 歩行開始遅延への移行時

clear後に古いcommandや投影結果を再利用しない。

---

## 8. 実現可能速度投影IK

### 8.1 投影用ロボット

`WbmsPostureControl`は専用の`cnoid::BodyPtr wbmsPostureRobot`を保持する。

- `init()`で`genRobot->clone()`する。
- 毎周期、solve前に現在の`genRobot`からroot位置姿勢と全関節角をコピーする。
- 毎周期cloneしてはならない。
- forward kinematicsとcenter of massを更新する。

このロボットは目標を累積する状態ではなく、各周期の局所投影用ワークモデルとして使う。

### 8.2 IK変数

変数は以下に限定する。

- floating root。
- 左右足のparent linkからrootまでのancestor joint。
- `chestLinkName`からrootまでのancestor joint。

上記のunionを作り、`jointControllable`がtrueの関節だけを入れる。腕の下流関節、頭、指等は入れない。

実装ではinit時にjoint ID集合を構築し、rootの後にjoint ID順で変数を並べる。毎周期ancestor探索やvector再構築を行わない。

この限定により、以下を満たす。

- COM計算には腕の現在姿勢・質量を含める。
- 体幹姿勢を腕関節の逃げで満たさない。
- 変数数を抑え、500 Hzへ収める。

### 8.3 1周期先のCHEST目標

現在CHEST姿勢をfoot-mid座標へ変換する。

```cpp
R_FC = footMidR.transpose() * chestR;
```

適用角速度を `w = [rollRate, pitchRate, yawRate]` とする。

```cpp
R_FC_des_unclamped = rotFromRpy(w * dt) * R_FC;
```

WBMS開始時姿勢との差分を求める。

```cpp
R_delta = R_FC_des_unclamped * wbmsStartChestRInFootMid.transpose();
deltaRpy = rpyFromRot(R_delta);
```

`deltaRpy`をlower/upper limitへclampし、目標を再構成する。

```cpp
R_FC_des = rotFromRpy(clampedDeltaRpy) * wbmsStartChestRInFootMid;
R_chest_des = footMidR * R_FC_des;
```

小角度かつ制限範囲内での操縦を想定する。ユーザー入力をroot姿勢へ変換しない。

### 8.4 1周期先のCOM目標

現在のrobot COMをfoot-mid座標へ変換する。

```cpp
p_FCOM = footMidT.inverse() * robotCOM;
p_FCOM_des = p_FCOM + appliedComVelocity * dt;
```

開始時COMとの差分をlower/upper limitへclampする。

```cpp
offset = clamp(p_FCOM_des - wbmsStartComInFootMid,
               wbmsComOffsetLowerLimit,
               wbmsComOffsetUpperLimit);
p_FCOM_des = wbmsStartComInFootMid + offset;
```

さらにXYを支持多角形内へclampする。

### 8.5 支持多角形margin

両足の`legHull`頂点を現在の`genCoords`でgenerate frameへ変換し、convex hullを作る。これをfoot-mid座標へ変換する。

`wbms_com_xy_support_margin`だけ内側へ縮小した凸多角形を作るhelperを`MathUtil`へ追加する。

推奨helper例:

```cpp
std::vector<Eigen::Vector3d> shrinkConvexHull2D(
  const std::vector<Eigen::Vector3d>& hull,
  double margin);
```

CCW凸包の各辺を内向き法線方向へmarginだけ平行移動し、隣接直線の交点から縮小凸包を構成する。

- marginが0なら元のhullを返す。
- 縮小後に有効な多角形を作れない場合、COM XY速度入力をその周期はゼロ扱いとする。
- `calcNearestPointOfHull()`を用いて目標XYを縮小凸包へ射影する。
- Zはこの処理で変更しない。

支持多角形制約は、最終IKへ新しい硬い拘束を加えるためではなく、ユーザー目標を事前に安全側へ制限するために使う。

### 8.6 拘束の優先度

投影IKの優先度は以下とする。

#### 優先度0: 関節安全

- `JointVelocityConstraint`
- `JointLimitMinMaxTableConstraint`

対象は投影IK変数に含まれる関節のみ。

#### 優先度1: 自己干渉

- `ClientCollisionConstraint`
- 現行と同様、距離が近いpairのみを入れる。
- 距離閾値は現行最終IKの値を踏襲する。

#### 優先度2: 両足拘束

- 左右足の位置姿勢6自由度。
- targetは現在の`abcEETargetPose`。
- static両足支持を前提とする。
- weightは既存足EE weightを利用可能。

#### 優先度3: 主操作タスク

- CHEST姿勢3自由度。
- COM位置3自由度。

同じ優先度のソフトタスクとして解く。

CHESTのPositionConstraintは位置weightを0とする。

```text
weight = [0, 0, 0,
          torsoWeightRoll,
          torsoWeightPitch,
          torsoWeightYaw]
```

COMConstraintは`wbms_com_position_weight`を使う。

#### 優先度4: 最小変位／姿勢選択

基本はsolverの`dqWeight`による最小変位を利用する。

必要な場合のみ、現在の`genRobot`関節角をtargetとする小さな`JointAngleConstraint`を追加する。ただし、手作業で股関節・膝・足首offsetを生成してはならない。

### 8.7 solver設定

- `maxIteration = 1`
- `dt = dt`
- `precision = 0.0`
- `wn`、`we`は現行reference IKの安定値を出発点とする。
- `dqWeight`は既存`FullbodyIKSolver::dqWeight`と同じ値を参照する。
- 1周期内のline search、複数回solve、非線形反復は追加しない。

目標はすべて現在状態から`dt`後の微小差分であるため、1反復での速度レベル投影として扱う。

### 8.8 maxError

CHEST姿勢の角度maxErrorは以下とする。

```cpp
maxErrorAngular[i] = wbmsTorsoOrientationMaxError[i] * dt;
```

COM位置のmaxErrorは以下を基本とする。

```cpp
maxErrorPosition[i] = wbmsComVelocityLimit[i] * dt;
```

足拘束等の既存maxErrorは現行値を踏襲する。

### 8.9 投影結果の検証

solve後に以下を確認する。

- root位置姿勢、対象関節角、COM、CHEST姿勢がfinite。
- 関節角がlimit内。
- 両足位置姿勢誤差が許容値内。
- 異常な1周期変位がない。

投影結果がinvalidな場合は以下とする。

- `wbmsPostureReferenceValid = false`
- reference qは現在の`genRobot`値。
- projected CHESTは現在値。
- projected COMは現在値。
- realized velocityはゼロ。
- `genCog`、ZMP参照を投影結果で上書きしない。
- 次周期に残差を持ち越さない。

solver失敗を理由に姿勢目標を積み増してはならない。

### 8.10 実現速度の算出

投影前後の状態から実現速度を計算する。

COM:

```cpp
realizedComVelocity =
  (p_FCOM_projected - p_FCOM_current) / dt;
```

CHEST:

```cpp
R_step = R_FC_projected * R_FC_current.transpose();
realizedTorsoAngularVelocity = rpyFromRot(R_step) / dt;
```

これはdebugおよび入出力関係の検証に用いる。実現できなかった速度を次周期へ加算しない。

---

## 9. 投影結果と既存COM/ZMP生成系の整合

### 9.1 問題意識

最終IKのCOM targetだけを変更すると、以下が不整合になる。

- `gaitParam.genCog`
- `genCogVel`
- `genCogAcc`
- `omega`
- `l`
- `refZmpTraj`
- Stabilizerが計算する目標ZMP・反力
- 実際の生成姿勢のCOM

したがって、投影済みCOMは最終IKだけでなく、static WBMS中の共通COM参照へ反映する。

### 9.2 `execAutoStabilizer()`内の呼び出し順

概念的に以下の順序にする。

```text
RefToGenFrameConverter
ActToGenFrameConverter
ExternalForceHandler
ImpedanceController
LegManualController
CmdVelGenerator
FootStepGenerator
LegCoordsGenerator::calcLegCoords
LegCoordsGenerator::calcCOMCoords       # nominal値
abcEETargetPose更新
WbmsPostureControl::proc                # ここでstatic WBMS時だけ上書き
Stabilizer
FullbodyIKSolver
```

`WbmsPostureControl::proc`はStabilizerより前に呼ぶ。

### 9.3 `genCog`等の更新

投影成功かつ操作モード有効時、投影robot COMを使って以下を更新する。

```cpp
genCog = projectedRobotCom - sbpOffset;
genCogVel = realizedProjectedCogVelocity;
genCogAcc = clamp((genCogVel - previousGenCogVel) / dt, safeAccelerationLimit);
```

実際には`wbmsOperationModeValue`でnominal値と投影値を滑らかにblendする。

```cpp
out = nominal * (1.0 - operationMode)
    + projected * operationMode;
```

加速度は不連続やノイズを避けるため、入力加速度limitと同程度にclampする。

### 9.4 COM高さと倒立振子パラメータ

投影後のCOM高さから、static WBMS中の有効高さを求める。

```cpp
newRefdz = max(projectedRobotCom.z() - footMidCoords.translation().z(), 0.1);
```

ExternalForceHandlerが計算した鉛直外力効果を極力維持するため、上書き前の`omega`と`refdz`から有効重力を求める。

```cpp
effectiveGravity = omega * omega * max(oldRefdz, 0.1);
refdz = blend(oldRefdz, newRefdz, operationMode);
l.z() = refdz;
omega = sqrt(effectiveGravity / max(refdz, 0.1));
```

`l.x/y`は既存値を維持する。

この処理はstatic WBMS時だけ行う。歩行中は既存の`refdz/omega/l`生成へ完全に戻す。

### 9.5 static WBMS用ZMP参照

投影済みCOMを準静的に保持するには、ZMP参照もCOM移動へ合わせる必要がある。

以下を基本とする。

```cpp
wbmsZmp = genCog - l;
wbmsZmp.x() -= genCogAcc.x() / (omega * omega);
wbmsZmp.y() -= genCogAcc.y() / (omega * omega);
```

- Zは支持面高さとする。
- XYは縮小支持多角形内へclampする。
- nominalな`refZmpTraj[0]`と`wbmsOperationModeValue`でblendする。
- static両足支持中は、blend後の値をstart/goalとする長さ1のtrajectoryへ更新してよい。
- 歩行中・歩行開始遅延中は既存trajectoryを変更しない。

体幹前傾に伴う角運動量を厳密に扱うことは今回の範囲外である。低速操作を前提とし、既存AngularMomentumConstraintを維持する。

---

## 10. 最終IKへの反映

### 10.1 reference IKの移動

現行`FullbodyIKSolver::calcWbmsPostureReference()`の責務は`WbmsPostureControl`へ移す。

以下を`FullbodyIKSolver`から削除する。

- `wbmsPostureRobot`
- `wbmsPostureFootConstraint`
- `wbmsPostureRootConstraint`
- `wbmsPostureTasks`
- 旧`calcWbmsPostureReference()`
- `FullbodyIKSolver`内部の`wbmsWalkingStabilityMode`

歩行安定化モードは`WbmsPostureControl`が更新し、`GaitParam`のvalueを最終IKが参照する。

### 10.2 CHEST姿勢拘束の追加

最終IKへCHEST姿勢用`PositionConstraint`を追加する。

- `A_link = genRobot->link(chestLinkName)`
- `A_localpos = Identity`
- `B_link = nullptr`
- `B_localpos.linear = wbmsProjectedChestR`
- 位置weight = 0
- 姿勢weight = `wbms_torso_orientation_weight * wbmsOperationModeValue`
- maxError = `wbms_torso_orientation_max_error * dt`
- 通常タスクと同じ優先度へ入れる。

この目標は投影IKで足・限界等を考慮した結果なので、生のユーザー目標を直接入れる場合より競合が少ない。

### 10.3 COM拘束

`WbmsPostureControl`がstatic WBMS中の`genCog`を整合済み値へ更新するため、最終IKのCOM targetは従来どおり以下でよい。

```cpp
cogTarget = gaitParam.genCog + gaitParam.sbpOffset;
```

ただしweightはモードで切り替える。

- static WBMS操作中: `wbms_com_position_weight`
- walkingまたは通常モード: 現行weight
- 遷移中: 線形blend

歩行中の通常COM XY weightとroot姿勢復帰を壊さない。

### 10.4 reference angle

最終IKの低優先度reference angleには以下を使う。

- 投影成功時: `wbmsPostureReferenceQ`
- 投影無効時: 従来の`refRobot`関節角
- 遷移中: `wbmsOperationModeValue`で両者をblend

投影変数に含まれない腕等の関節は、従来referenceを使う。

### 10.5 腕拘束

WBMS中の上半身エンドエフェクタは、現在のCHEST相対拘束をそのまま維持する。

```cpp
B_link = torsoGenLink;
B_localpos = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i];
```

CHEST相対拘束は共通ancestorの運動を相殺するため、体幹のworld姿勢操作と腕の相対操作を分離できる。

### 10.6 最終IK優先度

基本優先度を以下とする。

1. joint velocity / joint limit
2. self collision
3. 足EE
4. 上半身CHEST相対EE、投影済みCHEST姿勢、投影済みCOM、角運動量、歩行用root姿勢
5. 投影済みreference angle

既存の安全系・足拘束より上へユーザー指令を置かない。

---

## 11. モード遷移

### 11.1 モード値

現行ロジックを`WbmsPostureControl`へ移す。

```cpp
walkingStabilityTarget =
  (!gaitParam.isStatic() || gaitParam.isWbmsWalkingStartDelay) ? 1.0 : 0.0;

wbmsStabilityMode = max(1.0 - wbmsMode,
                        wbmsWalkingStabilityModeValue);

wbmsOperationMode = wbmsMode
                  * (1.0 - wbmsWalkingStabilityModeValue);
```

`wbmsOperationModeValue`は以下すべてに共通使用する。

- 適用速度の有効化。
- 投影COMとnominal COMのblend。
- CHEST最終拘束weight。
- COM最終拘束weight。
- reference angleのblend。
- static WBMS用ZMP参照のblend。

各モジュールが独自に異なるモード判定を持たないようにする。

### 11.2 歩行開始

歩行開始遅延へ入ったら以下とする。

- raw commandをclearする。
- applied commandを加速度limitでゼロへ戻す。
- operation modeを0へ補間する。
- COM/ZMP/CHEST目標をnominalへ戻す。
- 既存のroot姿勢、COM Z安定化weightを復帰する。
- future footstep生成開始前に姿勢復帰を進める。

### 11.3 歩行終了

静止へ戻るとoperation modeは徐々に1へ戻るが、速度commandはclear済みのため姿勢が勝手に動き始めてはならない。

新しいユーザー入力を受信した後だけ操作を再開する。

---

## 12. 計算量要件

### 12.1 必須条件

500 Hz周期、すなわち1周期2 ms以内を前提とする。

今回の投影IKは、現行reference IKを置換するものであり、solver呼び出し数を増やさないこと。

- 投影IK: 1反復
- 最終IK: 1反復
- 1周期内の再solveなし
- 外部非線形最適化なし

### 12.2 毎周期allocationの抑制

以下を`init()`で構築・reserveする。

- 投影IKのvariables。
- dqWeight。
- constraint object。
- 優先度別constraint vector。
- solver task cache。
- joint ID集合。

毎周期以下を行わない。

- robot clone。
- constraint objectのnew。
- ancestor chain探索。
- 不要なvectorの再確保。
- 全collision pairの追加。

self collision入力数が変化する場合のみ必要なresizeを許容する。

### 12.3 計測

少なくともdebug buildまたは一時ログで以下を計測できるようにする。

- `WbmsPostureControl::proc()`時間。
- `FullbodyIKSolver::solveFullbodyIK()`時間。
- `onExecute()`全体時間。
- 平均、最大、可能ならp99。

通常実行時に毎周期標準出力へ表示してはならない。debug levelまたは一定周期の集計表示にする。

受入時には、対象シミュレータ環境で500 Hzを継続できることを確認する。

---

## 13. Debug情報

新しい公開IDLサービスを増やす必要はないが、調整のため以下を確認可能にする。

- raw COM velocity。
- applied COM velocity。
- realized COM velocity。
- raw torso angular velocity。
- applied torso angular velocity。
- realized torso angular velocity。
- WBMS開始時からのCOM offset。
- WBMS開始時からのCHEST RPY offset。
- `wbmsOperationModeValue`。
- `wbmsWalkingStabilityModeValue`。
- projector valid flag。
- projector計算時間。

既存`DebugData`または既存debug OutPortへ、互換性を壊さない方法で追加する。既存`cpViewerLog`の固定indexを変更すると既存viewerへ影響する場合は、新規`TimedDoubleSeq` OutPortを追加する。

実装範囲を抑える必要がある場合でも、少なくとも内部変数とdebug print用コードは用意する。

---

## 14. 変更対象ファイル

### 14.1 必須

- `GaitParam.h`
  - 新規パラメータ・状態・投影結果を追加。
  - reset/clear処理を追加。
  - 旧`wbmsTorsoTargetRpy`と`refTorsoAnglVel`を廃止。

- `AutoStabilizer.cpp`
  - `refTorsoVelIn`のsample-and-hold化。
  - `vx/vy/vz`をCOM速度として保存。
  - start/stop/reset時の初期化。
  - `WbmsPostureControl::proc()`をStabilizer前へ挿入。
  - set/get parameter更新。

- `AutoStabilizer.h`
  - `WbmsPostureControl`メンバへ変更。
  - 必要ならdebug OutPort追加。

- `WbmsPostureControl.h`
- `WbmsPostureControl.cpp`
  - 速度command処理。
  - モード補間。
  - 投影IK。
  - COM/ZMP/omega/l更新。

- `FullbodyIKSolver.h`
  - 旧reference IKメンバを削除。
  - CHEST最終拘束を追加。

- `FullbodyIKSolver.cpp`
  - 旧`calcWbmsPostureReference()`削除。
  - `GaitParam`の投影結果を使用。
  - COM/CHEST/reference angleのモードblend。

- `MathUtil.h`
- `MathUtil.cpp`
  - 凸包縮小helperを追加。

- `auto_stabilizer/idl/AutoStabilizerService.idl`
  - 新規パラメータ追加。
  - 既存体幹パラメータコメントの意味を更新。

- `auto_stabilizer/rtc/AutoStabilizer/CMakeLists.txt`
  - `WbmsTorsoControl.cpp`を`WbmsPostureControl.cpp`へ変更。

### 14.2 ドキュメント

- `auto_stabilizer/docs`に、本書または実装結果を反映した作業記録を追加する。
- 実装後の実測値、採用パラメータ、残課題を記載する。

---

## 15. 実装手順

### Phase 1: 入力処理修正

1. `refTorsoVelIn`のlinear/angular成分をraw commandへ保存する。
2. 新着なしでゼロへ戻す処理を削除する。
3. timeout状態を追加する。
4. 加速度limitによるapplied commandを実装する。
5. start/stop/walking transitionでclearする。
6. 50 Hz入力時に一定applied velocityとなることを確認する。

この段階では旧reference IKへ接続せず、debug値だけ確認してよい。

### Phase 2: `WbmsPostureControl`作成

1. 旧`WbmsTorsoControl`を置換する。
2. walking stability modeを移す。
3. 投影robotとvariable集合をinitする。
4. 足・安全・CHEST・COM拘束を作る。
5. 1周期先目標とlimit処理を実装する。
6. 投影結果のvalid判定を実装する。

### Phase 3: COM/ZMP参照統合

1. 投影COMを`genCog/genCogVel/genCogAcc`へblendする。
2. `refdz/omega/l`を更新する。
3. static用ZMP参照を生成する。
4. Stabilizerより前に処理されることを確認する。

### Phase 4: 最終IK統合

1. 旧reference IKを`FullbodyIKSolver`から削除する。
2. CHEST姿勢拘束を追加する。
3. COM weightをモードblendする。
4. reference angleへ投影qを使う。
5. CHEST相対腕拘束が維持されていることを確認する。

### Phase 5: IDL・パラメータ

1. IDLへ新規項目を追加する。
2. set/getを実装する。
3. finite、length、lower/upper、非負チェックを実装する。
4. force-cmakeでビルドする。

### Phase 6: 計測と調整

1. projector、final IK、onExecuteの時間を測る。
2. 体幹単独、COM単独、腕同時、歩行遷移を確認する。
3. 初期weightとlimitを保守的な範囲で調整する。
4. docsへ結果を書く。

---

## 16. 検証項目

テストコードは作成せず、シミュレータで以下を確認する。

### 16.1 50 Hz入力保持

- 500 HzのAutoStabilizerへ50 Hzで一定`vp`を送る。
- applied pitch velocityが周期間でゼロへ落ちない。
- 実現CHEST pitchが概ね等速で変化する。
- 入力周期を50 Hzから100 Hzへ変えても速度が大きく変化しない。

### 16.2 timeout

- 一定入力後、送信を停止する。
- `wbms_velocity_command_timeout`まではholdする。
- timeout後、加速度limitに従って速度がゼロへ戻る。
- 目標姿勢がその後勝手に進み続けない。

### 16.3 体幹単独

- roll、pitch、yawを個別に操作する。
- rootではなくCHEST世界姿勢が操作意図へ追従する。
- limitへ到達すると滑らかに停止する。
- 同じ方向入力を続けてもhidden goalが蓄積しない。
- 逆入力で直ちにlimitから離れる。

### 16.4 COM単独

- `vz < 0`で一定速度のしゃがみ。
- `vz > 0`で一定速度の立ち上がり。
- COM高さlimitで停止する。
- `vx/vy`で小さな重心移動が可能。
- COM/ZMPが支持多角形margin外へ出ない。
- 入力停止後、位置を概ね保持する。

### 16.5 前屈＋しゃがみ

- pitch前傾とCOM下降を同時に入力する。
- CHEST関節だけでなく股関節・膝・足首を含む全身姿勢になる。
- 旧方式のように、入力意図と無関係に腰を落とし切る挙動がない。
- 床の物体へ手を伸ばせる範囲が改善する。

### 16.6 腕同時操縦

- 両腕6自由度操作と体幹pitch、COM下降を同時に行う。
- 手先がworld固定のように体幹運動を阻害しない。
- 腕はCHEST相対で操作できる。
- 足踏み、yaw振動、急激な下半身振動が発生しない。

### 16.7 到達不能姿勢

- 関節limitまたは自己干渉へ近づく入力を継続する。
- 実現速度が低下またはゼロになる。
- solverが不安定化しない。
- 入力を反転すると即座に戻る。
- 数秒後に蓄積目標を追いかける挙動がない。

### 16.8 歩行遷移

- 前傾・しゃがみ状態から`goVelocity()`を送る。
- 歩行開始遅延中にoperation modeが0へ戻る。
- COM/ZMP/CHEST参照がnominalへ滑らかに戻る。
- future footstep生成前に姿勢安定化が進む。
- 下半身の瞬間的振動が再発しない。
- 歩行終了後、古い体幹・COM速度commandが再開しない。
- 腕操縦と歩行は従来どおり併用できる。

### 16.9 stop/start

- WBMS停止・再開時に姿勢が不連続に変化しない。
- baselineが再取得される。
- 旧command、旧projected target、旧solver状態が残らない。

### 16.10 計算時間

- 500 Hzで継続動作する。
- 投影IK追加による周期超過が継続しない。
- self collision pairが増えた場合も異常な最大時間が出ないか確認する。

---

## 17. 受入基準

以下をすべて満たした場合に実装完了とする。

1. 50 Hzの一定入力で、体幹・COMが入力周期に依存せず概ね一定速度で動く。
2. ユーザーは体幹姿勢とCOM高さを独立に調整できる。
3. 到達不能入力を継続してもhidden goalが蓄積しない。
4. 制約到達後、逆方向入力へ即座に反応する。
5. 両腕操縦と同時に使用できる。
6. 腕のCHEST相対拘束が維持される。
7. 既存歩行APIと歩行開始遅延が維持される。
8. 歩行中はCOM操縦が無効化され、通常安定化へ戻る。
9. 足拘束、関節limit、自己干渉をユーザー指令より優先する。
10. 500 Hz周期実行を維持する。
11. IDLのset/getで全新規パラメータを読み書きできる。
12. ビルドが成功する。

---

## 18. 実装してはならない方式

過去の失敗を繰り返さないため、以下は採用しない。

- root姿勢をユーザー指令として最終IKへ直接強く拘束する。
- CHEST姿勢の到達不能な絶対目標を別状態へ積分し続ける。
- COM Zを、生の積分目標として最終IKへ後付けする。
- COM入力から股関節・膝・足首の固定係数offsetを手作業で生成する。
- 腕姿勢weightを単純に下げるだけで過拘束を解消しようとする。
- 体幹角速度をrootのAngularVelocityConstraintへ直接入れる。
- 最終IKの足・安全制約より上位へユーザー指令を置く。
- 歩行中にもstatic時と同じCOM速度操作を有効にする。
- 1周期中に複数回QPを解いて計算時間を増やす。
- 通信データが新着でないだけで速度をゼロへ戻す。

---

## 19. 実装意図のまとめ

今回の中心的な変更は、操縦入力を「達成すべき絶対姿勢」として扱うのではなく、「現在状態から次の1周期に実現可能な速度」として扱うことである。

この方式では、毎周期の目標が小さく、足・関節limit・自己干渉を考慮した投影結果だけが最終IKとStabilizerへ渡る。達成できなかった差分は捨てられるため、長時間入力後に内部目標が残る問題がない。

また、腕はCHEST相対、体幹はCHEST世界姿勢、COMはfoot-mid基準速度として責務を分離する。これにより、腕操縦、体幹姿勢、しゃがみ、既存歩行を同一の硬い拘束集合へ直接押し込まず、既存AutoStabilizerの安全・安定化構造を活用できる。

実装では厳密な人間姿勢再現や最適な全身動作を目標にしない。ユーザーが一定入力と一定応答の関係を理解でき、制約内で容易に姿勢を調整できることを優先する。
