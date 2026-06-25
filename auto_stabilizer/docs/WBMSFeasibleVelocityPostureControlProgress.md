# WBMS実現可能速度投影型・体幹/COM操縦 進捗記録

## 現在の全体状態

対象ブランチは `wbms-dev`。現在の作業は、承認済み実装計画の「マイルストーン1: API・状態・入力保持基盤」まで完了している。

本記録時点で、以下はコード上実装済みである。

- `WbmsTorsoControl` から `WbmsPostureControl` への置換。
- `refTorsoVelIn` の sample-and-hold 化。
- `vx/vy/vz` をCOM速度、`vr/vp/va` をCHEST角速度として保持する入力処理。
- timeout、操作不可、invalid時に desired command をゼロにし、applied command を成分ごとの加速度limitで追従させる基盤。
- WBMS開始時baseline、raw/applied command、command age/valid、投影結果、mode値の状態追加。
- WBMS停止、AutoBalancer停止、RTC activate/deactivate、`MODE_SYNC_TO_ABC` 初期化、歩行開始遅延移行での stale command clear。
- 正式仕様書6.2の新規WBMS IDL parameter追加と set/get 反映。
- 旧root-reference WBMS姿勢IKへの新command接続回避。

本記録時点で、以下は未実装であり、後続マイルストーン範囲である。

- 投影IKによる実現可能速度投影。
- 投影結果のCOM/ZMP/refdz/omega/l統合。
- `MathUtil::shrinkConvexHull2D()` 相当の支持多角形縮小helper。
- 最終IKへのCHEST姿勢拘束、COM weight blend、投影reference angle統合。
- debug OutPortまたはdebug logの公開整備。
- シミュレータ上の応答確認と500 Hz実測。

## 完了済みマイルストーン一覧

| マイルストーン | 名称 | 状態 |
|---|---|---|
| 1 | API・状態・入力保持基盤 | 完了。ビルド成功、review指摘対応済み |
| 2 | 実現可能速度投影とCOM/ZMP統合 | 未着手 |
| 3 | 最終IK統合・旧方式削除・debug/計測 | 未着手 |

## 今回のマイルストーン完了記録

### マイルストーン番号と名称

マイルストーン1: API・状態・入力保持基盤。

### 完了状態

完了。`catkin build auto_stabilizer --no-deps --force-cmake` に成功している。reviewで報告されたP2指摘2件は修正済みで、最終reviewでは今回範囲に明確な欠陥は報告されていない。

### 実装した範囲

- `WbmsTorsoControl.h/.cpp` を削除し、`WbmsPostureControl.h/.cpp` を追加。
- `AutoStabilizer.h` のincludeとメンバを `WbmsPostureControl` へ更新。
- `CMakeLists.txt` のビルド対象を `WbmsPostureControl.cpp` へ更新。
- `GaitParam` に以下を追加。
  - `wbmsRawComVelocityCommand`
  - `wbmsRawTorsoAngularVelocityCommand`
  - `wbmsAppliedComVelocityCommand`
  - `wbmsAppliedTorsoAngularVelocityCommand`
  - `wbmsVelocityCommandAge`
  - `wbmsVelocityCommandValid`
  - `wbmsStartChestRInFootMid`
  - `wbmsStartComInFootMid`
  - `wbmsPostureBaselineValid`
  - `wbmsPostureReferenceQ`
  - `wbmsProjectedChestR`
  - `wbmsProjectedRobotCom`
  - `wbmsRealizedComVelocity`
  - `wbmsRealizedTorsoAngularVelocity`
  - `wbmsPostureReferenceValid`
  - `wbmsWalkingStabilityModeValue`
  - `wbmsOperationModeValue`
- `GaitParam` から旧 `refTorsoAnglVel` と `wbmsTorsoTargetRpy` を削除。
- `readInPortData()` で、finiteな `refTorsoVelIn` 新着時のみ raw command、command age、valid を更新するよう変更。
- 新着がない周期では raw command を変更しない。
- invalid入力時は raw command を上書きせず、valid false とする。
- `WbmsPostureControl::updateVelocityCommand()` で command age、timeout、操作可否、加速度limitを処理。
- qRefが届かない周期でも `updateVelocityCommand()` を呼び、timeoutとゼロ復帰を進める。
- `WbmsPostureControl::start()` でWBMS開始時CHEST姿勢とrobot COMのfoot-mid基準baselineを取得。
- `clearWbmsPostureCommand()` / `clearWbmsPostureReference()` / `clearStaleCommand()` を追加。
- `stopWholeBodyMasterSlave()`, `stopAutoBalancer()`, RTC activate/deactivate, `MODE_SYNC_TO_ABC` 初期化、歩行開始遅延移行で stale command をclear。
- `FullbodyIKSolver` から旧root-reference WBMS姿勢IKの内部ロボット・constraint・task・関数を削除。
- `FullbodyIKSolver` のreference angleは従来 `refRobot` 由来とし、新commandを旧root reference IKへ接続しない状態にした。
- IDLに正式仕様書6.2の新規parameterを追加。
- `setAutoStabilizerParam()` / `getAutoStabilizerParam()` へ新規parameterを反映。
- 新規parameterについて length、finite、非負値、lower/upper正規化を実装。

### 意図的に未実装とした後続範囲

承認済み実装計画に従い、M1では以下を実装していない。

- 投影IKのconstraints、variables、task cache構築。
- `WbmsPostureControl::proc()` のStabilizer前COM/ZMP上書き。
- 投影robotからの `wbmsPostureReferenceQ` 更新。
- `wbmsProjectedChestR` / `wbmsProjectedRobotCom` / realized velocity の毎周期計算。
- 最終IKへのCHEST姿勢拘束追加。
- COM weightの `wbmsOperationModeValue` blend。
- `wbmsPostureReferenceQ` を最終IK reference angleへ接続する処理。
- support polygon shrinkとCOM XY射影。
- debug OutPortまたはcpViewerLog拡張。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/idl/AutoStabilizerService.idl` | WBMS新規parameter追加。既存体幹parameterのコメントをCHEST/COM速度投影仕様へ更新 |
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | raw/applied command、age/valid、baseline、投影結果、mode値、新規parameter、clear/reset関数を追加。旧体幹積分状態を削除 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h` | `WbmsPostureControl` include/メンバへ更新。`readInPortData()` シグネチャ更新 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | sample-and-hold入力処理、qRef未更新時timeout更新、start/stop/reset clear、set/get parameter検証を実装 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h` | 新クラス宣言を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | baseline取得、速度command更新、操作可否判定、加速度limit、mode値更新を実装 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsTorsoControl.h` | 削除 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsTorsoControl.cpp` | 削除 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp` | 歩行開始遅延移行時に raw command をclear |
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h` | 旧root-reference WBMS姿勢IK用メンバを削除 |
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp` | 旧 `calcWbmsPostureReference()` を削除し、新command未接続のreference angle経路へ整理 |
| `auto_stabilizer/rtc/AutoStabilizer/CMakeLists.txt` | `WbmsPostureControl.cpp` へ更新 |

### 追加・変更した主要class、function、state、parameter

#### class

- `WbmsPostureControl`
  - 速度command処理、WBMS操作可否、mode値更新、WBMS開始時baseline取得の担当。
  - M1時点では投影IK本体、COM/ZMP統合は未実装。

#### function

- `WbmsPostureControl::init()`
  - M2で使う投影用robotのcloneを初期化。
- `WbmsPostureControl::reset()`
  - walking stability mode補間器をリセット。
- `WbmsPostureControl::start()`
  - WBMS開始時にraw/applied command、投影結果をclearし、CHEST姿勢とCOMのfoot-mid baselineを取得。
- `WbmsPostureControl::clearStaleCommand()`
  - stale commandと投影結果をclear。
- `WbmsPostureControl::updateVelocityCommand()`
  - command age更新、timeout、操作不可、invalid、加速度limit追従を処理。
- `WbmsPostureControl::proc()`
  - walking stability mode、operation mode、速度command更新を処理。
- `GaitParam::clearWbmsPostureCommand()`
  - raw command、age、validをclearし、必要に応じてapplied commandもゼロ化。
- `GaitParam::clearWbmsPostureReference()`
  - 投影結果状態を安全側へclear。
- `GaitParam::resetWbmsPostureControl()`
  - command、投影結果、baseline、mode値を初期化。

#### state

次のM2/M3が利用する主な状態は `GaitParam` に追加済み。

- raw/applied COM velocity command
- raw/applied CHEST angular velocity command
- command age/valid
- WBMS開始時CHEST姿勢・COM baseline
- projection reference q
- projected CHEST/COM
- realized COM/CHEST velocity
- projection valid flag
- `wbmsWalkingStabilityModeValue`
- `wbmsOperationModeValue`

#### parameter

IDLと `GaitParam` に以下を追加。

- `wbms_velocity_command_timeout`
- `wbms_torso_angular_acceleration_limit`
- `wbms_com_velocity_limit`
- `wbms_com_acceleration_limit`
- `wbms_com_offset_lower_limit`
- `wbms_com_offset_upper_limit`
- `wbms_com_position_weight`
- `wbms_com_xy_support_margin`

既存parameterの意味を以下へ更新。

- `wbms_torso_angular_velocity_limit`: CHEST角速度limit。
- `wbms_torso_rpy_lower_limit/upper_limit`: WBMS開始時CHEST姿勢からの実現姿勢差分limit。
- `wbms_torso_orientation_weight`: 投影IKおよび最終IKのCHEST姿勢weight。
- `wbms_torso_orientation_max_error`: 1秒あたりの最大姿勢補正量。

## 仕様との差異

### M1で旧reference IKの一部削除を先行した

承認済み計画では旧reference IKの本格削除はM3に含まれるが、M1で旧 `refTorsoAnglVel` / `wbmsTorsoTargetRpy` を削除し、かつ新commandを旧root reference IKへ接続しない必要があった。このため、`FullbodyIKSolver::calcWbmsPostureReference()` と旧root-reference WBMS姿勢IK専用メンバをM1で削除した。

これは新しい制御機能の先行実装ではない。M1終了時点で、新commandは最終IKにも旧root reference IKにも接続されていない。

### `wbmsPostureRobot_` はM1でcloneのみ保持

正式仕様の投影IK本体はM2範囲である。M1では `WbmsPostureControl::init()` で投影用robot cloneを作り、`start()` で現在 `genRobot` へ同期するところまで実装している。毎周期投影IKには使っていない。

### qRef未更新時の速度command更新

review指摘対応として、qRef未更新で出力更新をskipする場合でも `WbmsPostureControl::updateVelocityCommand()` だけは実行する。これは正式仕様5.2の「毎周期 command age を増加」と stale command 防止のためであり、出力やIKを追加実行するものではない。

### WBMS停止fade-out中の操作不可判定

review指摘対応として、`WbmsPostureControl::isOperationAllowed()` は `wbmsMode.getGoal() > 0.0` をWBMS操作可否条件に使う。停止fade-out中は `wbmsMode.value()` が正でも `getGoal()==0` であり、新しい速度入力はdesiredへ採用しない。

## ビルド・review結果

### ビルドコマンドと結果

IDL変更を含むため、以下を実行した。

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

結果は成功。OpenRTM生成系の既存YAML warningは出ているが、CMake、IDL生成、コンパイル、リンクはいずれも成功している。

### 静的確認コマンドと結果

以下を実行した。

```sh
rg -n "refTorsoAnglVel|wbmsTorsoTargetRpy|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
```

結果はヒットなし。旧体幹積分状態と旧class名は `AutoStabilizer` 制御コードから除去されている。

以下を実行した。

```sh
rg -n "wbms_velocity_command_timeout|wbms_com_velocity_limit|wbmsOperationModeValue" auto_stabilizer
```

結果は想定どおり、IDL、仕様/計画docs、`GaitParam`、set/get、`WbmsPostureControl` にヒットした。

### `/review`で報告された重要な指摘と対応

#### 指摘1: qRef停止中も速度指令timeoutを進める必要がある

- 分類: 修正対象。
- 問題: qRef未更新で早期returnすると `WbmsPostureControl::proc()` が呼ばれず、command ageとapplied commandのゼロ復帰が止まる。
- 修正:
  - `WbmsPostureControl::updateVelocityCommand()` を追加。
  - 通常周期では `proc()` から呼ぶ。
  - qRef未更新時の早期return前にも `updateVelocityCommand()` を呼ぶ。
- 確認:
  - qRef未更新時にも `wbmsVelocityCommandAge += dt` と加速度limitによるdesired追従が実行されるコード構造を確認。
  - ビルド成功。

#### 指摘2: WBMS停止中は速度commandを受け付けない

- 分類: 修正対象。
- 問題: `stopWholeBodyMasterSlave()` 後のfade-out中は `wbmsMode.value()` が正のため、teleopが継続publishするとclear後に速度commandが再有効化し得る。
- 修正:
  - `WbmsPostureControl::isOperationAllowed()` のWBMS判定を `wbmsMode.getGoal() > 0.0` に変更。
  - 停止fade-out中は `getGoal()==0` のため操作不可となり、desiredはゼロになる。
- 確認:
  - `stopWholeBodyMasterSlave()` が `wbmsMode.setGoal(0.0, 5.0)` と `clearStaleCommand(..., true)` を呼ぶことを確認。
  - ビルド成功。

#### 最終review

最終reviewでは、sample-and-hold、timeout/acceleration limit、stale command reset経路、IDL set/get対称性、初期化、旧root-reference IKへの接続回避について明確な欠陥は報告されていない。M2/M3相当の未実装部分は、指示どおり欠陥として扱われていない。

## コードとビルドで確認済みの事項

- `refTorsoVelIn` 新着なし周期で raw command をゼロ化しない。
- finiteな新着時のみ raw command、age、validを更新する。
- `vx/vy/vz` はCOM速度、`vr/vp/va` はCHEST角速度として保存する。
- qRef未更新中でも command age とapplied command更新が進む。
- timeout、操作不可、invalid時は desired がゼロになる。
- applied commandは成分ごとの加速度limitでdesiredへ追従する。
- `wbms_interpolate_duration` は速度command処理には使っていない。
- WBMS開始時にCHEST姿勢とrobot COMのfoot-mid基準baselineを取得する。
- WBMS開始時にraw/applied command、age、valid、投影結果を安全側へ初期化する。
- WBMS停止、AB停止、RTC activate/deactivate、sync初期化、歩行開始遅延移行時に stale command をclearする。
- WBMS停止fade-out中は速度commandを操作可能扱いしない。
- IDL新規parameterのset/getは対称に実装されている。
- IDL sequenceの長さ、finite、非負値、lower/upper関係はset側で検証されている。
- 旧 `WbmsTorsoControl` 依存は除去されている。
- 新しいcommandは旧root reference IKへ接続されていない。
- buildは成功している。

## 未解決事項

- M2の投影IK本体は未実装。
- M2のCOM/ZMP/refdz/omega/l統合は未実装。
- M2の支持多角形縮小helperとCOM XY射影は未実装。
- M3の最終IK CHEST姿勢拘束とCOM weight blendは未実装。
- M3のdebug出力方式は未決定。`cpViewerLog` 固定indexを壊す可能性があるため、既存viewer互換を確認してから決める。
- `wbmsPostureReferenceQ` など投影結果状態はM1で入れ物のみ用意されており、毎周期の投影結果では更新されていない。
- シミュレータ確認前のため、実応答、操作感、500 Hz実測は未確認。
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp/.h` は新規未追跡ファイルとして存在する。コミット前に追加対象として確認すること。
- `AGENTS.md`、`auto_stabilizer/docs/`、`auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json` は未追跡として存在する。既存ユーザー変更扱いで、不要に削除しないこと。

## シミュレータ確認待ち項目

以下はコードとビルドだけではPASS扱いしない。

- 50 Hz入力を500 Hz制御で受けたとき、applied velocityが新着なし周期でゼロ落ちしないこと。
- 入力停止後、`wbms_velocity_command_timeout` まではholdし、その後加速度limitに従ってゼロへ戻ること。
- qRef停止がtimeoutより長く続いた後に復帰しても、古い速度commandが再開しないこと。
- WBMS停止fade-out中にteleopが継続publishしても、applied velocityが再上昇しないこと。
- WBMS停止後に再startしたとき、新しい入力受信まで古いcommandが再開しないこと。
- 歩行開始遅延へ入った後、古いcommandが歩行終了後に再開しないこと。
- 腕のCHEST相対拘束が従来どおり維持されること。
- 既存歩行APIと歩行開始遅延が壊れていないこと。
- 500 Hz周期が継続すること。

## 次マイルストーンへの引き継ぎ

### 次のマイルストーンが使用するinterface、state、前提条件

M2は以下を前提に開始する。

- `WbmsPostureControl` が存在し、`AutoStabilizer` にメンバとして保持されている。
- `WbmsPostureControl::init()` で投影用robot cloneが作られている。
- `WbmsPostureControl::start()` で `genRobot` 由来のCHEST姿勢・robot COM baselineがfoot-mid基準で取得される。
- `GaitParam` にraw/applied command、age/valid、baseline、投影結果、mode値が存在する。
- `WbmsPostureControl::proc()` は毎周期呼ばれており、M2でStabilizer前のCOM/ZMP統合処理へ拡張する入口として使える。
- `wbmsOperationModeValue` は `wbmsMode.value() * (1 - wbmsWalkingStabilityModeValue)` として更新されている。
- `WbmsPostureControl::isOperationAllowed()` は、AB running、WBMS有効目標、static、非歩行開始遅延、両足support、脚manual control無効を判定する。
- 新commandはまだ最終IKにも旧reference IKにも接続されていない。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
  - `readInPortData()`
  - `onExecute()`
  - `execAutoStabilizer()`
  - `startWholeBodyMasterSlave()`
  - `stopWholeBodyMasterSlave()`
  - `setAutoStabilizerParam()`
  - `getAutoStabilizerParam()`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
  - 腕CHEST相対拘束の既存処理。
  - M3まで新commandを接続しない前提のreference angle処理。
- `auto_stabilizer/rtc/AutoStabilizer/MathUtil.h/.cpp`
  - M2で支持多角形縮小helperを追加する候補。
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlCodexPlan.md`

### 次に実行すべきビルド・確認コマンド

M2作業開始前に以下を実行し、M1完了状態から崩れていないことを確認する。

```sh
git status --short
catkin build auto_stabilizer --no-deps
rg -n "refTorsoAnglVel|wbmsTorsoTargetRpy|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
rg -n "wbms_velocity_command_timeout|wbms_com_velocity_limit|wbmsOperationModeValue" auto_stabilizer
```

M2実装後は承認済み計画に従い、少なくとも以下を実行する。

```sh
catkin build auto_stabilizer --no-deps
rg -n "clone\\(|ancestor|wbmsPostureRobot|shrinkConvexHull2D|calcNearestPointOfHull" auto_stabilizer/rtc/AutoStabilizer
rg -n "WbmsPostureControl::proc|execStabilizer|calcCOMCoords" auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp
```
