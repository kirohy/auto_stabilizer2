# WBMS実現可能速度投影型・体幹/COM操縦 進捗記録

## 文書参照順とマイルストーンindex

後続スレッドでは、まず本Progress文書で最新の作業状態、作業ログ、計測ログを確認する。仕様判断や実装方針は、対象範囲ごとに後続の計画書を優先する。

本Progress文書は時系列の作業記録であり、下記の各時点の状態記述よりも、より後方に追記された作業記録を新しい情報として扱う。

推奨参照順:

1. `WBMSFeasibleVelocityPostureControlProgress.md`: 全体の作業履歴、最新ログ、実施済み修正の確認。
2. `WBMSComputationReductionImplementationPlan.md`: `M5: 500 Hz計算量削減` の計画。計算量削減ではこれを優先する。
3. `WBMSWalkingPreparationDesignRevisionPlan.md`: `M4.2.2設計修正` の最新計画。歩行準備遷移の判断ではこれを優先する。
4. `WBMSWalkingPreparationTransitionImplementationPlan.md`: `M4.2.2設計修正` の前提となる歩行準備遷移計画。
5. `WBMSProjectionAcceptanceFixImplementationPlan.md`: `M4.1` から `M4.2` 系のprojector候補採用判定修正計画。
6. `WBMSFeasibleVelocityPostureControlImplementationPlan.md`: 初期M1からM3の基礎仕様。後続文書と矛盾する場合は後続文書を優先する。
7. `WBMSFeasibleVelocityPostureControlCodexPlan.md`: 初期M1からM3の作業分割・引き継ぎ計画。現行仕様の優先元としては扱わない。

背景資料として残す文書:

- `WBMSWalkingControlSummary.md`: WBMS中の上半身操縦と歩行制御を整理した初期引き継ぎ資料。歩行準備遷移の最新仕様は `WBMSWalkingPreparationDesignRevisionPlan.md` を優先する。
- `WBMSTorsoArmIKDesignPlan.md`: 体幹・腕協調IKの初期設計案。現行仕様ではなく、背景資料として扱う。
- `WBMSTorsoArmIKExperimentLog.md`: 体幹・腕IKの古い試行錯誤と不採用理由の実験ログ。現行仕様ではなく、判断時の参考資料として扱う。

## 現在の全体状態

対象ブランチは `wbms-dev`。現在の作業は、承認済み実装計画の「マイルストーン3: 最終IK統合・旧方式削除・debug/計測」まで完了している。

本記録時点で、以下はコード上実装済みである。

- `WbmsTorsoControl` から `WbmsPostureControl` への置換。
- `refTorsoVelIn` の sample-and-hold 化。
- `vx/vy/vz` をCOM速度、`vr/vp/va` をCHEST角速度として保持する入力処理。
- timeout、操作不可、invalid時に desired command をゼロにし、applied command を成分ごとの加速度limitで追従させる基盤。
- WBMS開始時baseline、raw/applied command、command age/valid、投影結果、mode値の状態追加。
- WBMS停止、AutoBalancer停止、RTC activate/deactivate、`MODE_SYNC_TO_ABC` 初期化、歩行開始遅延移行での stale command clear。
- 正式仕様書6.2の新規WBMS IDL parameter追加と set/get 反映。
- 旧root-reference WBMS姿勢IKへの新command接続回避。
- 投影用robot、投影IK variables、constraint object、優先度別constraint vector、task cache、支持多角形用work bufferの初期化時構築。
- 毎周期、現在の `genRobot` を投影用robotへ同期し、1周期先だけを投影する処理。
- 投影IK変数の root、左右脚ancestor、CHEST ancestor、かつ `jointControllable=true` の関節への限定。
- CHEST姿勢目標をrootではなく `gaitParam.chestLinkName` の世界姿勢として扱う処理。
- CHEST角速度とCOM速度をfoot-mid基準で扱い、WBMS開始時baselineからの差分limitへ制限する処理。
- `MathUtil::shrinkConvexHull2D()` と、allocation抑制用のbuffer渡し凸包helper。
- 両足支持領域から縮小支持多角形を作り、COM XY目標とZMP XYを縮小支持多角形内へ射影する処理。
- joint velocity、joint limit、self collision、両足、CHEST/COM、低weight姿勢参照の優先度で投影IKを構築する処理。
- 投影solverを1周期1回、`maxIteration=1` で呼ぶ処理。
- solver戻り値に加え、finite、関節limit、足誤差、異常な1周期変位を独自検証する処理。
- invalid時に現在姿勢へfallbackし、COM/ZMP統合を行わず、hidden goalや残差を次周期へ持ち越さない処理。
- 投影成功時に `wbmsPostureReferenceQ`、projected CHEST/COM、realized velocity、valid flagを `GaitParam` へ反映する処理。
- static WBMS操作中だけ、投影COMを `genCog`、`genCogVel`、`genCogAcc`、`refdz`、`omega`、`l`、static用 `refZmpTraj` へblend反映する処理。
- `WbmsPostureControl::proc()` を `calcCOMCoords` および `abcEETargetPose` 更新後、Stabilizerより前に呼ぶ処理。
- 最終IKへ投影済みCHEST姿勢 `PositionConstraint` を追加する処理。
- 最終IKのCOM targetを `genCog + sbpOffset` のまま維持し、COM weightだけを通常値と `wbms_com_position_weight` の間で `wbmsOperationModeValue` によりblendする処理。
- 最終IKのreference angleを、投影validかつ投影IK variableに含まれる関節だけ `wbmsPostureReferenceQ` と従来 `refRobot` の間でmode blendする処理。
- 投影variable外の腕などの関節へ従来referenceを使う処理。
- 投影invalid時に最終IKのCHEST姿勢拘束と投影reference angleを無効化し、従来referenceへfallbackする処理。
- 上半身EEのWBMS中CHEST相対拘束を維持する処理。
- 旧root reference IK関連識別子と旧 `WbmsTorsoControl` の削除。
- 既存 `cpViewerLog` の固定indexを変更せず、新規 `wbmsDebugOut` OutPortでWBMS debug情報と計算時間を公開する処理。
- projector、final IK、`onExecute()` の時間計測。通常実行で毎周期標準出力しない。

本記録時点で、以下は未実装であり、後続マイルストーン範囲である。

- シミュレータ上での安定性、操作感、腕同時操作、歩行遷移、逆方向応答、500 Hz実時間性能の合否判定。
- `wbmsDebugOut` を用いた運用側logger/viewer設定。
- `wbmsDebugOut` にprojector invalid理由や `solveIKLoop()` 戻り値を追加公開するかどうかの判断。
- projector validが常にfalseになるシミュレータ問題の原因切り分けと修正。

## 完了済みマイルストーン一覧

| マイルストーン | 名称 | 状態 |
|---|---|---|
| 1 | API・状態・入力保持基盤 | 完了。ビルド成功、review指摘対応済み |
| 2 | 実現可能速度投影とCOM/ZMP統合 | 完了。ビルド成功、review指摘対応済み |
| 3 | 最終IK統合・旧方式削除・debug/計測 | 完了。ビルド成功、review指摘対応済み。シミュレータでprojector invalid問題を確認 |

## 完了済みマイルストーン1記録

### 実装した範囲

- `WbmsTorsoControl.h/.cpp` を削除し、`WbmsPostureControl.h/.cpp` を追加。
- `AutoStabilizer.h` のincludeとメンバを `WbmsPostureControl` へ更新。
- `GaitParam` にraw/applied command、command age/valid、WBMS開始時baseline、投影結果、mode値を追加。
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

### M1 review対応

- qRef停止中も速度指令timeoutを進める必要がある指摘を修正した。
  - qRef未更新時の早期return前にも `updateVelocityCommand()` を呼ぶ。
- WBMS停止中は速度commandを受け付けない必要がある指摘を修正した。
  - `isOperationAllowed()` のWBMS判定を `wbmsMode.getGoal() > 0.0` にした。

## 完了済みマイルストーン2記録

### マイルストーン番号と名称

マイルストーン2: 実現可能速度投影とCOM/ZMP統合。

### 完了状態

完了。`catkin build auto_stabilizer --no-deps` に成功している。M2実装後の `/review` 指摘は修正または仕様根拠に基づいて対応不要判断済みである。

### 実装した範囲

- `WbmsPostureControl::init()` で投影用robot、IK variables、constraint object、優先度別constraint vector、task cache、支持多角形work buffer、IKParamを構築。
- 投影IK variableをfloating root、左右脚ancestor、CHEST ancestorのunionに限定し、関節は `jointControllable=true` のものだけに限定。
- 毎周期 `genRobot` のroot姿勢と全関節角を投影用robotへ同期し、FK/COMを更新。
- CHEST目標を `gaitParam.chestLinkName` の世界姿勢として計算。
- CHEST角速度をfoot-mid基準で1周期先へ積分し、WBMS開始時CHEST姿勢からのRPY差分limitへclamp。
- COM目標をrobot生COMとして扱い、foot-mid基準で1周期先へ積分し、WBMS開始時COMからのoffset limitへclamp。
- 両足 `legHull` からfoot-mid基準の支持凸包を作り、`wbmsComXYSupportMargin` で縮小支持多角形を生成。
- COM XY目標を縮小支持多角形へ射影。
- 支持多角形無効時、縮小支持多角形退化時、または過大margin時は投影全体を失敗扱いにし、現在姿勢へfallback。
- 投影IK拘束を以下の優先度で構築。
  - 優先度0: joint velocity、joint limit。
  - 優先度1: 近いself collisionのみ。
  - 優先度2: 両足6D拘束。
  - 優先度3: CHEST姿勢3D、COM位置3D。
  - 優先度4: 必要時の小weight姿勢参照。
- `solveIKLoop()` を1周期1回、`maxIteration=1` で呼ぶ。
- 投影結果をfinite、関節limit、足誤差、root異常変位で独自検証。
- invalid時は `wbmsPostureReferenceQ` を現在姿勢へ戻し、projected CHEST/COMを現在値へ戻し、realized velocityをゼロにし、valid flagをfalseにする。
- valid時は `wbmsPostureReferenceQ`、`wbmsProjectedChestR`、`wbmsProjectedRobotCom`、`wbmsRealizedComVelocity`、`wbmsRealizedTorsoAngularVelocity`、`wbmsPostureReferenceValid` を更新。
- static WBMS操作中、かつ投影valid時だけ、投影COMを `genCog`、`genCogVel`、`genCogAcc`、`refdz`、`omega`、`l`、static用 `refZmpTraj` へblend反映。
- ZMP XYを縮小支持多角形内へ射影。
- `WbmsPostureControl::proc()` を `AutoStabilizer::execAutoStabilizer()` 内で `calcCOMCoords` と `abcEETargetPose` 更新後、Stabilizer前に呼ぶ。

### 意図的に未実装とした後続範囲

承認済み実装計画に従い、M2では以下を実装していない。

- 最終IKへの投影CHEST拘束追加。
- 最終IKのCOM weight切替。
- 最終IKのreference angleを `wbmsPostureReferenceQ` へ切り替える処理。
- 腕CHEST相対拘束の変更。
- debug OutPort最終仕様。
- 500 Hz実時間計測。
- 後続で削除する別の暫定projector。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | `WbmsPostureControl::init()` に `GaitParam` を渡す。`execAutoStabilizer()` で `calcCOMCoords` / `abcEETargetPose` 更新後、Stabilizer前に `wbmsPostureControl.proc()` を呼ぶ |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h` | `WbmsPostureControl` 初期化・呼び出し変更に合わせた宣言更新 |
| `auto_stabilizer/rtc/AutoStabilizer/MathUtil.h` | `calcConvexHull()` のbuffer渡しoverload、`shrinkConvexHull2D()` の戻り値版とbuffer渡し版を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/MathUtil.cpp` | buffer渡し凸包生成、縮小支持多角形生成、過大margin/退化検出を実装 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h` | 投影robot、variable、constraint、IKParam、支持多角形buffer、投影処理用helper関数を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | 投影IK、COM/ZMP統合、fallback、独自検証、allocation抑制、support hull処理を実装 |

### 追加・変更した主要class、function、state、parameter

#### class

- `WbmsPostureControl`
  - M1の入力保持・baseline取得に加え、M2で実現可能速度投影、投影結果検証、COM/ZMP統合を担当する。

#### function

- `WbmsPostureControl::init(const cnoid::BodyPtr&, const GaitParam&)`
  - 投影用robot、variables、constraints、priority vectors、IKParam、support hull buffersを初期化する。
- `WbmsPostureControl::addAncestorJointIds()`
  - 左右脚parent linkとCHEST linkからrootまでのancestor joint集合を作る。
- `WbmsPostureControl::syncProjectionRobot()`
  - 毎周期、現在の `genRobot` を投影用robotへ同期する。
- `WbmsPostureControl::updateSupportHull()`
  - 両足支持領域からfoot-mid基準の縮小支持多角形とworld基準の縮小支持多角形を作る。
- `WbmsPostureControl::calcProjectionTargets()`
  - 1周期先のCHEST姿勢目標とrobot COM目標を計算する。
- `WbmsPostureControl::solveProjection()`
  - 優先度付きIKを1回実行し、投影結果を `GaitParam` へ反映する。
- `WbmsPostureControl::validateProjection()`
  - solver後にfinite、limit、足誤差、異常変位を検証する。
- `WbmsPostureControl::setFallbackReference()`
  - invalid時に現在姿勢・現在CHEST/COMへfallbackし、realized velocityをゼロにする。
- `WbmsPostureControl::applyStaticComZmpIntegration()`
  - static WBMS操作中のみ投影COMを既存COM/ZMP生成系へblend反映する。
- `mathutil::calcConvexHull(vertices, convexHull, tmpVertices)`
  - 呼び出し側bufferを使う凸包生成。
- `mathutil::shrinkConvexHull2D()`
  - 支持凸包を内側へmargin縮小し、退化時は空またはfalseを返す。

#### state

M2で主に利用・更新される状態は以下。

- `wbmsPostureRobot_`
- `projectionJointIds_`
- `projectionVariables_`
- `projectionConstraints_`
- `projectionTasks_`
- `projectionIKParam_`
- `supportVerticesInFootMid_`
- `supportHullTmp_`
- `supportHullInFootMid_`
- `shrunkSupportHullInFootMid_`
- `shrunkSupportHullInWorld_`
- `shrinkShiftedPoints_`
- `shrinkShiftedDirs_`
- `GaitParam::wbmsPostureReferenceQ`
- `GaitParam::wbmsProjectedChestR`
- `GaitParam::wbmsProjectedRobotCom`
- `GaitParam::wbmsRealizedComVelocity`
- `GaitParam::wbmsRealizedTorsoAngularVelocity`
- `GaitParam::wbmsPostureReferenceValid`

#### parameter

M2で実際に使用開始した主なparameterは以下。

- `wbms_torso_angular_velocity_limit`
- `wbms_torso_angular_acceleration_limit`
- `wbms_torso_rpy_lower_limit`
- `wbms_torso_rpy_upper_limit`
- `wbms_torso_orientation_weight`
- `wbms_torso_orientation_max_error`
- `wbms_com_velocity_limit`
- `wbms_com_acceleration_limit`
- `wbms_com_offset_lower_limit`
- `wbms_com_offset_upper_limit`
- `wbms_com_position_weight`
- `wbms_com_xy_support_margin`

## M2の重要な実装判断とその理由

- 投影IK variableへ腕関節を含めない。
  - 正式仕様の「腕関節を投影IKのvariableへ含めない。ただし腕の現在姿勢と質量はCOM計算へ含める」を満たすため。
- COMは `genCog` ではなくrobot生COMとして計算する。
  - `sbpOffset` との混同を避け、仕様の `robot COM` 定義を守るため。
- 支持多角形が無効な場合は投影全体を失敗扱いにする。
  - COM XYを現在値に戻すだけだとZMP射影ができないまま `refZmpTraj` を上書きできるため、安全側fallbackを優先した。
- `IKParam` はメンバ `projectionIKParam_` として保持する。
  - 500 Hz経路で `dqWeight` vector代入によるheap allocationを避けるため。
- `MathUtil` には戻り値版とbuffer渡し版の両方を用意する。
  - 既存コード互換を保ちつつ、M2の500 Hz経路ではbuffer渡し版を使うため。
- COM拘束とCHEST姿勢拘束は同じ優先度3に置く。
  - 正式仕様書8.6と承認済み計画M2が「CHEST姿勢3D + COM位置3D」を同じ優先度の主操作タスクとして指定しているため。
- 最終IKにはM2で触れない。
  - M3の範囲を先行実装せず、後続で削除する暫定接続を作らないため。

## M2時点の仕様との差異

### M1で旧reference IKの一部削除を先行した

承認済み計画では旧reference IKの本格削除はM3に含まれるが、M1で旧 `refTorsoAnglVel` / `wbmsTorsoTargetRpy` を削除し、かつ新commandを旧root reference IKへ接続しない必要があった。このため、`FullbodyIKSolver::calcWbmsPostureReference()` と旧root-reference WBMS姿勢IK専用メンバをM1で削除した。

これは新しい制御機能の先行実装ではない。M2終了時点でも、新commandと投影結果は最終IKへはまだ接続されていない。

### 支持多角形退化時の扱い

正式仕様書8.5には「縮小後に有効な多角形を作れない場合、COM XY速度入力をその周期はゼロ扱い」とある。M2実装では、review指摘を受け、支持多角形が無効な場合は投影IK全体を失敗扱いにして現在姿勢へfallbackする。

理由は、縮小支持多角形が無い状態で処理を継続すると、ZMP XYを支持多角形内へ射影できないまま `refZmpTraj` を上書きできるためである。COM XYをゼロ扱いにする意図は安全側制限であり、ZMP射影不能時はfallbackの方が安全側である。

### COMとCHESTの優先度

COM拘束をCHEST姿勢拘束より高優先度へ分離するreview指摘があったが、正式仕様書8.6と承認済み計画M2は、COM位置3DとCHEST姿勢3Dを同じ優先度のソフトタスクとして解くことを明記している。このため、コードは `projectionConstraints_[3]` に両方を入れる現状を維持している。

## M2のビルド・review結果

### ビルドコマンドと結果

M2実装後およびreview対応後に以下を実行した。

```sh
catkin build auto_stabilizer --no-deps
```

結果は成功。`All 1 packages succeeded`、warningsなし。

### 静的確認コマンドと結果

以下を実行した。

```sh
rg -n "clone\\(|ancestor|wbmsPostureRobot|shrinkConvexHull2D|calcNearestPointOfHull" auto_stabilizer/rtc/AutoStabilizer
```

結果:

- `WbmsPostureControl::init()` の `genRobot->clone()` とancestor構築を確認。
- 周期内の `syncProjectionRobot()` ではcloneやancestor探索を行っていない。
- `MathUtil::shrinkConvexHull2D()` と `calcNearestPointOfHull()` の利用箇所を確認。
- `GaitParam` 内の既存cloneや `FootStepGenerator` の既存 `calcNearestPointOfHull()` 利用もヒットするが、M2投影周期内の新規cloneではない。

以下を実行した。

```sh
rg -n "WbmsPostureControl::proc|execStabilizer|calcCOMCoords" auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp
```

結果:

- 指定patternでは `calcCOMCoords` と `execStabilizer` を確認。
- 呼び出し実体は `wbmsPostureControl.proc(...)` であるため、追加で以下を実行した。

```sh
rg -n "wbmsPostureControl\\.proc|calcCOMCoords|execStabilizer" auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp
```

結果:

- `calcCOMCoords` が先、`wbmsPostureControl.proc(...)` が次、`execStabilizer` が後であることを確認。

review対応時に以下も実行した。

```sh
rg -n "IKParam param|param\\.dqWeight|projectionDqWeight|projectionIKParam|updateSupportHull\\(|shrunkSupportHullInWorld_|refZmpTraj\\.clear|make_shared|resize\\(|reserve\\(|clone\\(|addAncestorJointIds" auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h
```

結果:

- `IKParam` のローカル生成と `dqWeight` vector代入は残っていない。
- `reserve()`、constraint生成、ancestor探索は初期化時。
- self collision入力数が増えた場合のみ `resize()` / `make_shared()` が残る。これは正式仕様書12.2の許容範囲。

以下も確認した。

```sh
rg -n "projectionConstraints_\\[3\\]|chestConstraint_|comConstraint_|同じ優先度|優先度3" auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlCodexPlan.md
```

結果:

- 正式仕様書の「優先度3」「同じ優先度」と、実装の `projectionConstraints_[3]` へのCHEST/COM配置が一致していることを確認。

### `/review`で報告された重要な指摘と対応

#### 指摘1: 過大なmarginでは空の支持多角形を返す必要がある

- 分類: 修正対象。
- 問題: 縮小後半平面の共通部分が空でも、隣接直線交点を凸包化し直すと不正な小多角形を有効扱いできる。
- 修正:
  - `shrinkConvexHull2D()` で縮小後の各交点が全ての縮小後半平面を満たすか検証。
  - 過大margin、退化辺、非finite、面積ゼロ以下では空/falseを返す。
- 確認:
  - コード確認とビルド成功。

#### 指摘2: 500 Hzループ内のsupport hull用heap allocationを避ける必要がある

- 分類: 修正対象。
- 問題: `shiftedPoints` / `shiftedDirs` などの一時vectorを周期内で生成していた。
- 修正:
  - `calcConvexHull()` と `shrinkConvexHull2D()` にbuffer渡しoverloadを追加。
  - `WbmsPostureControl` にsupport hull用メンバbufferを追加し、`init()` でreserve。
  - `updateSupportHull()` でメンバbufferを再利用。
- 確認:
  - 静的検索とビルド成功。

#### 指摘3: 支持多角形が無効な場合は投影を失敗させる必要がある

- 分類: 修正対象。
- 問題: 縮小支持多角形が無い状態でCOM XYだけ現在値に戻して処理継続すると、ZMP射影なしで `refZmpTraj` を上書きできる。
- 修正:
  - `updateSupportHull()` 失敗時に `calcProjectionTargets()` がfalseを返すよう変更。
  - 呼び出し元で `setFallbackReference()` を実行し、COM/ZMP統合をスキップ。
- 確認:
  - 該当経路のコード確認とビルド成功。

#### 指摘4: 500 Hzループ内で `IKParam` のvectorコピーを避ける必要がある

- 分類: 修正対象。
- 問題: ローカル `IKParam` 生成と `param.dqWeight = projectionDqWeight_` が周期内allocation要因になり得る。
- 修正:
  - `IKParam` を `projectionIKParam_` としてメンバ化。
  - `dqWeight` の実体を `projectionIKParam_.dqWeight` に一本化し、周期内では値更新のみ行う。
- 確認:
  - `IKParam param`、`param.dqWeight`、`projectionDqWeight` が残っていないことを静的検索で確認。
  - ビルド成功。

#### 指摘5: COM拘束をCHEST姿勢より高優先度にする

- 分類: 対応不要。
- 判断理由:
  - 正式仕様書8.6と承認済み計画M2は、CHEST姿勢3DとCOM位置3Dを同じ優先度3のソフトタスクとして解くと明記している。
  - COMだけを上位へ分離すると正式仕様の優先度設計変更になる。
- 対応:
  - コード変更なし。
  - `projectionConstraints_[3]` にCHEST/COMを同居させる現状を維持。
- 確認:
  - 仕様・計画・コードの一致を静的検索で確認。

## M2でコードとビルドにより確認済みの事項

- 投影用robot cloneは `init()` 時のみ。
- ancestor探索は `init()` 時のみ。
- 投影IK constraint objectは `init()` 時に構築される。
- self collision入力数が増えた場合のみ、追加constraint object生成を許容する実装である。
- `WbmsPostureControl::proc()` は `calcCOMCoords` と `abcEETargetPose` 更新後、Stabilizer前に呼ばれる。
- CHEST目標はrootではなく `gaitParam.chestLinkName` のlink姿勢である。
- COM目標はrobot生COMをfoot-mid基準で扱う。
- COM/ZMP XYは縮小支持多角形へ射影される。
- 縮小支持多角形が無効な場合、投影はfallbackし、COM/ZMP統合は行われない。
- solverは1周期1回、`maxIteration=1`。
- solver戻り値だけでなく独自検証を行う。
- invalid時は現在姿勢へfallbackし、realized velocityはゼロ、valid flagはfalseになる。
- 投影成功時はreference q、projected CHEST/COM、realized velocity、valid flagが更新される。
- static WBMS操作中だけCOM/ZMP/refdz/omega/lをblend更新する。
- 歩行中および歩行開始遅延中は `isOperationAllowed()` によりCOM/ZMP/CHEST操作が無効になる。
- ビルドは成功している。

## M2終了時点の未解決事項

- M3の最終IK CHEST姿勢拘束は未実装だったが、M3で実装済み。
- M3のCOM weight blendは未実装だったが、M3で実装済み。
- M3のreference angle切替は未実装だったが、M3で実装済み。
- M3の旧reference IK最終整理は未実装だったが、M3で実装済み。
- M3のdebug OutPortまたはdebug logの最終仕様は未決定だったが、M3で新規 `wbmsDebugOut` OutPortを採用済み。
- M3の500 Hz実時間計測は未実施だったが、M3で計測値の公開は実装済み。実時間性能の合否判定はシミュレータ未確認。
- self collision入力数が周期中に増える場合は、正式仕様で許容されている範囲ではあるが、その周期に `resize()` / `make_shared()` が発生し得る。
- 支持多角形縮小が頻繁に退化する場合、パラメータ調整または追加の異常報告が必要。
- シミュレータ確認前のため、実応答、操作感、安定性、500 Hz実測は未確認。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md` は未追跡として存在する。既存ユーザー変更扱いで、不要に削除しないこと。

## M2終了時点のシミュレータ確認待ち項目

以下はコードとビルドだけではPASS扱いしない。

- 50 Hz入力を500 Hz制御で受けたとき、applied velocityが新着なし周期でゼロ落ちしないこと。
- 入力停止後、`wbms_velocity_command_timeout` まではholdし、その後加速度limitに従ってゼロへ戻ること。
- qRef停止がtimeoutより長く続いた後に復帰しても、古い速度commandが再開しないこと。
- WBMS停止fade-out中にteleopが継続publishしても、applied velocityが再上昇しないこと。
- WBMS停止後に再startしたとき、新しい入力受信まで古いcommandが再開しないこと。
- 歩行開始遅延へ入った後、古いcommandが歩行終了後に再開しないこと。
- CHEST roll/pitch/yawが概ね入力速度に応じて変化すること。
- COM前後左右/上下が概ね入力速度に応じて変化すること。
- COM/ZMPが支持多角形margin外へ出ないこと。
- pitch前傾とCOM下降の同時入力で足踏み、yaw振動、急激な下半身振動が発生しないこと。
- 腕同時操縦で手先がworld固定的に体幹運動を阻害しないこと。
- 腕のCHEST相対拘束が従来どおり維持されること。
- 歩行中はCOM/CHEST操縦が無効化され、通常安定化へ戻ること。
- 既存歩行APIと歩行開始遅延が壊れていないこと。
- 500 Hz周期が継続すること。

## M2終了時点のM3引き継ぎ

### 次のマイルストーンが使用するinterface、state、前提条件

M3は以下を前提に開始する。

- `WbmsPostureControl` は `AutoStabilizer` にメンバとして保持されている。
- `WbmsPostureControl::init()` で投影用robot、variables、constraints、IKParam、support hull bufferが構築済み。
- `WbmsPostureControl::proc()` はStabilizer前に呼ばれている。
- `GaitParam` にraw/applied command、age/valid、baseline、投影結果、mode値が存在する。
- 投影成功時、`GaitParam::wbmsPostureReferenceQ` が全関節分更新される。
- 投影成功時、`GaitParam::wbmsProjectedChestR`、`GaitParam::wbmsProjectedRobotCom`、`GaitParam::wbmsRealizedComVelocity`、`GaitParam::wbmsRealizedTorsoAngularVelocity`、`GaitParam::wbmsPostureReferenceValid` が更新される。
- invalid時、`wbmsPostureReferenceValid=false`、realized velocityゼロ、現在姿勢fallbackになる。
- static WBMS操作中だけ `genCog`、`genCogVel`、`genCogAcc`、`refdz`、`omega`、`l`、`refZmpTraj` が投影結果でblend更新される。
- `wbmsOperationModeValue` は `wbmsMode.value() * (1 - wbmsWalkingStabilityModeValue)` として更新されている。
- `WbmsPostureControl::isOperationAllowed()` は、AB running、WBMS有効目標、static、非歩行開始遅延、両足support、脚manual control無効を判定する。
- 新commandと投影結果は、M2終了時点ではまだ最終IKへ接続されていない。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
  - `init()`
  - `calcProjectionTargets()`
  - `solveProjection()`
  - `validateProjection()`
  - `setFallbackReference()`
  - `applyStaticComZmpIntegration()`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
  - `wbmsPostureReferenceQ`
  - `wbmsProjectedChestR`
  - `wbmsProjectedRobotCom`
  - `wbmsPostureReferenceValid`
  - clear/reset関数。
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
  - 最終IKのCOM target、COM weight、reference angle処理。
  - M3で投影CHEST拘束を追加する位置。
- `auto_stabilizer/rtc/AutoStabilizer/MathUtil.h/.cpp`
  - `calcConvexHull()` buffer渡し版。
  - `shrinkConvexHull2D()`。
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlCodexPlan.md`

### 次に実行すべきビルド・確認コマンド

M3作業開始前に以下を実行し、M2完了状態から崩れていないことを確認する。

```sh
git status --short
catkin build auto_stabilizer --no-deps
rg -n "clone\\(|ancestor|wbmsPostureRobot|shrinkConvexHull2D|calcNearestPointOfHull" auto_stabilizer/rtc/AutoStabilizer
rg -n "wbmsPostureControl\\.proc|calcCOMCoords|execStabilizer" auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp
rg -n "IKParam param|param\\.dqWeight|projectionDqWeight" auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h
```

M3実装後は承認済み計画に従い、最終IKのCHEST拘束、COM weight blend、reference angle切替、腕CHEST相対拘束維持、旧reference IK整理、debug/計測を確認する。

## 今回のマイルストーン完了記録（M3）

### マイルストーン番号と名称

マイルストーン3: 最終IK統合・旧方式削除・debug/計測。

### 完了状態

コード、API、構造面の実装は完了。`catkin build auto_stabilizer --no-deps` に成功している。M3実装後の `/review` 指摘は修正済みであり、最終 `/review` では差分上の破壊的問題は報告されていない。

シミュレータでは `wbmsDebugOut` のログ取得まで実施済み。ただし、体幹角速度指令を送っても前傾しない問題を確認しており、M3完了後の未解決事項として扱う。ログ上はraw/applied pitch角速度指令は入っている一方、projector valid flagが全周期0、realized torso angular velocityが全周期0であった。

### 実装した範囲

- `FullbodyIKSolver` の最終IKへ、投影済み `wbmsProjectedChestR` をtargetとするCHEST姿勢 `PositionConstraint` を追加した。
- CHEST姿勢拘束は位置weightをゼロ、姿勢weightを `wbmsTorsoOrientationWeight * wbmsOperationModeValue` とした。
- CHEST姿勢拘束の角度 `maxError` は `wbmsTorsoOrientationMaxError * dt` とした。
- COM拘束targetは従来どおり `gaitParam.genCog + gaitParam.sbpOffset` とし、weightだけを通常値 `[10, 10, wbmsStabilityMode]` と `wbmsComPositionWeight` の間で `wbmsOperationModeValue` によりblendした。
- reference angleは、投影validかつ投影IK variableに含まれる関節だけ `wbmsPostureReferenceQ` と従来 `refRobot` を `wbmsOperationModeValue` でblendするようにした。
- 投影IK variable外の腕などの関節は従来 `refRobot` referenceを使う。
- 投影invalid時は `wbmsPostureReferenceJointMask` を全falseにし、最終IKのreference angleは従来referenceへfallbackする。
- 投影invalid時は最終IKのCHEST姿勢拘束も追加しない。歩行開始遅延や投影失敗時に現在CHEST姿勢保持がroot姿勢復帰・通常COM安定化と競合しないようにするため。
- 上半身EEのWBMS中CHEST相対拘束は、従来どおり `B_link = torsoGenLink`、`B_localpos = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i]` のまま維持した。
- 既存 `cpViewerLog` は固定indexを変更せず、新規 `wbmsDebugOut` `TimedDoubleSeq` OutPortを追加した。
- `WbmsPostureControl::proc()`、`FullbodyIKSolver::solveFullbodyIK()`、`AutoStabilizer::onExecute()` の計算時間を `std::chrono::steady_clock` で計測し、毎周期標準出力へは出さず `wbmsDebugOut` で確認できるようにした。
- `GaitParam::wbmsPostureReferenceJointMask` を追加し、投影IK variableに含まれる関節だけ最終IKのreference angle blend対象にした。
- `GaitParam::DebugData` に `wbmsProjectorTime`、`wbmsFinalIKTime`、`onExecuteTime` を追加した。
- `FullbodyIKSolver::solveFullbodyIK()` は計測値を書き込むため `const GaitParam&` ではなく `GaitParam&` を受け取るようにした。

### 意図的に未実装とした後続範囲

- simulator上のゲイン・limit調整。
- logger/viewer側の `wbmsDebugOut` 表示設定。
- projector invalid理由の詳細debug公開。
- `wbmsDebugOut` の統計集計。現状は周期ごとの値のみを出力する。
- 歩行中にstatic WBMSと同じCOM速度操作を有効化する機能。正式仕様どおり今回範囲外。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp` | 最終IKのCHEST姿勢拘束、COM weight mode blend、reference angle mode blend、final IK時間計測を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h` | CHEST姿勢拘束メンバを追加し、`solveFullbodyIK()` の引数を `GaitParam&` へ変更 |
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | `wbmsPostureReferenceJointMask` とdebug計測値を追加し、clear処理へ反映 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | 投影成功時のjoint mask設定、fallback時のmask clear、projector時間計測を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | `wbmsDebugOut` OutPort登録、debug値の詰め替え、`onExecute()` 全体時間計測を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h` | `wbmsDebugOut` 用 `TimedDoubleSeq` とOutPortを追加 |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | M3実装結果、review対応、シミュレータで確認した未解決事項、次セッションへの引き継ぎを記録 |

### 追加・変更した主要class、function、state、parameter

#### class / function

- `FullbodyIKSolver::solveFullbodyIK(double dt, GaitParam& gaitParam, cnoid::BodyPtr& genRobot)`
  - 最終IKの主拘束へ投影済みCHEST姿勢拘束を追加する。
  - COM weightを通常値とWBMS用値の間でmode blendする。
  - reference angleを投影joint maskに従ってmode blendする。
  - final IK計算時間を `gaitParam.debugData.wbmsFinalIKTime` に保存する。
- `WbmsPostureControl::solveProjection()`
  - 投影成功時に `wbmsPostureReferenceJointMask` を設定する。
- `WbmsPostureControl::setFallbackReference()`
  - fallback時に `wbmsPostureReferenceJointMask` を全falseにする。
- `WbmsPostureControl::proc()`
  - projector計算時間を `gaitParam.debugData.wbmsProjectorTime` に保存する。
- `AutoStabilizer::writeOutPortData()`
  - `wbmsDebugOut` の30要素を出力する。
- `AutoStabilizer::onExecute()`
  - OutPort書き込み後に `onExecute()` 全体時間を確定し、次回出力周期の `wbmsDebugOut[29]` で公開する。

#### state

- `FullbodyIKSolver::chestPositionConstraint`
- `GaitParam::wbmsPostureReferenceJointMask`
- `GaitParam::DebugData::wbmsProjectorTime`
- `GaitParam::DebugData::wbmsFinalIKTime`
- `GaitParam::DebugData::onExecuteTime`
- `AutoStabilizer::Ports::m_wbmsDebug_`
- `AutoStabilizer::Ports::m_wbmsDebugOut_`

#### parameter

M3で新しい設定parameterは追加していない。M1/M2で追加済みの以下を最終IKでも使用する。

- `wbms_torso_orientation_weight`
- `wbms_torso_orientation_max_error`
- `wbms_com_position_weight`

### 重要な実装判断とその理由

- 最終IKのCHEST拘束は投影valid時だけ追加する。
  - 投影invalid時や歩行開始遅延中に現在CHEST姿勢保持が残ると、root姿勢復帰・通常COM安定化と競合するため。
- CHEST拘束の位置weightはゼロにする。
  - 体幹位置ではなくCHEST世界姿勢だけを操作対象にする正式仕様に従うため。
- CHEST拘束の姿勢weightは `wbmsTorsoOrientationWeight * wbmsOperationModeValue` とする。
  - static WBMS操作と歩行安定化切替の共通係数を `wbmsOperationModeValue` に一本化するため。
- COM targetは `genCog + sbpOffset` のままとし、targetではなくweightをmode blendする。
  - M2でstatic WBMS中の `genCog` は投影済みCOMと整合済みであり、最終IK側で別targetを作るとCOM/ZMP/omega/lとの整合を崩すため。
- reference angleは投影validかつ投影IK variableの関節だけ投影Qを使う。
  - 腕など投影variable外の関節commandを上書きしないため。
- debugは既存 `cpViewerLog` へ追加せず、新規OutPortにした。
  - `cpViewerLog` の固定indexを壊すと既存viewer互換へ影響するため。
- `onExecute()` 時間はOutPort書き込み後に確定し、次回出力周期で公開する。
  - `wbmsDebugOut` 自身を含むOutPort書き込み時間を計測値へ含めるため。

### `wbmsDebugOut` の要素順

`wbmsDebugOut` は30要素の `TimedDoubleSeq` とする。

| index | 内容 |
|---|---|
| 0-2 | raw COM velocity |
| 3-5 | applied COM velocity |
| 6-8 | realized COM velocity |
| 9-11 | raw torso angular velocity |
| 12-14 | applied torso angular velocity |
| 15-17 | realized torso angular velocity |
| 18-20 | WBMS開始時からのCOM offset |
| 21-23 | WBMS開始時からのCHEST RPY offset |
| 24 | `wbmsOperationModeValue` |
| 25 | `wbmsWalkingStabilityModeValue` |
| 26 | projector valid flag |
| 27 | projector計算時間[s] |
| 28 | final IK計算時間[s] |
| 29 | 前回出力更新周期の `onExecute()` 全体計算時間[s]。OutPort書き込み後に確定した値 |

### 採用初期値

M3で新しい調整パラメータは追加していない。M1/M2で追加済みの正式仕様書6.3の推奨初期値を継続して使う。

- `wbms_torso_orientation_weight = [0.3, 0.3, 0.3]`
- `wbms_torso_orientation_max_error = [0.15, 0.15, 0.30]`
- `wbms_com_position_weight = [3.0, 3.0, 1.0]`
- その他の速度limit、加速度limit、offset limit、support marginも正式仕様書6.3どおり。

## 仕様との差異

### `wbmsDebugOut[29]` は前回出力更新周期の値

`onExecute()` 全体時間にOutPort書き込み時間を含めるため、`writeOutPortData()` の後で `debugData.onExecuteTime` を更新している。このため、`wbmsDebugOut[29]` に出る値は同一周期で確定した値ではなく、前回出力更新周期の値である。

### 支持多角形退化時は投影全体をfallback

M2 review対応で記録済みの差異をM3でも継続している。正式仕様書は「COM XY速度入力をその周期はゼロ扱い」としているが、縮小支持多角形が無い状態ではZMP XY射影もできないため、現在実装は投影全体を失敗扱いにして現在姿勢へfallbackする。

### projector valid判定が保守的すぎる可能性

現在コードは `solveIKLoop()` の戻り値と独自検証の両方がtrueの場合だけ投影validにしている。承認済み計画では `maxIteration=1` のため戻り値だけで成功/失敗を判断すると過剰失敗の可能性があり、独自検証を最終判定に使う方針が記載されている。シミュレータログではvalidが全周期0であり、この点が未解決リスクとして残っている。

## ビルド・review結果

### ビルドコマンドと結果

M3実装後およびreview対応後に以下を実行した。

```sh
catkin build auto_stabilizer --no-deps
```

結果は成功。`All 1 packages succeeded`、warningsなし。

### 静的確認コマンドと結果

以下を実行した。

```sh
rg -n "wbmsTorsoTargetRpy|refTorsoAnglVel|calcWbmsPostureReference|wbmsPostureRootConstraint|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
```

結果は該当なし。旧root姿勢積分方式と旧 `WbmsTorsoControl` 識別子は制御コード配下に残っていない。

以下を実行した。

```sh
rg -n "B_link\\(\\) = torsoGenLink|wbmsProjectedChestR|wbmsComPositionWeight|wbmsOperationModeValue" auto_stabilizer/rtc/AutoStabilizer
```

結果:

- 腕CHEST相対拘束の `B_link() = torsoGenLink` を確認。
- 最終IKの投影済みCHEST姿勢target、COM weight blend、operation mode参照を確認。
- `WbmsPostureControl` 側のoperation mode計算とCOM/ZMP統合処理を確認。

以下を実行した。

```sh
git diff --stat
```

## M4.2.2 Work Package B 歩行準備遷移本体 実装記録

### 実装範囲

M4.2.2 Work Package Bとして、固定タイマだけでpending walking commandをreleaseしていた `WbmsWalkingCommandDelay` を、明示的な歩行準備phase state machineへ拡張した。

phaseは次を使用する。

| phase | 意味 |
|---:|---|
| 0 | INACTIVE |
| 1 | REQUESTED |
| 2 | DECELERATING |
| 3 | RETURNING |
| 4 | HANDOFF |
| 5 | READY |
| 6 | WALKING_HOLD |
| 7 | FAILED |

`goPos()`、`goVelocity()`、`setFootSteps()` 受付時はpending commandを `WbmsWalkingCommandDelay` 内へ保存し、体幹/COM raw commandだけをclearする。applied commandは即時ゼロにせず、既存の加速度limitでDECELERATING中にゼロへ戻す。腕EE commandと `wbmsMode` はclearしない。

REQUESTEDの次制御周期で、`footMidCoords` 基準の現在CHEST姿勢、現在robot COM、保持COM高さ、WBMS統合前nominal COM X/Y、root姿勢をsnapshotする。保持COM高さは、validな `wbmsProjectedRobotCom` を優先し、使えない場合は `genRobot->centerOfMass()` を使う。どちらもfiniteでなければFAILEDに遷移し、pending commandは破棄する。

### parameter意味

M4.2.2では外部調整が必要な準備遷移parameterだけIDLへ追加した。

| parameter | 意味 |
|---|---|
| `wbms_walking_preparation_timeout` | REQUESTEDからREADY/releaseまでの上限時間。超過時はFAILED |
| `wbms_walking_preparation_return_time` | RETURNINGでCHESTとCOM X/Yを歩行可能基準へ戻す時間 |
| `wbms_walking_preparation_handoff_time` | HANDOFFでWBMS姿勢拘束を通常歩行側へ渡す時間 |
| `wbms_walking_preparation_settle_time` | READY条件の連続成立時間 |
| `wbms_walking_preparation_velocity_eps` | applied torso/COM command収束判定 |
| `wbms_walking_preparation_chest_error_eps` | CHEST基準姿勢誤差のREADY閾値 |
| `wbms_walking_preparation_com_xy_error_eps` | COM X/Y nominal誤差のREADY閾値 |
| `wbms_walking_preparation_com_z_error_eps` | COM Z保持誤差のREADY閾値 |
| `wbms_walking_preparation_root_error_eps` | root姿勢誤差のREADY閾値 |
| `wbms_walking_preparation_max_joint_delta_eps` | 一周期最大関節変化のREADY閾値 |

`wbms_walking_stability_start_time` と `wbms_walking_stability_stop_time` は、従来どおり歩行安定化weightの補間時間として維持した。5秒へ延長するだけのrejected trialは採用していない。

### COM X/YとZの分離

RETURNINGでは、CHESTはsnapshot姿勢から `wbmsStartChestRInFootMid` へ補間し、COM X/Yはsnapshot robot COMからWBMS統合前nominal static walking COMへ補間する。COM Zは `heldRobotComHeightInFootMid` で固定し、`wbmsStartComInFootMid.z` へ戻さない。

COM高さ保持はprojector validや `wbmsOperationModeValue` へ依存させず、独立経路で扱う。`RefToGenFrameConverter::convertFrame()` 後に `refdz`、`l.z`、`omega` を保持高さへ再整合し、`LegCoordsGenerator::calcCOMCoords()` 後に `genCog.z`、`genCogVel.z`、`genCogAcc.z` だけを速度/加速度limit付きで補正する。X/Yは通常歩行バランス系へ任せる。

### READYとrelease

READY条件は、applied command速度、CHEST error、COM XY error、COM Z hold error、root error、candidate safe、max joint delta、handoff完了、settle timeを確認する。READYになった周期ではpending commandを投入せず、`wbmsWalkingPreparationReleaseRequested` だけを立てる。次の `onExecute()` 冒頭の `WbmsWalkingCommandDelay::proc()` でpending commandをreleaseし、WALKING_HOLDへ遷移する。

### clear/cancel

`goStop()`、`stopWholeBodyMasterSlave()`、`stopAutoBalancer()`、`MODE_SYNC_TO_ABC` 初期化、`onActivated()`、`onDeactivated()` でphase、pending、height hold、stale commandをclearする。timeout/FAILEDでは歩行を強行せず、pending commandを破棄し、failure codeをdebugへ残す。

### 腕操作維持

歩行準備・歩行中に `wbmsMode` は停止しない。`RefToGenFrameConverter` の上半身EE差分変換と、`FullbodyIKSolver` の `B_link() = torsoGenLink`、`B_localpos() = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i]` によるCHEST相対拘束を維持する。

### build・静的確認

IDLを変更したため、初回buildは `catkin build auto_stabilizer --no-deps --force-cmake` を使用した。

| 確認 | 結果 | 備考 |
|---|---|---|
| `catkin build auto_stabilizer --no-deps --force-cmake` | PASS | OpenRTM helper由来のYAML warningのみ |
| `git diff --check` | PASS | whitespace指摘なし |
| rejected trialの実コード混入なし | PASS | `isProjectionReferenceAllowed` と5秒既定値変更は実コード/IDLに存在しない |
| debug lengthと代入数 | PASS | `wbmsDebugOut` length 60、代入数60 |
| index 0-45不変 | PASS | 既存順序は維持し、46-59をM4.2.2値へ接続 |
| READYとreleaseが別周期 | PASS | READYは `WbmsPostureControl::proc()`、releaseは次周期冒頭の `WbmsWalkingCommandDelay::proc()` |
| `wbmsStartComInFootMid.z`へ戻さない | PASS | RETURNING/height holdは `heldRobotComHeightInFootMid` を使用 |
| `refZmpTraj`時間構造維持 | PASS | 既存segment時間を維持し、空/時間和0だけ正時間fallback |
| state clear経路 | PASS | goStop、WBMS停止、ABC停止、sync init、activate/deactivateでclear |
| optional M4.2.3未実装 | PASS | 歩行中COM Z速度操作は追加していない |

シミュレータ確認は未実施である。

### review指摘と処置

| 順序 | 指摘要約 | 分類 | 処置 |
|---|---|---|---|
| 1 | 準備遷移開始条件が旧 `wbmsWalkingStabilityStartTime` に依存し、0秒設定でM4.2.2安全ゲートを迂回する | 修正対象 | `WbmsWalkingCommandDelay::shouldDelay()` から旧timer条件を削除し、static WBMS activeなら準備遷移へ入るよう修正 |
| 2 | READY判定が最終IK前に実行され、現在周期のfinal IK後joint delta/root/CHEST/COMを確認せずrelease予約する | 修正対象 | READY評価を `WbmsPostureControl::updateWalkingPreparationReadiness()` へ移し、`solveFullbodyIK()` と `updateWbmsFinalIKDiagnostics()` の後に呼ぶよう修正 |

上記修正により、READY成立は現在周期のfinal IK後diagnosticsを確認してから行い、pending command releaseは従来どおり次周期冒頭に分離される。

### Work Package B 最終整理

#### 1. 実装範囲

Work Package Bでは、WBMS静止操作中の歩行開始を固定timer releaseから明示的な歩行準備phase state machineへ置き換えた。

- `WbmsWalkingCommandDelay` がpending `goPos` / `goVelocity` / `setFootSteps` を保持し、READY後の次周期冒頭でreleaseする。
- 歩行準備開始時にraw torso/COM commandをclearし、applied commandは既存加速度limitでゼロへ減速する。
- REQUESTEDでCHEST姿勢、robot COM、保持COM高さ、WBMS統合前nominal COM、root姿勢をsnapshotする。
- DECELERATING、RETURNING、HANDOFF、READY、WALKING_HOLD、FAILEDを明示的に扱う。
- COM高さ保持を `RefToGenFrameConverter::convertFrame()` 後と `LegCoordsGenerator::calcCOMCoords()` 後の独立経路へ接続した。
- READY判定はfinal IKとfinal IK diagnostics更新後に行い、releaseは次周期の `WbmsWalkingCommandDelay::proc()` で行う。
- timeout、FAILED、goStop、WBMS停止、ABC停止、sync初期化、activate/deactivateでpending command、phase、height hold、stale commandを安全側にclearする。
- 歩行準備中・歩行中も `wbmsMode` と腕EE commandを維持し、歩行中はtorso角速度 commandとCOM X/Y commandを無効化する。

#### 2. 対象外

- optional M4.2.3。
- 歩行中COM Z速度操作。
- 歩行中体幹角度操作。
- task scaling。
- dependency solver変更。
- 全関節のWBMS開始時姿勢への復帰。
- validation閾値緩和。
- clang-format。
- 新規テストコード。
- simulator上の安定性、操作感、歩行開始時の実挙動合否判定。

#### 3. 変更ファイル

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | 歩行準備phase enum、failure code、parameter、snapshot/readiness/height hold/debug state、clear helperを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.h/.cpp` | pending command保持、phase開始、snapshot、timeout、READY次周期release、pending clearを実装 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h/.cpp` | 許可条件分離、DECELERATING/RETURNING/HANDOFF処理、COM高さ保持、READY判定、FAILED処理、RETURNING開始点ラッチを実装 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | COM高さ保持の呼び順、pre-WBMS nominal COM保存、READY判定呼び出し、FAILED時pending clear、debug index接続、IDL parameter set/getを追加 |
| `auto_stabilizer/idl/AutoStabilizerService.idl` | 歩行準備timeout、return/handoff/settle時間、READY閾値parameterを追加 |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | Work Package Bの実装、review対応、acceptance、引き継ぎを記録 |

#### 4. phase/state/debug index

phase番号は `GaitParam::WbmsWalkingPreparationPhase` の値をそのまま `wbmsDebugOut[46]` へ出す。

| phase | 意味 |
|---:|---|
| 0 | INACTIVE |
| 1 | REQUESTED |
| 2 | DECELERATING |
| 3 | RETURNING |
| 4 | HANDOFF |
| 5 | READY |
| 6 | WALKING_HOLD |
| 7 | FAILED |

`wbmsDebugOut[0-45]` は既存定義を維持する。Work Package Bで接続したindexは次の通り。

| index | 意味 |
|---:|---|
| 46 | walking preparation phase |
| 47 | preparation elapsed time |
| 48 | return alpha |
| 49 | handoff alpha |
| 50 | held robot COM height in footMid |
| 51 | current robot COM height in footMid |
| 52 | CHEST error |
| 53 | COM XY error |
| 54 | COM Z hold error |
| 55 | root error |
| 56 | max joint delta |
| 57 | pending command release event |
| 58 | preparation failure code |
| 59 | runtime `wbmsWalkingStabilityStartTime` |

M4.2.2の初回シミュレータログ解析後、原因切り分けに不足していた値を追加するため、既存index 0-59を維持したまま `wbmsDebugOut` を84要素へ拡張した。追加indexは次の通り。

| index | 意味 | 単位・値 |
|---:|---|---|
| 60 | final IK後root roll | generate frame、rad |
| 61 | final IK後root pitch | generate frame、rad |
| 62 | final IK後root yaw | generate frame、rad |
| 63 | `stTargetRootPose` roll | generate frame、rad |
| 64 | `stTargetRootPose` pitch | generate frame、rad |
| 65 | `stTargetRootPose` yaw | generate frame、rad |
| 66 | `refdz` | m |
| 67 | `l.z` | m |
| 68 | `omega` | 1/s |
| 69 | `refZmpTraj[0]` start X | generate frame、m |
| 70 | `refZmpTraj[0]` start Y | generate frame、m |
| 71 | `refZmpTraj[0]` start Z | generate frame、m |
| 72 | `refZmpTraj[0]` goal X | generate frame、m |
| 73 | `refZmpTraj[0]` goal Y | generate frame、m |
| 74 | `refZmpTraj[0]` goal Z | generate frame、m |
| 75 | `refZmpTraj[0]` time | s |
| 76 | `refZmpTraj` total time | s |
| 77 | `footstepNodesList.size()` | 個数 |
| 78 | `footstepNodesList[0].remainTime` | s |
| 79 | 現在footstepの `elapsedTime` | s |
| 80 | 右脚support flag | supportなら1 |
| 81 | 左脚support flag | supportなら1 |
| 82 | 右脚 `swingState` | `LIFT=0, SWING=1, DOWN=2` |
| 83 | 左脚 `swingState` | `LIFT=0, SWING=1, DOWN=2` |

failure codeは `NONE=0`、`SNAPSHOT=1`、`TIMEOUT=2`、`NONFINITE=3`、`UNSAFE=4`、`CANCELLED=5` である。

#### 5. held COM heightの定義

`heldRobotComHeightInFootMid` は、歩行指令受付後、REQUESTEDを処理する最初の制御周期でsnapshotしたrobot COMの `footMidCoords` 基準Zである。

取得元の優先順位は次の通り。

1. `wbmsPostureReferenceValid` かつfiniteな `wbmsProjectedRobotCom`。
2. finiteな `genRobot->centerOfMass()`。
3. どちらも使えない場合はsnapshot失敗としてFAILED。

この値は `wbmsStartComInFootMid.z` とは別物であり、WBMS開始時高さへ戻さない。寿命は歩行準備開始からWALKING_HOLD後のstatic復帰、cancel、FAILED、各clear経路までである。

#### 6. COM X/YとZの扱い

- COM X/Yは通常歩行側のnominal targetへ戻す。nominalは `LegCoordsGenerator::calcCOMCoords()` 直後、WBMS統合前の `genCog + sbpOffset` を保存して使う。
- COM Zは `heldRobotComHeightInFootMid` で保持する。RETURNING targetでも、DECELERATING中の投影targetでも、`wbmsStartComInFootMid.z` へ戻さない。
- `refdz`、`l.z`、`omega` は `RefToGenFrameConverter::convertFrame()` 直後に保持高さから再整合する。
- `genCog.z`、`genCogVel.z`、`genCogAcc.z` は `LegCoordsGenerator::calcCOMCoords()` 後に速度/加速度limit付きで補正する。
- static WBMSおよび歩行準備中のprojector/static COM-ZMP統合は維持するが、歩行中はstatic torso/COM XY projectorを無効化し、COM Z保持だけを独立して継続する。
- `refZmpTraj` は既存segmentの時間構造を変えず、必要な場合だけstart/goalを平行移動する。空または総時間0の異常時だけ正時間fallbackを使う。

更新順は `convertFrame()`、COM高さから `refdz/l.z/omega` 再整合、`calcCOMCoords()`、pre-WBMS nominal保存、`genCog.z/genCogVel.z/genCogAcc.z` 補正、WBMS projection/static統合、Stabilizer、final IK、final diagnostics、READY判定である。

#### 7. 腕操作維持

歩行準備中・歩行中も腕EE commandと `wbmsMode` はclearしない。

腕操作維持のコード根拠は、上半身EEのtarget生成を継続する `refEEPose` / `abcEETargetPose` 経路と、`FullbodyIKSolver` の上半身EE拘束で `B_link() = torsoGenLink`、`B_localpos() = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i]` を使うCHEST相対拘束である。足、安全、バランスを優先し、腕は投影variable外では従来referenceを使う。

#### 8. 重要判断

- 準備遷移開始条件は旧 `wbmsWalkingStabilityStartTime` ではなく、static WBMS activeに基づける。
- pending commandは `WbmsWalkingCommandDelay` 内に保持し、GaitParamには共有phase、readiness、snapshot、debugに必要な状態だけを置く。
- READY成立周期とpending release周期を分離する。
- READY判定はfinal IK後diagnosticsで行う。
- timeoutはreleaseより前に判定し、timeout超過後はreleaseしない。
- 歩行準備中の再コマンドはpending commandだけを更新し、snapshot、held height、phase、elapsed timeを初期化しない。
- HANDOFF alpha更新後に `wbmsOperationModeValue` を計算し、その周期のfinal IKへ実際に渡るWBMS重みとREADY判定を一致させる。
- DECELERATING完了時に現在の投影CHEST姿勢/robot COMをRETURNING開始点へラッチし、受付時snapshotへtargetが巻き戻らないようにする。
- 姿勢制御側FAILEDでも、同じ `onExecute()` 内でdelay側pending commandだけを破棄し、FAILED診断statusは残す。
- 500 Hz経路に追加IK solve、clone/new、毎周期ancestor searchを追加しない。

#### 9. 計画との差異

- `wbms_walking_preparation_timeout` と `wbms_walking_preparation_settle_time` だけでなく、return/handoff時間とREADY閾値もIDLへ追加した。理由は調整単位をruntime parameterとして露出した方が、シミュレータ確認時に閾値緩和なしで挙動確認しやすいためである。
- `WbmsWalkingCommandDelay::proc()` は `execAutoStabilizer()` より前に呼ばれるため、READY判定は `WbmsPostureControl` 側へ分け、releaseだけを次周期のdelay側で処理する構成にした。
- FAILED時のpending破棄は、GaitParamのFAILED診断statusを残すため `clear()` ではなく `clearPendingCommand()` を追加して行う。
- review対応により、DECELERATINGからRETURNINGへの復帰開始点は歩行指令受付時snapshotではなくDECELERATING完了時の現在投影状態へ更新する。保持COM高さそのものは変更しない。

#### 10. build・静的確認

| 確認 | 結果 | 備考 |
|---|---|---|
| `catkin build auto_stabilizer --no-deps --force-cmake` | PASS | IDL変更あり。OpenRTM helper由来のYAML warningのみ |
| `git diff --check` | PASS | whitespace指摘なし |
| rejected trial識別子残存確認 | PASS | `isProjectionReferenceAllowed` は残していない |
| 5秒既定値変更残存確認 | PASS | rejected trial由来の既定値変更は残していない。既存WBMS mode遷移の5秒指定は別物 |
| index 0-45不変 | PASS | 既存debug順序を維持 |
| debug lengthと代入数 | PASS | `wbmsDebugOut` length 60、代入数60 |
| READY/release周期分離 | PASS | READYはfinal IK後、releaseは次周期冒頭 |
| state clear経路 | PASS | goStop、WBMS停止、ABC停止、sync init、activate/deactivate、FAILEDで処理 |
| COM Zが `wbmsStartComInFootMid.z` へ戻らない | PASS | `heldRobotComHeightInFootMid` を使用 |
| `refZmpTraj`総時間 | PASS | 既存時間構造を維持し、異常時のみ正時間fallback |
| 腕CHEST相対拘束 | PASS | `B_link() = torsoGenLink` とCHEST相対 `B_localpos()` を確認 |

#### 11. review指摘と処置

| 順序 | 指摘要約 | 分類 | 処置 |
|---|---|---|---|
| 1 | 旧timer 0秒設定で準備遷移を迂回できる | 修正対象 | `shouldDelay()` をstatic WBMS active基準へ変更 |
| 2 | READY判定がfinal IK前の古いdiagnosticsを見る | 修正対象 | READY評価をfinal IK後へ移動 |
| 3 | READY後、timeout超過周期でもpending releaseできる | 修正対象 | release前にtimeoutを判定 |
| 4 | nominal COM XY snapshotがWBMS統合済み `genCog` を使う | 修正対象 | `calcCOMCoords()` 直後のpre-WBMS nominalを保存して使用 |
| 5 | 準備中に残留COM Z速度が投影targetへ混ざる | 修正対象 | 準備中の投影target Z速度を0にし、held heightへ固定 |
| 6 | 準備中の再コマンドでsnapshot/timeout/phaseを初期化する | 修正対象 | pendingだけ更新し、phase/snapshot/elapsedを維持 |
| 7 | handoff alpha更新前の重みでfinal IKし、更新後alphaでREADY判定できる | 修正対象 | alpha更新後に `wbmsOperationModeValue` を計算 |
| 8 | DECELERATING後にRETURNING targetが受付時snapshotへ巻き戻る | 修正対象 | DECELERATING完了時の現在投影状態をRETURNING開始点へラッチ |
| 9 | 姿勢制御側FAILEDでdelay側pendingが残る | 修正対象 | FAILED検出時に `clearPendingCommand()` を呼びhidden goalを破棄 |
| 10 | 重点項目で追加修正必須の不具合なし | 対応不要 | 追加コード変更なし |

#### 12. acceptanceごとの結果

| acceptance | 結果 | 備考 |
|---|---|---|
| phase state machineが明示的 | PASS | 8 phaseをGaitParamに定義 |
| pending commandをdelay内に保持 | PASS | GaitParamへpending内容を置かない |
| 受付時にraw torso/COMをclearしappliedをlimit減速 | PASS | 腕EE commandと `wbmsMode` は維持 |
| snapshot項目を取得 | PASS | CHEST、robot COM、held height、nominal COM、rootを取得 |
| held COM height優先順位 | PASS | projected COM、genRobot COM、失敗時FAILED |
| COM ZをWBMS開始時高さへ戻さない | PASS | held heightを使用 |
| DECELERATINGでprojector/static統合維持 | PASS | walking stability modeはRETURNING以降 |
| RETURNINGでCHEST/COM XYを滑らかに戻す | PASS | simulatorでの滑らかさは未確認 |
| HANDOFFでtarget保持とweight消去を分離 | PASS | alpha更新後weightを使用 |
| COM高さ保持経路 | PASS | `refdz/l.z/omega/genCog.z` を接続 |
| READY条件 | PASS | final IK後diagnosticsで判定 |
| READY周期とrelease周期分離 | PASS | 次周期冒頭release |
| WALKING_HOLDでstatic torso/COM XY projector無効 | PASS | COM Z保持は継続 |
| 歩行中torso角度/COM X/Y command無効 | PASS | optional M4.2.3は未実装 |
| timeout/FAILEDで歩行を強行しない | PASS | pendingを破棄 |
| state clear経路 | PASS | 指定経路にclearを追加 |
| 腕操作維持 | PASS | CHEST相対EE拘束を維持 |
| 許可条件の責務分離 | PASS | velocity、projection、static integration、height holdを分離 |
| rejected trialの統合/除去 | PASS | 識別子と既定値変更を残していない |
| 必要parameterだけIDL追加 | PASS | timeout、return/handoff、settle、READY閾値 |
| 500 Hz対策 | PASS | 追加IK solveなし。buffer/constraintは既存初期化方針 |
| M4.2 safe candidate採用条件維持 | PASS | 採用条件は緩和していない |
| M4.2.1 ZMP修正維持 | PASS | 時間構造を維持 |
| build成功 | PASS | `catkin build ... --force-cmake` 成功 |
| 実ロボット相当の歩行開始安全性 | シミュレータ未確認 | ログと挙動確認が必要 |
| COM高さ保持の実挙動 | シミュレータ未確認 | Z/refdz/l/omegaの連続性確認が必要 |
| 腕同時操作中の歩行準備 | シミュレータ未確認 | CHEST相対拘束と足/安全優先の確認が必要 |
| 500 Hz実時間性能 | シミュレータ未確認 | debug時間と周期落ち確認が必要 |

#### 13. 未解決事項

- シミュレータで歩行準備遷移を通したCOM高さ、CHEST姿勢、COM XY、root error、joint delta、release eventの時系列確認が未実施。
- `wbmsDebugOut` のlogger/viewer設定は運用側で追加が必要。
- projector invalidが継続する既存シミュレータ問題は、M4.1 status/metricsで切り分ける必要がある。
- final IKのCHEST姿勢weight、COM weight、各READY閾値は保守的初期値であり、シミュレータ確認後に調整が必要。
- self collision入力数が周期中に増えた場合の `resize()` / constraint生成はM2時点の許容範囲として残る。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md`、`auto_stabilizer/log/` は未追跡として存在する。不要に削除しない。

#### 14. 次のWork Packageまたはsimulatorへの引き継ぎ

- simulatorで `goVelocity`、`goPos`、`setFootSteps` それぞれについてREQUESTEDからWALKING_HOLDまでのphase遷移を確認する。
- WBMS中にCOM Zをずらしてから歩行開始し、`heldRobotComHeightInFootMid`、`refdz`、`l.z`、`omega`、`genCog.z` が連続であることを確認する。
- COM/torso速度指令が残った状態で歩行開始し、DECELERATINGからRETURNINGへ入る瞬間にtarget巻き戻りがないことを確認する。
- timeout、NONFINITE、unsafe候補時にpending commandが破棄され、歩行が強行されないことを確認する。
- 腕EE操作を継続したまま歩行準備・歩行開始し、CHEST相対拘束が維持され、足・安全・バランスが優先されることを確認する。
- optional M4.2.3を行う場合は、歩行中COM Z速度操作だけを独立Work Packageとして扱い、COM X/Yと体幹角度操作は引き続き無効のままにする。

結果はM3対象ファイルと進捗文書のみの変更である。

### `/review`で報告された重要な指摘と対応

#### 指摘1: `onExecute()` 時間がOutPort書き込み時間を含まない

- 分類: 修正対象。
- 問題: `writeOutPortData()` 前に `wbmsDebugOut[29]` 用の値を確定すると、新規 `wbmsDebugOut` 自身を含むOutPort書き込み時間が計測値に含まれない。
- 修正:
  - `AutoStabilizer::onExecute()` で `writeOutPortData()` 後に `debugData.onExecuteTime` を更新するようにした。
  - `wbmsDebugOut[29]` は前回出力更新周期の確定値として扱う。
- 確認:
  - 該当コード再確認、ビルド成功、静的確認実施。

#### 指摘2: 投影無効時にも最終IKのCHEST拘束が残る

- 分類: 修正対象。
- 問題: 歩行開始遅延や投影失敗で `wbmsPostureReferenceValid=false` の周期でもCHEST姿勢拘束が残ると、root姿勢復帰・通常COM安定化と競合する。
- 修正:
  - `FullbodyIKSolver` のCHEST拘束追加条件を `chestLink && gaitParam.wbmsPostureReferenceValid` にした。
  - fallback時は `wbmsPostureReferenceJointMask` も全falseにする。
- 確認:
  - 該当コード再確認、ビルド成功、静的確認実施。

#### 指摘3: 最終review結果

- 分類: 対応不要。
- 内容: CHEST相対腕拘束維持、旧root reference IK除去、COM/reference angle mode blend、debug OutPort追加、計測追加に対して、差分上で明確な破壊的問題は見つからなかった。ビルドも成功している。
- 対応:
  - 追加コード変更なし。

## M4.1記録: statusとvalidation metrics

### マイルストーン番号と名称

マイルストーン4.1: statusとvalidation metrics。

### 完了状態

実装、ビルド、review対応は完了。M4.1ではprojectorの失敗理由とcandidate validation metricsを観測可能にした。

この段階では、正式追加計画どおりcandidate採用条件を変更していない。`solveIKLoop()` の戻り値は `allConstraintsSatisfied` として記録するが、M3と同じく `allConstraintsSatisfied && validation.safe` の両方がtrueの場合だけ `wbmsPostureReferenceValid=true` になる。

### 実装した範囲

- `GaitParam` に `WbmsProjectionStatus` enumとprojection debug stateを追加した。
- `WbmsPostureControl` に `ProjectionValidationResult` を追加した。
- 旧 `validateProjection()` を `validateProjectionCandidate()` へ変更し、candidate safety、失敗理由status、validation metricsを返すようにした。
- target生成前の失敗理由をstatusへ記録するようにした。
  - 操作不可: `WBMS_PROJECTION_DISABLED`
  - baseline invalid: `WBMS_PROJECTION_BASELINE_INVALID`
  - 投影variableなし: `WBMS_PROJECTION_NO_VARIABLE`
  - target生成失敗: `WBMS_PROJECTION_TARGET_INVALID`
  - support hull失敗: `WBMS_PROJECTION_SUPPORT_HULL_INVALID`
- validation失敗理由をstatusへ記録するようにした。
  - 非finite: `WBMS_PROJECTION_INVALID_NONFINITE`
  - joint limit: `WBMS_PROJECTION_INVALID_JOINT_LIMIT`
  - joint step: `WBMS_PROJECTION_INVALID_JOINT_STEP`
  - root translation: `WBMS_PROJECTION_INVALID_ROOT_TRANSLATION`
  - root rotation: `WBMS_PROJECTION_INVALID_ROOT_ROTATION`
  - foot position: `WBMS_PROJECTION_INVALID_FOOT_POSITION`
  - foot rotation: `WBMS_PROJECTION_INVALID_FOOT_ROTATION`
- `solveIKLoop()` のbool戻り値を `allConstraintsSatisfied` として保存し、debugへ出すようにした。
- `validation.safe && !allConstraintsSatisfied` の暫定fallback経路を `WBMS_PROJECTION_VALID_BLOCKED` として記録するようにした。
- `setFallbackReference()` はstatusとmetricsを上書きしない。fallback後も直前の失敗理由が `wbmsDebugOut` に残る。
- `wbmsDebugOut` を30要素から40要素へ拡張した。既存index 0-29の順序と意味は変更していない。
- M4.1実装内容、review対応、未解決事項、次マイルストーンへの引き継ぎを本進捗文書へ記録した。

### 意図的に実装しなかった後続範囲

- M4.2のsafe candidate採用は未実装。
- `allConstraintsSatisfied` を採用条件から外す変更は未実装。
- `VALID_IDLE` / `VALID_ACTIVE` / `VALID_BLOCKED` の最終的な実現速度norm分類は未実装。M4.1では、safeだが全constraint未充足でfallbackする暫定経路だけを `VALID_BLOCKED` と記録する。
- M4.3のfinal IK後COM/CHEST realized velocity debug index 40-45は未実装。
- dependency solver本体、task scaling付き速度QP、提案B、新規テストコード、IDL/API変更は実装していない。
- validation閾値、腕CHEST相対拘束、歩行開始遅延、既存COM/ZMP統合、最終IK拘束優先度は変更していない。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | `WbmsProjectionStatus` enum、projection debug state、clear時初期化を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h` | `ProjectionValidationResult`、`validateProjectionCandidate()`、`storeProjectionValidationResult()` を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | target生成前status、validation result化、metrics保存、`allConstraintsSatisfied`記録、safeだが未充足の暫定blocked statusを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | `wbmsDebugOut` index 30-39を末尾追加 |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | M4.1実装、確認結果、review対応、次マイルストーン引き継ぎを記録 |

### 追加・変更した主要class、function、state、debug index

#### class / struct

- `GaitParam::WbmsProjectionStatus`
  - projectorの状態公開用enum。制御判断ではなくdebug／診断用途。
- `WbmsPostureControl::ProjectionValidationResult`
  - candidate安全判定、status、root/joint/foot metricsをまとめて返す内部struct。

#### function

- `WbmsPostureControl::calcProjectionTargets(...)`
  - support hull有効性を呼び出し側へ返す引数を追加した。
- `WbmsPostureControl::validateProjectionCandidate()`
  - 旧 `validateProjection()` を置換し、boolではなく `ProjectionValidationResult` を返す。
- `WbmsPostureControl::storeProjectionValidationResult()`
  - validation resultを `GaitParam` のdebug stateへ保存する。
- `WbmsPostureControl::solveProjection()`
  - 周期開始時にprojection debug stateを初期化する。
  - candidate生成前失敗、solver戻り値、candidate validation resultを保存する。
  - 採用条件は `allConstraintsSatisfied && validation.safe` のまま維持する。

#### state

- `GaitParam::wbmsProjectionStatus`
- `GaitParam::wbmsProjectionAllConstraintsSatisfied`
- `GaitParam::wbmsProjectionCandidateSafe`
- `GaitParam::wbmsProjectionSupportHullValid`
- `GaitParam::wbmsProjectionRootTranslationStep`
- `GaitParam::wbmsProjectionRootRotationStep`
- `GaitParam::wbmsProjectionMaxJointStep`
- `GaitParam::wbmsProjectionMinJointLimitMargin`
- `GaitParam::wbmsProjectionMaxFootPositionError`
- `GaitParam::wbmsProjectionMaxFootRotationError`

#### `wbmsDebugOut` index

既存index 0-29はM3の定義から変更していない。M4.1で末尾へ以下を追加した。

| index | 内容 |
|---:|---|
| 30 | `wbmsProjectionStatus` |
| 31 | `wbmsProjectionAllConstraintsSatisfied` |
| 32 | `wbmsProjectionCandidateSafe` |
| 33 | `wbmsProjectionSupportHullValid` |
| 34 | candidate root translation step [m] |
| 35 | candidate root rotation step [rad] |
| 36 | candidate max joint step [rad or m] |
| 37 | candidate minimum joint limit margin |
| 38 | candidate max foot position error [m] |
| 39 | candidate max foot rotation error [rad] |

### 重要な実装判断と理由

- 採用条件をM3と同じ `allConstraintsSatisfied && validation.safe` に維持した。
  - M4.1の目的はstatusとmetricsの観測であり、safe candidate採用はM4.2範囲だからである。
- `solveIKLoop()` のboolを `allConstraintsSatisfied` として保存した。
  - dependency solverの戻り値はQP成功ではなくconstraint充足状態を表すため、名前と用途を診断値へ限定するためである。
- fallback関数ではstatus/metricsを変更しない。
  - fallback後も失敗理由を `wbmsDebugOut` に残すためである。
- safe candidateだが `allConstraintsSatisfied=false` でM4.1暫定fallbackする経路は `VALID_BLOCKED` と記録する。
  - candidateは安全だが、現行採用条件により最終IKへ接続されない経路を `VALID_ACTIVE` と表示しないためである。
- validation閾値は変更しなかった。
  - M4.1の目的は原因識別であり、閾値調整はログ取得後の別変更とする正式計画に従った。
- `wbmsDebugOut` は末尾追加のみとした。
  - M3で公開済みのindex 0-29を利用するlogger/viewer互換を壊さないためである。

### 正式計画からの差異

- `VALID_IDLE` / `VALID_ACTIVE` / `VALID_BLOCKED` のnorm分類はM4.1では行っていない。
  - 正式追加計画ではこの分類はM4.2のsafe candidate採用に含まれる。M4.1では採用条件を維持するため、暫定的に `validation.safe && !allConstraintsSatisfied` のfallback経路だけを `VALID_BLOCKED` として記録している。
- `target invalid` と `support hull invalid` は、`calcProjectionTargets()` 内でsupport hull更新まで到達したかどうかで分離している。
  - 追加計画では実装上分離できない場合は同一statusも許容されているが、現行実装では分離可能だったため分けた。
- final IK後realized velocity index 40-45は追加していない。
  - M4.3の任意後続変更であり、M4.1の完了条件ではない。

### ビルド・静的確認

#### ビルド

```sh
catkin build auto_stabilizer --no-deps
```

結果:

- 成功。
- warningsなし。

#### 静的確認

```sh
rg -n "allConstraintsSatisfied|validation\\.safe|WBMS_PROJECTION_VALID_BLOCKED|bool valid =|validateProjectionCandidate|wbmsProjectionStatus" \
  auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp \
  auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h \
  auto_stabilizer/rtc/AutoStabilizer/GaitParam.h \
  auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md
```

結果:

- `allConstraintsSatisfied` が記録されることを確認。
- `validation.safe && !allConstraintsSatisfied` の暫定blocked statusを確認。
- 採用条件が `allConstraintsSatisfied && validation.safe` のまま維持されていることを確認。
- `validateProjectionCandidate()` と `wbmsProjectionStatus` の追加箇所を確認。

```sh
rg -n "m_wbmsDebug_\\.data\\.length|wbmsProjection|data\\[index\\+\\+\\]" \
  auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp \
  auto_stabilizer/rtc/AutoStabilizer/GaitParam.h
```

結果:

- `wbmsDebugOut` の長さが40になっていることを確認。
- index 0-29の既存代入順が維持され、30-39が末尾追加であることを確認。
- projection debug stateの初期値と出力箇所を確認。

```sh
git diff --check
```

結果:

- 問題なし。

### reviewで報告された重要指摘と対応

#### 指摘1: `allConstraintsSatisfied == false` かつ candidate safe のfallback経路が `VALID_ACTIVE` と出る

- 分類: 修正対象。
- 問題:
  - M4.1では採用条件をM3同等に維持しているため、`validation.safe == true` でも `allConstraintsSatisfied == false` の場合はfallbackする。
  - その際、statusがvalidator由来の `VALID_ACTIVE` のままだと、fallbackした理由をindex 30だけで識別しにくい。
- 修正:
  - `validation.safe && !allConstraintsSatisfied` の場合、fallback前に `wbmsProjectionStatus` を `WBMS_PROJECTION_VALID_BLOCKED` へ上書きする。
  - 採用条件 `allConstraintsSatisfied && validation.safe` は変更しない。
  - `wbmsPostureReferenceValid=false` とfallback動作はM3/M4.1のまま維持する。
- 確認:
  - build成功。
  - 静的確認で `bool valid = allConstraintsSatisfied && validation.safe` が残っていることを確認。
  - M4.2のsafe candidate採用は先行実装していない。

#### 指摘2: M4.1最終review

- 分類: 対応不要。
- 内容:
  - 既存 `wbmsDebugOut` index 0-29維持、末尾追加、失敗時status、fallback後の保持、finiteな初期値、500 Hz経路での目立つallocation増加なし、採用条件維持について、差分上のブロッキングな問題は報告されなかった。
- 対応:
  - 追加コード変更なし。

### 完了条件

| 条件 | 状態 | 備考 |
|---|---|---|
| build成功 | PASS | `catkin build auto_stabilizer --no-deps` 成功、warningsなし |
| 既存debug index 0-29不変 | PASS | 既存代入順は維持し、30-39のみ末尾追加 |
| validatorの全return経路でstatus設定 | PASS | `validateProjectionCandidate()` の失敗returnにstatusを設定 |
| target生成前の失敗理由をstatus設定 | PASS | disabled、baseline、variable、target、support hullを設定 |
| fallback後も直前の失敗理由がログへ残る | PASS | `setFallbackReference()` はstatus/metricsを上書きしない |
| 制御挙動はM3完了時点と同じ | PASS | 採用条件は `allConstraintsSatisfied && validation.safe` のまま |
| M4.2のsafe candidate採用を先行しない | PASS | `allConstraintsSatisfied` は採用条件に残している |
| シミュレータでstatus/metricsが期待通り出る | シミュレータ未確認 | ログ取得が必要 |
| projector valid問題の解消 | FAIL | M4.1範囲外。M4.2で扱う |

### 未解決事項

- `allConstraintsSatisfied == false` でもcandidate safeな場合、M4.1では `WBMS_PROJECTION_VALID_BLOCKED` と診断しつつfallbackする。candidate採用はM4.2で実装する。
- M4.1で追加した `wbmsDebugOut[30-39]` の実ログ確認は未実施。
- `projector valid == 0` 問題自体はM4.1では解消していない。
- 利用側logger/viewerが `wbmsDebugOut` 40要素を記録する設定になっているかは未確認。
- `wbmsProjectionMinJointLimitMargin` は投影対象jointが空の場合は初期値のままだが、空variableは `NO_VARIABLE` でcandidate生成前にfallbackする。
- M3から残るシミュレータ上の体幹pitch非応答は、M4.1の診断値取得後に原因切り分けが必要。

### 次のマイルストーンへのinterfaceと前提条件

- M4.2は `GaitParam::wbmsProjectionAllConstraintsSatisfied` と `GaitParam::wbmsProjectionCandidateSafe` を使って、constraint完全充足とcandidate safetyをログ上で分離できる。
- M4.2では `allConstraintsSatisfied` を採用条件から外し、`validation.safe` をcandidate安全採否の主条件にする。
- M4.2ではsafe candidate採用後に `VALID_IDLE` / `VALID_ACTIVE` / `VALID_BLOCKED` のnorm分類を実装する。
- M4.2でsafe candidateを採用する場合も、M4.1で構造化したvalidation閾値とmetricsを維持する。
- `setFallbackReference()` はstatus/metricsを消さない前提で使う。
- `wbmsDebugOut` index 0-29は引き続き不変、M4.1追加分は30-39として扱う。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
  - `solveProjection()`: `allConstraintsSatisfied` と `validation.safe` のvalid判定。
  - `validateProjectionCandidate()`: statusとmetricsの設定経路。
  - `storeProjectionValidationResult()`: `GaitParam` debug stateへの保存。
  - `setFallbackReference()`: status/metricsを上書きしないfallback。
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
  - `WbmsProjectionStatus` とprojection debug stateの初期値。
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
  - `wbmsDebugOut` index 0-39の出力順。
- `auto_stabilizer/log/wbmsDebugOut.txt`
  - 次回シミュレータ確認時、index 30-39でprojector invalid原因を確認する。

## M4.2 safe candidate採用記録

### マイルストーン番号と名称

M4.2: safe candidate採用。

### 実装した範囲

- `WbmsPostureControl::solveProjection()` で、`solveIKLoop()` のbool戻り値をcandidate採用条件から外した。
- `solveIKLoop()` の戻り値は `allConstraintsSatisfied` として扱い、`gaitParam.wbmsProjectionAllConstraintsSatisfied` へ保存する診断値に限定した。
- candidate採否は `validateProjectionCandidate()` が返す `validation.safe` だけで判断するようにした。
- `validation.safe == false` の場合だけ `setFallbackReference()` へ進み、現在姿勢fallback、realized velocityゼロ、`wbmsPostureReferenceValid=false` を維持する。
- `validation.safe == true` の場合、`allConstraintsSatisfied == false` でも以下を保存して最終IKへ接続できるようにした。
  - `wbmsPostureReferenceQ`
  - `wbmsPostureReferenceJointMask`
  - `wbmsProjectedChestR`
  - `wbmsProjectedRobotCom`
  - `wbmsRealizedComVelocity`
  - `wbmsRealizedTorsoAngularVelocity`
  - `wbmsPostureReferenceValid = true`
- safe candidate採用後、applied commandとrealized velocityから `VALID_IDLE` / `VALID_ACTIVE` / `VALID_BLOCKED` を分類する処理を追加した。
- `VALID_ACTIVE` / `VALID_BLOCKED` 判定では、COMとtorsoを別々に扱い、指令された軸に対応するrealized velocity normだけを見るようにした。
- hidden goalや未達成残差を次周期へ保持する処理は追加していない。従来どおり毎周期 `genRobot` から投影robotを同期し、1周期候補だけを生成する。

### 意図的に実装しなかった後続範囲

- final IK後のCOM realized velocityとCHEST realized angular velocityを `wbmsDebugOut[40-45]` へ追加するM4.3候補は実装していない。
- `wbmsDebugOut` 長を46へ拡張する変更は行っていない。M4.2ではM4.1で追加済みのindex 30-39を維持する。
- 指令方向への内積判定は実装していない。M4.2では計画書8.5の「初回実装ではnorm判定でよい」に従い、指令軸だけを抽出したnorm判定に留めた。
- validation閾値、weight、maxError、COM/CHEST優先度、dependency solver、task scaling、parameter tuningは変更していない。
- self collision完全充足や `JointVelocityConstraint` 完全充足を新しいcandidate validation条件へ追加していない。
- 腕CHEST相対拘束、歩行中COM操作、新規テストコードは追加していない。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | `allConstraintsSatisfied && validation.safe` の採用条件を削除し、`validation.safe` のみでfallbackを判断するよう変更。safe candidate保存後にIDLE/ACTIVE/BLOCKED分類を追加 |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | M4.2の実装内容、review対応、確認結果、未解決事項、次マイルストーンへの引き継ぎを追記 |

### 追加・変更した主要class、function、state、debug index

#### class

- 新規classは追加していない。
- `WbmsPostureControl` の `solveProjection()` 内の採用フローとstatus分類だけを変更した。

#### function

- `WbmsPostureControl::solveProjection()`
  - `allConstraintsSatisfied` をdebug保存専用へ変更。
  - unsafe candidateだけfallbackするように変更。
  - safe candidateの投影姿勢、CHEST、COM、realized velocity、valid flag保存を `allConstraintsSatisfied` から独立させた。
  - safe candidate採用後にIDLE/ACTIVE/BLOCKED分類を実行するようにした。
- `WbmsPostureControl::validateProjectionCandidate()`
  - 閾値と検査項目は変更していない。

#### state

- 新規stateは追加していない。
- M4.1で追加済みの以下をM4.2で採用フローに使用した。
  - `wbmsProjectionAllConstraintsSatisfied`
  - `wbmsProjectionCandidateSafe`
  - `wbmsProjectionStatus`
  - `wbmsPostureReferenceQ`
  - `wbmsPostureReferenceJointMask`
  - `wbmsProjectedChestR`
  - `wbmsProjectedRobotCom`
  - `wbmsRealizedComVelocity`
  - `wbmsRealizedTorsoAngularVelocity`
  - `wbmsPostureReferenceValid`

#### debug index

M4.2ではdebug indexの追加・順序変更は行っていない。M4.1で追加済みの以下をそのまま使う。

| index | 内容 |
|---:|---|
| 30 | `wbmsProjectionStatus` |
| 31 | `wbmsProjectionAllConstraintsSatisfied` |
| 32 | `wbmsProjectionCandidateSafe` |
| 33 | `wbmsProjectionSupportHullValid` |
| 34 | candidate root translation step [m] |
| 35 | candidate root rotation step [rad] |
| 36 | candidate max joint step [rad or m] |
| 37 | candidate minimum joint limit margin |
| 38 | candidate max foot position error [m] |
| 39 | candidate max foot rotation error [rad] |

### 重要な実装判断とその理由

- `allConstraintsSatisfied` を採用条件へ使わない。
  - 計画書5.1が、`solveIKLoop()` のbool戻り値を安全性判定として扱わず、debug出力と観測に限定すると定めているため。
- candidate採用条件を `validation.safe == true` に限定する。
  - 計画書5.2と8.4が、安全性と全constraint完全充足を分離し、独立validationを通ったcandidateを採用する方針を定めているため。
- `VALID_ACTIVE` / `VALID_BLOCKED` は指令された軸のrealized velocity normで判定する。
  - 未指令軸の副作用的な移動だけでACTIVEにすると、計画書5.4の「非ゼロ指令に対する実現速度が極小」を表すBLOCKED診断が崩れるため。
- unsafe candidateのfallbackは維持する。
  - 数値異常、limit違反、足誤差過大、root変位過大などは候補を破棄して現在姿勢へ戻す安全側挙動が必要なため。
- hidden goalを作らない。
  - 毎周期 `syncProjectionRobot()` で現在の `genRobot` から投影を開始する既存構造を維持し、blocked時も未達成目標を次周期へ蓄積しないため。

### 正式計画からの差異

- 計画書8.5の概念例はrealized velocity全体のnormでACTIVE判定しているが、review指摘を受け、実装では指令された軸だけのrealized velocity normを使う。
  - これは指令方向への内積判定ではなく、初回実装のnorm判定の範囲内で、未指令軸の副作用をACTIVE判定から除外するための限定的な差異である。
- M4.2ではdebug index 30-39を新規追加していない。
  - M4.1で追加済みであり、M4.2ではその値を使って採用条件とstatus分類を確認する。
- self collision完全充足と `JointVelocityConstraint` 完全充足をvalidationへ追加していない。
  - 計画書5.2のcandidate採用最低条件に含まれておらず、追加すると新しいstatusや閾値設計を伴うため、M4.2では扱わない。

### 実行したビルド・静的確認コマンドと結果

M4.2実装後およびreview対応後に以下を実行した。

```sh
catkin build auto_stabilizer --no-deps
git diff --check
rg -n "solved|allConstraintsSatisfied|validateProjection|wbmsPostureReferenceValid" auto_stabilizer/rtc/AutoStabilizer
```

- `catkin build auto_stabilizer --no-deps`: 成功。
- `git diff --check`: 指摘なし。
- `rg`: `solved` は該当なし。`allConstraintsSatisfied` は `solveIKLoop()` 戻り値の変数宣言と `wbmsProjectionAllConstraintsSatisfied` 保存だけに残る。

### reviewで報告された重要指摘と対応

#### 指摘1: `VALID_ACTIVE` / `VALID_BLOCKED` が未指令軸の移動で誤分類され得る

- 分類: 修正対象。
- 対応: 指令ベクトルで非ゼロの軸だけを抽出し、その軸に対応するrealized velocity normでACTIVE/BLOCKEDを判定するよう修正した。
- 補足: 指令方向への内積判定や閾値変更は行っていない。

#### 指摘2: `allConstraintsSatisfied` を採用条件から外すとsolver内の安全制約未充足候補が採用され得る

- 分類: 対応不要。
- 判断理由: 計画書5.1は `allConstraintsSatisfied` をdebug専用とし、採用可否に直接使わないことを明記している。また計画書5.2のcandidate採用最低条件は、finite、joint limit、1周期関節変位、root変位、足誤差、support hull/target、projected CHEST/COM/realized velocityの検証であり、self collision完全充足はvalidation最低条件に含まれていない。
- 対応: コード変更なし。`JointVelocityConstraint` の完全充足やself collision完全充足を採用条件へ追加すると、`allConstraintsSatisfied`相当を採用条件へ戻す、または新しいvalidation項目・status・閾値を追加する設計変更になるため、M4.2では行わない。

### 完了条件ごとの結果

| 完了条件 | 結果 | 備考 |
|---|---|---|
| `rg`で`solved && validateProjection`相当が残っていない | PASS | `solved` は該当なし |
| `allConstraintsSatisfied` がdebug以外の採用条件に使われていない | PASS | 変数宣言と `wbmsProjectionAllConstraintsSatisfied` 保存のみ |
| unsafe candidateでは従来どおりfallbackする | PASS | `!validation.safe` で `setFallbackReference()` へ進む |
| safe candidateでは `allConstraintsSatisfied == false` でもvalidになり得る | PASS | 採用条件は `validation.safe` のみ |
| safe candidateが最終IKへ接続される | PASS | `wbmsPostureReferenceValid=true`、joint mask、CHEST、COM、realized velocityを保存する |
| IDLE/ACTIVE/BLOCKEDを区別する | PASS | applied commandと指令軸realized velocityでstatus設定する |
| hidden goalや未達成残差を次周期へ保持しない | PASS | 新しい蓄積stateは追加していない。毎周期現在姿勢から投影する既存構造を維持 |
| build成功 | PASS | `catkin build auto_stabilizer --no-deps` 成功 |
| `git diff --check` 指摘なし | PASS | 指摘なし |
| シミュレータでstatus/valid/realized velocityが期待通り出る | シミュレータ未確認 | ログ取得が必要 |
| projector valid問題の解消 | シミュレータ未確認 | M4.2の主目的だが、実ログ確認は未実施 |

### 未解決事項

- シミュレータで `wbmsDebugOut[26]` のprojector valid flagが1になるかは未確認。
- `wbmsDebugOut[30]` のstatusが `20` / `21` / `22` としてIDLE、ACTIVE、BLOCKEDを期待通り区別するかは未確認。
- `wbmsDebugOut[31] == 0` かつ `wbmsDebugOut[32] == 1` の周期で、safe candidateが採用されるかは未確認。
- BLOCKED時にhidden goalが蓄積せず、逆方向入力へ反応するかは未確認。
- self collision近傍での挙動がシミュレータ上で安全側に見えるかは未確認。
- 利用側logger/viewerが `wbmsDebugOut` 40要素を記録する設定になっているかは未確認。

### 次のマイルストーンへのinterfaceと前提条件

- `WbmsPostureControl::solveProjection()` は、`validation.safe == true` のcandidateを採用し、`allConstraintsSatisfied` はdebug値としてだけ公開する。
- 投影成功時は `wbmsPostureReferenceValid=true`、`wbmsPostureReferenceQ`、`wbmsPostureReferenceJointMask`、`wbmsProjectedChestR`、`wbmsProjectedRobotCom`、realized velocityが更新される。
- 投影失敗時は `wbmsPostureReferenceValid=false`、joint mask全false、realized velocityゼロ、現在姿勢fallbackになる。
- `wbmsProjectionStatus` は、candidate生成前失敗、INVALID、VALID_IDLE、VALID_ACTIVE、VALID_BLOCKEDを区別する。
- `wbmsDebugOut` は40要素のまま。index 0-29はM3定義、30-39はM4.1定義を維持する。
- M4.3で最終IK伝達診断を追加する場合は、計画書9.3のindex 40-45を候補として扱う。
- M4.3へ進む前に、M4.2後のシミュレータログでprojector valid、status、candidate safe、all constraints satisfied、realized velocityを確認する。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
  - `solveProjection()`: `allConstraintsSatisfied` 保存、`validation.safe` fallback、safe candidate保存、IDLE/ACTIVE/BLOCKED分類。
  - `validateProjectionCandidate()`: finite、joint limit、joint step、root step、foot errorの検証範囲。
  - `setFallbackReference()`: invalid時の現在姿勢fallbackとdebug値保持。
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
  - `wbmsPostureReferenceValid` を使うCHEST姿勢拘束。
  - `wbmsPostureReferenceJointMask` を使うreference angle blend。
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
  - `wbmsDebugOut` index 0-39の出力順。
- `auto_stabilizer/log/wbmsDebugOut.txt`
  - 次回シミュレータ確認時、index 26、30-39、12-20を確認する。

## M4.2.1 ZMP軌道破綻修正記録

### マイルストーン番号と名称

M4.2.1: ZMP軌道破綻修正。

### 実装した範囲

- M4.2でsafe candidateが採用されるようになった結果、static WBMS中に `applyStaticComZmpIntegration()` が実行されるようになった。
- `applyStaticComZmpIntegration()` が `refZmpTraj` を0秒の1点軌道へ置き換えていたため、後段の `footguidedcontroller::calcFootGuidedControl()` でZMP軌道の時間和が0になり、ゼロ除算チェックにかかっていた。
- `refZmpTraj` を0秒軌道へ潰す処理をやめ、既存preview軌道を `blendedZmp - nominalZmp` だけ平行移動するようにした。
- 既存 `refZmpTraj` が空、または時間和が0の場合だけ、最低1周期分の定常ZMP軌道を作るfallbackを追加した。
- M4.2のcandidate採用条件、validation閾値、debug index、最終IK接続条件は変更していない。

### 意図的に実装しなかった後続範囲

- 歩行開始時・歩行中の体幹/COM操縦と歩行安定化の両立は未解決として残した。
- final IK後のCOM/CHEST realized velocityを `wbmsDebugOut[40-45]` へ追加するM4.3相当の診断拡張は行っていない。
- ZMP軌道の時間和が0になる根本原因を `FootGuidedController` 側で吸収する変更は行っていない。
- ZMP軌道生成全体の再設計、COM/ZMP制御則変更、歩行中COM操作、task scaling、solver変更、validation閾値変更は行っていない。
- 新規テストコードは追加していない。

### 変更ファイルと変更概要

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | `applyStaticComZmpIntegration()` の `refZmpTraj` 更新を、0秒1点軌道への置換から既存preview軌道の平行移動へ変更。空または時間和0の軌道だけ1周期以上の定常軌道へfallback |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | M4.2.1記録を追記 |

歩行移行問題の試行として `WbmsPostureControl.h`、`WbmsPostureControl.cpp`、`GaitParam.h` に別差分がworking treeへ存在するが、跳ねる挙動は未解決であるためM4.2.1のコミット対象には含めない。

### 追加・変更した主要class、function、state、debug index

#### class

- 新規classは追加していない。

#### function

- `WbmsPostureControl::applyStaticComZmpIntegration()`
  - `refZmpTraj.clear()` 後に0秒軌道を1つだけ入れる処理を削除。
  - `nominalZmp` から `blendedZmp` へのoffsetを既存preview軌道全体へ加える処理を追加。
  - 既存軌道が空または時間和0の場合だけ、`dt` 以上の定常軌道を生成するfallbackを追加。
- `WbmsPostureControl::isProjectionReferenceAllowed()`
  - 歩行移行問題への試行としてworking treeに存在するが、M4.2.1のコミット対象ではない。歩行開始遅延中も投影referenceを維持し、`wbmsOperationModeValue` で滑らかに抜くことを狙った。
  - シミュレータでは歩行指令時の跳ねは解消しなかったため、未解決事項として扱う。

#### state

- ZMP軌道破綻修正として新規stateは追加していない。
- 歩行移行問題への試行として、`wbmsWalkingStabilityStartTime` の既定値を2.0秒から5.0秒へ変更する差分がworking treeに存在するが、M4.2.1のコミット対象ではない。

#### debug index

- debug indexの追加・順序変更は行っていない。
- 既存の `wbmsDebugOut[24]` (`wbmsOperationModeValue`)、`[25]` (`wbmsWalkingStabilityModeValue`)、`[26]` (`wbmsPostureReferenceValid`)、`[30-39]` を確認対象とする。

### 重要な実装判断とその理由

- `refZmpTraj` の総時間を0にしない。
  - `FootGuidedController.h` の `calcFootGuidedControl()` は、入力ZMP軌道の時間和が0だと分母 `1 - exp(-2 * w * Tj)` が0になり破綻するため。
- 既存preview軌道を平行移動する。
  - 軌道時間構造を維持でき、foot guided controlの終端条件を不要に壊さないため。
- 空または時間和0の場合だけfallbackする。
  - 通常の `LegCoordsGenerator::calcLegCoords()` が作る正の時間を持つpreview軌道を優先し、異常時だけ安全側に最低1周期分の定常軌道を作るため。
- M4.2のsafe candidate採用条件には触れない。
  - 正式計画では `allConstraintsSatisfied` を採用条件へ戻さないことがM4.2の目的であり、今回の問題はZMP軌道更新側の破綻であるため。

### 正式計画からの差異

- `WBMSProjectionAcceptanceFixImplementationPlan.md` は主にcandidate採用条件とdebug拡張を扱っており、`refZmpTraj` の具体的な更新方法までは定義していない。
- 本修正は、M4.2でsafe candidateが採用されるようになったことで表面化した既存COM/ZMP統合経路の破綻を直す補修である。
- M4.2.1ではdebug indexやstatus enumは変更していない。
- 歩行開始時の跳ね対策として、歩行開始遅延中も投影referenceを維持する試行と `wbmsWalkingStabilityStartTime=5.0` への変更を行ったが、シミュレータで跳ねが残ったため、正式な解決策としては確定していない。

### 実行したビルド・静的確認コマンドと結果

M4.2.1実装後に以下を実行した。

```sh
catkin build auto_stabilizer --no-deps
git diff --check
rg -n "solved|allConstraintsSatisfied|validateProjection|wbmsPostureReferenceValid" auto_stabilizer/rtc/AutoStabilizer
```

- `catkin build auto_stabilizer --no-deps`: 成功。
- `git diff --check`: 指摘なし。
- `rg`: `solved` は該当なし。`allConstraintsSatisfied` は `solveIKLoop()` 戻り値の変数宣言と `wbmsProjectionAllConstraintsSatisfied` 保存だけに残る。

### reviewで報告された重要指摘と対応

M4.2.1に対する `/review` は未実施。

シミュレータ確認で、`startWholeBodyMasterSlave()` 直後から `[calcFootGuidedControl] (1 - exp(-2 * w * Tj))==0 !` が連続出力され、足踏みを繰り返す問題が報告された。これはreview指摘ではないが、M4.2.1で修正対象とした。

対応:

- `applyStaticComZmpIntegration()` が `refZmpTraj` を0秒1点軌道へ置換する処理を削除した。
- 既存preview軌道をZMP offsetで平行移動する処理へ置き換えた。
- 空または時間和0の場合だけ、正の時間を持つ定常軌道を生成するfallbackを追加した。

### 完了条件ごとの結果

| 完了条件 | 結果 | 備考 |
|---|---|---|
| `refZmpTraj` を0秒1点軌道へ潰さない | PASS | 通常時は既存preview軌道を平行移動する |
| 空または時間和0のZMP軌道でfallbackする | PASS | `dt` 以上の定常軌道を作る |
| M4.2のsafe candidate採用条件を維持する | PASS | `allConstraintsSatisfied` はdebug専用のまま |
| build成功 | PASS | `catkin build auto_stabilizer --no-deps` 成功 |
| `git diff --check` 指摘なし | PASS | 指摘なし |
| start直後の `calcFootGuidedControl` 連続エラーが止まる | シミュレータ未確認 | 修正後のログ取得が必要 |
| 体幹角速度指令でほぼ等速に傾く | PASS | ユーザー確認済み |
| COM Z速度指令でしゃがみながら前屈する | PASS | ユーザー確認済み |
| 体幹を傾けた状態から `goPos` / `goVelocity` で安定に歩行移行する | FAIL | 跳ねる挙動が残る |

### 未解決事項

- 体幹を傾けた状態で `goPos` / `goVelocity` を送ると、歩行開始時に跳ねる挙動が残る。
- 歩行移行時は重心・体幹操縦よりバランスを優先し、腕操縦は歩行中も可能にする方針だが、その実装は未確定。
- 試行した内容:
  - `isOperationAllowed()` とは別に `isProjectionReferenceAllowed()` を追加し、歩行開始遅延中も投影referenceを維持するようにした。
  - `solveProjection()` のcandidate生成前判定を `isProjectionReferenceAllowed()` へ変更した。
  - 速度指令とCOM/ZMP統合は従来通り `isOperationAllowed()` で止める構造にした。
  - `wbmsWalkingStabilityStartTime` の既定値を5.0秒へ変更した。
- 試行結果:
  - シミュレータでは、体幹を傾けた状態から `goPos` / `goVelocity` を送ると、以前と同様に跳ねる挙動が残った。
  - この試行差分は現在のworking treeに残っているが、解決策としては未確定であり、M4.2.1のコミット対象には含めない。
- 次に切り分けるべき仮説:
  - 歩行開始遅延中に最終IKのroot姿勢・COM Z復帰と投影referenceが同時に競合している。
  - `wbmsOperationModeValue` は滑らかでも、`refRobot` / `genRobot` / `stTargetRootPose` / COM targetの基準が不連続に切り替わっている。
  - 腕CHEST相対拘束は歩行中も維持される一方、CHEST姿勢拘束やreference angle blendの消し方が歩行開始遷移に対して適切でない。
  - `WbmsWalkingCommandDelay` の遅延中にfuture stepは生成していないが、姿勢復帰だけで既に下半身IKが大きく動いている可能性がある。

### 次のマイルストーンへのinterfaceと前提条件

- M4.2.1後の `applyStaticComZmpIntegration()` は、`refZmpTraj` の時間構造を維持する前提でZMP offsetを加える。
- `refZmpTraj` が空または時間和0の場合でも、正の時間を持つ定常軌道へfallbackする。
- M4.2のsafe candidate採用条件、projection status、debug index 30-39は維持されている。
- 歩行移行問題を扱う次マイルストーンでは、現在working treeに残っている `isProjectionReferenceAllowed()` 試行と `wbmsWalkingStabilityStartTime=5.0` 変更を、採用するか戻すかを明示的に判断する必要がある。
- 歩行中・歩行開始遷移中は、体幹/COM操縦指令よりバランスを優先する。ただし腕操縦はCHEST相対拘束を通して継続可能にする方針を維持する。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
  - `applyStaticComZmpIntegration()`: `refZmpTraj` の平行移動とfallback。
  - `isOperationAllowed()`: static WBMS操作、速度指令、COM/ZMP統合の許可条件。
  - `isProjectionReferenceAllowed()`: 歩行開始遅延中reference維持の試行差分。
  - `solveProjection()`: projection referenceの有効/無効判定。
- `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp`
  - `startDelay()` と `proc()` による歩行開始遅延、pending command投入タイミング。
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
  - `wbmsOperationModeValue` によるCHEST姿勢拘束weight、COM weight、reference angle blend。
  - 上半身EEのCHEST相対拘束。
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
  - `onExecute()` の `wbmsWalkingCommandDelay_.proc()` と `execAutoStabilizer()` の呼び順。
  - `wbmsDebugOut` index 24-26、30-39。

## M4.2.2 Work Package A 遷移診断記録

### 実装範囲

Work Package Aでは、制御挙動を変更せず、歩行準備遷移を診断するためのdebug出力だけを追加した。

- `wbmsDebugOut` を40要素から60要素へ拡張した。
- 既存index 0-39の順序と意味を維持した。
- index 40-45へ、projector出力ではなくfinal IK後の実現COM速度とCHEST角速度を追加した。
- index 46以降へ、現行delay状態、経過時間、予約値、現在COM高さ、姿勢誤差、最大関節差分、pending command release event、failure予約値、実行時 `wbmsWalkingStabilityStartTime` を追加した。
- qRef欠落、RTC activate/deactivate、`MODE_SYNC_TO_ABC` 初期化、`GaitParam::reset()` でfinal IK診断の前回値を無効化し、非連続サンプル間の差分を通常dtで割らないようにした。
- 500 Hz経路でdebug用の大きな毎周期allocationを追加しないよう、前回関節角bufferは初期化時に確保し、毎周期は既存vectorの要素更新に限定した。

### 対象外

承認済み計画のWork Package Aに従い、以下は実装していない。

- M4.2.2必須遷移本体のphase state machine。
- COM高さsnapshot、保持、RETURNING、HANDOFF。
- readiness判定、timeoutによる歩行抑止、READY次周期release。
- 歩行開始タイミング、制御weight、target、projector許可条件の変更。
- rejected trialである `isProjectionReferenceAllowed()` 追加と `wbmsWalkingStabilityStartTime` 既定値延長の採用。
- optional M4.2.3。
- dependency solver変更、task scaling、validation閾値変更。
- 新規テストコード追加。

### 変更ファイル

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | final IK後診断の計算、debug index 40-59の出力、非連続サンプル時の診断reset呼び出しを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | final IK後診断値、前回値、最大関節差分、pending release event、reset helperを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp` | pending command release eventをdebug用に1周期だけ立てる処理を追加 |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | 本記録を追記 |

### phase/state/debug index

#### 既存index

`wbmsDebugOut[0-39]` は既存M3/M4.1定義を維持する。順序変更、意味変更、削除は行っていない。

#### Work Package Aで確定したindex

| index | 意味 | 単位・値 |
|---|---|---|
| 40 | final IK後robot COM realized velocity X | foot-mid座標、m/s |
| 41 | final IK後robot COM realized velocity Y | foot-mid座標、m/s |
| 42 | final IK後robot COM realized velocity Z | foot-mid座標、m/s |
| 43 | final IK後CHEST realized angular velocity roll軸相当 | foot-mid軸、rad/s |
| 44 | final IK後CHEST realized angular velocity pitch軸相当 | foot-mid軸、rad/s |
| 45 | final IK後CHEST realized angular velocity yaw軸相当 | foot-mid軸、rad/s |
| 46 | 現行phase相当のdelay状態 | `isWbmsWalkingStartDelay ? 1 : 0` |
| 47 | delay経過時間 | delay中は `startTime - remainTime`、非delay時0、s |
| 48 | return alpha予約値 | 0 |
| 49 | handoff alpha予約値 | 0 |
| 50 | held COM height予約値 | 0 |
| 51 | current robot COM height | foot-mid座標Z、m |
| 52 | CHEST基準姿勢error | WBMS開始時baselineからの角度誤差、rad |
| 53 | COM XY error予約値 | 0 |
| 54 | COM Z hold error予約値 | 0 |
| 55 | root姿勢error | `genRobot` rootと `stTargetRootPose` の角度誤差、rad |
| 56 | max joint delta per cycle | final IK後、前回診断サンプルとの差分最大値 |
| 57 | pending command release event | 現周期でlegacy delay pending commandをreleaseしたら1 |
| 58 | timeout/failure code予約値 | 0 |
| 59 | runtime `wbmsWalkingStabilityStartTime` | s |

### held COM heightの定義

Work Package AではCOM高さ保持本体を実装しないため、`held COM height` は予約値として `0.0` を出力する。

後続Work Package Bで意味を持つ場合は、歩行準備phaseでsnapshotしたrobot COMのfoot-mid座標Zを `heldRobotComHeightInFootMid` 相当として扱う想定である。ただし本記録時点ではsnapshot stateは未実装であり、`0.0` は「保持高さ0m」ではなく「未接続のfinite neutral値」である。

### COM X/YとZの扱い

- index 40-42のfinal IK後COM realized velocityは、final IK後の `genRobot->centerOfMass()` をfoot-mid座標へ変換し、前回診断サンプルとの差分を実経過サンプルとしてdtで割る。
- index 51のcurrent robot COM heightは、final IK後のrobot COMをfoot-mid座標へ変換したZである。
- index 53のCOM XY errorは、Work Package AではXY readinessやreturn/handoffを実装しないため予約値 `0.0` とする。
- index 54のCOM Z hold errorは、COM高さ保持が未実装のため予約値 `0.0` とする。
- COM XYとZの制御target、許可条件、統合経路は変更していない。

### 腕操作維持

Work Package Aでは腕操作経路を変更していない。投影IK variable外の腕関節は従来referenceを使い、上半身EEのCHEST相対拘束を維持する既存方針をそのまま残している。

歩行準備中に体幹/COM操縦をどう抜くか、腕操作をどう維持するかの本体制御はWork Package B以降の対象であり、本Packageではdebug上の観測点追加に限定した。

### 重要判断

- 40-45はprojectorの `wbmsRealized*` ではなく、final IK後の `genRobot` から計算する。
- 40-45の座標系は、既存raw/applied/projector realized velocityと比較できるようfoot-mid座標/軸に揃える。
- 診断が呼ばれない周期を挟んだ場合は前回値を無効化し、古い姿勢との差分を通常dtで割らない。
- M4.2.2本体前で意味を持たないdebug値は、NaNではなくfiniteなneutral値として `0.0` を出す。
- `wbmsWalkingStabilityStartTime` の既定値は2.0秒のまま維持する。
- `isProjectionReferenceAllowed()` 追加による歩行開始遅延中projector許可は採用しない。
- untrackedのrejected trial patchは誤適用を避けるため、repository外の `/tmp/auto_stabilizer2_rejected_trials/M4-2-2.patch` へ退避した。

### 計画との差異

- Work Package Aは計画通り診断追加だけを実装した。
- 計画でWork Package Bへ割り当てられたphase state machine、COM高さ保持、readiness、timeout/failure code本体は実装していない。
- index 40-45は計画上はM4.3予約だったが、Work Package Aの明示範囲として先に実装した。
- index 46は正式phase enumではなく、現行delay状態の診断値として `0/1` を出す。正式phase番号はWork Package Bで確定する。
- index 48-50、53-54、58は、後続実装用の予約値としてfinite neutral値を出す。

### build・静的確認

Work Package A実装後に以下を確認した。

| 確認 | 結果 | 備考 |
|---|---|---|
| `catkin build auto_stabilizer --no-deps` | PASS | ビルド成功 |
| `git diff --check` | PASS | whitespace指摘なし |
| `wbmsDebugOut` lengthと代入数 | PASS | length 60、代入数60 |
| index 0-39不変 | PASS | 既存代入順の差分なし |
| 40-45の取得元 | PASS | projector出力ではなくfinal IK後の `genRobot` / `chestLink` |
| 40-45の座標系 | PASS | foot-mid座標/軸へ変換後に出力 |
| rejected trialの実コード混入 | PASS | 許可条件と既定待ち時間は既存挙動に復帰 |

### review指摘と処置

| 順序 | 指摘要約 | 分類 | 処置 |
|---|---|---|---|
| 1 | 歩行開始遅延中のprojector許可変更と既定待ち時間延長が混入している | 修正対象 | `solveProjection()` の許可条件を既存 `isOperationAllowed()` に戻し、`wbmsWalkingStabilityStartTime` 既定値を2.0秒へ戻した |
| 2 | final IK診断が非連続サンプル間を通常dtで差分して速度スパイクを出す | 修正対象 | 診断前回値reset helperを追加し、qRef欠落、reset、activate/deactivate、`MODE_SYNC_TO_ABC` で無効化した |
| 3 | final IK後実現速度が既存debugと座標系不一致 | 修正対象 | COM速度とCHEST角速度をfoot-mid座標/軸へ変換して出力するよう修正した |
| 4 | rejected trial patchが未追跡で残り、誤コミット/誤適用リスクがある | 修正対象 | repository外の `/tmp/auto_stabilizer2_rejected_trials/M4-2-2.patch` へ退避した |
| 5 | 重点項目で修正必須不具合なし | 対応不要 | 追加修正なし。index、length、40-45、build成功は確認済み |

### acceptanceごとの結果

| acceptance | 結果 | 備考 |
|---|---|---|
| build成功 | PASS | `catkin build auto_stabilizer --no-deps` 成功 |
| index 0-39不変 | PASS | 既存debug順序と意味を維持 |
| 40-45がfinal IK後実現量 | PASS | final IK後の `genRobot` とCHEST姿勢から計算 |
| 40-45がprojector値と混同されない | PASS | `wbmsRealized*` ではなく別debug stateへ保存 |
| transition debugがfinite | PASS | 未接続値は `0.0`、runtime値はfinite化 |
| 制御挙動に意図的変更なし | PASS | weight、target、許可条件、歩行開始タイミングは変更なし |
| 後続M4.2.2本体が混入していない | PASS | phase本体、COM高さ保持、readiness、timeout抑止は未実装 |
| optional M4.2.3が混入していない | PASS | optional機能は未実装 |
| 500 Hz周期に不要な大きなallocationを追加しない | PASS | 前回関節角bufferは初期化時確保 |
| simulator上でindex 40-59が期待通り記録される | シミュレータ未確認 | logger/viewer設定と実機相当ログ確認が必要 |
| 歩行遷移時の跳ねが解消する | シミュレータ未確認 | Work Package Aは診断のみであり、挙動改善は対象外 |

### 未解決事項

- Work Package Bのphase state machine、COM高さsnapshot/保持、RETURNING/HANDOFF、readiness、timeout/failure codeは未実装。
- index 48-50、53-54、58は予約値のままである。
- index 46は正式phase enumではなく現行delay状態である。
- simulator上で `wbmsDebugOut` 60要素がlogger/viewerに記録されるか未確認。
- 体幹を傾けた状態からの `goPos` / `goVelocity` で跳ねる問題は未解決であり、Work Package Aでは改善対象にしていない。
- rejected trial patchはrepository外へ退避済みだが、必要なら `/tmp/auto_stabilizer2_rejected_trials/M4-2-2.patch` から内容確認できる。

### 次のWork Packageまたはsimulatorへの引き継ぎ

- simulatorで `wbmsDebugOut[40-59]` を記録し、歩行開始遷移の不連続点を確認する。
- Work Package Bに進む前に、index 46を正式phase enumへ置き換えるmappingを確定する。
- held COM heightは、後続で歩行準備phaseのsnapshot値としてfoot-mid座標Zを保持する。
- COM XY errorとCOM Z hold errorは、後続のreadiness/height hold実装時に意味を接続する。
- timeout/failure codeは、後続のphase timeoutとfailure分類実装時に接続する。
- 腕操作維持は、既存CHEST相対拘束を壊さずに、体幹/COM遷移制御と分離して確認する。

## M4.2.2設計修正 Commit 1 service APIと歩行API gate 実装記録

### 1. 実装範囲

M4.2.2設計修正のCommit 1として、歩行準備を歩行APIから分離するためのservice APIと、WBMS中未READYの歩行API gateを実装した。

- `AutoStabilizerService.idl` に `WbmsWalkingPreparationPhase`、`WbmsWalkingPreparationState`、`startWbmsWalkingPreparation()`、`cancelWbmsWalkingPreparation()`、`getWbmsWalkingPreparationState()` を追加した。
- C++ service実装とEusLisp wrapperを追加した。
- `goVelocity`、`goPos`、`setFootSteps`、`setFootStepsWithParam` の入口で、WBMS activeかつwalking preparation READYでない場合にrejectするgateを追加した。
- 未READY reject時はfootstep生成、goVelocity mode開始、pending保存を行わず、`false` を返す。
- READY後に歩行APIが明示的に呼ばれた場合だけ既存歩行APIを実行する。READY成立だけでは歩行を自動開始しない。
- M4.2.2B以前のpending command自動release設計は無効化し、歩行指令を保存しない設計へ変更した。
- READY/WALKING_HOLD後はpreparation timeoutを進めず、operator判断または上位側ポーリング中にREADYがFAILEDへ退行しないようにした。
- 既存 `wbmsDebugOut[0-45]` の意味は変更せず、Commit 1用のphase/ready/failed/reject/accept/service eventを既存拡張領域へ追加した。

### 2. 対象外

Commit 1では次を実装していない。

- 固定秒returnの廃止後に使う速度・加速度limit型pre-walk姿勢生成本体。
- COM X/Yをnominal位置へ戻すRETURNING制御。
- COM Z保持を `refdz`、`l.z`、`omega`、`genCog.z` へ完全接続する本体。
- READY判定の完全実装と各READY閾値調整。
- 歩行中COM Z速度操作。
- task scaling、dependency solver変更、validation閾値変更。
- 新規テストコード。
- シミュレータ上の跳ね上がり解消確認。

### 3. 変更ファイル

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/idl/AutoStabilizerService.idl` | WBMS walking preparation用enum、state struct、service 3件を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizerService_impl.h/.cpp` | service methodをC++ servantへ追加 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h/.cpp` | service本体、歩行API gate、READY後accept処理、state取得、debug出力を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | walking preparation phase/failure/debug event/clear状態を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.h/.cpp` | pending保存・自動releaseを削除し、preparation開始/cancel/clear/timeout管理へ責務を縮小 |
| `auto_stabilizer/euslisp/auto-stabilizer-interface.l` | 追加service wrapperを追加し、state取得はresponseの `:state` を返すようにした |
| `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md` | 本記録を追記 |

### 4. 追加service / API gate / state machine

追加serviceは次の3件である。

```idl
boolean startWbmsWalkingPreparation();
boolean cancelWbmsWalkingPreparation();
boolean getWbmsWalkingPreparationState(out WbmsWalkingPreparationState state);
```

`startWbmsWalkingPreparation()` はWBMS active、AutoBalancer running、staticを確認し、歩行準備状態だけを開始する。歩行開始、footstep生成、goVelocity mode開始、pending保存は行わない。`cancelWbmsWalkingPreparation()` はpreparation状態をcancelし、pending歩行指令は保持しない。腕EE commandと `wbmsMode` はclearしない。

Commit 1時点のstate machineは、歩行API gateに必要な最小状態として `INACTIVE`、`REQUESTED`、`DECELERATING`、`RETURNING`、`HANDOFF`、`READY`、`WALKING_HOLD`、`FAILED` を持つ。`REQUESTED` ではsnapshotを試行し、成功時に `DECELERATING` へ進む。速度・加速度limit型のRETURNING/HANDOFF本体はCommit 2対象のため、Commit 1単体では安全側にREADYへ自動到達しない経路を残す。

歩行API gateは `goVelocity`、`goPos`、`setFootStepsWithParam` の実処理前に置き、`setFootSteps` は従来どおり `setFootStepsWithParam` へ委譲するため同じgateを通る。

### 5. `goVelocity(0,0,0)` の既存仕様維持

`goVelocity(0.0, 0.0, 0.0)` の既存意味は変更していない。

- WBMS外ではgateが発動せず、従来どおり `cmdVelGenerator_.refCmdVel` 更新と `footStepGenerator_.isGoVelocityMode = true` を行う。
- WBMS中でもwalking preparation READYまたはWALKING_HOLD後は、従来どおり歩行API実体へ進む。
- WBMS中かつ未READYの場合だけ、歩行API全体の安全gateとしてrejectする。

したがって、READY後の `goVelocity(0,0,0)` による既存のその場足踏み開始は禁止していない。

### 6. WBMS未READY時の歩行API reject仕様

WBMS activeかつwalking preparation READYでない場合、対象歩行APIは次の挙動にする。

- `false` を返す。
- footstep生成を行わない。
- `footStepGenerator.isGoVelocityMode` をtrueにしない。
- pending commandへ保存しない。
- READYになった後に自動実行しない。
- reject理由をログへ出す。
- debug eventとしてwalking API rejectedを1周期だけ出す。

このgateにより、WBMS姿勢から歩行可能姿勢へ戻る操作は `goVelocity` ではなく `startWbmsWalkingPreparation()` で開始する。

### 7. 固定秒returnの廃止または無効化

Commit 1では、旧 `WbmsWalkingCommandDelay` の「歩行APIをpending保存し、固定遅延後に自動releaseする」経路を削除または無効化した。

- `storeGoVelocity`、`storeGoPos`、`storeFootSteps`、`releasePendingCommand`、`shouldDelay` の旧pending設計は残していない。
- `hasPendingCommand()` は常にfalseを返す。
- READY成立だけでは歩行APIを自動releaseしない。
- READY/WALKING_HOLD後はpreparation timeout加算を止め、READY待機中にFAILEDへ退行しない。

固定秒で姿勢を戻す制御本体を速度・加速度limit型へ置き換える作業はCommit 2対象であり、Commit 1では自動歩行開始経路を止めるところまでを実装した。

### 8. 速度・加速度limit型pre-walk姿勢生成

Commit 1では速度・加速度limit型pre-walk姿勢生成本体は未実装である。

Commit 2で、CHEST roll/pitch/yaw、COM X/Y、root姿勢を、固定秒ではなく速度limitと加速度limitに従って歩行可能姿勢へ戻す。Commit 1ではその前提として、歩行APIと歩行準備serviceを分離し、未READY中の歩行APIをrejectする入口を確定した。

### 9. held COM heightの定義

Commit 1で扱う `heldRobotComHeightInFootMid` は、`startWbmsWalkingPreparation()` 後に `REQUESTED` を処理する制御周期でsnapshotするrobot COMのfoot-mid座標Zである。

取得元は、validな `wbmsProjectedRobotCom` を優先し、使えない場合は `genRobot->centerOfMass()` を使う。どちらもfiniteでない場合はsnapshot failureとしてFAILEDへ遷移する。

この値はWBMS開始時COM高さではなく、歩行準備開始時のfootMid基準robot COM高さである。Commit 1ではstate取得とdebug用の有限値として保持する。COM Z制御本体への完全接続はCommit 2対象である。

### 10. COM X/YとCOM Zの扱い

Commit 1ではCOM X/Y復帰とCOM Z保持の制御本体は実装していない。

- COM X/YはCommit 2で通常歩行開始に適したnominal位置へ戻す。
- COM ZはWBMS開始時高さへ戻さず、歩行準備開始時のheld COM heightを保持する方針を維持する。
- Commit 1ではsnapshot値、state値、debug値を準備し、歩行API gateと自動pending release排除を優先した。
- optionalな歩行中COM Z速度操作は実装していない。

### 11. 腕操作維持

Commit 1では腕EE commandと `wbmsMode` をclearしない。

`startWbmsWalkingPreparation()`、`cancelWbmsWalkingPreparation()`、未READY reject、READY後acceptのいずれでも、腕EE commandを消去せず、WBMS modeも停止しない。上半身EEのCHEST相対拘束は既存の `FullbodyIKSolver` 経路を維持する。腕同時操作時の実挙動はシミュレータ未確認である。

### 12. 重要判断

- `goVelocity(0,0,0)` の既存仕様を変更せず、WBMS未READY中だけ歩行API全体をrejectする。
- 歩行準備開始は歩行API流用ではなく専用serviceにする。
- 未READY中の歩行APIはpending保存しない。
- READYになっただけで自動歩行開始しない。
- READY後に `startWbmsWalkingPreparation()` が再度呼ばれてもdelay flagを再設定しない。
- READY/WALKING_HOLD後はpreparation timeoutを進めない。
- out引数service実装は生成済みCORBA C++ mappingに従う。`WbmsWalkingPreparationState_out` はfixed-size structの参照型として生成されるため、service側で `new` しない。
- 500 Hz経路にclone/newや追加IK solveを増やさない。

### 13. 計画との差異

- 計画では `WbmsWalkingCommandDelay` を `WbmsWalkingPreparationController` へ置き換える案があったが、Commit 1では差分を小さくするため既存ファイル名を維持し、責務だけpending delayからpreparation状態管理へ変更した。
- `WbmsWalkingPreparationState` のfailure codeはIDL候補どおり `long failure_code` として保持し、内部enum値を数値で返す。
- Commit 1ではREADY完全判定とpre-walk姿勢生成本体を入れず、安全側にREADYへ自動到達しない暫定状態を許容した。
- review対応により、READY/WALKING_HOLD後にtimeoutを進めない分岐をCommit 1へ含めた。これはREADY後に明示的な歩行APIを待つ設計を成立させるためである。

### 14. build・静的確認

| 確認 | 結果 | 備考 |
|---|---|---|
| `catkin build auto_stabilizer --no-deps --force-cmake` | PASS | IDL変更あり。OpenRTM helper由来のYAML warningのみ |
| `git diff --check` | PASS | whitespace指摘なし |
| service/state識別子検索 | PASS | IDL、C++ service、EusLisp wrapper、state/debug参照を確認 |
| 歩行API入口検索 | PASS | `goVelocity`、`goPos`、`setFootSteps`、`setFootStepsWithParam` の入口を確認 |
| pending旧経路検索 | PASS | `storeGoVelocity`、`storeGoPos`、`storeFootSteps`、`releasePendingCommand`、`shouldDelay` は残存なし |
| 生成C++ mapping確認 | PASS | `WbmsWalkingPreparationState_out` は参照型として生成されている |
| `git diff --stat` | PASS | Commit 1対象9ファイルの差分を確認 |

### 15. review指摘と処置

| 順序 | 指摘要約 | 分類 | 処置 |
|---|---|---|---|
| 1 | `getWbmsWalkingPreparationState` のout引数を `new` してから渡すべき | 対応不要 | 生成済みC++ mappingでは `WbmsWalkingPreparationState_out` が参照型であり、現行実装が正しい。`--force-cmake` build成功で確認 |
| 2 | READY中に再度 `startWbmsWalkingPreparation()` を呼ぶとdelay flagが残る | 修正対象 | READY/WALKING_HOLD中のstartはno-opにし、delay flagを再設定しない |
| 3 | EusLisp wrapperがservice response全体を返している | 修正対象 | `:raw-get-wbms-walking-preparation-state` を追加し、responseの `:state` を返すよう修正 |
| 4 | READY後もtimeoutが進み、明示歩行API待ち中にFAILEDへ落ちる | 修正対象 | READY/WALKING_HOLDではelapsed/timeout更新へ進まず、remain timeを0にしてreturnする |
| 5 | out引数を `new` すべきという再指摘 | 対応不要 | fixed-size structの生成mappingを再確認し、build成功を根拠に変更しない |

review全文や生ログは貼らず、処置だけを記録する。

### 16. acceptanceごとの結果

| acceptance | 結果 | 備考 |
|---|---|---|
| IDL serviceが追加されている | PASS | enum、state struct、service 3件を追加 |
| C++ service実装がある | PASS | servantとcomponent側を追加 |
| EusLisp wrapperがある | PASS | state取得は `:state` を返す |
| WBMS外の歩行API挙動維持 | PASS | gate条件はWBMS active時のみ |
| WBMS中未READYで歩行API reject | PASS | 実処理前に `false` を返す |
| 未READY rejectでfootstep生成しない | PASS | gateを生成処理前に配置 |
| 未READY rejectでgoVelocity mode開始しない | PASS | `isGoVelocityMode=true` より前にreturn |
| 未READY rejectでpending保存しない | PASS | pending保存経路を削除 |
| READY後の歩行API挙動維持 | PASS | READY/WALKING_HOLDはgateを通過 |
| READY後の `goVelocity(0,0,0)` を禁止しない | PASS | 既存goVelocity本体へ進む |
| READY成立だけで自動歩行開始しない | PASS | pending release経路なし |
| `startWbmsWalkingPreparation()` が歩行指令を保存しない | PASS | preparation状態だけを開始 |
| 腕EE commandと `wbmsMode` をclearしない | PASS | service/gateでclearしない |
| debug index 0-45を壊さない | PASS | 既存indexは維持 |
| Commit 2相当の姿勢生成本体が混入しない | PASS | 速度・加速度limit型RETURNING本体は未実装 |
| build成功 | PASS | `catkin build ... --force-cmake` 成功 |
| 実RTC service呼び出し | シミュレータ未確認 | runtimeでのservice応答確認が必要 |
| WBMS未READY rejectの実ログ確認 | シミュレータ未確認 | simulatorまたはRTC実行で確認が必要 |
| READY後の `goVelocity(0,0,0)` 足踏み開始 | シミュレータ未確認 | READYを作るCommit 2後に確認する |
| 跳ね上がり挙動改善 | シミュレータ未確認 | Commit 1はgate中心で、姿勢生成本体は未実装 |

### 17. 未解決事項

- Commit 1単体では速度・加速度limit型pre-walk姿勢生成が未実装であり、READYへ安定到達する本体はCommit 2で実装する。
- COM X/Y復帰、COM Z保持の完全接続、root姿勢復帰、READY判定はCommit 2へ残る。
- `heldRobotComHeightInFootMid` はsnapshot/state/debug用に用意したが、`refdz`、`l.z`、`omega`、`genCog.z` との完全整合はCommit 2で確認する。
- WBMS未READY reject、READY後accept、EusLisp wrapperの実service応答はシミュレータまたはRTC runtimeで未確認。
- out引数に関するreview再指摘は、生成C++ mappingとbuild成功を根拠に対応不要と判断したが、別ORB/別IDL mappingを使う環境がある場合は再確認が必要。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md`、`auto_stabilizer/docs/WBMSWalkingPreparationDesignRevisionPlan.md`、`auto_stabilizer/log/` は未追跡として存在する。不要に削除しない。

### 18. 次のcommitまたはsimulatorへの引き継ぎ

- Commit 2で速度・加速度limit型pre-walk姿勢生成を実装する。
- Commit 2でCHEST roll/pitch/yaw、COM X/Y、root姿勢の復帰target、limit、READY条件を接続する。
- Commit 2でCOM Z保持を `refdz`、`l.z`、`omega`、`genCog.z`、`genCogVel.z`、`genCogAcc.z` と整合させる。
- simulatorで `startWbmsWalkingPreparation()`、未READY `goVelocity(0,0,0)` reject、READY後 `goVelocity(0,0,0)` acceptの順序を確認する。
- simulatorでREADY後に数秒待機してもFAILEDへ退行しないことを確認する。
- 腕EE操作を継続したまま歩行準備を開始し、腕commandとCHEST相対拘束が維持されることを確認する。
- Commit 3でシミュレータ検証記録とparameter調整を行う。

## M4.2.2設計修正 Commit 2 速度・加速度制限型pre-walk姿勢生成 実装記録

### 1. 実装範囲

M4.2.2設計修正のCommit 2として、Commit 1で追加した `startWbmsWalkingPreparation()` 起点の歩行準備状態機械に、固定秒returnではない速度・加速度limit型のpre-walk姿勢生成を接続した。

主な実装範囲は次の通りである。

- CHEST roll/pitch/yawを、歩行可能基準姿勢 `wbmsStartChestRInFootMid` へ速度limitと角加速度limitで戻す。
- COM X/Yを、snapshot時に保存したnominal walking robot COM X/Yへ速度limitと加速度limitで戻す。
- COM ZはWBMS開始時高さへ戻さず、歩行準備開始時のfootMid基準robot COM高さ `heldRobotComHeightInFootMid` を保持する。
- DECELERATING、RETURNING、SETTLE、READY、FAILEDの状態遷移を、固定return時間ではなく速度、誤差、安全候補、力学値、settle連続成立で進める。
- READY後に投影安全候補が崩れた場合は `FAILED/UNSAFE` へ落とし、古いREADY状態で歩行APIを受け付けない。
- READY判定に、pre-walk return target velocity、projector candidate safe、final IK最大関節変化、`genCog`、`refdz`、`l`、`omega`、`refZmpTraj` 総時間を含める。
- `wbms_walking_preparation_*_velocity_limit` / `*_acceleration_limit` をIDL parameterとして追加し、set/getへ接続する。
- debug出力は既存index 0-45を維持し、末尾に追加診断値を増やした。

### 2. 対象外

以下はCommit 2の対象外として実装していない。

- 歩行中COM Z速度操作。
- task scaling。
- dependency solver変更。
- 新規テストコード。
- simulatorでのparameter tuning。
- `WbmsWalkingCommandDelay` のファイル名変更。
- 腕EE commandの新しい制御経路追加。
- READY成立時の自動歩行開始。
- READY前の歩行API pending保存。

### 3. 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `auto_stabilizer/idl/AutoStabilizerService.idl` | pre-walk専用のCHEST角速度/角加速度limit、COM速度/加速度limit parameterを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h` | pre-walk専用limit、RETURN target CHEST/COM、RETURN target velocity状態を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp` | snapshot時にRETURN targetとRETURN target velocityを初期化 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h` | 速度・加速度limit型target更新helperを追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | RETURN target生成、READY条件、COM高さ保持、READY中unsafe降格、固定秒return無効化を実装 |
| `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp` | parameter set/get、debug出力追加を実装 |

### 4. 追加service / API gate / state machine

service APIと歩行API gateはCommit 1で追加済みであり、Commit 2ではその上に状態機械本体を接続した。状態の対応は次の通りである。

| 仕様上のphase | 内部phase | 実装内容 |
|---|---|---|
| IDLE | `INACTIVE` | 歩行準備なし |
| SNAPSHOT | `REQUESTED` | 最初の制御周期でCHEST、COM、nominal COM、held COM heightをsnapshot |
| DECELERATE_COMMAND | `DECELERATING` | raw commandを使わず、applied torso/COM velocityを加速度limitでゼロへ落とす |
| RETURN_TO_WALKABLE_POSTURE | `RETURNING` | CHESTとCOM targetを速度・加速度limitで歩行可能姿勢へ戻す |
| SETTLE | `HANDOFF` | READY条件の連続成立時間を確認する。固定handoff秒では進めない |
| READY | `READY` | 歩行APIを許可する。ただし投影失敗時はFAILEDへ落とす |
| WALKING | `WALKING_HOLD` | READY後に歩行APIが成功した状態。held COM heightを維持する |
| FAILED | `FAILED` | timeout、snapshot失敗、非finite、unsafeなど |
| CANCELLED | `FAILED` + `FAILURE_CANCELLED` | cancel serviceで診断上cancelledとして返す |

### 5. `goVelocity(0,0,0)` の既存仕様維持

Commit 2では歩行API本体の既存挙動は変更していない。

- WBMS外では `goVelocity(0,0,0)` は従来どおり既存goVelocity処理へ進む。
- WBMS中でもREADY後は既存goVelocity処理へ進み、`isGoVelocityMode=true` へ到達し得る。
- WBMS中未READYだけ、歩行API gateで `false` を返す。

これにより、`goVelocity(0,0,0)` を「その場足踏み開始」として使う既存仕様は、WBMS外およびREADY後で維持される。

### 6. WBMS未READY時に歩行APIをrejectする仕様

Commit 1のgateを維持し、Commit 2でREADY条件を実装した。

- `goPos`
- `goVelocity`
- `setFootSteps`
- `setFootStepsWithParam`

上記はいずれも、WBMS activeかつ歩行準備READYでない場合、footstep生成、goVelocity mode開始、pending保存より前にrejectする。Commit 2ではREADY後に投影がunsafeになった場合もFAILEDへ落とすため、古いREADY状態のまま歩行APIを受け付け続ける経路を残していない。

### 7. 固定秒returnの廃止または無効化

`wbms_walking_preparation_return_time` と `wbms_walking_preparation_handoff_time` はIDL互換のためset/getに残しているが、pre-walk姿勢生成の主制御には使っていない。

Commit 2後のRETURN進行は次で決まる。

- target CHEST error。
- target COM XY error。
- return target velocity。
- applied velocity。
- projector candidate safe。
- final IK後の最大関節変化。
- COM/ZMP/倒立振子関連値のfinite/positive条件。
- settle timeの連続成立。

傾きやCOM差分が小さい場合は短時間でREADY条件へ近づき、大きい場合は速度・加速度limitに従って長くかかる。

### 8. 速度・加速度limit型pre-walk姿勢生成

RETURNINGでは、`wbmsWalkingPreparationTargetChestRInFootMid` と `wbmsWalkingPreparationTargetRobotComInFootMid` を毎周期更新する。

CHEST:

- target CHEST姿勢と歩行可能基準姿勢のRPY誤差を計算する。
- 誤差と加速度limitから停止可能な角速度 `sqrt(2*a*abs(error))` を軸ごとに計算する。
- 角速度limitでclampする。
- 角加速度limitで前周期target velocityからの変化を制限する。
- 制限後の角速度を積分してtarget CHEST姿勢を更新する。

COM:

- target robot COMとnominal walking robot COMの差分をfootMid基準で計算する。
- X/Yはnominal walking COMへ戻す。
- Zは `heldRobotComHeightInFootMid` に固定する。
- 残距離と加速度limitから停止可能な速度を計算し、速度limitと加速度limitを通した速度を積分する。

review指摘を受け、終端で `nextVelocity = step / dt` のように速度状態を直接上書きする処理は入れていない。終端でも速度状態は加速度limitを通して更新される。

### 9. held COM heightの定義

`heldRobotComHeightInFootMid` は、`startWbmsWalkingPreparation()` 後に `REQUESTED` を処理する最初の制御周期でsnapshotする。

定義:

```text
heldRobotComHeightInFootMid
  = (footMidCoords.inverse() * currentRobotCom).z
```

`currentRobotCom` の優先順位:

1. finiteかつvalidな `wbmsProjectedRobotCom`
2. `genRobot->centerOfMass()`
3. どちらも使えない場合はsnapshot failureとしてFAILED

この値はWBMS開始時COM高さではなく、歩行準備開始時のfootMid基準robot COM高さである。歩行準備中、READY後、歩行開始後のheight holdの基準として使う。

### 10. COM X/YとCOM Zの扱い

COM X/Y:

- `wbmsNominalGenCogBeforeWbmsIntegration + sbpOffset` をfootMid基準へ変換したnominal walking robot COM X/Yへ戻す。
- 支持多角形の縮小hullでtarget X/Yをclipし、ZMP安全性を壊さない方向へ制限する。

COM Z:

- `wbmsStartComInFootMid.z` へ戻さない。
- `heldRobotComHeightInFootMid` を保持する。
- `applyWalkingComHeightHoldToReference()` で `refdz`、`l.z`、`omega` を同一周期で更新する。
- `applyWalkingComHeightHoldToGenCog()` で `genCog.z`、`genCogVel.z`、`genCogAcc.z` を速度・加速度limit付きで更新する。

このため、COM X/Y復帰とCOM Z保持は明示的に分離している。

### 11. 腕操作維持

Commit 2でも腕EE commandと `wbmsMode` はclearしない。

- 歩行準備開始で腕EE commandを消さない。
- cancelで腕EE commandを消さない。
- 未READY rejectで腕EE commandを消さない。
- READY後/WALKING_HOLDでも腕操作経路を維持する。
- 上半身EEは既存 `FullbodyIKSolver` のCHEST相対拘束経路を維持する。

足、関節安全、自己干渉、ZMP、root姿勢、バランスは腕より優先する既存IK優先度を変更していない。

### 12. 重要判断

- `wbms_walking_preparation_return_time` は互換parameterとして残し、主制御では使わない。
- `WBMS_WALKING_PREPARATION_HANDOFF` はIDL上のSETTLE相当として残し、固定handoff時間ではなくREADY条件の連続成立確認に使う。
- READY中にprojectionがunsafeになった場合はREADYを維持せずFAILEDへ落とす。
- target終端で速度を直接ゼロへ上書きせず、停止距離に基づく目標速度を作って加速度limitで減速する。
- `WALKING_HOLD` は歩行API成功後の状態であり、READY中unsafe降格とは別に扱う。
- 500 Hz経路では固定サイズのVector/Matrix状態だけを追加し、clone/newや追加solveを増やさない。

### 13. 計画との差異

- 計画書では `WbmsWalkingCommandDelay` を `WbmsWalkingPreparationController` へ置き換える案があるが、Commit 2でも既存ファイル名を維持した。差分を小さくし、Commit 1で変更したservice/gateとの連続性を優先した。
- `HANDOFF` という内部名は残しているが、意味は固定秒handoffではなくSETTLE phaseである。
- pre-walk専用limit parameterをIDLへ追加した。通常WBMS操作limitを流用すると歩行準備だけの安全速度調整がしにくいためである。
- RETURN targetの終端処理は、review対応により単純なovershoot clampではなく停止距離ベースの減速に変更した。

### 14. build・静的確認

| 確認 | 結果 | 備考 |
|---|---|---|
| `catkin build auto_stabilizer --no-deps --force-cmake` | PASS | IDL変更あり。OpenRTM helper由来のYAML warningのみ |
| `git diff --check` | PASS | whitespace指摘なし |
| `rg -n "WbmsWalkingPreparation|startWbmsWalkingPreparation|heldRobotComHeight|walking_preparation" auto_stabilizer` | PASS | service、IDL、state、COM height、parameter接続を確認 |
| `rg -n "goVelocity|goPos|setFootSteps|setFootStepsWithParam" auto_stabilizer/rtc/AutoStabilizer` | PASS | 歩行API入口とgate位置を確認 |
| `git diff --stat` | PASS | Commit 2対象6ファイルの差分を確認 |

### 15. review指摘と処置

| 順序 | 指摘要約 | 分類 | 処置 |
|---|---|---|---|
| 1 | READY中のprojection失敗でもREADYを維持し、直後の歩行APIが受理され得る | 修正対象 | `solveProjection()` 失敗時のFAILED対象に `READY` を追加し、`FAILURE_UNSAFE` へ落とす |
| 2 | RETURN終端で `nextVelocity = step / dt` により加速度limitを破る | 修正対象 | 終端の速度直接上書きを削除し、停止距離 `sqrt(2*a*abs(error))` に基づく目標速度を加速度limitで更新 |
| 3 | 最終reviewで重点項目に明確な不具合なし | 対応不要 | 追加修正なし。前2件の修正後の差分で、READY前reject、自動release抑止、COM Z保持、ZMP軌道時間、IDL set/get整合性に破綻なし |

review全文や生ログは貼らず、判断と処置のみ記録する。

### 16. acceptanceごとの結果

| acceptance | 結果 | 備考 |
|---|---|---|
| build成功 | PASS | `catkin build auto_stabilizer --no-deps --force-cmake` 成功 |
| fixed return-timeではなく速度・加速度limitで戻る | PASS | return/handoff時間を主制御に使わず、target velocityをlimitで更新 |
| 傾きが小さいほど早くREADY、大きいほど時間がかかる | コード上PASS / シミュレータ未確認 | 誤差ベースのtarget velocityで実装。実時間挙動は未確認 |
| READYは状態条件とsettleで決まる | PASS | 速度、誤差、safe candidate、joint delta、力学値、settle timeで判定 |
| COM ZはstartWbmsWalkingPreparation時高さを保持 | PASS | `heldRobotComHeightInFootMid` を基準にtarget/refdz/genCogへ接続 |
| COM X/Yはnominal walking COMへ戻る | PASS | snapshotしたnominal robot COM X/YをRETURN目標に使用 |
| `genCog.z`、`refdz`、`l.z`、`omega`が整合 | コード上PASS / シミュレータ未確認 | 同一周期でheight holdへ接続。ログでの連続性確認は未実施 |
| `refZmpTraj`総時間が正 | PASS | READY条件でtotal time > 0を確認。既存segment時間は保持 |
| READY前に歩行APIが実行されない | PASS | gateはfootstep生成/goVelocity mode開始前 |
| READY後に既存歩行APIが使える | コード上PASS / シミュレータ未確認 | gateはREADY/WALKING_HOLDで通過。runtime確認は未実施 |
| 腕操作維持 | コード上PASS / シミュレータ未確認 | command clearなし、CHEST相対拘束経路維持。実挙動は未確認 |
| hidden goalなし | PASS | pending保存なし。RETURN targetは状態機械内の現在targetのみ |
| 既存debug互換維持 | PASS | 既存index 0-45は維持し、末尾を96要素へ拡張 |
| 500 Hz経路で追加solve/clone/newなし | PASS | 固定サイズ状態と3軸計算のみ追加 |
| `goVelocity(0,0,0)` 既存仕様維持 | PASS | WBMS外/READY後の既存goVelocity本体は維持 |
| WBMS外の歩行API挙動維持 | PASS | gateはWBMS active時のみ |
| READY後の自動歩行開始なし | PASS | pending releaseなし |
| 歩行中COM Z速度操作を実装しない | PASS | optional機能は未実装 |
| simulatorで跳ね上がり改善 | シミュレータ未確認 | Commit 2後の実機/シミュレータ確認が必要 |
| READY後unsafe時に歩行APIが受理されない | コード上PASS / シミュレータ未確認 | READY中projection失敗でFAILEDへ落とす |
| RETURN終端で加速度limitを破らない | コード上PASS / シミュレータ未確認 | 速度直接上書きを削除。実ログでの速度連続性は未確認 |

### 17. 未解決事項

- simulatorで、RETURN target velocityが終端まで滑らかに収束することは未確認。
- simulatorで、READY後に `goVelocity(0,0,0)` が従来どおり足踏み開始することは未確認。
- simulatorで、READY後にprojection unsafeへ崩れた場合にFAILEDへ落ち、歩行APIがrejectされることは未確認。
- simulatorで、`genCog.z`、`refdz`、`l.z`、`omega`、`refZmpTraj` 総時間の連続性は未確認。
- 腕操作を継続しながら歩行準備、READY、歩行開始へ進めた場合の実挙動は未確認。
- pre-walk専用limit parameterの初期値は保守的なコード初期値であり、robot別の調整は未実施。
- `wbms_walking_preparation_return_time` と `handoff_time` は互換目的で残るが、主制御には使わないため、外部設定側の説明更新が必要になる可能性がある。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md`、`auto_stabilizer/log/` は未追跡として存在する。不要に削除しない。

### 18. 次のcommitまたはsimulatorへの引き継ぎ

- simulatorで `startWbmsWalkingPreparation()` から READY までのphase、return target velocity、CHEST error、COM XY/Z error、root error、safe candidate、final IK max joint deltaを確認する。
- 小さい姿勢差と大きい姿勢差でREADY到達時間が変わることを確認する。
- COM Zをずらしてから歩行準備を開始し、`heldRobotComHeightInFootMid`、`genCog.z`、`refdz`、`l.z`、`omega` が連続であることを確認する。
- READY前 `goVelocity(0,0,0)` reject、READY後 `goVelocity(0,0,0)` accept、READYだけでは自動歩行開始しないことを確認する。
- READY後にprojection unsafeを意図的に作れる条件で、READY維持ではなくFAILEDへ落ちることを確認する。
- 腕EE操作を継続したまま、歩行準備、READY、歩行開始を行い、腕commandとCHEST相対拘束が維持されることを確認する。
- 必要に応じてpre-walk専用limit parameterをrobot/simulatorに合わせて調整する。

## M4.2.2設計修正後 simulatorログ解析記録 2026-07-06 16:42

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061642*
```

試験条件:

- 事前に体幹を前傾させた姿勢から開始。
- ログ開始から約1秒後に `startWbmsWalkingPreparation()` を呼ぶ。
- 5秒後から `goVelocity(0.0, 0.0, 0.0)` を `true` になるまで0.1秒周期で呼ぶ。
- WBMS parameterは初期値、腕姿勢指令なし。

総合判定はFAILである。

READY前の歩行API gate自体は期待通り動作した。`goVelocity(0.0, 0.0, 0.0)` は相対時刻6.009-6.714sに8回rejectされ、READY後の6.816sに1回acceptされた。FAILED flagは全周期0、failure codeも0であった。

一方で、`startWbmsWalkingPreparation()` だけでREADY前に footstep が生成されている。相対時刻1.005sにRETURNINGへ入り、1.377sで `footstepNodesList.size()` が1から11へ増えた。その後、1.533sにLLEG supportがfalse、2.473sにRLEG supportがfalseへ変化しており、歩行API accepted前に足踏み相へ入っている。この挙動は「READYになるだけでは自動歩行開始しない」「`startWbmsWalkingPreparation()` は歩行準備状態だけを開始する」というM4.2.2設計修正仕様に反する。

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000 | INACTIVE、`footstepNodesList.size()=1` |
| 1.005 | `startWbmsWalkingPreparation()` event、RETURNING開始、held COM height=0.921282m |
| 1.377 | `footstepNodesList.size()` が1から11へ増加 |
| 1.533 | LLEG support=false |
| 2.473 | RLEG support=false |
| 5.407 | `footstepNodesList.size()` が一旦1へ戻る |
| 6.009-6.714 | READY前 `goVelocity(0,0,0)` reject 8回 |
| 6.546 | HANDOFF |
| 6.731 | READY flag true |
| 6.816 | `goVelocity(0,0,0)` accepted、WALKING_HOLD、`footstepNodesList.size()` が1から7へ増加 |
| 7.785 | READY後歩行でRLEG support=false |

最大値・範囲:

| 項目 | 値 |
|---|---|
| CHEST error | max 0.265233rad、READY時0.028246rad |
| COM XY error | max 0.000026m |
| COM Z hold error | max 0.000312m |
| held COM height | 0.921282m |
| current COM height | 0.920970-0.921313m |
| return velocity max | 0.100010m/sまたはrad/s |
| final IK後CHEST実現角速度 | max 1.11207rad/s at 1.348s |
| final IK後COM実現速度 | max 0.153142m/s at 1.348s |
| final IK max joint delta | max 0.010294rad |
| `ast_q` 一周期最大差分 | max 0.010293rad、joint 2、t=1.298s |
| `refdz` / `l.z` | 0.921028-0.921298m |
| `omega` | 3.262573-3.263051 |
| `refZmpTraj` 総時間 | 0.598-1.598s |
| target ZMP一周期最大jump | 0.155685m at 5.259s |
| actual COM一周期最大jump | 0.014736m |
| act-gen DCM error | max 0.073947m |
| projector time | mean 0.352ms、p99 0.769ms、max 2.145ms |
| final IK time | mean 0.779ms、p99 1.394ms、max 1.973ms |
| onExecute time | mean 1.442ms、p95 2.102ms、p99 2.438ms、max 6.257ms、2ms超過225周期 |

COM Z保持は概ね良好であり、`heldRobotComHeightInFootMid`、`refdz`、`l.z`、`omega` は大きく破綻していない。actual DCMも発散は見えない。ただし、READY前 footstep生成によりtarget ZMPが周期間で最大15.6cm飛び、support phaseも変化しているため、ZMP/footstep時系列としては異常ありと判定する。

前傾角度が狭い件は、初期値 `wbmsTorsoRpyUpperLimit.y = 0.25rad` が主要原因候補である。ログ上のCHEST pitch offset最大は0.265rad程度であり、約15deg相当で頭打ちになっている。これは実行時parameter調整で扱うため、この記録時点ではコード修正対象にしない。

最小原因候補:

- `AutoStabilizer::execAutoStabilizer()` の通常AutoBalancer経路で、歩行準備中も `FootStepGenerator::procFootStepNodesList()` と `FootStepGenerator::calcFootSteps()` が通常通り実行されていた。
- READY前歩行API gateはfootstep生成API入口には効くが、CP/emergency stepやmodify footstepの自動生成経路には効いていなかった。
- CHEST return target速度は0.1rad/s程度に制限されているが、final IK後の実現CHEST角速度が1.11rad/sまで出ており、別途実現速度側の監視または制限が必要である。
- 500Hz周期に対して `onExecute` はp95で2msを超えており、実時間余裕は未達である。

このログを受けた修正方針:

- `REQUESTED`、`DECELERATING`、`RETURNING`、`HANDOFF`、`READY` 中は通常の footstep 時系列更新と自動footstep生成を止め、`footstepNodesList` を静止状態に保持する。
- `READY` 後に明示的な walking API が受理されて `WALKING_HOLD` へ入った場合だけ、従来どおり `goVelocity(0,0,0)` の足踏み開始を許可する。
- 前傾角上限初期値は変更しない。大きな前傾試験では実行時に `wbmsTorsoRpyUpperLimit.y` を調整する。

## M4.2.2設計修正後 simulatorログ解析記録 2026-07-06 17:00

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061700*
```

試験条件:

- 事前に体幹を前傾させた姿勢から開始。
- ログ開始から約1秒後に `startWbmsWalkingPreparation()` を呼ぶ。
- 5秒後から `goVelocity(0.0, 0.0, 0.0)` を `true` になるまで0.1秒周期で呼ぶ。
- 前回問題になった前傾角度制限は、実行時parameterで `wbmsTorsoRpyUpperLimit.y` を広げて確認。
- 腕姿勢指令なし。

総合判定はFAILである。ただし前回 `test_start_walking202607061642*` で発生した、READY前の想定外footstep生成は解消している。

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000 | INACTIVE、`footstepNodesList.size()=1` |
| 1.004 | `startWbmsWalkingPreparation()` event、RETURNING開始 |
| 1.3-1.6 | CHEST/root/脚関節の急変、右腕を含む腕関節の大きな変化が集中 |
| 6.008-7.123 | READY前 `goVelocity(0,0,0)` reject 12回 |
| 7.012 | HANDOFF |
| 7.133 | READY flag true |
| 7.218 | `goVelocity(0,0,0)` accepted、WALKING_HOLD、`footstepNodesList.size()` が1から7へ増加 |
| 8.151 | READY後歩行でRLEG support=false |

footstep関連:

- `startWbmsWalkingPreparation()` 後からREADYまでは `footstepNodesList.size()` は1のまま。
- support phaseもREADY後歩行開始まで両脚支持のまま。
- READY後の `goVelocity(0,0,0)` acceptedで `footstepNodesList.size()` が1から7、直後に8へ増加し、既存の足踏み開始挙動に戻った。

最大値・範囲:

| 項目 | 値 |
|---|---|
| CHEST pitch offset | 0.468rad付近から1.56sに0.176rad付近へ急減 |
| CHEST error | max 0.467997rad、READY時0.014630rad |
| COM XY error | max 0.000015m |
| COM Z hold error | max 0.000649m |
| held COM height | 0.921258m |
| current COM height | 0.920609-0.921415m |
| final IK後CHEST実現角速度 | max 3.427rad/s at 1.560s |
| final IK後COM実現速度 | max 0.103318m/s |
| final IK max joint delta | max 0.011342rad |
| `el_q` 一周期最大差分 | max 0.011343rad、RLEG_JOINT2、t=1.334s |
| `RobotHardware0_q` 一周期最大差分 | max 0.017471rad、LLEG_JOINT2、t=1.321s |
| target ZMP一周期最大jump | 0.078117m |
| actual DCM一周期最大jump | 0.029595m |
| projector time | mean 0.348ms、p99 0.691ms、max 0.932ms |
| final IK time | mean 0.775ms、p99 1.334ms、max 2.114ms |
| onExecute time | mean 1.417ms、p95 2.048ms、p99 2.387ms、max 3.416ms、2ms超過216周期 |

腕関節の観測:

- `el_q` と `ast_q` はほぼ一致しており、RTC出力段階で急な関節角指令になっている。
- 右腕は1.7-3.0sで特に振動的に変化した。
- 右腕上腕部を含む関節変化は、腕指令なしでも発生した。

代表値:

| 関節 | 1.7-3.0sの範囲 |
|---|---:|
| RARM_JOINT0 | span 0.145rad |
| RARM_JOINT1 | span 0.160rad |
| RARM_JOINT3 | span 0.109rad |
| RARM_JOINT6 | span 0.124rad |
| LARM_JOINT0 | span 0.146rad |
| LARM_JOINT1 | span 0.160rad |
| LARM_JOINT3 | span 0.110rad |
| LARM_JOINT6 | span 0.124rad |

原因候補:

- `wbmsWalkingPreparationReturnTorsoAngularVelocity` は0.1rad/s程度に制限されているが、最終IK後の実現CHEST pitch速度は3.43rad/sまで出ている。したがって、pre-walk return targetの速度制限だけでは、実際の `el_q` とCHEST実現速度を安全に制限できていない。
- RETURNING開始後、`wbmsOperationModeValue=1.0` のまま `wbmsWalkingStabilityModeValue` が0から1へrampする。これにより、final IK内のroot姿勢拘束weightが立ち上がり、CHEST/COM/root/腕EE拘束が同時に強く効く。
- 腕指令なしでもWBMS active中は上半身EE拘束がCHEST相対で維持される。体幹/rootが急に戻ると、手先相対拘束を満たすために腕関節が補償し、右腕上腕部の振動として現れた可能性が高い。
- 1.3-1.6sでは脚hip pitch、CHEST pitch、root pitchが同時に大きく変化しており、急激に直立へ近づいた後、`wbmsWalkingStabilityModeValue` が十分立ち上がった状態で0.1rad/s級のゆっくりした戻りに切り替わったため、目視上「急動作の後に低速復帰」の2段階に見えたと考えられる。

このログから設計文書へ移した暫定修正候補:

- READY前のroot姿勢拘束立ち上げを、CHEST/COM returnと同じ速度・加速度制限の管理下に入れる。
- `wbmsWalkingStabilityModeValue` をRETURNING開始直後から上げない、またはroot errorが安全速度で減るように専用rate limitを入れる。
- READY条件だけでなくRETURNING中も、final IK後CHEST速度、final IK後COM速度、`el_q` 相当の一周期関節差分を監視し、閾値超過時はFAILED/UNSAFEへ落とす。
- 腕EE拘束は歩行準備中に急な体幹復帰を増幅し得るため、歩行準備中はweight ramp、maxError制限、または保持姿勢の再ラッチ方針を検討する。
- `wbmsWalkingPreparationReturnTorsoAngularVelocity` だけを安全判定に使わず、最終IK後に実際に出るCHEST速度と関節差分を安全判定へ含める。

この修正候補は、事前に計画書で区切ったタスクではなく、シミュレータ検証中に発見された追加修正である。正式な修正方針、実装候補、acceptance criteriaは `WBMSWalkingPreparationDesignRevisionPlan.md` の「11. 追加検証で判明した修正項目」へ移した。次スレッドでは、詳細ログ値は本節、実装方針は同設計文書11章を参照する。

## M4.2.2 11.3修正後 simulatorログ解析記録 2026-07-06 17:47

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061747*
```

試験条件:

- 事前に体幹を前傾させた姿勢から開始。
- ログ開始から約1秒後に `startWbmsWalkingPreparation()` を呼ぶ。
- 5秒後から `goVelocity(0.0, 0.0, 0.0)` を `true` になるまで0.1秒周期で呼ぶ。
- 腕姿勢指令なし。
- 11.3対策として、root return target追加、root return targetの速度・加速度limit、RETURNING中の `wbmsWalkingStabilityModeValue` 同期、RETURNING中final IK後安全監視を実装済み。

総合判定はFAILである。ただし、急激な直立復帰と腕振動は大きく改善した。

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000-1.005 | INACTIVE |
| 1.007 | `startWbmsWalkingPreparation()` event、RETURNING開始 |
| 6.011-11.146 | READY前 `goVelocity(0,0,0)` reject 52回 |
| 8.162 | `FAILED/TIMEOUT`、failure code 2 |

phase遷移:

```text
0.000-1.005s   INACTIVE
1.007-8.160s   RETURNING
8.162s以降     FAILED
```

最大値・範囲:

| 項目 | 値 |
|---|---:|
| 初期 CHEST error | 0.465123rad |
| 初期 root error | 0.212094rad |
| timeout直前 CHEST error | 0.006991rad |
| timeout直前 root error | 0.000078rad |
| timeout直前 returnNorm | 0.000755 |
| timeout直前 returnAlpha | 0.999177 |
| final IK後CHEST角速度 max | 0.100028rad/s |
| final IK後COM速度 max | 0.000866m/s |
| final IK max joint delta max | 0.002020rad |
| `RobotHardware0_q` 一周期最大差分 | 0.001062rad |
| `footstepNodesList.size()` | 全期間1 |
| `goVelocity` accepted | 0回 |

腕関節の観測:

- `el_q` と `ast_q` は完全一致。
- 前回0.1rad級だった腕関節spanは、1.7-3.0sでは代表的な腕関節で `3e-5` から `6e-5rad` 程度。
- 腕指令なし条件での大きな腕振動は、このログでは再現していない。

推定:

- 見えているREADY条件の多くはtimeout前に満たしているが、当時のdebugにはroot return速度が出ていなかった。
- `wbmsWalkingPreparationReturnRootAngularVelocity` がREADY条件を阻害している可能性が高いと推定した。
- timeoutを単純に延長するだけで解決するかは未確定であり、READY条件booleanとroot return速度を追加debugする必要があると判断した。

実施したdebug追加:

- `wbmsDebugOut` 末尾に、root return速度、COM/CHEST/root return速度norm、root target RPY、root target error、final IK後速度norm、READY条件boolean、RETURNING中final IK安全判定、timeout残り時間、速度epsを追加した。
- 既存96列は維持した。
- 実装上 `data.length(126)` に対し、意味を持つ追加列は28列で、末尾2列は未使用の0として出力される状態である。

## M4.2.2 11.3 debug追加後 simulatorログ解析記録 2026-07-06 18:14/18:16/18:17

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061814*
auto_stabilizer/log/test_start_walking202607061816*
auto_stabilizer/log/test_start_walking202607061817*
```

試験条件:

- `wbms_walking_preparation_timeout = 10.0`。
- 事前に体幹を前傾させた姿勢から開始。
- ログ開始から約1秒後に `startWbmsWalkingPreparation()` を呼ぶ。
- 5秒後から `goVelocity(0.0, 0.0, 0.0)` を0.1秒周期、最大10秒retryする。
- 3本とも同じ起動方法、同じparameter設定。
- 腕姿勢指令なし。

総合判定はFAILである。3本で挙動が分岐した。

| ログ | 判定 | 概要 |
|---|---|---|
| `061814` | FAIL | READY後に `goVelocity` accepted。ただし足踏み開始直後に股関節付近の急動作が残る |
| `061816` | PASS相当 | READY後に `goVelocity` accepted。滑らかに足踏み開始 |
| `061817` | FAIL | READYへ到達せず、`FAILED/TIMEOUT`。`goVelocity` はacceptedされない |

### ログ1: `test_start_walking202607061814*`

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000-1.000 | INACTIVE |
| 1.002-8.297 | RETURNING |
| 8.300-8.417 | HANDOFF |
| 8.419-8.425 | READY |
| 8.427 | `goVelocity(0,0,0)` accepted、WALKING_HOLD |

主要値:

| 項目 | 値 |
|---|---:|
| 初期 CHEST error max | 0.495147rad |
| root error max | 0.628610rad |
| READY時 rootError | 0.068002rad |
| READY時 `returnRootVelNorm` | 0.000855 |
| READY時 `wbmsWalkingStabilityModeValue` | 0.902 |
| accepted直後 `wbmsOperationModeValue` | 0.098 |
| accepted直後 `footstepNodesList.size()` | 1 -> 7 -> 8 |
| accepted直後 final IK後CHEST角速度 max | 4.885705rad/s |
| accepted直後 final IK後COM速度 max | 2.091680m/s |
| `el_q` 一周期最大差分 | 0.018394rad at 8.439s |

READY条件:

- READY phase中は追加debugのREADY booleanは全項目true。
- RETURNING後半では `readyReturnRootVelocity=false` が長く残り、最後にtrueへ落ちてHANDOFFへ進んだ。
- `readyRootError=true` は `rootError=0.068rad` でも成立しており、現在の `wbmsWalkingPreparationRootErrorEps=0.08rad` は足踏み開始安全条件として緩い可能性がある。

急動作の発生箇所:

- 急動作はRETURNING中ではなく、`goVelocity` accepted直後のWALKING_HOLD / footstep生成後に発生。
- accepted直後に `postureRefValid=0`、`candidateSafe=0`、`footstepNodesList.size()` が増加し、同時に `wbmsOperationModeValue=0.098` が残っていた。
- `el_q` の最大一周期差分は脚関節で、accept直後windowの上位差分はRLEG/LLEG hip pitch周辺に集中した。

### ログ2: `test_start_walking202607061816*`

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000-1.004 | INACTIVE |
| 1.006-10.137 | RETURNING |
| 10.138-10.258 | HANDOFF |
| 10.260-10.343 | READY |
| 10.348 | `goVelocity(0,0,0)` accepted、WALKING_HOLD |

主要値:

| 項目 | 値 |
|---|---:|
| 初期 CHEST error max | 0.503451rad |
| root error max | 0.037241rad |
| READY時 rootError | 0.000111rad |
| READY時 `returnRootVelNorm` | 0.000990 |
| READY時 `wbmsWalkingStabilityModeValue` | 1.000 |
| accepted直後 `wbmsOperationModeValue` | 0.000 |
| accepted直後 final IK後CHEST角速度 | 0.044rad/s |
| accept window `el_q` 一周期最大差分 | 0.000287rad |

READY条件:

- READY phase中は追加debugのREADY booleanは全項目true。
- `wbmsWalkingStabilityModeValue=1.0`、`wbmsOperationModeValue=0.0` で歩行開始しており、ログ1のような足踏み開始直後の急動作は出ていない。

### ログ3: `test_start_walking202607061817*`

主要時刻:

| 相対時刻[s] | 事象 |
|---:|---|
| 0.000-1.001 | INACTIVE |
| 1.003-12.888 | RETURNING |
| 12.890以降 | `FAILED/TIMEOUT` |
| 6.008-16.175 | `goVelocity(0,0,0)` reject 102回 |

主要値:

| 項目 | 値 |
|---|---:|
| 初期 CHEST error max | 0.572723rad |
| root error max | 0.456161rad |
| timeout直前 CHEST error | 0.000001rad |
| timeout直前 rootError | 0.000033rad |
| timeout直前 `returnRootVelNorm` | 0.004483 |
| timeout直前 `returnTorsoVelNorm` | 0.001274 |
| timeout直前 `returnAllVelNorm` | 0.004483 |
| failure code | 2 (`TIMEOUT`) |
| `goVelocity` accepted | 0回 |
| `footstepNodesList.size()` | 全期間1 |

READY条件:

- timeout直前、誤差系と安全系のREADY booleanはtrue。
- `readyReturnRootVelocity=false`、`readyReturnTorsoVelocity=false` が残り、READYへ進まなかった。
- 姿勢誤差は十分小さいため、root/torso return target速度のゼロ収束条件がREADY阻害要因である。

### 3本の比較からの推定

- ログ1とログ2の差は、READY時のroot error、`wbmsWalkingStabilityModeValue`、歩行API accepted直後の `wbmsOperationModeValue` に現れている。
- ログ1は `rootError=0.068rad`、`wbmsWalkingStabilityModeValue=0.902` でREADYとなり、accepted直後に `wbmsOperationModeValue=0.098` が残る。この状態でfootstep生成が始まり、final IK後CHEST/COM速度と脚関節差分が急増する。
- ログ2は `rootError=0.000111rad`、`wbmsWalkingStabilityModeValue=1.000`、`wbmsOperationModeValue=0.000` で歩行開始しており、急動作が出ない。
- ログ3は姿勢誤差が十分小さいにもかかわらず、root/torso return速度条件によりREADYにならない。

このログから設計文書へ移した修正候補:

- READY後およびWALKING_HOLD中は `wbmsOperationModeValue=0.0` を強制し、歩行API accepted後にWBMS操作blendを復活させない。
- READY条件からroot return target速度の厳格条件を外す、または専用閾値へ分離する。
- root READY条件を厳しくし、`rootError=0.068rad` 程度でREADYにしない。`wbmsWalkingStabilityModeValue > 0.99` をREADY条件に含める案も検討する。
- HANDOFF/READY中もroot targetと `stTargetRootPose` の同期を継続するか、root側が十分settleしてからREADY判定する。
- `wbmsDebugOut` の未使用末尾2列を整理する。

## M4.2.2 11.4 READY判定と歩行開始直後blend残留対策

11.3後ログから、RETURNING中の急激な姿勢復帰と腕振動は大きく改善した一方で、READY判定と歩行API accepted直後に次の問題が残っていた。

- `061814` では `rootError=0.068002rad`、`wbmsWalkingStabilityModeValue=0.902` でREADYになり、accepted直後に `wbmsOperationModeValue=0.098` が残った。
- `061816` では `rootError=0.000111rad`、`wbmsWalkingStabilityModeValue=1.000`、accepted直後 `wbmsOperationModeValue=0.000` で滑らかに足踏み開始した。
- `061817` では姿勢誤差が十分小さいが、return target速度条件だけが残ってREADYに到達しなかった。

今回の実装:

- `WALKING_HOLD` 中も `wbmsOperationModeValue=0.0` を強制し、歩行API accepted後にWBMS操作blendを復活させないようにした。
- READY条件から `wbmsWalkingPreparationReturnComVelocity`、`wbmsWalkingPreparationReturnTorsoAngularVelocity`、`wbmsWalkingPreparationReturnRootAngularVelocity` の厳格な速度ゼロ条件を外した。
- READY条件へ `wbmsWalkingStabilityModeValue >= 0.99` を追加した。
- `wbmsWalkingPreparationRootErrorEps` の既定値を `0.08rad` から `0.01rad` に変更した。
- RETURNINGからHANDOFFへ遷移する時、およびHANDOFF中にREADY条件が成立している時、root return targetを現在の `stTargetRootPose.linear()` へ同期し、root return速度をゼロにするようにした。
- `wbmsDebugOut` に `readyWalkingStability` booleanを追加し、`data.length` を実際の書き込み列数である125へ変更した。

意図した効果:

- `061814` 型の、READY時stability不足とoperation blend残留による歩行開始直後の急動作を防ぐ。
- `061817` 型の、誤差は収束しているのにreturn target速度だけでtimeoutするケースをREADYへ進める。
- READY直後またはWALKING_HOLD移行直後のroot target切替を小さくする。

未確認:

- シミュレータでの再現性。`wbms_walking_preparation_timeout=10.0`、初期前傾条件で3本以上ログを取り、READY到達、accepted直後 `wbmsOperationModeValue=0.0`、final IK後速度、`el_q` 一周期最大差分を確認する必要がある。

ビルド結果:

```sh
catkin build auto_stabilizer --no-deps
```

成功。warningsなし。

## M4.2.2 11.4修正後 simulatorログ解析記録 2026-07-06 18:48/18:51/18:53

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061848*
auto_stabilizer/log/test_start_walking202607061851*
auto_stabilizer/log/test_start_walking202607061853*
```

試験条件:

- `wbms_walking_preparation_timeout = 10.0`。
- 事前に体幹を前傾させた姿勢から開始。
- ログ開始から約1秒後に `startWbmsWalkingPreparation()` を呼ぶ。
- 5秒後から `goVelocity(0.0, 0.0, 0.0)` を0.1秒周期、最大10秒retryする。
- 3本とも同じ起動方法、同じparameter設定。
- 腕姿勢指令なし。

総合判定はPASS相当である。ただし、ログ1/2にはoptional改善候補が残る。

| ログ | 判定 | 概要 |
|---|---|---|
| `061848` | PASS相当、optional課題あり | READY後に `goVelocity` accepted。operation blend残留なし。root pitchは -0.303rad付近で足踏み開始し、その後直立側へ戻る |
| `061851` | PASS相当、optional課題あり | READY後に `goVelocity` accepted。operation blend残留なし。root pitchは -0.367rad付近で足踏み開始し、その後直立側へ戻る |
| `061853` | PASS | READY後に `goVelocity` accepted。root pitchは -0.052rad付近で足踏み開始し、ほぼ直立に近い |

### 主要イベント時刻

| ログ | RETURNING開始[s] | HANDOFF[s] | READY[s] | `goVelocity` accepted[s] | timeout残り[s] |
|---|---:|---:|---:|---:|---:|
| `061848` | 1.006 | 7.228 | 7.350 | 7.416 | 4.690 |
| `061851` | 1.007 | 9.546 | 9.670 | 9.740 | 2.706 |
| `061853` | 1.005 | 11.022 | 11.152 | 11.245 | 1.650 |

`timeout残り` は内部 `wbmsWalkingPreparationTimeout - wbmsWalkingPreparationElapsedTime` であり、3本ともtimeout境界ではない。

### READY/accept時の主要値

| ログ | READY時root error[rad] | accept時root pitch[rad] | accept時`stTargetRootPose` pitch[rad] | accept時stability | accept時operation |
|---|---:|---:|---:|---:|---:|
| `061848` | 0.000292 | -0.303 | -0.303 | 0.990750 | 0.0 |
| `061851` | 0.000354 | -0.367 | -0.367 | 0.990061 | 0.0 |
| `061853` | 0.000020 | -0.052 | -0.052 | 0.990098 | 0.0 |

11.4で意図した以下は満たした。

- `goVelocity` accepted直後に `wbmsOperationModeValue=0.0`。
- READY時に `wbmsWalkingStabilityModeValue >= 0.99`。
- READY時root errorは既定 `wbmsWalkingPreparationRootErrorEps=0.01rad` 以下。
- return target速度だけがREADYを阻害してtimeoutする挙動は再現しない。
- READY前footstep抑制、READY前walking API reject、READY後walking API acceptは退行していない。

### accept直後のfinal IK診断

| ログ | final IK COM速度最大[m/s] | final IK CHEST角速度最大[rad/s] | `el_q`一周期最大差分[rad] |
|---|---:|---:|---:|
| `061848` | 0.520 | 2.103 | 0.0137 |
| `061851` | 0.310 | 3.558 | 0.0159 |
| `061853` | 0.103 | 2.019 | 0.0041 |

一周期最大関節差分は既存閾値 `wbmsWalkingPreparationMaxJointDeltaEps=0.08rad` より十分小さい。`el_q` と `ast_q` はaccept直後windowで一致しており、AutoStabilizer出力段階での不連続な大差分は確認されない。

関節差分の傾向:

- RETURNING中の主な大差分は脚pitch系とtorso pitch系に出る。
- accept後1秒では、ログ1/2で脚・torsoが通常歩行側のroot targetへ追従しながら動く。
- 腕指令なし条件で、11.3以前に問題となった0.1rad級腕振動は確認されない。腕関節のaccept後1秒span最大はログ1で0.0008rad、ログ2で0.0027rad、ログ3で0.0005rad程度である。

### rootが直立に戻りきらない挙動の原因

ログ1/2では、歩行開始時にroot linkが直立付近へ戻りきらず、足踏みしながら直立へ戻るように見える。

原因は、rootが `stTargetRootPose` に追従できていないことではない。accept時のroot pitchと `stTargetRootPose` pitchはほぼ一致しており、root errorも十分小さい。

実際の原因は、READY条件が「root姿勢が `stTargetRootPose` に一致したか」を見ており、「root linkがworld/footMid基準で直立したか」は見ていないことである。`stTargetRootPose` はStabilizer由来であり、ログ1/2では `stTargetRootPose` 自体が -0.30rad〜-0.37rad程度前傾側に残ったままREADY条件を満たしている。

accept後は `wbmsOperationModeValue=0.0` のまま、通常歩行側のroot targetに従ってroot pitchが0.1rad/s級で直立側へ戻る。

| ログ | accept時root pitch[rad] | accept+1.0s root pitch[rad] | 変化量[rad/s概算] |
|---|---:|---:|---:|
| `061848` | -0.303 | -0.205 | 0.098 |
| `061851` | -0.367 | -0.247 | 0.120 |
| `061853` | -0.052 | -0.046 | 0.007 |

### optionalに追加可能な仕様

「歩行開始前にroot linkを直立付近まで戻す」ことは、現時点では必須仕様ではない。11.4の必須目的である、READY到達、READY後API accept、operation blend残留防止、READY前footstep抑制、腕大振動抑制は今回ログで概ね満たしている。

ただし、操作感・見た目・運用上の要件として「足踏み開始前にroot linkを絶対姿勢として直立付近にする」ことを求める場合は、optional仕様として追加できる。

optional仕様案:

- READY条件へ absolute root upright 条件を追加する。
- 判定対象は既存 `wbmsWalkingPreparationRootErrorEps` ではなく、world/footMid基準のroot roll/pitch絶対値、または `stTargetRootPose` roll/pitch絶対値とする。
- 新規parameter例として `wbms_walking_preparation_root_upright_error_eps` を追加する案がある。
- 初期候補値は `0.05rad` 程度。ただし実ログと操作感で調整し、現時点では確定しない。
- このoptional仕様を採用するとREADY到達が遅くなる可能性があるため、`wbms_walking_preparation_timeout` とroot姿勢復帰速度の再評価が必要である。

現時点の方針:

- 破綻対策としての追加修正は不要。
- optional仕様を採用するかどうかは、「足踏み開始前にroot絶対姿勢がどの程度直立している必要があるか」という運用要件として別途判断する。

## M5.1 1-iteration運用方針とIK parameter整理 実装記録

M5: 500 Hz計算量削減の最初の作業として、`WBMSComputationReductionImplementationPlan.md` のM5.1を実施した。

M5.1は挙動変更ではなく、後続のM5.2以降で `checkFinalState=false` やIK軽量化を安全に進めるための前提整理である。現行コードではprojector/final IKともに `maxIteration=1` で運用しており、`precision=0.0` は「反復をmax loopまで強制する」設定ではない。この誤解を招くコメントを削除し、1 iteration固定運用、戻り値、診断値の扱いをコードコメントとして明記した。

### 実装した範囲

- `FullbodyIKSolver.cpp` の各 `precision() = 0.0` に付いていた「強制的にIKをmax loopまで回す」という現状と合わないコメントを削除した。
- final IKの `IKParam param` 設定箇所に、WBMS final IKは500 Hz運用のため `maxIteration=1` 固定で使うことを明記した。
- final IKでは、`precision=0.0` は `maxIteration=1` 条件では反復数を増やさず、`solveIKLoop()` の最終満足判定だけを厳しくすることを明記した。
- final IKでは、現状の制御判断が `solveIKLoop()` の戻り値に依存していないことを明記した。
- `WbmsPostureControl.cpp` のprojector `projectionIKParam_.maxIteration = 1` 設定箇所に、projectorは1周期先の安全な小ステップ候補を作る用途であり、500 Hz運用では1 iteration固定で使うことを明記した。
- projectorでは、`maxIteration>1` の収束設計は本計画の対象外で別途扱うことを明記した。
- projectorの `solveIKLoop()` 呼び出し箇所に、`allConstraintsSatisfied` は採用判定ではなく診断値であり、候補採用可否は `validateProjectionCandidate()` で判定することを明記した。

### 変更ファイル

| ファイル | 変更概要 |
|---|---|
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp` | 古い `precision=0.0` コメントを削除し、final IKの1 iteration固定運用と戻り値非依存を明記 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | projectorの1 iteration固定運用、`maxIteration>1` 対象外、`allConstraintsSatisfied` が診断値であることを明記 |

### acceptance確認

| 項目 | 結果 | 備考 |
|---|---|---|
| コメントが現行挙動と一致する | PASS | `precision=0.0` が反復数を増やすというコメントを削除 |
| `maxIteration=1` 固定運用の意図が伝わる | PASS | projector/final IKそれぞれのIKParam設定箇所へ明記 |
| projector採用判定と診断値の区別が明確 | PASS | `validateProjectionCandidate()` が採用判定、`allConstraintsSatisfied` は診断値と明記 |
| final IK戻り値の扱いが明確 | PASS | 現状の制御判断は戻り値に依存しないと明記 |
| 制御挙動を変更しない | PASS | コメントのみの変更 |
| ビルド | PASS | `catkin build auto_stabilizer --no-deps` 成功、warningsなし |

### コミット

```text
e25a5f5 Clarify WBMS one-iteration IK policy
```

### 後続M5への引き継ぎ

- M5.1は完了済み。コードの挙動変更、IDL変更、debug index変更、シミュレータログ再取得は行っていない。
- M5.2では、計画書どおり `prioritized_inverse_kinematics_solver2::IKParam::checkFinalState` を実装し、projector/final IKで `checkFinalState=false` を適用する。
- M5.2で `checkFinalState=false` を適用する際も、solve後のFK/COM更新は省かない。projectorはsolve後の姿勢とCOMを `validateProjectionCandidate()` と投影結果保存で使うためである。
- projectorは `solveIKLoop()` 戻り値に依存させず、既存どおり `validateProjectionCandidate()` による安全判定を採用条件にする。
- final IKは現状どおり `solveIKLoop()` 戻り値を制御判断に使っていないため、M5.2で戻り値の意味が「最終満足判定未実施」相当になっても制御フローを変えない。
- M5.2以降で計算時間を比較する場合、制御挙動baselineは `test_start_walking202607061848*`、`061851*`、`061853*` のPASS相当ログ、時間統計baselineは `061700` 記録を参照する。
- 作業時点で未追跡の `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/log/` はM5.1コミット対象外として残した。後続作業でも不要に削除しない。

## M5.2 `checkFinalState=false` 実装と適用 実装・評価記録

`WBMSComputationReductionImplementationPlan.md` のM5.2として、`prioritized_inverse_kinematics_solver2::IKParam::checkFinalState` を実装し、WBMS projector/final IKで `checkFinalState=false` を適用した。

### 実装した範囲

- `prioritized_inverse_kinematics_solver2::IKParam::checkFinalState` のヘッダコメントに、`false` 時は最終状態のconstraint再評価を省略し、戻り値がfalseになることを明記した。
- `prioritized_inverse_kinematics_solver2::solveIKLoop()` で、最終iteration後の速度計算、FK、COM更新は従来どおり実行し、その後 `checkFinalState=false` かつ `loop+1 >= maxIteration` の場合は `updateConstraints()` と `checkConstraintsSatisfied()` を省略してfalseを返すようにした。
- 既定値は `checkFinalState=true` のままとし、既存利用者の挙動を維持した。
- `WbmsPostureControl::init()` で projector IK の `projectionIKParam_.checkFinalState = false` を設定した。
- `FullbodyIKSolver::solveFullbodyIK()` で final IK の `param.checkFinalState = false` を設定した。
- projector側コメントを、`allConstraintsSatisfied` は `checkFinalState=false` では常にfalse相当の診断値であり、採用判定は `validateProjectionCandidate()` で行う、という現行挙動へ更新した。

### 変更ファイル

| ファイル | 変更概要 |
|---|---|
| `../ik_solvers2/prioritized_inverse_kinematics_solver2/include/prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h` | `checkFinalState=false` 時の戻り値と再評価省略を明記 |
| `../ik_solvers2/prioritized_inverse_kinematics_solver2/src/prioritized_inverse_kinematics_solver2.cpp` | 最終iteration後のFK/COM更新後に、constraint再評価を省略する分岐を追加 |
| `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp` | projector IKで `checkFinalState=false` を設定し、診断値コメントを更新 |
| `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp` | final IKで `checkFinalState=false` を設定 |

### ビルド確認

```sh
catkin build prioritized_inverse_kinematics_solver2 auto_stabilizer --no-deps
```

結果:

```text
All 2 packages succeeded.
Warnings: None.
```

### シミュレータ評価ログ

M5.2実装後、最新PASS相当ログ `061848`, `061851`, `061853` と同じ条件でシミュレータ確認を行った。

対象ログ:

```text
auto_stabilizer/log/test_start_walking202607071526.*
```

比較対象:

```text
auto_stabilizer/log/test_start_walking202607061848.*
auto_stabilizer/log/test_start_walking202607061851.*
auto_stabilizer/log/test_start_walking202607061853.*
```

ログ解析では `ast_wbmsDebug` を用いた。ログファイルの1列目は時刻であり、`wbmsDebugOut` のdata index `i` はログ上の `i+2` 列目に対応する。現行125要素の対応表は本節末尾の「現行 `wbmsDebugOut` 対応表」にまとめた。

### 歩行準備・歩行開始挙動

`test_start_walking202607071526.ast_wbmsDebug` から読み取った主要イベント:

| 項目 | 値 |
|---|---:|
| RETURNING開始 | 1.000s |
| HANDOFF | 7.269s |
| READY | 7.388s |
| `goVelocity(0,0,0)` accepted | 7.412s |
| READY前walking API reject | 14回 |
| READY前walking API accept | 0回 |
| READY前 `footstepNodesList.size()` 最大 | 1 |
| accept後 `footstepNodesList.size()` 最大 | 8 |
| FAILED遷移 | なし |

READY/accept時の主要値:

| 項目 | 値 |
|---|---:|
| READY時root error | 0.000010rad |
| READY時 `wbmsWalkingStabilityModeValue` | 0.990109 |
| READY時 `wbmsOperationModeValue` | 0.000000 |
| READY時timeout残り | 4.558s |
| READY時candidate safe | 1 |
| READY時max joint delta | 0.000015rad |
| accept直後 `wbmsOperationModeValue` | 0.000000 |
| accept直後 `wbmsWalkingStabilityModeValue` | 0.990119 |
| accept直後root pitch | -0.042310rad |
| accept直後 `stTargetRootPose` pitch | -0.042310rad |
| accept直後final IK COM速度norm | 0.000014m/s |
| accept直後final IK CHEST角速度norm | 0.028184rad/s |
| accept直後max joint delta | 0.000173rad |

accept直後から1.0s windowの最大値:

| 項目 | 値 |
|---|---:|
| final IK COM速度norm最大 | 0.103319m/s |
| final IK CHEST角速度norm最大 | 1.145691rad/s |
| max joint delta最大 | 0.002290rad |
| `footstepNodesList.size()` 最大 | 8 |
| window終端root pitch | -0.037991rad |

評価:

- READY後に `goVelocity(0,0,0)` がacceptedされる。
- READY前walking API reject、READY前footstep抑制、READY後walking API acceptは維持されている。
- accepted直後に `wbmsOperationModeValue=0.0` が維持され、operation blend残留はない。
- READY時 `wbmsWalkingStabilityModeValue >= 0.99` が維持されている。
- accept直後1.0s windowのmax joint deltaは既存閾値 `wbmsWalkingPreparationMaxJointDeltaEps=0.08rad` より十分小さい。
- accept時root pitchは -0.042310rad であり、過去3本のうち直立に近かった `061853` と同程度である。

以上より、M5.2による歩行準備・歩行開始安全条件の退行は確認されない。

### `allConstraintsSatisfied` の扱い

M5.2後の `wbmsProjectionAllConstraintsSatisfied` は全周期0である。

これは `checkFinalState=false` により `solveIKLoop()` の戻り値を「最終満足判定未実施」としてfalseにしているためであり、M5.2の想定挙動である。projector採用可否は `validateProjectionCandidate()` による `candidate safe` と独自validationで判定される。

今回ログでは:

| 項目 | 値 |
|---|---:|
| `wbmsProjectionAllConstraintsSatisfied` unique | 0のみ |
| projector valid rate | 0.879 |
| candidate safe rate | 0.879 |
| projection status | 20が3152周期、1が433周期 |

`allConstraintsSatisfied=false` がprojector validを常時falseへ落とす退行は起きていない。

### 計算時間評価

M5.2後ログ `071526` の計算時間統計:

| 項目 | mean | p95 | p99 | max |
|---|---:|---:|---:|---:|
| projector | 0.321ms | 0.517ms | 0.582ms | 0.747ms |
| final IK | 0.662ms | 0.945ms | 1.077ms | 1.441ms |
| onExecute | 1.277ms | 1.799ms | 2.018ms | 4.157ms |

最新PASS相当3本 `061848`, `061851`, `061853` 平均との比較:

| 項目 | 最新3本平均 | M5.2後 `071526` | 差分 |
|---|---:|---:|---:|
| projector mean | 0.361ms | 0.321ms | -11.1% |
| projector p99 | 0.730ms | 0.582ms | -20.3% |
| projector max | 1.062ms | 0.747ms | -29.6% |
| final IK mean | 0.766ms | 0.662ms | -13.7% |
| final IK p99 | 1.325ms | 1.077ms | -18.7% |
| final IK max | 2.115ms | 1.441ms | -31.9% |
| onExecute mean | 1.420ms | 1.277ms | -10.0% |
| onExecute p95 | 2.081ms | 1.799ms | -13.6% |
| onExecute p99 | 2.386ms | 2.018ms | -15.4% |
| onExecute max | 4.418ms | 4.157ms | -5.9% |
| onExecute 2ms超過周期数 | 286.7周期 | 37周期 | -87.1% |

M5計画書に記載した `061700` 時間baselineとの比較:

| 項目 | `061700` baseline | M5.2後 `071526` | 評価 |
|---|---:|---:|---|
| projector mean | 0.348ms | 0.321ms | 改善 |
| projector p99 | 0.691ms | 0.582ms | 改善 |
| projector max | 0.932ms | 0.747ms | 改善 |
| final IK mean | 0.775ms | 0.662ms | 改善 |
| final IK p99 | 1.334ms | 1.077ms | 改善 |
| final IK max | 2.114ms | 1.441ms | 改善 |
| onExecute mean | 1.417ms | 1.277ms | 改善 |
| onExecute p95 | 2.048ms | 1.799ms | 改善 |
| onExecute p99 | 2.387ms | 2.018ms | 改善 |
| onExecute max | 3.416ms | 4.157ms | 悪化 |
| onExecute 2ms超過周期数 | 216周期 | 37周期 | 改善 |

onExecute max単体は `061700` baselineより悪化している。ただし、maxは単発外れ値の影響を受けやすく、p95/p99、mean、2ms超過周期数は明確に改善している。M5.2の主目的である、projector/final IKのsolve後constraint再評価削減による500Hz余裕増加は確認できた。

### M5.2 acceptance確認

| 項目 | 結果 | 備考 |
|---|---|---|
| `checkFinalState=true` の既存挙動が維持される | PASS相当 | 既定値はtrueのまま。M5.2ではfalse指定時だけ分岐 |
| projectorは `checkFinalState=false` で動作する | PASS | `allConstraintsSatisfied` は0固定相当、candidate safe/validは維持 |
| projector採用判定が `validateProjectionCandidate()` で維持される | PASS | valid rate 0.879、READY/accept到達 |
| final IKは `checkFinalState=false` で動作し、戻り値に依存しない | PASS | final IK後limit checkと出力は継続 |
| 最新PASS相当条件でREADY到達 | PASS | READY 7.388s |
| READY後walking API accept | PASS | accepted 7.412s |
| operation blend残留なし | PASS | accepted直後operation 0.000000 |
| READY時stability >= 0.99 | PASS | 0.990109 |
| READY前footstep抑制 | PASS | READY前footstep max 1 |
| projector/final IK/onExecute時間が悪化しない | PASS相当 | p95/p99/meanと2ms超過数は改善。onExecute maxのみ `061700` baseline比で悪化 |

### M5.2結論

M5.2は完了扱いでよい。

制御挙動の退行は今回ログでは確認されず、計算時間はprojector/final IKともに改善した。`onExecute` もmean/p95/p99と2ms超過周期数が改善しており、500Hz運用余裕は増えた。一方で、`onExecute max` は単発外れ値として `061700` baselineより大きいため、今後のM5.3以降でもmaxだけを単独指標にせず、p99、2ms超過周期数、発生phaseを併せて見る。

M5.3へ進む場合は、計画書どおりprojector priority 4姿勢参照の削減を次候補とする。

## 現行 `wbmsDebugOut` 対応表

この表はM5.2ログ解析時点の `AutoStabilizer::writeOutPortData()` 実装に基づく。`ast_wbmsDebug` ログでは1列目が時刻であり、data index `i` はログ上の `i+2` 列目に対応する。

| index | 内容 |
|---:|---|
| 0 | raw COM velocity X |
| 1 | raw COM velocity Y |
| 2 | raw COM velocity Z |
| 3 | applied COM velocity X |
| 4 | applied COM velocity Y |
| 5 | applied COM velocity Z |
| 6 | realized COM velocity X |
| 7 | realized COM velocity Y |
| 8 | realized COM velocity Z |
| 9 | raw torso angular velocity roll |
| 10 | raw torso angular velocity pitch |
| 11 | raw torso angular velocity yaw |
| 12 | applied torso angular velocity roll |
| 13 | applied torso angular velocity pitch |
| 14 | applied torso angular velocity yaw |
| 15 | realized torso angular velocity roll |
| 16 | realized torso angular velocity pitch |
| 17 | realized torso angular velocity yaw |
| 18 | WBMS開始時からのCOM offset X |
| 19 | WBMS開始時からのCOM offset Y |
| 20 | WBMS開始時からのCOM offset Z |
| 21 | WBMS開始時からのCHEST RPY offset roll |
| 22 | WBMS開始時からのCHEST RPY offset pitch |
| 23 | WBMS開始時からのCHEST RPY offset yaw |
| 24 | `wbmsOperationModeValue` |
| 25 | `wbmsWalkingStabilityModeValue` |
| 26 | projector valid flag (`wbmsPostureReferenceValid`) |
| 27 | projector計算時間[s] |
| 28 | final IK計算時間[s] |
| 29 | 前回出力更新周期の `onExecute()` 全体計算時間[s] |
| 30 | `wbmsProjectionStatus` |
| 31 | `wbmsProjectionAllConstraintsSatisfied` |
| 32 | `wbmsProjectionCandidateSafe` |
| 33 | `wbmsProjectionSupportHullValid` |
| 34 | candidate root translation step[m] |
| 35 | candidate root rotation step[rad] |
| 36 | candidate max joint step[rad or m] |
| 37 | candidate minimum joint limit margin |
| 38 | candidate max foot position error[m] |
| 39 | candidate max foot rotation error[rad] |
| 40 | final IK後robot COM realized velocity X[foot-mid, m/s] |
| 41 | final IK後robot COM realized velocity Y[foot-mid, m/s] |
| 42 | final IK後robot COM realized velocity Z[foot-mid, m/s] |
| 43 | final IK後CHEST realized angular velocity roll軸相当[foot-mid, rad/s] |
| 44 | final IK後CHEST realized angular velocity pitch軸相当[foot-mid, rad/s] |
| 45 | final IK後CHEST realized angular velocity yaw軸相当[foot-mid, rad/s] |
| 46 | `wbmsWalkingPreparationPhase` |
| 47 | `wbmsWalkingPreparationElapsedTime`[s] |
| 48 | `wbmsWalkingPreparationReturnAlpha` |
| 49 | `wbmsWalkingPreparationHandoffAlpha` |
| 50 | held robot COM height in footMid[m]。保持無効時0 |
| 51 | current robot COM height in footMid[m] |
| 52 | `wbmsWalkingPreparationChestError`[rad] |
| 53 | `wbmsWalkingPreparationComXYError`[m] |
| 54 | `wbmsWalkingPreparationComZError`[m] |
| 55 | `wbmsWalkingPreparationRootError`[rad] |
| 56 | final IK後max joint delta per cycle[rad or m] |
| 57 | pending command release event |
| 58 | `wbmsWalkingPreparationFailureCode` |
| 59 | runtime `wbmsWalkingStabilityStartTime`[s] |
| 60 | final IK後root roll[generate frame, rad] |
| 61 | final IK後root pitch[generate frame, rad] |
| 62 | final IK後root yaw[generate frame, rad] |
| 63 | `stTargetRootPose` roll[generate frame, rad] |
| 64 | `stTargetRootPose` pitch[generate frame, rad] |
| 65 | `stTargetRootPose` yaw[generate frame, rad] |
| 66 | `refdz`[m] |
| 67 | `l.z`[m] |
| 68 | `omega`[1/s] |
| 69 | `refZmpTraj[0]` start X[generate frame, m] |
| 70 | `refZmpTraj[0]` start Y[generate frame, m] |
| 71 | `refZmpTraj[0]` start Z[generate frame, m] |
| 72 | `refZmpTraj[0]` goal X[generate frame, m] |
| 73 | `refZmpTraj[0]` goal Y[generate frame, m] |
| 74 | `refZmpTraj[0]` goal Z[generate frame, m] |
| 75 | `refZmpTraj[0]` time[s] |
| 76 | `refZmpTraj` total time[s] |
| 77 | `footstepNodesList.size()` |
| 78 | `footstepNodesList[0].remainTime`[s] |
| 79 | current footstep `elapsedTime`[s] |
| 80 | right leg support flag |
| 81 | left leg support flag |
| 82 | right leg `swingState` |
| 83 | left leg `swingState` |
| 84 | READY phase flag |
| 85 | FAILED phase flag |
| 86 | walking API rejected-not-ready event |
| 87 | walking API accepted-ready event |
| 88 | walking preparation start event |
| 89 | walking preparation cancel event |
| 90 | applied velocity command norm |
| 91 | return velocity command norm |
| 92 | phase is READY or WALKING_HOLD flag |
| 93 | walking preparation snapshot valid flag |
| 94 | walking preparation settle elapsed time[s] |
| 95 | walking start delay remain time[s] |
| 96 | return root angular velocity roll[rad/s] |
| 97 | return root angular velocity pitch[rad/s] |
| 98 | return root angular velocity yaw[rad/s] |
| 99 | return COM velocity norm[m/s] |
| 100 | return torso angular velocity norm[rad/s] |
| 101 | return root angular velocity norm[rad/s] |
| 102 | return all velocity norm |
| 103 | walking preparation target root roll[rad] |
| 104 | walking preparation target root pitch[rad] |
| 105 | walking preparation target root yaw[rad] |
| 106 | walking preparation target root error[rad] |
| 107 | final IK COM velocity norm[m/s] |
| 108 | final IK CHEST angular velocity norm[rad/s] |
| 109 | READY条件: applied velocity |
| 110 | READY条件: return COM velocity |
| 111 | READY条件: return torso velocity |
| 112 | READY条件: return root velocity |
| 113 | READY条件: CHEST error |
| 114 | READY条件: COM XY error |
| 115 | READY条件: COM Z error |
| 116 | READY条件: root error |
| 117 | READY条件: walking stability |
| 118 | READY条件: candidate safe |
| 119 | READY条件: final IK joint step |
| 120 | READY条件: dynamics |
| 121 | RETURNING中安全判定: final IK COM velocity safe |
| 122 | RETURNING中安全判定: final IK CHEST velocity safe |
| 123 | walking preparation timeout残り[s] |
| 124 | `wbmsWalkingPreparationVelocityEps` |

## コードとビルドで確認済みの事項

- 旧 `wbmsTorsoTargetRpy`、`refTorsoAnglVel`、`calcWbmsPostureReference`、`wbmsPostureRootConstraint`、`WbmsTorsoControl` は `auto_stabilizer/rtc/AutoStabilizer` 配下に残っていない。
- `FullbodyIKSolver` 側に旧reference IK専用robot、旧root姿勢拘束、旧task cache、内部walking stability modeは残っていない。
- `WbmsPostureControl` 側の `wbmsPostureRobot_` はM2の実現可能速度投影用robotであり、削除対象の旧reference IK robotではない。
- 最終IKの優先度は、優先度0 joint velocity / joint limit、優先度1 self collision、優先度2 足、優先度3 上半身EE・CHEST姿勢・COM・角運動量・root姿勢、優先度4 reference angle の構成である。
- 最終IKのCHEST姿勢targetは `gaitParam.wbmsProjectedChestR` である。
- CHEST姿勢拘束の位置weightはゼロ、姿勢weightは `wbmsTorsoOrientationWeight * wbmsOperationModeValue` である。
- CHEST姿勢拘束の角度 `maxError` は `wbmsTorsoOrientationMaxError * dt` である。
- COM targetは `gaitParam.genCog + gaitParam.sbpOffset` である。
- COM weightは通常値と `wbmsComPositionWeight` の間で `wbmsOperationModeValue` によりblendされる。
- reference angleは投影valid、operation mode、投影対象joint maskを考慮して切り替わる。
- 投影variable外の腕などは従来referenceを使う。
- 上半身EEのCHEST相対拘束は `FullbodyIKSolver.cpp` の `B_link() = torsoGenLink` と `B_localpos() = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i]` で維持されている。
- `cpViewerLog` の固定indexは変更していない。
- debug/計測は `wbmsDebugOut` で確認でき、毎周期標準出力は行わない。

## 未解決事項

- M4.2.2 review対応として、READY後のpending releaseより前にtimeoutを判定するよう修正した。timeout超過時は `FAILED/TIMEOUT` へ遷移し、pending commandはreleaseしない。
- M4.2.2 review対応として、歩行準備snapshotのnominal COM X/Yは `LegCoordsGenerator::calcCOMCoords()` 直後、WBMS統合前の `genCog + sbpOffset` から取得するよう修正した。保存値が無効な場合はsnapshot failureとし、WBMS統合済み `genCog` へfallbackしない。
- M4.2.2 review対応として、歩行準備中の投影targetでは残留 `wbmsAppliedComVelocityCommand.z` を混ぜず、snapshot済みの `heldRobotComHeightInFootMid` へ固定するよう修正した。
- M4.2.2 review対応として、歩行準備中の再コマンドではpending commandだけを更新し、phase、snapshot、保持COM高さ、timeout elapsedを初期化しないよう修正した。
- M4.2.2 review対応として、HANDOFF alpha更新後に `wbmsOperationModeValue` を計算し、その周期の投影・final IKへ実際に渡るWBMS重みとREADY判定が一致するよう修正した。
- M4.2.2 review対応として、DECELERATING完了時に現在の投影CHEST姿勢/robot COMをRETURNING開始点へラッチし、復帰targetが歩行指令受付時snapshotへ巻き戻らないよう修正した。
- M4.2.2 review対応として、姿勢制御側でFAILEDになった場合も同じ `onExecute()` 内で `WbmsWalkingCommandDelay` のpending commandだけを破棄し、FAILED診断statusを残したままhidden goalを消すよう修正した。
- `wbmsDebugOut` は新規OutPortであり、既存viewer互換は壊さない判断だが、利用側のlogger設定追加は別途必要。
- `wbmsDebugOut[29]` はOutPort書き込み時間を含めるため、同一周期ではなく前回出力更新周期の確定値を公開する。
- 時間計測は各周期の値を出すのみで、平均、最大、p99の集計は未実装。必要であればlogger側または後続実装で集計する。
- final IKのCHEST姿勢weight、COM weight、各limitは正式仕様の保守的初期値であり、シミュレータ確認後に調整が必要。
- self collision入力数が周期中に増えた場合の `resize()` / constraint生成はM2時点の許容範囲として残る。
- シミュレータログ `auto_stabilizer/log/wbmsDebugOut.txt` では、体幹pitch角速度指令を送っている間も `wbmsDebugOut[26]` のprojector valid flagが全周期0、`wbmsDebugOut[15-17]` のrealized torso angular velocityが全周期0だった。このため最終IKのCHEST拘束と投影reference angleが無効化され、前傾しない。
- 上記問題の直接原因はprojector invalid継続である。M4.1で `solveIKLoop()` 戻り値、candidate safe、validation失敗理由、validation metricsを `wbmsDebugOut[30-39]` へ追加したため、次のシミュレータ確認では失敗原因を切り分けられる。
- `WbmsPostureControl::solveProjection()` はM4.1時点で `bool valid = allConstraintsSatisfied && validation.safe` としている。`maxIteration=1` の `solveIKLoop()` 戻り値をvalid条件に含めることが過剰失敗の原因になっている可能性は残っているが、採用条件変更はM4.2範囲である。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md`、`auto_stabilizer/log/` は未追跡として存在する。既存ユーザー変更またはログ扱いで、不要に削除しないこと。

## シミュレータ確認待ち項目

以下はコードとビルドだけではPASS扱いしない。

- 50Hz入力時のsample-and-hold応答と入力周期非依存性。
- CHEST roll/pitch/yaw、COM前後左右/上下の実応答。
- limit到達時のhidden goal非蓄積と逆方向入力への応答。
- 腕同時操縦時にCHEST相対拘束が期待通り機能すること。
- 前傾、しゃがみ、腕操作の同時実行時に足踏み、yaw振動、急激な下半身振動が出ないこと。
- 歩行開始遅延中にoperation modeが0へ戻り、通常安定化とroot姿勢復帰が維持されること。
- 歩行終了後に古い体幹/COM速度commandが再開しないこと。
- `wbmsDebugOut` のprojector/final IK/onExecute時間を用いた500Hz実時間性能。
- projector invalid問題を解消した後、同じログ条件で `wbmsDebugOut[26]` が1になり、`wbmsDebugOut[15-17]` とCHEST RPY offsetが指令方向に変化すること。

## 次マイルストーンへの引き継ぎ

### 次のマイルストーンが使用するinterface、state、前提条件

- `WbmsPostureControl::proc()` はStabilizer前に呼ばれ、投影結果を `GaitParam` に保存する。
- 投影成功時は `wbmsPostureReferenceValid=true`、`wbmsPostureReferenceQ`、`wbmsPostureReferenceJointMask`、`wbmsProjectedChestR`、`wbmsProjectedRobotCom`、realized velocityが更新される。
- 投影失敗時は `wbmsPostureReferenceValid=false`、`wbmsPostureReferenceJointMask` 全false、realized velocityゼロ、現在姿勢fallbackになる。
- 最終IKは投影valid時だけ `wbmsProjectedChestR` をCHEST姿勢拘束として使う。
- 最終IKのCOM targetは `genCog + sbpOffset`。M2のCOM/ZMP統合によりstatic WBMS中の `genCog` は投影COMへblend済みである。
- 最終IKのreference angleはjoint mask trueの関節だけ投影Qへmode blendし、腕などmask falseの関節は従来referenceを使う。
- `wbmsOperationModeValue` はstatic WBMS操作と通常歩行安定化を切り替える共通係数として使う。
- `wbmsDebugOut` はM5.2評価時点で125要素。現行indexは本書の「現行 `wbmsDebugOut` 対応表」を参照する。`ast_wbmsDebug` ログでは1列目が時刻で、data index `i` はログ上の `i+2` 列目である。

### 次のセッションで最初に確認すべきコード箇所

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
  - `solveProjection()`: `allConstraintsSatisfied` と `validation.safe` のvalid判定。
  - `validateProjectionCandidate()`: nominal姿勢でもunsafeになり得る条件。
  - `setFallbackReference()`: invalid時のdebug値とfallback。
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
  - CHEST拘束追加条件。
  - COM weight blend。
  - reference angle blend。
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
  - `wbmsDebugOut` 要素順。
  - `onExecute()` 計測タイミング。
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
  - `wbmsPostureReferenceJointMask` とdebug計測値。
- `auto_stabilizer/log/wbmsDebugOut.txt`
  - シミュレータで前傾しなかったログ。raw/applied指令、valid、realized velocity、mode値を確認する。

### 次に実行すべきビルド・確認コマンド

```sh
git status --short
catkin build auto_stabilizer --no-deps
rg -n "wbmsTorsoTargetRpy|refTorsoAnglVel|calcWbmsPostureReference|wbmsPostureRootConstraint|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
rg -n "B_link\\(\\) = torsoGenLink|wbmsProjectedChestR|wbmsComPositionWeight|wbmsOperationModeValue" auto_stabilizer/rtc/AutoStabilizer
rg -n "solveIKLoop|bool valid|validateProjection|wbmsPostureReferenceValid|wbmsPostureReferenceJointMask" auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp
git diff --stat
```
