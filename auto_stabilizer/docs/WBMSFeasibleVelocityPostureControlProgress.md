# WBMS実現可能速度投影型・体幹/COM操縦 進捗記録

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
- `wbmsDebugOut` はM4.1時点で40要素。既存index 0-29はM3定義を維持し、30-39にprojection statusとvalidation metricsを追加している。

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
