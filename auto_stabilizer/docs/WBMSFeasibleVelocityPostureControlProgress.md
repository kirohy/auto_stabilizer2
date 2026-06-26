# WBMS実現可能速度投影型・体幹/COM操縦 進捗記録

## 現在の全体状態

対象ブランチは `wbms-dev`。現在の作業は、承認済み実装計画の「マイルストーン2: 実現可能速度投影とCOM/ZMP統合」まで完了している。

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

本記録時点で、以下は未実装であり、後続マイルストーン範囲である。

- 最終IKへの投影CHEST姿勢拘束追加。
- 最終IKのCOM weight blend。
- 最終IKのreference angleを `wbmsPostureReferenceQ` へ切り替える処理。
- 旧reference IK関連の最終整理。
- 腕CHEST相対拘束の最終確認と、M3範囲の最終IK優先度整理。
- debug OutPortまたはdebug logの公開整備。
- 500 Hz実時間計測。
- シミュレータ上の応答、安定性、操作感確認。

## 完了済みマイルストーン一覧

| マイルストーン | 名称 | 状態 |
|---|---|---|
| 1 | API・状態・入力保持基盤 | 完了。ビルド成功、review指摘対応済み |
| 2 | 実現可能速度投影とCOM/ZMP統合 | 完了。ビルド成功、review指摘対応済み |
| 3 | 最終IK統合・旧方式削除・debug/計測 | 未着手 |

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

## 今回のマイルストーン完了記録

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

## 重要な実装判断とその理由

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

## 仕様との差異

### M1で旧reference IKの一部削除を先行した

承認済み計画では旧reference IKの本格削除はM3に含まれるが、M1で旧 `refTorsoAnglVel` / `wbmsTorsoTargetRpy` を削除し、かつ新commandを旧root reference IKへ接続しない必要があった。このため、`FullbodyIKSolver::calcWbmsPostureReference()` と旧root-reference WBMS姿勢IK専用メンバをM1で削除した。

これは新しい制御機能の先行実装ではない。M2終了時点でも、新commandと投影結果は最終IKへはまだ接続されていない。

### 支持多角形退化時の扱い

正式仕様書8.5には「縮小後に有効な多角形を作れない場合、COM XY速度入力をその周期はゼロ扱い」とある。M2実装では、review指摘を受け、支持多角形が無効な場合は投影IK全体を失敗扱いにして現在姿勢へfallbackする。

理由は、縮小支持多角形が無い状態で処理を継続すると、ZMP XYを支持多角形内へ射影できないまま `refZmpTraj` を上書きできるためである。COM XYをゼロ扱いにする意図は安全側制限であり、ZMP射影不能時はfallbackの方が安全側である。

### COMとCHESTの優先度

COM拘束をCHEST姿勢拘束より高優先度へ分離するreview指摘があったが、正式仕様書8.6と承認済み計画M2は、COM位置3DとCHEST姿勢3Dを同じ優先度のソフトタスクとして解くことを明記している。このため、コードは `projectionConstraints_[3]` に両方を入れる現状を維持している。

## ビルド・review結果

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

## コードとビルドで確認済みの事項

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

## 未解決事項

- M3の最終IK CHEST姿勢拘束は未実装。
- M3のCOM weight blendは未実装。
- M3のreference angle切替は未実装。
- M3の旧reference IK最終整理は未実装。
- M3のdebug OutPortまたはdebug logの最終仕様は未決定。
- M3の500 Hz実時間計測は未実施。
- self collision入力数が周期中に増える場合は、正式仕様で許容されている範囲ではあるが、その周期に `resize()` / `make_shared()` が発生し得る。
- 支持多角形縮小が頻繁に退化する場合、パラメータ調整または追加の異常報告が必要。
- シミュレータ確認前のため、実応答、操作感、安定性、500 Hz実測は未確認。
- `auto_stabilizer/.cache/`、`auto_stabilizer/compile_commands.json`、`auto_stabilizer/docs/WBMSTorsoArmIKDesignPlan.md`、`auto_stabilizer/docs/WBMSTorsoArmIKExperimentLog.md`、`auto_stabilizer/docs/WBMSWalkingControlSummary.md` は未追跡として存在する。既存ユーザー変更扱いで、不要に削除しないこと。

## シミュレータ確認待ち項目

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

## 次マイルストーンへの引き継ぎ

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
