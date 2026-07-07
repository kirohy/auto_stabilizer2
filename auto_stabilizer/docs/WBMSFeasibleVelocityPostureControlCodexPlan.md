# WBMS実現可能速度投影型・体幹/COM操縦 引き継ぎ計画書

## 現行優先度に関する注記

本書は、最初期の `WBMSFeasibleVelocityPostureControlImplementationPlan.md` をM1からM3の実装作業へ分割するために作成された引き継ぎ計画書である。

後続作業により、本書の「正式仕様書としてImplementationPlanを必ず読む」という記述は、初期M1からM3の文脈に限定して扱う。現行コードの最新仕様・優先順位を判断する場合は、まず `WBMSFeasibleVelocityPostureControlProgress.md` を読み、対象範囲ごとの後続計画書を優先する。

特に以下は本書より後続文書を優先する。

- projector候補採用判定、safe candidate、`solveIKLoop()` 戻り値の扱い: `WBMSProjectionAcceptanceFixImplementationPlan.md`
- 歩行準備遷移、READY判定、歩行API受付可否、pre-walk姿勢生成: `WBMSWalkingPreparationDesignRevisionPlan.md`
- 歩行準備遷移の前提・経緯: `WBMSWalkingPreparationTransitionImplementationPlan.md`
- 500 Hz計算量削減、IK parameter、`checkFinalState`、prioritized IK軽量化: `WBMSComputationReductionImplementationPlan.md`

本書と後続文書が矛盾する場合、本書を根拠に後続文書の仕様を上書きしてはならない。

## 正式仕様書

正式仕様書として、以下を必ず読むこと。

`auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`

対象リポジトリは現在の作業ディレクトリ。対象ブランチは `wbms-dev`。

本計画は上記仕様書を実装へ落とすためのマイルストーン再編成であり、仕様書と矛盾する場合は推測せず確認する。

## 調査で判明した現行コードの重要事項

- `git branch --show-current` は `wbms-dev`。
- `git status --short` では `AGENTS.md`, `auto_stabilizer/docs/`, `auto_stabilizer/.cache/`, `auto_stabilizer/compile_commands.json` が未追跡だった。既存変更として扱い、勝手に削除・巻き戻ししない。
- `AGENTS.md` は存在し、以下を守る必要がある。
  - `clang-format` を実行しない。
  - テストコードは新規作成しない。
  - コメント/Markdownは日本語。
  - 実ロボット向けなので安全側に倒す。
  - 指令値の不連続、hidden goal、mode切替時の無limiter速度指令を避ける。
- 現行 `WbmsTorsoControl` は `wbmsTorsoTargetRpy` を積分しており、正式仕様の「到達不能目標を蓄積しない」と不一致。
- 現行 `readInPortData()` は `refTorsoVelIn` が新着でない周期に体幹角速度目標をゼロへ戻しており、sample-and-hold要求と不一致。
- 現行 `FullbodyIKSolver::calcWbmsPostureReference()` はroot姿勢目標を使う旧reference IK方式。正式仕様では `WbmsPostureControl` へ移し、CHEST姿勢+COM速度投影方式へ置換する。
- 腕のWBMS中CHEST相対拘束は現行 `FullbodyIKSolver.cpp` に存在する。これは必ず維持する。
- `WbmsWalkingCommandDelay`、`isWbmsWalkingStartDelay`、`wbmsWalkingStartDelayRemainTime`、`wbmsWalkingStabilityStartTime`、`wbmsWalkingStabilityStopTime` は現行に存在する。歩行開始遅延は維持する。
- IK制約APIは利用可能。
  - `ik_constraint2::PositionConstraint`
  - `ik_constraint2::COMConstraint`
  - `ik_constraint2::JointVelocityConstraint`
  - `ik_constraint2_joint_limit_table::JointLimitMinMaxTableConstraint`
  - `ik_constraint2::ClientCollisionConstraint`
  - `ik_constraint2::JointAngleConstraint`
  - `prioritized_inverse_kinematics_solver2::solveIKLoop()`
- `MathUtil` には凸包生成、内外判定、最近傍射影があるが、縮小凸包helperは未実装。
- 現行実行順は概ね以下。
  - frame変換
  - 外力処理
  - impedance
  - leg manual
  - cmd vel
  - footstep
  - leg coords
  - COM coords
  - `abcEETargetPose` 更新
  - Stabilizer
  - FullbodyIKSolver
- 正式仕様の `WbmsPostureControl::proc()` は `calcCOMCoords` / `abcEETargetPose` 更新後、Stabilizer前に挿入する。

## 最終マイルストーン構成

3マイルストーンで実装する。

1. API・状態・入力保持基盤
2. 実現可能速度投影とCOM/ZMP統合
3. 最終IK統合・旧方式削除・debug/計測

各マイルストーン終了時にリポジトリはビルド可能な状態にする。後続で破棄する暫定実装は作らない。

## マイルストーン1: API・状態・入力保持基盤

### 変更対象

- `auto_stabilizer/idl/AutoStabilizerService.idl`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsTorsoControl.h/.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h/.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/CMakeLists.txt`

### 実装内容

- `WbmsTorsoControl` を `WbmsPostureControl` に置換する。
- IDLへ正式仕様書6.2の新規WBMSパラメータを追加する。
- 既存体幹パラメータのコメント/意味を正式仕様に合わせる。
- `GaitParam` へ以下の状態を追加する。
  - raw/applied COM velocity command
  - raw/applied torso angular velocity command
  - command age / valid
  - WBMS開始時CHEST姿勢・COM baseline
  - 投影結果
  - `wbmsWalkingStabilityModeValue`
  - `wbmsOperationModeValue`
  - debug用状態
- `refTorsoVelIn` はsample-and-holdへ変更する。
  - finiteな新着だけraw command更新。
  - 新着なしではraw commandを変更しない。
  - `vx/vy/vz` はCOM速度、`vr/vp/va` はCHEST角速度として扱う。
  - timeout、操作不可、invalid時はdesiredをゼロにする。
  - applied commandは成分ごとの加速度limitで追従させる。
- `wbms_interpolate_duration` はエンドエフェクタ姿勢補間用に残し、速度commandのsample-and-holdには使わない。
- `WbmsPostureControl::start()` 相当で以下を行う。
  - `genRobot` のFK/COM更新。
  - `footMidCoords` 取得。
  - foot-mid基準の開始時CHEST姿勢とrobot COMを保存。
  - raw/applied velocityをゼロ化。
  - command ageをtimeout超過、valid falseへ。
  - 投影robotを現在 `genRobot` へ同期。
  - `wbmsPostureReferenceValid=false`。
- 以下でclearする。
  - `stopWholeBodyMasterSlave()`
  - `stopAutoBalancer()`
  - `MODE_SYNC_TO_ABC` 初期化
  - RTC activate/deactivate
  - 歩行開始遅延への移行時
- M1では投影IKや最終IKへ暫定接続しない。

### 依存関係

なし。

### 完了条件

- `refTorsoVelIn` 新着なし周期でcommandがゼロ化されない。
- timeout後は加速度limitでゼロへ戻る。
- WBMS停止、AB停止、RTC deactivate、歩行開始遅延、sync initでstale commandが再開しない。
- IDL set/getで新規パラメータを読み書きできる。
- 旧 `WbmsTorsoControl` 依存が除去される。
- ただし旧root reference IKへ新入力を暫定接続しない。

### 確認コマンド

```sh
catkin build auto_stabilizer --no-deps --force-cmake
rg -n "refTorsoAnglVel|wbmsTorsoTargetRpy|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
rg -n "wbms_velocity_command_timeout|wbms_com_velocity_limit|wbmsOperationModeValue" auto_stabilizer
```

### 想定リスク

- IDL変更後の初回buildに `--force-cmake` が必要。
- `readInPortData()` の引数変更漏れ。
- clear箇所漏れによるstale command再開。

## マイルストーン2: 実現可能速度投影とCOM/ZMP統合

### 変更対象

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/MathUtil.h`
- `auto_stabilizer/rtc/AutoStabilizer/MathUtil.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`

### 実装内容

- `WbmsPostureControl::init()` で以下を事前構築する。
  - 投影用robot clone
  - 投影IK variables
  - dqWeight
  - constraint object
  - priority別constraint vector/task cache
  - ancestor joint ID集合
- 毎周期行ってはいけないもの。
  - robot clone
  - constraint object new
  - ancestor探索
  - 不要なvector再確保
- 投影IK変数は以下に限定する。
  - floating root
  - 左右足parent linkからrootまでのancestor joint
  - `chestLinkName` からrootまでのancestor joint
  - 上記unionのうち `jointControllable=true`
  - rootの後はjoint ID順
- 操作有効条件は正式仕様4.4に従う。
  - AutoBalancer実行中。
  - WBMS有効または有効へ遷移中。
  - `gaitParam.isStatic()`。
  - 両脚support phase。
  - `isWbmsWalkingStartDelay=false`。
  - 左右脚がmanual control対象でない。
- CHEST姿勢目標はrootではなく `gaitParam.chestLinkName` の世界姿勢を扱う。
  - foot-mid基準へ変換。
  - applied angular velocityから1周期先目標を作る。
  - WBMS開始時CHEST姿勢からの差分をlower/upperへclamp。
  - world CHEST目標へ戻す。
- COM目標はrobot生COMを扱う。
  - foot-mid基準へ変換。
  - applied COM velocityから1周期先目標を作る。
  - WBMS開始時COMからのoffsetをlower/upperへclamp。
  - XYは縮小支持多角形へ射影。
- `MathUtil::shrinkConvexHull2D()` を追加する。
  - margin 0なら元hull。
  - 退化時はCOM XY速度をその周期ゼロ扱い。
  - `calcNearestPointOfHull()` で目標XYを射影。
- 投影IK優先度は正式仕様8.6に従う。
  - 0: `JointVelocityConstraint`, `JointLimitMinMaxTableConstraint`
  - 1: 近いself collisionのみ `ClientCollisionConstraint`
  - 2: 両足6D拘束
  - 3: CHEST姿勢3D + COM位置3D
  - 4: 必要時のみ小weight `JointAngleConstraint`
- solver設定。
  - `maxIteration=1`
  - `dt=dt`
  - `precision=0.0`
  - `wn/we` は現行reference IK相当から開始
  - `dqWeight` は既存 `FullbodyIKSolver::dqWeight` 参照
  - 1周期内の再solveなし
- 投影結果検証を行う。
  - finite
  - 関節limit内
  - 両足位置姿勢誤差許容内
  - 異常な1周期変位なし
- invalid時は以下。
  - `wbmsPostureReferenceValid=false`
  - reference qは現在 `genRobot`
  - projected CHEST/COMは現在値
  - realized velocityはゼロ
  - `genCog` / ZMPを投影結果で上書きしない
  - 残差を次周期へ持ち越さない
- 投影成功かつoperation mode有効時、以下をblend更新する。
  - `genCog`
  - `genCogVel`
  - `genCogAcc`
  - `refdz`
  - `omega`
  - `l`
  - static WBMS用 `refZmpTraj`
- `execAutoStabilizer()` では `calcCOMCoords` と `abcEETargetPose` 更新後、Stabilizer前に `WbmsPostureControl::proc()` を呼ぶ。

### 依存関係

M1の状態、パラメータ、command処理が必要。

### 完了条件

- 投影結果が `GaitParam` のreference q、projected CHEST/COM、realized velocity、valid flagに反映される。
- 投影失敗時にhidden goalや残差を持ち越さない。
- 歩行中・歩行開始遅延中はCOM/ZMP/CHEST参照を変更しない。
- 毎周期allocationを増やさない構造。
- M2終了時点でbuild可能。

### 確認コマンド

```sh
catkin build auto_stabilizer --no-deps
rg -n "clone\\(|ancestor|wbmsPostureRobot|shrinkConvexHull2D|calcNearestPointOfHull" auto_stabilizer/rtc/AutoStabilizer
rg -n "WbmsPostureControl::proc|execStabilizer|calcCOMCoords" auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp
```

### 想定リスク

- `solveIKLoop()` は `maxIteration=1` なので戻り値だけで成功/失敗を判断すると過剰失敗の可能性がある。戻り値に加え、独自検証を最終判定に使う。
- COM/ZMP/refdz/omegaはStabilizer入力へ直接効くため、不連続抑制と加速度clampを保守的にする。
- 支持多角形縮小が退化する姿勢ではCOM XY入力が無効化される。

## マイルストーン3: 最終IK統合・旧方式削除・debug/計測

### 変更対象

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
- 必要ならdebug OutPort
- `auto_stabilizer/docs` の作業記録

### 実装内容

- `FullbodyIKSolver` から旧reference IK関連を削除する。
  - `wbmsPostureRobot`
  - `wbmsPostureFootConstraint`
  - `wbmsPostureRootConstraint`
  - `wbmsPostureTasks`
  - `calcWbmsPostureReference()`
  - 内部 `wbmsWalkingStabilityMode`
- 最終IKへCHEST姿勢 `PositionConstraint` を追加する。
  - `A_link = genRobot->link(chestLinkName)`
  - `A_localpos = Identity`
  - `B_link = nullptr`
  - `B_localpos.linear = wbmsProjectedChestR`
  - 位置weight = 0
  - 姿勢weight = `wbms_torso_orientation_weight * wbmsOperationModeValue`
  - maxError = `wbms_torso_orientation_max_error * dt`
- COM拘束targetは従来どおり `gaitParam.genCog + gaitParam.sbpOffset`。
- COM weightは通常値と `wbms_com_position_weight` をmode blendする。
- reference angleは以下。
  - 投影成功時: `wbmsPostureReferenceQ`
  - 投影無効時: 従来 `refRobot`
  - 遷移中: `wbmsOperationModeValue` でblend
  - 投影変数外の腕などは従来reference
- 腕CHEST相対拘束を維持する。
  - `B_link = torsoGenLink`
  - `B_localpos = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i]`
- 最終IK優先度を整理する。
  - 0: joint velocity / joint limit
  - 1: self collision
  - 2: 足EE
  - 3: 上半身CHEST相対EE、投影CHEST姿勢、投影COM、角運動量、歩行root姿勢
  - 4: reference angle
- debug情報を確認可能にする。
  - raw/applied/realized COM velocity
  - raw/applied/realized torso angular velocity
  - WBMS開始時からのCOM offset
  - WBMS開始時からのCHEST RPY offset
  - `wbmsOperationModeValue`
  - `wbmsWalkingStabilityModeValue`
  - projector valid flag
  - projector計算時間
- `cpViewerLog` 固定indexを壊す恐れがあれば、新規 `TimedDoubleSeq` OutPortを使う。
- 時間計測を通常実行で毎周期標準出力しない形で用意する。
  - `WbmsPostureControl::proc()`
  - `FullbodyIKSolver::solveFullbodyIK()`
  - `onExecute()`
- docsへ実装結果、採用パラメータ、残課題、シミュレータ確認項目を日本語で記録する。

### 依存関係

M2の投影結果が必要。

### 完了条件

- 旧root姿勢積分方式が制御経路から消えている。
- CHEST相対腕拘束と歩行開始遅延が維持されている。
- debug/計測値を確認できる。
- buildが成功する。
- 仕様書17のコード/API/構造面の受入基準を満たす。

### 確認コマンド

```sh
catkin build auto_stabilizer --no-deps
rg -n "wbmsTorsoTargetRpy|refTorsoAnglVel|calcWbmsPostureReference|wbmsPostureRootConstraint|WbmsTorsoControl" auto_stabilizer/rtc/AutoStabilizer
rg -n "B_link\\(\\) = torsoGenLink|wbmsProjectedChestR|wbmsComPositionWeight|wbmsOperationModeValue" auto_stabilizer/rtc/AutoStabilizer
git diff --stat
```

### 想定リスク

- COM weight切替で歩行中の通常COM Z/root姿勢復帰を弱めると歩行安定化を壊す。`wbmsOperationModeValue` を唯一の切替係数として使う。
- debug OutPort追加はRTC port互換や既存viewerへ影響する可能性がある。port名衝突と `cpViewerLog` 固定indexを確認する。
- CHEST姿勢拘束weightが高すぎると腕・COM・足拘束と競合する。正式仕様の保守的初期値から開始する。

## 仕様書との対応表

| 仕様書セクション | 内容 | 実装/検証 |
|---|---|---|
| 3.1 | 腕操縦と歩行API維持、腕CHEST相対拘束維持 | M3、シミュレータ |
| 3.2 | 歩行開始遅延維持 | M1 clear連携、M2/M3 mode反映 |
| 3.3, 4.1 | 到達不能目標を蓄積しない速度投影 | M2、M3 |
| 4.2 | rootではなくCHEST姿勢操作 | M2、M3 |
| 4.3 | `refTorsoVelIn` をCOM速度+CHEST角速度に再利用 | M1、M2 |
| 4.4 | 静止両足支持中のみ有効 | M1/M2 |
| 5 | sample-and-hold、timeout、加速度limit、stale禁止 | M1 |
| 6 | `GaitParam`状態、IDLパラメータ、set/get | M1 |
| 7 | WBMS開始/停止/リセット初期化 | M1 |
| 8 | 実現可能速度投影IK | M2 |
| 9 | COM/ZMP/omega/l整合 | M2 |
| 10 | 最終IK反映、旧reference IK削除 | M3 |
| 11 | mode遷移 | M1/M2/M3 |
| 12 | 500Hz、allocation抑制、時間計測 | M2/M3 |
| 13 | debug情報 | M3 |
| 14 | 変更対象ファイル | M1-M3 |
| 15 | 実装手順 | M1-M3へ再編 |
| 16 | 検証項目 | コード確認 + シミュレータ |
| 17 | 受入基準 | 下表へ割当 |
| 18 | 禁止方式 | 全Mで遵守 |
| 19 | 実装意図 | 全Mで遵守 |

## 仕様書と異なる判断、その理由

| 項目 | 仕様書の方式 | 本計画の方式 | 理由 | 要求を満たせる根拠 |
|---|---|---|---|---|
| 作業フェーズ数 | Phase 1-6 | M1-M3へ統合 | 各終了時にbuild可能にし、暫定実装を減らすため | 内容は削らず、API/投影/最終IKへ整理しただけ |
| 投影IK成功判定 | solve後に検証 | solver戻り値に加え独自検証を最終判定 | `maxIteration=1` では戻り値だけだと過剰失敗の恐れ | finite、limit、足誤差、変位を直接確認するため安全要求を満たす |
| Phase 1のdebug確認 | 旧reference IKへ接続せずdebug値だけ確認可 | M1では新状態だけ作り、旧方式へ接続しない | 後で破棄する暫定接続を避けるため | M2で投影結果をCOM/ZMPへ接続し、M3で最終IKへ接続する |
| debug出力 | 既存DebugDataまたは新規OutPort | 既存viewer互換を確認し、危険なら新規OutPort | `cpViewerLog` 固定index変更は既存viewerを壊す可能性 | debug確認可能性を維持しつつ互換性リスクを避ける |

## 受入基準1〜12の割当

| 受入基準 | 割当 |
|---|---|
| 1. 50Hz一定入力で周期非依存の一定速度 | M1入力保持、M2/M3統合、シミュレータ |
| 2. 体幹姿勢とCOM高さを独立調整 | M2投影、M3最終IK、シミュレータ |
| 3. hidden goalが蓄積しない | M2速度投影、M3旧積分削除 |
| 4. 制約到達後に逆方向入力へ即応 | M2投影、シミュレータ |
| 5. 両腕操縦と同時使用 | M3腕拘束維持、シミュレータ |
| 6. 腕CHEST相対拘束維持 | M3コード確認 |
| 7. 既存歩行APIと歩行開始遅延維持 | M1/M2/M3、シミュレータ |
| 8. 歩行中COM操縦無効、通常安定化復帰 | M2 mode/COM/ZMP制御、シミュレータ |
| 9. 足拘束、関節limit、自己干渉優先 | M2投影IK、M3最終IK優先度 |
| 10. 500Hz周期維持 | M2 allocation抑制、M3計測、シミュレータ |
| 11. IDL set/getで新規パラメータ読み書き | M1 |
| 12. ビルド成功 | 各M終了時 |

## 実装時に維持すべき不変条件

- `clang-format` を実行しない。
- テストコードを新規作成しない。
- コメントとMarkdownは日本語。
- 腕WBMS中の上半身EE拘束はCHEST相対のまま維持する。
- root/world基準へ戻さない。
- `WbmsWalkingCommandDelay` と歩行開始遅延状態を維持する。
- 歩行開始遅延に入った時点から体幹/COM操縦は滑らかに無効化する。
- `refTorsoVelIn` 新着なしだけで速度をゼロにしない。
- 到達不能な姿勢/COM目標を内部に蓄積しない。
- COM速度とCHEST角速度は加速度limitを通す。
- 歩行中にstatic WBMSと同じCOM速度操作を有効にしない。
- ユーザー指令を足拘束、関節limit、自己干渉より高優先度にしない。
- 1周期内のQP/IK solve回数を増やさない。
- 毎周期robot clone、constraint object new、ancestor探索をしない。
- 500Hz実行を設計条件とする。
- IDL変更後の初回buildは `catkin build auto_stabilizer --no-deps --force-cmake`。
- それ以外は `catkin build auto_stabilizer --no-deps`。
- 既存の未追跡/未コミット変更を勝手に削除・巻き戻ししない。

## コード・ビルドだけで確認できる項目

- IDL新規項目の生成とset/get反映。
- sample-and-holdで新着なしゼロ化が消えていること。
- stale command clear箇所がWBMS停止、AB停止、RTC deactivate、歩行開始遅延、sync initにあること。
- 旧積分状態・旧root reference IKが制御経路から消えていること。
- `WbmsPostureControl::proc()` がStabilizer前に呼ばれること。
- CHEST相対腕拘束が維持されていること。
- 各マイルストーン終了時のcatkin build成功。

## 最終シミュレータ確認が必要な項目

- 50Hz入力保持。
- timeout。
- limit到達後のhidden goal非蓄積。
- 逆入力応答。
- CHEST roll/pitch/yaw。
- COM上下/前後左右。
- 前屈+しゃがみ。
- 腕同時操縦で手先がworld固定的に体幹運動を阻害しないこと。
- 到達不能姿勢でsolver不安定化や蓄積追従がないこと。
- 歩行開始遅延中のoperation mode低下。
- future footstep生成前の姿勢復帰。
- 歩行終了後のstale command非再開。
- 歩行中はCOM操縦が無効で、腕操縦と歩行APIは従来どおり併用可能なこと。
- 500Hz継続。
- projector/final IK/onExecuteの平均・最大・可能ならp99。

## 未解決事項

- debug情報を既存 `cpViewerLog` に追加するか、新規 `TimedDoubleSeq` OutPortにするかは、既存viewer互換を確認して決める。固定indexを壊す恐れがあれば新規OutPortを使う。
- `solveIKLoop()` の戻り値をどの程度重視するかは、M2実装時に現行solver挙動を見て調整する。ただし最終判定はfinite、limit、足誤差、異常変位の独自検証を必須にする。
- `wbms_com_position_weight`、`wbms_torso_orientation_weight`、各limitは正式仕様の推奨初期値から始めるが、最終値はシミュレータ確認で調整対象。
- 時間計測の具体的な出力方法は未決定。通常実行で毎周期標準出力しないことだけは固定。
- 支持多角形縮小が退化する場合の操作者向け挙動は「その周期のCOM XY速度ゼロ扱い」とするが、頻発する場合はパラメータ調整または追加報告が必要。
