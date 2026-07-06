# M4.2.2設計修正 実装計画

## 1. 文書の位置づけ

この文書は、`WBMSWalkingPreparationTransitionImplementationPlan.md` の後続として、現状コードとログ調査を踏まえた「M4.2.2設計修正」の実装計画を記録する。

正式な新マイルストーン番号は追加しない。本作業は `M4.2.2設計修正` と呼ぶ。

この文書は設計判断、修正方針、acceptance criteria、次スレッドへの実装引き継ぎを扱う。シミュレータログの数値詳細、時刻表、実施済み作業の経過は `WBMSFeasibleVelocityPostureControlProgress.md` に記録する。

既存文書とこの文書が矛盾する場合、次の点についてはこの文書を優先する。

- WBMS中に歩行準備がREADYでない場合、歩行APIを実行しない。
- 未READY中の歩行APIをpending保存しない。
- READYになっただけで自動歩行開始しない。
- `goVelocity(0.0, 0.0, 0.0)` の既存仕様はWBMS外およびREADY後で維持する。
- WBMS姿勢から歩行可能姿勢への移行は、歩行APIではなく専用serviceで開始する。
- 固定秒returnではなく、速度・加速度limit型のpre-walk姿勢生成へ変更する。

## 2. 現状コードとログから見た破綻経路

### 2.1 歩行API入口

歩行APIの入口は次の通りである。

- `AutoStabilizerService_impl::goPos()` から `AutoStabilizer::goPos()`
- `AutoStabilizerService_impl::goVelocity()` から `AutoStabilizer::goVelocity()`
- `AutoStabilizerService_impl::setFootSteps()` から `AutoStabilizer::setFootSteps()`
- `AutoStabilizerService_impl::setFootStepsWithParam()` から `AutoStabilizer::setFootStepsWithParam()`

`AutoStabilizer::goVelocity()` は、WBMS外では現在も次の既存挙動を持つ。

```text
cmdVelGenerator_.refCmdVel = input velocity
footStepGenerator_.isGoVelocityMode = true
return true
```

したがって `goVelocity(0.0, 0.0, 0.0)` は、速度がゼロでもgoVelocity modeを開始し、その場足踏みを開始し得る。これは既存運用で使われてきた仕様なので変更しない。

### 2.2 現行M4.2.2Bの問題

現行コードには `WbmsWalkingCommandDelay` があり、WBMS static中の歩行API呼び出しに対して次を行う。

```text
歩行API呼び出し
↓
shouldDelay() がtrue、または isWbmsWalkingStartDelay がtrue
↓
storeGoVelocity / storeGoPos / storeFootSteps
↓
pending commandを保存
↓
preparation phaseがREADYになる
↓
releasePendingCommand()
↓
footstep生成、またはgoVelocity mode開始
```

この設計は、今回の正式仕様と矛盾する。正式仕様では、WBMS中かつ歩行準備がREADYでない場合、歩行APIは実行せず、pending保存もせず、`false` を返す必要がある。

### 2.3 ログからの推定

調査対象ログ:

```text
auto_stabilizer/log/test_start_walking202607061324*
```

条件:

- 体幹を前傾させた状態から開始
- ログ取得開始から1秒後に `goVelocity(0.0, 0.0, 0.0)` を送信
- 5秒間ログ取得
- 腕指令なし
- WBMS関連パラメータは初期値

`ast_wbmsDebug` では、指令後にpreparation phaseが進む一方で、`currentRobotComHeightInFootMid`、`refdz`、ZMP軌道時間、footstep関連値が大きく変動している。READYへ安定収束する前に歩行側の時系列が変わり、体幹・COM・root・ZMPのhandoffが不連続になって跳ね上がった可能性が高い。

推定される破綻経路は次である。

```text
WBMS static中に goVelocity(0,0,0)
↓
現行コードが歩行APIをpending保存、または直接goVelocity modeへ進める
↓
歩行可能姿勢への復帰完了前にfootstep列またはgoVelocity modeが変化
↓
同じ制御周期付近でnominal COM/ZMP、root姿勢weight、WBMS姿勢weightが変化
↓
COM Z / refdz / l.z / omega の整合が崩れ、姿勢とZMPが不連続化
↓
シミュレータ上で跳ね上がり、最終的に破綻
```

## 3. 修正方針

### 3.1 歩行API gate

歩行APIは次の挙動にする。

```text
WBMS無効
  既存挙動を完全維持

WBMS有効、かつ歩行準備READYでない
  footstep生成しない
  goVelocity modeを開始しない
  pending保存しない
  falseを返す

WBMS有効、かつ歩行準備READY
  既存歩行APIを実行する
  成功した場合だけ WALKING_HOLD へ移行する
```

これにより、`goVelocity(0.0, 0.0, 0.0)` の既存仕様はWBMS外とREADY後で維持される。一方、未READY中は安全側にrejectする。

### 3.2 専用service

WBMS姿勢から歩行可能姿勢へ戻す操作は、歩行APIではなく専用serviceで開始する。

IDL案:

```idl
enum WbmsWalkingPreparationPhase {
  WBMS_WALKING_PREPARATION_INACTIVE,
  WBMS_WALKING_PREPARATION_REQUESTED,
  WBMS_WALKING_PREPARATION_DECELERATING,
  WBMS_WALKING_PREPARATION_RETURNING,
  WBMS_WALKING_PREPARATION_HANDOFF,
  WBMS_WALKING_PREPARATION_READY,
  WBMS_WALKING_PREPARATION_WALKING_HOLD,
  WBMS_WALKING_PREPARATION_FAILED
};

enum WbmsWalkingPreparationFailureCode {
  WBMS_WALKING_PREPARATION_FAILURE_NONE,
  WBMS_WALKING_PREPARATION_FAILURE_SNAPSHOT,
  WBMS_WALKING_PREPARATION_FAILURE_TIMEOUT,
  WBMS_WALKING_PREPARATION_FAILURE_NONFINITE,
  WBMS_WALKING_PREPARATION_FAILURE_UNSAFE,
  WBMS_WALKING_PREPARATION_FAILURE_CANCELLED
};

struct WbmsWalkingPreparationState {
  WbmsWalkingPreparationPhase phase;
  WbmsWalkingPreparationFailureCode failure_code;
  boolean active;
  boolean ready;
  boolean com_height_hold_valid;
  double elapsed_time;
  double phase_elapsed_time;
  double settle_elapsed_time;
  double held_robot_com_height_in_foot_mid;
  double chest_error;
  double com_xy_error;
  double com_z_error;
  double root_error;
};

boolean startWbmsWalkingPreparation();
boolean cancelWbmsWalkingPreparation();
boolean getWbmsWalkingPreparationState(out WbmsWalkingPreparationState state);
```

`startWbmsWalkingPreparation()` は歩行指令を保存しない。歩行準備状態だけを開始する。

想定手順:

```text
startWbmsWalkingPreparation()
↓
getWbmsWalkingPreparationState() などでREADY確認
↓
goVelocity / goPos / setFootSteps / setFootStepsWithParam
```

READYになっただけで自動歩行開始してはならない。

### 3.3 `WbmsWalkingCommandDelay` の扱い

既存 `WbmsWalkingCommandDelay` は名前と責務が現在の正式仕様に合わない。

推奨方針:

- `WbmsWalkingCommandDelay` を `WbmsWalkingPreparationController` へ置き換える。
- pending commandの保存・release処理を削除する。
- preparation phase管理、snapshot、clear、failure設定のみを扱う。

互換のために `isWbmsWalkingStartDelay` は集約フラグとして残してよいが、内部判断はphase enumを使う。

## 4. Pre-Walk姿勢生成設計

### 4.1 固定秒returnの廃止

現行M4.2.2Bでは、RETURNING中に `wbmsWalkingPreparationReturnAlpha = elapsed / return_time` を使ってCHESTとCOM XYを補間している。

この方式は、戻す量が大きい場合も小さい場合も同じ秒数で戻すため、安全な最大速度・最大加速度を保証しない。設計修正では、固定秒returnを廃止し、速度・加速度limit型へ変更する。

`wbms_walking_preparation_return_time` は既存IDL互換のため残してよいが、pre-walk姿勢生成の主制御には使わない。

### 4.2 CHEST roll/pitch/yaw

戻す対象:

- CHEST roll
- CHEST pitch
- CHEST yaw

戻し先:

- `wbmsStartChestRInFootMid`

制御:

- footMid基準で現在target RPYとtarget RPY速度を持つ。
- 目標RPYとの差分からdesired angular velocityを作る。
- `wbmsTorsoAngularVelocityLimit` で速度制限する。
- `wbmsTorsoAngularAccelerationLimit` で速度変化を制限する。
- 制限後の速度を積分してtarget CHEST姿勢を更新する。

### 4.3 COM X/Y

戻す対象:

- robot COM X
- robot COM Y

戻し先:

- `LegCoordsGenerator::calcCOMCoords()` が生成し、WBMS統合を適用する前のnominal `genCog + sbpOffset`
- footMid基準のX/Yのみ使う

制御:

- footMid基準で現在target COM XYとtarget COM XY速度を持つ。
- nominal COM XYとの差分からdesired velocityを作る。
- `wbmsComVelocityLimit.head<2>()` で速度制限する。
- `wbmsComAccelerationLimit.head<2>()` で速度変化を制限する。
- 制限後の速度を積分してtarget COM XYを更新する。

### 4.4 COM Z

戻さない対象:

- COM Z

保持する高さ:

```text
heldRobotComHeightInFootMid
  = (footMidCoords.inverse() * currentRobotCom).z
```

snapshot時点:

- `startWbmsWalkingPreparation()` service呼び出し直後ではなく、次の `onExecute()` 冒頭でpreparation controllerが `REQUESTED` を処理する時点。
- service内では `footMidCoords`、`genRobot`、`wbmsProjectedRobotCom` が同一制御周期で確定しているとは限らないため。

`currentRobotCom` の優先順位:

1. finiteかつvalidな `wbmsProjectedRobotCom`
2. `genRobot->centerOfMass()`
3. どちらも使えなければ準備開始失敗

寿命:

- `READY` 後の歩行中も `WALKING_HOLD` として保持する。
- `goStop()` 後にstaticへ戻る、`cancelWbmsWalkingPreparation()`、`stopWholeBodyMasterSlave()`、`stopAutoBalancer()`、`onActivated()`、`onDeactivated()`、`MODE_SYNC_TO_ABC` 初期化でclearする。

### 4.5 `refdz`、`l.z`、`omega`、`genCog.z` の整合

`RefToGenFrameConverter::convertFrame()` は毎周期 `refRobotRaw` から `refdz` を再生成する。そのため、COM Z保持は `WbmsPostureControl` 内部だけでは不十分である。

更新順は次を維持する。

```text
RefToGenFrameConverter::convertFrame()
  nominal refdzを生成
↓
applyWalkingComHeightHoldToReference()
  refdz = heldRobotComHeightInFootMid
  l.z = refdz
  omega = sqrt(g / refdz)
↓
LegCoordsGenerator::calcCOMCoords()
  nominal genCog/genCogVel/genCogAccを生成
↓
applyWalkingComHeightHoldToGenCog()
  genCog.z/genCogVel.z/genCogAcc.zを速度・加速度limit付きで更新
↓
WbmsPostureControl::proc()
  static COM/ZMP統合、projection、handoff
```

M4.2.1のZMP軌道時間構造を壊さないため、ZMP軌道は時間列を作り直さず、既存どおりstart/goalの平行移動に留める。

## 5. READY条件

READYは、`HANDOFF` phase中に次を一定時間連続して満たした場合のみ成立する。

- applied torso velocityが閾値以下
- applied COM velocityが閾値以下
- pre-walk return target velocityが閾値以下
- CHEST姿勢誤差が `wbmsWalkingPreparationChestErrorEps` 以下
- COM XY誤差が `wbmsWalkingPreparationComXYErrorEps` 以下
- COM Z保持誤差が `wbmsWalkingPreparationComZErrorEps` 以下
- root姿勢誤差が `wbmsWalkingPreparationRootErrorEps` 以下
- `wbmsProjectionCandidateSafe == true`
- final IK後の一周期最大関節変化が `wbmsWalkingPreparationMaxJointDeltaEps` 以下
- handoff係数が完了
- timeoutしていない
- 非finiteでない

READY成立後も歩行APIを自動実行しない。ユーザーが明示的に `goVelocity` / `goPos` / `setFootSteps` / `setFootStepsWithParam` を再度呼んだときだけ受け付ける。

## 6. 腕操作維持

歩行準備や歩行開始時に次を行ってはならない。

- 腕EE指令をclearしない。
- `wbmsMode` を停止しない。
- 全関節角をWBMS開始時qへ戻さない。

歩行中は次を許容する。

- 体幹roll/pitch/yaw操作を無効化する。
- COM X/Y操作を無効化する。
- 腕EE操作は継続する。

上半身EE拘束は、既存のCHEST相対拘束を維持する。

```text
WBMS有効:
  B_link = torsoGenLink
  B_localpos = torsoRefLink->T().inverse() * abcEETargetPose[i]
```

そのため、`wbmsMode` を歩行準備や歩行開始時に停止してはならない。

## 7. 実装単位

### 7.1 Commit 1: service APIと歩行API gate

変更ファイル候補:

- `auto_stabilizer/idl/AutoStabilizerService.idl`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizerService_impl.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizerService_impl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.h`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp`
- `auto_stabilizer/euslisp/auto-stabilizer-interface.l`

作業内容:

- 専用serviceをIDLへ追加する。
- C++ service実装を追加する。
- `WbmsWalkingCommandDelay` をpreparation controllerへ置き換える、または責務をpendingなしのpreparation管理へ変更する。
- `goVelocity` / `goPos` / `setFootSteps` / `setFootStepsWithParam` の入口でWBMS中未READY gateを追加する。
- 未READY中はpending保存せず `false` を返す。
- WBMS外とREADY後の既存歩行API挙動を維持する。

### 7.2 Commit 2: 速度・加速度制限型pre-walk姿勢生成

変更ファイル候補:

- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`

作業内容:

- 固定秒return alphaを主制御から外す。
- CHEST RPY targetとtarget velocityを状態量として持つ。
- COM XY targetとtarget velocityを状態量として持つ。
- 速度・加速度limitで毎周期pre-walk targetを更新する。
- COM Zは `heldRobotComHeightInFootMid` を保持する。
- `refdz`、`l.z`、`omega`、`genCog.z` の更新順を明示的に保つ。
- READY条件を速度・誤差・safe candidate・joint step・handoff完了で判定する。

### 7.3 Commit 3: シミュレータ検証記録・parameter調整

変更ファイル候補:

- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
- `auto_stabilizer/docs/WBMSWalkingPreparationDesignRevisionPlan.md`
- 必要なら検証ログの記録文書

作業内容:

- 既存debug indexを壊さず、末尾追加のみ行う。
- 追加debug候補:
  - walking API reject event/count
  - walking API accepted event
  - service start/cancel event
  - return target CHEST RPY
  - return target CHEST RPY velocity
  - return target COM XY
  - return target COM XY velocity
  - READY判定boolean各項目
- 同条件のシミュレータ検証を実施する。
- 未READY呼び出しでfootstep数、goVelocity mode、ZMP時間列が変わらないことを確認する。
- READY後の `goVelocity(0.0, 0.0, 0.0)` が既存どおり足踏み開始することを確認する。

## 8. build・確認コマンド

IDL変更後の初回build:

```bash
catkin build auto_stabilizer --no-deps --force-cmake
```

それ以外:

```bash
catkin build auto_stabilizer --no-deps
```

静的確認:

```bash
rg -n "storeGoVelocity|storeGoPos|storeFootSteps|releasePendingCommand|shouldDelay" auto_stabilizer/rtc/AutoStabilizer
rg -n "startWbmsWalkingPreparation|cancelWbmsWalkingPreparation|getWbmsWalkingPreparationState" auto_stabilizer
```

動作確認:

- WBMS外 `goVelocity(0.0, 0.0, 0.0)` が `true`。
- WBMS中未READYの `goVelocity(0.0, 0.0, 0.0)` が `false`。
- 未READY reject時にfootstep列、goVelocity mode、pending commandが変化しない。
- `startWbmsWalkingPreparation()` 後、READYになるまで歩行APIがrejectされる。
- READY後の `goVelocity(0.0, 0.0, 0.0)` が `true` で、既存どおり足踏み開始する。
- READYになっただけでは自動歩行開始しない。
- COM ZがWBMS開始時高さではなく、準備開始時のfootMid基準COM高さを保持する。
- 腕EE指令と `wbmsMode` が歩行準備・歩行開始でclearされない。

## 9. review観点

- 未READY中の歩行API rejectが、footstep生成、goVelocity mode開始、pending保存の前に行われていること。
- WBMS外の歩行API挙動が変わっていないこと。
- READY後の `goVelocity(0.0, 0.0, 0.0)` による足踏み開始が禁止されていないこと。
- `startWbmsWalkingPreparation()` が歩行指令を保存しないこと。
- READY成立だけで歩行が自動開始しないこと。
- timeout / cancel / stopWBMS / stopAutoBalancer / deactivate で状態と保持COM高さがclearされること。
- COM ZをWBMS開始時高さへ戻していないこと。
- 全関節角をWBMS開始時qへ戻していないこと。
- 腕EE指令をclearしていないこと。
- `wbmsMode` を停止していないこと。
- M4.2のsafe candidate採用条件を壊していないこと。
- M4.2.1のZMP軌道時間構造を壊していないこと。
- 500 Hz経路で不要なclone/new/追加IK solveを増やしていないこと。
- debug indexは既存部分を維持し、追加分は末尾に限定していること。

## 10. 実装前に人間が判断すべき点

実装前に次を確認する。

- IDL enum名とstruct名をこの文書の案で確定してよいか。
- `wbms_walking_preparation_return_time` を互換目的で残し、pre-walk姿勢生成では未使用扱いにしてよいか。
- `WbmsWalkingCommandDelay` をファイル名ごと `WbmsWalkingPreparationController` に変更するか、履歴差分を小さくするため既存ファイル名のまま責務だけ変更するか。
- FAILED後の `startWbmsWalkingPreparation()` 再実行を許可するか。
- 初期limit値は既存 `wbmsComVelocityLimit`、`wbmsComAccelerationLimit`、`wbmsTorsoAngularVelocityLimit`、`wbmsTorsoAngularAccelerationLimit` の流用から始めてよいか。

## 11. 追加検証で判明した修正項目

この章は、M4.2.2設計修正の実装後にシミュレータ検証を進める中で判明した追加修正を扱う。事前に独立した計画書で区切ったタスクではないが、安全上重要であり、次スレッドではこの章を実装起点として扱う。

詳細なログ解析値は `WBMSFeasibleVelocityPostureControlProgress.md` の次の節を参照する。

- `M4.2.2設計修正後 simulatorログ解析記録 2026-07-06 16:42`
- `M4.2.2設計修正後 simulatorログ解析記録 2026-07-06 17:00`
- `M4.2.2 11.3修正後 simulatorログ解析記録 2026-07-06 17:47`
- `M4.2.2 11.3 debug追加後 simulatorログ解析記録 2026-07-06 18:14/18:16/18:17`

### 11.1 文書の役割分担

項目ごとの記録先は次の通りとする。

| 項目 | 記録先 |
|---|---|
| ログ名、試験条件、主要時刻、最大値、PASS/FAIL | `WBMSFeasibleVelocityPostureControlProgress.md` |
| 破綻経路の推定、設計上の原因候補 | 本文書 |
| 修正方針、実装対象、acceptance criteria | 本文書 |
| 実装後のbuild結果、simulator再検証結果 | `WBMSFeasibleVelocityPostureControlProgress.md` |
| 次スレッドで最初に読むべき入口 | 本文書の11章とProgress文書の該当ログ節 |

### 11.2 READY前footstep進行抑制

検証ログ:

```text
auto_stabilizer/log/test_start_walking202607061642*
```

判明した問題:

- `startWbmsWalkingPreparation()` だけでREADY前に `footstepNodesList` が1から11へ増加した。
- support phaseもREADY前に変化し、歩行API accepted前に足踏み相へ入った。
- READY前歩行API gateは効いていたが、通常AutoBalancer経路の `FootStepGenerator::procFootStepNodesList()` と `FootStepGenerator::calcFootSteps()` からの自動footstep生成は抑止できていなかった。

修正方針:

- `REQUESTED`、`DECELERATING`、`RETURNING`、`HANDOFF`、`READY` 中は通常のfootstep時系列更新と自動footstep生成を止める。
- `READY` 後に明示的なwalking APIが受理され、phaseが `WALKING_HOLD` へ移った後だけ、既存のfootstep進行と `goVelocity(0.0, 0.0, 0.0)` 由来の足踏み開始を許可する。
- footstep進行抑制は、歩行API gateとは別の安全条件として扱う。

進捗:

- 実装済み。
- `GaitParam::shouldKeepFootStepsStaticForWbmsWalkingPreparation()` を追加し、該当phase中は `AutoStabilizer::execAutoStabilizer()` で `procFootStepNodesList()` / `calcFootSteps()` を呼ばないようにした。
- `catkin build auto_stabilizer --no-deps` は成功済み。
- `test_start_walking202607061700*` でREADY前に `footstepNodesList.size()` が1のまま保持されることを確認済み。

acceptance criteria:

- `startWbmsWalkingPreparation()` だけではREADY前に `footstepNodesList.size()` が増えない。
- READY前に support phase が変化しない。
- READY前の `goVelocity` / `goPos` / `setFootSteps` はrejectされる。
- READYになっただけではfootstep生成しない。
- READY後に `goVelocity(0.0, 0.0, 0.0)` がacceptedされた場合は、既存どおり足踏み開始する。

### 11.3 RETURNING中の急激な姿勢復帰と腕振動

検証ログ:

```text
auto_stabilizer/log/test_start_walking202607061700*
```

判明した問題:

- READY前footstep生成は解消したが、`startWbmsWalkingPreparation()` 直後から1.3-1.6s付近にCHEST/root/脚関節が急変した。
- `el_q` と `ast_q` はほぼ一致しており、AutoStabilizerの出力段階で既に急な関節角指令になっている。
- 腕姿勢指令なしでも、右腕上腕部を含む腕関節が1.7-3.0sで大きく振動的に変化した。
- pre-walk return targetの速度制限値は0.1rad/s級である一方、最終IK後のCHEST pitch実現速度は3rad/s超まで出ており、target側の速度制限だけでは実ロボットへ送る関節角の安全速度を保証できていない。

原因候補:

- RETURNING開始後、`wbmsOperationModeValue=1.0` のまま `wbmsWalkingStabilityModeValue` が0から1へrampし、final IK内のroot姿勢拘束weightが立ち上がる。
- CHEST/COM return target、root姿勢拘束、COM拘束、上半身EEのCHEST相対拘束が同時に効き、IK解が体幹と腕関節を急激に動かしている。
- 上半身EE拘束はWBMS中にCHEST相対で維持されるため、体幹/rootが急に戻ると、腕指令なしでも腕関節が補償動作を行う。
- READY条件は最終IK後joint deltaを見ているが、RETURNING中の実現CHEST速度や関節一周期差分を安全監視していない。

修正方針:

- root姿勢復帰もpre-walk returnの制御対象として扱い、速度・加速度limitを通す。
- `wbmsWalkingStabilityModeValue` の立ち上げをRETURNING開始直後から無条件に進めない。root姿勢復帰targetと整合するように、phaseまたはroot errorに応じてramp開始条件・速度を制限する。
- RETURNING中も final IK後CHEST実現速度、final IK後COM実現速度、final IK後一周期最大関節差分を安全監視する。
- 安全監視が閾値を超えた場合、READY待ちを継続するだけでなく、必要に応じて `FAILED/UNSAFE` へ落とす。
- 腕EE拘束は「clearしない」方針を維持する。ただし歩行準備中の急な体幹復帰を増幅しないよう、weight ramp、maxError制限、または歩行準備開始時のCHEST相対目標再ラッチを検討する。
- `wbmsWalkingPreparationReturnTorsoAngularVelocity` だけを安全判定に使わず、最終IK後に実際に出る速度と関節差分を制御・判定対象に含める。

次スレッドでの実装候補:

1. `GaitParam` にroot return target姿勢、root return角速度、実現速度監視用debug値を追加する。
2. `WbmsPostureControl` のRETURNING処理で、CHEST/COMに加えてroot姿勢targetを速度・加速度limitで更新する。
3. `wbmsWalkingStabilityModeValue` の目標値またはramp時間を、root return targetの進行と同期させる。
4. `updateWbmsFinalIKDiagnostics()` または同等の経路で、RETURNING中の実現CHEST速度・COM速度・関節差分が閾値を超えた場合にunsafe扱いできるようにする。
5. 腕EE拘束については、まずdebugで歩行準備中の腕EE constraint error / weight / maxErrorを確認できるようにし、weight rampかtarget再ラッチのどちらが必要か判断する。

進捗:

- 実装済み。
- `GaitParam` に `wbmsWalkingPreparationTargetRootR` と `wbmsWalkingPreparationReturnRootAngularVelocity` を追加した。
- 歩行準備snapshot時とRETURNING開始ラッチ時にroot return targetを現在root姿勢へ初期化するようにした。
- RETURNING中にroot return targetをCHEST returnと同じ `wbmsWalkingPreparationTorsoAngularVelocityLimit` / `wbmsWalkingPreparationTorsoAngularAccelerationLimit` で更新するようにした。
- `FullbodyIKSolver` のroot姿勢拘束targetは、RETURNING/HANDOFF/READY中のみ `stTargetRootPose` ではなく制限済みroot return targetを使うようにした。
- RETURNING中は `wbmsWalkingStabilityModeValue` をreturn進行度へ同期し、RETURNING開始直後から無条件に1へrampしないようにした。
- RETURNING中の final IK後CHEST実現角速度、final IK後COM実現速度、final IK後一周期最大関節差分を安全監視し、閾値超過時は `FAILED/UNSAFE` へ落とすようにした。
- debug追加として、`wbmsDebugOut` 末尾にroot return速度、root target、READY条件boolean、final IK安全判定、timeout残り時間を追加した。
- 既存96列の順序は維持した。ただし実装時点で `data.length(126)` に対し、意味を持つ追加列は28列で、末尾2列は未使用の0として出ている。
- `catkin build auto_stabilizer --no-deps` は成功済み。

修正後ログから判明した残課題:

- `test_start_walking202607061747*` では急激な直立復帰と腕振動は大きく改善したが、`wbms_walking_preparation_timeout=6.0` ではREADYにならず `FAILED/TIMEOUT` となった。
- `test_start_walking202607061814*` / `061816*` / `061817*` では `wbms_walking_preparation_timeout=10.0` で同条件を3回試したが、READY後歩行開始直後に急動作が出るケース、滑らかに歩行開始するケース、READYにならずtimeoutするケースが分かれた。
- ログ1 `061814` はREADYには到達したが、READY時 `rootError=0.068002rad`、`wbmsWalkingStabilityModeValue=0.902`、歩行API accepted直後 `wbmsOperationModeValue=0.098` が残り、footstep生成直後に final IK後CHEST角速度 `4.885705rad/s`、COM速度 `2.091680m/s`、`el_q` 一周期最大差分 `0.018394rad` が出た。
- ログ2 `061816` はREADY時 `rootError=0.000111rad`、`wbmsWalkingStabilityModeValue=1.000`、歩行API accepted直後 `wbmsOperationModeValue=0.000` で、accept windowの `el_q` 一周期最大差分は `0.000287rad` に収まった。
- ログ3 `061817` はtimeout直前に `chestError=0.000001rad`、`rootError=0.000033rad` まで収束していたが、`returnRootVelNorm=0.004483`、`returnTorsoVelNorm=0.001274` により `readyReturnRootVelocity=false`、`readyReturnTorsoVelocity=false` のまま `FAILED/TIMEOUT` となった。
- 腕指令なし条件での0.1rad級腕振動は、修正後ログでは再現していない。

残る原因候補:

- READY条件がroot/torso return target速度のゼロ収束へ過剰に依存している。`stTargetRootPose` がStabilizer由来で動き続けるため、姿勢誤差が十分小さくても `returnRootVelocity` が `wbmsWalkingPreparationVelocityEps=1e-3` 以下へ落ちにくい。
- `wbmsWalkingPreparationRootErrorEps=0.08rad` はREADY条件として緩すぎる。ログ1では `rootError=0.068rad` でREADYになり、その後の歩行開始直後に急動作が発生した。
- READY後またはWALKING_HOLD移行後に `wbmsWalkingStabilityModeValue` が1未満だと、既存式により `wbmsOperationModeValue` が再び非ゼロとなり、footstep生成直後のCOM weight/reference blendを通じてfinal IK急変を誘発する。
- HANDOFF/READY中にroot targetと `stTargetRootPose` の同期が不十分な場合、READY直後のroot姿勢拘束target切替やWBMS操作blend解除が歩行開始時の不連続として残る。

acceptance criteria:

- `startWbmsWalkingPreparation()` 直後に、final IK後CHEST実現速度が設定した安全閾値を超えない。
- `el_q` 相当の一周期最大関節差分が、RETURNING全期間で安全閾値以下に収まる。
- 腕指令なし条件で、RARM/LARMの上腕関節が0.1rad級に振動しない。
- 体幹復帰が「急動作の後に低速復帰」の2段階に見えず、速度制限に従った一貫した復帰になる。
- READY前footstep抑制、READY前walking API reject、READY後 `goVelocity(0.0, 0.0, 0.0)` acceptは退行しない。
- COM Z保持、`refdz`、`l.z`、`omega` の整合は維持する。
- 腕EE commandと `wbmsMode` はclearしない。ただし歩行準備中の腕拘束weightやtargetを安全側に調整する場合、その仕様を明示する。

次に必要な修正:

1. READY後およびWALKING_HOLD中は `wbmsOperationModeValue=0.0` を強制する。歩行API accepted後に `wbmsWalkingStabilityModeValue` が1未満でも、WBMS操作blendを復活させない。
2. READY条件からroot return target速度の厳格条件を外す、または専用閾値へ分離する。姿勢誤差、final IK後速度、関節差分が安全ならREADYへ進める方針へ変更する。
3. root READY条件を厳しくする。少なくとも `wbmsWalkingPreparationRootErrorEps` を0.08radより小さくするか、READY条件へ `wbmsWalkingStabilityModeValue > 0.99` 相当を追加する。
4. HANDOFF/READY中もroot targetと `stTargetRootPose` の同期を継続するか、HANDOFF前にroot側が十分settleしてからREADY判定する。
5. `wbmsDebugOut` の `data.length` と実際に意味を持つ列数を一致させ、未使用末尾2列を整理する。

追加acceptance criteria:

- READY後の歩行API accepted直後に `wbmsOperationModeValue` が0である。
- READY時のroot errorが、足踏み開始直後にfinal IK急変を起こさない閾値以下である。
- `wbms_walking_preparation_timeout=10.0`、初期前傾量に多少のばらつきがある条件で、READY到達と歩行開始挙動が再現性を持つ。
- 歩行API accepted直後の `finalIKChestAngVelNorm`、`finalIKComVelNorm`、`el_q` 一周期最大差分が安全閾値以下である。

未確定事項:

- root return用の速度・加速度limitを既存pre-walk torso limitと共用するか、専用parameterを追加するか。
- RETURNING中の安全監視閾値を既存READY閾値から流用するか、専用parameterを追加するか。
- 腕EE拘束の対策をweight ramp、maxError制限、target再ラッチのどれから試すか。
- 実機向け安全閾値を、シミュレータ初期値からどこまで保守的に設定するか。
- root return target速度をREADY条件から外す場合、代替としてどのroot error閾値とfinal IK後速度閾値を採用するか。
