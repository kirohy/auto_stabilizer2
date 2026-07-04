# WBMS歩行準備遷移・重心高さ保持 追加実装計画書

## 1. 文書の位置づけ

本書は、以下の既存文書を補完する追加計画書である。

1. `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`
2. `auto_stabilizer/docs/WBMSProjectionAcceptanceFixImplementationPlan.md`
3. `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md`
4. `auto_stabilizer/docs/WBMSWalkingControlSummary.md`

既存仕様と本書が矛盾する場合、歩行開始遷移、歩行中の腕操作、重心高さ保持については本書を優先する。  
本書は提案Bのtask scaling付き速度QPを対象としない。

---

## 2. 背景

M4.2でsafe candidateを採用できるようになり、M4.2.1でstatic WBMS中のZMP軌道時間構造を維持する修正を行った。

残る問題は、体幹を傾けた状態から`goPos()`、`goVelocity()`、`setFootSteps()`を実行すると、歩行開始前または歩行開始直後に急激な姿勢変更が発生し、跳ねるような挙動になることである。

既存の歩行開始遅延は、時間経過に応じてWBMS姿勢拘束を弱め、通常root姿勢拘束を強める構成である。しかし、以下を保証していない。

- 明示的な体幹復帰軌道
- COM、ZMP、`refdz`、`omega`、`l`の連続なhandoff
- 姿勢復帰完了の状態判定
- 歩行指令投入を次周期へ分離すること
- 歩行開始直前の重心高さの保持

`isProjectionReferenceAllowed()`追加と`wbmsWalkingStabilityStartTime`延長だけでは、上記を満たさないため、最終解とはしない。

### 2.1 実装前調査で確定した現行呼び順

M4.2.2実装前に確認した現行コードの重要な呼び順は次の通りである。後続実装ではこの順序を前提に、必要な挿入点を明示して変更する。

1. service APIの`goPos()`、`goVelocity()`、`setFootStepsWithParam()`で入力検証を行う。
2. static WBMSかつ遅延条件成立時は`WbmsWalkingCommandDelay::store*()`へpending commandを保存する。
3. `WbmsWalkingCommandDelay::startDelay()`が`isWbmsWalkingStartDelay=true`、`wbmsWalkingStartDelayRemainTime=wbmsWalkingStabilityStartTime`を設定し、体幹／COM速度指令をclearする。
4. `onExecute()`では`readInPortData()`、mode/gaitParam更新の後、ABC中なら`WbmsWalkingCommandDelay::proc()`を`execAutoStabilizer()`より前に呼ぶ。
5. `execAutoStabilizer()`では`RefToGenFrameConverter::convertFrame()`が毎周期`refdz`をrefRobotRawのfootMid基準COM高さから再計算する。
6. その後、`ExternalForceHandler`、`LegCoordsGenerator::calcLegCoords()`、`LegCoordsGenerator::calcCOMCoords()`が通常の`refZmpTraj`、`genCoords`、nominal `genCog/genCogVel/genCogAcc`を作る。
7. `WbmsPostureControl::proc()`はStabilizer前に呼ばれ、`wbmsWalkingStabilityModeValue`と`wbmsOperationModeValue`を更新し、投影とstatic COM/ZMP統合を行う。
8. `Stabilizer::execStabilizer()`は`stTargetRootPose`を`refRobot->rootLink()`と`stOffsetRootRpy`から生成する。
9. `FullbodyIKSolver::solveFullbodyIK()`は足、COM、CHEST、root、reference q、腕EE拘束を同じ最終IKで扱う。

現行の不連続は、pending commandのreleaseが固定時間だけで決まり、release周期の冒頭でfootstep列が変わり、その同じ周期でnominal ZMP/COMとIK weightが切り替わることで発生する。`refdz`はframe変換で毎周期上書きされるため、COM高さ保持を`applyStaticComZmpIntegration()`の内部だけに実装しても、歩行中の高さ保持としては不十分である。

### 2.2 rejected trialを採用しない理由

作業ツリーや`M4-2-2.patch`に、次のtrial差分が存在する場合がある。

- `isProjectionReferenceAllowed()`を追加し、`solveProjection()`だけ遅延中に許可する。
- `wbmsWalkingStabilityStartTime`既定値を`2.0`から`5.0`へ延長する。

これは最終解として採用しない。理由は次の通りである。

- `applyStaticComZmpIntegration()`と`updateVelocityCommand()`は依然として`isOperationAllowed()`依存であり、delay中のCOM/ZMP統合、速度減速、明示的RETURNINGを扱えない。
- 固定時間延長はCHEST誤差、COM XY/Z誤差、safe candidate、root誤差、settleを確認しない。
- COM Z保持対象が「歩行指令受付時のfootMid基準COM高さ」になっておらず、歩行開始後に保持する独立経路もない。
- READY成立周期とpending command投入周期が分離されない。
- M4.2.1のZMP軌道時間構造維持やM4.2のsafe candidate採用条件を壊さないための責務分離が不足している。

---

## 3. 要件

### 3.1 必須要件

1. 体幹を傾けたstatic WBMS姿勢から歩行へ遷移しても、急激な姿勢変更や跳ねを発生させない。
2. 歩行準備中は体幹姿勢を歩行可能な基準姿勢へ滑らかに戻す。
3. 歩行準備中に全関節角をWBMS開始時姿勢へ戻してはならない。
4. COM X/Yは通常歩行系が要求する静止時基準へ滑らかにhandoffする。
5. **COM ZはWBMS開始時高さへ戻さず、歩行指令を受けた時点の高さを保持する。**
6. 保持する高さは、world Zではなく、原則として`footMidCoords`基準のロボットCOM高さとする。
7. 保持したCOM高さを歩行開始後にも引き継ぐ。
8. 歩行中は足拘束、関節limit、自己干渉、ZMP／バランス、root姿勢安定化を体幹操縦より優先する。
9. 歩行中は体幹roll/pitch/yaw操縦とCOM X/Y操縦を無効にしてよい。
10. **歩行中も腕のWBMS操作を継続可能にする。**
11. 腕EEは既存どおりCHEST相対拘束を維持する。
12. 歩行準備や歩行開始時に`wbmsMode`を停止しない。
13. 歩行準備開始時に体幹・COM速度指令は加速度limitに従ってゼロへ減速する。
14. 歩行開始は固定タイマだけで決めず、姿勢・速度・安全状態の完了条件を確認する。
15. 完了条件を満たした周期と同じ周期にfootstep列を変更せず、次周期以降にpending commandを投入する。
16. 準備がtimeoutした場合は歩行を強行しない。pending commandをキャンセルまたは保持停止し、診断可能なstatusを残す。
17. M4.2のsafe candidate採用条件、M4.2.1のZMP軌道時間構造維持、安全validationを壊さない。
18. hidden goalとして未達成ユーザー指令を蓄積しない。
19. 500 Hz経路で不要なclone、動的確保、全探索、追加の反復IKを導入しない。

### 3.2 Optional要件

歩行中のCOM高さ操作は必須修正と同時に実装しない。

必須のM4.2.2がシミュレータで安定した後、optional M4.2.3として次を検討する。

- 歩行中に`refTorsoVelIn.vz`だけをCOM高さ速度指令として受け付ける。
- COM X/Yと体幹角速度指令は歩行中無効のままとする。
- 初回実装では両足支持期だけ高さtargetを更新し、片足支持期は保持してよい。
- 足、関節limit、ZMP、root姿勢、歩行安定化を常に優先する。
- 保守的な速度・加速度limitを使用する。
- optional機能が無効でも、M4.2.2の「歩行直前高さ保持」は機能する。

---

## 4. 用語と状態量

### 4.1 保持COM高さ

歩行準備開始時に次を一度だけsnapshotする。

```text
heldRobotComHeightInFootMid
  = (footMidCoords.inverse() * currentRobotCom).z
```

`currentRobotCom`の優先順位は次とする。

1. finiteかつvalidな`wbmsProjectedRobotCom`
2. `genRobot->centerOfMass()`
3. いずれも利用不能なら準備開始失敗

snapshotはservice API呼び出し直後ではなく、次の`onExecute()`冒頭で`WbmsWalkingCommandDelay::proc()`が`REQUESTED`を処理する時点で行う。service API内では`footMidCoords`、`genRobot`、`wbmsProjectedRobotCom`が同一制御周期の確定値とは限らないためである。

この値は`wbmsStartComInFootMid.z`とは別物である。歩行準備中にWBMS開始時高さへ補間してはならない。

### 4.2 通常歩行用COM X/Y

歩行準備開始後、`LegCoordsGenerator::calcCOMCoords()`が生成し、WBMS統合を適用する前のnominal `genCog + sbpOffset`を使用する。

- X/Y: nominal static walking targetへ滑らかに戻す。
- Z: `heldRobotComHeightInFootMid`を維持する。

実装ではrobot COMと`genCog`を混同しないこと。`FullbodyIKSolver`のCOM targetは`genCog + sbpOffset`であるため、footMid基準で作ったrobot COM targetをgenerate frameへ戻した後、`targetGenCog = targetRobotComWorld - sbpOffset`として`genCog`へ反映する。

### 4.3 体幹基準姿勢

体幹の復帰先は既存の`wbmsStartChestRInFootMid`とする。  
root姿勢や全関節角をWBMS開始時値へ直接戻す方式は採用しない。

上半身EE拘束は`wbmsMode`が有効である間、`FullbodyIKSolver`で`B_link() = torsoGenLink`、`B_localpos() = torsoRefLink->T().inverse() * abcEETargetPose[i]`としてCHEST相対になる。したがって歩行開始時に`wbmsMode`を停止してはならない。

---

## 5. 状態機械

既存の`isWbmsWalkingStartDelay`は互換用の集約フラグとして残してよいが、内部判断は明示的なphase enumを使用する。

推奨phase:

```text
INACTIVE
REQUESTED
DECELERATING
RETURNING
HANDOFF
READY
WALKING_HOLD
FAILED
```

### 5.1 INACTIVE

通常のstatic WBMSまたは通常歩行状態。

### 5.2 REQUESTED

`goPos()`、`goVelocity()`、`setFootSteps()`を保留した直後。

次の制御周期で以下をsnapshotする。

- 現在CHEST姿勢
- 現在ロボットCOM
- 保持COM高さ
- nominal COM X/Y
- 必要なら現在root姿勢
- 準備開始時刻

snapshot完了後に`DECELERATING`へ遷移する。

### 5.3 DECELERATING

- raw torso/COM commandをゼロ扱いにする。
- applied commandは既存加速度limitでゼロへ減速する。
- projector、CHEST、COM、static COM/ZMP統合は有効のまま維持する。
- walking stability weightをまだ増加させない。
- 腕操作は継続する。

applied torso/COM速度が閾値以下になったら`RETURNING`へ遷移する。

### 5.4 RETURNING

明示的なtask-space復帰targetを生成する。

- CHEST: snapshot姿勢から`wbmsStartChestRInFootMid`へ補間
- COM X/Y: snapshot COM X/Yからnominal static walking COM X/Yへ補間
- COM Z: `heldRobotComHeightInFootMid`で固定
- 足:現在の両足targetを維持
- 関節角:projectorが安全制約下で決定
- 全関節をWBMS開始時qへ補間しない

このphaseではWBMS projectorとCOM/ZMP統合を有効に保つ。

### 5.5 HANDOFF

RETURNINGのtargetを保持した状態で、制御権限を滑らかに通常歩行側へ渡す。

- WBMS CHEST姿勢weightを0へ
- 投影下半身reference angle blendを0へ
- 通常root姿勢weightを歩行値へ
- 通常COM X/Y／ZMP制御を有効化
- COM Z高さ保持は独立経路として維持
- 体幹targetを動かしながらweightを消さない

### 5.6 READY

以下を一定時間連続して満たした状態。

- applied torso/COM速度が閾値以下
- CHEST基準姿勢誤差が閾値以下
- COM X/Y nominal誤差が閾値以下
- COM Zの保持高さ誤差が閾値以下
- projector candidateがsafe
- root姿勢誤差が許容範囲
- 対象関節の一周期変化が許容範囲
- handoff係数が完了
- timeoutしていない

READYになった周期ではpending commandを投入しない。次の`onExecute()`冒頭で投入する。

実装上は、READY成立周期では`releaseRequested`相当の1周期遅延フラグだけを立てる。`WbmsWalkingCommandDelay::proc()`は`execAutoStabilizer()`より前に呼ばれるため、次周期冒頭でこのフラグを見てpending commandを投入すれば、READY判定とfootstep列変更を別周期に分離できる。

### 5.7 WALKING_HOLD

footstep列投入後。

- 体幹角速度、COM X/Y速度操縦は無効
- projectorのstatic体幹／下半身姿勢操縦は無効
- 通常歩行バランス制御を優先
- 腕WBMS操作は有効
- COM Zは`heldRobotComHeightInFootMid`を基準として維持
- optional M4.2.3未実装時は高さtargetを変更しない

### 5.8 FAILED

- snapshot失敗
- candidate unsafe継続
- readiness timeout
- 非finite
- その他の安全条件違反

FAILEDでは歩行指令を強行しない。診断statusを残し、static状態を維持する。

`goStop()`、`stopWholeBodyMasterSlave()`、`stopAutoBalancer()`、`MODE_SYNC_TO_ABC`初期化、`onActivated()`、`onDeactivated()`では、pending command、phase、release要求、保持COM高さをclearする。現行`stopWholeBodyMasterSlave()`はpending delayをclearしないため、M4.2.2で追加が必要である。

---

## 6. 制御許可条件の分離

単一の`isOperationAllowed()`へすべての責務を持たせない。

最低限、次を分離する。

```text
isVelocityCommandAllowed()
isPostureProjectionAllowed()
isStaticComZmpIntegrationAllowed()
isWalkingComHeightHoldAllowed()
```

各許可条件は次の責務だけを持つ。単一の`isOperationAllowed()`を流用して、速度指令、projector、COM/ZMP統合、歩行中高さ保持をまとめて停止してはならない。

- `isVelocityCommandAllowed()`: static WBMS中のみ外部の体幹／COM速度指令を受理する。準備中と歩行中はraw指令をゼロ扱いし、applied commandは加速度limitでゼロへ戻す。
- `isPostureProjectionAllowed()`: static WBMSと歩行準備中のtask-space RETURNINGに使う。歩行中の体幹姿勢操作とCOM XY操作には使わない。
- `isStaticComZmpIntegrationAllowed()`: static WBMSと歩行準備中だけ、投影COMを`genCog/refZmpTraj`へ接続する。
- `isWalkingComHeightHoldAllowed()`: 歩行準備中、READY後、歩行中にCOM Z保持だけを独立して許可する。projector valid flagや`wbmsOperationModeValue`だけに依存させない。

推奨表:

| 状態 | torso/COM速度指令 | projector | static COM/ZMP統合 | COM高さ保持 | 腕 |
|---|---:|---:|---:|---:|---:|
| static WBMS | 有効 | 有効 | 有効 | WBMS投影値 | 有効 |
| REQUESTED/DECELERATING | ゼロへ減速 | 有効 | 有効 | snapshot値 | 有効 |
| RETURNING | 無効 | 有効 | 有効 | snapshot値 | 有効 |
| HANDOFF | 無効 | target保持 | blend | snapshot値 | 有効 |
| WALKING_HOLD | 無効 | 無効 | 無効 | 有効 | 有効 |
| FAILED | 無効 | fallback | 安全側 | current保持 | 有効または安全側 |

---

## 7. COM高さ保持経路

### 7.1 必須動作

歩行中のCOM高さ保持は、体幹projectorのvalid flagや`wbmsOperationModeValue`だけに依存させない。

保持対象:

- `genCog.z`
- `refdz`
- `l.z`
- `omega`
- 必要な`genCogVel.z`
- 必要な`genCogAcc.z`

XYは通常歩行生成・ZMP制御へ任せる。

### 7.2 連続性

- `refdz`はframe変換で毎周期生成されるため、保持値を適用する処理順を明示する。
- `l.z`と`omega`は同じ周期で整合させる。
- `effectiveGravity = omega^2 * refdz`を維持する既存方針を再利用してよい。
- `genCog.z`を一周期で上書きして飛ばさず、速度・加速度limitまたは連続な補間を適用する。
- M4.2.1の`refZmpTraj`時間構造を維持する。
- 歩行中のZMP X/YへWBMS offsetを強制しない。

### 7.3 実装順序

COM高さ保持の更新順は次を標準とする。

1. `RefToGenFrameConverter::convertFrame()`は従来通り`refdz`と`footMidCoords`を更新する。
2. M4.2.2の高さ保持が有効なら、直後に`heldRobotComHeightInFootMid`から`refdz`、`l.z`、`omega`を再整合する。
3. `ExternalForceHandler`と`LegCoordsGenerator::calcCOMCoords()`は、再整合済みの`refdz/l/omega`を使う。
4. `LegCoordsGenerator::calcCOMCoords()`がnominal `genCog`を生成した後、COM Z保持が有効なら`genCog.z`、`genCogVel.z`、`genCogAcc.z`を速度・加速度limit付きで補正する。
5. `WbmsPostureControl::applyStaticComZmpIntegration()`はstatic WBMSまたは歩行準備中だけXYを含むWBMS投影統合を行い、歩行中はZ保持経路へ責務を渡す。

`refdz = max(heldRobotComHeightInFootMid, 0.1)`、`l.z = refdz`を同一周期で更新する。`omega`は`effectiveGravity = oldOmega * oldOmega * max(oldRefdz, 0.1)`を用いて`omega = sqrt(effectiveGravity / max(refdz, 0.1))`とし、既存の有効重力方針を維持する。

`refZmpTraj`を補正する場合は、M4.2.1と同じく既存segmentの時間を変えず、start/goalだけを必要最小限に平行移動する。歩行中のCOM Z保持ではZMP X/YへWBMS offsetを加えない。

---

## 8. 腕操作

必須要件:

- `wbmsMode`は歩行準備・歩行中も有効のままとする。
- `RefToGenFrameConverter`の上半身EE差分変換を停止しない。
- `FullbodyIKSolver`の上半身EEはCHEST相対拘束を維持する。
- 歩行準備開始時にclearするのは体幹／COM速度指令だけであり、腕EE指令をclearしない。
- 歩行中は足、関節安全、自己干渉、バランスが上位優先度である。
- 腕指令が実現不能な場合は上位制約を優先し、下半身バランスを崩して追従しない。

---

## 9. 作業単位

細分化を増やしすぎないため、正式な必須マイルストーン番号は`M4.2.2`だけとする。  
内部作業は番号ではなくWork Packageで管理する。

### Work Package A: 遷移診断

目的: 挙動を変えず、遷移不連続を測定可能にする。

変更対象は原則として`GaitParam.h`、`AutoStabilizer.cpp`、必要最小限の`FullbodyIKSolver.cpp`とする。制御挙動、weight、許可条件、pending release時刻は変更しない。

実装候補:

- 既存M4.3予定のfinal IK後COM/CHEST realized velocityをindex 40-45へ追加
- transition debugをindex 46以降へ追加
- phase
- elapsed/timeout
- return alpha
- handoff alpha
- held COM height
- current robot COM height
- CHEST error
- COM XY error
- COM Z hold error
- root姿勢error
- 最大一周期関節変化
- pending command release event
- timeout/failure status
- 実行時`wbmsWalkingStabilityStartTime`

制御挙動、weight、許可条件、歩行開始タイミングは変更しない。

完了後:

- build
- `/review`
- Progress更新
- 独立コミット
- 現行問題のbaselineログ取得

### Work Package B: M4.2.2必須遷移本体

目的: 本書3.1をすべて実装する。

実装対象は`GaitParam.h`、`WbmsWalkingCommandDelay.h/.cpp`、`WbmsPostureControl.h/.cpp`、`AutoStabilizer.cpp`を中心とする。`FullbodyIKSolver.cpp`は腕CHEST相対拘束の維持確認と、必要なweight接続に限定する。dependency solver、task scaling、新規テストコードは追加しない。

含むもの:

- phase state machine
- snapshot
- command deceleration
- task-space RETURNING
- COM Z保持
- HANDOFF
- readiness
- timeout
- 次周期release
- walking COM height hold
- 腕操作維持
- reset/cancel処理
- IDL parameterが必要なら追加
- debug更新

関数単位の実装手順:

1. `WbmsWalkingCommandDelay::store*()`はpending保存だけを行い、phaseを`REQUESTED`へ設定する。
2. `WbmsWalkingCommandDelay::proc()`は`REQUESTED`のsnapshot、phase遷移、timeout、READY次周期release、clearを担当する。
3. `WbmsPostureControl::updateVelocityCommand()`は新しい`isVelocityCommandAllowed()`を使い、準備中と歩行中はdesired commandをゼロにする。
4. `WbmsPostureControl::solveProjection()`は新しい`isPostureProjectionAllowed()`を使い、RETURNING/HANDOFF中のCHEST/COM task-space targetを扱う。
5. `WbmsPostureControl::applyStaticComZmpIntegration()`は新しい`isStaticComZmpIntegrationAllowed()`を使い、歩行中の高さ保持とは分離する。
6. `AutoStabilizer::execAutoStabilizer()`内では、`convertFrame()`直後と`calcCOMCoords()`直後にCOM高さ保持処理を挿入する。
7. `goStop()`、`stopWholeBodyMasterSlave()`、`stopAutoBalancer()`、sync初期化、activation/deactivationでphaseとpendingをclearする。

含まないもの:

- 歩行中COM高さ速度操作
- task scaling
- solver dependency変更
- 歩行中体幹姿勢操作
- 全関節のWBMS開始姿勢復帰

完了後:

- force-cmakeが必要なら実施
- `/review`
- Progress更新
- 独立コミット
- シミュレータ試験

### Validation Package

新しい正式マイルストーン番号は付けない。M4.2.2の検証記録として扱う。

- 直立高さ＋体幹前傾から歩行
- 低いCOM＋体幹前傾から歩行
- COM高さだけ変更して歩行
- 腕静止／腕操作中の歩行
- `goPos`
- `goVelocity`
- `setFootSteps`
- timeout／cancel
- 歩行停止後のstatic復帰
- 500 Hz時間統計

不具合修正が必要な場合だけ、原因単位でfix commitを作る。

### Optional M4.2.3: 歩行中COM高さ操作

M4.2.2がPASSするまで開始しない。

初回範囲:

- 歩行中は`vz`のみ受理
- X/Yと体幹角速度は無効
- 両足支持期のみ高さtarget更新可
- 片足支持期はtarget保持
- 保守的速度・加速度limit
- 高さ上下限
- balance優先
- 腕操作維持
- 専用debugとacceptance

---

## 10. 想定変更ファイル

- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsWalkingCommandDelay.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`
- 必要なら`AutoStabilizerService.idl`
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md`

変更前に実際の呼び順と責務を確認し、不要なファイルは変更しない。

IDL変更は必須ではない。Work Package AではIDL変更しない。Work Package Bでは`wbms_walking_preparation_timeout`と`wbms_walking_preparation_settle_time`が外部調整に必要な場合だけ追加する。IDLを変更した場合、初回buildは`catkin build auto_stabilizer --no-deps --force-cmake`とする。

---

## 11. Debug index方針

既存index 0-39は変更しない。

### 11.1 既存M4.3予約

| index | 内容 |
|---:|---|
| 40-42 | final IK後COM realized velocity |
| 43-45 | final IK後CHEST realized angular velocity |

### 11.2 M4.2.2候補

| index | 内容 |
|---:|---|
| 46 | walking preparation phase |
| 47 | phase elapsed time |
| 48 | return alpha |
| 49 | handoff alpha |
| 50 | held robot COM height in footMid |
| 51 | current robot COM height in footMid |
| 52 | CHEST orientation error |
| 53 | COM XY error |
| 54 | COM Z hold error |
| 55 | root orientation error |
| 56 | max joint delta per cycle |
| 57 | pending command release event |
| 58 | preparation timeout/failure code |
| 59 | runtime walking stability start time |

最終indexは実装前調査で確定する。既存indexを再利用・並べ替えしない。

---

## 12. 完了条件

### 12.1 Work Package A

- build成功
- index 0-39不変
- 40-45がprojector出力ではなくfinal IK後実現量を表す
- transition debugがfinite
- 制御挙動に意図的変更なし
- baselineログ取得可能

### 12.2 M4.2.2必須

- 体幹前傾状態から歩行しても急激な跳ねがない
- 体幹は基準姿勢へ速度制限付きで戻る
- COM ZはWBMS開始時値へ戻らない
- 歩行指令受付時のCOM高さが歩行開始後も維持される
- COM X/Yは通常歩行系へ連続にhandoffする
- `refdz`、`l.z`、`omega`が非finite・不連続にならない
- `refZmpTraj`総時間が正
- readiness前にpending commandを投入しない
- READY周期とrelease周期が分離される
- timeout時に歩行を強行しない
- 歩行中の体幹／COM X/Y操縦は無効
- 歩行中も腕指令に応答する
- 腕はCHEST相対
- 足／安全／バランスが腕より優先
- 歩行停止後に古い体幹／COM速度指令が再開しない
- hidden goalなし
- 500 Hzで許容可能
- build成功
- reviewの重大指摘解消

静的確認項目:

- `isOperationAllowed()`が速度指令、projector、COM/ZMP統合、歩行中高さ保持の全責務をまとめて担っていない。
- `isProjectionReferenceAllowed()`追加と`wbmsWalkingStabilityStartTime`延長だけのtrial解になっていない。
- 歩行開始経路で`wbmsMode.setGoal(0.0, ...)`を呼ばない。
- 上半身EEのCHEST相対拘束が`wbmsActive`中に維持される。
- `refZmpTraj`の既存segment時間を理由なく変更しない。
- 500 Hz経路でclone、追加FK/COM計算の乱用、vectorの無制限蓄積を入れない。

### 12.3 Optional M4.2.3

- M4.2.2の挙動を退行させない
- 歩行中`vz`だけが高さtargetを変更する
- 片足支持期で急激な高さ変化を起こさない
- 腕操作を維持する
- balance、安全制約を優先する
- 無効時はM4.2.2の固定高さ保持と同じ

---

## 13. コミット方針

推奨:

```text
Add WBMS walking transition diagnostics
Implement WBMS walking preparation transition
Record WBMS walking transition simulator results
```

optional:

```text
Add optional WBMS walking COM height control
```

計画書は実装前に独立コミットする。

```text
Add WBMS walking preparation transition plan
```

---

## 14. 禁止事項

- rejected trial patchをそのまま最終解として採用しない
- 時間を5秒へ延長するだけで完了扱いしない
- weight補間だけで姿勢復帰完了扱いしない
- COM Zを`wbmsStartComInFootMid.z`へ戻さない
- 全関節をWBMS開始時qへ戻さない
- walking開始時に`wbmsMode`を停止しない
- 腕EE commandをclearしない
- M4.2.2とoptional M4.2.3を同時実装しない
- dependency solverを変更しない
- task scalingを実装しない
- validation閾値を理由なく緩和しない
- `clang-format`を実行しない
- 新規テストコードを追加しない
- 無関係なユーザー変更を削除・巻き戻ししない

---

## 15. 実装前の注意事項と既知の矛盾

### 15.1 正式仕様として扱う文書

実装時は本書に加え、次の文書を正式仕様として読むこと。

1. `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`
2. `auto_stabilizer/docs/WBMSProjectionAcceptanceFixImplementationPlan.md`
3. `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md`
4. `auto_stabilizer/docs/WBMSWalkingControlSummary.md`

ただし、歩行開始遷移、歩行中の腕操作、COM高さ保持について矛盾がある場合は本書を優先する。

### 15.2 working tree上のtrial差分

実装開始時のworking treeに、`isProjectionReferenceAllowed()`追加や`wbmsWalkingStabilityStartTime`既定値延長のtrial差分、または同内容の`M4-2-2.patch`が残っている可能性がある。これは調査用のrejected trialであり、そのまま採用しない。実装時は、ユーザー変更を勝手に削除せず、必要なら別patchとして退避したうえで、本書のphase state machineに置き換える。

### 15.3 現時点で実装不能ではないが注意が必要な点

- `RefToGenFrameConverter`が毎周期`refdz`を上書きするため、COM高さ保持は`convertFrame()`後の再整合を含めないと成立しない。
- `WbmsWalkingCommandDelay::proc()`は`execAutoStabilizer()`より前に呼ばれるため、同周期でREADY判定して同周期releaseする設計は不可能である。READYは後段で確定し、releaseは次周期冒頭に行う。
- `FullbodyIKSolver`の腕CHEST相対拘束は`wbmsMode`またはそのgoalが有効であることに依存する。歩行開始時にWBMSを停止すると腕操作維持要件と矛盾する。
- `stopWholeBodyMasterSlave()`は現行ではpending walking delayをclearしない。M4.2.2では古いpending commandやheld heightを残さないためclear経路に含める。
- `WBMSWalkingControlSummary.md`が未追跡ファイルであっても、タスク指示で正式仕様として指定された場合は読むこと。未追跡であることだけを理由に無視しない。
