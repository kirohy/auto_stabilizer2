# WBMS投影候補採用判定修正・診断拡張 実装計画

## 1. 文書の位置づけ

本書は、`auto_stabilizer2` の `wbms-dev` ブランチに実装済みの
「WBMS実現可能速度投影型・体幹／COM操縦」に対し、シミュレータで確認された
`projector valid == 0` 問題を修正するための追加実装計画である。

対象は提案Aのみとする。

- 現行の2段構成を維持する。
  - 第1段: `WbmsPostureControl` による1周期先の実現可能姿勢投影。
  - 第2段: `FullbodyIKSolver` による最終全身IK。
- `solveIKLoop()` のbool戻り値を安全性判定として扱わない。
- 有限性、関節limit、足拘束誤差、root変位などを独立検証し、安全な1ステップ候補を採用する。
- 投影不能と、制約によって速度がほぼゼロになるblocked状態を区別する。
- projectorから最終IKまでの伝達を観測可能にする。

提案Bのtask scaling付き速度QPへの移行は、本書の範囲外である。

参照順の補足:

- 本書は `M4.1` から `M4.2` 系のprojector候補採用判定修正計画である。
- 歩行準備遷移は `M4.2.2` 系として `WBMSWalkingPreparationTransitionImplementationPlan.md` と `WBMSWalkingPreparationDesignRevisionPlan.md` を参照する。
- 計算量削減は本書の範囲外であり、`M5: 500 Hz計算量削減` として `WBMSComputationReductionImplementationPlan.md` を参照する。

---

## 2. 対象リポジトリ

- repository: `kirohy/auto_stabilizer2`
- branch: `wbms-dev`
- package: `auto_stabilizer`
- main directory: `auto_stabilizer/rtc/AutoStabilizer`

既存文書:

- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlImplementationPlan.md`
- `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md`

`WBMSFeasibleVelocityPostureControlImplementationPlan.md` は初期M1からM3の基礎仕様として参照する。projector候補採用判定、safe candidate、`solveIKLoop()` 戻り値の扱いについては、本書を優先する。

本書をリポジトリへ配置する場合の推奨パス:

```text
auto_stabilizer/docs/WBMSProjectionAcceptanceFixImplementationPlan.md
```

---

## 3. 確認された現象

`wbmsDebugOut`ログでは、静止WBMS中にpitch角速度指令を送った際に以下が確認された。

- raw torso pitch angular velocityは`0.1 rad/s`。
- applied torso pitch angular velocityは加速度limitに従って`0.1 rad/s`へ到達。
- `wbmsOperationModeValue`は最終的に`1.0`。
- `wbmsPostureReferenceValid`は全周期`false`。
- `wbmsRealizedTorsoAngularVelocity`は全周期ゼロ。
- CHEST姿勢拘束、投影reference q、投影COM統合が最終IKへ接続されない。
- projector計算時間は500 Hz経路として許容可能な範囲に見えるが、機能が無効化されている。

入力保持、timeout、加速度limit、operation modeまでは動作している。
直接の問題は入力ではなく、投影候補の採用条件にある。

---

## 4. 根本原因

現行`WbmsPostureControl::solveProjection()`は概念的に以下となっている。

```cpp
bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(...);

wbmsPostureRobot_->calcForwardKinematics();
wbmsPostureRobot_->calcCenterOfMass();

bool valid = solved && validateProjection(gaitParam, dt);
if(!valid){
  setFallbackReference(gaitParam);
  return false;
}
```

しかし、`solveIKLoop()`のboolは「QP計算が実行され、有限な1ステップ候補が得られたか」ではなく、
最終的に対象constraint群が`isSatisfied()`になったかを表す。

現行projectorは以下の設計である。

- `maxIteration = 1`
- 1周期先の局所目標だけを生成
- CHEST、COM、足、関節安全、姿勢参照を同時に扱う
- 各taskは1回の線形化・QP更新で完全一致することを前提としていない
- `precision = 0.0`を用いるconstraintがある

そのため、安全で有効な小さな1ステップが生成されても、全constraintが1回で完全充足しない限り
`solveIKLoop()`はfalseになり得る。現行コードはその候補を無条件に破棄している。

本修正ではboolの変数名を`allConstraintsSatisfied`とし、診断値としてのみ保持する。
安全性と採否は独立したcandidate validationで決定する。

---

## 5. 設計原則

### 5.1 戻り値の意味を明確にする

禁止:

```cpp
bool solved = solveIKLoop(...);
bool valid = solved && validateProjection(...);
```

修正方針:

```cpp
bool allConstraintsSatisfied = solveIKLoop(...);
ProjectionValidationResult validation = validateProjectionCandidate(...);

if(!validation.safe){
  fallback();
  return false;
}

acceptCandidate();
classifyActiveOrBlocked();
return true;
```

`allConstraintsSatisfied`は以下に使う。

- debug出力
- solver／constraint調整時の観測
- 安全候補が生成されているが完全充足していない状態の識別

採用可否には直接使わない。

### 5.2 安全な候補と完全充足を分離する

candidateを採用できる最低条件:

- root、CHEST、COM、対象関節がfinite。
- 全対象関節がjoint limit内。
- 1周期の関節変位が異常値でない。
- root並進・回転変位が異常値でない。
- 両足の位置・姿勢誤差が安全閾値内。
- support hullとtarget生成が有効。
- projected CHEST／COMとrealized velocityがfinite。

candidateが安全であれば、全taskの完全充足は要求しない。

### 5.3 hidden goalを作らない

従来どおり毎周期、現在の`genRobot`から開始する。

- 未達成姿勢目標を次周期へ累積しない。
- blocked時に内部目標を積み上げない。
- 逆方向指令へ即時に反応できる構造を維持する。

### 5.4 INVALID、BLOCKED、ACTIVEを区別する

- `INVALID`
  - 数値異常、limit違反、足誤差過大、root変位過大、support hull不正など。
  - candidateを破棄してfallbackする。
  - `wbmsPostureReferenceValid = false`。
- `VALID_BLOCKED`
  - candidateは安全。
  - 非ゼロ指令に対する実現速度が極小。
  - candidateを採用しても現在姿勢とほぼ同じ。
  - hidden goalは持たない。
  - `wbmsPostureReferenceValid = true`。
- `VALID_ACTIVE`
  - candidateは安全。
  - 指令方向へ有限な実現速度がある。
  - `wbmsPostureReferenceValid = true`。
- `VALID_IDLE`
  - 指令がほぼゼロでcandidateも静止。
  - `wbmsPostureReferenceValid = true`としてもよいが、既存挙動との整合を確認する。

### 5.5 既存安全閾値を初回修正で緩和しない

初回修正では、現行`validateProjection()`の以下の閾値を維持する。

- root translation: `0.20 m`
- root rotation: `0.50 rad`
- foot translation error: `0.02 m`
- foot rotation error: `0.10 rad`
- joint limit margin: 現行limit計算
- joint one-cycle step: 現行判定

まず採用条件の誤りだけを修正する。
閾値調整は新しいdebugログを取得した後に別変更として行う。

---

## 6. ステータス設計

`GaitParam`に、制御判断ではなくdebug／状態公開用として以下に相当するenumを追加する。

実際の配置は現行コーディング規約へ合わせてよいが、値の意味を固定する。

```cpp
enum WbmsProjectionStatus {
  WBMS_PROJECTION_NOT_RUN = 0,
  WBMS_PROJECTION_DISABLED = 1,
  WBMS_PROJECTION_BASELINE_INVALID = 2,
  WBMS_PROJECTION_NO_VARIABLE = 3,
  WBMS_PROJECTION_TARGET_INVALID = 4,
  WBMS_PROJECTION_SUPPORT_HULL_INVALID = 5,

  WBMS_PROJECTION_INVALID_NONFINITE = 10,
  WBMS_PROJECTION_INVALID_JOINT_LIMIT = 11,
  WBMS_PROJECTION_INVALID_JOINT_STEP = 12,
  WBMS_PROJECTION_INVALID_ROOT_TRANSLATION = 13,
  WBMS_PROJECTION_INVALID_ROOT_ROTATION = 14,
  WBMS_PROJECTION_INVALID_FOOT_POSITION = 15,
  WBMS_PROJECTION_INVALID_FOOT_ROTATION = 16,

  WBMS_PROJECTION_VALID_IDLE = 20,
  WBMS_PROJECTION_VALID_ACTIVE = 21,
  WBMS_PROJECTION_VALID_BLOCKED = 22
};
```

必要に応じて、target invalidとsupport hull invalidを実装上分離できない場合は、
最初の実装で同一statusとしてもよい。ただし進捗記録に差異を明記する。

---

## 7. ValidationResult設計

`WbmsPostureControl`内部へ、以下に相当するvalidation結果を追加する。

```cpp
struct ProjectionValidationResult {
  bool safe;
  GaitParam::WbmsProjectionStatus status;

  double rootTranslationStep;
  double rootRotationStep;
  double maxJointStep;
  double minJointLimitMargin;
  double maxFootPositionError;
  double maxFootRotationError;
};
```

初期化時は必ずfiniteな値を設定する。

推奨初期値:

```cpp
safe = false;
status = WBMS_PROJECTION_NOT_RUN;
rootTranslationStep = 0.0;
rootRotationStep = 0.0;
maxJointStep = 0.0;
minJointLimitMargin = std::numeric_limits<double>::max();
maxFootPositionError = 0.0;
maxFootRotationError = 0.0;
```

`validateProjection()`はboolではなく、この結果を返す
`validateProjectionCandidate()`相当へ変更する。

---

## 8. candidate採用フロー

`WbmsPostureControl::solveProjection()`を以下の順序へ変更する。

### 8.1 周期開始

1. debug statusを`NOT_RUN`へ初期化。
2. solverの全constraint充足フラグをfalseへ初期化。
3. validation metricsをゼロまたは安全な初期値へ初期化。
4. `syncProjectionRobot()`を実行。

### 8.2 操作可否とtarget生成

以下はcandidate生成前の失敗としてstatusを設定する。

- operation不可: `DISABLED`
- baseline invalid: `BASELINE_INVALID`
- variableなし: `NO_VARIABLE`
- support hull不正: `SUPPORT_HULL_INVALID`
- その他target生成失敗: `TARGET_INVALID`

これらでは現行と同様にfallbackする。

### 8.3 solver実行

```cpp
bool allConstraintsSatisfied =
  prioritized_inverse_kinematics_solver2::solveIKLoop(...);
```

`allConstraintsSatisfied`をdebugへ保存する。

その後、明示的にFKとCOMを更新する。

### 8.4 candidate validation

`ProjectionValidationResult validation =
validateProjectionCandidate(gaitParam, dt);`

- `validation.safe == false`
  - statusとmetricsをdebugへ保存。
  - fallback。
  - return false。
- `validation.safe == true`
  - projected q、joint mask、CHEST、COM、realized velocityを保存。
  - `wbmsPostureReferenceValid = true`。
  - ACTIVE／BLOCKED／IDLEを分類。
  - return true。

### 8.5 ACTIVE／BLOCKED分類

指令normと実現速度normを、COMとtorsoで別々に評価する。

推奨定数:

```cpp
const double commandEps = 1e-6;
const double realizedComVelocityEps = 1e-5;
const double realizedTorsoVelocityEps = 1e-5;
```

概念:

```cpp
bool comCommanded = appliedComVelocity.norm() > commandEps;
bool torsoCommanded = appliedTorsoAngularVelocity.norm() > commandEps;

bool comActive = realizedComVelocity.norm() > realizedComVelocityEps;
bool torsoActive = realizedTorsoAngularVelocity.norm() > realizedTorsoVelocityEps;

if(!comCommanded && !torsoCommanded){
  status = VALID_IDLE;
}else if((comCommanded && comActive) ||
         (torsoCommanded && torsoActive)){
  status = VALID_ACTIVE;
}else{
  status = VALID_BLOCKED;
}
```

初回実装ではnorm判定でよい。
指令方向との内積判定はログ評価後の追加改善とする。

---

## 9. debug拡張

### 9.1 既存indexを変更しない

既存`wbmsDebugOut`の先頭30要素は順序・意味を変更しない。

末尾へ追加する。

### 9.2 追加推奨項目

データindexは0始まりで以下を推奨する。

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

この段階では`wbmsDebugOut`長を30から40へ変更する。

### 9.3 projectorと最終IKの分離観測

M4.3で必要に応じて以下を末尾へ追加する。

| index | 内容 |
|---:|---|
| 40–42 | final IK後のCOM realized velocity |
| 43–45 | final IK後のCHEST realized angular velocity |

この値を追加する場合、`wbmsDebugOut`長は46とする。

計算コストやFK／COM更新の影響があるため、まずindex 30–39を実装し、
projector valid化を確認してから40–45を追加してもよい。

---

## 10. 変更対象ファイル

### 10.1 必須

#### `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`

追加:

- `WbmsProjectionStatus`
- debug status
- `allConstraintsSatisfied`
- candidate safe
- support hull valid
- validation metrics

reset／clear時にdebug値を安全な初期値へ戻す。
debug値は制御入力に使用しない。

#### `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`

変更:

- `ProjectionValidationResult`追加
- `validateProjection()`を結果構造体版へ変更
- 必要なhelper追加
- status設定用helperは必要なら追加

#### `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`

変更:

- `solved`を`allConstraintsSatisfied`へ改名
- `solved && validateProjection`を削除
- candidate validationを独立化
- safe candidate採用
- ACTIVE／BLOCKED／IDLE分類
- target生成失敗理由のstatus設定
- validation metrics保存
- fallback時もstatusを消さない
- `setFallbackReference()`がstatus／metricsを不用意に上書きしないよう注意

#### `auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp`

変更:

- `wbmsDebugOut`末尾へstatusとmetricsを追加
- 既存0–29の順序を維持
- data lengthを更新
- index表をコメントまたは文書へ記載

#### `auto_stabilizer/docs/WBMSFeasibleVelocityPostureControlProgress.md`

追記:

- M4実装内容
- bool戻り値の意味
- safe candidate採用条件
- debug index
- build結果
- `/review`結果
- 未実施のシミュレータ項目

### 10.2 任意の後続変更

#### `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`

projector採用後も最終IKが姿勢を消していないか確認するため、
final IK前後のCHEST／COM変化をdebugへ追加する場合のみ変更する。

機能修正の初回コミットへ必須ではない。

---

## 11. 実装マイルストーン

## M4.1: statusとvalidation metrics

目的:

- invalid理由を1周期単位で識別可能にする。
- まだ採用条件は変更しない。

実装:

- enum／debug state追加。
- `ProjectionValidationResult`追加。
- 現行validatorを結果構造体へ変換。
- `wbmsDebugOut` index 30–39追加。
- 現行の`solved && validation.safe`は一時的に維持してよい。
- statusとmetricsが正しく出ることを静的確認。

完了条件:

- build成功。
- 既存debug index 0–29不変。
- validatorの全return経路でstatusが設定される。
- fallback後も直前の失敗理由がログへ残る。
- 制御挙動はM3完了時点と同じ。

## M4.2: safe candidate採用

目的:

- `solveIKLoop()`の全constraint充足フラグを採用条件から外す。
- 安全な1ステップ候補を最終IKへ接続する。

実装:

- `solved`を`allConstraintsSatisfied`へ改名。
- `validation.safe`だけで安全採否を判断。
- safe candidateならprojected q／CHEST／COM／realized velocityを保存。
- `wbmsPostureReferenceValid = true`。
- ACTIVE／BLOCKED／IDLE分類。
- unsafe candidateだけfallback。

完了条件:

- `rg`で`solved && validateProjection`相当が残っていない。
- `allConstraintsSatisfied`はdebug以外の採用条件に使われていない。
- unsafe candidateでは従来どおりfallbackする。
- safe candidateでは`allConstraintsSatisfied == false`でもvalidになり得る。
- build成功。
- `/review`で安全上の重大問題なし。

## M4.3: 最終IK伝達診断と整理

目的:

- projector候補が最終IK出力へ反映されることを確認可能にする。
- 文書とログ定義を確定する。

実装候補:

- final IK前後のCHEST／COM realized velocityを追加。
- 必要な場合だけ追加FK／COM計算を行う。
- 追加計算による500 Hz影響を計測。
- 進捗文書へdebug index全体を記載。

完了条件:

- projector realizedとfinal IK realizedを比較可能。
- projector activeなのにfinal IK realizedがゼロの場合、最終IK側の競合として切り分け可能。
- build成功。
- 既存腕CHEST相対拘束、歩行開始遅延、静止時限定条件を維持。

---

## 12. ビルド・静的確認

各マイルストーン後:

```sh
catkin build auto_stabilizer --no-deps
```

確認:

```sh
rg -n "solved|allConstraintsSatisfied|validateProjection|ProjectionValidationResult|wbmsProjectionStatus" \
  auto_stabilizer/rtc/AutoStabilizer

rg -n "m_wbmsDebug_\.data\.length|wbmsProjection" \
  auto_stabilizer/rtc/AutoStabilizer/AutoStabilizer.cpp \
  auto_stabilizer/rtc/AutoStabilizer/GaitParam.h

rg -n "wbmsPostureReferenceValid|wbmsProjectedChestR|wbmsProjectedRobotCom" \
  auto_stabilizer/rtc/AutoStabilizer
```

禁止:

- `clang-format`
- unrelated refactor
- 新規テストコード追加
- dependency solverのAPI変更
- task scaling導入
- 歩行中COM操作の有効化
- torso pitchとCOM Zの自動連動
- 既存debug index 0–29の変更

---

## 13. シミュレータ確認

## 13.1 体幹pitch単独

入力:

```text
COM velocity = [0, 0, 0]
torso angular velocity = [0, +0.03, 0] rad/s
```

段階的に:

```text
+0.03
+0.05
+0.10
0.00
-0.03
-0.05
```

確認:

- raw／applied pitch
- projector status
- allConstraintsSatisfied
- candidate safe
- realized torso pitch
- CHEST pitch offset
- operation mode
- final IK後CHEST速度（M4.3実装時）
- foot errors
- joint margin
- cycle time

期待:

- `allConstraintsSatisfied == 0`でもcandidate safeなら`VALID_ACTIVE`になり得る。
- `wbmsPostureReferenceValid == 1`。
- realized pitchの符号が入力と一致。
- CHEST offsetが連続的に変化。
- 指令ゼロで停止。
- 逆方向指令で戻る。
- limit到達後にhidden goalを蓄積しない。

## 13.2 COM Z単独

入力:

```text
torso angular velocity = [0, 0, 0]
COM velocity = [0, 0, -0.01] m/s
```

段階的に:

```text
-0.01
-0.02
0.00
+0.01
```

確認:

- raw／applied／realized COM Z
- projected robot COM Z
- genCog Z
- refdz
- omega
- ZMP
- knee／hip／ankle姿勢
- foot errors
- joint margin

初回試験では大きな下降指令を使わない。

## 13.3 同時操作

入力:

```text
torso pitch > 0
COM Z < 0
```

確認:

- 両方のrealized velocity
- ACTIVE／BLOCKED
- COM保持がCHESTを完全に抑制していないか
- CHEST保持がCOM下降を完全に抑制していないか
- 腕CHEST相対拘束
- ZMPと足誤差

---

## 14. 合格基準

最低合格条件:

1. 静止両足支持WBMS中、有限なpitch指令でprojector statusが`VALID_ACTIVE`になる。
2. `wbmsPostureReferenceValid`がtrueになる。
3. realized torso pitchが非ゼロで入力と同符号。
4. CHEST pitch offsetが連続的に変化。
5. COM Z負指令でrealized COM Zが負になる。
6. unsafe candidateは従来どおりfallbackする。
7. 既存の腕CHEST相対拘束を維持する。
8. 歩行中および歩行開始遅延中は既存どおり操作を抑制する。
9. debug index 0–29を維持する。
10. `catkin build auto_stabilizer --no-deps`成功。
11. projector／final IK／onExecute時間に著しい悪化がない。

望ましい合格条件:

- pitch単独試験でprojector valid率99%以上。
- 制約から離れた範囲で入力速度と実現速度が概ね比例。
- limit到達後、同方向入力を継続しても内部残差が蓄積しない。
- 逆方向指令で直ちに戻る。
- `onExecute`のp99が2 ms以内。

---

## 15. 本計画で行わないこと

以下は明示的に対象外とする。

- task scaling変数の導入。
- 新しい速度QP solverへの置換。
- 現行projectorの削除。
- final IKへの完全統合。
- 外部50 Hz retargeter。
- 姿勢マップ／lookup table。
- pitchからCOM Zを自動生成するcrouch coupling。
- 歩行中のユーザーCOM操作。
- 新規IDL parameter追加。
- 既存weight／limitの本格調整。
- `prioritized_inverse_kinematics_solver2`本体のAPI変更。

案Aが動作し、新しいログで入力・projector・final IKの関係を確認した後に、
必要性が認められた場合のみ次段階を計画する。

---

## 16. 実装上の注意

- `solveIKLoop()`のboolを`solverSuccess`と命名しない。
- `allConstraintsSatisfied`または同等の意味が明確な名前を使う。
- falseをQP失敗と断定しない。
- safe candidateが現在姿勢と同一の場合はBLOCKEDとして扱う。
- fallback関数で失敗statusを消去しない。
- debug stateを制御判断へ逆流させない。
- 1周期先targetと現在状態からの局所投影を維持する。
- 毎周期の新規clone、ancestor探索、不要なheap allocationを追加しない。
- M4.1、M4.2、M4.3をそれぞれbuild・reviewしてから次へ進む。
