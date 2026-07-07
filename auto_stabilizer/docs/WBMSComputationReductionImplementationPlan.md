# WBMS計算量削減 実装計画

## 1. 文書の位置づけ

本書は、`auto_stabilizer` のWBMS実現可能速度投影型・体幹/COM操縦に対する計算量削減計画である。

対象は、現行の2段IK構成を維持したまま、500 Hz実機運用での計算時間余裕を増やすことである。

現行の2段IK構成:

- 第1段: `WbmsPostureControl` による1周期先の実現可能姿勢投影。
- 第2段: `FullbodyIKSolver` による最終全身IK。

本書は後続実装スレッドへ渡すための計画書である。計算時間ログの根拠は `WBMSFeasibleVelocityPostureControlProgress.md` に記録された作業ログを用いる。削除済みまたは古い `auto_stabilizer/log/wbmsDebugOut.txt` は根拠にしない。

本作業は、既存のM1からM4.2.2系作業の後続として `M5: 500 Hz計算量削減` として扱う。

既存文書との関係:

- WBMSの制御仕様、安全仕様、歩行準備遷移仕様は以下を優先する。
  - `WBMSFeasibleVelocityPostureControlProgress.md`
  - `WBMSWalkingPreparationDesignRevisionPlan.md`
  - `WBMSWalkingPreparationTransitionImplementationPlan.md`
  - `WBMSProjectionAcceptanceFixImplementationPlan.md`
- `WBMSFeasibleVelocityPostureControlImplementationPlan.md` と `WBMSFeasibleVelocityPostureControlCodexPlan.md` は、初期M1からM3の基礎仕様・作業分割として参照する。
- 本書は計算量削減だけを扱う。
- 計算量削減については、本書を初期文書より優先する。
- 計算量削減のために安全判定、mode遷移、hidden goal非蓄積、速度/加速度limitを弱めてはならない。

本書内の作業単位index:

- `M5`: 500 Hz計算量削減。
- `M5.1`: 1-iteration運用方針とIK parameter整理。
- `M5.2`: `checkFinalState=false` 実装と適用。
- `M5.3`: projector priority 4姿勢参照の削減。
- `M5.4`: final IKのAngularMomentumConstraint切り分け。
- `M5.5`: 姿勢専用constraintの軽量化。
- `M5.6`: final IKの固定buffer化。
- `M5.7`: `solveIKLoop()` の1-iteration fast path。
- `M5.8`: self collision active setの安定化。
- `M5.9`: prioritized_qp fixed-structure fast path。

## 2. 前提

### 2.1 `maxIteration=1` 固定運用

本計画では、WBMS projectorおよびfinal IKを `maxIteration=1` のまま運用することを前提にする。

理由:

- 現行でも500 Hzに対する余裕は大きくない。
- 2段IK構成では、projectorとfinal IKでそれぞれprioritized IKを1回ずつ実行する。
- `maxIteration>1` は計算時間を直接増やし、500 Hz実機運用と両立しにくい。
- 現在のprojectorは「1周期先の安全な小ステップ候補を得る」設計であり、全constraintを反復で完全収束させる設計ではない。

したがって、`maxIteration>1` に備えた早期終了設計は本計画の主目的ではない。もし将来 `maxIteration>1` を検討する場合は、別途設計文書で計算時間、安全判定、収束条件を定義する。

### 2.2 最新ログの扱い

Progress文書の最新のログ解析は、現行最新コードに対する計測・確認結果である。このため、本計画の開始時に同じ条件で再計測する必要性は低い。

最新のPASS相当確認ログ:

```text
auto_stabilizer/log/test_start_walking202607061848*
auto_stabilizer/log/test_start_walking202607061851*
auto_stabilizer/log/test_start_walking202607061853*
```

これらは、11.4修正後の現行最新コードでの確認であり、以下を満たした。

- READY後に `goVelocity(0.0, 0.0, 0.0)` がacceptedされる。
- accepted直後に `wbmsOperationModeValue=0.0`。
- READY時に `wbmsWalkingStabilityModeValue >= 0.99`。
- READY前footstep抑制、READY前walking API reject、READY後walking API acceptが維持される。
- 腕指令なし条件で、11.3以前に問題となった0.1 rad級腕振動は確認されない。

一方、Progress文書に明示的な計算時間統計がまとまっている直近記録は `061700` である。

`061700` の計算時間記録:

| 項目 | 値 |
|---|---:|
| projector time | mean 0.348 ms、p99 0.691 ms、max 0.932 ms |
| final IK time | mean 0.775 ms、p99 1.334 ms、max 2.114 ms |
| onExecute time | mean 1.417 ms、p95 2.048 ms、p99 2.387 ms、max 3.416 ms、2 ms超過216周期 |

本計画では、最新PASS相当ログを制御挙動のbaseline、`061700` の時間統計を明示済み計算時間baselineとして扱う。再計測は各改善後の効果確認として行う。

## 3. 調査したコード範囲

主に以下を確認した。

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2`
- `~/catkin_ws/cnoid2/src/prioritized_qp/prioritized_qp_base`
- `~/catkin_ws/cnoid2/src/prioritized_qp/prioritized_qp_osqp`
- `~/catkin_ws/cnoid2/src/ik_solvers2/ik_constraint2`
- `~/catkin_ws/cnoid2/src/ik_solvers2/ik_constraint2_joint_limit_table`

## 4. コード分析結果

### 4.1 支配的な負荷は2つのprioritized IK

現行WBMS中は、通常の1周期で次の2つのIKが走る。

1. `WbmsPostureControl::solveProjection()`
2. `FullbodyIKSolver::solveFullbodyIK()`

`WbmsPostureControl` 側は、projection robot、variables、constraints、priority vector、IKParam、support hull bufferを初期化時に構築しており、500 Hz経路でのallocation抑制はある程度実装済みである。

一方、`FullbodyIKSolver::solveFullbodyIK()` は、毎周期ローカルに以下を構築している。

- `std::vector<double> refq`
- `std::vector<cnoid::LinkPtr> variables`
- `std::vector<double> dqWeight`
- `ikConstraint0` から `ikConstraint4`
- `constraints`
- `prioritized_inverse_kinematics_solver2::IKParam param`

これはallocation削減候補である。ただし、支配的なのはvector生成そのものではなく、`solveIKLoop()` 内部のFK/COM更新、constraint更新、QP行列再構築、OSQP solveである。

### 4.2 `solveIKLoop()` は `maxIteration=1` でもconstraint更新を2回行う

`prioritized_inverse_kinematics_solver2::solveIKLoop()` は概略として次を行う。

```text
初期状態保存
速度計算
FK/COM
updateConstraints()
solveIKOnce()
速度計算
FK/COM
updateConstraints()
checkConstraintsSatisfied()
return satisfied
```

現行のprojectorもfinal IKも `maxIteration=1` で使っている。

この場合でも、solve前とsolve後で `updateConstraints()` が2回走る。solve後のconstraint再評価は、戻り値として「全constraintが満足したか」を返すために必要になっている。

しかし、現在の使い方では次の通りである。

- projectorは、`solveIKLoop()` 戻り値ではなく独自の `validateProjectionCandidate()` で採用可否を判定する。
- final IKは、`solveIKLoop()` の戻り値を制御判断に使っていない。

したがって、現行用途ではsolve後の全constraint再評価は、計算コストに対して利益が小さい。

### 4.3 `IKParam::checkFinalState` は宣言済みだが実装に使われていない

`prioritized_inverse_kinematics_solver2::IKParam` には以下が存在する。

```cpp
bool checkFinalState = true;
```

コメント上は、最終状態で各constraintを満たすか確認するかどうかを切り替える意図のparameterである。

しかし、確認した実装では `checkFinalState` が `solveIKLoop()` の分岐に使われていない。このため、現状では呼び出し側から最終constraint再評価を無効化できない。

これは最優先の改善候補である。

### 4.4 `precision=0.0` は現行では満足判定をほぼfalseにする設定

`FullbodyIKSolver.cpp` と `WbmsPostureControl.cpp` には、以下のような設定が複数ある。

```cpp
constraint->precision() = 0.0; // 強制的にIKをmax loopまで回す
```

このコメントは、`maxIteration=1` 固定運用の現状では実態と合っていない。

現状の実際の意味:

- `maxIteration=1` なので、iteration数は常に1回で終わる。
- `precision=0.0` は、solve後の `checkConstraintsSatisfied()` で完全一致に近い満足判定を要求する。
- projectorでは `allConstraintsSatisfied` がfalseになりやすく、診断値として分かりにくい。
- final IKでは戻り値を使っていないため、制御分岐には直接効かない。
- ただし、`checkFinalState` が未実装のため、満足判定のためのsolve後constraint再評価コストは残る。

したがって、`precision=0.0` は「iteration数を増やす問題」ではなく、「現行1 iteration運用で不要な満足判定と分かりにくい診断を生む問題」として扱う。

### 4.5 zero weight制約でも内部計算が残る場合がある

`PositionConstraint` は、`weight` が0の成分をeq行から除外する。しかし、内部の6D Jacobian計算自体は、位置weightが0でも軽くならない。

該当例:

- final IKのCHEST姿勢拘束
- final IKのroot姿勢拘束
- projectorのCHEST姿勢拘束

これらは位置weightを0、姿勢weightを非ゼロにしているが、`PositionConstraint::updateJacobian()` ではA/B/evalの6D Jacobianを構築・更新してから、非zero weight行だけを取り出す。

つまり、「姿勢だけを拘束したい」用途に対して、汎用6D position constraintは過剰である。

### 4.6 `AngularMomentumConstraint` は低weightでも高コスト

`FullbodyIKSolver::solveFullbodyIK()` は優先度3にAngularMomentumConstraintを入れている。

現行設定:

```cpp
targetAngularMomentum = Zero
maxError = 1.0 * dt
weight = [1e-4, 1e-4, 0.0]
precision = 0.0
```

再調査で分かった問題:

- `AngularMomentumConstraint::updateBounds()` は常に3行eqを作る。
- z軸weightが0でも、eq行は3行残る。
- `updateBounds()` で `cnoid::calcAngularMomentumJacobian()` が呼ばれる。
- `updateJacobian()` でも角運動量Jacobian係数計算が走る。
- weightが非常に小さいため、挙動への寄与に対して計算コストが大きい可能性が高い。

これは、final IK内で最も費用対効果を疑うべきconstraintである。

### 4.7 projector priority 4姿勢参照は追加QP solveを発生させる

`WbmsPostureControl::solveProjection()` は、priority 4に低weight姿勢参照を追加している。

現行設定:

```cpp
postureReferenceConstraint.weight = 1e-4
targetq = gaitParam.genRobot current q
```

projectorは毎周期、現在の `genRobot` から開始し、hidden goalを持たない設計である。安全性は以下で担保している。

- joint velocity
- joint limit
- self collision
- 足拘束
- CHEST/COM拘束
- candidate validation
- invalid時fallback

このため、priority 4の低weight姿勢参照は、計算量に対して効果が小さい可能性がある。優先度が別段であるため、QP solveを1回追加する点も重い。

### 4.8 final IK priority 4 reference angleも追加QP solve

final IKのpriority 4 reference angleは、投影対象外の腕や全身姿勢のnullspace安定化として意味があるため、projectorの姿勢参照ほど簡単には削除できない。

ただし、計算量上は追加QP solveを発生させている。M5.1からM5.5の効果が不足する場合は、reference angleを別の正則化や同一優先度内の軽量表現へ移す検討対象になる。

### 4.9 `solveIKLoop()` の汎用処理はauto_stabilizer用途に対して過剰

`solveIKLoop()` は汎用solverであるため、毎回以下を行う。

- `std::set<cnoid::BodyPtr>` の構築
- `std::unordered_map<cnoid::LinkPtr, InitialJointState>` の構築
- `prevFrame` の構築
- rejection処理
- path出力処理
- viewer処理
- 複数body対応

auto_stabilizerのprojector/final IK用途では、基本的に次が前提である。

- `maxIteration=1`
- rejectionなし
- pathなし
- viewerなし
- bodyは1個

したがって、auto_stabilizer用途向けのfast pathを作る余地がある。

### 4.10 QP層は毎回行列を組み直す

`prioritized_qp_base::solve()` は優先度を進めるたびに以下を行う。

- `As/lBs/uBs/w_exts` を `conservativeResize()` で増やす。
- taskごとに `taskA`、`taskC` をコピーする。
- `H`、`A`、`gradient`、`upperBound`、`lowerBound` を毎回生成する。
- OSQP solverへ `UpdateObjectiveAndConstraintMatrices()` または `initializeSolver()` を行う。

制約数と優先度構成が周期ごとに変わらない場合でも、現行実装は毎周期同じ構造の行列を再構築する。

さらに、self collision constraintの採用数が距離閾値で増減すると、QPの制約行数が変わり、OSQPの再初期化が起こりやすくなる。

## 5. 改善案の優先度順

### 優先度A: まず実施する

1. `maxIteration=1` 固定方針をコードコメント/計画書に明記し、`precision=0.0` コメントを現状の意味に合わせて修正する。
2. `IKParam::checkFinalState` を実装し、projector/final IKで不要なsolve後constraint再評価を省く。
3. projector priority 4の低weight姿勢参照を削除または無効化する。
4. final IKのAngularMomentumConstraintを無効化可能にし、計算時間と挙動差を比較する。
5. CHEST/root等の姿勢だけを扱うconstraintを軽量化する。

### 優先度B: Aの効果確認後に実施する

6. `FullbodyIKSolver::solveFullbodyIK()` の毎周期ローカルvector/IKParamをメンバbufferへ移す。
7. `solveIKLoop()` に `maxIteration=1`、rejectionなし、pathなし、body 1個向けfast pathを追加する。
8. self collision constraintのactive数変化にヒステリシスを入れ、QP次元変化とOSQP再初期化を減らす。

### 優先度C: 必要なら実施する

9. `prioritized_qp_base` に固定構造fast pathを追加し、毎周期の `conservativeResize()` と行列再構築を減らす。
10. final IK priority 4 reference angleを、可能なら軽量な正則化または同一優先度内の表現へ寄せる。
11. constraint更新の並列化を検討する。ただし500 Hz制御周期で毎回threadを生成する現行 `threadsNum` 実装は避ける。

## 6. 実装マイルストーン

## M5.1: 1-iteration運用方針とIK parameter整理

### 目的

WBMS projector/final IKが `maxIteration=1` 固定運用であることを明確にし、`precision=0.0` を含むIK parameterの意味を整理する。

### 実装内容

- `FullbodyIKSolver.cpp` と `WbmsPostureControl.cpp` の `precision=0.0` コメントを修正する。
- 「強制的にIKをmax loopまで回す」という現状と合わないコメントを削除する。
- `maxIteration=1` 固定前提であること、`maxIteration>1` は本計画の対象外であることをコメントまたは文書へ明記する。
- projectorの採用判定は `validateProjectionCandidate()` で行い、`allConstraintsSatisfied` は診断値であることを明記する。
- final IKは `solveIKLoop()` 戻り値を制御判断に使っていないことを明記する。

### 変更候補ファイル

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- 必要なら `prioritized_inverse_kinematics_solver2` のヘッダコメント

### acceptance criteria

- コメントが現行挙動と一致する。
- `maxIteration=1` 固定運用の意図が後続実装者に伝わる。
- 制御挙動は変更しない。
- ビルドが通る。

## M5.2: `checkFinalState=false` 実装と適用

### 目的

`maxIteration=1` の2段IKで、不要なsolve後constraint再評価を削減する。

### 設計

`prioritized_inverse_kinematics_solver2::IKParam::checkFinalState` を実装する。

推奨挙動:

```text
checkFinalState == true
  従来どおり、solve後にFK/COM、updateConstraints、checkConstraintsSatisfiedを行う。

checkFinalState == false
  solve後にFK/COMは行う。
  updateConstraintsとcheckConstraintsSatisfiedは行わない。
  戻り値はfalse、または「満足判定未実施」を表す扱いにする。
```

注意点:

- FK/COM更新は省かない。呼び出し側はsolve後のrobot姿勢とCOMを使うためである。
- projectorは独自validationを行うので、constraint再評価なしでも採用可否を判断できる。
- final IKは戻り値を使っていないため、constraint再評価なしでも制御フローは変わらない。
- 既存利用者への影響を避けるため、既定値は現行互換の `true` とする。
- `checkFinalState=false` 時の戻り値意味をヘッダコメントへ明記する。

### 変更候補ファイル

- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2/include/prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h`
- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2/src/prioritized_inverse_kinematics_solver2.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`

### acceptance criteria

- `checkFinalState=true` の既存挙動が維持される。
- projectorは `checkFinalState=false` で動作し、`validateProjectionCandidate()` による安全判定が維持される。
- final IKは `checkFinalState=false` で動作し、戻り値に依存しない。
- 最新PASS相当条件で、READY到達、歩行API accept、operation blend残留なしが維持される。
- projector/final IK/onExecute時間が悪化しない。改善が確認できれば望ましい。

## M5.3: projector priority 4姿勢参照の削減

### 目的

projector側の低weight姿勢参照による追加QP solveを削減する。

### 現状

`WbmsPostureControl::solveProjection()` はpriority 4に各projection jointの `JointAngleConstraint` を入れている。

```text
targetq = 現在の genRobot joint q
weight = 1e-4
```

projectorは毎周期現在姿勢から開始し、未達成goalを蓄積しない。そのため、この低weight姿勢参照は計算量に対して効果が小さい可能性がある。

### 実装方針

- 初回はparameterまたは内部flagで無効化できるようにする。
- 無効化時はpriority 4を空にする、またはpriority 4自体をconstraintsから外す。
- projection candidate validationは維持する。
- 無効化で姿勢が暴れる場合は、別の軽量なregularizationへ置き換える。

### 変更候補ファイル

- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`

### acceptance criteria

- projectorがvalid candidateを生成できる。
- hidden goal非蓄積が維持される。
- joint limit、joint velocity、self collision、足拘束、CHEST/COM、candidate validationが維持される。
- projector timeが改善する。
- final IK後の関節差分、CHEST/COM速度、歩行準備遷移が悪化しない。

## M5.4: final IKのAngularMomentumConstraint切り分け

### 目的

費用対効果が疑わしいAngularMomentumConstraintを切り分け、final IK時間を削減する。

### 設計

段階的に進める。

1. まずparameterまたは内部flagでAngularMomentumConstraintを無効化可能にする。
2. 無効化時はconstraint vectorへpushしない。
3. 有効/無効のログを比較し、挙動差と計算時間差を確認する。
4. 挙動差が十分小さければ、既定無効化を検討する。

注意点:

- AngularMomentumConstraintを外しても、足、COM、root、CHEST、joint safetyは維持する。
- 歩行中のバランス制御やStabilizerの責務を混同しない。
- もしAngularMomentumConstraintが急動作抑制に効いているログが出た場合、低周期更新またはweight条件付き有効化を検討する。
- 残す場合も、zero weight軸をeq行から落とす修正を検討する。

### 変更候補ファイル

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- 必要ならIDL parameter。ただし初回は内部flagまたは既存設定で比較してから判断する。

### acceptance criteria

- AngularMomentumConstraint有効/無効で、同一ログ条件のfinal IK時間を比較できる。
- 無効化時にfinal IK後CHEST角速度、COM速度、`el_q` 一周期最大差分が悪化しない。
- 最新PASS相当条件で、READY到達、READY後API accept、operation blend残留なしが維持される。
- 実機向けには、安全性に明確な悪化がないことを確認するまで既定無効化しない。

## M5.5: 姿勢専用constraintの軽量化

### 目的

位置weightが0で姿勢だけを拘束する用途に対し、汎用6D `PositionConstraint` の過剰計算を削減する。

### 対象

- final IK CHEST姿勢拘束
- final IK root姿勢拘束
- projector CHEST姿勢拘束

### 設計候補

候補A: `OrientationConstraint` を新設する。

- 3D姿勢誤差だけをeqにする。
- 位置Jacobianを計算しない。
- eval linkが必要な場合は姿勢Jacobianだけ計算する。
- 既存 `PositionConstraint` の挙動を変えないため、他パッケージへの影響が小さい。

候補B: `PositionConstraint` にfast pathを追加する。

- 位置weightが全ゼロの場合、位置Jacobian計算を省く。
- 既存classを使い続けられる。
- ただし他利用者への影響範囲が広い。

推奨は候補Aである。新規constraintとして追加した方が、影響範囲をauto_stabilizer用途に限定しやすい。

### 変更候補ファイル

- `~/catkin_ws/cnoid2/src/ik_solvers2/ik_constraint2/include/ik_constraint2/OrientationConstraint.h`
- `~/catkin_ws/cnoid2/src/ik_solvers2/ik_constraint2/src/OrientationConstraint.cpp`
- `~/catkin_ws/cnoid2/src/ik_solvers2/ik_constraint2/CMakeLists.txt`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`

### acceptance criteria

- CHEST/root姿勢拘束の意味が変わらない。
- 位置拘束を誤って追加しない。
- final IK後CHEST角速度、COM速度、joint deltaが悪化しない。
- projector/final IK時間が改善する。

## M5.6: final IKの固定buffer化

### 目的

`FullbodyIKSolver::solveFullbodyIK()` の毎周期allocationとvector構築を削減する。

### 設計

`FullbodyIKSolver` に以下のmutable memberを追加する。

- `refqBuffer`
- `variablesBuffer`
- `dqWeightBuffer`
- `ikConstraint0Buffer`
- `ikConstraint1Buffer`
- `ikConstraint2Buffer`
- `ikConstraint3Buffer`
- `ikConstraint4Buffer`
- `constraintsBuffer`
- `ikParam`

初期化時または初回呼び出し時にcapacityを確保し、毎周期は `clear()` と代入だけにする。

注意点:

- `gaitParam.selfCollision.size()` が増える場合はresizeが必要。これは既存でも発生し得るので、増加時のみ許容する。
- constraintのpriority構成は変えない。
- `refq` は固定長なので、`resize()` を毎周期呼ばずに初期化時に長さを合わせる。
- `dqWeight` はcontrollable joint数が変わらない前提なら固定長化できる。
- `jointControllable` が実行中に変わり得るなら、変更検出時だけ再構築する。

### 変更候補ファイル

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`

### acceptance criteria

- final IKのpriority構成とconstraint内容が変更前と一致する。
- 毎周期のvector allocationが減る。
- ビルドが通る。
- 最新PASS相当条件でfinal IK時間の平均またはp99が改善する。
- 動作ログで関節差分、CHEST/COM速度、安全判定が悪化しない。

## M5.7: `solveIKLoop()` の1-iteration fast path

### 目的

auto_stabilizer用途に対して過剰な汎用処理を削減する。

### 対象前提

- `maxIteration=1`
- rejectionなし
- pathなし
- viewerなし
- body 1個
- final state constraint checkなし

### 削減候補

- `std::set<cnoid::BodyPtr>` の構築
- `std::unordered_map<cnoid::LinkPtr, InitialJointState>` の構築
- `prevFrame` の構築
- 空rejection vector生成
- path/viewer/rejection関連分岐

### 設計

既存 `solveIKLoop()` の互換性を壊さず、新APIまたはIKParam optionでfast pathを追加する。

例:

```cpp
bool solveIKLoopOnceFast(...);
```

または:

```cpp
param.assumeSingleBodyOneIterationNoRejection = true;
```

初回はauto_stabilizerからのみ使う。

### 変更候補ファイル

- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2/include/prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h`
- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2/src/prioritized_inverse_kinematics_solver2.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`

### acceptance criteria

- 既存APIの挙動を壊さない。
- fast path使用時もFK/COM更新、constraint更新、QP solve、変数更新が正しく行われる。
- projector/final IKの安全validationが維持される。
- 計算時間が改善する。

## M5.8: self collision active setの安定化

### 目的

self collision constraint数の周期間変動を減らし、QP次元変化とOSQP再初期化を抑える。

### 現状

final IKとprojectorはいずれも、自己干渉情報のうち距離が近いものだけをconstraintへ入れている。

現行例:

```cpp
if(gaitParam.selfCollision[i].distance < 0.05){
  ikConstraint1.push_back(this->selfCollisionConstraint[i]);
}
```

この閾値をまたぐ接触候補があると、周期ごとにconstraint数が変わり、task行列サイズとOSQP問題サイズが変わる可能性がある。

### 設計

active setにヒステリシスを入れる。

例:

```text
inactive -> active: distance < 0.05
active -> inactive: distance > 0.07
```

または、一定周期だけactiveを保持する。

注意点:

- safety側に倒すため、active解除は遅くしてよい。
- hidden goalではなく、collision安全制約の保持であることを明確にする。
- self collision入力そのものが不連続な場合でも、constraint数の頻繁な増減を避ける。

### 変更候補ファイル

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`

### acceptance criteria

- active self collision constraint数が短周期で振動しにくくなる。
- collisionが近い状態では制約が外れない。
- projector/final IK時間のp99/maxが改善する。
- 安全性を損なう解除遅れや解除早すぎがない。

## M5.9: prioritized_qp fixed-structure fast path

### 目的

`prioritized_qp_base::solve()` の行列再構築コストを削減する。

### 設計案

固定構造のQP問題として扱える場合のfast pathを追加する。

対象条件:

- task数が固定。
- 各taskのeq/ineq行数が固定。
- ext列を使わない、またはext列構造が固定。
- dimが固定。

fast pathで行うこと:

- `As/lBs/uBs` の最大サイズを事前確保する。
- `H/A/gradient/bounds` のサイズを固定し、値だけ更新する。
- OSQPのproblem sizeを変えず、毎周期updateで済ませる。

注意点:

- `prioritized_qp_base` は他パッケージも使うため、既存APIの挙動を壊さない。
- fast pathは新APIまたは新optionとして追加する。
- まずauto_stabilizerのprojector/final IKで使える最小機能に限定する。

### 変更候補ファイル

- `~/catkin_ws/cnoid2/src/prioritized_qp/prioritized_qp_base`
- `~/catkin_ws/cnoid2/src/prioritized_qp/prioritized_qp_osqp`
- `~/catkin_ws/cnoid2/src/ik_solvers2/prioritized_inverse_kinematics_solver2`
- 呼び出し側として `WbmsPostureControl` / `FullbodyIKSolver`

### acceptance criteria

- 既存 `prioritized_qp_base::solve()` の互換性を維持する。
- fixed-structure pathを使った場合、OSQP再初期化回数が減る。
- projector/final IK時間のp99/maxが改善する。
- QP failure時のfallbackは従来どおり安全側に倒れる。

## 7. 今回は採用しない方針

以下は本計画では採用しない。

- 2段IKを廃止して1段IKへ統合する。
- Projection IKを間引き実行する。
- final IKを間引き実行する。
- `maxIteration>1` にする。
- joint limit、足拘束、self collision、安全validationを弱める。
- mode切替時の速度/加速度limitを緩める。
- hidden goalを蓄積して未達成分を後で回収する。
- 500 Hz経路で毎周期threadを生成する並列化。

特にProjection IKの間引きは、計算量だけを見ると魅力があるが、1周期先の実現可能速度投影という現在の安全設計と衝突する。採用する場合は別設計文書で安全性を再定義する必要がある。

## 8. 検証計画

### 8.1 ビルド確認

IDL変更なしの場合:

```sh
catkin build auto_stabilizer --no-deps
```

IDL変更ありの場合:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

prioritized IKまたはprioritized QP側を変更した場合は、依存パッケージのビルド範囲を別途確認する。

### 8.2 ログ確認

最新PASS相当条件を再実行し、少なくとも以下を比較する。

- projector time mean / p95 / p99 / max
- final IK time mean / p95 / p99 / max
- onExecute time mean / p95 / p99 / max
- onExecute 2 ms超過周期数
- READY到達時刻
- `goVelocity` accepted時刻
- accepted直後 `wbmsOperationModeValue`
- final IK後CHEST角速度最大
- final IK後COM速度最大
- `el_q` 一周期最大差分
- 腕関節span

### 8.3 acceptance criteria

全体として以下を満たす。

- 最新PASS相当条件で、歩行開始挙動が退行しない。
- READY後のwalking API accepted直後に `wbmsOperationModeValue=0.0` が維持される。
- `wbmsWalkingStabilityModeValue >= 0.99` でREADYへ進む。
- final IK後CHEST角速度、COM速度、`el_q` 一周期最大差分がProgress文書の最新PASS相当ログより悪化しない。
- 腕指令なし条件で0.1 rad級腕振動が再発しない。
- projector/final IK/onExecute時間のp99またはmaxが改善する。
- 500 Hz実行で2 ms超過周期数が減る。

## 9. 実装時の注意

- `AGENTS.md` に従い、clang-formatは適用しない。
- コメントやmarkdownは日本語で書く。
- C++のclass名、関数名、変数名は原則英語にする。
- 実ロボット安全側の挙動を優先する。
- 計算量削減のために指令値を不連続にしてはならない。
- mode切替時の速度指令はlimiter/filterを通す。
- fallback時にhidden goalを残してはならない。
- 本リポジトリではテスト作成は不要だが、ビルドとログ確認は行う。

## 10. 次スレッドへの最初の作業指示案

次スレッドでは、まずM5.1とM5.2を実施する。

作業順:

1. `git status --short` で作業ツリーを確認する。
2. `FullbodyIKSolver.cpp` と `WbmsPostureControl.cpp` の `precision=0.0` コメントを確認する。
3. `maxIteration=1` 固定運用と、`solveIKLoop()` 戻り値の扱いをコメントに反映する。
4. `prioritized_inverse_kinematics_solver2` の `IKParam::checkFinalState` が未実装であることを再確認する。
5. `checkFinalState=false` 時にsolve後constraint再評価を省く実装を追加する。
6. `WbmsPostureControl::solveProjection()` と `FullbodyIKSolver::solveFullbodyIK()` のIKParamで `checkFinalState=false` を指定する。
7. `catkin build auto_stabilizer --no-deps` を実行する。
8. 必要に応じて最新PASS相当条件でログを取得し、安全指標と計算時間を比較する。

M5.2の効果が十分でない場合、M5.3のprojector priority 4姿勢参照削減へ進む。
