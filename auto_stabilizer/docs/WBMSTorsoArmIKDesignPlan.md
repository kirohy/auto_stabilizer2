# WBMS体幹・腕協調IK設計計画

## 現行優先度に関する注記

本書は、WBMS体幹・腕協調IKの最初期設計案を記録した履歴資料である。現行のWBMS実現可能速度投影型・体幹/COM操縦、歩行準備遷移、計算量削減の最新計画ではない。

後続作業により、本書の一部方針は更新または不採用になっている。特に以下の点は、現行仕様として本書を優先してはならない。

- 体幹姿勢目標を内部で積分して保持する方針。
- CHEST姿勢を最終IKで直接扱う初期案。
- COM Z低下オフセットやかがみ動作補助を現行必須機能として扱う方針。
- 歩行開始遅延を固定時間のweight復帰として扱う初期整理。

現行仕様・実装計画を判断する場合は、まず `WBMSFeasibleVelocityPostureControlProgress.md` の参照順に従う。計算量削減は `WBMSComputationReductionImplementationPlan.md`、歩行準備遷移は `WBMSWalkingPreparationDesignRevisionPlan.md`、projector候補採用判定は `WBMSProjectionAcceptanceFixImplementationPlan.md` を優先する。

本書は、体幹・腕IKで何を問題視していたか、なぜCHEST相対の腕拘束や過拘束回避が重視されたかを理解するための背景資料として読む。

## 目的

この文書は、`startWholeBodyMasterSlave()` 起動後の操縦モードにおいて、体幹姿勢操縦と腕操縦を同時に扱うためのIK設計方針をまとめる。

想定する主な動作は、ロボット視点映像を見ながら1人称視点で操縦し、床に置いてある物体をかがんで拾うような作業である。

この操縦モードでは、WBMS起動時のmaster側エンドエフェクタ姿勢指令値とslave側ロボット現在値を保存し、それを基準値として、以後の姿勢差分をロボットへ反映する。

## 前提と編集範囲

- 実装対象は `auto_stabilizer/rtc/AutoStabilizer` 配下を基本とする。
- IDLにパラメータを追加する場合のみ、`auto_stabilizer/idl/AutoStabilizerService.idl` の編集を許容する。
- コメントやMarkdownは日本語で記述する。
- `clang-format` は適用しない。
- 本リポジトリではテスト作成は不要である。
- Choreonoid基準で体幹リンク名は `CHEST_JOINT2` として扱う。
- IDL編集後の初回ビルドは以下を使う。

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

以後、依存パッケージを編集していない場合は以下でよい。

```sh
catkin build auto_stabilizer --no-deps
```

## 現状整理

### WBMSの腕操縦

以前の作業で、WBMS中の直接操縦対象は上半身エンドエフェクタに限定された。

脚はWBMS指令で直接動かさず、`goVelocity()`、`goPos()`、`setFootSteps()` などの既存歩行APIに任せる。

また、歩行と腕操縦を同時実行した際に、腕がodom/world基準で空間固定されるのではなく、体幹に自然に追従するよう、WBMS起動時の上半身エンドエフェクタ姿勢は `CHEST_JOINT2` 相対で保存する設計になっている。

現在の実装では、`startWholeBodyMasterSlave()` で以下のように上半身エンドエフェクタの基準姿勢を保存している。

```cpp
torsoReferenceLink->T().inverse() * refEEPose[i]
```

`RefToGenFrameConverter::convertRefEEPoseRawDifferential()` では、master側差分をこの体幹相対基準に反映し、`refEEPoseWithOutFK[i]` を生成している。

### 現在のIK側の注意点

現状の `FullbodyIKSolver` では、上半身エンドエフェクタ拘束の `B_link` が `genRobot->rootLink()` になっている。

つまり、目標生成段階では体幹相対で手先目標を作っているが、IK拘束としてはroot相対へ変換されている。

このため、体幹姿勢を新たに動かす場合、手先6自由度拘束が体幹運動を阻害する可能性がある。体幹操縦を設計する場合、上半身エンドエフェクタ拘束自体も `CHEST_JOINT2` 相対の拘束へ変更する必要がある。

### `ee_eval_link_name` の意味

`ee_eval_link_name` は、IK拘束の基準リンクを変更するものではなく、誤差評価座標系を指定するパラメータである。

`ee_eval_link_name = CHEST_JOINT2` としても、`B_link` が `CHEST_JOINT2` になるわけではない。

したがって、腕を本当に体幹相対IK拘束として扱うには、`FullbodyIKSolver` 側で上半身エンドエフェクタ拘束の `B_link` と `B_localpos` の作り方を明示的に変更する必要がある。

## ロボット自由度と拘束の見積もり

対象モデルは以下である。

```text
~/catkin_ws/cnoid2/src/msl_hand_controller/msl_hand_controller/models/JAXON_RED_SENSORS.urdf
```

URDFから読み取れる主な自由度は以下である。

- 脚: 左右6自由度ずつ、合計12自由度。
- 体幹: `CHEST_JOINT0`、`CHEST_JOINT1`、`CHEST_JOINT2` の3自由度。
- 腕: 左右8自由度ずつ、合計16自由度。
- 現運用でheadをcontrollableから外す場合、制御対象関節は合計31自由度。
- IK変数にはfloating root 6自由度も含まれるため、探索変数としては最大37自由度相当になる。

ただし、両足接地中は足先拘束が強く、floating root 6自由度は実質的に足拘束と強く結びつく。単純な変数数だけで余裕があるとは判断できない。

静止両足支持で考えると、代表的な拘束は以下である。

- 両足6自由度拘束: 12自由度相当。
- COM XY拘束: 2自由度相当。
- COM Z拘束: 状態により0から1自由度相当。
- 左右手先6自由度拘束: 12自由度相当。
- 体幹姿勢3自由度拘束: 3自由度相当。
- 将来の左右swivel angle拘束: 2自由度相当。

合計すると31から32自由度相当となる。

これは数の上では解けそうに見えるが、実際には関節可動域、自己干渉、joint limit table、体幹pitch/rollの狭さにより、実効自由度はかなり減る。

特に体幹関節の可動域は以下のように狭い。

- `CHEST_JOINT0`: roll軸、約 `[-0.198, 0.198]` rad。
- `CHEST_JOINT1`: pitch軸、約 `[-0.035, 0.611]` rad。
- `CHEST_JOINT2`: yaw軸、約 `[-1.057, 1.057]` rad。

このため、体幹姿勢、左右手先6自由度、左右swivel angleをすべて硬い同列拘束にする設計は避ける。

## 採用すべき基本設計

### 体幹は低優先度の余剰タスクにしない

ロボット視点映像を見た1人称視点操縦では、体幹または視点方向は操縦者の作業フレームである。

そのため、体幹操縦を腕IKの余り自由度で動けばよい低優先度タスクとして扱うのは不適切である。

体幹姿勢は操縦意図の主要成分として扱う。ただし、実ロボットの安全制約、足接触、COM、joint limit、self collisionより上位には置かない。

### 体幹姿勢は硬い直接拘束にしない

コメントアウトされているrootLink姿勢直接指定は、root姿勢を強く拘束するため、足拘束、COM、歩行安定化と衝突しやすい。

`CHEST_JOINT2` の姿勢3自由度を直接指定する案はrootLink指定より局所的ではあるが、危険が消えるわけではない。

体幹は3自由度しかなく、さらに可動域が狭い。そこへ手先6自由度と将来swivel angle拘束を同時に課すと、単にrootLink直接拘束を体幹リンク直接拘束へ置き換えただけになり得る。

したがって、体幹姿勢は以下の性質を持つソフト拘束として実装する。

- WBMS開始時姿勢からの有界な姿勢差分目標を持つ。
- 角速度入力、積分姿勢差分、1周期補正量をすべてlimitする。
- joint limit接近時やIK残差増大時に無理に追従しない。
- 歩行開始遅延中および歩行中は、歩行安定化を優先してweightを下げる。
- stop/start時に明示的に初期化し、hidden goalを残さない。

## IKタスク構造

推奨する優先度構造は以下である。

### 最優先タスク

- joint velocity limit。
- joint limit。
- self collision。

これらは安全上の制約であり、操縦指令より上位に置く。

### 接触・安定タスク

- 足エンドエフェクタ拘束。
- COM XY拘束。
- 歩行中またはWBMS歩行開始遅延中のroot姿勢復帰。
- 歩行中またはWBMS歩行開始遅延中のCOM Z復帰。

WBMS中の静止時は、体幹や腕操縦の自由度を確保するため、root姿勢とCOM Z拘束は弱める。

歩行開始時は既存の `wbmsWalkingStabilityMode` を用い、歩行安定化側へ滑らかに戻す。

### 主操作タスク

- `CHEST_JOINT2` 姿勢3自由度。
- 左右手先6自由度。

体幹姿勢と手先6自由度は、1人称視点操縦の主操作である。

ただし、両者を同じ意味の硬い拘束にはしない。

手先拘束は `CHEST_JOINT2` 相対で扱う。これにより、体幹が動いたときに手先目標も体幹とともに動き、手先拘束が体幹運動をworld基準で阻害しにくくなる。

体幹姿勢拘束は、強すぎる姿勢固定ではなく、有界目標への追従タスクとする。

### 腕形状タスク

- 将来のswivel angle拘束。
- 肘位置や肘方向などの補助拘束。
- reference posture。

swivel angleは手先6自由度と同列に置かない。

swivel angleは「手先6自由度を満たしたうえで腕の形状を選ぶ拘束」として扱う。手先、足、COM、体幹姿勢と衝突する場合は、swivel側が譲る設計にする。

## 上半身エンドエフェクタ拘束の変更

WBMS中の上半身エンドエフェクタ拘束は、`CHEST_JOINT2` 相対に変更する。

現状の概念は以下である。

```cpp
B_link = genRobot->rootLink();
B_localpos = gaitParam.refRobot->rootLink()->T().inverse() * gaitParam.abcEETargetPose[i];
```

変更後は、WBMS中の上半身エンドエフェクタについて以下の概念にする。

```cpp
torsoLink = genRobot->link(gaitParam.chestLinkName);
B_link = torsoLink;
B_localpos = torsoTargetPose.inverse() * gaitParam.abcEETargetPose[i];
```

実装時には、`abcEETargetPose[i]` がどの座標系で保持されているかを確認し、`CHEST_JOINT2` 相対の目標を過不足なく構成する。

WBMSでない場合は既存挙動を維持する。

## 体幹姿勢指令の生成

入力ポートは既存の `refTorsoVelIn` を使う。

`refTorsoVelIn` の角速度成分は体幹相対角速度として扱う。

実装上は以下の状態を `GaitParam` に追加する。

- WBMS開始時の体幹基準姿勢。
- WBMS体幹姿勢差分目標。
- 体幹姿勢目標の現在値または補間状態。
- かがみ動作用COM Zオフセット。

入力角速度は以下の順で処理する。

1. finiteチェックを行う。
2. `wbms_torso_angular_velocity_limit` で成分ごとにclampする。
3. 体幹相対角速度として姿勢差分へ積分する。
4. 積分されたroll/pitch/yaw差分を `wbms_torso_rpy_limit` でclampする。
5. WBMS無効時、WBMS停止時、AutoBalancer停止時には0へ戻す。

## 体幹姿勢IK拘束

`FullbodyIKSolver` に、体幹姿勢用の `PositionConstraint` を追加する。

対象リンクは `genRobot->link(gaitParam.chestLinkName)` とする。

この拘束では位置3自由度のweightは0にし、姿勢3自由度のみを使う。

weightは以下の要素で決める。

- `wbms_torso_orientation_weight`
- `gaitParam.wbmsMode.value()`
- `wbmsWalkingStabilityMode`
- joint limit接近時の退避係数

歩行中または歩行開始遅延中は、`wbmsWalkingStabilityMode` に応じて体幹姿勢weightを弱める。

体幹姿勢拘束の `maxError` はパラメータ化し、1周期で大きな姿勢差を解こうとしない。

## かがみ動作用COM Z連動

床の物体を拾う動作では、体幹を前方へ傾けるだけでは脚が伸び切りやすい。

そのため、WBMS静止中に限り、体幹pitch目標に応じてCOM Z目標を下げる補助機能を用意する。

ただし、この機能は本質的な腕・体幹IKではなく、作業姿勢を作る補助であるため、デフォルトでは無効または保守的な値にする。

有効時の概念は以下である。

```text
wbmsTorsoCogZOffset = -clamp(abs(forwardPitch) * wbms_crouch_cog_z_gain,
                             0,
                             wbms_crouch_cog_z_limit)
```

注意点は以下である。

- 無制限にCOMを下げない。
- 歩行開始遅延中および歩行中は0へ戻す。
- joint limitや自己干渉により姿勢が厳しい場合、COM Z低下を優先しすぎない。
- 体幹姿勢追従のための必須機能ではないため、検証しながら有効化する。

## 追加パラメータ案

`AutoStabilizerParam` に以下を追加する。

```idl
sequence<double,3> wbms_torso_angular_velocity_limit;
sequence<double,3> wbms_torso_rpy_limit;
sequence<double,3> wbms_torso_orientation_weight;
sequence<double,3> wbms_torso_orientation_max_error;
boolean use_wbms_crouch_cog_z;
double wbms_crouch_cog_z_gain;
double wbms_crouch_cog_z_limit;
```

推奨初期値は実装時に保守的に設定する。

例としては以下のような方針とする。

- 角速度limitは実機で急変しない値にする。
- roll/pitch/yaw姿勢差分limitは、体幹joint limitより十分内側にする。
- pitchは前屈方向を主用途とし、後屈方向は小さくする。
- `wbms_torso_orientation_weight` は手先6自由度を完全に壊さない範囲から開始する。
- `use_wbms_crouch_cog_z` は初期状態ではfalseまたは極小gainから開始する。

## 実装候補ファイル

主な実装対象は以下である。

- `GaitParam.h`
  - WBMS体幹操縦状態とパラメータを追加する。
  - `reset()` で残留状態を消す。
- `AutoStabilizer.cpp`
  - `refTorsoVelIn` の読み取り処理を拡張する。
  - `startWholeBodyMasterSlave()` / `stopWholeBodyMasterSlave()` で体幹操縦状態を明示的に初期化する。
  - `setAutoStabilizerParam()` / `getAutoStabilizerParam()` に追加パラメータを反映する。
- `FullbodyIKSolver.h`
  - 体幹姿勢拘束用のconstraintを保持する。
- `FullbodyIKSolver.cpp`
  - WBMS中の上半身EE拘束を `CHEST_JOINT2` 相対へ変更する。
  - 体幹姿勢タスクを追加する。
  - COM Z低下オフセットを反映する。
- `auto_stabilizer/idl/AutoStabilizerService.idl`
  - 追加パラメータをIDLに追加する。

## 検証項目

### ビルド確認

IDL編集後初回:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

以後:

```sh
catkin build auto_stabilizer --no-deps
```

### 静止WBMSでの体幹単独操縦

- roll、pitch、yaw入力に対して `CHEST_JOINT2` が追従すること。
- 体幹関節がlimitに張り付かないこと。
- 入力停止後に角速度や姿勢目標が無制限に残留しないこと。

### 腕と体幹の同時操縦

- 体幹pitch/yaw入力と左右手先6自由度入力を同時に与える。
- 手先がworld固定のように体幹運動を妨げないこと。
- 操縦者視点で、体幹または視点方向が意図に追従していること。
- 手先追従だけが優先され、体幹が置いていかれる挙動にならないこと。

### かがみ動作

- 前屈入力に対してCOM Zが滑らかに下がること。
- 脚が伸び切る方向ではなく、膝や股関節を使ってかがむ姿勢になること。
- COM Z低下が強すぎて足拘束やjoint limitを悪化させないこと。

### 将来swivel angleを想定した検証

swivel angle自体は今回実装しない。

ただし、将来のswivel相当として、仮の肘位置拘束または肘方向拘束を低優先度で追加した検証を行う。

確認することは以下である。

- 左右手先6自由度 + 体幹3自由度 + 左右腕形状2自由度で、IK残差が急増しないこと。
- 腕形状拘束が手先6自由度や足/COMを破ってまで満たされないこと。
- 体幹姿勢weightを上げすぎた場合に、どの値から腕IKやCOM/足拘束へ悪影響が出るかを確認すること。

### 歩行開始時

- WBMS静止中に体幹をかがめた状態から `goVelocity()` 等を送る。
- 歩行開始遅延中に体幹姿勢weightとCOM Z低下が安全側へ戻ること。
- future footstep生成前に歩行安定化用のroot姿勢/COM Z拘束が復帰すること。
- 歩行開始時に下半身が不連続に振動しないこと。

## 判断基準

採用する設計は、以下を満たす必要がある。

- 1人称視点操縦として、体幹または視点方向が操縦意図に追従する。
- 腕手先6自由度が体幹運動とworld基準で衝突しない。
- 足接触、COM、安全制約を破ってまで体幹や腕を追従しない。
- 将来のswivel angle拘束を追加しても、手先6自由度や安定拘束を壊さない優先度構造になっている。
- mode切替時に速度指令や姿勢目標が不連続に変化しない。
- 内部で無制限に蓄積されるhidden goalを作らない。

## 現時点の結論

体幹操縦を単なる低優先度タスクにするのは、1人称視点操縦として不十分である。

一方で、`CHEST_JOINT2` 姿勢3自由度を硬く直接指定するだけでは、rootLink姿勢直接指定の問題を完全には解決できない。

最適な方向性は、以下の組み合わせである。

- 腕手先6自由度を `CHEST_JOINT2` 相対拘束にする。
- 体幹姿勢は主操作タスクとして扱うが、有界なソフト拘束にする。
- COM Z低下は前屈動作の補助として明示的かつ有界に入れる。
- 将来のswivel angleは手先6自由度より低い腕形状タスクとして追加する。

この設計により、体幹が操縦者の作業フレームとして機能しつつ、足接触、COM、joint limit、self collisionに対して安全側の挙動を維持できる。
