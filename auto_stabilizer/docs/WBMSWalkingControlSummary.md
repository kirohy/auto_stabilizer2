# WBMS中の上半身操縦と歩行制御の整理

## 現行優先度に関する注記

本書は、WBMS中の上半身操縦と歩行制御を整理した初期作業の引き継ぎ資料である。腕操縦を体幹相対にする背景、WBMS中に脚を直接操縦しない方針、歩行開始遅延を導入した経緯を理解するために残す。

後続作業により、歩行準備遷移の仕様は `WBMSWalkingPreparationTransitionImplementationPlan.md` および `WBMSWalkingPreparationDesignRevisionPlan.md` で更新されている。特に、WBMS中かつ歩行準備READYでない場合の歩行API受付、pending commandの扱い、READY判定、pre-walk姿勢生成については、本書ではなく `WBMSWalkingPreparationDesignRevisionPlan.md` を優先する。

現行仕様・実装計画を判断する場合は、`WBMSFeasibleVelocityPostureControlProgress.md` の参照順を優先する。本書は、歩行準備遷移の最新仕様ではなく、初期の上半身操縦/歩行制御整理と実装経緯の背景資料として読む。

## この文書の目的

この文書は、本スレッドで行った `startWholeBodyMasterSlave()` 起動後の操縦モードに関する設計・実装内容を、別スレッドの Codex や後日の作業者が読み込んで理解できるようにまとめたものである。

扱った主題は、WBMS中の腕操縦と歩行指令を安全に同時実行するための制御整理である。特に以下を対象にした。

- WBMSで直接操縦する対象を上半身に限定する。
- 腕の操縦を体幹相対にする。
- WBMS中に歩行を開始するとき、IK重みの復帰とfootstep生成のタイミングを分離する。
- 実装上の責務を整理し、`AutoStabilizer` に後付けの状態を直置きしない。

## 背景と初期状態の問題

WBMSは、起動時のエンドエフェクタ姿勢の指令値とロボット現在値を保存し、それらを基準値として、以降の指令値との差分から目標姿勢を計算する仕組みである。

初期状態では、`startWholeBodyMasterSlave()` 起動後に送られてくる両手両足の指令値に従って姿勢を生成していた。しかし、この設計ではWBMSの指令系が脚にも直接影響するため、歩行指令系と役割が重なる。

今回の方針では、脚の操作は既存の歩行APIである `goVelocity()`、`goPos()`、`setFootSteps()` などを外部から起動することで行い、WBMSは上半身、特に腕の操縦に限定することにした。

また、腕の操縦に関して、従来は両足中心またはfoot originに近い基準で手先位置を維持するような挙動になっていた。例えばその場足踏みの歩行指令を送ると、本来は腕が上半身の揺れに追従してほしいが、実際には手先位置をその場に維持しようとして、体幹から見た腕姿勢が変化していた。

このため、腕操縦の基準を体幹に変更した。

## 目標と確認済み挙動

本スレッドで目標とした挙動は以下である。

- WBMS中の直接操縦対象は上半身エンドエフェクタのみとする。
- 脚はWBMS指令では直接動かさず、通常の歩行生成系に任せる。
- 腕の目標は体幹相対で計算する。
- 腕操縦と歩行指令を同時に実行できる。
- WBMS中に歩行開始しても、開始瞬間に下半身が不連続に振動しない。

シミュレータでは以下が確認された。

- 腕の操縦と歩行指令を同時に実行できた。
- 腕の操縦が体幹相対になっていることが確認された。
- 以前観測されていた、`goVelocity(0.0, 0.0, 0.0)` 送信直後の下半身の一瞬の振動は見られなくなった。

## 上半身のみをWBMS対象にする設計

WBMSで両手両足を直接操縦すると、脚の目標がWBMS指令とfootstep生成の両方から与えられる可能性がある。これは歩行時の責務が曖昧になり、制御の破綻や予期しない干渉につながる。

そこで、WBMSの直接操縦対象は上半身エンドエフェクタに限定した。

設計上の分担は以下である。

- WBMS: 腕など上半身エンドエフェクタの姿勢差分を扱う。
- `FootStepGenerator`: `goVelocity()`、`goPos()`、`setFootSteps()` に基づいて脚のfuture stepを生成する。
- `LegCoordsGenerator`: footstep列に従って脚軌道を生成する。

この分担により、WBMS中でも脚の動作は従来の歩行制御系だけを通る。

## 腕操縦を体幹相対にする設計

腕の操縦では、WBMS起動時のmaster側姿勢とslave側姿勢を保存し、以降のmaster側の差分をslave側へ反映する。

下半身を歩行系に任せる場合、腕の基準を足裏中心やworld寄りにすると、歩行中に体幹が揺れたとき手先が空間上に固定されるような挙動になる。今回の目的は、体幹から見た腕姿勢を維持・操作することであるため、上半身エンドエフェクタのslave側基準姿勢は体幹リンク相対で保存する。

体幹リンク名はRTC propertyの `torso_link_name` から読む。読み込んだ名前は初期化時にロボットモデルに存在するか検証し、`GaitParam::chestLinkName` に保存する。

`startWholeBodyMasterSlave()` では、上半身エンドエフェクタについて以下の考え方で基準姿勢を保存する。

```cpp
torsoReferenceLink->T().inverse() * refEEPose[i]
```

これにより、歩行中に体幹が動いた場合でも、腕は体幹に対する相対姿勢として扱われる。

## WBMS中の歩行安定化IK重み

WBMS中は、腕操縦や体幹姿勢指令の自由度を確保するため、root linkのIK重みを下げる設計が入っていた。

しかし、その状態で歩行を行うと、腰から上が左右に振り子のように揺れる挙動が出た。歩行中はroot姿勢やCOM Z方向の拘束をある程度強くしないと、歩行に必要な姿勢安定性が不足する。

一方で、常にroot姿勢を強く固定すると、床のものを拾うような上半身をかがめる動作ができなくなる。

今回の妥協点は以下である。

- 静止中のWBMSでは、root姿勢やCOM Z拘束を弱めて上半身操作の自由度を確保する。
- 歩行開始前に、`wbms_walking_stability_start_time` をかけて歩行安定化用の重みを強める。
- 歩行終了後に静止へ戻ったら、`wbms_walking_stability_stop_time` をかけて重みを弱める。

この切り替えは `FullbodyIKSolver` 側の `wbmsWalkingStabilityMode` で扱う。

## 歩行開始遅延を導入した理由

単に「歩行開始と同時にIK重みを強くする」方式では、ロボットがかがんだ姿勢など歩行に適さない姿勢にある場合、footstep生成とIK重みの復帰が同時に走り、IKが破綻する危険がある。

そのため、歩行指令を受け取ったらすぐにfootstepを生成するのではなく、まず歩行安定化用のIK重みを復帰させ、その後に歩行を開始する方針にした。

初期案では、生成された `footstepNodesList` の先頭nodeの `remainTime` を `walkingStabilityTime` 分だけ延ばすことを考えた。しかしシミュレータで確認すると、`goVelocity(0.0, 0.0, 0.0)` を送信した瞬間に下半身が一瞬振動し、その後に設定時間だけ遅れて足踏みが始まる挙動が出た。

この振動の原因として、footstep開始自体は遅らせていても、future step列が即座に生成されていたことが考えられる。future step列が存在することで、後段の処理が「歩行予定あり」の状態を見て反応し、下半身に小さな不連続が出た可能性が高い。

そこで現在の実装では、歩行開始遅延中はfuture step列を生成しない。

## 現在の歩行開始遅延の挙動

WBMS中かつstatic状態で `goVelocity()`、`goPos()`、`setFootSteps()` を受け取った場合、通常のfootstep生成へすぐ渡さない。

代わりに以下を行う。

1. 受け取った歩行指令をpending commandとして保存する。
2. `GaitParam::isWbmsWalkingStartDelay` をtrueにする。
3. `GaitParam::wbmsWalkingStartDelayRemainTime` に `wbmsWalkingStabilityStartTime` をセットする。
4. 遅延中は `footstepNodesList` をstaticのまま維持する。
5. `FullbodyIKSolver` は `isWbmsWalkingStartDelay` を見て、歩行中と同じようにroot姿勢とCOM Z拘束を復帰させる。
6. 遅延時間が0になったら、保存していた歩行指令を通常の歩行生成系へ投入する。

これにより、「姿勢安定化のための重み復帰」と「future footstepの生成」が時間的に分離される。

## `WbmsWalkingCommandDelay` の役割

当初、pending commandの状態と処理は `AutoStabilizer.h` に直接追加していた。しかし、既存コードの構成から見ると、`AutoStabilizer` に個別制御ロジックの状態が増えるのは後付け感が強かった。

このリポジトリでは、`CmdVelGenerator`、`FootStepGenerator`、`LegCoordsGenerator`、`FullbodyIKSolver` のように、特定の責務を小さなクラスへ分ける構成になっている。

そのため、pending walking commandの管理は `WbmsWalkingCommandDelay` という専用クラスへ切り出した。

`WbmsWalkingCommandDelay` が保持するものは以下である。

- pending commandの種類
  - none
  - `goVelocity`
  - `goPos`
  - `setFootSteps`
- pending中の速度指令
- pending中のgoPos指令
- pending中のfootstep列

提供する主な関数は以下である。

- `shouldDelay(const GaitParam&)`
- `storeGoVelocity(...)`
- `storeGoPos(...)`
- `storeFootSteps(...)`
- `proc(...)`
- `clear(...)`
- `hasPendingCommand()`

`AutoStabilizer` は、サービスAPIの入口で入力値の検証とmutex管理を行い、WBMS歩行開始遅延が必要な場合は `WbmsWalkingCommandDelay` に処理を委譲する。

## `GaitParam` に残した状態

pending commandの中身は `GaitParam` へ入れなかった。

理由は、pending commandは歩容状態そのものではなく、サービス呼び出しを一時的に保留するための制御状態だからである。`GaitParam` はすでに多数のモジュールから共有されているため、ここに指令キューの中身まで入れると責務がさらに曖昧になる。

一方で、以下の2つは `GaitParam` に残した。

- `isWbmsWalkingStartDelay`
- `wbmsWalkingStartDelayRemainTime`

これらは `FullbodyIKSolver` や `waitFootSteps()` が参照する共有状態であり、「現在、歩行開始前の姿勢復帰待ちである」という制御状態として扱う必要があるためである。

## `AutoStabilizer` 側の処理

`goVelocity()`、`goPos()`、`setFootStepsWithParam()` では、まず通常通り入力値のfiniteチェックなどを行う。

その後、以下の条件でWBMS歩行開始遅延へ入る。

- 現在staticである。
- WBMSが有効、または有効へ遷移中である。
- `wbmsWalkingStabilityStartTime` が0より大きい。

遅延が必要な場合は、footstep生成を行わず、`WbmsWalkingCommandDelay` に指令を保存してreturnする。

`onExecute()` では、通常の `execAutoStabilizer()` の前に `WbmsWalkingCommandDelay::proc()` を呼ぶ。ここで遅延時間を進め、0になったらpending commandを実際の歩行生成系へ渡す。

`goStop()` では、pending commandと遅延状態を破棄する。すでに `goVelocity` による歩行が開始済みの場合のみ、従来通り `FootStepGenerator::goStop()` を呼ぶ。まだ遅延中でfootstepを生成していない場合は、単にpending commandをキャンセルする。

`stopAutoBalancer()` や `MODE_SYNC_TO_ABC` 初期化時にも、pending commandが残らないように `clear()` を呼ぶ。

## `FullbodyIKSolver` 側の処理

`FullbodyIKSolver` では、歩行安定化モードの目標値を以下のように考える。

- `!gaitParam.isStatic()` のときは歩行中なので有効。
- `gaitParam.isWbmsWalkingStartDelay` のときも、これから歩行を開始するため有効。
- staticかつ遅延中でないときは無効。

つまり、footstep列がまだstaticであっても、歩行開始待ちの間はroot姿勢とCOM Z拘束を歩行用に復帰させる。

この設計により、future stepを生成する前にIK重みだけを安全側へ寄せられる。

## パラメータ

今回の作業に関連する主なパラメータは以下である。

- `wbms_interpolate_duration`
- `wbms_walking_stability_start_time`
- `wbms_walking_stability_stop_time`

`wbms_walking_stability_start_time` は、WBMS中にstatic状態から歩行を開始するとき、footstep生成を遅らせて姿勢安定化重みを復帰させる時間である。

`wbms_walking_stability_stop_time` は、歩行終了後にstaticへ戻ったとき、歩行安定化重みを弱める時間である。

これらはIDLに追加され、`setAutoStabilizerParam()` と `getAutoStabilizerParam()` から読み書きできるようにした。

## 重要な仕様

現在の仕様は以下である。

- 遅延中に保持されるpending walking commandは1つだけである。
- 遅延中に新しい `goVelocity()`、`goPos()`、`setFootSteps()` が来た場合、最後の指令で上書きする。
- 遅延中に `goStop()` が来た場合、pending commandを破棄し、歩行は開始しない。
- 遅延中は `footstepNodesList` をstaticのまま維持する。
- 遅延中でも `FullbodyIKSolver` の歩行安定化モードは有効化される。
- WBMSが無効化された場合は、遅延時間を0へ進めてpending処理を解決する。

## 主な変更ファイル

本スレッドで中心的に扱ったファイルは以下である。

- `AutoStabilizer.cpp`
- `AutoStabilizer.h`
- `GaitParam.h`
- `FullbodyIKSolver.cpp`
- `FullbodyIKSolver.h`
- `RefToGenFrameConverter.cpp`
- `RefToGenFrameConverter.h`
- `WbmsWalkingCommandDelay.cpp`
- `WbmsWalkingCommandDelay.h`
- `AutoStabilizerService.idl`
- `CMakeLists.txt`

## ビルド確認

以下でビルド確認を行った。

```sh
catkin build auto_stabilizer
```

ビルドは成功した。残っているwarningは依存パッケージやOpenRTM helper script由来のものであり、今回追加したWBMS歩行開始遅延処理に直接起因するものではない。

## 今後見るべき点

今回の実装では、「かがんだ姿勢で歩行指令を送る」ケースに対して、まず歩行安定化重みを復帰させてから歩行を開始することで安全側に寄せている。

ただし、極端に歩行に適さない姿勢からの復帰が常に可能とは限らない。将来的に実機で扱う場合は、歩行開始前にroot姿勢、COM高さ、股関節まわりの余裕などを評価し、歩行開始を拒否する条件や、ユーザへ警告する条件を追加する余地がある。
