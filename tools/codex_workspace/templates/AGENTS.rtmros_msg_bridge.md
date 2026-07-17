# rtmros_msg_bridge repository instructions

この文書は`rtmros_msg_bridge` repository rootまたは新規操縦bridge packageの`AGENTS.md`として配置するtemplateである。

## Repository role

含む:

- ROS command bundleからRTM command dataへの変換。
- RTM state dataからROS state bundleへの変換。
- finite、quaternion norm、schema version check。
- enum/task maskの一対一mapping。
- latest-only mailbox。
- invalid/drop/out-of-order/overwrite counter。
- 独立process launch。

含まない:

- IK、task scaling。
- joint/velocity/acceleration limit。
- stale policy、mode gate、walking preparation。
- COM/ZMP。
- source ownership。
- RobotHardware指令。

## 正式仕様

Project Contextに固定された中央文書を読む。

1. Implementation Plan Revision 2。
2. Codex Workflow Revision 1。
3. Current Checkpoint。
4. MultiRepositoryOperations。
5. 制御仕様のImplementation Plan / Revision。
6. protocol Parent Contract。
7. bridge sub-unit Contract。

workflowの旧gateと矛盾する場合、Workflow Revision 1を優先する。

## Repository境界

- Codexはrepository rootから起動する。
- 一つのimplementation実行がWRITEするrepositoryは本repositoryだけ。
- message/IDL schema確定後に実装する。
- schema意味変更時はParent planへ戻る。
- unrelated cleanupを混ぜない。
- user変更を無断でreset、stash、checkout、cleanしない。

## Risk

典型:

- R0: branch/AGENTS/package bootstrap。
- R1: bridge skeleton、field mapping、diagnostics、launch。
- R2: 原則なし。stale/mode/control policyをbridgeへ入れない。
- R3: なし。

R0はSELF、bootstrap package末尾にfocused review。
R1はmessage/IDL/bridge compatible set完成時にcross-repository review一回。
sub-unitごとのfull fresh reviewとProgress-only reviewを要求しない。

## Process境界

- `hrpEC`へ参加させない。
- 独立process。
- ROS callbackとRTM周期処理をlatest-value mailboxで分離。
- callback内で重い処理をしない。
- queue backlogを作らない。
- subscriber queueは原則1。
- old messageを順番に処理し続けない。

## Timestampとsequence

- source timestampを`ros::Time::now()`で上書きしない。
- bridge receive timeは別diagnostic。
- session、epoch、sequenceを保持。
- out-of-order、duplicate、schema mismatchをreject。
- 別PC clock同期を安全性の必須前提にしない。

## Mapping

Parent Contractで固定する。

- field/type。
- array/sequence長。
- unit/frame。
- quaternion順序。
- task mask/enum。
- schema version。
- session/epoch/sequence。
- timestamp semantics。
- joint name/position/velocity。

fieldをsilent dropしない。
unknown/unsupportedはrejectまたはdiagnostic。

## Thread safety

- callback threadとRTC threadの共有状態を明示。
- 無制限queueなし。
- data raceなし。
- lockはbridge process内のboundedな短時間に限定。
- AutoStabilizer 500 Hz threadを待たせない。

## Build

```sh
catkin build whole_body_teleop_rtmros_bridge --no-deps
```

message/IDL/CMake再生成が必要ならContractで`--force-cmake`。
dependency確認時だけ`--no-deps`を外す。
exact command、execution directory、resultを記録する。

## Review

compatible-set review重点:

- ROS/RTM round-tripで全field保持。
- quaternion/RPY。
- source timestamp。
- enum/mask/schema。
- out-of-order/latest-only。
- data race。
- bridgeへIK/limit/stale/modeが混入していない。
- independent process launch。
- producer/consumer SHA。

非本質修正はTARGETED follow-upでよい。
schema意味変更時だけfull compatible-set reviewをやり直す。

## Commitとcheckpoint

- R0/R1はParentのstanding authorizationを使用できる。
- 一回に本repositoryだけstage/commit。
- explicit pathだけstage。
- commit SHAを次sub-unitへ直接渡す。
- 中央Progressはcompatible set/checkpointでbatch更新。
- push、merge、PRは別許可。

## Code style

- 既存bridge style。
- `clang-format`を自動適用しない。
- コメント/Markdownは日本語。
- identifiersは英語。
- mapping責務を小さなfunction/classへ分離。
