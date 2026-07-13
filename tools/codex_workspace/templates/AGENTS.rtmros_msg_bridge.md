# rtmros_msg_bridge repository instructions

この文書は`rtmros_msg_bridge` repository rootまたは新規操縦bridge packageの`AGENTS.md`として配置するtemplateである。

## Repository role

本repositoryはROS messageとRTM IDL dataの変換を担当する。

含む責務:

- ROS custom command bundleからRTM command dataへの変換。
- RTM state dataからROS custom state bundleへの変換。
- finite check。
- quaternion norm check。
- schema version check。
- enum/task maskの一対一mapping。
- latest-only mailbox。
- invalid、drop、out-of-order、overwrite counter。
- 独立processとしてのlaunch。

含まない責務:

- IK。
- task scaling。
- joint/velocity/acceleration limit。
- stale policy。
- mode gate。
- walking preparation。
- COM/ZMP処理。
- source ownership。
- RobotHardware指令。

## 正式仕様

`docs/WBMSExternalTeleopProjectContext.md`に記録された中央計画、Revision、Progress、Parent Contractを読む。

優先:

1. Implementation Plan Revision 2。
2. MultiRepositoryOperations。
3. Implementation Plan Revision 1。
4. Implementation Plan。
5. 中央Progress。
6. protocol Parent Contract。
7. bridge sub-unit Contract。

## Work Unit

- Codexは本repository rootから起動する。
- 一つのimplementation taskがWRITEするrepositoryは本repositoryだけとする。
- ROS message schemaとRTM IDL schemaがParent Contractで確定してから実装する。
- schema変更が必要になった場合、producer/consumerへ暗黙に合わせずParent planへ戻る。
- unrelatedな既存bridge cleanupを新機能commitへ混ぜない。

## Process境界

- 新bridgeは`hrpEC`へ参加させない。
- 独立processとして動かす。
- ROS callbackとRTM周期処理の間はlatest-value mailboxで分離する。
- callback内で重い処理を行わない。
- queue backlogを作らない。
- subscriber queueは原則1。
- old messageを順番に処理し続けない。

## Timestampとsequence

- source timestampを`ros::Time::now()`で上書きしない。
- bridge receive timeは別diagnosticとして保持してよい。
- session、epoch、sequenceを保持する。
- out-of-order、duplicate、schema mismatchをrejectする。
- HMD等の別PC clock同期を安全性の必須前提にしない。

## Mapping

ROS message、RTM IDL、bridge実装のmapping表をContractで固定する。

必ず照合する。

- field名と型。
- array/sequence長。
- 単位。
- coordinate frame。
- quaternion順序。
- task mask bit。
- enum値。
- schema version。
- session、epoch、sequence。
- timestamp semantics。
- joint name、position、velocity配列。

fieldをsilent dropしない。未対応fieldやunknown enumは明示的にrejectまたはdiagnosticを出す。

## Thread safety

- callback threadとRTC execution threadの共有状態を明示する。
- 無制限queueを使わない。
- data raceを作らない。
- lockを使う場合、bridge process内のboundedな短時間処理に限定する。
- AutoStabilizerの500 Hz threadを待たせる仕組みを作らない。

## Build

workspace一括buildを標準にしない。

package名確定後、通常は次を使う。

```sh
catkin build whole_body_teleop_rtmros_bridge --no-deps
```

message/IDL/CMake再生成が必要な場合はContractで`--force-cmake`を指定する。

依存関係まで確認する場合だけ`--no-deps`を外す。

exact commandと実行directoryを記録する。

## Review

重点:

- ROS/RTM round-tripで全fieldが保持されるか。
- quaternionとRPYの変換誤り。
- source timestampの上書き。
- enum/task mask/schema mismatch。
- out-of-orderとlatest-only。
- callbackとRTM portのdata race。
- bridgeへIK、limit、stale policyが混入していないか。
- independent process launch。

## Code style

- 既存bridgeのstyleに合わせる。
- `clang-format`を自動適用しない。
- コメントとMarkdownは日本語。
- class、function、variable、topic、message fieldは英語。
- 変換責務を小さなfunction/classへ分離する。

## Progressとcommit

- 実装taskとは別のread-only reviewを行う。
- P0/P1/P2がなくなるまでfresh reviewする。
- ROS message、IDL、bridgeのcompatible SHAを記録する。
- commit後、依存するconsumer/producer sub-unit前に中央ProgressへSHAを同期する。
- commitはユーザーの明示許可時だけ。
- push、merge、PR作成は別許可。
