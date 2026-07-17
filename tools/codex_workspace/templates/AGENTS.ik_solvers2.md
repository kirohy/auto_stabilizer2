# ik_solvers2 repository instructions

この文書は`ik_solvers2` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

- generic IK constraint/solver library。
- external whole-body IKと既存`auto_stabilizer`の双方から利用される。
- robot-specific teleoperation policyをlibraryへ埋め込まない。
- 既存API semanticsを暗黙に変更しない。

## 正式仕様

Project Contextに固定された中央文書を読む。

1. Implementation Plan Revision 2。
2. Codex Workflow Revision 1。
3. Current Checkpoint。
4. MultiRepositoryOperations。
5. Parent Work Package / Contract。
6. repository sub-unit Contract。

workflowの旧gateと矛盾する場合、Workflow Revision 1を優先する。

## Repository境界

- Codexはrepository rootから起動する。
- 一つのimplementation実行がWRITEするrepositoryは本repositoryだけ。
- sibling repositoryは明示SHAをREAD。
- dependent consumer変更を同じtaskで行わない。
- user変更を無断でreset、stash、checkout、cleanしない。

## Risk

典型:

- R0: AGENTS、Project Context、文書。
- R1: 明示API、constraint interface、diagnostic field。
- R2: solver behavior、Jacobian/bounds更新、constraint実装。
- R3: 既存consumerの安全挙動を変えるsemantic change。R3として扱い人間gateへ戻す。

Review:

- R0: SELF。
- R1: dependent compatible-set review。
- R2: repository full review、benchmark/build。
- R3: safety full reviewとdependent simulation計画。

## API互換

- 既存`auto_stabilizer`APIを暗黙に変えない。
- external generator専用機能は明示API、parameter、featureとして分離。
- default behaviorを変更する場合は全consumerを列挙。
- task priority、slack、tolerance、update semanticsを文書化。
- bounds-only/Jacobian update等は古いcall pathを壊さない。
- failure/status semanticsを変える場合はParent Contractへ戻る。

## Safety・数値

- finite check。
- dimension consistency。
- unknown/duplicate joint。
- Jacobianとboundsの同じstate/version。
- solver failureを成功扱いしない。
- warm start/cold retry/statusを切り分ける。
- constraintをsilent dropしない。
- numerical tolerance変更はdependent behaviorへ影響するためR2以上。

## Build

workspace一括buildを標準にしない。
変更packageを`--no-deps`でbuildし、dependent compatibilityを確認する場合だけ依存buildを追加する。

例:

```sh
catkin build ik_constraint2 --no-deps
catkin build ik_constraint2_joint_limit_table --no-deps
catkin build prioritized_inverse_kinematics_solver2 --no-deps
```

exact command、execution directory、resultを記録する。

## Review・evidence

- API/header変更はconsumer mappingを確認。
- behavior変更は代表problem/benchmarkを記録。
- reviewed commit SHAまたはdiff hashを保存。
- Progress/Markdown追記だけでsource reviewを無効化しない。
- 非本質修正はTARGETED follow-up。
- material semantic変更はfull repository review。

## Commitとcheckpoint

- R0〜R2はParentのstanding authorizationを使用できる。
- 一回に本repositoryだけstage/commit。
- explicit pathだけstage。
- dependent consumerを同じcommitへ混ぜない。
- commit SHAをcompatible setへ渡す。
- 中央Progressはcompatible-set/checkpointで更新。
- push、merge、PR、simulation、実機は別許可。

## Code style

- generic責務を維持。
- robot-specific assumptionを避ける。
- 既存style。
- `clang-format`を自動適用しない。
- コメント/Markdownは日本語。
- identifiersは英語。
