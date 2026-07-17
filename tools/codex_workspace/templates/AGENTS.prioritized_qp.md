# prioritized_qp repository instructions

この文書は`prioritized_qp` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

- prioritized QP backend。
- task semantics、matrix、bounds、warm start、solver statusを提供する。
- robot-specific teleoperation policyを入れない。
- 既存consumerの数値挙動を暗黙に変更しない。

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

- Codexはrepository rootから起動。
- 一つのimplementation実行がWRITEするrepositoryは本repositoryだけ。
- sibling consumerは明示SHAをREAD。
- consumer修正を同じtaskで行わない。
- user変更を無断でreset、stash、checkout、cleanしない。

## Risk

典型:

- R0: 文書、Project Context。
- R1: status/diagnostic API、非semantic scaffolding。
- R2: matrix update、warm start、solver interface、performance。
- R3: feasibility、constraint semantics、default tolerance等で実機安全へ影響する変更。

Review:

- R0: SELF。
- R1: compatible-set review。
- R2: repository full review、benchmark、dependent build。
- R3: safety full reviewとdependent simulation plan。

## Solver互換

- priority、slack、hard/soft constraint semanticsを維持。
- matrix/vector dimensionを検証。
- bounds順序とtask mappingを維持。
- warm start failureとcold retryを区別。
- solver statusをsilentに成功へ変換しない。
- default tolerance、iteration、rho等の変更は明示する。
- matrix reuse/bounds-only updateはrevision/version管理を持つ。
- API変更時は全consumerを列挙する。

## Performance

- allocation、factorization、matrix update、iterationを計測可能にする。
- benchmark条件、problem size、solver optionを記録。
- performance改善で安全constraintやfailure checkを削除しない。
- first-order/dense等backend変更は別Work Package。

## Build

workspace一括buildを標準にしない。

```sh
catkin build prioritized_qp_base --no-deps
catkin build prioritized_qp_osqp --no-deps
```

dependent compatibilityが必要な場合だけ`--no-deps`を外すかconsumer buildを追加する。
exact command、execution directory、resultを記録する。

## Review・evidence

- API/header変更。
- matrix/bounds mapping。
- solver status/failure。
- numerical tolerance。
- thread safety。
- allocation/performance。
- dependent package behavior。

reviewed commit SHAまたはdiff hashを保存する。
Progress/Markdown追記だけでsource reviewを無効化しない。
非本質修正はTARGETED、material solver changeはfull review。

## Commitとcheckpoint

- R0〜R2はParent standing authorizationを使用可能。
- 一回に本repositoryだけstage/commit。
- explicit pathだけstage。
- consumer変更を混ぜない。
- commit SHAをcompatible setへ渡す。
- 中央Progressはcompatible-set/checkpointで更新。
- push、merge、PR、simulation、実機は別許可。

## Code style

- generic library責務。
- 既存style。
- `clang-format`を自動適用しない。
- コメント/Markdownは日本語。
- identifiersは英語。
