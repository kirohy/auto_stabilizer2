# prioritized_qp repository instructions

この文書は`prioritized_qp` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

本repositoryはprioritized QP / solver backendを提供する。

本プロジェクトでは`ik_solvers2`を介して、external whole-body IKおよび既存`auto_stabilizer`から利用される可能性がある。

## 正式仕様

`docs/WBMSExternalTeleopProjectContext.md`に記録された中央計画、Revision、Progress、Parent/Sub-unit Contractを読む。

優先:

1. Implementation Plan Revision 2。
2. MultiRepositoryOperations。
3. Implementation Plan Revision 1。
4. Implementation Plan。
5. 中央Progress。
6. QP backend変更sub-unit Contract。

## Repository境界

- generic QP backendの責務を維持する。
- robot-specific task、WBMS mode、walking preparation policyを入れない。
- 既存task/priority semanticsを暗黙に変更しない。
- matrix structure、bounds、objective、external variable、warm start、statusの意味を明示する。
- external generator固有機能が必要な場合、既存consumerへ影響しない明示APIとして分離する。

## Work Unit

- Codexは本repository rootから起動する。
- 一つのimplementation taskがWRITEするrepositoryは本repositoryだけとする。
- `ik_solvers2`、`auto_stabilizer2`、`whole_body_teleop`はContractでREADとされた範囲だけ参照する。
- APIまたは数値semantics変更はParent Contractとdependent sub-unitを先に定義する。

## Solver compatibility

変更時に確認する。

- public API。
- matrix dimensionとsparsity structure。
- objective/constraint sign convention。
- lower/upper bound semantics。
- shared external variable semantics。
- warm start/cold retry。
- factorization reuse。
- structure revisionとnumeric revision。
- solver tolerance、iteration、termination status。
- infeasible/failed resultの扱い。
- thread safety。

既存consumerのdefault behaviorを変更する場合、明示的なversion/flag/APIを検討する。

## Safetyとfailure

- NaN/Inf、dimension mismatch、invalid boundを検出する。
- lower > upperをsilentに通さない。
- solver failure時に古い解を成功として返さない。
- warm-start failureとcold retryを診断可能にする。
- hard constraintをweight調整だけで暗黙にsoft化しない。
- tolerance変更を安全改善として無根拠に扱わない。

## Performance

- matrix update、factorization、solve、retryを分けて計測可能にする。
- structure reuseを壊す変更を避ける。
- 毎周期の不要なinitialize、allocation、copyを避ける。
- problem size、nonzero数、iteration、mean/p99/maxを記録する。
- benchmarkは既存consumerとexternal generatorの代表problemで行う。

## Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <prioritized_qp-package-name> --no-deps
```

依存関係まで確認する場合だけ`--no-deps`を外す。

変更が`ik_solvers2`、`auto_stabilizer`、`whole_body_teleop_reference_generator`へ影響する場合、dependent package buildを別verification stepとして指定する。

exact commandと実行directoryを記録する。

## Review

重点:

- matrix/bounds/objective mapping。
- structure cache invalidation。
- warm-start/cold-retry path。
- solver statusとfailure propagation。
- toleranceとtermination。
- external variable mapping。
- performance regression。
- existing consumer compatibility。
- dependent compatible SHA。

## Code style

- 既存repository styleに合わせる。
- `clang-format`を無断で全体適用しない。
- コメントとMarkdownは日本語。
- public APIと数値semanticsをdocumentする。
- unrelated cleanupをsolver behavior変更へ混ぜない。

## Progressとcommit

- 実装taskとは別のread-only reviewを行う。
- P0/P1/P2がなくなるまでfresh reviewする。
- package build、benchmark、dependent build、compatible SHAを記録する。
- commit後、依存するsub-unit前に中央ProgressへSHAを同期する。
- commitはユーザーの明示許可時だけ。
- push、merge、PR作成は別許可。
