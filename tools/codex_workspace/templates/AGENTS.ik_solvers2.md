# ik_solvers2 repository instructions

この文書は`ik_solvers2` repository rootの`AGENTS.md`として配置するtemplateである。

## Repository role

本repositoryはgenericなinverse kinematics libraryを提供する。

本プロジェクトでは、`whole_body_teleop_reference_generator`のexternal whole-body IKと、既存`auto_stabilizer`のIKから利用される可能性がある。

## 正式仕様

`docs/WBMSExternalTeleopProjectContext.md`に記録された中央計画、Revision、Progress、Parent/Sub-unit Contractを読む。

優先:

1. Implementation Plan Revision 2。
2. MultiRepositoryOperations。
3. Implementation Plan Revision 1。
4. Implementation Plan。
5. 中央Progress。
6. solver変更sub-unit Contract。

## Repository境界

- generic IK libraryの責務を維持する。
- JAXON、WBMS、walking preparation等のrobot/application固有policyをlibraryへ直接入れない。
- external generator固有機能が必要な場合、明示的なAPI、parameter、helperとして分離する。
- 既存`auto_stabilizer`利用者のsemanticsを暗黙に変更しない。
- priority、constraint、task、iteration、termination、update semanticsの変更を明示する。

## Work Unit

- Codexは本repository rootから起動する。
- 一つのimplementation taskがWRITEするrepositoryは本repositoryだけとする。
- `auto_stabilizer2`、`whole_body_teleop`、`prioritized_qp`はContractでREADとされた範囲だけ参照する。
- API変更はParent Contractとdependent repository sub-unitを先に定義する。
- compatibility不明のままdependent sourceを同時変更しない。

## API compatibility

変更時に確認する。

- public header。
- ABIに影響する型。
- default parameter。
- priority/task semantics。
- constraint update order。
- Jacobian/bounds update semantics。
- iteration countとtermination。
- error/status return。
- warm startとtask cache。
- dynamic allocationとthread safety。

既存consumerの挙動を変える変更は、feature flag、明示API、versioned behaviorのいずれかを検討する。

## Safetyと数値挙動

- NaN/Infを伝播させない。
- dimension mismatch、unknown variable、duplicate constraintを検出する。
- joint/Cartesian hard constraint semanticsを弱めない。
- `maxIteration`、precision、maxErrorの意味を暗黙に変えない。
- solver failureとpartial resultの扱いを明示する。
- external generatorで使う場合も、最終hardware safetyをlibraryが保証すると表現しない。

## Performance

- 既存consumerの計算量を不必要に増やさない。
- 毎周期の不要なallocation、clone、structure rebuildを避ける。
- cache reuse、bounds-only update等を追加する場合、既存pathと比較する。
- benchmark条件、problem size、iteration、mean/p99/maxを記録する。

## Build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <ik_solvers2-package-name> --no-deps
```

依存関係まで確認する場合だけ`--no-deps`を外す。

変更が`auto_stabilizer`または`whole_body_teleop_reference_generator`へ影響する場合、dependent package buildを別のverification stepとして指定する。

exact commandと実行directoryを記録する。

## Review

重点:

- existing API semantics regression。
- task/priority order。
- matrix/vector dimension。
- stale cacheとstructure change。
- solver failure path。
- NaN/Inf。
- performance regression。
- robot-specific policyの混入。
- dependent compatible SHA。

## Code style

- 既存repository styleに合わせる。
- `clang-format`を無断で全体適用しない。
- コメントとMarkdownは日本語。
- public APIの非自明なsemanticsをcomment/documentする。
- unrelated refactorをsolver behavior変更へ混ぜない。

## Progressとcommit

- 実装taskとは別のread-only reviewを行う。
- P0/P1/P2がなくなるまでfresh reviewする。
- package build、benchmark、dependent build、compatible SHAを記録する。
- commit後、依存するsub-unit前に中央ProgressへSHAを同期する。
- commitはユーザーの明示許可時だけ。
- push、merge、PR作成は別許可。
