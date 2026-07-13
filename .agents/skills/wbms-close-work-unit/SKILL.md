---
name: wbms-close-work-unit
description: 実装とfresh reviewが完了したWBMS外部whole-body操縦repository sub-unitを1つだけ完了処理する。review・package build・repository状態・中央Progress同期・compatible SHA・commit readinessを確認し、明示許可されたexact repository commitだけを作成できる。機能実装、複数repository同時commit、push、merge、PR作成、別途許可のない実機実行には使用しない。
---

# WBMS Work Unit完了処理

## 目的

実装・修正・fresh reviewが完了した一つのrepository sub-unitについて、Progress更新、central Progress sync要否、compatible set、commit readinessを確認する。

このSkillは未完了の実装を隠してcommitするためのものではない。blocking条件が一つでもあればcommitせず、具体的な不足を報告する。

## Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

## 起動directoryとcommit範囲

- Codexはcommit対象repository rootから起動する。
- 一回のclosureで扱うcommit repositoryは一つだけとする。
- sibling repositoryはread-onlyとする。
- 複数repositoryを一回でstageまたはcommitしない。
- 他repository commit後の中央Progress同期は、`auto_stabilizer2`のdocument-only sub-unitとして別に扱ってよい。

## 使用条件

次が揃っている場合だけ使用する。

- Work Unit IDとtitle。
- Parent Work Unit ID。
- `${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`。
- Codex launch directory。
- repository access matrix。
- 承認済みParent/Sub-unit Contract。
- 実装結果。
- 最新diff全体に対するfresh repository review結果。
- cross-repository reviewが必要な場合はその結果。
- P0/P1/P2 findingの解消記録。
- 指定package build/check結果。
- 最新中央Progressとrepository Project Context。
- compatible input set。
- commitする場合は、userによる明示的なcommit許可、対象repository、subject。

review前、finding修正途中、Contract未承認、WRITE repositoryが複数、scopeが拡大した状態では使用しない。

## 読む順序

1. source-root `AGENTS.md`、cross-repository closureまたはcentral Progress syncの場合。
2. commit対象repository rootの`AGENTS.md`。
3. nearest package/module `AGENTS.md`。
4. repository Project Context、存在する場合。
5. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
6. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
7. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
8. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
9. 最新中央Progress。
10. Parent/Sub-unit Contract。
11. Codex WorkflowとOperator Guide。
12. implementation report。
13. 全review roundとresolution。
14. 最新diff。

## Repository状態確認

commit対象repositoryで次を取得する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
git diff --check
git diff --stat
```

必要に応じて次も確認する。

```sh
git diff -- <changed files>
git diff --cached --stat
git log -1 --oneline
```

READ repositoryでは最低限次を確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- Contractとbranch/base SHAを照合する。
- userの既存変更を保持する。
- unrelated fileをstageしない。
- generated artifact、log、cache、core dumpを含めない。
- 複数repositoryのcompatible SHAを記録する。
- Project Contextと中央Progressの参照先を照合する。

## Blocking条件

次のいずれかがあればcommit-readyではない。

- P0、P1、P2 findingが未解消。
- finding修正後のfresh reviewがない。
- Contractの主要受入条件がFAIL。
- 必須package build/checkが未実行またはFAILし、Contractで明示的に後続化されていない。
- workspace一括buildを実行していないことを理由に、package build結果を不当にFAIL扱いしている。
- 実装、Project Context、中央Progressが矛盾する。
- branch、base SHA、launch directoryが不明。
- WRITE repositoryが複数ある。
- unrelated diffが混在する。
- interfaceのproducer/consumer mappingが未照合。
- compatible dependency SHAが不明。
- schema変更に必要なcross-repository reviewがない。
- simulation/実機未確認をPASSと表記している。
- userの既存変更を上書きするおそれがある。
- commit権限の明示がないのにcommitを要求している。

simulationや実機確認が環境上できないだけでは、source commitを必ずしもblockしない。ただし`UNVERIFIED`として明記し、release-readyや実機検証済みとは扱わない。

## Commit readiness checklist

### Workspace / repository

- [ ] `${CATKIN_WORKSPACE}`と`${CATKIN_SOURCE_ROOT}`が記録されている。
- [ ] Codex launch directoryがcommit対象repository rootである。
- [ ] 正しいbranch。
- [ ] Contractのbase SHAと整合。
- [ ] WRITE repositoryが一つ。
- [ ] user変更を消していない。
- [ ] unrelated fileなし。
- [ ] dependency SHA記録済み。

### Diff

- [ ] `git diff --check` PASS。
- [ ] 全changed fileを読んだ。
- [ ] format-only大規模差分なし。
- [ ] generated artifactなし。
- [ ] cleanupと機能変更を混在させていない。
- [ ] sibling repositoryを変更していない。

### Interface

- [ ] ROS message / RTM IDL / bridge mapping一致。
- [ ] schema version確認。
- [ ] enum / task mask一致。
- [ ] frame / 単位 / quaternion順序一致。
- [ ] joint name mappingとreject条件確認。
- [ ] Parent Contractとcompatible input setに一致。

### Safety

- [ ] finite check。
- [ ] joint limit。
- [ ] stale/hold。
- [ ] mode/support gate。
- [ ] hidden goalなし。
- [ ] failure/reject path。
- [ ] 500 Hz blockingなし。
- [ ] walking preparation回帰なし。

### Verification

- [ ] Contract指定package build/check結果あり。
- [ ] 通常buildは`catkin build <package> --no-deps`を使用、または別commandの理由が記録されている。
- [ ] dependency確認で`--no-deps`を外した場合、その目的が記録されている。
- [ ] IDL/CMake変更後の`--force-cmake`実行。
- [ ] exact commandと実行directoryが記録されている。
- [ ] simulation/実機の未確認項目を`UNVERIFIED`表記。
- [ ] fresh reviewでP0/P1/P2なし。

### Documentation

- [ ] repository Work Unit reportまたはProject Context更新済み。
- [ ] 中央Progress sync要否が明示されている。
- [ ] 計画との差異と理由を記録。
- [ ] review roundとfinding resolutionを記録。
- [ ] compatible setを記録。
- [ ] next entry pointを記録。
- [ ] commit subjectが一目的を表す。

## Package build記録

次の形式で記録する。

```markdown
| execution directory | package | command | dependency scope | result |
|---|---|---|---|---|
```

workspace一括buildは要求しない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存確認時だけ:

```sh
catkin build <package-name>
```

## 中央Progress同期

中央Progressの正本:

```text
auto_stabilizer2/auto_stabilizer/docs/
  WBMSExternalWholeBodyTeleoperationProgress.md
```

### `auto_stabilizer2`内のWork Unit

同一repository内で中央Progressを更新できる場合、commit前に同じdocument scopeへ含めてよい。ただしcontrol変更と巨大な履歴整理を混ぜない。

### 他repositoryのWork Unit

`whole_body_teleop`、`rtmros_msg_bridge`、`ik_solvers2`、`prioritized_qp`では、repository commit後にcentral Progress sync sub-unitを作る。

順序:

```text
1. repository sub-unit commit
2. commit SHA取得
3. auto_stabilizer2でcentral Progress sync
4. compatible set更新
5. dependent sub-unit開始
```

依存する次sub-unitへ進む前に同期する。

## Progress追記

Progressはappend-onlyとする。過去entryを静かに修正しない。

次を含む新entryを追加する。

```markdown
## YYYY-MM-DD <Work Unit ID> <title>

### Status

### Workspace context
- catkin workspace root
- source root
- Codex launch directory

### Repository state
| repository | branch | base SHA | current SHA | access | dirty |

### Goal

### Scope and out-of-scope

### Code investigation

### Decisions
- adopted
- rejected and reason

### Changes
| repository | file | change | reason |

### Commands and results
| execution directory | command | result | evidence |

### Simulation / log evidence

### Review
| round | reviewer/task | findings | resolution |

### Acceptance
| criterion | result | evidence |

### Unverified

### Open issues

### Compatible dependency set
| repository | SHA |

### Central Progress sync
- required / completed / pending

### Next entry point

### Commit
- repository
- SHA
- subject
- cherry-pick notes
```

commit前はCommit欄を`PENDING`としてよい。他repository commitのSHAはcentral Progress sync entryで記録する。

## Commitを許可された場合

userがexact Work Unit、repository、commit subjectを明示した場合だけ、次を行う。

1. checklistを再確認する。
2. 今回Work Unitのfileだけをstageする。
3. `git diff --cached --check`を実行する。
4. `git diff --cached --stat`とstaged diffを確認する。
5. 指定subjectでatomic commitする。
6. commit SHAを取得する。
7. 完了報告へ記録する。
8. 他repositoryの場合、central Progress syncを次actionとして明示する。

次は行わない。

- `git add -A`による無差別stage。
- unrelated変更のstage。
- sibling repositoryのstage/commit。
- amend、rebase、force push。
- push、merge、PR作成。
- 実機実行。

これらは別の明示許可が必要である。

## Commit subject

良い例:

```text
Add WBMS external command schema
Implement head non-IK reference override
Connect external COM reference to static ZMP integration
Add external IK self-collision constraints
Record M1 compatible repository set
```

避ける例:

```text
Implement teleop and cleanup
Fix various WBMS issues
Update everything
```

## 必須出力

commit未許可またはblockingあり:

```markdown
# Work Unit <ID> closure

## Readiness
NOT READY / READY FOR COMMIT

## Commit repository

## Blocking items

## Checklist

## Progress / Project Context update

## Verification

## Review status

## Compatible dependency set

## Central Progress sync

## Next action
```

commit実施時:

```markdown
# Work Unit <ID> completion

## Result

## Commit
| repository | SHA | subject |

## Verification
| execution directory | command/test | result |

## Review

## Acceptance

## Unverified

## Compatible dependency set

## Central Progress sync

## Next Work Unit

## Risks / notes
```

実装完了、source検証完了、repository commit完了、central Progress同期完了、simulation検証完了、実機検証完了を区別する。
