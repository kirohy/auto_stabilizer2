---
name: wbms-close-work-unit
description: 実装とfresh reviewが完了したWBMS外部whole-body操縦Work Unitを1つだけ完了処理する。review・verification・repository状態・Progress・compatible SHA・commit readinessを確認し、明示許可されたexact commitだけを作成できる。機能実装、push、merge、PR作成、別途許可のない実機実行には使用しない。
---

# WBMS Work Unit完了処理

## 目的

実装・修正・fresh reviewが完了した一つのWork Unitについて、Progress更新とcommit readiness確認を行う。

このSkillは未完了の実装を隠してcommitするためのものではない。blocking条件が一つでもあればcommitせず、具体的な不足を報告する。

## 使用条件

次が揃っている場合だけ使用する。

- Work Unit IDとtitle。
- 承認済みWork Unit Contract。
- 実装結果。
- 最新diff全体に対するfresh review結果。
- P0/P1/P2 findingの解消記録。
- 指定build/check結果。
- 最新Progress file。
- commitする場合は、userによる明示的なcommit許可とsubject。

review前、finding修正途中、Contract未承認、scopeが拡大した状態では使用しない。

## 読む順序

1. root `AGENTS.md`。
2. nearest package/module `AGENTS.md`。
3. Implementation Plan。
4. 最新Progress。
5. Work Unit Contract。
6. Codex Workflow。
7. implementation report。
8. 全review roundとresolution。
9. 最新diff。

## Repository状態確認

全対象repositoryで次を取得する。

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

- Contractとbranch/base SHAを照合する。
- userの既存変更を保持する。
- unrelated fileをstageしない。
- generated artifact、log、cache、core dumpを含めない。
- 複数repositoryのcompatible SHAを記録する。

## Blocking条件

次のいずれかがあればcommit-readyではない。

- P0、P1、P2 findingが未解消。
- finding修正後のfresh reviewがない。
- Contractの主要受入条件がFAIL。
- 必須build/checkが未実行またはFAILし、Contractで明示的に後続化されていない。
- 実装とProgressが矛盾する。
- branchまたはbase SHAが不明。
- unrelated diffが混在する。
- interfaceのproducer/consumer mappingが未照合。
- compatible dependency SHAが不明。
- simulation/実機未確認をPASSと表記している。
- userの既存変更を上書きするおそれがある。
- commit権限の明示がないのにcommitを要求している。

simulationや実機確認が環境上できないだけでは、source commitを必ずしもblockしない。ただし`UNVERIFIED`として明記し、release-readyや実機検証済みとは扱わない。

## Commit readiness checklist

### Repository

- [ ] 正しいbranch。
- [ ] Contractのbase SHAと整合。
- [ ] user変更を消していない。
- [ ] unrelated fileなし。
- [ ] dependency SHA記録済み。

### Diff

- [ ] `git diff --check` PASS。
- [ ] 全changed fileを読んだ。
- [ ] format-only大規模差分なし。
- [ ] generated artifactなし。
- [ ] cleanupと機能変更を混在させていない。

### Interface

- [ ] ROS message / RTM IDL / bridge mapping一致。
- [ ] schema version確認。
- [ ] enum / task mask一致。
- [ ] frame / 単位 / quaternion順序一致。
- [ ] joint name mappingとreject条件確認。

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

- [ ] Contract指定build/check結果あり。
- [ ] IDL変更後の`--force-cmake`実行。
- [ ] dependency変更時のdependent build実行。
- [ ] simulation/実機の未確認項目を`UNVERIFIED`表記。
- [ ] fresh reviewでP0/P1/P2なし。

### Documentation

- [ ] Progress追記済み。
- [ ] 計画との差異と理由を記録。
- [ ] review roundとfinding resolutionを記録。
- [ ] compatible setを記録。
- [ ] next entry pointを記録。
- [ ] commit subjectが一目的を表す。

## Progress追記

Progressはappend-onlyとする。過去entryを静かに修正しない。

次を含む新entryを追加する。

```markdown
## YYYY-MM-DD <Work Unit ID> <title>

### Status

### Repository state
| repository | branch | base SHA | current SHA | dirty |

### Goal

### Scope and out-of-scope

### Code investigation

### Decisions
- adopted
- rejected and reason

### Changes
| file | change | reason |

### Commands and results
| command | result | evidence |

### Simulation / log evidence

### Review
| round | reviewer/task | findings | resolution |

### Acceptance
| criterion | result | evidence |

### Unverified

### Open issues

### Compatible dependency set
| repository | SHA |

### Next entry point

### Commit
- SHA
- subject
- cherry-pick notes
```

commit前はCommit欄を`PENDING`としてよい。commit後、同じ作業中に追記できる場合はSHAを記録する。Progress更新を別commitに分ける場合は、Work Unitとの関係を明記する。

## Commitを許可された場合

userがexact Work Unitとcommit subjectを明示した場合だけ、次を行う。

1. checklistを再確認する。
2. 今回Work Unitのfileだけをstageする。
3. `git diff --cached --check`を実行する。
4. `git diff --cached --stat`とstaged diffを確認する。
5. 指定subjectでatomic commitする。
6. commit SHAを取得する。
7. 完了報告へ記録する。

次は行わない。

- `git add -A`による無差別stage。
- unrelated変更のstage。
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

## Blocking items

## Checklist

## Progress update

## Verification

## Review status

## Compatible dependency set

## Next action
```

commit実施時:

```markdown
# Work Unit <ID> completion

## Result

## Commit
| repository | SHA | subject |

## Verification
| command/test | result |

## Review

## Acceptance

## Unverified

## Compatible dependency set

## Next Work Unit

## Risks / notes
```

実装完了、source検証完了、simulation検証完了、実機検証完了を区別する。