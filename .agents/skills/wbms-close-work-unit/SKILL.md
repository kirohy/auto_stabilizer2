---
name: wbms-close-work-unit
description: WBMS外部whole-body操縦のrepository sub-unitまたはParent Work Packageをrisk-basedに完了処理する。verification evidenceを再利用し、R0/R1の不要なfresh reviewやProgress-only gateを省き、R3・compatible set・milestoneでは厳格なclosureを行う。明示許可されたlocal commit以外のpush、merge、PR作成、simulation、実機実行には使用しない。
---

# WBMS Risk-based Closure

## 目的

次のいずれかを完了処理する。

- repository sub-unit commit。
- Parent Work Package checkpoint。
- compatible set freeze。
- milestone closure。
- R3 safety closure。

一つのclosureでstage/commitするrepositoryは一つだけとする。

最初に読む。

1. nearest `AGENTS.md` chain。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
3. `WBMSExternalWholeBodyTeleoperationCodexWorkflowRevision1.md`。
4. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`。
5. Parent Work Package / Contract。
6. implementation report。
7. review result。
8. verification evidence。
9. 必要なProgress履歴。

## Risk別closure

### R0

必要:

- scope/path確認。
- `git diff --check`またはcached check。
- 必要な構文/package discovery。
- SELF review。
- commit authorization。

不要:

- detached review。
- Progress-only fresh review。
- 中央Progress entry。
- package build。
- 関連control source再読。

Parent末尾のfocused reviewとCurrent Checkpoint更新へbatchしてよい。

### R1

必要:

- targeted package build。
- schema/mapping SELF確認。
- repository commit authorization。
- compatible set完成時のCOMPATIBLE_SET review。

各repository commit前のfull reviewは不要。

### R2

必要:

- package build。
- REPOSITORY_FULL review。
- P0/P1/P2解消。
- 非本質修正はTARGETED follow-up。
- commit authorization。

material修正ならfull fresh review。

### R3

必要:

- full safety checklist。
- package build。
- SAFETY_FULL review。
- P0/P1/P2解消。
- material修正後full fresh review。
- simulation状態。
- exact人間commit承認。

## Evidence再利用

verification/reviewを次で識別する。

- verified commit SHA。
- uncommitted diff hash。
- build command/result。
- review type/result。

commit前のsource diff identityがreview/build時と同じなら、
source再読、build再実行、full review再実行を要求しない。

Progress、Current Checkpoint、Markdownだけの変更はsource evidenceを無効化しない。

source diffが変わった場合は影響に応じて必要checkだけ再実行する。

## Repository状態

commit対象repository:

```sh
git status --short
git branch --show-current
git rev-parse HEAD
git diff --check
git diff --stat
```

commit直前:

```sh
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
```

確認する。

- branch。
- expected parent/base。
- stage対象。
- unrelated/generated fileなし。
- user変更なし。
- one-write-repository。
- commit subject。

READ sibling repositoryは、compatible SHAが必要な場合だけbranch/HEAD/dirty stateを確認する。
毎回全repositoryを再走査しない。

## Blocking条件

### 全risk共通

- scope外diff。
- user変更との衝突。
- WRITE repository複数。
- unrelated/generated artifact。
- required verification FAIL。
- commit authorizationなし。
- destructive Git操作が必要。
- compatible SHA不明、必要な場合。
- simulation/実機をPASS扱い。

### R1追加

- Parent schema/mappingと不一致。
- compatible-set reviewが必要なのに未実施。

### R2追加

- P0/P1/P2未解消。
- repository reviewなし。

### R3追加

- safety full reviewなし。
- material修正後full fresh reviewなし。
- simulation gate未処理。
- safety invariant不明。

R0でProgress-only detached reviewがないことはblocking条件ではない。

## Commit authorization

### exact

ユーザーがWork Unit、repository、subjectを指定した一commit。

### standing

Parent Work Packageで事前許可されたR0〜R2 local commit。

standing authorizationでcommit可能な条件:

- Parent scope内。
- risk R0〜R2。
- 一回に一repository。
- required check/review PASS。
- stop conditionなし。
- push、merge、PRなし。

R3はstanding authorizationでcommitしない。

## Commit手順

1. explicit pathだけstage。
2. cached diff check。
3. cached name/stat確認。
4. subject確認。
5. atomic commit。
6. commit SHA取得。
7. Current Checkpoint/Parent working stateへSHAを記録。
8. next eligible sub-unitへexact SHAを渡す。

`git add -A`を既定にしない。
amend、rebase、force pushを行わない。

## Progress policy

### Current Checkpoint

sub-unit完了後、現在地、commit SHA、next actionを更新してよい。
これはmutableであり、中央Progress entryを毎回作る必要はない。

### 中央Progress

次の場合に一件追加する。

- Parent Work Package完了。
- compatible set確定。
- architecture decision変更。
- simulation/実機結果。
- blocking issue。
- milestone完了。
- 引き継ぎ。
- release candidate。

repository commitごとのProgress-only sub-unitを通常作らない。

## Parent closure

Parent Work Package末尾で確認する。

- expected sub-unit完了。
- repository commit SHA。
- compatible set。
- required review。
- package build。
- Current Checkpoint。
- 中央Progress checkpoint要否。
- simulation/hardware gate。
- next Parent Work Package。

## 必須出力

commitしない場合:

```markdown
# Closure

## Readiness
READY / NOT READY

## Risk
## Evidence identity
## Blocking items
## Required checks
## Review status
## Commit authorization
## Current Checkpoint update
## Next mandatory action
```

commitした場合:

```markdown
# Completion

## Commit
| repository | SHA | subject |

## Risk
## Verification evidence
## Review
## Compatible set
## Current Checkpoint
## Progress checkpoint
## Next Work Package
## Unverified
```

## Stop conditions

- riskがR3へ上昇。
- schema/safety invariant変更。
- sibling WRITEが必要。
- user変更と衝突。
- destructive Git操作。
- build failure修正先が別repository。
- simulation/hardware。
- compatible SHA移動。

push、merge、PR作成、無許可simulation・実機実行は行わない。
