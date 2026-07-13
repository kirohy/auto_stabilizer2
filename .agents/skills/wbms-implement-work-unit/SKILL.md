---
name: wbms-implement-work-unit
description: Implement exactly one approved WBMS external whole-body teleoperation Work Unit. Use after a Work Unit Contract has been frozen and the task requires scoped source changes, prescribed verification, and an implementation report. Do not use for planning, detached review, progress-only closure, commit, push, merge, PR creation, simulation, or real-robot execution unless separately authorized.
---

# WBMS Work Unit実装

## 目的

承認済みのWork Unit Contractを、一つの主要責務に限定して実装する。

このSkillは実装、指定されたbuild/check、自己点検までを担当する。commit、push、merge、PR作成は行わない。

## 必須入力

実装開始前に次を特定する。

- Work Unit IDとtitle。
- 対象repository、branch、base SHA、現在HEAD。
- 承認済みWork Unit Contract。
- 正式Implementation PlanとRevision。
- 最新Progress entry。
- 実行すべきverification command。
- 変更を許可されたfile。

Contractがない、branchやbase SHAが一致しない、scopeが曖昧、または実装を左右する未決定事項がある場合はsourceを変更しない。

## 読む順序

`auto_stabilizer/`以下では次を順に読む。

1. repository rootの`AGENTS.md`。
2. `auto_stabilizer/AGENTS.md`。
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`。
6. 対象Work Unit Contract。
7. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
8. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
9. 必要な既存正式文書。

他repositoryでもnearest `AGENTS.md`、Revision、Implementation Plan、Progress、Contractを先に読む。

## Repository状態確認

全対象repositoryで次を確認して記録する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- userの未commit変更を消さない。
- 無断で`reset`、`stash`、`checkout`、`clean`、削除を行わない。
- Contractのbase SHAと異なる場合は、その差分が意図されたものか確認するまで停止する。
- 複数repositoryではdependency branchとSHAを記録する。

## 実装前調査

変更対象classまたはmoduleについて、関数単体ではなく少なくとも以下を読む。

- class定義と初期化。
- 呼び出し元。
- 呼び出し先。
- 入出力port、message、IDL、parameter。
- mode遷移とfailure path。
- 対応するdiagnostics。

計画書の記述と現行コードが異なる場合、推測で実装せず差異を報告する。Contractを変更する必要がある場合はplanへ戻る。

## 実装ルール

- Contract外の機能変更を行わない。
- 一つのWork Unitに一つの主要責務だけを含める。
- diagnostic変更、control behavior変更、cleanupを不用意に混ぜない。
- `clang-format`を実行しない。
- 既存styleに合わせる。
- コメントとMarkdownは日本語で記述する。
- magic numberを避け、parameterまたは名前付き定数にする。
- 不明な安全仕様を推測しない。
- 実行していない検証をPASS扱いしない。

## 500 Hz経路の禁止事項

`auto_stabilizer::onExecute()`およびその同期呼び出し経路へ、次を追加しない。

- ROS callback。
- network I/O。
- blocking I/O。
- 外部process待ち。
- condition variable待ち。
- 無制限のmutex待ち。
- 毎周期thread生成。
- 無制限queue。
- 不要なrobot clone。
- 毎周期の全探索。
- 反復回数の無制限増加。
- 不要な動的確保。

external generatorまたはbridge停止時にも500 Hz経路を待たせない。

## 安全不変条件

該当するWork Unitでは次を明示的に保持する。

- joint position / velocity limit。
- self collision。
- 足拘束、接触、歩行安定化。
- COM、ZMP、`genCog`、`sbpOffset`、`refdz`、`omega`、`l`の整合。
- M4.2.2 walking preparation、READY、歩行API gate、COM高さ保持。
- static両足支持時だけのoperator CHEST/COM。
- walking preparation・歩行中のCHEST/COM無効化、腕・頭部継続。
- stale、invalid、solver failure時のcurrent accepted/generated state hold。
- hidden goalを作らない。
- `q_nominal`をhardwareへ直接出力しない。

## Interface変更

message、IDL、bridge、producer、consumerのいずれかを変更する場合は、同じWork Unit Contractにmapping表を持たせる。

必ず確認する。

- field名と型。
- 単位。
- frame。
- quaternion順序。
- task mask bit。
- enum値。
- schema version。
- timestamp、session、epoch、sequence。
- unknown/duplicate joint名。
- producerとconsumerのcompatible SHA。

schema未確定のproducer/consumerを並行実装しない。

## 実装後の最低限check

```sh
git diff --check
git status --short
git diff --stat
git diff -- <changed files>
```

Contractで指定されたbuild/checkも実行する。

IDL変更後の`auto_stabilizer`初回build:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

それ以外:

```sh
catkin build auto_stabilizer --no-deps
```

commandが失敗した場合、errorを隠さず記録する。simulation、実機確認が実行できない場合は`UNVERIFIED`とする。

## 自己点検

完了前に最新diff全体を読み直し、次を確認する。

- Contract外の差分がない。
- failure、stale、mode transitionが実装されている。
- source selectorがlast-writer-winsになっていない。
- finite checkとreject reasonがある。
- diagnosticsがfailureを切り分けられる。
- 既存legacy経路に意図しない回帰がない。
- generated artifact、log、cache、core dumpが含まれていない。

## 必須出力

```markdown
# Work Unit <ID> implementation result

## Result

## Repository state
| repository | branch | base SHA | current SHA | dirty |

## Changes
| file | change | reason |

## Important decisions

## Commands and results
| command | result | evidence |

## Acceptance status
| criterion | result | evidence |

## Unverified

## Review focus

## Contract deviations

## Compatible dependency set
| repository | SHA |
```

このSkill使用中はcommit、push、merge、PR作成を行わない。
