---
name: wbms-implement-work-unit
description: 承認済みContractに基づき、WBMS外部whole-body操縦プロジェクトのrepository sub-unitを1つだけ実装する。原則一つのWRITE repositoryに限定し、scope内のsource変更、指定package build/check、自己点検、実装報告までを行う。計画、独立review、複数repository同時変更、Progressだけの完了処理、commit、push、merge、PR作成、別途許可のないsimulation・実機実行には使用しない。
---

# WBMS Work Unit実装

## 目的

承認済みのWork Unit Contractを、一つの主要責務、一つのWRITE repositoryに限定して実装する。

このSkillは実装、指定されたpackage build/check、自己点検までを担当する。commit、push、merge、PR作成は行わない。

## Workspace path

```text
${CATKIN_WORKSPACE}
  = catkin_ws/<workspace_name> の絶対パス

${CATKIN_SOURCE_ROOT}
  = ${CATKIN_WORKSPACE}/src
```

`catkin_ws/src`を固定layoutとして仮定しない。

## 起動directoryとwrite範囲

- Codexは対象repository rootから起動する。
- Contractの`WRITE` repositoryは原則一つだけとする。
- sibling repositoryはContractで`READ`と明記された範囲だけ参照する。
- `${CATKIN_SOURCE_ROOT}`から複数repositoryを同時編集しない。
- 他repositoryの変更が必要になった場合、現在の実装を停止し、Parent Contractまたは別sub-unitへ戻す。

## 必須入力

実装開始前に次を特定する。

- Work Unit IDとtitle。
- Parent Work Unit ID。
- `${CATKIN_WORKSPACE}`、`${CATKIN_SOURCE_ROOT}`。
- Codex launch directory。
- 対象WRITE repository、branch、base SHA、現在HEAD。
- READ-only sibling repository一覧とSHA。
- 承認済みWork Unit Contract。
- 正式Implementation Plan Revision 2、MultiRepositoryOperations、Revision 1、元計画。
- 最新中央Progress entry。
- repository Project Context、存在する場合。
- 実行すべきpackage verification command。
- 変更を許可されたfile。

Contractがない、launch directoryが対象repository rootでない、WRITE repositoryが複数、branch/base SHAが一致しない、scopeが曖昧、または実装を左右する未決定事項がある場合はsourceを変更しない。

## 読む順序

`auto_stabilizer/`以下では次を順に読む。

1. repository rootの`AGENTS.md`。
2. `auto_stabilizer/AGENTS.md`。
3. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`。
4. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`。
5. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`。
6. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
7. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md`。
8. 対象Work Unit Contract。
9. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
10. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOperatorGuide.md`。
11. `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`。
12. 必要な既存正式文書。

他repositoryでは次を読む。

1. 対象repository rootの`AGENTS.md`。
2. nearest package/module `AGENTS.md`。
3. `docs/WBMSExternalTeleopProjectContext.md`、存在する場合。
4. 中央のRevision 2、MultiRepositoryOperations、Revision 1、元計画、Progress。
5. Parent Contractと対象sub-unit Contract。

## Repository状態確認

WRITE repositoryと必要なREAD repositoryで次を確認して記録する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

- userの未commit変更を消さない。
- 無断で`reset`、`stash`、`checkout`、`clean`、削除を行わない。
- Contractのbase SHAと異なる場合は、その差分が意図されたものか確認するまで停止する。
- READ repositoryを変更しない。
- dependency branchとSHAを記録する。

## 実装前調査

変更対象classまたはmoduleについて、関数単体ではなく少なくとも以下を読む。

- class定義と初期化。
- 呼び出し元。
- 呼び出し先。
- 入出力port、message、IDL、parameter。
- mode遷移とfailure path。
- 対応するdiagnostics。
- Parent Contractで固定されたproducer/consumer mapping。
- Project Contextとcompatible input set。

計画書またはContractと現行コードが異なる場合、推測で実装せず差異を報告する。Contractを変更する必要がある場合はplanへ戻る。

## 実装ルール

- Contract外の機能変更を行わない。
- WRITE repository以外を変更しない。
- 一つのWork Unitに一つの主要責務だけを含める。
- diagnostic変更、control behavior変更、cleanupを不用意に混ぜない。
- `clang-format`を実行しない。
- 既存styleに合わせる。
- コメントとMarkdownは日本語で記述する。
- magic numberを避け、parameterまたは名前付き定数にする。
- 不明な安全仕様を推測しない。
- 実行していない検証をPASS扱いしない。
- schema未確定のproducer、bridge、consumerを同時実装しない。

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

message、IDL、bridge、producer、consumerのいずれかを変更する場合は、Parent Contractとsub-unit Contractのmapping表に従う。

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

schemaの変更が必要になった場合、現在sub-unitで暗黙変更せずParent planへ戻る。

## Package build

workspace一括buildを標準にしない。

通常:

```sh
catkin build <package-name> --no-deps
```

依存関係まで確認する場合だけ:

```sh
catkin build <package-name>
```

IDL変更後の`auto_stabilizer`初回build:

```sh
catkin build auto_stabilizer --no-deps --force-cmake
```

`catkin build`の実行directoryは固定しない。実行したdirectoryを結果に記録する。

build failureが別repository修正を必要とする場合、READ repositoryをその場で変更せず、対象repositoryのfix sub-unitを提案する。

## 実装後の最低限check

WRITE repositoryで次を実行する。

```sh
git diff --check
git status --short
git diff --stat
git diff -- <changed files>
```

Contractで指定されたpackage build/checkも実行する。

commandが失敗した場合、errorを隠さず記録する。simulation、実機確認が実行できない場合は`UNVERIFIED`とする。

## 自己点検

完了前に最新diff全体を読み直し、次を確認する。

- WRITE repository以外の変更がない。
- Contract外の差分がない。
- failure、stale、mode transitionが実装されている。
- source selectorがlast-writer-winsになっていない。
- finite checkとreject reasonがある。
- diagnosticsがfailureを切り分けられる。
- 既存legacy経路に意図しない回帰がない。
- generated artifact、log、cache、core dumpが含まれていない。
- compatible input SHAが変化していない。

## 必須出力

```markdown
# Work Unit <ID> implementation result

## Result

## Workspace context
- catkin workspace root
- source root
- Codex launch directory

## Repository access
| repository | branch | base SHA | current SHA | access | dirty |

## Changes
| file | change | reason |

## Important decisions

## Commands and results
| execution directory | command | result | evidence |

## Acceptance status
| criterion | result | evidence |

## Unverified

## Review focus

## Contract deviations

## Compatible dependency set
| repository | SHA |

## Central Progress sync
- required / not required
- pending information
```

このSkill使用中はcommit、push、merge、PR作成を行わない。
