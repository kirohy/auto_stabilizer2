# WBMS外部Whole-Body操縦 Codex作業ワークフロー

## 1. 文書の目的

本書は、`WBMSExternalWholeBodyTeleoperationImplementationPlan.md`をCodexで実装する際の標準作業手順を定める。

対象は単一repository内の短い変更ではなく、以下を含む長期・複数repository・安全重要プロジェクトである。

- `auto_stabilizer2`
- `whole_body_teleop`
- `rtmros_msg_bridge`
- 必要に応じて`ik_solvers2`
- 必要に応じて`prioritized_qp`

本書の目的は、長い一つの会話へ全作業を詰め込むことではない。

```text
小さく検証可能なwork unitを定義
  -> read-only調査
  -> 実装
  -> 独立review
  -> 修正
  -> 検証
  -> 履歴
  -> atomic commit
```

を反復し、別task・別担当・context圧縮後でも安全に引き継げる状態を維持することである。

---

## 2. 公式Codex資料から採用する原則

本書は、作成時点のOpenAI公式Codex資料を参考にする。

- Codex long-running tasks:
  - https://developers.openai.com/codex/app/features/long-running-tasks
- Codex plans:
  - https://developers.openai.com/codex/app/features/plans
- Codex code review:
  - https://developers.openai.com/codex/app/features/code-review
- Codex AGENTS.md:
  - https://developers.openai.com/codex/guides/agents-md
- Codex Skills:
  - https://developers.openai.com/codex/skills
- Codex worktrees:
  - https://developers.openai.com/codex/app/features/worktrees
- Codex model selection:
  - https://developers.openai.com/codex/models

公式資料から、今回の運用では以下を採用する。

1. 要件や実装経路が不明な場合は、実装前にplan modeで仕様と成功条件を固める。
2. 実装開始時は、成果、制約、検証方法を明示したgoalを与える。
3. `/review`は作業treeを変更しない独立したreviewerとして使う。
4. `AGENTS.md`はrepository全体の安定した規約に使い、task固有仕様は計画書へ置く。
5. 繰り返す定型作業はSkillへ分離する。
6. parallel taskは独立性がある場合だけ使い、同じsource fileを複数taskで同時編集しない。
7. worktreeは独立変更を隔離するために使い、同じbranch・同じ生成物を複数worktreeで競合させない。

Codex UI、model名、slash commandの具体名は更新される可能性がある。利用時に公式資料と現在のmodel pickerを確認し、本書の意図を維持したうえで現行UIへ読み替える。

---

## 3. 基本単位: Work Unit

### 3.1 定義

Work Unitは、一つの明確な受入条件と、一つのreview可能な変更集合を持つ作業単位である。

良い例:

- custom ROS command messageとenumを追加する。
- bridgeのcommand方向だけを実装する。
- `NonIkJointReferenceController`のqRef overrideだけを実装する。
- group別stale state machineだけを追加する。
- external IKへ両手位置taskを追加する。

悪い例:

- 外部操縦システム全体を実装する。
- message、bridge、auto_stabilizer、solverを一度に完成させる。
- 複数repositoryの未関連cleanupまで同時に行う。

### 3.2 推奨サイズ

一つのWork Unitは、原則として次を満たす。

- 主目的が一文で説明できる。
- 主要な変更責務が一つである。
- 変更fileを事前に概ね列挙できる。
- review時に変更意図を一度で追跡できる。
- buildまたは静的checkの完了条件を定義できる。
- 一つのatomic commitにできる。

大きすぎる場合は、protocol、producer、consumer、safety、diagnosticsの順に分割する。

### 3.3 repository境界

一つの論理機能が複数repositoryを変更する場合でも、commitはrepositoryごとに分ける。

例:

```text
WU-M1-A:
  whole_body_teleop_msgsにschema追加

WU-M1-B:
  rtmros_msg_bridgeにROS<->RTM mapping追加

WU-M1-C:
  auto_stabilizer2にIDLとInPort追加
```

同じWork Unit IDのsub-unitとして関連付け、Progressに互換commit SHAを記録する。

---

## 4. Taskと会話の分割方針

### 4.1 プロジェクト全体を一つのtaskで実行しない

長大なtaskは、以下の問題を起こす。

- context圧縮で細かい判断根拠が失われる。
- 過去の不採用案を再実装しやすい。
- unrelated diffが混ざる。
- review観点が曖昧になる。
- build未実施項目と実施済み項目が混ざる。

したがって、原則として一つのWork Unitごとに新しいCodex taskを開始する。

### 4.2 推奨task構成

```text
Task A: Work Unit plan
  read-only調査
  Work Unit Contract作成

Task B: Work Unit implementation
  approved contractを実装
  build/check
  初回自己点検

Task C: detached review
  diffと仕様をread-only review

Task Bへ戻る:
  review finding修正

Task D: fresh review
  修正後diffを最初からreview

Task B:
  Progress更新
  commit readiness
  明示指示後commit
```

実装と修正は同じtaskで続けてもよい。ただしcontextが大きくなった場合は、ProgressとWork Unit Contractを更新して新taskへ引き継ぐ。

reviewは原則として実装taskと分ける。実装者の思い込みをreviewへ持ち込まないためである。

### 4.3 parallel task

parallel taskを使ってよい例:

- 一方がOpenRTM/IDL仕様のread-only調査、他方がROS message設計のread-only調査。
- 異なるrepository、異なるfileを変更し、schemaが既に固定されている。
- simulation log解析とsource reviewが独立している。

使ってはいけない例:

- 同じmessage schemaを二つのtaskが編集する。
- 同じ`AutoStabilizer.cpp`を二つのtaskが編集する。
- producerとconsumerをschema未確定のまま並行実装する。
- 一方がbranchをrebase中に他方が同じbranchへcommitする。

---

## 5. Modeとmodelの使い分け

### 5.1 Plan mode

Plan modeを使用する場面:

- branch archaeology。
- 現行呼び順・責務境界の確認。
- 複数repository interface設計。
- safety invariantの特定。
- 変更fileと受入条件の確定。
- 過去文書の優先順位解釈。

Plan modeではsourceを変更しない。

出力はWork Unit Contractに限定する。

### 5.2 Goal / implementation mode

仕様が承認済みの場合、goalに以下を含める。

- 何を完成させるか。
- 対象repository、branch、base SHA。
- 変更してよいfile。
- 変更してはいけない責務。
- acceptance criteria。
- 実行するbuild/check。
- Progressへ記録する項目。
- commitしない、または明示許可時だけcommitする指示。

### 5.3 Review mode

`/review`または同等のread-only code reviewを使用する。

reviewerは実装を変更しない。

review scopeは明示する。

- uncommitted diff。
- specific commit。
- base branchとの差分。
- custom instructionsを含むproject-specific review。

### 5.4 model選択

model名や提供形態は変わり得るため、以下は役割で選ぶ。

#### 最も強いreasoning / coding model

使用場面:

- `auto_stabilizer`の安全重要変更。
- walking preparation。
- COM/ZMP/refdz/omega/l統合。
- branch archaeology。
- cross-repository protocol変更。
- 最終release review。

reasoning levelはHighまたはExtra Highを基本とする。

#### 標準coding model

使用場面:

- schema確定後のmessage/IDL実装。
- bridge mapping。
- diagnostics追加。
- 小さなclass分割。
- build error修正。

MediumまたはHighを基本とする。

#### 軽量・高速model

使用場面:

- 仕様が完全固定された反復的変換。
- enum mapping表の機械的更新。
- Progressの定型追記。
- build commandの実行と結果整理。

安全判断、mode遷移、IK priority、COM/ZMP変更を単独で任せない。

#### subagent / Ultra相当

使用してよい場面:

- 独立したread-only調査を並列化する。
- repositoryごとに書込み対象が完全分離されている。
- coordinatorがschemaと受入条件を固定している。

routine workでは使用しない。分割可能性が低いtaskへ無理にsubagentを増やさない。

---

## 6. Environmentの使い分け

### 6.1 Localを既定にする

今回の実装は以下に依存する。

- ROS / catkin workspace。
- OpenRTM / hrpsys。
- 複数repositoryのbranch整合。
- Choreonoid robot model。
- simulator log。
- 最終的には実機。

したがって、build、simulation、実機確認はLocal environmentを既定とする。

### 6.2 Worktree

worktreeを使う場面:

- 計画文書だけを編集する。
- bridgeとauto_stabilizerの独立変更を分ける。
- detached review用にclean checkoutを作る。
- 比較実験用に`1.0`、pre-M5、new branchを同時に保持する。

ルール:

- 一つのworktreeに一つのbranch。
- 同じbranchを二つのworktreeで編集しない。
- generated IDL/build artifactをrepositoryへcommitしない。
- worktreeの依存repository SHAをProgressへ記録する。

### 6.3 Cloud

Cloud taskは、local hardwareや未公開依存が不要なread-only調査、message設計、文書reviewには使用できる。

初期実装、catkin build、simulation、実機確認の既定にはしない。

---

## 7. Work Unit標準ライフサイクル

## Phase 0: 準備

実行前に次を確認する。

```sh
git status --short
git branch --show-current
git rev-parse HEAD
```

複数repositoryの場合、全repositoryで記録する。

- userの既存未commit変更を確認。
- 変更を上書きしない。
- dependency branchとSHAを記録。
- authoritative planと最新Progressを読む。
- nearest `AGENTS.md`を読む。

既存変更がある場合、無断でreset、checkout、stash、削除しない。

## Phase 1: Read-only Plan

Plan taskへ以下を渡す。

```text
対象Work Unit ID:
対象repository/branch/HEAD:
上位計画書:
最新Progress:
変更目的:
既知の制約:

実装は行わず、現行コードと履歴を読んで以下を出力してください。
1. 現行呼び順と責務。
2. 変更候補file、class、function。
3. 保持すべきinvariant。
4. 仕様とコードの差異。
5. 失敗経路。
6. 受入条件。
7. build/check。
8. Work Unit Contract。
不明点は、実装を左右するものだけ質問してください。
```

### Work Unit Contract形式

```markdown
# Work Unit Contract: <ID>

## Goal

## Scope
- repository
- branch
- base SHA
- files allowed

## Out of scope

## Current behavior

## Required behavior

## Safety invariants

## Interface/schema impact

## Implementation steps

## Acceptance criteria

## Verification commands

## Review focus

## Known unverified items
```

Contractが承認されるまで実装しない。

## Phase 2: Contract freeze

実装前に次を固定する。

- Work Unit ID。
- base SHA。
- schema version。
- 変更対象file。
- acceptance criteria。
- validation command。
- out-of-scope。

実装中にcontract変更が必要になった場合、勝手にscopeを広げず、Progressに理由を記録してplanへ戻る。

## Phase 3: Implementation

実装taskの推奨prompt:

```text
承認済みWork Unit Contractに従って実装してください。

必須:
- 最初に対象branch/HEADと既存変更を確認する。
- Contract外の機能変更をしない。
- 既存user変更を保持する。
- clang-formatを実行しない。
- コメントとMarkdownは日本語。
- 500 Hz経路へblocking I/O、ROS callback、thread生成、重い動的確保を追加しない。
- hidden goalを作らない。
- 不明な安全仕様を推測しない。
- 実装後に指定されたbuild/checkを実行する。
- 失敗したcommandとerrorを隠さない。
- この段階ではcommitしない。

完了時に以下を報告してください。
1. 変更概要。
2. 変更file。
3. 重要な設計判断。
4. 実行したcommandと結果。
5. 未実行・未確認項目。
6. reviewで重点確認すべき点。
```

### Implementation中のルール

- source変更前に該当class全体を読む。
- 呼び出し元と呼び出し先を確認する。
- IDL/message変更はproducer/consumer mapping表と同時に扱う。
- diagnosticだけの変更とcontrol behavior変更を混ぜない。
- cleanupを機能変更に混ぜない。
- magic numberを追加せずparameterまたは名前付き定数にする。
- 既存の日本語comment styleへ合わせる。

## Phase 4: Developer verification

最低限:

```sh
git diff --check
git status --short
git diff --stat
git diff -- <changed files>
```

加えてContractのbuild/checkを実行する。

実行できないsimulation、実機確認はPASSと書かず、`UNVERIFIED`と記録する。

## Phase 5: Dedicated Review

実装taskと別のreview taskを推奨する。

review promptは本書第10章を使用する。

reviewerはfindingだけを返し、sourceを変更しない。

## Phase 6: Finding修正

- findingごとに根本原因を確認する。
- reviewerの提案を無条件採用しない。
- 仕様と矛盾するfindingは、根拠を示して`not applicable`とする。
- finding修正でscopeが広がる場合はplanへ戻る。
- 修正後に同じbuild/checkを再実行する。

## Phase 7: Fresh Review

修正後はincremental findingだけを見るのではなく、最新diff全体をfresh reviewする。

commit条件:

- P0 findingなし。
- P1 findingなし。
- P2 findingなし。
- P3は採用、却下、後続化の判断が記録済み。

review loopを回数で打ち切らない。新たなP0〜P2がなくなるまで繰り返す。

## Phase 8: Progress更新

commit前にProgressへ追記する。

Progressが未更新のままcommitしない。

## Phase 9: Commit readiness

第12章のchecklistを満たす。

Codexは明示的なcommit指示がある場合だけcommitする。

## Phase 10: Atomic Commit

一つのcommitには一つの主要目的だけを含める。

例:

```text
Add WBMS external command schema
Implement head direct reference override
Connect external COM reference to static ZMP integration
Add external IK self-collision constraints
```

避ける例:

```text
Implement teleop and cleanup
Fix various WBMS issues
Update everything
```

---

## 8. Multi-repository変更手順

### 8.1 protocol-first

schema変更は次の順で行う。

1. protocol contractとmapping表。
2. message/IDL生成物。
3. bridge。
4. producer。
5. consumer。
6. end-to-end verification。

### 8.2 compatibility

- `schema_version`を更新する条件を定義する。
- old/newの互換性を明記する。
- incompatible consumerはsilent fallbackせずrejectする。
- repositoryごとの対応SHAをProgressへ記録する。

### 8.3 commit順

一時的にbuild不能な中間commitを避ける。

例:

```text
whole_body_teleop_msgs commit A
rtmros_msg_bridge commit B, requires A
auto_stabilizer2 commit C, requires A+B
whole_body_teleop generator commit D, requires A
```

Progressへ次を記録する。

```text
Compatible set:
- whole_body_teleop: <sha>
- rtmros_msg_bridge: <sha>
- auto_stabilizer2: <sha>
- ik_solvers2: <sha>
- prioritized_qp: <sha>
```

### 8.4 dependency変更

solver repositoryを変更する場合:

- auto_stabilizer既存APIを暗黙に変えない。
- feature flagまたは明示APIを使う。
- dependent packageのbuildを実行する。
- dependency diffも独立reviewする。

---

## 9. Review severity

### P0: Critical

- 実機へ危険な不連続指令を出す。
- joint limit、collision、足拘束を破る。
- stale後に残留commandを再生する。
- data corruption、NaN、未初期化値。
- schema解釈違いで不正な関節へ指令する。
- build不能でWork Unitが使用できない。

### P1: High

- 主要仕様を満たさない。
- walking preparation、mode gate、ownershipが破綻する。
- 500 Hz経路へblocking処理を入れる。
- source/session/stale処理で誤ったtaskを有効化する。
- COM/ZMP/refdz等が不連続になる。
- legacy modeに回帰を入れる。

### P2: Medium

- 診断不足で失敗原因を切り分けられない。
- interface validation不足。
- maintainability上、次のMilestoneを危険にする責務混在。
- test/build手順の欠落。
- Progressと実装が矛盾する。

### P3: Low / Suggestion

- 命名改善。
- optional cleanup。
- performance微改善。
- 今回scope外の将来拡張。

P3はcommitを止めないが、判断を記録する。

---

## 10. Review prompt

```text
この変更をread-onlyでreviewしてください。sourceは変更しないでください。

参照順:
1. WBMSExternalWholeBodyTeleoperationImplementationPlan.md
2. WBMSExternalWholeBodyTeleoperationProgress.md
3. 対象Work Unit Contract
4. WBMSWalkingPreparationDesignRevisionPlan.md
5. AGENTS.md

対象:
- repository / branch / base SHA
- changed filesまたはcommit

findingはP0/P1/P2/P3で分類し、各findingに以下を含めてください。
- severity
- file:line
- 問題の具体的な実行経路
- 破られる仕様またはinvariant
- 再現条件
- 最小修正方針

重点確認:
1. ROS message、RTM IDL、bridge mappingのfield、単位、enum、quaternion順序。
2. frame変換、foot-mid、CHEST相対、device worldの取り違え。
3. session、epoch、sequence、out-of-order、schema version。
4. task group別enabled/valid/stale/hold。
5. hidden goal、未実現commandの蓄積。
6. static両足支持gate、walking preparation ownership、walking task mask。
7. joint position/velocity limit、collision、両足拘束、final validation。
8. COM、genCog、sbpOffset、ZMP、refdz、omega、lの整合と連続性。
9. qRef読込後、FK前のnon-IK joint override順序。
10. 首・将来指関節のallowlist、limit、stop時復帰。
11. legacyとexternal sourceの排他性、last-writer-wins回避。
12. 500 Hz経路のblocking I/O、ROS callback、mutex待ち、thread生成、不要なallocation。
13. producer/consumerのcompatible SHAとbuild依存。
14. failure時のhold、reject、diagnostics。
15. Progress、計画、実装の一致。

次を問題として報告しないでください。
- clang-formatを適用していないこと。
- auto_stabilizer repositoryで新規unit testを追加していないこと。
- コメントとMarkdownが日本語であること。
- legacy portを初回実装で残していること。
- pre-M5 final IK、maxIteration=1を初期baselineとして維持していること。
- single-QP final WBCを今回実装していないこと。
- walking中CHEST/COM操作を未実装であること。
- simulatorまたは実機を実行できていないこと自体。これはUNVERIFIEDとして記録すべきであり、コードfindingとは区別してください。
- 過去文書に古い不採用案が残っていること。新しい優先文書を誤って上書きしている場合だけfindingにしてください。

styleのみのfindingは、実害が説明できない限りP3以下としてください。
```

---

## 11. Progress記録

### 11.1 原則

Progressはappend-onlyの時系列記録とする。

過去entryを静かに書き換えない。誤りが判明した場合は、新しい訂正entryを追加する。

各entryは、別taskが会話履歴なしで再開できる粒度にする。

### 11.2 必須情報

```markdown
## <date> <Work Unit ID> <title>

### Status
PLANNED / IN_PROGRESS / BLOCKED / IMPLEMENTED / REVIEWED / VERIFIED / COMMITTED

### Repository state
| repository | branch | base SHA | current SHA | dirty |

### Goal

### Scope and out-of-scope

### Code investigation
- current call order
- relevant classes/functions
- discovered discrepancy

### Decisions
- adopted
- rejected and reason

### Changes
| file | change | reason |

### Commands and results
```sh
<exact command>
```
- PASS / FAIL / NOT RUN
- relevant output or error

### Simulation / log evidence
- log path
- conditions
- sample count
- metrics
- interpretation

### Review
| round | reviewer/task | findings | resolution |

### Acceptance
| criterion | result | evidence |

### Unverified

### Open issues

### Compatible dependency set
| repository | SHA |

### Next entry point
- first files/functions to read
- exact next Work Unit
- warnings

### Commit
- SHA
- subject
- cherry-pick notes
```

### 11.3 粒度

記録する:

- 実際に確認したcall order。
- なぜその設計を選んだか。
- rejected案と理由。
- exact command。
- failure outputの要点。
- simulator条件。
- review roundとfinding解消。
- 未確認事項。

記録しない:

- 全terminal outputの無加工貼付。
- 一時的な思考メモ。
- 根拠のない「問題なさそう」。
- 実行していないcheckをPASS扱いする記述。

---

## 12. Commit checklist

commit前に以下を全て確認する。

### Repository state

- [ ] 正しいbranchである。
- [ ] base SHAがWork Unit Contractと一致する。
- [ ] userの既存変更を消していない。
- [ ] unrelated fileがstage対象に入っていない。
- [ ] dependency SHAがProgressへ記録されている。

### Diff

- [ ] `git diff --check` PASS。
- [ ] `git diff --stat`を確認。
- [ ] 全changed fileを読んだ。
- [ ] generated build artifact、log、cache、core dumpを含まない。
- [ ] clang-formatによる大規模format差分がない。
- [ ] cleanupと機能変更を混在させていない。

### Interface

- [ ] message、IDL、bridge mappingが一致する。
- [ ] schema versionを確認。
- [ ] enum/task mask mappingを確認。
- [ ] frame、単位、quaternion順序を確認。
- [ ] unknown joint、duplicate joint、name配列変更の扱いを確認。

### Safety

- [ ] finite check。
- [ ] joint limit。
- [ ] stale/hold。
- [ ] mode/support gate。
- [ ] no hidden goal。
- [ ] failure/reject path。
- [ ] 500 Hz blockingなし。
- [ ] walking preparation回帰なし。

### Verification

- [ ] 指定build PASS。
- [ ] IDL変更後`--force-cmake`を実行。
- [ ] dependency変更時はdependent buildを実行。
- [ ] simulation/実機未実施項目をUNVERIFIEDとして記録。
- [ ] reviewでP0/P1/P2なし。
- [ ] review後の修正に対してfresh review済み。

### Documentation

- [ ] Progress更新済み。
- [ ] 計画との差異を記録。
- [ ] next entry pointがある。
- [ ] commit subjectが一目的を表す。

Codexへcommitさせるprompt:

```text
Commit checklistを一項目ずつ確認してください。
未達項目があればcommitせず報告してください。
全項目を満たす場合だけ、今回Work Unitの変更だけをstageし、指定subjectでcommitしてください。
push、merge、PR作成は行わないでください。
```

---

## 13. Work Unit完了報告

完了報告は次の形式にする。

```markdown
# Work Unit <ID> completion

## Result

## Commits
| repository | SHA | subject |

## Verification
| command/test | result |

## Review
- rounds
- final findings

## Acceptance

## Unverified

## Compatible dependency set

## Next Work Unit

## Risks / notes
```

「実装完了」と「simulation/実機検証完了」を区別する。

---

## 14. Skill構成

repository-local Skillを`.agents/skills/`へ置く。

```text
.agents/skills/
  wbms-plan-work-unit/
    SKILL.md
  wbms-implement-work-unit/
    SKILL.md
  wbms-review-work-unit/
    SKILL.md
  wbms-close-work-unit/
    SKILL.md
```

各Skillは一つのjobだけを扱う。

- Plan Skill: read-only調査とContract作成。
- Implement Skill: approved Contract実装、build、自己点検。commitしない。
- Review Skill: read-only review。source変更しない。
- Close Skill: Progress、checklist、commit readiness。明示指示時のみcommit。

Skillへtask固有の全仕様を複製しない。planとProgressへのpathを読むようにする。

---

## 15. 推奨実行例

### M1-A message schema

```text
1. New task, strongest or standard coding model, plan mode
   -> $wbms-plan-work-unit M1-A

2. Contract review and approval

3. New implementation task
   -> $wbms-implement-work-unit M1-A

4. New detached review task
   -> $wbms-review-work-unit M1-A

5. Implementation taskでfinding修正

6. Fresh review task

7. Implementation task
   -> $wbms-close-work-unit M1-A

8. Userがcommitを明示
   -> commit only
```

### Safety-critical auto_stabilizer work unit

- 最も強いcoding/reasoning model。
- planはExtra High相当。
- implementationはHigh以上。
- reviewは実装taskと別task。
- simulation未実施でrelease-readyとしない。

### Bridge mapping work unit

- schema固定後は標準coding modelでよい。
- reviewでは全fieldのbidirectional round-tripを重点確認する。

---

## 16. 禁止事項

- project全体を一度の巨大goalで実装しない。
- review前にcommitしない。
- Progress更新前にcommitしない。
- review taskにsource修正をさせない。
- 同じfileを複数taskで同時編集しない。
- schema未確定のproducer/consumerを並行実装しない。
- userの未commit変更をreset、stash、checkoutで無断変更しない。
- build未実行をPASSと書かない。
- simulator未確認を安定動作確認済みと書かない。
- safety findingをweight調整だけで隠さない。
- stale commandを後で再生しない。
- unrelated cleanupを機能commitへ混ぜない。
- Codexへpush、merge、実機実行を暗黙に許可しない。

---

## 17. Workflow自体の変更

本書を変更する場合:

1. 変更理由をProgressへ記録する。
2. project固有仕様と一般workflowを混在させない。
3. OpenAI公式資料の更新を確認する。
4. SkillとAGENTS.mdの矛盾を確認する。
5. workflow変更だけの独立commitにする。
