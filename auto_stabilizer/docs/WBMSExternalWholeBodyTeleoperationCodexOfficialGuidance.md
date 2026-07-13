# WBMS外部Whole-Body操縦 Codex公式情報適用指針

## 1. 文書の目的

本書は、`WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`を、2026-07-13時点のOpenAI公式Codexドキュメントへ対応づける補助文書である。

CodexのUI、model名、mode名、slash command、設定項目は更新され得る。このため、長期プロジェクトでは次を分離する。

- 変化しにくい原則: Work Unit、Contract、read-only review、Progress、atomic commit、安全不変条件。
- 変化し得る操作: model picker名、reasoning level名、plan capabilityのUI、review delivery設定。

利用時は本書の原則を維持し、現行Codex UIと公式ドキュメントへ読み替える。

---

## 2. 参照したOpenAI公式資料

- AGENTS.md:
  - https://learn.chatgpt.com/docs/agent-configuration/agents-md
- Skills:
  - https://learn.chatgpt.com/docs/build-skills
- Code review:
  - https://learn.chatgpt.com/docs/code-review
- Long-running work:
  - https://learn.chatgpt.com/docs/long-running-work
- Projects、chats、tasks:
  - https://learn.chatgpt.com/docs/projects
- Environments:
  - https://learn.chatgpt.com/docs/environments/modes
- Git worktrees:
  - https://learn.chatgpt.com/docs/environments/git-worktrees
- Models:
  - https://learn.chatgpt.com/docs/models

実装task開始時には、上記URLが現行であるか確認する。

---

## 3. AGENTS.mdの適用

公式仕様では、Codexは作業前に`AGENTS.md`を読み、project rootからcurrent working directoryへ向かう階層のinstructionを連結する。current directoryに近いinstructionが後に置かれ、より具体的な指示として作用する。

今回の構成:

```text
repository root/AGENTS.md
  全体で安定した開発規約

repository root/auto_stabilizer/AGENTS.md
  auto_stabilizer package固有の500 Hz、安全、walking preparation規約

Work Unit Contract
  一回の変更だけに適用するscope、acceptance、verification
```

運用原則:

- 恒久的で広い規約だけを`AGENTS.md`へ置く。
- task固有のparameter、field、Milestone詳細を`AGENTS.md`へ複製しない。
- nearest `AGENTS.md`と正式計画が矛盾した場合は、推測で実装せずplanへ戻る。
- `AGENTS.md`を過度に長くしない。詳細は正式計画、Progress、Skillへ分離する。
- subdirectory固有の規約が増えた場合は、必要なdirectoryへ追加の`AGENTS.md`を置く。

---

## 4. Skillの適用

公式仕様では、Skillは`.agents/skills/<skill-name>/SKILL.md`へ置く。`SKILL.md`には`name`と`description`が必要である。Codexは最初にname、description、pathだけを読み、選択したSkillの本文を後から読むprogressive disclosureを使う。

今回のSkill:

```text
wbms-plan-work-unit
  read-only調査とWork Unit Contract

wbms-implement-work-unit
  承認済みContractの実装、build/check、自己点検

wbms-review-work-unit
  read-only dedicated review

wbms-close-work-unit
  Progress、commit readiness、明示許可時のatomic commit
```

原則:

- 一つのSkillは一つのjobだけを担当する。
- descriptionに「使用する場面」と「使用しない場面」を明記する。
- project全仕様をSkillへ複製せず、正式計画とProgressを読むよう指示する。
- 頻繁に変わるWork Unit固有情報をSkill本文へ固定しない。
- Skill本文を変更した場合、Codexで認識されないときはsessionを再起動する。
- `$wbms-plan-work-unit`等の明示呼出しを標準とする。暗黙選択だけに依存しない。

---

## 5. Task分割

公式資料は、distinct outcomeごとに別taskを開始し、durable guidanceを`AGENTS.md`またはchecked-in documentationへ置くことを推奨している。

今回の原則:

```text
一つのWork Unit
  = 一つのdistinct outcome
  = 一つのContract
  = 一つの主要責務
  = 原則一つのatomic commit
```

推奨task分割:

1. Planning task。
2. Implementation task。
3. Detached review task。
4. Finding修正はimplementation task。
5. 修正後のfresh detached review。
6. Closure task。

同じtaskを継続するもの:

- 同一Contract内の実装とreview finding修正。
- 同一Work Unit内のbuild error修正。

別taskにするもの:

- schema設計とconsumer実装。
- auto_stabilizer control behaviorとbridge mapping。
- unrelated cleanup。
- 次Milestone。
- independent read-only調査。

長いtaskでcontext圧縮が発生しそうな場合、圧縮に頼る前にProgressとContractを更新し、新taskへ引き継ぐ。

---

## 6. Planning phase

Codex clientに明示的なplan capabilityがある場合は使用してよい。ただし、特定のUI名へ依存せず、次のread-only contractを守る。

```text
- sourceを変更しない。
- branch、HEAD、dirty stateを記録する。
- 現行call pathを読む。
- 仕様差異、安全不変条件、失敗経路を特定する。
- 変更file、out-of-scope、acceptance、verificationを固定する。
- Work Unit Contractだけを出力する。
```

明示的なplan modeがないclientでは、新しいtaskへ次を指示する。

```text
このtaskはread-only planningです。fileを変更しないでください。
対象Work Unitの現行コード、履歴、正式計画を読み、Work Unit Contractを作成してください。
Contractが承認されるまで実装、stage、commitを行わないでください。
```

planningでは最も強いreasoningが必要になる場面が多い。

- branch archaeology。
- walking preparation ownership。
- COM/ZMP/refdz/omega/l。
- cross-repository schema。
- safety invariant。

---

## 7. Goal / implementation phase

OpenAI公式のlong-running work資料は、長い作業にclear outcome、constraints、definition of doneを与えることを推奨し、interactive Codex CLIでは`/goal`を利用できるとしている。

今回の実装taskでは、Goalへ次を含める。

```text
Outcome:
  Work Unit ContractのGoal

Constraints:
  branch/base SHA
  files allowed
  out-of-scope
  500 Hz禁止事項
  safety invariants
  no commit

Definition of done:
  acceptance criteria
  exact build/check
  implementation report
  unverified itemsの明示
```

推奨prompt:

```text
$wbms-implement-work-unit を使用してください。
Work Unit: <ID>
承認済みContract: <path>

Contractの範囲だけを実装してください。
指定build/checkを実行してください。
このtaskではcommit、push、merge、PR作成、実機実行を行わないでください。
```

一回のGoalでM0からM12までを実装させない。

---

## 8. Review phase

公式資料では、`/review`はbase branch、uncommitted changes、exact commit等を対象に、working treeを変更せずprioritized findingを返す専用reviewerを起動する。

今回の標準:

- implementation taskとは別のdetached review taskを使う。
- review対象をexactに指定する。
- custom review instructionsとしてWork Unit Contractとproject固有観点を渡す。
- reviewerへ修正させない。
- finding修正後はincremental diffだけでなく最新diff全体をfresh reviewする。

推奨prompt:

```text
$wbms-review-work-unit を使用してください。
この変更をread-onlyでreviewしてください。sourceを変更しないでください。

Work Unit: <ID>
Scope: uncommitted / commit <SHA> / base <branch>
Contract: <path>

P0/P1/P2/P3でfindingを分類し、file:line、実行経路、破られる仕様、再現条件、最小修正方針を示してください。
```

final review条件:

- P0なし。
- P1なし。
- P2なし。
- P3は採否を記録。
- simulation/実機の未確認をコードfindingと混同しない。

---

## 9. Local、Worktree、Cloud

公式Codex環境はLocal、Worktree、Cloudに分かれる。

### Local

今回の既定とする。

- ROS/catkin build。
- OpenRTM/hrpsys。
- Choreonoid model。
- simulator/log。
- 複数dependency repository。
- 実機に接続する前のlocal確認。

### Worktree

独立taskを隔離する場合に使う。

適する例:

- planning document変更。
- bridgeとauto_stabilizerの別branch。
- detached review用clean checkout。
- `1.0`、pre-M5、新branch比較。

禁止:

- 同じbranchを複数worktreeで編集する。
- 同じsource fileを複数taskで同時編集する。
- schema未確定のproducer/consumerを並行実装する。

### Cloud

local hardwareやprivate dependencyが不要な作業に限定する。

- read-only文書review。
- message schema案。
- generic C++ design review。

初期build、simulation、実機確認の既定にはしない。

---

## 10. GPT-5.6 model選択

2026-07-13時点の公式Codex model資料では、GPT-5.6 familyとしてSol、Terra、Lunaが案内されている。

### Sol

complex、open-ended、high-valueな作業へ使う。

今回:

- branch archaeology。
- Implementation Plan変更。
- auto_stabilizer safety-critical change。
- walking preparation。
- COM/ZMP/refdz/omega/l。
- external IK priority設計。
- protocol全体review。
- release前review。

reasoning:

- 通常High。
- 複数の安全tradeoffがある場合Extra High。
- Maxは単一taskとして最難関の場合だけ。

### Terra

日常的な強いcoding/tool workへ使う。

今回:

- schema確定後のmessage/IDL実装。
- bridge mapping。
- diagnostics。
- small class extraction。
- build error修正。
- Work Unit内の明確なimplementation。

reasoning:

- MediumまたはHigh。

### Luna

仕様が明確な反復・変換へ使う。

今回:

- enum mapping表の機械的反映。
- Progress template追記。
- field名の一対一変換。
- exact command実行結果の構造化。

安全判断、mode ownership、IK priority、COM/ZMP変更をLuna単独へ任せない。

### MaxとUltra

公式資料では、Maxは単一taskへより長いreasoningを与え、Ultraはsubagentで分割可能な複雑taskを並列化する。多くのtaskはMax/Ultraを必要としない。

Ultraを使ってよい例:

- 独立したread-only branch archaeologyをrepository別に分ける。
- schema固定後、別repository・別fileのmapping調査を並列化する。
- source reviewとlog解析を分離する。

使わない例:

- 同じ`AutoStabilizer.cpp`を複数agentが編集する。
- schema未確定でproducerとconsumerを並行実装する。
- walking preparation ownershipを複数agentが別々に変更する。

coordinatorはsubagentの結果を統合し、最終diffを単一のfresh reviewへ通す。

---

## 11. Context管理

context圧縮を作業履歴の正本にしない。

正本:

1. Implementation Plan。
2. Progress。
3. Work Unit Contract。
4. git history。
5. compatible dependency SHA。

各Work Unit完了前にProgressへ次を記録する。

- branch、base/current SHA、dirty。
- 実際のcall order。
- 設計判断とrejected案。
- changed file。
- exact commandと結果。
- review round。
- acceptance evidence。
- unverified。
- next entry point。

新taskの最初のpromptへ会話全履歴を貼らない。Work Unit IDと正本pathを渡す。

---

## 12. 推奨する実行シーケンス

```text
1. New planning task
   model: Sol High/Extra High
   environment: Localまたはread-only Worktree
   skill: wbms-plan-work-unit

2. Contract human review
   scope、acceptance、safetyを固定

3. New implementation task
   model: SolまたはTerra、難度に応じHigh
   environment: Local
   skill: wbms-implement-work-unit
   commit禁止

4. Developer verification
   build/check

5. Detached review task
   model: Sol High以上
   /review + wbms-review-work-unit相当のcustom instructions

6. Implementation task
   finding修正

7. Fresh detached review
   最新diff全体

8. Closure task
   skill: wbms-close-work-unit
   Progress、checklist、commit readiness

9. Explicit commit authorization
   atomic commitのみ
   push/merge/PRなし

10. Next Work Unit
   新taskを開始
```

---

## 13. 公式情報更新時の扱い

Codexのmodel、mode、Skill、review機能が更新された場合:

1. OpenAI公式資料を確認する。
2. 変化したUI名だけを読み替える。
3. Work Unit、Contract、read-only review、Progress、atomic commit、安全不変条件を維持する。
4. workflow変更理由をProgressへ記録する。
5. workflow/Skill変更をcontrol implementationと同じcommitへ混ぜない。
