# WBMS外部Whole-Body操縦 Codex運用ガイド

## 1. 本書の目的

本書は、WBMS外部whole-body操縦プロジェクトをCodexで実際に進める人向けの操作手順である。

詳細な仕様と規則は次を正本とする。

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
2. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
3. `WBMSExternalWholeBodyTeleoperationProgress.md`
4. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`
5. `WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`
6. 対象Work Unit Contract

本書は、上記を毎回全文入力せずに運用するための実用ガイドである。

---

## 2. 基本原則

このプロジェクト全体を一つのCodex taskで実装しない。

```text
一つのWork Unit
  -> read-only計画
  -> 人間によるContract承認
  -> 実装
  -> build/check
  -> 独立review
  -> finding修正
  -> fresh review
  -> Progress更新とcommit readiness
  -> 人間の明示許可後にatomic commit
```

一つのWork Unitは、原則として一つの主要責務と一つのatomic commitに対応する。

例:

- M1-A: ROS message schema。
- M2-A: command方向bridge mapping。
- M5-A: non-IK head reference controller。
- M8-C: external COM referenceとZMP統合。

---

## 3. 最初に行う作業

次のWork UnitはM0-Aである。

```text
M0-A: branch archaeology and exact baseline selection
```

新しいCodex taskをLocal環境またはread-only Worktreeで開始し、GPT-5.6 SolのHighまたはExtra Highを使用する。

入力例:

```text
$wbms-plan-work-unit を使用してください。

Work Unit: M0-A branch archaeology and exact baseline selection
Repository: kirohy/auto_stabilizer2
Planning branch: wbms-external-teleop-plan

このtaskはread-only planningです。source、branch、index、working treeを変更しないでください。

次の順に読んでください。
1. repository root AGENTS.md
2. auto_stabilizer/AGENTS.md
3. auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md
4. auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlan.md
5. auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationProgress.md の最新entry
6. auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationCodexWorkflow.md
7. auto_stabilizer/docs/WBMSWalkingPreparationDesignRevisionPlan.md
8. 旧Progress後方のM4.2.2完了記録と最初のM5記録
9. git historyと該当source

目的:
- CHEST相対腕拘束を含むcommitを特定する。
- M4.2.2の正式仕様と全安全review修正を特定する。
- 最初のM5変更を特定する。
- 単一の安全なpre-M5 base commitが存在するか判定する。
- 存在しない場合はsynthetic baselineのcherry-pick候補と除外対象を列挙する。
- 1.0、pre-M5候補、synthetic候補、wbms-devのfinal IK差分を整理する。
- exact build/checkと未確認項目を定義する。

Work Unit Contractだけを出力してください。実装、branch作成、cherry-pick、commit、push、PR作成は行わないでください。
```

CodexがContractを出したら、人間が少なくとも次を確認する。

- base SHAの根拠。
- walking preparation修正が欠けていないか。
- M5変更が混入していないか。
- 変更対象とout-of-scope。
- acceptance criteria。
- build、simulation、log確認。

承認前に実装taskへ進まない。

---

## 4. Work Unitごとの標準操作

## 4.1 計画task

新しいtaskでPlan Skillを明示指定する。

```text
$wbms-plan-work-unit を使用してください。
Work Unit: <IDとtitle>
対象repository/branch: <...>
正式計画: <path>
最新Progress: <pathとentry>

read-onlyで現行コードと履歴を調査し、Work Unit Contractを作成してください。
source変更、stage、commitは行わないでください。
```

出力Contractを人間が承認する。

実装を左右する疑義がある場合だけ、この段階で質問させる。

## 4.2 実装task

Contract承認後、新しいtaskでImplement Skillを明示指定する。

```text
$wbms-implement-work-unit を使用してください。

Work Unit: <IDとtitle>
Contract: <path>
対象repository/branch/base SHA: <...>

承認済みContractの範囲だけを実装してください。
指定build/checkを実行してください。
既存user変更を保持してください。
このtaskではcommit、push、merge、PR作成、simulation、実機実行を行わないでください。

完了時は、変更file、設計判断、実行commandと結果、未確認項目、review重点を報告してください。
```

実装taskは、Contract内のsource変更、build/check、自己点検まで自走できる。

次の場合は自動でscopeを広げず停止させる。

- branchまたはbase SHA不一致。
- Contractと現行コードの重要な矛盾。
- 安全仕様の未決定。
- userの既存変更との衝突。
- 別repositoryの未確定schemaが必要。
- simulationまたは実機判断が必要。

## 4.3 review task

実装taskとは別のtaskまたは`/review`を使用する。

```text
$wbms-review-work-unit を使用してください。
この変更をread-onlyでreviewしてください。sourceを変更しないでください。

Work Unit: <IDとtitle>
Scope: uncommitted changes
Base: <base branchまたはSHA>
Contract: <path>

P0/P1/P2/P3でfindingを分類してください。
各findingにfile:line、実行経路、破られる仕様、安全上の影響、再現条件、最小修正方針を含めてください。
```

reviewerには修正させない。

## 4.4 finding修正

元のimplementation taskへreview結果を渡す。

```text
次のreview findingを確認してください。
<finding全文>

承認済みContractと正式計画に照らして、findingごとに採用、not applicable、後続化を判断してください。
採用するfindingだけを最小差分で修正し、指定build/checkを再実行してください。
commitは行わないでください。
```

reviewerの提案を無条件に採用しない。scopeが変わる場合はPlanへ戻る。

## 4.5 fresh review

修正後は最新diff全体を別review taskで再確認する。

```text
$wbms-review-work-unit を使用してください。
前回finding修正後の最新diff全体を、最初からfresh reviewしてください。
前回の変更箇所だけに限定しないでください。
sourceは変更しないでください。
```

P0/P1/P2がなくなるまで繰り返す。回数で打ち切らない。

## 4.6 closure

fresh review完了後、Close Skillを使用する。

```text
$wbms-close-work-unit を使用してください。

Work Unit: <IDとtitle>
Contract: <path>
最新review: <結果>

Progressをappend-onlyで更新し、commit checklistを確認してください。
この時点ではcommitしないでください。
未達項目があればREADY FOR COMMITとせず、blocking itemを具体的に報告してください。
```

## 4.7 commit

Codexが`READY FOR COMMIT`と報告し、人間が内容を確認した後だけ明示する。

```text
Work Unit <ID>についてcommitを許可します。
$wbms-close-work-unit のcommit checklistを一項目ずつ再確認してください。
今回Work Unitのfileだけをstageしてください。
全条件を満たす場合だけ、次のsubjectでatomic commitしてください。

<commit subject>

push、merge、PR作成、amend、rebaseは行わないでください。
```

---

## 5. Codexが自走できる範囲

### 自走させてよい

- AGENTS、正式計画、Progress、Contractの読込。
- branch/HEAD/dirty stateの確認。
- read-only code investigation。
- Work Unit Contract作成。
- Contract内source実装。
- static check、指定build。
- compile errorのContract内修正。
- diff自己点検。
- detached read-only review。
- finding修正。
- Progress entry案の作成。
- commit readiness確認。
- 明示許可後の限定的なatomic commit。
- 既存logの解析。

### 人間の確認を必須とする

- Work Unit Contract承認。
- scope変更。
- safety invariant変更。
- schemaの非互換変更。
- branch baseまたはsynthetic baselineの最終採用。
- simulation scenarioの実行許可。
- 実機実行。
- commit許可。
- push、merge、PR作成。
- release-ready判定。

### Codexへ任せない

- user変更の無断reset、stash、checkout、clean。
- 同じsource fileを複数taskで同時編集。
- schema未確定でproducerとconsumerを並行実装。
- simulation未実行をPASS扱いすること。
- 実機未確認を安全確認済みとすること。
- stale中の未実現commandを後から再生すること。

---

## 6. Simulation verification gate

## 6.1 既定方針

simulationは自動実装処理と分離する。

Contractにsimulation確認が必要と記載された場合、Codexはbuild/check完了後に一旦停止し、次の形式で人間へ確認を求める。

```markdown
# Simulation verification request

## Work Unit

## Purpose

## Preconditions
- branch / SHA
- dependency SHA
- build result
- required configuration

## Scenario
- initial posture
- command sequence
- duration
- expected mode transitions

## Commands proposed

## Logs to collect

## Metrics and acceptance thresholds

## Safety / abort conditions

## Items Codex can analyze after log collection
```

明示許可がない状態で、simulation processを起動したり、制御commandを送ったりしない。

## 6.2 人間が実行する方式

初期段階では、人間が普段の手順でsimulationとlog取得を行い、次をCodexへ渡す方式を既定とする。

- 実行したexact command。
- 使用branchとSHA。
- dependency SHA。
- parameter file。
- scenario操作手順。
- log path。
- 目視所見。

Codexはlogを解析し、acceptance表とProgress entryを作成できる。

## 6.3 自動化する方式

同じsimulation手順を複数Work Unitで反復する場合は、人間の既存runbookを先に文書化する。

その後、別Work Unitとして次を作成する。

- scenario起動script。
- command送信script。
- timeout。
- log収集。
- process終了処理。
- abort条件。
- 結果summary。

自動化後も、simulation実行は明示的なSkill呼出しまたは人間の許可を必要とする。

実機手順をsimulation Skillへ混ぜない。

## 6.4 推奨する段階

1. 最初の1〜2回は人間が既存手順で実行する。
2. exact command、依存関係、失敗しやすい点をProgressへ記録する。
3. 手順が安定した後、simulation runbook用の独立Work Unitを作る。
4. scriptとSkillをread-only reviewする。
5. 以後は明示許可付きで半自動実行する。

---

## 7. Modelとmodeの選択

### GPT-5.6 Sol High / Extra High

- branch archaeology。
- Work Unit計画。
- safety-critical `auto_stabilizer`変更。
- walking preparation。
- COM/ZMP/`refdz`/`omega`/`l`。
- external IK priority設計。
- protocol全体review。

### GPT-5.6 Terra Medium / High

- schema確定後のmessage/IDL実装。
- bridge mapping。
- diagnostics。
- build error修正。
- scopeが明確な通常Work Unit。

### GPT-5.6 Luna

- 完全に固定されたfield mapping。
- enum表の機械的反映。
- Progress templateの定型追記。

Luna単独へ安全判断、mode ownership、IK priority、COM/ZMP変更を任せない。

### Max

単一taskとして非常に難しく、深い検討が必要な場合だけ使用する。

### Ultra

独立して分割できるread-only調査や、schema固定後の別repository調査に限定する。

同じsource fileの並行編集には使用しない。

---

## 8. Taskを切り替える目安

新taskにする。

- 次Work Unitへ進む。
- planningからimplementationへ移る。
- implementationからdetached reviewへ移る。
- unrelatedなrepositoryへ移る。
- contextが大きくなり、Progressで引き継げる状態になった。
- sourceとsimulation/log解析を分離できる。

同じtaskを続けてよい。

- 同一Contract内の実装。
- 同一Contract内のcompile error修正。
- review finding修正。
- 同じWork Unitのbuild/check再実行。

context圧縮へ依存する前に、ProgressとContractを更新して新taskへ引き継ぐ。

---

## 9. 文書言語

人間が読むことを前提とする次の文書は日本語で記述する。

- Implementation PlanとRevision。
- Progress。
- Codex Workflow。
- Official Guidance。
- 本Operator Guide。
- AGENTS.md。
- Work Unit Contract。
- review result。
- コメント。
- Skill本文と`description`。

以下は英語またはASCII identifierを維持してよい。

- file名。
- Skillの`name` slug。
- class、function、variable、topic、message、IDL field名。
- command。
- commit subject。
- schema enum identifier。
- 外部API・公式資料の固有名。

日本語本文の中で英語の技術用語を使う場合は、意味が曖昧にならないよう統一する。

---

## 10. 次の具体的操作

1. planning branchをLocalへ取得する。
2. `git diff --check`を実行する。
3. Codexにactive `AGENTS.md` chainとSkill一覧を列挙させる。
4. `/review`または`$wbms-review-work-unit`でplanning文書群を正式reviewする。
5. P0/P1/P2を解消する。
6. M0-A planning taskを開始する。

Skill認識確認用prompt:

```text
このrepositoryで現在適用されているAGENTS.mdのchainを、適用順とpath付きで列挙してください。
また、利用可能なrepository-local Skillのname、description、pathを列挙してください。
fileは変更しないでください。
```

planning文書review用prompt:

```text
$wbms-review-work-unit を使用し、wbms-devをbaseとしてwbms-external-teleop-planの文書、AGENTS、Skill差分をread-only reviewしてください。
sourceは変更しないでください。

重点:
- 文書間の優先順位と矛盾。
- M0-M12の依存順。
- branch/synthetic baseline手順。
- external-onlyとlegacy-onlyの開始条件。
- Skillのscope境界。
- simulation gate。
- Progress引き継ぎ可能性。
- commit/checklistの安全性。

control source未変更、local build/simulation未実施をそれ自体ではfindingにしないでください。
```
