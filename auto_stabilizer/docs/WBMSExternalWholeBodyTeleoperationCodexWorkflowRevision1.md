# WBMS外部Whole-Body操縦 Codexワークフロー Revision 1

## 1. 文書の位置づけ

本書は、`WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`および
`WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`に定義された作業手順を、
安全性を維持したまま簡略化するための最新workflow revisionである。

本書は**制御仕様を変更しない**。以下は引き続き従来の正式計画を正本とする。

- 外部reference generatorの責務。
- 500 Hz `auto_stabilizer`の責務。
- walking preparation、READY、walking API gate。
- joint limit、self collision、足拘束、COM/ZMP、安全validation。
- message、IDL、frame、unit、session、stale等のinterface仕様。
- one-write-repository rule。
- simulation・実機の人間gate。

workflow、review頻度、Progress頻度、commit前確認について本書と旧文書が矛盾する場合、
**本書を優先する**。

参照順:

1. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
2. 本書
3. `WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md`
4. `WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
5. `WBMSExternalWholeBodyTeleoperationImplementationPlanRevision1.md`
6. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`
7. 最新のParent Work Package / Work Unit Contract
8. `WBMSExternalWholeBodyTeleoperationProgress.md`

---

## 2. 変更理由

旧workflowは、実機安全に直結する変更へ適用する最大保証型processとしては妥当だった。
一方、M0の文書、bootstrap、Progress同期にも同じprocessを適用したことで、次が発生した。

- Progress追記だけのために独立Work Unit、fresh review、commit readinessを反復した。
- repository commitごとに中央Progress commitを必須gateとした。
- typo、compile fix、文書追記後にも最新diff全体のfresh reviewを要求した。
- review済みsourceへProgressを追加しただけでsource reviewを無効化した。
- 安全重要変更と管理作業のreview強度が同一になった。

本revisionでは、作業を小さく保つ原則は維持しつつ、
review・Progress・commit gateを**risk boundaryと自己完結した成果**へ移す。

---

## 3. 維持する不変条件

以下はrisk levelにかかわらず維持する。

### 3.1 Repository境界

```text
一つのimplementation実行
  = 一つのWRITE repository
```

- sibling repositoryは明示されたSHAをREADするだけとする。
- 一つのcommitで複数repositoryをstageしない。
- schema未確定のproducer、bridge、consumerを並行実装しない。
- userの変更を無断でreset、stash、checkout、clean、削除しない。

一つのParent Work Packageの中で、異なるrepository sub-unitを順に自走することは許可する。
各sub-unitのWRITE repositoryは一つだけでなければならない。

### 3.2 500 Hz・安全

- `auto_stabilizer::onExecute()`へROS callback、network I/O、blocking waitを追加しない。
- joint limit、collision、足拘束、接触、歩行安定化をoperator taskより優先する。
- stale、invalid、solver failure時はcurrent accepted/generated stateをholdする。
- 未実現commandをhidden goalとして蓄積しない。
- walking preparation、READY、walking API gate、COM高さ保持を維持する。
- `q_nominal`をhardwareへ直接出力しない。
- safety findingをweight調整だけで隠さない。

### 3.3 Simulation・実機

- simulationの起動、command送信、log取得はユーザーの明示許可を必要とする。
- 実機実行は常に別の明示許可を必要とする。
- 未実施項目を`PASS`と記載しない。
- simulation・実機前の必要なreviewは省略しない。

---

## 4. Risk level

各Parent Work Packageまたはrepository sub-unitは、開始時に最高riskを一つ選ぶ。

## R0: 管理・文書・bootstrap

例:

- Progress、Current Checkpoint。
- `AGENTS.md`、Skill、Project Context、workspace manifest。
- package skeleton。
- branch/bootstrap記録。
- build手順、comment、Markdown。
- symlink配置、repository初期化。
- control behaviorを変えないdiagnostic文書。

必須:

- scopeと対象pathの確認。
- `git diff --check`または`git diff --cached --check`。
- XML/YAML/JSON等の必要な構文確認。
- package skeletonならcatkin package discovery。
- unrelated/generated fileがないこと。
- one-write-repository rule。

通常不要:

- sub-unitごとのdetached review。
- Progress-only diffのfresh full review。
- package build。
- simulation。
- commit前の関連control source再走査。

R0はParent Work Package末尾のfocused document/bootstrap reviewへbatchしてよい。

## R1: Interface・scaffolding・transport

例:

- ROS `.msg`。
- RTM IDL。
- enum、task mask、schema version。
- CMake/package metadata。
- bridgeの機械的field mapping。
- heartbeat/diagnostic schema。
- launch skeleton。

必須:

- Parent protocol contract。
- field、type、unit、frame、quaternion順序のmapping。
- targeted package build。
- compatible-set review。
- 可能な場合のround-trip確認。

各repository sub-unitで独立full reviewを繰り返さず、
compatible setが揃った時点でcross-repository reviewを一回行う。

## R2: 非リアルタイム制御ロジック

例:

- external reference generator。
- 手差分mapping、HMD mapping、head recenter。
- CHEST/COM局所target生成。
- external whole-body IK。
- external collision preview。
- external task state machine。

必須:

- coherent feature Contract。
- package build。
- repository full reviewを一回。
- failure、stale、frame、hidden goalの確認。
- simulation前review。

小さな修正はtargeted follow-upでよい。
state machine、frame、priority、constraint、fallbackが変わる場合はfull fresh reviewする。

## R3: リアルタイム・実機安全

例:

- `auto_stabilizer` 500 Hz経路。
- COM/ZMP、`genCog`、`sbpOffset`、`refdz`、`omega`、`l`。
- walking preparation、READY、walking API gate。
- final IK、projection削除。
- joint limit、collision、足拘束。
- stale時の最終出力。
- non-IK joint override。
- mode ownershipとfailure path。

必須:

- full Work Unit Contract。
- package build。
- dedicated safety full review。
- P0/P1/P2の解消。
- material修正後のfull fresh review。
- simulation gateとlog確認。
- 実機gate。
- commitごとの人間による明示承認。

---

## 5. Work Packageとsub-unit

### 5.1 Parent Work Package

複数のcoherent sub-unitを一つの成果としてまとめる。

例:

```text
M1-P protocol Work Package
  M1-A ROS message
  M1-B RTM IDL
  M1-C bridge mapping
  M1-D compatible-set review
  M1-E package build確認
```

Parent Work Packageで固定する。

- outcome。
- repository dependency順。
- risk level。
- stop conditions。
- review boundary。
- Progress checkpoint。
- commit authorization。
- simulation/hardware gate。

### 5.2 Repository sub-unit

- WRITE repositoryは一つ。
- Parent Contractの範囲を暗黙に広げない。
- repository commitはatomicに保つ。
- Parent Work Package内で次のeligible sub-unitへ連続して進めてよい。
- 各micro-stepのために新しい中央Progress entryを作らない。

### 5.3 R0 Work Brief

R0ではfull Contractの代わりに短いWork Briefを使ってよい。

```markdown
# Work Brief

- Parent Work Package:
- Goal:
- Risk: R0
- WRITE repository/path:
- READ repositories:
- Allowed paths:
- Verification:
- Review boundary:
- Commit authorization:
- Stop conditions:
```

R1〜R3は影響に応じたContractを使用する。

---

## 6. Review policy

Review type:

```text
SELF
TARGETED
REPOSITORY_FULL
COMPATIBLE_SET
SAFETY_FULL
```

| Risk | 既定review |
|---|---|
| R0 | SELF。Work Package末尾にfocused reviewを一回 |
| R1 | repository SELF + COMPATIBLE_SETを一回 |
| R2 | REPOSITORY_FULLを一回。修正後は原則TARGETED |
| R3 | SAFETY_FULL。material修正後はfull fresh review |

### 6.1 Full fresh reviewが必要な条件

以下のいずれかに該当する場合だけ、最新source diff全体をfresh reviewする。

1. P0またはP1を修正した。
2. P2修正で設計または実行経路が変わった。
3. message/IDLのfield、type、unit、frame、enumを変えた。
4. state machine、session、stale、mode gateを変えた。
5. joint limit、collision、足、COM/ZMPを変えた。
6. thread、mutex、blocking、allocationモデルを変えた。
7. solver variable、constraint、priorityを変えた。
8. simulationまたは実機へ進む直前。
9. reviewerが前提崩壊を理由にfull reviewを指定した。

### 6.2 Targeted follow-upでよい変更

- include漏れ。
- compile error修正。
- typo、comment、Markdown。
- Progress、Current Checkpoint。
- diagnostic名。
- pathやcommandの訂正。
- behaviorを変えない局所的なguard。
- P3 finding。
- reviewed sourceに影響しない記録追記。

### 6.3 Progress-only変更

ProgressまたはCurrent Checkpointだけのdiffは、control source reviewを無効化しない。
内容の参照整合と構文をSELF確認し、独立detached reviewを通常要求しない。

---

## 7. Verification evidenceの再利用

review/build結果には、対象を識別できる証拠を付ける。

commit済みの場合:

```text
verified commit SHA
```

uncommittedの場合の候補:

```sh
git diff --binary --no-ext-diff | sha256sum
```

記録する。

- verified SHAまたはdiff hash。
- command。
- execution directory。
- result。
- affected package/file。
- review type。

### 7.1 Evidence再利用条件

review/build後に対象source diffが変わっていない場合:

- source全体の再読は不要。
- package buildの再実行は不要。
- full reviewの再実行は不要。

Progress、Current Checkpoint、Markdownだけが後から変わった場合、
source review evidenceは有効なままとする。

source diffが変わった場合は影響に応じて再確認する。

| 変更 | 再確認 |
|---|---|
| 文書だけ | `git diff --check` |
| C++ source | 対象package build |
| IDL/CMake | `--force-cmake`を含む指定build |
| control semantics | riskに応じたreview |
| schema semantics | compatible-set review |

---

## 8. ProgressとCurrent Checkpoint

### 8.1 Current Checkpoint

現在地は次で管理する。

```text
WBMSExternalWholeBodyTeleoperationCurrentCheckpoint.md
```

ここはmutableでよい。

必須項目:

- workflow version。
- active Parent Work Package。
- completed sub-units。
- repository commit SHA。
- next eligible action。
- pending review/build。
- simulation/hardware gate。
- blockers。

各taskは最初にCurrent Checkpointを読み、必要な履歴entryだけProgressから参照する。

### 8.2 中央Progress

中央Progressの履歴entryはappend-onlyを維持する。
更新はcheckpoint単位に限定する。

更新する:

- Parent Work Package完了。
- compatible set確定。
- architecture decision変更。
- simulation/実機結果。
- blocking issue。
- milestone完了。
- 別task・別担当への引き継ぎ。
- release candidate確定。

通常更新しない:

- typo、include、compile error修正。
- `AGENTS.md`一枚の配置。
- SHA一件の同期。
- review一回の実行。
- package skeleton一件。
- Progress自身の更新。

### 8.3 他repository commit後

旧規則:

```text
repository commit
  -> central Progress sync commit
  -> next sub-unit
```

新規則:

```text
repository commit
  -> Current CheckpointまたはParent working stateへSHA記録
  -> 次の独立sub-unit
  -> integration / simulation / milestone前に中央Progress checkpoint
```

次sub-unitのpromptにexact commit SHAが含まれていれば、
中央Progress commitを待たずに進めてよい。

---

## 9. Commit policy

### 9.1 共通最小check

```sh
git branch --show-current
git status --short
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
```

確認する。

- 正しいbranch。
- stage対象が予定pathだけ。
- unrelated/generated artifactなし。
- commit subjectが一目的。
- user変更を含めない。

### 9.2 Risk別追加check

| Risk | 追加check |
|---|---|
| R0 | 文書/XML/YAML構文、package discovery等 |
| R1 | package build、schema/mapping照合 |
| R2 | package build、repository review、failure path |
| R3 | full safety checklist、review、simulation状態 |

### 9.3 Standing authorization

ユーザーはParent Work Packageに対し、複数のR0〜R2 local commitを事前許可できる。

条件:

- Parent scope内。
- 一回に一つのrepositoryだけstage/commit。
- push、merge、PR作成なし。
- riskがR3へ上がった場合は停止。
- schema/safety invariant変更が必要なら停止。
- user変更と衝突したら停止。
- simulation/hardware前で停止。

R3はcommitごとのexact人間承認を維持する。

### 9.4 Close Skillを必須としない場合

R0/R1で、standing authorizationがあり、必要checkがPASSした場合、
implementation task内でatomic commitまで進めてよい。

Close Skillは以下で使用する。

- R3。
- compatible set freeze。
- milestone/checkpoint closure。
- commit authorizationが不明。
- verification evidenceが複数taskに分散した場合。
- release candidate。

---

## 10. 標準lifecycle

### R0

```text
Parent Work Brief承認
  -> 複数R0 sub-unitを順次実行
  -> 必要なrepository commit
  -> Work Package末尾でfocused review一回
  -> Current Checkpoint更新
```

### R1

```text
Parent protocol Contract
  -> repository sub-unit実装・targeted build・commit
  -> compatible set完成
  -> cross-repository review一回
  -> 必要な修正
  -> checkpoint Progress
```

### R2

```text
Feature Contract
  -> 実装
  -> package build
  -> repository full review
  -> targeted修正/review
  -> simulation gate
  -> checkpoint Progress
```

### R3

```text
Full Contract
  -> 実装
  -> package build
  -> safety full review
  -> P0/P1/P2修正
  -> material修正後full fresh review
  -> simulation gate
  -> log review
  -> exact commit承認
  -> 実機gate
```

---

## 11. M0-B3地点からの移行

### 11.1 完了済みとして再利用するもの

M0-B3 `whole_body_teleop` bootstrapは次が完了している。

- repository commit。
- exact 7 pathsの確認。
- repository fresh review PASS。
- XML syntax PASS。
- catkin package discovery PASS。
- staged scope PASS。
- post-commit clean。
- sibling repository非変更。

これらをやり直さない。

### 11.2 Supersedeする旧gate

M0-B3 Progress-only entryに残る次のgateは本revisionでsupersedeする。

- Progress-only diffのdetached fresh review。
- Progress-only commit readinessのための全file再走査。
- M0-B4をProgress-only review/commitまでblockする規則。

M0-B3 repository source reviewは有効なままとする。

### 11.3 残りM0

```text
WORKFLOW-R1 commit
  -> M0-REMAINDER-R0
       - M0-B4 rtmros_msg_bridge bootstrap
       - M0-B5 source-root AGENTS / common Skill setup
       - M0-B6 Project Context / manifest
       - whole_body_teleop AGENTSのworkflow同期
  -> R0 focused bootstrap review一回
  -> M0-B7 exact compatible-set package builds
  -> M0-B8 cross-repository review
  -> M0 completion checkpoint / Progress一件
  -> M1-P
```

M0-B4〜B6は一つのR0 Parent Work Packageとして承認できる。
repository commitは分離するが、各commit間の中央Progress syncは不要である。

---

## 12. Stop conditions

自走中も次の場合は停止して人間へ報告する。

- Parent scope外の変更が必要。
- riskがR3へ上がる。
- schema、frame、unit、enumの意味変更が必要。
- safety invariant変更が必要。
- sibling repositoryをWRITEする必要が生じた。
- userの未commit変更と衝突する。
- destructive Git操作が必要。
- package build failureの修正先が別repository。
- simulationまたは実機実行が必要。
- compatible SHAが不明または移動した。
- Contract/Current Checkpoint/現行コードが重大に矛盾する。

---

## 13. Prompt共通末尾

各planning/implementation/review taskの末尾に次を含める。

```text
完了時に以下を出力してください。

## Workflow state
- risk level
- Parent Work Package
- completed sub-units
- repository commit SHAs
- verification evidence
- pending review/build
- blockers

## Next mandatory action
- next Work Package / sub-unit
- Codex launch directory
- WRITE/READ repositories
- required Skill or review type
- human approval gate
- copy-paste prompt

stop conditionが発生していない場合は、承認済みParent Work Package内の
次eligible sub-unitへ進んでよいかも明記してください。
```

---

## 14. 旧規則の扱い

以下の旧規則は一律適用しない。

- 全Work Unitでdetached review必須。
- あらゆる修正後に最新diff全体のfresh review必須。
- Progress更新前のcommit禁止。
- repository commitごとの中央Progress sync commit。
- Progress-only変更のための独立Contract/review/closure。
- commit前に関連する全sourceを最初から再走査。
- 各micro-stepを独立Work Unitとすること。

ただしR3、schema意味変更、compatible-set integration、simulation/実機前には、
本書で定義した厳格なreviewと人間gateを適用する。
