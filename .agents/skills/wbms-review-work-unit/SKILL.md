---
name: wbms-review-work-unit
description: Perform a read-only, dedicated code review of one WBMS external whole-body teleoperation Work Unit. Use for an uncommitted diff, exact commit, or branch diff after implementation. Report prioritized P0-P3 findings with evidence and do not modify files, apply fixes, stage, commit, push, merge, or open a PR.
---

# WBMS Work Unit専用レビュー

## 目的

実装taskから独立したreviewerとして、一つのWork Unitの最新diffをread-onlyで検査する。

findingは、実行経路、安全不変条件、interface contract、受入条件に基づいて報告する。styleだけの一般論や、今回scope外の将来機能をblocking findingにしない。

## 必須入力

- Work Unit IDとtitle。
- 対象repository、branch、base SHA。
- review scope。
  - uncommitted changes。
  - exact commit。
  - base branchとの差分。
- 承認済みWork Unit Contract。
- 正式Implementation Plan。
- 最新Progress entry。

review対象が曖昧な場合は、対象diffを特定するまでfindingを作らない。

## 読む順序

1. repository rootの`AGENTS.md`。
2. nearest package/module `AGENTS.md`。
3. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
4. `WBMSExternalWholeBodyTeleoperationProgress.md`。
5. Work Unit Contract。
6. `WBMSExternalWholeBodyTeleoperationCodexWorkflow.md`。
7. 該当する既存正式仕様。
8. review対象diffと周辺source。

古いprojection IK文書を新計画より優先しない。

## Read-only制約

このSkillでは次を行わない。

- source、文書、設定fileの変更。
- findingの自動修正。
- stage、commit、push、merge、PR作成。
- branch切替、reset、stash、clean。
- simulationまたは実機実行。

buildや静的checkは、review環境と権限が明示されている場合だけ実行してよい。実行した場合もworking treeを変更するgenerator出力をcommit対象にしてはならない。

## Review severity

### P0 Critical

- 実機へ危険な不連続指令を出す。
- joint limit、collision、足拘束、接触安全を破る。
- stale後に過去commandを再生する。
- NaN、未初期化、data corruption。
- schema解釈違いで別関節や別taskへ指令する。
- build不能でWork Unitが使用不能。

### P1 High

- Work Unitの主要仕様・受入条件を満たさない。
- walking preparation、READY、歩行API gate、ownershipを壊す。
- 500 Hz経路へblocking処理を追加する。
- session、epoch、sequence、staleで誤taskを有効化する。
- COM/ZMP/`refdz`/`omega`/`l`が不連続または不整合になる。
- legacy経路へ重大な回帰を入れる。

### P2 Medium

- failure原因を切り分けられないdiagnostics不足。
- interface validation不足。
- 次Work Unitを危険にする責務混在。
- 必要なbuild/check/Progress記録が欠落。
- 計画・Contract・実装が矛盾する。

### P3 Low / Suggestion

- 命名、局所的可読性、optional cleanup。
- 小さなperformance改善。
- 今回scope外の将来拡張。

P3はcommitを止めない。実害を説明できないstyle findingはP3以下にする。

## 重点確認

### Protocol / bridge

- ROS message、RTM IDL、bridge mappingの全field。
- 単位、frame、quaternion順序。
- task mask、enum、schema version。
- timestamp、session、epoch、sequence。
- out-of-order、duplicate、unknown joint。
- command/stateのatomicity。
- source timestampをbridge時刻で上書きしていないか。

### State machine / stale

- task group別enabled、valid、stale、hold。
- generator/bridge heartbeat。
- zero velocity freshとstaleの区別。
- same-epoch rebase。
- epoch変更時の再enable。
- stale中の未実現commandを復帰後に回収していないか。

### Frames / control

- device world、foot-mid、CHEST相対、generate worldの取り違え。
- hand baseline、position scale、rotation delta。
- CHEST/COM速度のfoot-mid解釈。
- accepted generated state基準の局所target。
- hidden goalの有無。

### Safety / walking

- joint position/velocity limit。
- self collision。
- 両足pose、support phase、static gate。
- walking preparation ownership。
- walking中CHEST/COM無効、腕/頭部継続。
- COM、`genCog`、`sbpOffset`、ZMP、`refdz`、`omega`、`l`。
- final validationとreject/hold。

### Non-IK direct joint

- qRef読込後、FK/COM前にoverrideしているか。
- upstream q/dqを毎周期復元しているか。
- allowlistと`jointControllable=false`確認。
- finite、position、velocity、acceleration limit。
- stale hold、WBMS start/stop blend。
- final IK後だけ上書きしてmodelと出力を不一致にしていないか。

### Performance

- 500 Hz経路のROS callback、network I/O、blocking I/O。
- condition variable、mutex待ち。
- 毎周期thread生成。
- 無制限queue、clone、全探索、allocation。
- external process停止時に500 Hzを待たないか。

### Repository / workflow

- Contract外のfileやcleanupが混在していないか。
- compatible dependency SHA。
- exact build/check結果。
- simulation/実機未確認をPASS扱いしていないか。
- Progress、計画、実装の一致。

## 問題として扱わない事項

次を単独理由でfindingにしない。

- `clang-format`未適用。
- `auto_stabilizer`で新規unit testを追加していないこと。
- コメントとMarkdownが日本語であること。
- legacy portを初回実装で残していること。
- pre-M5 final IKと`maxIteration=1`を初期baselineにしていること。
- single-QP final WBCを今回実装していないこと。
- walking中CHEST/COM操作を未実装であること。
- simulationや実機を現在の環境で実行できないこと自体。これは`UNVERIFIED`であり、虚偽のPASS表記がある場合だけfindingにする。
- 古い文書に不採用案が残っていること。
- future extensionを実装していないこと。

## Finding形式

findingがある場合、重要度順に並べる。

```markdown
## Findings

### [P1] <短いtitle>
- Location: `path/file.cpp:123`
- Execution path:
- Broken specification or invariant:
- Reproduction condition:
- Impact:
- Minimal correction direction:
- Evidence:
```

同じ根本原因による複数箇所は一つのfindingにまとめる。

曖昧な可能性だけでfindingを作らず、具体的な実行経路と影響を示す。

## Findingがない場合

```markdown
## Findings

P0/P1/P2 findingはありません。

## Residual risks / unverified
- ...

## Review scope
- ...
```

P3提案がある場合はblocking findingと分ける。

## 完了条件

- review対象が明示されている。
- 最新diff全体を読んでいる。
- 周辺call pathを確認している。
- P0-P3分類が一貫している。
- findingにfile:line、実行経路、破られる仕様、最小修正方針がある。
- 未確認事項がfindingと分離されている。

このSkill使用中はsourceを変更しない。