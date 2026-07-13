# WBMS外部Whole-Body操縦Reference Generator 実装計画 Revision 1

## 1. 文書の位置づけ

本書は`WBMSExternalWholeBodyTeleoperationImplementationPlan.md`の初回文書reviewで判明した曖昧点を修正する。

以下の項目については、本書を元計画より優先する。それ以外は元計画を維持する。

参照順:

1. 本書。
2. `WBMSExternalWholeBodyTeleoperationImplementationPlan.md`。
3. `WBMSExternalWholeBodyTeleoperationProgress.md`。
4. 対象Work Unit Contract。

本書の内容はM0のbranch archaeologyとM1のprotocol contractへ反映し、実装branch作成後に必要であれば元計画へ統合する。

---

## 2. 実装branch基点の修正

元計画は「M4.2.2の全修正完了後、最初のM5変更前のcommit」を基点候補としている。

しかし、実際の履歴でM4.2.2 review修正とM5変更が時間的に交錯している場合、必要条件を全て満たす単一commitが存在しない可能性がある。

M0-Aでは次の順で判断する。

### 2.1 第一候補

次を同時に満たす単一commitが存在する場合、そのSHAをbaseとする。

- CHEST相対腕拘束を含む。
- M4.2.2 walking preparationの正式仕様を含む。
- M4.2.2の安全上必要なreview修正を全て含む。
- M5のprojection/final IK計算量削減変更を含まない。
- build可能である。

### 2.2 単一commitが存在しない場合

synthetic baseline branchを構築する。

1. 最初のM5変更直前のpre-M5 commitからbranchを作る。
2. 後続commitから、walking preparation、CHEST相対腕拘束、安全review修正だけを一件ずつ分類する。
3. 必要なcommitまたは最小diffだけをcherry-pickする。
4. M5のsolver、workspace、profiling、projection軽量化変更は混入させない。
5. conflict解消内容をProgressへ記録する。
6. `1.0`、元pre-M5、synthetic baseline、`wbms-dev`の差分表を作る。
7. build、simulation可能な範囲のbaseline確認を行う。

synthetic baselineを採用した場合、そのbranch HEADを本プロジェクトの実装base SHAとして固定する。元commit SHAだけでなく、採用したcherry-pick集合と除外したM5 commitをProgressへ記録する。

### 2.3 禁止事項

- 現在の`wbms-dev` HEADからprojection/M5コードを大量削除するだけで実装baseとしない。
- walking preparation review修正を「pre-M5でない」という理由だけで落とさない。
- M5変更を一括cherry-pickしない。
- branch履歴だけを見て判断せず、最終コードと正式文書を照合する。

---

## 3. WBMS開始heartbeat条件の修正

元計画19.1のheartbeat必須条件を次へ置換する。

### 3.1 external sourceを使用する場合

`arm_reference_source`、`posture_reference_source`、`head_reference_source`のいずれかでexternal bundleを選択している場合、`startWholeBodyMasterSlave()`には次を要求する。

- generator heartbeat fresh。
- bridge heartbeat fresh。
- schema version compatible。
- session/epoch handshake可能。

個別task inputはoptionalとする。

- 右手inputなし: 現在右手pose hold。
- 左手inputなし: 現在左手pose hold。
- CHEST inputなし: 速度ゼロ。
- COM inputなし: 速度ゼロ。
- HMD inputなし: 現在首角hold。

### 3.2 全taskがlegacy/upstream sourceの場合

external generatorとbridgeのheartbeatを要求しない。

- 従来どおり`startWholeBodyMasterSlave()`を開始可能とする。
- external bundle未接続によってlegacy運用を退行させない。
- external portに古いdataが残っていても、source selectorがlegacyなら採用しない。

### 3.3 一部taskだけexternalの場合

externalを選択したtaskだけにheartbeat/session/stale policyを適用する。

例:

```text
arm_reference_source = LEGACY
posture_reference_source = EXTERNAL_BUNDLE
head_reference_source = UPSTREAM_QREF
```

この場合:

- generator/bridge heartbeatは必須。
- external CHEST/COMだけをbundleから使用する。
- 腕はlegacy port。
- 頭部は上流qRef。
- external右手・左手fieldが存在しても無視する。

### 3.4 source変更

- source selector変更はWBMS停止中だけを初期仕様とする。
- 実行中のlast-writer-winsや自動source切替を行わない。
- source変更後の次sessionでbaselineとheartbeat条件を再評価する。

---

## 4. Codex mode名の扱い

特定のCodex UI名へ仕様を依存させない。

- planningは「sourceを変更しないread-only planning phase」と定義する。
- clientに明示的なplan capabilityがある場合は使用してよい。
- interactive CLIで長期実装を行う場合は、公式資料に従い`/goal`を使用できる。
- reviewは`/review`または同等のread-only dedicated reviewerを使用する。
- model、reasoning level、Max、Ultraは現行model pickerと公式資料を確認する。

詳細は`WBMSExternalWholeBodyTeleoperationCodexOfficialGuidance.md`を参照する。

---

## 5. Skill完成条件

計画段階で次のrepository-local Skillを用意する。

```text
.agents/skills/
  wbms-plan-work-unit/SKILL.md
  wbms-implement-work-unit/SKILL.md
  wbms-review-work-unit/SKILL.md
  wbms-close-work-unit/SKILL.md
```

- Plan: read-only調査とContract。
- Implement: Contract内実装、build/check、commitなし。
- Review: read-only finding、修正なし。
- Close: Progress、checklist、commit readiness、明示許可時だけcommit。

Skillはtask固有仕様を複製せず、本Revision、元計画、Progress、Contractを読む。

---

## 6. M0 acceptanceへの追加

M0 acceptanceへ次を追加する。

- 単一base commitかsynthetic baselineかが明記されている。
- synthetic baselineの場合、cherry-pick一覧、除外一覧、conflict解消が記録されている。
- external source選択時とlegacy-only時のWBMS開始条件が別々に確認されている。
- planning branchの文書commit群を実装branchへ移す手順が記録されている。
- four Work Unit SkillsがCodexから認識される。
- active `AGENTS.md` chainをCodexに列挙させ、rootとpackage instructionが読まれている。
