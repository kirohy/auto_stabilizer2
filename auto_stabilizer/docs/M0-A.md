# Work Unit Contract: M0-A branch archaeology and exact baseline selection

## Goal

外部Whole-Body操縦実装の基点として、M4.2.2 walking preparationの全安全修正とCHEST相対腕拘束を保持し、旧M5計算量削減を含まないexact cross-repository baselineを固定する。

採用するnamed compatible setは`M0-pre-M5-baseline`とし、次のSHAを固定する。

| repository | ref | SHA | status |
|---|---|---|---|
| `auto_stabilizer2` | historical `wbms-dev` ancestor | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `2.0` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `master` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | 未作成 | — | NOT APPLICABLE |

`auto_stabilizer2`では必要な安全修正が全て最初の旧M5 commitより前に直列履歴として存在するため、synthetic baselineは不要である。

ここで「旧M5」は`WBMSComputationReductionImplementationPlan.md`の500 Hz計算量削減を指す。新アーキテクチャ計画のM5 non-IK head referenceとは別物である。

M0-Aは、M0-A-R1による本Contractとplanning assetの修正、fresh review、人間承認、document-only commit、M0-A5 central Progress syncが完了するまで完了扱いにしない。

## Workspace context

- catkin workspace root: `/home/kirohy/catkin_ws/cnoid2`
- source root: `/home/kirohy/catkin_ws/cnoid2/src`
- M0-A planning launch directory: `/home/kirohy/catkin_ws/cnoid2/src`
- repository implementation launch directory: `/home/kirohy/catkin_ws/cnoid2/src/auto_stabilizer2`
- M0-A-R1 fresh review launch directory: `/home/kirohy/catkin_ws/cnoid2/src`
- dedicated baseline workspace候補: `/home/kirohy/catkin_ws/cnoid2_m0_baseline`
- dedicated baseline source root候補: `/home/kirohy/catkin_ws/cnoid2_m0_baseline/src`

現在の`/home/kirohy/catkin_ws/cnoid2/src`にはuserのdirty checkoutが存在する。M0 baselineのbranch materializationとbuildには使用しない。隔離方式、exact path、worktree配置、underlay条件はM0-B1で人間承認を得てから確定する。

## Repository access

M0-Aのbranch/source archaeologyは全repository read-onlyで行った。M0-A-R1だけは、別の承認済みContractに従い`auto_stabilizer2`のplanning文書4ファイルだけをWRITEする。

| repository | path | branch | selected base SHA | planning時current SHA | M0-A access |
|---|---|---|---|---|---|
| `auto_stabilizer2` | `/home/kirohy/catkin_ws/cnoid2/src/auto_stabilizer2` | `wbms-external-teleop-plan` | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | `6e530edacecf663f2eedbdcfd5a787468dad70ce` | READ |
| `ik_solvers2` | `/home/kirohy/catkin_ws/cnoid2/src/ik_solvers2` | observed `teleop-dev` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | `47576209a01a35177ac0d594e586abfa90927dd7` | READ |
| `prioritized_qp` | `/home/kirohy/catkin_ws/cnoid2/src/prioritized_qp` | observed `teleop-dev` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | `7ce17d8e80a3b3a7fc8d24187d167b8b5055c9fd` | READ |
| `rtmros_msg_bridge` | `/home/kirohy/catkin_ws/cnoid2/src/rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | 同左 | READ |
| `whole_body_teleop` | `/home/kirohy/catkin_ws/cnoid2/src/whole_body_teleop` | 未作成 | — | — | NONE |

確認済みdirty state:

- `auto_stabilizer2`: `.cache/`、`compile_commands.json`、`docs/M0-A-R1.md`、`docs/M0-A.md`、`log/`がuntracked。
- `ik_solvers2`: cache、compile database、未追跡constraint sourceが存在する。
- `prioritized_qp`: clean。
- `rtmros_msg_bridge`: cache、compile database、生成message群がuntracked。
- これらを削除、上書き、移動、stage、checkout、reset、stash、cleanしない。

## Active instructions

- source-root `AGENTS.md`: 未配置。配置、4 common Skillのuser-level installation、各repository rootからの認識確認はM0-B5が単独所有する。
- `auto_stabilizer2/AGENTS.md`: 適用する。
- `auto_stabilizer2/auto_stabilizer/AGENTS.md`: 適用する。
- sibling repository root `AGENTS.md`: `ik_solvers2`、`prioritized_qp`、`rtmros_msg_bridge`には未配置。
- repository Project Context: 対象repositoryには未配置。
- 共通4 Work Unit Skillの正本: `auto_stabilizer2/.agents/skills`。

未配置instructionとProject ContextはM0-B bootstrap対象であり、historical baseline SHAを変更する根拠にはしない。Project Contextの配置または正式defer判断とworkspace manifestはM0-B6が単独所有し、M0-B3/M0-B4では変更しない。

## Parent Work Unit

- Parent ID: `M0`
- task type: cross-repository Parent Work Unit
- schema/protocol version: `N/A`
- compatible set name: `M0-pre-M5-baseline`
- runtime interface変更: なし

M0-A内部の責務は次のとおりとする。

| unit | access | sole responsibility | status |
|---|---|---|---|
| `M0-A1` | READ `auto_stabilizer2` | branch/source archaeologyとexact base選定 | 調査完了 |
| `M0-A2` | READ solver repositories | pre-M5 dependency SHA選定 | 調査完了 |
| `M0-A3` | — | executable sub-unitとして廃止。build commandの計画だけを本Contractへ残す | build実行なし |
| `M0-A4` | READ historical logs/docs | walking preparation baseline evidence確認 | historical evidenceのみ |
| `M0-A-R1` | WRITE `auto_stabilizer2`のexact 4 files | 欠落Contractとplanning assetのdocument-only修正 | implementation、review待ち |
| `M0-A5` | WRITE `auto_stabilizer2`の中央Progressのみ | approved baseline、M0-A-R1 commit SHA、`UNVERIFIED`状態のappend-only同期 | M0-A-R1 commit後に実施 |

正式なdependency順は次で固定する。

```text
M0-A1 branch/source archaeology
  -> M0-A2 dependency SHA selection
  -> M0-A4 historical walking evidence確認
  -> M0-A-R1 planning asset correction
  -> M0-A-R1 fresh review from CATKIN_SOURCE_ROOT
  -> 人間によるbaseline・diff・commit承認
  -> M0-A-R1 document-only commit
  -> M0-A5 central Progress sync
  -> M0-A complete
  -> M0-B1
  -> M0-B2
  -> M0-B3
  -> M0-B4
  -> M0-B5
  -> M0-B6
  -> M0-B7
  -> M0-B8
```

## Scope

M0-Aのread-only調査範囲:

- 正式Implementation Plan Revision 2、MultiRepositoryOperations、Revision 1、元計画、最新Progress。
- walking preparation正式文書と旧ProgressのM4.2.2/旧M5境界。
- `auto_stabilizer2`のbranch historyと対象source。
- `ik_solvers2`、`prioritized_qp`の旧M5 dependency history。
- `rtmros_msg_bridge`のbranch、HEAD、M0 baselineとの関係。
- historical walking preparation logの存在と記録済み判断。

M0-A-R1のdocument correction scopeは、承認済み`M0-A-R1.md`で指定された次の4ファイルだけとする。

- `auto_stabilizer/docs/M0-A.md`
- `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationImplementationPlanRevision2.md`
- `auto_stabilizer/docs/WBMSExternalWholeBodyTeleoperationMultiRepositoryOperations.md`
- `tools/codex_workspace/templates/WBMSExternalTeleopWorkspaceManifest.template.yaml`

## Out of scope

- control source、IDL、EusLisp、CMake、package metadataの変更。
- `AGENTS.md`、Work Unit Skill、中央ProgressのM0-A-R1内変更。
- sibling repositoryの変更。
- branch作成、checkout、cherry-pick、rebase、worktree作成。
- stage、commit、push、merge、PR作成。
- package build、simulation、実機実行。
- projection IK削除またはexternal reference実装。
- `whole_body_teleop` repository作成。
- M4.2.2のoptional absolute-root-upright条件追加。
- 旧M5変更の選択的再採用。
- userのdirty/untracked file整理。

## Current behavior

### Branch archaeology

| purpose | exact commit | evidence |
|---|---|---|
| CHEST相対腕拘束導入 | `32f5484760caa3bb95f0149e3b069b61e231bd05` | WBMS active時の上半身EEをCHEST相対拘束にする |
| walking preparation API gate | `fc34ca1faa208ddc4d92eafc7368f18ac5adedf7` | 未READY歩行APIをpendingなしでrejectする |
| 速度・加速度limit型pre-walk | `0bccebfad9c7e30ebe30694bf79c34c82887fea4` | CHEST/COM復帰とCOM高さ保持を導入する |
| READY前footstep抑止 | `af86022985aac24e886ad87569aee7c322f58ff4` | READY前のfootstep時系列を静止させる |
| root returnとfinal IK安全監視 | `61353146985251712a02aa33afdc09c701809106` | 急復帰と腕振動の安全対策を追加する |
| READY/handoff最終修正 | `575700381b2544f09aaaabe5d962ac82f3140c34` | stability条件とoperation blend残留対策を追加する |
| M4.2.2最終log記録 | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | 3本のhistorical PASS相当結果を記録する |
| 最初の旧M5 commit | `7df08e7f275127102f20e9d3f3fea37b6fec655f` | 旧M5計算量削減計画を追加する |
| 最初の旧M5 source commit | `e25a5f5e1e2f5508003216aa22fde10cf4779343` | one-iteration policyを文書化する |
| 最初の旧M5挙動変更 | `8b86463b5b1b8199d6cb751a92f70a8cae75c930` | `checkFinalState=false`を適用する |

`7df08e7f275127102f20e9d3f3fea37b6fec655f^`はexactに`c06b63c8e12dbf85bda4c8391a37544c2469731c`である。CHEST相対腕拘束とM4.2.2の必要commitは全て`c06b63c`のancestorであり、旧M5はその子commitから始まる。この直列境界により、必要な安全修正だけを再合成するsynthetic baselineは不要である。

`c06b63c..6dfa7c9`ではcontrol sourceに差分がなく、planning asset range `5c21cc0..6e530ed`にもcontrol source差分はない。`6dfa7c9`やplanning HEADはruntime baselineではなく、document-only履歴として扱う。

### Final IK差分

根拠source:

- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/FullbodyIKSolver.h`
- `auto_stabilizer/rtc/AutoStabilizer/WbmsPostureControl.cpp`
- `auto_stabilizer/rtc/AutoStabilizer/GaitParam.h`

| 項目 | `1.0` `641f4c8f88a2da8413a8e067c841260b0584a601` | baseline `c06b63c8e12dbf85bda4c8391a37544c2469731c` | `wbms-dev` `5c21cc0cb3c6ef6c906642836ddadf279c8266fd` |
|---|---|---|---|
| priority構造 | 3層 | 足、通常task、reference angleを分離した5層 | 5層を維持 |
| 上半身EE | world target | WBMS active時はCHEST相対 | CHEST相対を維持 |
| WBMS CHEST/root | CHEST taskなし、root `PositionConstraint` | 並進weightを0にした`PositionConstraint`で姿勢を扱い、walking preparation root targetを保持 | `OrientationConstraint`へ変更 |
| iteration | `maxIteration=1` | `maxIteration=1` | `maxIteration=1` |
| final-state再評価 | 既存solver挙動 | 既存solver挙動、明示的skipなし | `checkFinalState=false`で省略 |
| QP workspace | なし | なし | persistent QP workspaceあり |
| profiling/reference診断 | なし | walking preparationに必要なfinal IK診断のみ | 旧M5 profiling、workspace counter、reference-angle統計を追加 |
| baseline判定 | WBMS walking preparation前 | 採用 | 旧M5差分を含むため不採用 |

### Planning asset履歴

既存planning assetの移植範囲はcommit件数ではなく、次のexact rangeで固定する。

```text
range boundary parent:
  5c21cc0cb3c6ef6c906642836ddadf279c8266fd

first planning asset commit:
  8261114e9fe7bcd61fde14e12a6f5eda50b767d4

last existing planning asset commit:
  6e530edacecf663f2eedbdcfd5a787468dad70ce

exact revision range:
  5c21cc0cb3c6ef6c906642836ddadf279c8266fd..6e530edacecf663f2eedbdcfd5a787468dad70ce

equivalent inclusive cherry-pick range:
  8261114e9fe7bcd61fde14e12a6f5eda50b767d4^..6e530edacecf663f2eedbdcfd5a787468dad70ce
```

M0-A-R1 document correction commitとM0-A5 central Progress sync commitはこのrangeへ含めない。作成後の個別SHAを記録し、M0-B2で既存rangeの後に一つずつ順番に移植する。`6e530ed..latest`のような可変rangeやcommit件数に依存する手順は使用しない。

## Required behavior

### M0-B責務と順序

| unit | sole responsibility | WRITE boundary |
|---|---|---|
| `M0-B1` | current dirty workspaceを使用しない隔離方式、exact path、source-root外worktree配置、dedicated catkin workspace、underlay条件を人間承認付きで確定する | source/repository WRITEなし |
| `M0-B2` | `c06b63c`起点のisolated `auto_stabilizer2` implementation branchを作り、承認済みplanning assetをexact rangeと個別SHAで移植する | `auto_stabilizer2`だけ |
| `M0-B3` | `whole_body_teleop` repository/package bootstrapとroot/nearest `AGENTS.md`配置を行う。Project Contextは作成・変更せずM0-B6へdeferする | `whole_body_teleop`だけ |
| `M0-B4` | isolated `rtmros_msg_bridge` branchとroot/nearest `AGENTS.md`をbootstrapする。Project Contextは作成・変更せずM0-B6へdeferする | `rtmros_msg_bridge`だけ |
| `M0-B5` | `${CATKIN_SOURCE_ROOT}/AGENTS.md`配置、4 common Skillのuser-level symlink installation、各repository rootからのinstruction/Skill認識確認を所有する | Git repository WRITEなし。承認済みworkspace-level targetだけ |
| `M0-B6` | 全repository Project Contextの配置または正式defer状態と、workspace manifestの配置・exact SHA固定を単独所有する | repository別sub-unitへ分割し、各sub-unitは最大一つのWRITE repository。source-root manifest sub-unitはGit repository WRITEなし |
| `M0-B7` | exact compatible setのmaterializationと全package-specific baseline buildを単独所有し、exact command、execution directory、resultを記録する | source/repository WRITEなし。failure修正は別repository sub-unit |
| `M0-B8` | source rootからcentral Progress、instructions、Skill、Project Context、manifest、M0-B7 build resultをcross-repository fresh reviewする | 全repository READのみ |

M0-B1より先にM0-B2のbranch作成、worktree作成、dedicated workspace作成、buildを行わない。M0-A3、M0-A-R1、M0-A5はbuildを実行しない。全package buildの唯一のownerはM0-B7である。

M0-B6を一つのmulti-repository implementation taskとして実行しない。Project Contextを実際に配置する場合は対象repositoryごとのsub-unitへ分割し、各sub-unitのWRITE repositoryを一つだけにする。`ik_solvers2`と`prioritized_qp`を変更しない場合はbootstrapだけを目的とするcommitを要求せず、M0-B6がworkspace manifestへ明示的なdefer状態と理由を記録する。

M0-B8がreviewするassetのproducerを次で固定する。

| M0-B8 review asset | sole producer | consumer condition |
|---|---|---|
| M0-A approved baselineとM0-A-R1 SHA | `M0-A5` | append-only central Progress sync済み |
| isolated auto_stabilizer branch/planning asset | `M0-B2` | exact rangeとM0-A-R1/M0-A5個別SHAを順番に移植済み |
| `whole_body_teleop` root/nearest instruction | `M0-B3` | Project Contextは未所有 |
| `rtmros_msg_bridge` root/nearest instruction | `M0-B4` | Project Contextは未所有 |
| source-root `AGENTS.md` | `M0-B5` | template/targetと認識結果を記録済み |
| 4 common Skill installation/recognition | `M0-B5` | symlink先と各rootの認識結果を記録済み |
| 全Project Contextおよび明示defer状態 | `M0-B6` | repository別one-write sub-unitとcompatible SHAを記録済み |
| workspace manifest | `M0-B6` | `M0-pre-M5-baseline`のexact 4 SHAとrepository statusが一致 |
| package build result | `M0-B7` | 全exact command、execution directory、resultを記録済み |
| cross-repository fresh review | `M0-B8` | 上記producer完了後、全repository READのみ |

### Isolation

M0-B1では少なくとも次を人間承認付きで固定する。

- current `/home/kirohy/catkin_ws/cnoid2/src`のdirty checkoutを変更せず、baseline sourceやbuild sourceに使用しない。
- worktreeはcurrent source root外のdedicated catkin workspace、候補`/home/kirohy/catkin_ws/cnoid2_m0_baseline/src`へ配置する。
- current source root内へ同一packageを持つ別worktreeを作らない。
- exact compatible SHAごとにrepository pathを一意化し、同名package重複を作らない。
- current workspaceの`build`、`devel`、`install`、cache、compile database、logをbaseline evidenceへ混入させない。
- underlay、ROS/OpenRTM環境、依存解決方法を明記する。
- 候補pathが使用済み、dirty、または依存解決不能なら暗黙に別pathへ変更せずplanへ戻る。

### Central Progress gate

M0-A-R1のfresh reviewでP0/P1/P2 findingなしとなり、人間がbaseline、exact diff、commitを承認してdocument-only commitを作成した後、M0-A5を独立sub-unitとして行う。

M0-A5は中央Progressへ次をappend-onlyで同期する。

- approved `M0-pre-M5-baseline`の4 SHA。
- M0-A-R1のexact commit SHA。
- M0-A3廃止とM0-B7 build ownership。
- build、simulation、実機が`UNVERIFIED`であること。
- M0-B1の人間承認gateとnext entry point。

M0-A5のreview、承認、commitが完了する前にM0-Aを完了扱いせず、M0-B1を開始しない。

## Safety invariants

- `c06b63c`に含まれるCHEST相対腕拘束を維持する。
- WBMS中未READYの歩行APIをpending保存せずrejectする。
- READY成立だけで自動歩行開始しない。
- READY前のfootstep時系列を静止させる。
- CHEST、COM XY、rootの速度・加速度limit型returnを維持する。
- walking preparation開始時のCOM高さと`refdz`、`l.z`、`omega`、`genCog.z`の整合を維持する。
- `WALKING_HOLD`で`wbmsOperationModeValue=0.0`を維持する。
- timeout、nonfinite、unsafeで歩行を強行しない。
- 腕EE commandと`wbmsMode`をwalking preparation開始時にclearしない。
- 旧M5の`checkFinalState=false`、orientation-only constraint、persistent QP workspace、profiling/reference-angle診断をbaselineへ混入させない。
- current dirty repositoryをcheckout、reset、stash、clean、削除、上書きしない。
- document correctionによって500 Hz経路、指令値、mode遷移、runtime safety behaviorを変更しない。
- historical PASS相当ログを今回のfresh runtime verificationとして扱わない。
- 未実行のbuild、simulation、実機確認は`UNVERIFIED`を維持する。

## Interface/schema impact

Runtime interface/schema impactはない。

変更しないもの:

- ROS message。
- RTM IDL。
- service API。
- port、enum、task mask、parameter。
- frame、単位、quaternion順序。
- C++ ABI/API。
- control behavior。

Planning-only schemaとして、workspace manifestへ`M0-pre-M5-baseline`を追加する。baseline ref、baseline SHA、observed development branchを区別し、current `teleop-dev`をbaseline期待値として扱わない。`rtmros_msg_bridge`はcompatible setに含めるが、M0 baseline build非依存と明記する。

## Compatible input set

| repository | ref | SHA | status |
|---|---|---|---|
| `auto_stabilizer2` | historical `wbms-dev` ancestor | `c06b63c8e12dbf85bda4c8391a37544c2469731c` | SELECTED |
| `ik_solvers2` | `2.0` | `b5de6cd99a6bf89ddb9baadd2a77b63a52319add` | SELECTED |
| `prioritized_qp` | `master` | `624bc1e3e26d4a16f7765baf64fc5941865f2d64` | SELECTED |
| `rtmros_msg_bridge` | `jaxon-minimal` | `10e6fd0cd24fe4649cb7f1b6ef7bc5dc2aa83214` | SELECTED、M0 baseline build非依存 |
| `whole_body_teleop` | 未作成 | — | NOT APPLICABLE |

このcompatible setを変更する場合はM0-A planningへ戻り、新しいContractと人間承認を必要とする。

## Expected repository output

- M0-A-R1 target repository: `auto_stabilizer2`
- M0-A-R1 output type: document/template-only atomic change
- expected M0-A-R1 commit subject: `Correct M0 baseline planning assets`
- M0-A5 target repository: `auto_stabilizer2`
- M0-A5 output type: central Progress append-only document change
- expected M0-A5 commit subject: `Record approved M0 baseline compatible set`
- M0-A-R1 commitとM0-A5 commitは別commitとし、どちらも既存planning asset rangeへ含めない。

## Implementation steps

1. M0-A-R1で本Contract、Revision 2、MultiRepositoryOperations、workspace manifest templateをexact scope内で修正する。
2. `/home/kirohy/catkin_ws/cnoid2/src`からM0-A-R1最新diff全体をfresh read-only reviewする。
3. P0/P1/P2 findingを解消し、人間がbaseline、diff、commit subjectを承認する。
4. 明示許可後にM0-A-R1 document-only commitを作成し、exact SHAを取得する。
5. M0-A5でapproved baseline、M0-A-R1 SHA、`UNVERIFIED`状態を中央Progressへappend-only同期する。
6. M0-A5をreview、人間承認、commitし、M0-Aを完了させる。
7. M0-B1でcurrent dirty workspaceを使わない隔離方式とdedicated workspaceを承認する。
8. M0-B2で`c06b63c`起点のimplementation branchを隔離作成し、既存planning range、M0-A-R1 SHA、M0-A5 SHAを順番に移植する。
9. M0-B3/M0-B4でrepository/branchとroot/nearest instructionをbootstrapし、M0-B5でsource-root instruction、common Skill installation、recognitionを確定し、M0-B6のrepository別sub-unitでProject Contextまたは正式defer状態とmanifestを確定する。
10. M0-B7だけがexact compatible setをmaterializeし、全package-specific baseline buildを実行する。
11. M0-B8がcentral Progress、instructions、Skill、Project Context/defer状態、manifest、build resultをsource rootからfresh read-only reviewする。

## Acceptance criteria

- `7df08e7f275127102f20e9d3f3fea37b6fec655f^`がexactに`c06b63c8e12dbf85bda4c8391a37544c2469731c`である。
- CHEST相対腕拘束と全M4.2.2安全修正が`c06b63c`に含まれる。
- synthetic baseline不要の根拠が履歴とsourceで説明される。
- `1.0`、`c06b63c`、`wbms-dev`のfinal IK差分表がある。
- baseline sourceに`checkFinalState=false`、orientation-only constraint、persistent QP workspace、旧M5 profiling/reference-angle診断がない。
- exact M0 compatible setが4 SHAと一致する。
- M0-A3がbuildを実行せず、全package buildのownerがM0-B7だけである。
- M0-A5完了前にM0-B1へ進めない。
- M0-B1がM0-B2より前であり、M0-B1〜M0-B8の順序と責務が正式文書間で一致する。
- source-root `AGENTS.md`のsole ownerがM0-B5である。
- M0-B3/M0-B4がProject Contextを作成・変更せず、M0-B6へdeferする。
- 全Project Contextの配置または正式defer判断とworkspace manifestのsole ownerがM0-B6である。
- M0-B6の実書込みがrepository別sub-unitへ分かれ、各sub-unitが最大一つのWRITE repositoryを持つ。
- M0-B8の全review assetに一意のproducerがある。
- planning asset移植がexact SHA rangeで表現され、commit件数に依存しない。
- M0-A-R1とM0-A5のfuture commitが既存rangeと区別される。
- manifestがcurrent `teleop-dev`をbaseline期待値として扱わない。
- `rtmros_msg_bridge`がcompatible setに含まれ、M0 baseline build非依存と明記される。
- M0-B1でcurrent dirty checkoutを使わない隔離方式が承認されるまで、branch、worktree、dedicated workspaceを作成しない。
- M0-A-R1のscope外fileとcontrol sourceに差分がない。
- current dirty repositoriesを変更していない。
- build、simulation、実機が`UNVERIFIED`である。
- M0-A-R1のfresh reviewが`CATKIN_SOURCE_ROOT`から行われ、P0/P1/P2 findingがない。
- commit前に人間の明示承認がある。

## Package verification

M0-A3はexecutable sub-unitとして廃止する。M0-A-R1とM0-A5でもbuildしない。次の全package buildはM0-B7だけが所有する。

Execution directory候補:

```text
/home/kirohy/catkin_ws/cnoid2_m0_baseline
```

| package | command | execution directory | dependency scope |
|---|---|---|---|
| `prioritized_qp_base` | `catkin build prioritized_qp_base --no-deps` | `/home/kirohy/catkin_ws/cnoid2_m0_baseline` | package only |
| `prioritized_qp_osqp` | `catkin build prioritized_qp_osqp --no-deps` | 同上 | package only |
| `ik_constraint2` | `catkin build ik_constraint2 --no-deps` | 同上 | package only |
| `ik_constraint2_joint_limit_table` | `catkin build ik_constraint2_joint_limit_table --no-deps` | 同上 | package only |
| `prioritized_inverse_kinematics_solver2` | `catkin build prioritized_inverse_kinematics_solver2 --no-deps` | 同上 | package only |
| `auto_stabilizer` | `catkin build auto_stabilizer --no-deps --force-cmake` | 同上 | baseline初回のIDL/CMake生成 |
| `auto_stabilizer` integration | `catkin build auto_stabilizer --force-cmake` | 同上 | selected IK/QPを含むtransitive dependency確認 |
| `rtmros_msg_bridge` | 実行しない | — | M0 baseline build非依存 |

引数なしのworkspace一括`catkin build`は使用しない。

M0-B1はbuild前に、dedicated workspaceのexact path、source-root外worktree、underlay、同名package重複回避、current generated artifact非混入を承認済みにする。

## Cross-repository acceptance

- named compatible setの全SHAがM0-A、Revision 2、MultiRepositoryOperations、workspace manifest、M0-A5 Progressで一致する。
- current `ik_solvers2/teleop-dev` HEADをbaselineへ使用しない。
- current `prioritized_qp/teleop-dev` HEADをbaselineへ使用しない。
- `rtmros_msg_bridge`はexact SHAを記録するがM0 baseline buildへ含めない。
- `whole_body_teleop`をM0 baseline依存として発明しない。
- M0-B2のplanning asset移植後もcontrol treeが`c06b63c`相当である。
- dependency build failureは原因repositoryの別fix sub-unitへ割り当てる。
- current dirty checkoutを変更してbuild failureを修正しない。
- M0-B8はM0-B7の全package result、manifest、instructions、Skill recognitionをread-onlyでreviewする。
- M0-B3/M0-B4のContractはProject ContextをWRITE scopeへ含めない。
- M0-B6のrepository-specific ContractだけがProject ContextをWRITEできる。
- M0-B8はcentral Progress、Project Contextまたは明示defer状態もread-onlyでreviewする。

## Review focus

- `c06b63c`ではなく`6dfa7c9`、`5c21cc0`、planning HEADをruntime baselineにしていないか。
- `b5de6cd`と`624bc1e`ではなくcurrent `teleop-dev`をbaselineとして期待していないか。
- `checkFinalState=false`、orientation-only constraint、QP workspace、profiling/reference-angle診断をbaselineへ混入させていないか。
- CHEST相対腕拘束`32f5484`とwalking preparation最終安全修正`5757003`を保持しているか。
- READY前API gate、footstep静止、COM高さ保持、operation blend、failure pathが正しく説明されているか。
- M0-A3、M0-A5、M0-B7に責務重複がないか。
- source-root `AGENTS.md`がowner不在または複数ownerになっていないか。
- M0-B3/M0-B4にProject Contextの作成・固定SHA更新が残っていないか。
- M0-B6が一つのmulti-repository implementation taskとして記述されていないか。
- M0-B8の全review対象に一意のproducerがあるか。
- M0-B1より先にbranch、worktree、dedicated workspace、buildを開始する記述がないか。
- planning asset rangeとfuture document commitが分離されているか。
- workspace manifestがbaseline refとobserved development branchを混同していないか。
- scope外のSkill、AGENTS、control source、中央ProgressがM0-A-R1 diffへ混入していないか。
- `UNVERIFIED`項目をPASS扱いしていないか。

## Known unverified items

- exact compatible setのpackage build: `UNVERIFIED`
- dedicated baseline catkin workspaceの構築: `UNVERIFIED`
- planning assetのbaseline branchへの移植: `UNVERIFIED`
- simulator再実行: `UNVERIFIED`
- 実機実行: `UNVERIFIED`
- historical `061848`、`061851`、`061853`ログの今回の再解析・再実行: `UNVERIFIED`
- source-root `AGENTS.md`配置: `UNVERIFIED`
- common Skill user-level symlink: `UNVERIFIED`
- Project Context配置または正式defer状態: `UNVERIFIED`
- M0-A-R1 revised diffのfresh review: implementation後まで`UNVERIFIED`
- M0-A5 central Progress sync: M0-A-R1 commit後まで`PENDING`

Historical walking preparationログはbaseline選定根拠として参照できるが、今回のbuild、simulation、実機verificationを代替しない。

## Human approval gates

1. M0-A-R1 fresh reviewでP0/P1/P2なしとなるまでcommit許可を求めない。
2. exact M0-A-R1 diff、repository、subjectに対する明示許可なしにcommitしない。
3. M0-A5 central Progress syncとそのcommitは別の明示許可を必要とする。
4. M0-A5完了前にM0-B1へ進まない。
5. M0-B1で隔離workspace path、source-root外worktree方式、underlay、package重複回避を人間が承認する。
6. M0-B1承認前にworktree、branch、dedicated workspaceを作成しない。
7. build、simulation、実機実行はそれぞれ別の明示許可を必要とする。

## Open decisions blocking implementation

M0-A-R1 document correctionを左右する技術的未決定事項はない。

次はfuture gateであり、M0-A-R1 implementationの設計未決定事項ではない。

- M0-B1で専用workspace候補`/home/kirohy/catkin_ws/cnoid2_m0_baseline`を正式承認すること。
- M0-A-R1およびM0-A5のexact commit SHAはcommit後に取得すること。
- simulationおよび実機実行の許可は別途判断すること。
