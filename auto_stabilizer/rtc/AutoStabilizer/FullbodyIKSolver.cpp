#include "FullbodyIKSolver.h"
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace {
class ReferenceAngleErrorStats {
public:
  double count = 0.0;
  double maxError = 0.0;
  double squaredErrorSum = 0.0;

  void add(double error){
    if(!std::isfinite(error)) return;
    double absError = std::abs(error);
    this->count += 1.0;
    this->maxError = std::max(this->maxError, absError);
    this->squaredErrorSum += absError * absError;
  }

  double rms() const{
    return this->count > 0.0 ? std::sqrt(this->squaredErrorSum / this->count) : 0.0;
  }
};

bool isArmJointName(const std::string& name){
  return name.find("ARM") != std::string::npos ||
    name.find("Arm") != std::string::npos ||
    name.find("arm") != std::string::npos;
}
}

bool FullbodyIKSolver::solveFullbodyIK(double dt, GaitParam& gaitParam,
                                       cnoid::BodyPtr& genRobot) const{
  std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
  gaitParam.debugData.wbmsFinalIKQpSignatureHitDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpSignatureMissDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpInitializeDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpUpdateFailureDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpSolveFailureDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpStructureRebuildDelta = 0.0;
  gaitParam.debugData.wbmsFinalIKQpFastPathFallbackDelta = 0.0;
  gaitParam.debugData.resetWbmsFinalIKProfiling();
  double wbmsMode = gaitParam.wbmsMode.value();
  double wbmsStabilityMode = std::max(1.0 - wbmsMode, gaitParam.wbmsWalkingStabilityModeValue);
  double wbmsOperationMode = std::min(1.0, std::max(0.0, gaitParam.wbmsOperationModeValue));
  bool wbmsActive = (gaitParam.wbmsMode.value() > 0.0 || gaitParam.wbmsMode.getGoal() > 0.0);
  std::vector<double> refq;
  refq.resize(genRobot->numJoints());
  std::vector<double> referenceAngleTargetq(genRobot->numJoints(), 0.0);
  std::vector<bool> referenceAngleTargetValid(genRobot->numJoints(), false);
  std::vector<bool> referenceAngleProjectionMask(genRobot->numJoints(), false);
  std::vector<bool> referenceAngleArmMask(genRobot->numJoints(), false);
  for(int i=0;i<genRobot->numJoints();i++){
    refq[i] = gaitParam.refRobot->joint(i)->q();
    if(gaitParam.wbmsPostureReferenceValid &&
       i < gaitParam.wbmsPostureReferenceQ.size() &&
       i < gaitParam.wbmsPostureReferenceJointMask.size() &&
       gaitParam.wbmsPostureReferenceJointMask[i]){
      refq[i] = gaitParam.refRobot->joint(i)->q() * (1.0 - wbmsOperationMode) + gaitParam.wbmsPostureReferenceQ[i] * wbmsOperationMode;
    }
  }

  // !jointControllableの関節は指令値をそのまま入れる
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) genRobot->joint(i)->q() = gaitParam.refRobot->joint(i)->q();
  }

  // jointControllableの関節のみ、探索変数にする
  std::vector<cnoid::LinkPtr> variables; variables.reserve(1+genRobot->numJoints());
  std::vector<double> dqWeight; dqWeight.reserve(6+genRobot->numJoints());
  variables.push_back(genRobot->rootLink());
  for(int i=0;i<6;i++) dqWeight.push_back(1.0);
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(gaitParam.jointControllable[i]) {
      variables.push_back(genRobot->joint(i));
      dqWeight.push_back(this->dqWeight[i].value());
    }
  }

  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikConstraint0;

  // joint velocity
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    this->jointVelocityConstraint[i]->joint() = genRobot->joint(i);
    this->jointVelocityConstraint[i]->dt() = dt;
    this->jointVelocityConstraint[i]->maxError() = 1.0 * dt;
    this->jointVelocityConstraint[i]->weight() = 1.0;
    ikConstraint0.push_back(this->jointVelocityConstraint[i]);
  }

  // joint angle
  for(size_t i=0;i<genRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    this->jointLimitConstraint[i]->joint() = genRobot->joint(i);
    this->jointLimitConstraint[i]->jointLimitTables() = gaitParam.jointLimitTables[i];
    this->jointLimitConstraint[i]->maxError() = 1.0 * dt;
    this->jointLimitConstraint[i]->weight() = 1.0;
    ikConstraint0.push_back(this->jointLimitConstraint[i]);
  }

  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikConstraint1;
  this->selfCollisionConstraint.resize(gaitParam.selfCollision.size());
  for(size_t i=0;i<this->selfCollisionConstraint.size();i++){
    if(!this->selfCollisionConstraint[i]) this->selfCollisionConstraint[i] = std::make_shared<ik_constraint2::ClientCollisionConstraint>();
    this->selfCollisionConstraint[i]->A_link() = genRobot->link(gaitParam.selfCollision[i].link1);
    this->selfCollisionConstraint[i]->B_link() = genRobot->link(gaitParam.selfCollision[i].link2);
    this->selfCollisionConstraint[i]->tolerance() = 0.01;
    this->selfCollisionConstraint[i]->maxError() = 10.0*dt;
    this->selfCollisionConstraint[i]->weight() = 1.0;
    this->selfCollisionConstraint[i]->velocityDamper() = 0.1 / dt;
    this->selfCollisionConstraint[i]->A_localp() = gaitParam.selfCollision[i].point1;
    this->selfCollisionConstraint[i]->B_localp() = gaitParam.selfCollision[i].point2;
    this->selfCollisionConstraint[i]->direction() = gaitParam.selfCollision[i].direction21;

    // 全自己干渉情報を与えると計算コストが膨大になるため、距離が近いもののみ与える
    if(gaitParam.selfCollision[i].distance < 0.05){
      ikConstraint1.push_back(this->selfCollisionConstraint[i]);
    }
  }

  // 優先度順: 0 joint安全系, 1 自己干渉, 2 足, 3 通常タスク, 4 reference angle.
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikConstraint2;
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikConstraint3;
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > ikConstraint4;

  // EEF
  // 足
  for(int i=0;i<NUM_LEGS;i++){
    this->ikEEPositionConstraint[i]->A_link() = genRobot->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->ikEEPositionConstraint[i]->B_link() = nullptr;
    this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.abcEETargetPose[i];
    this->ikEEPositionConstraint[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->ikEEPositionConstraint[i]->precision() = 0.0;
    this->ikEEPositionConstraint[i]->weight() = this->ikEEPositionWeight[i].value();
    this->ikEEPositionConstraint[i]->eval_link() = genRobot->link(this->ikEEEvalLink[i]);
    if(this->ikEEPositionConstraint[i]->eval_link()) this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->eval_link()->R().transpose() * this->ikEEPositionConstraint[i]->B_localpos().linear();
    else this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->B_localpos().linear();
    ikConstraint2.push_back(this->ikEEPositionConstraint[i]);
  }

  // 上半身
  for(int i=NUM_LEGS;i<gaitParam.eeName.size();i++){
    this->ikEEPositionConstraint[i]->A_link() = genRobot->link(gaitParam.eeParentLink[i]);
    this->ikEEPositionConstraint[i]->A_localpos() = gaitParam.eeLocalT[i];
    cnoid::LinkPtr torsoGenLink = genRobot->link(gaitParam.chestLinkName);
    cnoid::LinkPtr torsoRefLink = gaitParam.refRobot->link(gaitParam.chestLinkName);
    if(wbmsActive && torsoGenLink && torsoRefLink){
      this->ikEEPositionConstraint[i]->B_link() = torsoGenLink;
      this->ikEEPositionConstraint[i]->B_localpos() = torsoRefLink->T().inverse() * gaitParam.abcEETargetPose[i];
    }else{
      this->ikEEPositionConstraint[i]->B_link() = genRobot->rootLink();
      this->ikEEPositionConstraint[i]->B_localpos() = gaitParam.refRobot->rootLink()->T().inverse() * gaitParam.abcEETargetPose[i];
    }
    this->ikEEPositionConstraint[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->ikEEPositionConstraint[i]->precision() = 0.0;
    this->ikEEPositionConstraint[i]->weight() = this->ikEEPositionWeight[i].value();
    this->ikEEPositionConstraint[i]->eval_link() = genRobot->link(this->ikEEEvalLink[i]);
    cnoid::Matrix3 eeTargetR;
    if(this->ikEEPositionConstraint[i]->B_link()) eeTargetR = this->ikEEPositionConstraint[i]->B_link()->R() * this->ikEEPositionConstraint[i]->B_localpos().linear();
    else eeTargetR = this->ikEEPositionConstraint[i]->B_localpos().linear();
    if(this->ikEEPositionConstraint[i]->eval_link()) this->ikEEPositionConstraint[i]->eval_localR() = this->ikEEPositionConstraint[i]->eval_link()->R().transpose() * eeTargetR;
    else this->ikEEPositionConstraint[i]->eval_localR() = eeTargetR;
    ikConstraint3.push_back(this->ikEEPositionConstraint[i]);
  }

  // COM
  {
    this->comConstraint->A_robot() = genRobot;
    this->comConstraint->A_localp() = cnoid::Vector3::Zero();
    this->comConstraint->B_robot() = nullptr;
    cnoid::Vector3 cogTarget = gaitParam.genCog + gaitParam.sbpOffset;
    this->comConstraint->B_localp() = cogTarget;
    this->comConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    this->comConstraint->precision() = 0.0;
    cnoid::Vector3 nominalComWeight(10.0, 10.0, 1.0 * wbmsStabilityMode);
    this->comConstraint->weight() = nominalComWeight * (1.0 - wbmsOperationMode) + gaitParam.wbmsComPositionWeight * wbmsOperationMode;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint3.push_back(this->comConstraint);
  }

  // WBMS CHEST姿勢。投影済み姿勢だけを使い、位置は拘束しない。
  {
    cnoid::LinkPtr chestLink = genRobot->link(gaitParam.chestLinkName);
    if(chestLink && gaitParam.wbmsPostureReferenceValid){
      this->chestOrientationConstraint->A_link() = chestLink;
      this->chestOrientationConstraint->A_localR() = cnoid::Matrix3::Identity();
      this->chestOrientationConstraint->B_link() = nullptr;
      this->chestOrientationConstraint->B_localR() = gaitParam.wbmsProjectedChestR;
      this->chestOrientationConstraint->maxError() <<
        gaitParam.wbmsTorsoOrientationMaxError[0] * dt,
        gaitParam.wbmsTorsoOrientationMaxError[1] * dt,
        gaitParam.wbmsTorsoOrientationMaxError[2] * dt;
      this->chestOrientationConstraint->precision() = 0.0;
      this->chestOrientationConstraint->weight() <<
        gaitParam.wbmsTorsoOrientationWeight[0] * wbmsOperationMode,
        gaitParam.wbmsTorsoOrientationWeight[1] * wbmsOperationMode,
        gaitParam.wbmsTorsoOrientationWeight[2] * wbmsOperationMode;
      this->chestOrientationConstraint->eval_link() = chestLink;
      this->chestOrientationConstraint->eval_localR() = cnoid::Matrix3::Identity();
      ikConstraint3.push_back(this->chestOrientationConstraint);
    }
  }

  // Angular Momentum
  {
    this->angularMomentumConstraint->robot() = genRobot;
    this->angularMomentumConstraint->targetAngularMomentum() = cnoid::Vector3::Zero(); // TODO
    this->angularMomentumConstraint->maxError() << 1.0*dt, 1.0*dt, 1.0*dt;
    this->angularMomentumConstraint->precision() = 0.0;
    this->angularMomentumConstraint->weight() << 1e-4, 1e-4, 0.0; // TODO
    this->angularMomentumConstraint->dt() = dt;
    this->comConstraint->eval_R() = cnoid::Matrix3::Identity();
    ikConstraint3.push_back(this->angularMomentumConstraint);
  }

  // root
  {
    cnoid::Isometry3 rootTargetPose = gaitParam.stTargetRootPose;
    if(gaitParam.wbmsWalkingPreparationSnapshotValid &&
       (gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_RETURNING ||
        gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_HANDOFF ||
        gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_READY) &&
       gaitParam.wbmsWalkingPreparationTargetRootR.allFinite()){
      rootTargetPose.linear() = gaitParam.wbmsWalkingPreparationTargetRootR;
    }
    this->rootOrientationConstraint->A_link() = genRobot->rootLink();
    this->rootOrientationConstraint->A_localR() = cnoid::Matrix3::Identity();
    this->rootOrientationConstraint->B_link() = nullptr;
    this->rootOrientationConstraint->B_localR() = rootTargetPose.linear();
    this->rootOrientationConstraint->maxError() << 10.0*dt, 10.0*dt, 10.0*dt;
    this->rootOrientationConstraint->precision() = 0.0;
    // 角運動量を利用するときは重みを小さく. 通常時、胴の質量・イナーシャやマスパラ誤差の大きさや、胴を大きく動かすための出力不足などによって、二足動歩行では胴の傾きの自由度を使わない方がよい
    this->rootOrientationConstraint->weight() << 3.0*wbmsStabilityMode, 3.0*wbmsStabilityMode, 3.0*wbmsStabilityMode;
    this->rootOrientationConstraint->eval_link() = genRobot->rootLink();
    this->rootOrientationConstraint->eval_localR() = cnoid::Matrix3::Identity();
    ikConstraint3.push_back(this->rootOrientationConstraint);
  }

  // reference angle
  {
    ReferenceAngleErrorStats preAllStats;
    ReferenceAngleErrorStats preProjectionMaskStats;
    ReferenceAngleErrorStats preNonProjectionMaskStats;
    ReferenceAngleErrorStats preArmStats;
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!gaitParam.jointControllable[i]) continue;
      this->refJointAngleConstraint[i]->joint() = genRobot->joint(i);
      this->refJointAngleConstraint[i]->maxError() = 10.0 * dt; // 高優先度のmaxError以下にしないと優先度逆転するおそれ
      // this->refJointAngleConstraint[i]->weight() = 1e-1; // 小さい値すぎると、qp終了判定のtoleranceによって無視されてしまう
      this->refJointAngleConstraint[i]->weight() = 1e-1;
      double u = gaitParam.refRobot->joint(i)->q_upper();
      double l = gaitParam.refRobot->joint(i)->q_lower();
      for(int j=0;j<gaitParam.jointLimitTables[i].size();j++){
        u = std::min(u,gaitParam.jointLimitTables[i][j]->getUlimit());
        l = std::max(l,gaitParam.jointLimitTables[i][j]->getLlimit());
      }
      double targetq = std::min(u, std::max(l, refq[i]));
      if(std::abs(targetq - refq[i]) > 1e-12) gaitParam.debugData.wbmsFinalIKReferenceAngleClampCount += 1.0;
      this->refJointAngleConstraint[i]->targetq() = targetq;
      this->refJointAngleConstraint[i]->precision() = 0.0;
      ikConstraint4.push_back(this->refJointAngleConstraint[i]);
      referenceAngleTargetq[i] = targetq;
      referenceAngleTargetValid[i] = true;
      bool projectionMask =
        gaitParam.wbmsPostureReferenceValid &&
        i < gaitParam.wbmsPostureReferenceJointMask.size() &&
        gaitParam.wbmsPostureReferenceJointMask[i];
      bool armMask = genRobot->joint(i) && isArmJointName(genRobot->joint(i)->name());
      referenceAngleProjectionMask[i] = projectionMask;
      referenceAngleArmMask[i] = armMask;

      gaitParam.debugData.wbmsFinalIKReferenceAngleConstraintCount += 1.0;
      if(projectionMask) gaitParam.debugData.wbmsFinalIKReferenceAngleProjectionMaskCount += 1.0;
      else gaitParam.debugData.wbmsFinalIKReferenceAngleNonProjectionMaskCount += 1.0;
      if(armMask) gaitParam.debugData.wbmsFinalIKReferenceAngleArmCount += 1.0;

      double preError = genRobot->joint(i)->q() - targetq;
      preAllStats.add(preError);
      if(projectionMask) preProjectionMaskStats.add(preError);
      else preNonProjectionMaskStats.add(preError);
      if(armMask) preArmStats.add(preError);
    }
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreErrorMax = preAllStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreErrorRms = preAllStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreProjectionMaskErrorMax = preProjectionMaskStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreProjectionMaskErrorRms = preProjectionMaskStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreNonProjectionMaskErrorMax = preNonProjectionMaskStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreNonProjectionMaskErrorRms = preNonProjectionMaskStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreArmErrorMax = preArmStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePreArmErrorRms = preArmStats.rms();
  }

  // 特異点近傍で振動するようなことは起こりにくいが、歩行動作中の一瞬だけIKがときにくい姿勢があってすぐに解ける姿勢に戻るといった場合に、その一瞬の間だけIKを解くために頑張って姿勢が大きく変化するので、危険.
  //  この現象を防ぐには、未来の情報を含んだIKを作るか、歩行動作中にIKが解きづらい姿勢を経由しないように着地位置等をリミットするか. 後者を採用
  //  歩行動作ではないゆっくりとした動作であれば、この現象が発生しても問題ない

  std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{ikConstraint0,ikConstraint1,ikConstraint2,ikConstraint3,ikConstraint4};
  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debugLevel() = 0;//debuglevel
    }
  }
  prioritized_inverse_kinematics_solver2::IKParam param;
  // WBMS final IKは500Hz運用のため1 iteration固定で使う。
  // checkFinalState=falseでsolve後のconstraint再評価を省く。
  // 現状の制御判断は戻り値に依存せず、solve後の姿勢を後段のlimit checkへ渡す。
  param.maxIteration = 1;
  param.checkFinalState = false;
  param.dqWeight = dqWeight;
  param.wn = 1e-6;
  param.we = 1e2; // 1e0だとやや不安定. 1e3だと大きすぎる
  param.debugLevel = 0;
  param.dt = dt;
  param.qpWorkspace = &this->qpWorkspace;
  prioritized_inverse_kinematics_solver2::IKProfile ikProfile;
  param.profile = &ikProfile;
  const size_t signatureHitCountBefore = this->qpWorkspace.signatureHitCount;
  const size_t signatureMissCountBefore = this->qpWorkspace.signatureMissCount;
  const size_t initializeCountBefore = this->qpWorkspace.initializeCount;
  const size_t updateFailureCountBefore = this->qpWorkspace.updateFailureCount;
  const size_t solveFailureCountBefore = this->qpWorkspace.solveFailureCount;
  const size_t structureRebuildCountBefore = this->qpWorkspace.structureRebuildCount;
  const size_t fastPathFallbackCountBefore = this->qpWorkspace.fastPathFallbackCount;
  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                     constraints,
                                                     this->tasks,
                                                     param
                                                     );
  {
    ReferenceAngleErrorStats postAllStats;
    ReferenceAngleErrorStats postProjectionMaskStats;
    ReferenceAngleErrorStats postNonProjectionMaskStats;
    ReferenceAngleErrorStats postArmStats;
    for(size_t i=0;i<genRobot->numJoints();i++){
      if(!referenceAngleTargetValid[i]) continue;
      double postError = genRobot->joint(i)->q() - referenceAngleTargetq[i];
      postAllStats.add(postError);
      if(referenceAngleProjectionMask[i]) postProjectionMaskStats.add(postError);
      else postNonProjectionMaskStats.add(postError);
      if(referenceAngleArmMask[i]) postArmStats.add(postError);
    }
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostErrorMax = postAllStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostErrorRms = postAllStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostProjectionMaskErrorMax = postProjectionMaskStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostProjectionMaskErrorRms = postProjectionMaskStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostNonProjectionMaskErrorMax = postNonProjectionMaskStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostNonProjectionMaskErrorRms = postNonProjectionMaskStats.rms();
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostArmErrorMax = postArmStats.maxError;
    gaitParam.debugData.wbmsFinalIKReferenceAnglePostArmErrorRms = postArmStats.rms();
  }
  gaitParam.debugData.wbmsFinalIKQpSignatureHitDelta = static_cast<double>(this->qpWorkspace.signatureHitCount - signatureHitCountBefore);
  gaitParam.debugData.wbmsFinalIKQpSignatureMissDelta = static_cast<double>(this->qpWorkspace.signatureMissCount - signatureMissCountBefore);
  gaitParam.debugData.wbmsFinalIKQpInitializeDelta = static_cast<double>(this->qpWorkspace.initializeCount - initializeCountBefore);
  gaitParam.debugData.wbmsFinalIKQpUpdateFailureDelta = static_cast<double>(this->qpWorkspace.updateFailureCount - updateFailureCountBefore);
  gaitParam.debugData.wbmsFinalIKQpSolveFailureDelta = static_cast<double>(this->qpWorkspace.solveFailureCount - solveFailureCountBefore);
  gaitParam.debugData.wbmsFinalIKQpStructureRebuildDelta = static_cast<double>(this->qpWorkspace.structureRebuildCount - structureRebuildCountBefore);
  gaitParam.debugData.wbmsFinalIKQpFastPathFallbackDelta = static_cast<double>(this->qpWorkspace.fastPathFallbackCount - fastPathFallbackCountBefore);
  gaitParam.debugData.wbmsFinalIKProfileValid = 1.0;
  gaitParam.debugData.wbmsFinalIKConstraintUpdateTime = ikProfile.constraintUpdateTime;
  gaitParam.debugData.wbmsFinalIKTaskGenerationTime = ikProfile.taskGenerationTime;
  gaitParam.debugData.wbmsFinalIKQpSolveTime = ikProfile.qpSolveTime;
  gaitParam.debugData.wbmsFinalIKPostForwardKinematicsTime = ikProfile.postForwardKinematicsTime;
  const size_t priorityProfileSize = std::min(this->qpWorkspace.priorityProfiles.size(),
                                              gaitParam.debugData.wbmsFinalIKPriorityPrepareTime.size());
  for(size_t i=0;i<priorityProfileSize;i++){
    const prioritized_qp_base::SolveWorkspace::PriorityProfile& profile = this->qpWorkspace.priorityProfiles[i];
    gaitParam.debugData.wbmsFinalIKPriorityPrepareTime[i] = profile.prepareTime;
    gaitParam.debugData.wbmsFinalIKPrioritySolverUpdateTime[i] = profile.solverUpdateTime;
    gaitParam.debugData.wbmsFinalIKPrioritySolverSolveTime[i] = profile.solverSolveTime;
    gaitParam.debugData.wbmsFinalIKPriorityQpVariables[i] = static_cast<double>(profile.qpVariables);
    gaitParam.debugData.wbmsFinalIKPriorityQpConstraints[i] = static_cast<double>(profile.qpConstraints);
    gaitParam.debugData.wbmsFinalIKPriorityExtVariables[i] = static_cast<double>(profile.extVariables);
    gaitParam.debugData.wbmsFinalIKPriorityToSolve[i] = profile.toSolve ? 1.0 : 0.0;
  }


  // 念の為limit check
  for(int i=0;i<gaitParam.refRobot->numJoints();i++){
    if(!gaitParam.jointControllable[i]) continue;
    cnoid::LinkPtr joint = genRobot->joint(i);
    double u = gaitParam.refRobot->joint(i)->q_upper();
    double l = gaitParam.refRobot->joint(i)->q_lower();
    for(int j=0;j<gaitParam.jointLimitTables[i].size();j++){
      u = std::min(u,gaitParam.jointLimitTables[i][j]->getUlimit());
      l = std::max(l,gaitParam.jointLimitTables[i][j]->getLlimit());
    }
    joint->q() = std::min(u, std::max(l, joint->q()));
  }

  gaitParam.debugData.wbmsFinalIKTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
  return true;
}
