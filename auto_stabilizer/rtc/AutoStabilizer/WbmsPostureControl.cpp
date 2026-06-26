#include "WbmsPostureControl.h"

#include "MathUtil.h"
#include <cnoid/EigenUtil>
#include <algorithm>
#include <chrono>
#include <cmath>

void WbmsPostureControl::init(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam){
  this->wbmsPostureRobot_ = genRobot->clone();
  this->wbmsPostureRobot_->calcForwardKinematics();
  this->wbmsPostureRobot_->calcCenterOfMass();

  std::vector<bool> jointUsed(genRobot->numJoints(), false);
  for(int i=0;i<NUM_LEGS;i++){
    this->addAncestorJointIds(genRobot->link(gaitParam.eeParentLink[i]), genRobot, jointUsed);
  }
  this->addAncestorJointIds(genRobot->link(gaitParam.chestLinkName), genRobot, jointUsed);

  this->projectionJointIds_.clear();
  for(int i=0;i<genRobot->numJoints();i++){
    if(jointUsed[i] && gaitParam.jointControllable[i]) this->projectionJointIds_.push_back(i);
  }

  this->projectionVariables_.clear();
  this->projectionVariables_.reserve(1 + this->projectionJointIds_.size());
  this->projectionVariables_.push_back(this->wbmsPostureRobot_->rootLink());
  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    this->projectionVariables_.push_back(this->wbmsPostureRobot_->joint(this->projectionJointIds_[i]));
  }

  this->projectionIKParam_.maxIteration = 1;
  this->projectionIKParam_.dqWeight.assign(6 + this->projectionJointIds_.size(), 1.0);
  this->projectionIKParam_.wn = 1e-6;
  this->projectionIKParam_.we = 1e2;
  this->projectionIKParam_.debugLevel = 0;

  this->jointVelocityConstraint_.clear();
  this->jointLimitConstraint_.clear();
  this->postureReferenceConstraint_.clear();
  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    this->jointVelocityConstraint_.push_back(std::make_shared<ik_constraint2::JointVelocityConstraint>());
    this->jointLimitConstraint_.push_back(std::make_shared<ik_constraint2_joint_limit_table::JointLimitMinMaxTableConstraint>());
    this->postureReferenceConstraint_.push_back(std::make_shared<ik_constraint2::JointAngleConstraint>());
  }

  this->footConstraint_.clear();
  for(int i=0;i<NUM_LEGS;i++){
    this->footConstraint_.push_back(std::make_shared<ik_constraint2::PositionConstraint>());
  }

  this->selfCollisionConstraint_.clear();
  for(size_t i=0;i<gaitParam.selfCollision.size();i++){
    this->selfCollisionConstraint_.push_back(std::make_shared<ik_constraint2::ClientCollisionConstraint>());
  }

  this->projectionConstraints_.resize(5);
  this->projectionConstraints_[0].reserve(this->projectionJointIds_.size() * 2);
  this->projectionConstraints_[1].reserve(std::max<size_t>(1, gaitParam.selfCollision.size()));
  this->projectionConstraints_[2].reserve(NUM_LEGS);
  this->projectionConstraints_[3].reserve(2);
  this->projectionConstraints_[4].reserve(this->projectionJointIds_.size());
  size_t supportVertexCapacity = 0;
  for(int i=0;i<NUM_LEGS;i++) supportVertexCapacity += gaitParam.legHull[i].size();
  supportVertexCapacity = std::max<size_t>(supportVertexCapacity, NUM_LEGS * 4);
  this->supportVerticesInFootMid_.reserve(supportVertexCapacity);
  this->supportHullTmp_.reserve(supportVertexCapacity);
  this->supportHullInFootMid_.reserve(2 * supportVertexCapacity);
  this->shrunkSupportHullInFootMid_.reserve(2 * supportVertexCapacity);
  this->shrunkSupportHullInWorld_.reserve(2 * supportVertexCapacity);
  this->shrinkShiftedPoints_.reserve(2 * supportVertexCapacity);
  this->shrinkShiftedDirs_.reserve(2 * supportVertexCapacity);
}

void WbmsPostureControl::reset(){
  this->wbmsWalkingStabilityMode_.reset(0.0);
}

void WbmsPostureControl::start(GaitParam& gaitParam){
  gaitParam.clearWbmsPostureCommand(true);
  gaitParam.clearWbmsPostureReference();

  if(gaitParam.genRobot){
    gaitParam.genRobot->calcForwardKinematics();
    gaitParam.genRobot->calcCenterOfMass();
    gaitParam.wbmsPostureReferenceQ.resize(gaitParam.genRobot->numJoints());
    for(int i=0;i<gaitParam.genRobot->numJoints();i++){
      gaitParam.wbmsPostureReferenceQ[i] = gaitParam.genRobot->joint(i)->q();
    }
    cnoid::LinkPtr chestLink = gaitParam.genRobot->link(gaitParam.chestLinkName);
    if(chestLink){
      const cnoid::Isometry3 footMid = gaitParam.footMidCoords.value();
      gaitParam.wbmsStartChestRInFootMid = footMid.linear().transpose() * chestLink->R();
      gaitParam.wbmsStartComInFootMid = footMid.inverse() * gaitParam.genRobot->centerOfMass();
      gaitParam.wbmsPostureBaselineValid = true;
      gaitParam.wbmsProjectedChestR = chestLink->R();
      gaitParam.wbmsProjectedRobotCom = gaitParam.genRobot->centerOfMass();
    }else{
      gaitParam.wbmsPostureBaselineValid = false;
    }

    if(this->wbmsPostureRobot_){
      this->wbmsPostureRobot_->rootLink()->p() = gaitParam.genRobot->rootLink()->p();
      this->wbmsPostureRobot_->rootLink()->R() = gaitParam.genRobot->rootLink()->R();
      for(int i=0;i<gaitParam.genRobot->numJoints();i++){
        this->wbmsPostureRobot_->joint(i)->q() = gaitParam.genRobot->joint(i)->q();
      }
      this->wbmsPostureRobot_->calcForwardKinematics();
      this->wbmsPostureRobot_->calcCenterOfMass();
    }
  }else{
    gaitParam.wbmsPostureBaselineValid = false;
  }
}

void WbmsPostureControl::clearStaleCommand(GaitParam& gaitParam, bool resetApplied) const{
  gaitParam.clearWbmsPostureCommand(resetApplied);
  gaitParam.clearWbmsPostureReference();
}

bool WbmsPostureControl::isOperationAllowed(const GaitParam& gaitParam, bool isABCRunning) const{
  if(!isABCRunning) return false;
  if(gaitParam.wbmsMode.getGoal() <= 0.0) return false;
  if(!gaitParam.isStatic()) return false;
  if(gaitParam.isWbmsWalkingStartDelay) return false;
  if(gaitParam.footstepNodesList.empty()) return false;
  if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] || !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) return false;
  for(int i=0;i<NUM_LEGS;i++){
    if(gaitParam.isManualControlMode[i].value() > 0.0 || gaitParam.isManualControlMode[i].getGoal() > 0.0) return false;
  }
  return true;
}

void WbmsPostureControl::addAncestorJointIds(const cnoid::LinkPtr& link, const cnoid::BodyPtr& robot, std::vector<bool>& jointUsed){
  cnoid::Link* current = link.get();
  while(current && current != robot->rootLink()){
    if(current->jointId() >= 0 && current->jointId() < jointUsed.size()){
      jointUsed[current->jointId()] = true;
    }
    current = current->parent();
  }
}

cnoid::Vector3 WbmsPostureControl::applyAccelerationLimit(const cnoid::Vector3& current, const cnoid::Vector3& desired, const cnoid::Vector3& limit, double dt) const{
  cnoid::Vector3 stepLimit = limit.cwiseMax(cnoid::Vector3::Zero()) * std::max(dt, 0.0);
  cnoid::Vector3 diff = desired - current;
  return current + mathutil::clampMatrix<cnoid::Vector3>(diff, stepLimit);
}

void WbmsPostureControl::updateVelocityCommand(GaitParam& gaitParam, double dt, bool isABCRunning) const{
  gaitParam.wbmsVelocityCommandAge += dt;

  cnoid::Vector3 desiredComVelocity = cnoid::Vector3::Zero();
  cnoid::Vector3 desiredTorsoAngularVelocity = cnoid::Vector3::Zero();
  const bool commandActive =
    gaitParam.wbmsVelocityCommandValid &&
    gaitParam.wbmsVelocityCommandAge <= gaitParam.wbmsVelocityCommandTimeout &&
    this->isOperationAllowed(gaitParam, isABCRunning);
  if(commandActive){
    desiredComVelocity = mathutil::clampMatrix<cnoid::Vector3>(gaitParam.wbmsRawComVelocityCommand, gaitParam.wbmsComVelocityLimit);
    desiredTorsoAngularVelocity = mathutil::clampMatrix<cnoid::Vector3>(gaitParam.wbmsRawTorsoAngularVelocityCommand, gaitParam.wbmsTorsoAngularVelocityLimit);
  }

  gaitParam.wbmsAppliedComVelocityCommand = this->applyAccelerationLimit(gaitParam.wbmsAppliedComVelocityCommand,
                                                                         desiredComVelocity,
                                                                         gaitParam.wbmsComAccelerationLimit,
                                                                         dt);
  gaitParam.wbmsAppliedTorsoAngularVelocityCommand = this->applyAccelerationLimit(gaitParam.wbmsAppliedTorsoAngularVelocityCommand,
                                                                                 desiredTorsoAngularVelocity,
                                                                                 gaitParam.wbmsTorsoAngularAccelerationLimit,
                                                                                 dt);
}

void WbmsPostureControl::syncProjectionRobot(const GaitParam& gaitParam){
  if(!this->wbmsPostureRobot_ || !gaitParam.genRobot) return;
  this->wbmsPostureRobot_->rootLink()->p() = gaitParam.genRobot->rootLink()->p();
  this->wbmsPostureRobot_->rootLink()->R() = gaitParam.genRobot->rootLink()->R();
  for(int i=0;i<gaitParam.genRobot->numJoints();i++){
    this->wbmsPostureRobot_->joint(i)->q() = gaitParam.genRobot->joint(i)->q();
  }
  this->wbmsPostureRobot_->calcForwardKinematics();
  this->wbmsPostureRobot_->calcCenterOfMass();
}

bool WbmsPostureControl::updateSupportHull(const GaitParam& gaitParam){
  this->supportHullInFootMid_.clear();
  this->shrunkSupportHullInFootMid_.clear();
  this->shrunkSupportHullInWorld_.clear();
  this->supportVerticesInFootMid_.clear();
  const cnoid::Isometry3 footMid = gaitParam.footMidCoords.value();
  const cnoid::Isometry3 footMidInv = footMid.inverse();

  for(int i=0;i<NUM_LEGS;i++){
    for(size_t j=0;j<gaitParam.legHull[i].size();j++){
      this->supportVerticesInFootMid_.push_back(footMidInv * (gaitParam.genCoords[i].value() * gaitParam.legHull[i][j]));
    }
  }
  mathutil::calcConvexHull(this->supportVerticesInFootMid_, this->supportHullInFootMid_, this->supportHullTmp_);
  mathutil::shrinkConvexHull2D(this->supportHullInFootMid_,
                               gaitParam.wbmsComXYSupportMargin,
                               this->shrunkSupportHullInFootMid_,
                               this->shrinkShiftedPoints_,
                               this->shrinkShiftedDirs_);
  if(this->shrunkSupportHullInFootMid_.size() < 3) return false;
  for(size_t i=0;i<this->shrunkSupportHullInFootMid_.size();i++){
    this->shrunkSupportHullInWorld_.push_back(footMid * this->shrunkSupportHullInFootMid_[i]);
  }
  return true;
}

bool WbmsPostureControl::calcProjectionTargets(const GaitParam& gaitParam, double dt, cnoid::Matrix3& targetChestR, cnoid::Vector3& targetRobotCom, cnoid::Vector3& currentComInFootMid, cnoid::Matrix3& currentChestRInFootMid){
  if(!this->wbmsPostureRobot_ || !gaitParam.wbmsPostureBaselineValid) return false;
  cnoid::LinkPtr chestLink = this->wbmsPostureRobot_->link(gaitParam.chestLinkName);
  if(!chestLink) return false;

  const cnoid::Isometry3 footMid = gaitParam.footMidCoords.value();
  currentChestRInFootMid = footMid.linear().transpose() * chestLink->R();
  cnoid::Matrix3 targetChestRInFootMidUnclamped =
    cnoid::rotFromRpy(gaitParam.wbmsAppliedTorsoAngularVelocityCommand * dt) * currentChestRInFootMid;
  cnoid::Matrix3 deltaR = targetChestRInFootMidUnclamped * gaitParam.wbmsStartChestRInFootMid.transpose();
  cnoid::Vector3 deltaRpy = cnoid::rpyFromRot(deltaR);
  cnoid::Vector3 clampedDeltaRpy = mathutil::clampMatrix<cnoid::Vector3>(deltaRpy, gaitParam.wbmsTorsoRpyLowerLimit, gaitParam.wbmsTorsoRpyUpperLimit);
  cnoid::Matrix3 targetChestRInFootMid = cnoid::rotFromRpy(clampedDeltaRpy) * gaitParam.wbmsStartChestRInFootMid;
  targetChestR = footMid.linear() * targetChestRInFootMid;

  currentComInFootMid = footMid.inverse() * this->wbmsPostureRobot_->centerOfMass();
  cnoid::Vector3 targetComInFootMid = currentComInFootMid + gaitParam.wbmsAppliedComVelocityCommand * dt;
  cnoid::Vector3 unclampedOffset = targetComInFootMid - gaitParam.wbmsStartComInFootMid;
  cnoid::Vector3 clampedOffset = mathutil::clampMatrix(unclampedOffset,
                                                       gaitParam.wbmsComOffsetLowerLimit,
                                                       gaitParam.wbmsComOffsetUpperLimit);
  targetComInFootMid = gaitParam.wbmsStartComInFootMid + clampedOffset;

  if(!this->updateSupportHull(gaitParam)) return false;
  cnoid::Vector3 nearest = mathutil::calcNearestPointOfHull(targetComInFootMid, this->shrunkSupportHullInFootMid_);
  targetComInFootMid[0] = nearest[0];
  targetComInFootMid[1] = nearest[1];
  targetRobotCom = footMid * targetComInFootMid;
  return targetChestR.allFinite() && targetRobotCom.allFinite();
}

bool WbmsPostureControl::solveProjection(GaitParam& gaitParam, double dt, const std::vector<cpp_filters::TwoPointInterpolator<double> >& referenceDqWeight){
  this->syncProjectionRobot(gaitParam);
  if(!this->isOperationAllowed(gaitParam, true) || !gaitParam.wbmsPostureBaselineValid || this->projectionJointIds_.empty()){
    this->setFallbackReference(gaitParam);
    return false;
  }

  cnoid::Matrix3 targetChestR;
  cnoid::Vector3 targetRobotCom;
  cnoid::Vector3 currentComInFootMid;
  cnoid::Matrix3 currentChestRInFootMid;
  if(!this->calcProjectionTargets(gaitParam, dt, targetChestR, targetRobotCom, currentComInFootMid, currentChestRInFootMid)){
    this->setFallbackReference(gaitParam);
    return false;
  }

  for(size_t i=0;i<this->projectionConstraints_.size();i++) this->projectionConstraints_[i].clear();

  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    int jointId = this->projectionJointIds_[i];
    cnoid::LinkPtr joint = this->wbmsPostureRobot_->joint(jointId);
    this->jointVelocityConstraint_[i]->joint() = joint;
    this->jointVelocityConstraint_[i]->dt() = dt;
    this->jointVelocityConstraint_[i]->maxError() = 1.0 * dt;
    this->jointVelocityConstraint_[i]->weight() = 1.0;
    this->projectionConstraints_[0].push_back(this->jointVelocityConstraint_[i]);

    this->jointLimitConstraint_[i]->joint() = joint;
    this->jointLimitConstraint_[i]->jointLimitTables() = gaitParam.jointLimitTables[jointId];
    this->jointLimitConstraint_[i]->maxError() = 1.0 * dt;
    this->jointLimitConstraint_[i]->weight() = 1.0;
    this->projectionConstraints_[0].push_back(this->jointLimitConstraint_[i]);
  }

  if(this->selfCollisionConstraint_.size() < gaitParam.selfCollision.size()){
    size_t prevSize = this->selfCollisionConstraint_.size();
    this->selfCollisionConstraint_.resize(gaitParam.selfCollision.size());
    for(size_t i=prevSize;i<this->selfCollisionConstraint_.size();i++){
      this->selfCollisionConstraint_[i] = std::make_shared<ik_constraint2::ClientCollisionConstraint>();
    }
  }
  for(size_t i=0;i<gaitParam.selfCollision.size();i++){
    this->selfCollisionConstraint_[i]->A_link() = this->wbmsPostureRobot_->link(gaitParam.selfCollision[i].link1);
    this->selfCollisionConstraint_[i]->B_link() = this->wbmsPostureRobot_->link(gaitParam.selfCollision[i].link2);
    this->selfCollisionConstraint_[i]->tolerance() = 0.01;
    this->selfCollisionConstraint_[i]->maxError() = 10.0 * dt;
    this->selfCollisionConstraint_[i]->weight() = 1.0;
    this->selfCollisionConstraint_[i]->velocityDamper() = 0.1 / dt;
    this->selfCollisionConstraint_[i]->A_localp() = gaitParam.selfCollision[i].point1;
    this->selfCollisionConstraint_[i]->B_localp() = gaitParam.selfCollision[i].point2;
    this->selfCollisionConstraint_[i]->direction() = gaitParam.selfCollision[i].direction21;
    if(gaitParam.selfCollision[i].distance < 0.05){
      this->projectionConstraints_[1].push_back(this->selfCollisionConstraint_[i]);
    }
  }

  for(int i=0;i<NUM_LEGS;i++){
    this->footConstraint_[i]->A_link() = this->wbmsPostureRobot_->link(gaitParam.eeParentLink[i]);
    this->footConstraint_[i]->A_localpos() = gaitParam.eeLocalT[i];
    this->footConstraint_[i]->B_link() = nullptr;
    this->footConstraint_[i]->B_localpos() = gaitParam.abcEETargetPose[i];
    this->footConstraint_[i]->maxError() << 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt, 10.0*dt;
    this->footConstraint_[i]->precision() = 0.0;
    this->footConstraint_[i]->weight() = cnoid::Vector6::Constant(3.0);
    this->footConstraint_[i]->eval_link() = this->wbmsPostureRobot_->link(gaitParam.eeParentLink[i]);
    this->footConstraint_[i]->eval_localR() = gaitParam.eeLocalT[i].linear();
    this->projectionConstraints_[2].push_back(this->footConstraint_[i]);
  }

  cnoid::LinkPtr chestLink = this->wbmsPostureRobot_->link(gaitParam.chestLinkName);
  this->chestConstraint_->A_link() = chestLink;
  this->chestConstraint_->A_localpos() = cnoid::Isometry3::Identity();
  this->chestConstraint_->B_link() = nullptr;
  this->chestConstraint_->B_localpos() = cnoid::Isometry3::Identity();
  this->chestConstraint_->B_localpos().linear() = targetChestR;
  this->chestConstraint_->B_localpos().translation() = chestLink->p();
  this->chestConstraint_->maxError() << 10.0*dt, 10.0*dt, 10.0*dt,
    gaitParam.wbmsTorsoOrientationMaxError[0] * dt,
    gaitParam.wbmsTorsoOrientationMaxError[1] * dt,
    gaitParam.wbmsTorsoOrientationMaxError[2] * dt;
  this->chestConstraint_->precision() = 0.0;
  this->chestConstraint_->weight() << 0.0, 0.0, 0.0,
    gaitParam.wbmsTorsoOrientationWeight[0],
    gaitParam.wbmsTorsoOrientationWeight[1],
    gaitParam.wbmsTorsoOrientationWeight[2];
  this->chestConstraint_->eval_link() = chestLink;
  this->chestConstraint_->eval_localR() = cnoid::Matrix3::Identity();
  this->projectionConstraints_[3].push_back(this->chestConstraint_);

  this->comConstraint_->A_robot() = this->wbmsPostureRobot_;
  this->comConstraint_->A_localp() = cnoid::Vector3::Zero();
  this->comConstraint_->B_robot() = nullptr;
  this->comConstraint_->B_localp() = targetRobotCom;
  this->comConstraint_->maxError() << gaitParam.wbmsComVelocityLimit[0] * dt,
    gaitParam.wbmsComVelocityLimit[1] * dt,
    gaitParam.wbmsComVelocityLimit[2] * dt;
  this->comConstraint_->precision() = 0.0;
  this->comConstraint_->weight() = gaitParam.wbmsComPositionWeight;
  this->comConstraint_->eval_R() = cnoid::Matrix3::Identity();
  this->projectionConstraints_[3].push_back(this->comConstraint_);

  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    int jointId = this->projectionJointIds_[i];
    cnoid::LinkPtr joint = this->wbmsPostureRobot_->joint(jointId);
    this->postureReferenceConstraint_[i]->joint() = joint;
    this->postureReferenceConstraint_[i]->targetq() = gaitParam.genRobot->joint(jointId)->q();
    this->postureReferenceConstraint_[i]->maxError() = 10.0 * dt;
    this->postureReferenceConstraint_[i]->precision() = 0.0;
    this->postureReferenceConstraint_[i]->weight() = 1e-4;
    this->projectionConstraints_[4].push_back(this->postureReferenceConstraint_[i]);
  }

  for(int i=0;i<6;i++) this->projectionIKParam_.dqWeight[i] = 1.0;
  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    int jointId = this->projectionJointIds_[i];
    this->projectionIKParam_.dqWeight[6+i] = jointId < referenceDqWeight.size() ? referenceDqWeight[jointId].value() : 1.0;
  }

  for(size_t i=0;i<this->projectionConstraints_.size();i++){
    for(size_t j=0;j<this->projectionConstraints_[i].size();j++) this->projectionConstraints_[i][j]->debugLevel() = 0;
  }

  this->projectionIKParam_.dt = dt;
  bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(this->projectionVariables_,
                                                                    this->projectionConstraints_,
                                                                    this->projectionTasks_,
                                                                    this->projectionIKParam_);

  this->wbmsPostureRobot_->calcForwardKinematics();
  this->wbmsPostureRobot_->calcCenterOfMass();
  bool valid = solved && this->validateProjection(gaitParam, dt);
  if(!valid){
    this->setFallbackReference(gaitParam);
    return false;
  }

  gaitParam.wbmsPostureReferenceQ.resize(gaitParam.genRobot->numJoints());
  gaitParam.wbmsPostureReferenceJointMask.assign(gaitParam.genRobot->numJoints(), false);
  for(int i=0;i<gaitParam.genRobot->numJoints();i++){
    gaitParam.wbmsPostureReferenceQ[i] = this->wbmsPostureRobot_->joint(i)->q();
  }
  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    int jointId = this->projectionJointIds_[i];
    if(jointId >= 0 && jointId < gaitParam.wbmsPostureReferenceJointMask.size()){
      gaitParam.wbmsPostureReferenceJointMask[jointId] = true;
    }
  }
  cnoid::LinkPtr projectedChestLink = this->wbmsPostureRobot_->link(gaitParam.chestLinkName);
  gaitParam.wbmsProjectedChestR = projectedChestLink->R();
  gaitParam.wbmsProjectedRobotCom = this->wbmsPostureRobot_->centerOfMass();
  const cnoid::Isometry3 footMid = gaitParam.footMidCoords.value();
  cnoid::Vector3 projectedComInFootMid = footMid.inverse() * gaitParam.wbmsProjectedRobotCom;
  cnoid::Matrix3 projectedChestRInFootMid = footMid.linear().transpose() * gaitParam.wbmsProjectedChestR;
  gaitParam.wbmsRealizedComVelocity = (projectedComInFootMid - currentComInFootMid) / dt;
  gaitParam.wbmsRealizedTorsoAngularVelocity = cnoid::rpyFromRot(projectedChestRInFootMid * currentChestRInFootMid.transpose()) / dt;
  gaitParam.wbmsPostureReferenceValid = true;
  return true;
}

bool WbmsPostureControl::validateProjection(const GaitParam& gaitParam, double dt) const{
  if(!this->wbmsPostureRobot_ || !this->wbmsPostureRobot_->rootLink()->p().allFinite() || !this->wbmsPostureRobot_->rootLink()->R().allFinite()) return false;
  cnoid::LinkPtr chestLink = this->wbmsPostureRobot_->link(gaitParam.chestLinkName);
  if(!chestLink || !chestLink->R().allFinite() || !this->wbmsPostureRobot_->centerOfMass().allFinite()) return false;

  const double maxRootTranslation = 0.20;
  const double maxRootRotation = 0.50;
  if((this->wbmsPostureRobot_->rootLink()->p() - gaitParam.genRobot->rootLink()->p()).norm() > maxRootTranslation) return false;
  if(cnoid::AngleAxis(this->wbmsPostureRobot_->rootLink()->R() * gaitParam.genRobot->rootLink()->R().transpose()).angle() > maxRootRotation) return false;

  for(size_t i=0;i<this->projectionJointIds_.size();i++){
    int jointId = this->projectionJointIds_[i];
    cnoid::LinkPtr joint = this->wbmsPostureRobot_->joint(jointId);
    if(!std::isfinite(joint->q())) return false;
    double u = gaitParam.refRobot->joint(jointId)->q_upper();
    double l = gaitParam.refRobot->joint(jointId)->q_lower();
    for(size_t j=0;j<gaitParam.jointLimitTables[jointId].size();j++){
      u = std::min(u, gaitParam.jointLimitTables[jointId][j]->getUlimit());
      l = std::max(l, gaitParam.jointLimitTables[jointId][j]->getLlimit());
    }
    if(joint->q() < l - 1e-6 || joint->q() > u + 1e-6) return false;
    if(std::abs(joint->q() - gaitParam.genRobot->joint(jointId)->q()) > std::max(0.20, 2.0 * dt)) return false;
  }

  for(int i=0;i<NUM_LEGS;i++){
    cnoid::LinkPtr footLink = this->wbmsPostureRobot_->link(gaitParam.eeParentLink[i]);
    if(!footLink) return false;
    cnoid::Isometry3 footPose = footLink->T() * gaitParam.eeLocalT[i];
    cnoid::Isometry3 error = gaitParam.abcEETargetPose[i].inverse() * footPose;
    if(error.translation().norm() > 0.02) return false;
    if(cnoid::AngleAxis(error.linear()).angle() > 0.10) return false;
  }
  return true;
}

void WbmsPostureControl::setFallbackReference(GaitParam& gaitParam) const{
  gaitParam.wbmsPostureReferenceQ.resize(gaitParam.genRobot->numJoints());
  gaitParam.wbmsPostureReferenceJointMask.assign(gaitParam.genRobot->numJoints(), false);
  for(int i=0;i<gaitParam.genRobot->numJoints();i++){
    gaitParam.wbmsPostureReferenceQ[i] = gaitParam.genRobot->joint(i)->q();
  }
  cnoid::LinkPtr chestLink = gaitParam.genRobot->link(gaitParam.chestLinkName);
  if(chestLink) gaitParam.wbmsProjectedChestR = chestLink->R();
  else gaitParam.wbmsProjectedChestR.setIdentity();
  gaitParam.genRobot->calcCenterOfMass();
  gaitParam.wbmsProjectedRobotCom = gaitParam.genRobot->centerOfMass();
  gaitParam.wbmsRealizedComVelocity.setZero();
  gaitParam.wbmsRealizedTorsoAngularVelocity.setZero();
  gaitParam.wbmsPostureReferenceValid = false;
}

void WbmsPostureControl::applyStaticComZmpIntegration(GaitParam& gaitParam, double dt){
  if(!gaitParam.wbmsPostureReferenceValid) return;
  if(!this->isOperationAllowed(gaitParam, true)) return;
  double operationMode = mathutil::clamp(gaitParam.wbmsOperationModeValue, 0.0, 1.0);
  if(operationMode <= 0.0) return;

  const cnoid::Vector3 nominalGenCog = gaitParam.genCog;
  const cnoid::Vector3 nominalGenCogVel = gaitParam.genCogVel;
  const cnoid::Vector3 nominalGenCogAcc = gaitParam.genCogAcc;
  const double oldRefdz = gaitParam.refdz;
  const double oldOmega = gaitParam.omega;
  const cnoid::Vector3 oldL = gaitParam.l;
  const cnoid::Vector3 nominalZmp = gaitParam.refZmpTraj.empty() ? cnoid::Vector3::Zero() : gaitParam.refZmpTraj[0].getStart();

  cnoid::Vector3 projectedGenCog = gaitParam.wbmsProjectedRobotCom - gaitParam.sbpOffset;
  cnoid::Vector3 projectedGenCogVel = gaitParam.footMidCoords.value().linear() * gaitParam.wbmsRealizedComVelocity;
  cnoid::Vector3 projectedGenCogAccRaw = (projectedGenCogVel - nominalGenCogVel) / std::max(dt, 1e-6);
  cnoid::Vector3 projectedGenCogAccLimit = gaitParam.wbmsComAccelerationLimit.cwiseMax(cnoid::Vector3::Constant(1e-6));
  cnoid::Vector3 projectedGenCogAcc = mathutil::clampMatrix(projectedGenCogAccRaw, projectedGenCogAccLimit);

  gaitParam.genCog = nominalGenCog * (1.0 - operationMode) + projectedGenCog * operationMode;
  gaitParam.genCogVel = nominalGenCogVel * (1.0 - operationMode) + projectedGenCogVel * operationMode;
  gaitParam.genCogAcc = nominalGenCogAcc * (1.0 - operationMode) + projectedGenCogAcc * operationMode;

  double newRefdz = std::max(gaitParam.wbmsProjectedRobotCom[2] - gaitParam.footMidCoords.value().translation()[2], 0.1);
  double effectiveGravity = oldOmega * oldOmega * std::max(oldRefdz, 0.1);
  gaitParam.refdz = oldRefdz * (1.0 - operationMode) + newRefdz * operationMode;
  gaitParam.l[0] = oldL[0];
  gaitParam.l[1] = oldL[1];
  gaitParam.l[2] = gaitParam.refdz;
  gaitParam.omega = std::sqrt(effectiveGravity / std::max(gaitParam.refdz, 0.1));

  cnoid::Vector3 wbmsZmp = gaitParam.genCog - gaitParam.l;
  double omega2 = std::max(gaitParam.omega * gaitParam.omega, 1e-6);
  wbmsZmp[0] -= gaitParam.genCogAcc[0] / omega2;
  wbmsZmp[1] -= gaitParam.genCogAcc[1] / omega2;
  wbmsZmp[2] = gaitParam.footMidCoords.value().translation()[2];
  if(!this->shrunkSupportHullInWorld_.empty()){
    cnoid::Vector3 nearest = mathutil::calcNearestPointOfHull(wbmsZmp, this->shrunkSupportHullInWorld_);
    wbmsZmp[0] = nearest[0];
    wbmsZmp[1] = nearest[1];
  }
  cnoid::Vector3 blendedZmp = nominalZmp * (1.0 - operationMode) + wbmsZmp * operationMode;
  gaitParam.refZmpTraj.clear();
  gaitParam.refZmpTraj.push_back(footguidedcontroller::LinearTrajectory<cnoid::Vector3>(blendedZmp, blendedZmp, 0.0));
}

void WbmsPostureControl::proc(GaitParam& gaitParam, double dt, bool isABCRunning, const std::vector<cpp_filters::TwoPointInterpolator<double> >& referenceDqWeight){
  std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
  double walkingStabilityTarget = (!gaitParam.isStatic() || gaitParam.isWbmsWalkingStartDelay) ? 1.0 : 0.0;
  if(this->wbmsWalkingStabilityMode_.getGoal() != walkingStabilityTarget){
    this->wbmsWalkingStabilityMode_.setGoal(walkingStabilityTarget,
                                            walkingStabilityTarget > this->wbmsWalkingStabilityMode_.getGoal() ? gaitParam.wbmsWalkingStabilityStartTime : gaitParam.wbmsWalkingStabilityStopTime);
  }
  this->wbmsWalkingStabilityMode_.interpolate(dt);
  gaitParam.wbmsWalkingStabilityModeValue = this->wbmsWalkingStabilityMode_.value();
  gaitParam.wbmsOperationModeValue = gaitParam.wbmsMode.value() * (1.0 - gaitParam.wbmsWalkingStabilityModeValue);

  this->updateVelocityCommand(gaitParam, dt, isABCRunning);
  if(this->solveProjection(gaitParam, dt, referenceDqWeight)){
    this->applyStaticComZmpIntegration(gaitParam, dt);
  }
  gaitParam.debugData.wbmsProjectorTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
}
