#include "WbmsPostureControl.h"

#include "MathUtil.h"
#include <cnoid/EigenUtil>

void WbmsPostureControl::init(const cnoid::BodyPtr& genRobot){
  this->wbmsPostureRobot_ = genRobot->clone();
  this->wbmsPostureRobot_->calcForwardKinematics();
  this->wbmsPostureRobot_->calcCenterOfMass();
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

void WbmsPostureControl::proc(GaitParam& gaitParam, double dt, bool isABCRunning){
  double walkingStabilityTarget = (!gaitParam.isStatic() || gaitParam.isWbmsWalkingStartDelay) ? 1.0 : 0.0;
  if(this->wbmsWalkingStabilityMode_.getGoal() != walkingStabilityTarget){
    this->wbmsWalkingStabilityMode_.setGoal(walkingStabilityTarget,
                                            walkingStabilityTarget > this->wbmsWalkingStabilityMode_.getGoal() ? gaitParam.wbmsWalkingStabilityStartTime : gaitParam.wbmsWalkingStabilityStopTime);
  }
  this->wbmsWalkingStabilityMode_.interpolate(dt);
  gaitParam.wbmsWalkingStabilityModeValue = this->wbmsWalkingStabilityMode_.value();
  gaitParam.wbmsOperationModeValue = gaitParam.wbmsMode.value() * (1.0 - gaitParam.wbmsWalkingStabilityModeValue);

  this->updateVelocityCommand(gaitParam, dt, isABCRunning);
}
