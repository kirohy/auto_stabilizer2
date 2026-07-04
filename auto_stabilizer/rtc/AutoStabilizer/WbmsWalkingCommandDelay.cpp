#include "WbmsWalkingCommandDelay.h"

#include <algorithm>
#include <cmath>

bool WbmsWalkingCommandDelay::shouldDelay(const GaitParam& gaitParam) const{
  return gaitParam.isStatic() &&
    (gaitParam.wbmsMode.value() > 0.0 || gaitParam.wbmsMode.getGoal() > 0.0);
}

bool WbmsWalkingCommandDelay::hasPendingCommand() const{
  return this->command_ != Command::NONE;
}

void WbmsWalkingCommandDelay::storeGoVelocity(GaitParam& gaitParam, const cnoid::Vector3& refCmdVel){
  this->command_ = Command::GO_VELOCITY;
  this->velocity_ = refCmdVel;
  this->footsteps_.clear();
  this->requestPreparation(gaitParam);
}

void WbmsWalkingCommandDelay::storeGoPos(GaitParam& gaitParam, const cnoid::Vector3& goPos){
  this->command_ = Command::GO_POS;
  this->goPos_ = goPos;
  this->footsteps_.clear();
  this->requestPreparation(gaitParam);
}

void WbmsWalkingCommandDelay::storeFootSteps(GaitParam& gaitParam, const std::vector<FootStepGenerator::StepNode>& footsteps){
  this->command_ = Command::SET_FOOTSTEPS;
  this->footsteps_ = footsteps;
  this->velocity_.setZero();
  this->requestPreparation(gaitParam);
}

void WbmsWalkingCommandDelay::clearPendingCommand(){
  this->command_ = Command::NONE;
  this->velocity_.setZero();
  this->goPos_.setZero();
  this->footsteps_.clear();
}

void WbmsWalkingCommandDelay::clear(GaitParam& gaitParam){
  this->clearPendingCommand();
  gaitParam.clearWbmsWalkingPreparation();
}

void WbmsWalkingCommandDelay::proc(GaitParam& gaitParam, double dt, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator){
  gaitParam.debugData.wbmsWalkingPendingCommandReleaseEvent = false;
  if(gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_INACTIVE ||
     gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_FAILED) return;

  bool wbmsActive = (gaitParam.wbmsMode.value() > 0.0 || gaitParam.wbmsMode.getGoal() > 0.0);
  if(!wbmsActive){
    this->fail(gaitParam, GaitParam::WBMS_WALKING_PREPARATION_FAILURE_CANCELLED);
    return;
  }

  gaitParam.wbmsWalkingPreparationElapsedTime += std::max(dt, 0.0);
  gaitParam.wbmsWalkingPreparationPhaseElapsedTime += std::max(dt, 0.0);
  gaitParam.wbmsWalkingStartDelayRemainTime = std::max(0.0, gaitParam.wbmsWalkingPreparationTimeout - gaitParam.wbmsWalkingPreparationElapsedTime);

  if(gaitParam.wbmsWalkingPreparationElapsedTime > gaitParam.wbmsWalkingPreparationTimeout){
    this->fail(gaitParam, GaitParam::WBMS_WALKING_PREPARATION_FAILURE_TIMEOUT);
    return;
  }

  if(gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_READY &&
     gaitParam.wbmsWalkingPreparationReleaseRequested){
    this->releasePendingCommand(gaitParam, cmdVelGenerator, footStepGenerator);
    gaitParam.wbmsWalkingPreparationPhase = GaitParam::WBMS_WALKING_PREPARATION_WALKING_HOLD;
    gaitParam.wbmsWalkingPreparationReleaseRequested = false;
    gaitParam.isWbmsWalkingStartDelay = false;
    gaitParam.wbmsWalkingStartDelayRemainTime = 0.0;
    return;
  }

  if(gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_REQUESTED){
    if(!this->snapshotPreparation(gaitParam)){
      this->fail(gaitParam, GaitParam::WBMS_WALKING_PREPARATION_FAILURE_SNAPSHOT);
      return;
    }
    gaitParam.wbmsWalkingPreparationPhase = GaitParam::WBMS_WALKING_PREPARATION_DECELERATING;
    gaitParam.wbmsWalkingPreparationPhaseElapsedTime = 0.0;
    return;
  }
}

void WbmsWalkingCommandDelay::releasePendingCommand(GaitParam& gaitParam, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator){
  Command command = this->command_;
  if(command == Command::GO_VELOCITY){
    cmdVelGenerator.refCmdVel = this->velocity_;
    footStepGenerator.isGoVelocityMode = true;
  }else if(command == Command::GO_POS){
    footStepGenerator.goPos(gaitParam, this->goPos_[0], this->goPos_[1], this->goPos_[2],
                            gaitParam.footstepNodesList);
  }else if(command == Command::SET_FOOTSTEPS){
    footStepGenerator.setFootSteps(gaitParam, this->footsteps_,
                                   gaitParam.footstepNodesList);
  }
  this->clearPendingCommand();
  gaitParam.debugData.wbmsWalkingPendingCommandReleaseEvent = (command != Command::NONE);
}

void WbmsWalkingCommandDelay::requestPreparation(GaitParam& gaitParam){
  bool preparationRunning =
    gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_REQUESTED ||
    gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_DECELERATING ||
    gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_RETURNING ||
    gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_HANDOFF ||
    gaitParam.wbmsWalkingPreparationPhase == GaitParam::WBMS_WALKING_PREPARATION_READY;
  if(preparationRunning){
    gaitParam.isWbmsWalkingStartDelay = true;
    gaitParam.clearWbmsPostureCommand(false);
    return;
  }

  gaitParam.isWbmsWalkingStartDelay = true;
  gaitParam.wbmsWalkingPreparationPhase = GaitParam::WBMS_WALKING_PREPARATION_REQUESTED;
  gaitParam.wbmsWalkingPreparationFailureCode = GaitParam::WBMS_WALKING_PREPARATION_FAILURE_NONE;
  gaitParam.wbmsWalkingPreparationReleaseRequested = false;
  gaitParam.wbmsWalkingPreparationSnapshotValid = false;
  gaitParam.wbmsWalkingComHeightHoldValid = false;
  gaitParam.wbmsWalkingPreparationElapsedTime = 0.0;
  gaitParam.wbmsWalkingPreparationPhaseElapsedTime = 0.0;
  gaitParam.wbmsWalkingPreparationSettleElapsedTime = 0.0;
  gaitParam.wbmsWalkingPreparationReturnAlpha = 0.0;
  gaitParam.wbmsWalkingPreparationHandoffAlpha = 0.0;
  gaitParam.wbmsWalkingStartDelayRemainTime = gaitParam.wbmsWalkingPreparationTimeout;
  gaitParam.clearWbmsPostureCommand(false);
}

bool WbmsWalkingCommandDelay::snapshotPreparation(GaitParam& gaitParam){
  if(!gaitParam.genRobot) return false;
  cnoid::LinkPtr chestLink = gaitParam.genRobot->link(gaitParam.chestLinkName);
  if(!chestLink) return false;

  gaitParam.genRobot->calcForwardKinematics();
  gaitParam.genRobot->calcCenterOfMass();
  const cnoid::Isometry3 footMid = gaitParam.footMidCoords.value();
  const cnoid::Isometry3 footMidInv = footMid.inverse();

  cnoid::Vector3 robotCom = cnoid::Vector3::Zero();
  if(gaitParam.wbmsPostureReferenceValid && gaitParam.wbmsProjectedRobotCom.allFinite()){
    robotCom = gaitParam.wbmsProjectedRobotCom;
  }else if(gaitParam.genRobot->centerOfMass().allFinite()){
    robotCom = gaitParam.genRobot->centerOfMass();
  }else{
    return false;
  }
  cnoid::Vector3 robotComInFootMid = footMidInv * robotCom;
  if(!gaitParam.wbmsNominalGenCogBeforeWbmsIntegrationValid ||
     !gaitParam.wbmsNominalGenCogBeforeWbmsIntegration.allFinite()) return false;
  cnoid::Vector3 nominalRobotComInFootMid = footMidInv * (gaitParam.wbmsNominalGenCogBeforeWbmsIntegration + gaitParam.sbpOffset);
  if(!robotComInFootMid.allFinite() || !nominalRobotComInFootMid.allFinite()) return false;

  gaitParam.wbmsWalkingPreparationStartChestRInFootMid = footMid.linear().transpose() * chestLink->R();
  gaitParam.wbmsWalkingPreparationStartRobotComInFootMid = robotComInFootMid;
  gaitParam.wbmsWalkingPreparationNominalRobotComInFootMid = nominalRobotComInFootMid;
  gaitParam.heldRobotComHeightInFootMid = robotComInFootMid[2];
  gaitParam.wbmsWalkingPreparationStartRootR = gaitParam.genRobot->rootLink()->R();
  gaitParam.wbmsWalkingPreparationSnapshotValid = gaitParam.wbmsWalkingPreparationStartChestRInFootMid.allFinite() &&
    gaitParam.wbmsWalkingPreparationStartRootR.allFinite() &&
    std::isfinite(gaitParam.heldRobotComHeightInFootMid);
  gaitParam.wbmsWalkingComHeightHoldValid = gaitParam.wbmsWalkingPreparationSnapshotValid;
  return gaitParam.wbmsWalkingPreparationSnapshotValid;
}

void WbmsWalkingCommandDelay::fail(GaitParam& gaitParam, GaitParam::WbmsWalkingPreparationFailureCode code){
  this->clearPendingCommand();
  gaitParam.isWbmsWalkingStartDelay = false;
  gaitParam.wbmsWalkingStartDelayRemainTime = 0.0;
  gaitParam.wbmsWalkingPreparationPhase = GaitParam::WBMS_WALKING_PREPARATION_FAILED;
  gaitParam.wbmsWalkingPreparationFailureCode = code;
  gaitParam.wbmsWalkingPreparationReleaseRequested = false;
  gaitParam.wbmsWalkingComHeightHoldValid = false;
  gaitParam.clearWbmsPostureCommand(false);
}
