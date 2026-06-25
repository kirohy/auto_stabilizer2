#include "WbmsWalkingCommandDelay.h"

#include <algorithm>

bool WbmsWalkingCommandDelay::shouldDelay(const GaitParam& gaitParam) const{
  return gaitParam.isStatic() &&
    (gaitParam.wbmsMode.value() > 0.0 || gaitParam.wbmsMode.getGoal() > 0.0) &&
    gaitParam.wbmsWalkingStabilityStartTime > 0.0;
}

bool WbmsWalkingCommandDelay::hasPendingCommand() const{
  return this->command_ != Command::NONE;
}

void WbmsWalkingCommandDelay::storeGoVelocity(GaitParam& gaitParam, const cnoid::Vector3& refCmdVel){
  this->command_ = Command::GO_VELOCITY;
  this->velocity_ = refCmdVel;
  this->footsteps_.clear();
  this->startDelay(gaitParam);
}

void WbmsWalkingCommandDelay::storeGoPos(GaitParam& gaitParam, const cnoid::Vector3& goPos){
  this->command_ = Command::GO_POS;
  this->goPos_ = goPos;
  this->footsteps_.clear();
  this->startDelay(gaitParam);
}

void WbmsWalkingCommandDelay::storeFootSteps(GaitParam& gaitParam, const std::vector<FootStepGenerator::StepNode>& footsteps){
  this->command_ = Command::SET_FOOTSTEPS;
  this->footsteps_ = footsteps;
  this->velocity_.setZero();
  this->startDelay(gaitParam);
}

void WbmsWalkingCommandDelay::clear(GaitParam& gaitParam){
  this->command_ = Command::NONE;
  this->velocity_.setZero();
  this->goPos_.setZero();
  this->footsteps_.clear();
  gaitParam.isWbmsWalkingStartDelay = false;
  gaitParam.wbmsWalkingStartDelayRemainTime = 0.0;
}

void WbmsWalkingCommandDelay::proc(GaitParam& gaitParam, double dt, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator){
  if(!gaitParam.isWbmsWalkingStartDelay) return;

  bool wbmsActive = (gaitParam.wbmsMode.value() > 0.0 || gaitParam.wbmsMode.getGoal() > 0.0);
  if(wbmsActive) gaitParam.wbmsWalkingStartDelayRemainTime = std::max(0.0, gaitParam.wbmsWalkingStartDelayRemainTime - dt);
  else gaitParam.wbmsWalkingStartDelayRemainTime = 0.0;
  if(gaitParam.wbmsWalkingStartDelayRemainTime > 0.0) return;

  Command command = this->command_;
  cnoid::Vector3 velocity = this->velocity_;
  cnoid::Vector3 goPos = this->goPos_;
  std::vector<FootStepGenerator::StepNode> footsteps = this->footsteps_;
  this->clear(gaitParam);

  if(command == Command::GO_VELOCITY){
    cmdVelGenerator.refCmdVel = velocity;
    footStepGenerator.isGoVelocityMode = true;
  }else if(command == Command::GO_POS){
    footStepGenerator.goPos(gaitParam, goPos[0], goPos[1], goPos[2],
                            gaitParam.footstepNodesList);
  }else if(command == Command::SET_FOOTSTEPS){
    footStepGenerator.setFootSteps(gaitParam, footsteps,
                                   gaitParam.footstepNodesList);
  }
}

void WbmsWalkingCommandDelay::startDelay(GaitParam& gaitParam){
  if(!gaitParam.isWbmsWalkingStartDelay){
    gaitParam.isWbmsWalkingStartDelay = true;
    gaitParam.wbmsWalkingStartDelayRemainTime = gaitParam.wbmsWalkingStabilityStartTime;
    gaitParam.clearWbmsPostureCommand(false);
  }
}
