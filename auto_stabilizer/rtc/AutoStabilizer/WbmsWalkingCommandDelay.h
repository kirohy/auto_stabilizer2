#ifndef WBMSWALKINGCOMMANDDELAY_H
#define WBMSWALKINGCOMMANDDELAY_H

#include <vector>

#include "CmdVelGenerator.h"
#include "FootStepGenerator.h"

class WbmsWalkingCommandDelay{
public:
  bool shouldDelay(const GaitParam& gaitParam) const;
  bool hasPendingCommand() const;
  void storeGoVelocity(GaitParam& gaitParam, const cnoid::Vector3& refCmdVel);
  void storeGoPos(GaitParam& gaitParam, const cnoid::Vector3& goPos);
  void storeFootSteps(GaitParam& gaitParam, const std::vector<FootStepGenerator::StepNode>& footsteps);
  void clearPendingCommand();
  void clear(GaitParam& gaitParam);
  void proc(GaitParam& gaitParam, double dt, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator);

protected:
  enum class Command{ NONE, GO_VELOCITY, GO_POS, SET_FOOTSTEPS };

  void requestPreparation(GaitParam& gaitParam);
  bool snapshotPreparation(GaitParam& gaitParam);
  void releasePendingCommand(GaitParam& gaitParam, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator);
  void fail(GaitParam& gaitParam, GaitParam::WbmsWalkingPreparationFailureCode code);

  Command command_ = Command::NONE;
  cnoid::Vector3 velocity_ = cnoid::Vector3::Zero();
  cnoid::Vector3 goPos_ = cnoid::Vector3::Zero();
  std::vector<FootStepGenerator::StepNode> footsteps_;
};

#endif
