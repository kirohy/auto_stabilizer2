#ifndef WBMSWALKINGCOMMANDDELAY_H
#define WBMSWALKINGCOMMANDDELAY_H

#include <vector>

#include "CmdVelGenerator.h"
#include "FootStepGenerator.h"

class WbmsWalkingCommandDelay{
public:
  bool isWbmsActive(const GaitParam& gaitParam) const;
  bool isReady(const GaitParam& gaitParam) const;
  bool hasPendingCommand() const;
  void startPreparation(GaitParam& gaitParam);
  void cancelPreparation(GaitParam& gaitParam);
  void clearPendingCommand();
  void clear(GaitParam& gaitParam);
  void proc(GaitParam& gaitParam, double dt, CmdVelGenerator& cmdVelGenerator, FootStepGenerator& footStepGenerator);

protected:
  bool snapshotPreparation(GaitParam& gaitParam);
  void fail(GaitParam& gaitParam, GaitParam::WbmsWalkingPreparationFailureCode code);
};

#endif
