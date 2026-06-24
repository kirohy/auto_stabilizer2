#ifndef WBMSTORSOCONTROL_H
#define WBMSTORSOCONTROL_H

#include "GaitParam.h"

class WbmsTorsoControl{
public:
  void proc(GaitParam& gaitParam, double dt, bool isABCRunning) const;
};

#endif
