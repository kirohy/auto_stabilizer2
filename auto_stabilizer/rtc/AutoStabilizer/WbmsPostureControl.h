#ifndef WBMSPOSTURECONTROL_H
#define WBMSPOSTURECONTROL_H

#include "GaitParam.h"
#include <cpp_filters/TwoPointInterpolator.h>

class WbmsPostureControl{
public:
  void init(const cnoid::BodyPtr& genRobot);
  void reset();
  void start(GaitParam& gaitParam);
  void clearStaleCommand(GaitParam& gaitParam, bool resetApplied) const;
  void updateVelocityCommand(GaitParam& gaitParam, double dt, bool isABCRunning) const;
  void proc(GaitParam& gaitParam, double dt, bool isABCRunning);

private:
  cnoid::BodyPtr wbmsPostureRobot_;
  cpp_filters::TwoPointInterpolator<double> wbmsWalkingStabilityMode_ = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0,cpp_filters::HOFFARBIB);

  bool isOperationAllowed(const GaitParam& gaitParam, bool isABCRunning) const;
  cnoid::Vector3 applyAccelerationLimit(const cnoid::Vector3& current, const cnoid::Vector3& desired, const cnoid::Vector3& limit, double dt) const;
};

#endif
