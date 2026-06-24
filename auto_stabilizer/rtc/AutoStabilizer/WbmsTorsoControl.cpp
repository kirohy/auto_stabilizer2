#include "WbmsTorsoControl.h"

#include "MathUtil.h"
#include <cnoid/EigenUtil>

namespace {
  cnoid::Vector3 calcRealizedWbmsTorsoRpy(const GaitParam& gaitParam){
    cnoid::Matrix3 footMidR = gaitParam.footMidCoords.value().linear();
    cnoid::Matrix3 realizedRootDiffR = footMidR.transpose()
      * gaitParam.genRobot->rootLink()->R()
      * gaitParam.stTargetRootPose.linear().transpose()
      * footMidR;
    return cnoid::rpyFromRot(realizedRootDiffR);
  }
}

void WbmsTorsoControl::proc(GaitParam& gaitParam, double dt, bool isABCRunning) const{
  bool wbmsAcceptInput = isABCRunning && gaitParam.wbmsMode.getGoal() > 0.0;
  if(wbmsAcceptInput){
    cnoid::Vector3 angvel = mathutil::clampMatrix<cnoid::Vector3>(gaitParam.refTorsoAnglVel.value(), gaitParam.wbmsTorsoAngularVelocityLimit);
    gaitParam.wbmsTorsoTargetRpy += angvel * dt;
    gaitParam.wbmsTorsoTargetRpy = mathutil::clampMatrix<cnoid::Vector3>(gaitParam.wbmsTorsoTargetRpy,
                                                                         gaitParam.wbmsTorsoRpyLowerLimit,
                                                                         gaitParam.wbmsTorsoRpyUpperLimit);
    if(gaitParam.genRobot){
      // 到達不能な体幹姿勢目標が内部に残り続けないよう、実現姿勢から一定以上離れた分だけ戻す.
      cnoid::Vector3 realizedRpy = calcRealizedWbmsTorsoRpy(gaitParam);
      cnoid::Vector3 rpyErrorLimit(0.15, 0.15, 0.30);
      cnoid::Vector3 rpyError = gaitParam.wbmsTorsoTargetRpy - realizedRpy;
      gaitParam.wbmsTorsoTargetRpy = realizedRpy + mathutil::clampMatrix<cnoid::Vector3>(rpyError, rpyErrorLimit);
      gaitParam.wbmsTorsoTargetRpy = mathutil::clampMatrix<cnoid::Vector3>(gaitParam.wbmsTorsoTargetRpy,
                                                                           gaitParam.wbmsTorsoRpyLowerLimit,
                                                                           gaitParam.wbmsTorsoRpyUpperLimit);
    }
  }else if(gaitParam.wbmsMode.value() <= 0.0 && gaitParam.wbmsMode.getGoal() <= 0.0){
    gaitParam.wbmsTorsoTargetRpy = cnoid::Vector3::Zero();
  }
}
