#ifndef EXTERNALFORCEHANDER_H
#define EXTERNALFORCEHANDER_H

#include "GaitParam.h"
#include <list>

class ExternalForceHandler{
public:
  // ExternalForceHandlerだけが使うパラメータ
  bool useDisturbanceCompensation = true; // 長期的外乱補償を行うかどうか
  double disturbanceCompensationTimeConst = 2.0; // [s].長期的外乱補償の時定数. (外乱が減る方向には0.1倍になる). 0より大きい
  int disturbanceCompensationStepNum = 4; // footstepNodesListsの何ステップぶんを積算するか. 4なら、右脚+左脚ぶんになる. 片足を踏み出す時間だけだと左右方向にかなりずれるので、両足を踏み出す時間だけ積算したほうが良い. 1以上.
  double disturbanceCompensationStaticTime = 1.0; // 歩いていないときに、この時間で区切ってステップとする. (それをdisturbanceCompensationStepNumステップ積算する.). 0より大きい
  double disturbanceCompensationLimit = 0.05; // 長期的外乱補償の上限. 0以上

protected:
  // 内部で変更されるパラメータ. startAutoBalancer時にリセットされる
  mutable bool isInitial = true;
  mutable cnoid::Vector3 actCPPrev = cnoid::Vector3::Zero(); // generate frame.
  mutable std::list<std::pair<cnoid::Vector3, double> > disturbanceQueue = {std::pair<cnoid::Vector3, double>{cnoid::Vector3::Zero(),1.0}}; // 過去のfootstepNodesListの各ステップごとの外乱と時間
  mutable cnoid::Vector3 disturbance = cnoid::Vector3::Zero(); // generate frame. 現在のfootstepNodesListのステップの外乱
  mutable double disturbanceTime = 0.0; // 現在のfootstepNodesListのステップの時間
  mutable cnoid::Vector3 offsetPrev = cnoid::Vector3::Zero(); // generate frame. 前回の周期の長期的外乱補償の大きさ
public:
  // startAutoBalancer時に呼ばれる
  void reset() {
    isInitial = true;
    actCPPrev = cnoid::Vector3::Zero();
    disturbanceQueue = {std::pair<cnoid::Vector3, double>{cnoid::Vector3::Zero(),1.0}};
    disturbance = cnoid::Vector3::Zero();
    disturbanceTime = 0.0;
    offsetPrev = cnoid::Vector3::Zero();
  }

  bool initExternalForceHandlerOutput(const GaitParam& gaitParam, const cnoid::BodyPtr& genRobot,
                                      double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_genCog) const;

  bool handleExternalForce(const GaitParam& gaitParam, double mass, const cnoid::BodyPtr& actRobot, bool useActState, double dt,
                           double& o_omega, cnoid::Vector3& o_l, cnoid::Vector3& o_sbpOffset, cnoid::Vector3& o_actCog) const;

};

#endif
