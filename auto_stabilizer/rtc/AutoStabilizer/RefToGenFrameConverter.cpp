#include "RefToGenFrameConverter.h"
#include "CnoidBodyUtil.h"
#include "MathUtil.h"

bool RefToGenFrameConverter::initGenRobot(const GaitParam& gaitParam, // input
                                          cnoid::BodyPtr& genRobot, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords, cnoid::Vector3& o_genCogVel, cnoid::Vector3& o_genCogAcc) const{ // output
  genRobot->rootLink()->T() = gaitParam.refRobotRaw->rootLink()->T();
  genRobot->rootLink()->v() = gaitParam.refRobotRaw->rootLink()->v();
  genRobot->rootLink()->w() = gaitParam.refRobotRaw->rootLink()->w();
  for(int i=0;i<genRobot->numJoints();i++){
    genRobot->joint(i)->q() = gaitParam.refRobotRaw->joint(i)->q();
    genRobot->joint(i)->dq() = gaitParam.refRobotRaw->joint(i)->dq();
    genRobot->joint(i)->u() = gaitParam.refRobotRaw->joint(i)->u();
  }
  genRobot->calcForwardKinematics();
  cnoid::Isometry3 rleg = genRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Isometry3 lleg = genRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  cnoid::Isometry3 refFootMidCoords = this->calcRefFootMidCoords(rleg, lleg, gaitParam);
  cnoid::Isometry3 footMidCoords = mathutil::orientCoordToAxis(refFootMidCoords, cnoid::Vector3::UnitZ());
  cnoidbodyutil::moveCoords(genRobot, footMidCoords, refFootMidCoords);
  genRobot->calcForwardKinematics(true);
  genRobot->calcCenterOfMass();

  cnoid::Vector3 genCogVel = cnoid::Vector3::Zero();
  cnoid::Vector3 genCogAcc = cnoid::Vector3::Zero();

  o_footMidCoords.reset(footMidCoords);
  o_genCogVel = genCogVel;
  o_genCogAcc = genCogAcc;
  return true;
}

bool RefToGenFrameConverter::convertFrame(GaitParam& gaitParam, double dt, FootStepGenerator& footStepGenerator,// input
                                          cnoid::BodyPtr& refRobot, std::vector<cnoid::Isometry3>& o_refEEPose, std::vector<cnoid::Vector6>& o_refEEWrench, double& o_refdz, cpp_filters::TwoPointInterpolatorSE3& o_footMidCoords){ // output

  /*
    静止時に、足のrefEEPoseとgenCoordsの位置がそろっていて欲しい.(refEEPoseとgenCoordsの左右の足が水平で相対位置が同じなら)
    両足支持なら、
      LegCoordsGeneratorが両足のgenCoords + copOffset(local)の中間にrefZmpTrajを生成する. refZmpTraj + l(world)の位置にgenCogが来る.
      gaitParam.footMidCoordsのYawは両足のgenCoordsの中間. Yは両足のgenCoords + copOffset(local)の中間.
      refFootMidCoordsは
        XYZ Yawは両足のrefEEPose + copOffset(local)の中間. [一致する]
        XYZ Yawはreference足のrefEEPose + copOffset(local) - defaultTranslatePos(local).
    片脚支持なら、
      LegCoordsGeneratorが支持足のgenCoords + copOffset(local)の位置にrefZmpTrajを生成する. refZmpTraj + l(world)の位置にgenCogが来る.
      gaitParam.footMidCoordsのYawは支持脚のgenCoords. Yは支持脚のgenCoords + copOffset(local) - defaultTranslatePos(horizontal).
      refFootMidCoordsは
        XYZ Yawは両足のrefEEPose + copOffset(local)の中間.
        XYZ Yawはreference脚のrefEEPose + copOffset(local) - defaultTranslatePos(local). [handFixModeなら一致する]
   */

  /*
    次の2つの座標系が一致するようにreference frameとgenerate frameを対応付ける
    - refRobotRawの、refFootOriginWeightとdefaultTranslatePosとcopOffsetに基づいて求めた足裏中間座標 (イメージとしては静止状態の目標ZMP位置にdefaultTranslatePosを作用させたもの)
    - 位置はgenRobotの重心位置 - l. 姿勢はfootMidCoords. (ただしHandFixModeなら、位置のfootMidCoords座標系Y成分はfootMidCoordsの位置.)

    startAutoBalancer直後の初回は、refRobotRawの重心位置とfootOriginのXY位置が異なる場合, refEEPoseが不連続に変化してしまう. startAutoBalancerのtransition_timeで補間してごまかす.
  */

  // 現在のFootStepNodesListから、genRobotのfootMidCoordsを求める
  cpp_filters::TwoPointInterpolatorSE3 footMidCoords = gaitParam.footMidCoords; //generate frame. gaitParam.footMidCoords. 両足の中間
  this->calcFootMidCoords(gaitParam, dt, footMidCoords); // 1周期前のfootstepNodesListを使っているが、footMidCoordsは不連続に変化するものではないのでよい
  cnoid::Isometry3 genFootMidCoords;  //generate frame. 実際に対応づけに使用する
  genFootMidCoords.linear() = footMidCoords.value().linear();
  genFootMidCoords.translation() = gaitParam.genCog - gaitParam.l; // 1周期前のlを使っているtが、lは不連続に変化するものではないので良い
  cnoid::Vector3 trans_footMidCoordsLocal = footMidCoords.value().linear().transpose() * (genFootMidCoords.translation() - footMidCoords.value().translation());
  trans_footMidCoordsLocal[1] *= (1.0 - handFixMode.value());
  genFootMidCoords.translation() = footMidCoords.value().translation() + footMidCoords.value().linear() * trans_footMidCoordsLocal;

  // refRobotRawのrefFootMidCoordsを求めてrefRobotに変換する
  double refdz;
  std::vector<cnoid::Isometry3> refEEPoseFK(gaitParam.eeName.size());
  this->convertRefRobotRaw(gaitParam, genFootMidCoords,
                           refRobot, refEEPoseFK, refdz);

  // refEEPoseRawのrefFootMidCoordsを求めて変換する
  std::vector<cnoid::Isometry3> refEEPoseWithOutFK(gaitParam.eeName.size());
  if(gaitParam.isWbmsAbsolute){
    this->convertRefEEPoseRawAbsolute(gaitParam, genFootMidCoords, refEEPoseWithOutFK);
  }else{
    this->convertRefEEPoseRawDifferential(gaitParam, dt, genFootMidCoords, refEEPoseWithOutFK, footStepGenerator);
  }

  // refEEPoseを求める
  std::vector<cnoid::Isometry3> refEEPose(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPose[i] = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{refEEPoseFK[i], refEEPoseWithOutFK[i]},
                                           std::vector<double>{this->solveFKMode.value(), 1.0 - this->solveFKMode.value()});
  }

  // refEEWrenchを計算
  std::vector<cnoid::Vector6> refEEWrench(gaitParam.eeName.size());
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEWrench[i].head<3>() = footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].head<3>();
    refEEWrench[i].tail<3>() = footMidCoords.value().linear() * gaitParam.refEEWrenchOrigin[i].tail<3>();
  }

  o_refEEPose = refEEPose;
  o_refEEWrench = refEEWrench;
  o_refdz = refdz;
  o_footMidCoords = footMidCoords;

  return true;
}

// 現在のFootStepNodesListから、genRobotのfootMidCoordsを求める (gaitParam.footMidCoords)
void RefToGenFrameConverter::calcFootMidCoords(const GaitParam& gaitParam, double dt, cpp_filters::TwoPointInterpolatorSE3& footMidCoords) const {
  //footMidCoordsを進める
  cnoid::Isometry3 rleg = gaitParam.footstepNodesList[0].dstCoords[RLEG];
  rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
  cnoid::Isometry3 lleg = gaitParam.footstepNodesList[0].dstCoords[LLEG];
  lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
  cnoid::Isometry3 midCoords = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
  rleg = mathutil::orientCoordToAxis(rleg, cnoid::Vector3::UnitZ());
  lleg = mathutil::orientCoordToAxis(lleg, cnoid::Vector3::UnitZ());
  midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
  rleg.translation() -= rleg.linear() * gaitParam.defaultTranslatePos[RLEG].value();
  lleg.translation() -= lleg.linear() * gaitParam.defaultTranslatePos[LLEG].value();
  if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 両足支持
    footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
  }else if(gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 右足支持
    if(gaitParam.footstepNodesList.size() > 1 &&
       (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      cnoid::Isometry3 rleg = gaitParam.footstepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
      cnoid::Isometry3 lleg = gaitParam.footstepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
      cnoid::Isometry3 midCoords = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime + gaitParam.footstepNodesList[1].remainTime);
    }else{
      footMidCoords.setGoal(rleg, gaitParam.footstepNodesList[0].remainTime);
    }
  }else if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]){ // 左足支持
    if(gaitParam.footstepNodesList.size() > 1 &&
       (gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) // 次が両足支持
       ){
      cnoid::Isometry3 rleg = gaitParam.footstepNodesList[1].dstCoords[RLEG];
      rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
      cnoid::Isometry3 lleg = gaitParam.footstepNodesList[1].dstCoords[LLEG];
      lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();
      cnoid::Isometry3 midCoords = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{rleg, lleg}, std::vector<double>{1.0, 1.0});
      midCoords = mathutil::orientCoordToAxis(midCoords, cnoid::Vector3::UnitZ());
      footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime + gaitParam.footstepNodesList[1].remainTime);
    }else{
      footMidCoords.setGoal(lleg, gaitParam.footstepNodesList[0].remainTime);
    }
  }else{ // 滞空期 TODO
    if(gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) { // 次が右足支持
      footMidCoords.setGoal(rleg, gaitParam.footstepNodesList[0].remainTime);
    }else if(!gaitParam.footstepNodesList[1].isSupportPhase[RLEG] && gaitParam.footstepNodesList[1].isSupportPhase[LLEG]) { // 次が左足支持
      footMidCoords.setGoal(lleg, gaitParam.footstepNodesList[0].remainTime);
    }else{
      footMidCoords.setGoal(midCoords, gaitParam.footstepNodesList[0].remainTime);
    }
  }

  footMidCoords.interpolate(dt);
}

// refRobotRawをrefRobotに変換する.
void RefToGenFrameConverter::convertRefRobotRaw(const GaitParam& gaitParam, const cnoid::Isometry3& genFootMidCoords, cnoid::BodyPtr& refRobot, std::vector<cnoid::Isometry3>& refEEPoseFK, double& refdz) const{
  cnoidbodyutil::copyRobotState(gaitParam.refRobotRaw, refRobot);
  cnoid::Isometry3 rleg = refRobot->link(gaitParam.eeParentLink[RLEG])->T()*gaitParam.eeLocalT[RLEG];
  cnoid::Isometry3 lleg = refRobot->link(gaitParam.eeParentLink[LLEG])->T()*gaitParam.eeLocalT[LLEG];
  cnoid::Isometry3 refFootMidCoords = this->calcRefFootMidCoords(rleg, lleg, gaitParam);
  refdz = (refFootMidCoords.inverse() * refRobot->centerOfMass())[2]; // ref重心高さ

  cnoidbodyutil::moveCoords(refRobot, genFootMidCoords, refFootMidCoords);
  refRobot->calcForwardKinematics();
  refRobot->calcCenterOfMass();

  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseFK[i] = refRobot->link(gaitParam.eeParentLink[i])->T() * gaitParam.eeLocalT[i];
  }
}

// refEEPoseRawを変換する.
void RefToGenFrameConverter::convertRefEEPoseRawAbsolute(const GaitParam& gaitParam, const cnoid::Isometry3& genFootMidCoords, std::vector<cnoid::Isometry3>& refEEPoseWithOutFK) const{
  cnoid::Isometry3 refFootMidCoords = this->calcRefFootMidCoords(gaitParam.refEEPoseRaw[RLEG].value(), gaitParam.refEEPoseRaw[LLEG].value(), gaitParam);
  refFootMidCoords = mathutil::orientCoordToAxis(refFootMidCoords, cnoid::Vector3::UnitZ()); // 足裏を水平になるように傾け直さずに、もとの傾きをそのまま使うことに相当
  cnoid::Isometry3 transform = genFootMidCoords * refFootMidCoords.inverse();
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseWithOutFK[i] = transform * gaitParam.refEEPoseRaw[i].value();
  }
}

void RefToGenFrameConverter::convertRefEEPoseRawDifferential(GaitParam& gaitParam, double dt, const cnoid::Isometry3& genFootMidCoords, std::vector<cnoid::Isometry3>& refEEPoseWithOutFK, FootStepGenerator& footStepGenerator){
  std::vector<cnoid::Isometry3> ref_pose;
  for(int i=0;i<gaitParam.eeName.size();i++){
    cnoid::Isometry3 tmp_ref_pose, master_rel_pose, slave_rel_pose = cnoid::Isometry3::Identity();
    master_rel_pose = gaitParam.wbmsOffsetPoseMaster[i].inverse() * gaitParam.refEEPoseRaw[i].value();
    slave_rel_pose.translation() = master_rel_pose.translation() * gaitParam.humanToRobotRatio[i];
    slave_rel_pose.linear() = master_rel_pose.linear();
    if(i<NUM_LEGS){
      tmp_ref_pose = gaitParam.wbmsOffsetPoseSlave[i] * slave_rel_pose;
    }else{ // 上半身はrootLink基準姿勢
      tmp_ref_pose = gaitParam.refRobot->rootLink()->T() * (gaitParam.wbmsOffsetPoseSlave[i] * slave_rel_pose);
    }
    ref_pose.push_back(tmp_ref_pose);
  }

  cnoid::Isometry3 refFootMidCoords = this->calcRefFootMidCoords(ref_pose[RLEG], ref_pose[LLEG], gaitParam);
  refFootMidCoords = mathutil::orientCoordToAxis(refFootMidCoords, cnoid::Vector3::UnitZ()); // 足裏を水平になるように傾け直さずに、もとの傾きをそのまま使うことに相当
  cnoid::Isometry3 transform = genFootMidCoords * refFootMidCoords.inverse();
  for(int i=0;i<gaitParam.eeName.size();i++){
    refEEPoseWithOutFK[i] = transform * ref_pose[i];
  }

  if(gaitParam.isStatic()) {
    if((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[RLEG].translation().z() - refEEPoseWithOutFK[LLEG].translation().z()) > 0.10)) {
      // 右足上げ
      std::cerr << "\x1b[31m" << "[RefToGenFrameConverter] RLEG LIFT" << "\x1b[39m" << std::endl;
      this->refFootOriginWeight[RLEG].setGoal(0.0, 0.5);
      // this->refFootOriginWeight[LLEG].setGoal(1.0, 0.5);

      std::vector<FootStepGenerator::StepNode> footsteps;
      double height = refEEPoseWithOutFK[RLEG].translation().z() - refEEPoseWithOutFK[LLEG].translation().z();
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      stepNode.stepHeight = height;
      stepNode.stepTime = 0.5;
      stepNode.swingEnd = true;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }else if((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[LLEG].translation().z() - refEEPoseWithOutFK[RLEG].translation().z()) > 0.10)) {
      // 左足上げ
      std::cerr << "\x1b[31m" << "[RefToGenFrameConverter] LLEG LIFT" << "\x1b[39m" << std::endl;
      // this->refFootOriginWeight[RLEG].setGoal(1.0, 0.5);
      this->refFootOriginWeight[LLEG].setGoal(0.0, 0.5);

      std::vector<FootStepGenerator::StepNode> footsteps;
      double height = refEEPoseWithOutFK[LLEG].translation().z() - refEEPoseWithOutFK[RLEG].translation().z();
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      stepNode.stepHeight = height;
      stepNode.stepTime = 0.5;
      stepNode.swingEnd = true;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }else if((!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[RLEG].translation().z() - refEEPoseWithOutFK[LLEG].translation().z()) > 0.04)) {
      // 右足空中
      // TODO 急激に遷移
      if(gaitParam.isManualControlMode[RLEG].getGoal() != 1.0){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[RLEG]) {
          gaitParam.isManualControlMode[RLEG].setGoal(1.0, 2.0);
        }
      }
      if(gaitParam.isManualControlMode[LLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[LLEG].setGoal(0.0, 2.0);
      }

      std::vector<FootStepGenerator::StepNode> footsteps;
      double height = refEEPoseWithOutFK[RLEG].translation().z() - refEEPoseWithOutFK[LLEG].translation().z();
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      stepNode.stepHeight = height;
      stepNode.stepTime = dt;
      stepNode.swingEnd = true;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }else if((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[LLEG].translation().z() - refEEPoseWithOutFK[RLEG].translation().z()) > 0.04)) {
      // 左足空中
      if(gaitParam.isManualControlMode[LLEG].getGoal() != 1.0){
        if(!gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) {
          gaitParam.isManualControlMode[LLEG].setGoal(1.0, 2.0);
        }
      }
      if(gaitParam.isManualControlMode[RLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[RLEG].setGoal(0.0, 2.0);
      }

      std::vector<FootStepGenerator::StepNode> footsteps;
      double height = refEEPoseWithOutFK[LLEG].translation().z() - refEEPoseWithOutFK[RLEG].translation().z();
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      stepNode.stepHeight = height;
      stepNode.stepTime = dt;
      stepNode.swingEnd = true;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }else if((!gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[RLEG].translation().z() - refEEPoseWithOutFK[LLEG].translation().z()) <= 0.04)) {
      std::cerr << "\x1b[31m" << "[RefToGenFrameConverter] RLEG LANDING" << "\x1b[39m" << std::endl;
      // 右足下げ
      if(gaitParam.isManualControlMode[RLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[RLEG].setGoal(0.0, 2.0);
      }
      if(gaitParam.isManualControlMode[LLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[LLEG].setGoal(0.0, 2.0);
      }
      this->refFootOriginWeight[RLEG].setGoal(1.0, 0.5);
      // this->refFootOriginWeight[LLEG].setGoal(1.0, 0.5);

      std::vector<FootStepGenerator::StepNode> footsteps;
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      stepNode.stepHeight = 0.04;
      stepNode.stepTime = 0.5;
      stepNode.swingEnd = false;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }else if((gaitParam.footstepNodesList[0].isSupportPhase[RLEG] && !gaitParam.footstepNodesList[0].isSupportPhase[LLEG]) && ((refEEPoseWithOutFK[LLEG].translation().z() - refEEPoseWithOutFK[RLEG].translation().z()) <= 0.04)) {
      std::cerr << "\x1b[31m" << "[RefToGenFrameConverter] LLEG LANDING" << "\x1b[39m" << std::endl;
      // 左足下げ
      if(gaitParam.isManualControlMode[LLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[LLEG].setGoal(1.0, 2.0);
      }
      if(gaitParam.isManualControlMode[RLEG].getGoal() == 1.0){
        gaitParam.isManualControlMode[RLEG].setGoal(0.0, 2.0);
      }
      // this->refFootOriginWeight[RLEG].setGoal(1.0, 0.5);
      this->refFootOriginWeight[LLEG].setGoal(1.0, 0.5);

      std::vector<FootStepGenerator::StepNode> footsteps;
      FootStepGenerator::StepNode stepNode;
      stepNode.l_r = RLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[RLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[RLEG].linear();
      footsteps.push_back(stepNode);
      stepNode.l_r = LLEG;
      stepNode.coords.translation() = refEEPoseWithOutFK[LLEG].translation();
      stepNode.coords.linear() = refEEPoseWithOutFK[LLEG].linear();
      stepNode.stepHeight = 0.04;
      stepNode.stepTime = 0.5;
      stepNode.swingEnd = false;
      footsteps.push_back(stepNode);
      footStepGenerator.setFootSteps(gaitParam, footsteps, // input
                                                   gaitParam.footstepNodesList); // output
    }
  }
}


cnoid::Isometry3 RefToGenFrameConverter::calcRefFootMidCoords(const cnoid::Isometry3& rleg_, const cnoid::Isometry3& lleg_, const GaitParam& gaitParam) const {
  cnoid::Isometry3 rleg = rleg_;
  rleg.translation() += rleg.linear() * gaitParam.copOffset[RLEG].value();
  cnoid::Isometry3 lleg = lleg_;
  lleg.translation() += lleg.linear() * gaitParam.copOffset[LLEG].value();

  cnoid::Isometry3 bothmidcoords = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{rleg, lleg},
                                                          std::vector<double>{1.0, 1.0});
  cnoid::Isometry3 rlegmidcoords = rleg;
  rlegmidcoords.translation() -= rlegmidcoords.linear() * gaitParam.defaultTranslatePos[RLEG].value();
  cnoid::Isometry3 llegmidcoords = lleg;
  llegmidcoords.translation() -= llegmidcoords.linear() * gaitParam.defaultTranslatePos[LLEG].value();

  double bothweight = std::min(this->refFootOriginWeight[RLEG].value(), this->refFootOriginWeight[LLEG].value());
  double rlegweight = this->refFootOriginWeight[RLEG].value() - bothweight;
  double llegweight = this->refFootOriginWeight[LLEG].value() - bothweight;
  return mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{bothmidcoords, rlegmidcoords, llegmidcoords},
                                 std::vector<double>{bothweight, rlegweight, llegweight});
}
