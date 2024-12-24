#ifndef FULLBODYIKSOLVER_H
#define FULLBODYIKSOLVER_H

#include "GaitParam.h"
#include <ik_constraint2/PositionConstraint.h>
#include <ik_constraint2/COMConstraint.h>
#include <ik_constraint2/JointAngleConstraint.h>
#include <ik_constraint2/AngularMomentumConstraint.h>
#include <ik_constraint2_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <ik_constraint2/JointVelocityConstraint.h>
#include <ik_constraint2/ClientCollisionConstraint.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

class FullbodyIKSolver{
public:
  // FullbodyIKSolverでのみ使うパラメータ
  std::vector<cpp_filters::TwoPointInterpolator<double> > dqWeight; // 要素数と順序はrobot->numJoints()と同じ. 0より大きい. 各関節の変位に対する重みの比. default 1. 動かしたくない関節は大きくする. 全く動かしたくないなら、controllable_jointsを使うこと
  std::vector<cpp_filters::TwoPointInterpolator<cnoid::Vector6>> ikEEPositionWeight; // 要素数と順序はGaitParam.eeNameと同じ.
  std::vector<std::string> ikEEEvalLink; // 要素数と順序はGaitParam.eeNameと同じ.

  // FullbodyIKSolverでのみ使うパラメータ
  // 内部にヤコビアンの情報をキャッシュするが、クリアしなくても副作用はあまりない
  mutable std::vector<std::shared_ptr<ik_constraint2::PositionConstraint> > ikEEPositionConstraint; // 要素数と順序はeeNameと同じ.
  mutable std::vector<std::shared_ptr<ik_constraint2::JointAngleConstraint> > refJointAngleConstraint; // 要素数と順序はrobot->numJoints()と同じ
  mutable std::shared_ptr<ik_constraint2::PositionConstraint> rootPositionConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
  mutable std::shared_ptr<ik_constraint2::COMConstraint> comConstraint = std::make_shared<ik_constraint2::COMConstraint>();
  mutable std::shared_ptr<ik_constraint2::AngularMomentumConstraint> angularMomentumConstraint = std::make_shared<ik_constraint2::AngularMomentumConstraint>();
  mutable std::vector<std::shared_ptr<ik_constraint2_joint_limit_table::JointLimitMinMaxTableConstraint> > jointLimitConstraint;
  mutable std::vector<std::shared_ptr<ik_constraint2::JointVelocityConstraint> > jointVelocityConstraint;
  mutable std::vector<std::shared_ptr<ik_constraint2::ClientCollisionConstraint> > selfCollisionConstraint;
protected:
  // クリアしなくても副作用はあまりない
  mutable cnoid::VectorX jlim_avoid_weight;
  mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
public:
  // 初期化時に一回呼ばれる
  void init(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam){
    dqWeight.resize(genRobot->numJoints(), cpp_filters::TwoPointInterpolator<double>(1.0, 0.0, 0.0, cpp_filters::HOFFARBIB));
    ikEEPositionWeight.clear();
    for (int i=0;i<NUM_LEGS;i++) ikEEPositionWeight.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Constant(3.0), cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    for (int i=NUM_LEGS;i<gaitParam.eeName.size();i++) ikEEPositionWeight.push_back(cpp_filters::TwoPointInterpolator<cnoid::Vector6>(cnoid::Vector6::Constant(1.0), cnoid::Vector6::Zero(), cnoid::Vector6::Zero(), cpp_filters::HOFFARBIB));
    ikEEEvalLink.resize(gaitParam.eeName.size());
    ikEEPositionConstraint.clear();
    for(int i=0;i<gaitParam.eeName.size();i++) ikEEPositionConstraint.push_back(std::make_shared<ik_constraint2::PositionConstraint>());
    refJointAngleConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) refJointAngleConstraint.push_back(std::make_shared<ik_constraint2::JointAngleConstraint>());
    jointVelocityConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) jointVelocityConstraint.push_back(std::make_shared<ik_constraint2::JointVelocityConstraint>());
    jointLimitConstraint.clear();
    for(int i=0;i<genRobot->numJoints();i++) jointLimitConstraint.push_back(std::make_shared<ik_constraint2_joint_limit_table::JointLimitMinMaxTableConstraint>());
    selfCollisionConstraint.clear();
    for(int i=0;i<gaitParam.selfCollision.size();i++) selfCollisionConstraint.push_back(std::make_shared<ik_constraint2::ClientCollisionConstraint>());
  }

  // startAutoBalancer時に一回呼ばれる
  void reset(){
    for(int i=0;i<dqWeight.size();i++) dqWeight[i].reset(dqWeight[i].getGoal());
    for(int i=0;i<ikEEPositionWeight.size();i++) ikEEPositionWeight[i].reset(ikEEPositionWeight[i].getGoal());
  }

  // 毎周期呼ばれる
  void update(double dt){
    for(int i=0;i<dqWeight.size();i++) dqWeight[i].interpolate(dt);
    for(int i=0;i<ikEEPositionWeight.size();i++) ikEEPositionWeight[i].interpolate(dt);
  }

  bool solveFullbodyIK(double dt, const GaitParam& gaitParam,
                       cnoid::BodyPtr& genRobot) const;
};

#endif
