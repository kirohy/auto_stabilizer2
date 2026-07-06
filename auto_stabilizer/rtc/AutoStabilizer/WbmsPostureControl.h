#ifndef WBMSPOSTURECONTROL_H
#define WBMSPOSTURECONTROL_H

#include "GaitParam.h"
#include <cpp_filters/TwoPointInterpolator.h>
#include <ik_constraint2/PositionConstraint.h>
#include <ik_constraint2/COMConstraint.h>
#include <ik_constraint2/JointAngleConstraint.h>
#include <ik_constraint2/JointVelocityConstraint.h>
#include <ik_constraint2/ClientCollisionConstraint.h>
#include <ik_constraint2_joint_limit_table/JointLimitMinMaxTableConstraint.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <limits>

class WbmsPostureControl{
public:
  void init(const cnoid::BodyPtr& genRobot, const GaitParam& gaitParam);
  void reset();
  void start(GaitParam& gaitParam);
  void clearStaleCommand(GaitParam& gaitParam, bool resetApplied) const;
  void updateVelocityCommand(GaitParam& gaitParam, double dt, bool isABCRunning) const;
  void applyWalkingComHeightHoldToReference(GaitParam& gaitParam) const;
  void applyWalkingComHeightHoldToGenCog(GaitParam& gaitParam, double dt) const;
  void updateWalkingPreparationReadiness(GaitParam& gaitParam, double dt) const;
  void proc(GaitParam& gaitParam, double dt, bool isABCRunning, const std::vector<cpp_filters::TwoPointInterpolator<double> >& referenceDqWeight);

  size_t variableJointNum() const { return this->projectionJointIds_.size(); }
  const std::vector<int>& variableJointIds() const { return this->projectionJointIds_; }

private:
  struct ProjectionValidationResult {
    bool safe = false;
    GaitParam::WbmsProjectionStatus status = GaitParam::WBMS_PROJECTION_NOT_RUN;
    double rootTranslationStep = 0.0;
    double rootRotationStep = 0.0;
    double maxJointStep = 0.0;
    double minJointLimitMargin = std::numeric_limits<double>::max();
    double maxFootPositionError = 0.0;
    double maxFootRotationError = 0.0;
  };

  cnoid::BodyPtr wbmsPostureRobot_;
  cpp_filters::TwoPointInterpolator<double> wbmsWalkingStabilityMode_ = cpp_filters::TwoPointInterpolator<double>(0.0,0.0,0.0,cpp_filters::HOFFARBIB);
  std::vector<int> projectionJointIds_;
  std::vector<cnoid::LinkPtr> projectionVariables_;
  std::vector<std::shared_ptr<ik_constraint2::JointVelocityConstraint> > jointVelocityConstraint_;
  std::vector<std::shared_ptr<ik_constraint2_joint_limit_table::JointLimitMinMaxTableConstraint> > jointLimitConstraint_;
  std::vector<std::shared_ptr<ik_constraint2::ClientCollisionConstraint> > selfCollisionConstraint_;
  std::vector<std::shared_ptr<ik_constraint2::PositionConstraint> > footConstraint_;
  std::shared_ptr<ik_constraint2::PositionConstraint> chestConstraint_ = std::make_shared<ik_constraint2::PositionConstraint>();
  std::shared_ptr<ik_constraint2::COMConstraint> comConstraint_ = std::make_shared<ik_constraint2::COMConstraint>();
  std::vector<std::shared_ptr<ik_constraint2::JointAngleConstraint> > postureReferenceConstraint_;
  std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > projectionConstraints_;
  std::vector<std::shared_ptr<prioritized_qp_base::Task> > projectionTasks_;
  prioritized_inverse_kinematics_solver2::IKParam projectionIKParam_;
  std::vector<cnoid::Vector3> supportVerticesInFootMid_;
  std::vector<cnoid::Vector3> supportHullTmp_;
  std::vector<cnoid::Vector3> supportHullInFootMid_;
  std::vector<cnoid::Vector3> shrunkSupportHullInFootMid_;
  std::vector<cnoid::Vector3> shrunkSupportHullInWorld_;
  std::vector<cnoid::Vector3> shrinkShiftedPoints_;
  std::vector<cnoid::Vector3> shrinkShiftedDirs_;

  bool isVelocityCommandAllowed(const GaitParam& gaitParam, bool isABCRunning) const;
  bool isPostureProjectionAllowed(const GaitParam& gaitParam, bool isABCRunning) const;
  bool isStaticComZmpIntegrationAllowed(const GaitParam& gaitParam, bool isABCRunning) const;
  bool isWalkingComHeightHoldAllowed(const GaitParam& gaitParam) const;
  bool isWalkingPreparationActive(const GaitParam& gaitParam) const;
  bool isWalkingPreparationReturningTargetActive(const GaitParam& gaitParam) const;
  cnoid::Vector3 applyAccelerationLimit(const cnoid::Vector3& current, const cnoid::Vector3& desired, const cnoid::Vector3& limit, double dt) const;
  cnoid::Matrix3 interpolateRotation(const cnoid::Matrix3& from, const cnoid::Matrix3& to, double alpha) const;
  cnoid::Vector3 calcVelocityLimitedStep(const cnoid::Vector3& error, const cnoid::Vector3& currentVelocity, const cnoid::Vector3& velocityLimit, const cnoid::Vector3& accelerationLimit, double dt, cnoid::Vector3& nextVelocity) const;
  bool updateWalkingPreparationReturnTarget(GaitParam& gaitParam, double dt) const;
  void setWalkingPreparationPhase(GaitParam& gaitParam, GaitParam::WbmsWalkingPreparationPhase phase) const;
  void failWalkingPreparation(GaitParam& gaitParam, GaitParam::WbmsWalkingPreparationFailureCode code) const;
  bool latchWalkingPreparationReturnStart(GaitParam& gaitParam) const;
  void addAncestorJointIds(const cnoid::LinkPtr& link, const cnoid::BodyPtr& robot, std::vector<bool>& jointUsed);
  void syncProjectionRobot(const GaitParam& gaitParam);
  bool updateSupportHull(const GaitParam& gaitParam);
  bool calcProjectionTargets(GaitParam& gaitParam, double dt, cnoid::Matrix3& targetChestR, cnoid::Vector3& targetRobotCom, cnoid::Vector3& currentComInFootMid, cnoid::Matrix3& currentChestRInFootMid, bool& supportHullValid);
  bool solveProjection(GaitParam& gaitParam, double dt, const std::vector<cpp_filters::TwoPointInterpolator<double> >& referenceDqWeight);
  ProjectionValidationResult validateProjectionCandidate(const GaitParam& gaitParam, double dt) const;
  void storeProjectionValidationResult(GaitParam& gaitParam, const ProjectionValidationResult& result) const;
  void setFallbackReference(GaitParam& gaitParam) const;
  void applyStaticComZmpIntegration(GaitParam& gaitParam, double dt);
};

#endif
