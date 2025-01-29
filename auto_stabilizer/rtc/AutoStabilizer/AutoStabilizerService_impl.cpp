#include "AutoStabilizerService_impl.h"
#include "AutoStabilizer.h"

AutoStabilizerService_impl::AutoStabilizerService_impl()
{
}

AutoStabilizerService_impl::~AutoStabilizerService_impl()
{
}

CORBA::Boolean AutoStabilizerService_impl::goPos( CORBA::Double x,  CORBA::Double y,  CORBA::Double th)
{
  return this->comp_->goPos(x, y, th);
};

CORBA::Boolean AutoStabilizerService_impl::goVelocity( CORBA::Double vx,  CORBA::Double vy,  CORBA::Double vth)
{
  return this->comp_->goVelocity(vx, vy, vth);
};

CORBA::Boolean AutoStabilizerService_impl::goStop()
{
  return this->comp_->goStop();
};

CORBA::Boolean AutoStabilizerService_impl::jumpTo( CORBA::Double x,  CORBA::Double y,  CORBA::Double z,  CORBA::Double ts,  CORBA::Double tf)
{
  return this->comp_->jumpTo(x, y, z, ts, tf);
};

CORBA::Boolean AutoStabilizerService_impl::setFootSteps(const auto_stabilizer::AutoStabilizerService::FootstepSequence& fs)
{
  return this->comp_->setFootSteps(fs);
}

CORBA::Boolean AutoStabilizerService_impl::setFootStepsWithParam(const auto_stabilizer::AutoStabilizerService::FootstepSequence& fs, const auto_stabilizer::AutoStabilizerService::StepParamSequence& spss)
{
  return this->comp_->setFootStepsWithParam(fs, spss);
}

void AutoStabilizerService_impl::waitFootSteps()
{
  return this->comp_->waitFootSteps();
};

CORBA::Boolean AutoStabilizerService_impl::startAutoBalancer()
{
  return this->comp_->startAutoBalancer();
};

CORBA::Boolean AutoStabilizerService_impl::stopAutoBalancer()
{
  return this->comp_->stopAutoBalancer();
};

CORBA::Boolean AutoStabilizerService_impl::startStabilizer(void)
{
  return this->comp_->startStabilizer();
}

CORBA::Boolean AutoStabilizerService_impl::stopStabilizer(void)
{
  return this->comp_->stopStabilizer();
}

CORBA::Boolean AutoStabilizerService_impl::setAutoStabilizerParam(const auto_stabilizer::AutoStabilizerService::AutoStabilizerParam& i_param)
{
  return this->comp_->setAutoStabilizerParam(i_param);
};

CORBA::Boolean AutoStabilizerService_impl::getAutoStabilizerParam(auto_stabilizer::AutoStabilizerService::AutoStabilizerParam_out i_param)
{
  i_param = new auto_stabilizer::AutoStabilizerService::AutoStabilizerParam();
  return this->comp_->getAutoStabilizerParam(*i_param);
};

CORBA::Boolean AutoStabilizerService_impl::getFootStepState(auto_stabilizer::AutoStabilizerService::FootStepState_out i_param)
{
  i_param = new auto_stabilizer::AutoStabilizerService::FootStepState();
  return this->comp_->getFootStepState(*i_param);
};

CORBA::Boolean AutoStabilizerService_impl::releaseEmergencyStop()
{
    return this->comp_->releaseEmergencyStop();
};

CORBA::Boolean AutoStabilizerService_impl::startImpedanceController(const char *i_name_)
{
  return this->comp_->startImpedanceController(i_name_);
};

CORBA::Boolean AutoStabilizerService_impl::stopImpedanceController(const char *i_name_)
{
  return this->comp_->stopImpedanceController(i_name_);
};

CORBA::Boolean AutoStabilizerService_impl::startWholeBodyMasterSlave(void)
{
  return this->comp_->startWholeBodyMasterSlave();
}

CORBA::Boolean AutoStabilizerService_impl::stopWholeBodyMasterSlave(void)
{
  return this->comp_->stopWholeBodyMasterSlave();
}

void AutoStabilizerService_impl::setComp(AutoStabilizer *i_comp)
{
  this->comp_ = i_comp;
}
