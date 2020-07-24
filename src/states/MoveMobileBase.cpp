#include "MoveMobileBase.h"

#include "../PepperFSMController.h"

void MoveMobileBase::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void MoveMobileBase::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  if(!config_.has("taskCompletion")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MoveMobileBase | taskCompletion config entry missing");
  }
  config_("taskCompletion", taskCompletion_);

  if(!config_.has("sequentialTargets")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MoveMobileBase | sequentialTargets config entry missing");
  }
  config_("sequentialTargets", sequentialTargets_);

  // Compute new position with the added increment
  absTarget_ = sequentialTargets_[targetCnt_] * ctl_.robot().posW();
  ctl.mobileBaseTask()->set_ef_pose(absTarget_);
}

bool MoveMobileBase::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  if(ctl.mobileBaseTask()->eval().norm() < taskCompletion_){
    targetCnt_++;
    mc_rtc::log::info("MoveMobileBase run | Mobile base movement goal {} reached", targetCnt_);
    if(targetCnt_ < sequentialTargets_.size()){
      absTarget_ = sequentialTargets_[targetCnt_] * ctl_.robot().posW();
      ctl.mobileBaseTask()->set_ef_pose(absTarget_);
    }else{
      // Exit state when all goals are reached
      output("OK");
      return true;
    }
  }
  return false;
}

void MoveMobileBase::teardown(mc_control::fsm::Controller &)
{
  mc_rtc::log::info("MoveMobileBase teardown done");
}

EXPORT_SINGLE_STATE("MoveMobileBase", MoveMobileBase)
