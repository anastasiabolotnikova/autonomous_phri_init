#include "PreContactBack.h"
#include "../PepperFSMController.h"

#include <mc_tasks/MetaTaskLoader.h>

void PreContactBack::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void PreContactBack::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Set target gripper opening
  if(config_.has("grippers")){
    ctl.processGrippers(config_("grippers"));
  }

  // Load camera orientation task
  if(!config_.has("lookAtTarget")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | lookAtTarget config entry missing");
  }
  lookAtTarget_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtSurfaceTask>(ctl_.solver(), config_("lookAtTarget"));
  ctl_.solver().addTask(lookAtTarget_);

  // Posture goals
  if(!config_.has("sequentialPostureGoals")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | sequentialPostureGoals config entry missing");
  }
  config_("sequentialPostureGoals", sequentialPostureGoals_);
  if(!config_.has("jointNearTargetDelta")){
    mc_rtc::log::warning("PreContactBack start | jointNearTargetDelta config entry missing. Using default value: {}", delta_);
  }
  config_("jointNearTargetDelta", delta_);

  // Set first goal as target
  ctl_.getPostureTask("pepper")->target(sequentialPostureGoals_[currentPostureGoal_]);
  ctl_.getPostureTask("pepper")->stiffness(3.0);
  currentPostureGoalJoints_ = ctl.mapKeys(sequentialPostureGoals_[currentPostureGoal_]);

  mc_rtc::log::info("PreContactBack start done");
}

bool PreContactBack::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Check if current posture goal is reached
  if(ctl.jointsNearTarget("pepper", currentPostureGoalJoints_, delta_)){
    // When reached set next goal if exists
    currentPostureGoal_++;
    mc_rtc::log::info("PreContactBack run | Posture goal {} reached", currentPostureGoal_);
    if(currentPostureGoal_ < sequentialPostureGoals_.size()){
      ctl_.getPostureTask("pepper")->target(sequentialPostureGoals_[currentPostureGoal_]);
      currentPostureGoalJoints_ = ctl.mapKeys(sequentialPostureGoals_[currentPostureGoal_]);
    }else{
      // Exit state when all goals are reached
      mc_rtc::log::info("PreContactBack run | All posture goals reached");
      output("OK");
      return true;
    }
  }
  return false;
}

void PreContactBack::teardown(mc_control::fsm::Controller & ctl_)
{
  // Remove added tasks
  ctl_.solver().removeTask(lookAtTarget_);
  ctl_.getPostureTask("pepper")->stiffness(1.0);
  mc_rtc::log::info("PreContactBack teardown done");
}

EXPORT_SINGLE_STATE("PreContactBack", PreContactBack)
