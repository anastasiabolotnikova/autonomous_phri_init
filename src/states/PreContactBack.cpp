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

  // Arm posture goals
  if(!config_.has("armPosture1")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | armPosture1 config entry missing");
  }
  config_("armPosture1", armPostureGoal1_);
  if(!config_.has("armPosture2")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | armPosture2 config entry missing");
  }
  config_("armPosture2", armPostureGoal2_);
  if(!config_.has("jointNearTargetDelta")){
    mc_rtc::log::warning("PreContactBack start | jointNearTargetDelta config entry missing. Using default value: {}", delta_);
  }
  config_("jointNearTargetDelta", delta_);

  // Set first goal as target
  ctl_.getPostureTask("pepper")->target(armPostureGoal1_);

  mc_rtc::log::info("PreContactBack start done");
}

bool PreContactBack::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  if(ctl.jointNearTarget("pepper", "LShoulderRoll", delta_) && ctl.jointNearTarget("pepper", "LElbowRoll", delta_) && !goal1Reached_){
    goal1Reached_ = true;
    ctl_.getPostureTask("pepper")->target(armPostureGoal2_);
    mc_rtc::log::success("PreContactBack run | First posture goal reached");
  }else{
    if(ctl.jointNearTarget("pepper", "LShoulderPitch", delta_) && goal1Reached_){
      if(!goal2Reached_){
        goal2Reached_ = true;
        mc_rtc::log::success("PreContactBack run | Second posture goal reached");
      }
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
  mc_rtc::log::info("PreContactBack teardown done");
}

EXPORT_SINGLE_STATE("PreContactBack", PreContactBack)
