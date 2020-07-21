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

  // Load hand trajectory task and completion criteria
  if(!config_.has("handTrajectoryTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | handTrajectoryTask config entry missing");
  }
  handTrajectoryTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::BSplineTrajectoryTask>(ctl_.solver(), config_("handTrajectoryTask"));
  if(!config_("handTrajectoryTask").has("completion")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | completion config entry missing for handTrajectoryTask");
  }
  trajTaskCriteria_.configure(*handTrajectoryTask_, ctl_.solver().dt(), config_("handTrajectoryTask")("completion"));

  // Adjust trajectory task bspline end point Z axis value to the human height
  Eigen::Vector3d target = handTrajectoryTask_->spline().target();
  if(!std::isnan(ctl.humanUpperBackLevel())){
    // Perception based
    target(2) = ctl.humanUpperBackLevel();
  }else{
    // Model based
    target(2) = ctl.chairSeatHeight() + 0.3*ctl.humanHeight();
  }
  handTrajectoryTask_->spline().target(target);

  // Add hand trajectory task to the solver
  ctl_.solver().addTask(handTrajectoryTask_);

  // Load camera orientation task
  if(!config_.has("lookAtHandTarget")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactBack start | lookAtHandTarget config entry missing");
  }
  lookAtHandTarget_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtTask>(ctl_.solver(), config_("lookAtHandTarget"));
  if(!config_("lookAtHandTarget").has("targetPos")){
    // Look at (adjusted) hand trajectory task target
    lookAtHandTarget_->target(target); // TODO check that this target is expressed in world frame
  }
  ctl_.solver().addTask(lookAtHandTarget_);
}

bool PreContactBack::run(mc_control::fsm::Controller & ctl_)
{
  // State termination criteria
  if(trajTaskCriteria_.completed(*handTrajectoryTask_)){
    output("OK");
    return true;
  }
  return false;
}

void PreContactBack::teardown(mc_control::fsm::Controller & ctl_)
{
  // Remove added tasks
  ctl_.solver().removeTask(lookAtHandTarget_);
  ctl_.solver().removeTask(handTrajectoryTask_);
}

EXPORT_SINGLE_STATE("PreContactBack", PreContactBack)
