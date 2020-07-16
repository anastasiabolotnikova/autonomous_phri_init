#include "MakeContactBack.h"
#include "../PepperFSMController.h"

#include <mc_tasks/MetaTaskLoader.h>

void MakeContactBack::configure(const mc_rtc::Configuration & config)
{
  // Read state configuration
  config("moveInward", moveInward_);
  config("inContactDuration", inContactDuration_);
  // Load state config
  config_.load(config);
}

void MakeContactBack::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Load hand task
  if(!config_.has("handTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | handTask config entry missing");
  }
  handTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::RelativeEndEffectorTask>(ctl.solver(), config_("handTask"));
  // Set hand task target to move inward from the current position
  handTask_->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0, moveInward_, 0)));
  ctl.solver().addTask(handTask_);
}

bool MakeContactBack::run(mc_control::fsm::Controller & ctl_)
{
  return false;
}

void MakeContactBack::teardown(mc_control::fsm::Controller &)
{
}

EXPORT_SINGLE_STATE("MakeContactBack", MakeContactBack)
