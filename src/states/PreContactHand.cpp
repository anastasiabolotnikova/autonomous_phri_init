#include "PreContactHand.h"
#include "../PepperFSMController.h"

#include <mc_pepper/devices/VisualDisplay.h>
#include <mc_pepper/devices/Speaker.h>
#include <mc_tasks/MetaTaskLoader.h>

void PreContactHand::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void PreContactHand::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Set target gripper opening
  if(config_.has("grippers")){
    ctl.processGrippers(config_("grippers"));
  }

  // Load camera orientation task
  if(!config_.has("lookAtTarget")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactHand start | lookAtTarget config entry missing");
  }
  lookAtTarget_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtSurfaceTask>(ctl_.solver(), config_("lookAtTarget"));
  ctl_.solver().addTask(lookAtTarget_);

  // Arm posture goals
  if(!config_.has("armPosture")){
    mc_rtc::log::error_and_throw<std::runtime_error>("PreContactHand start | armPosture config entry missing");
  }
  config_("armPosture", armPostureGoal_);
  // Set first goal as target
  ctl_.getPostureTask("pepper")->target(armPostureGoal_);
  postureGoalJoints_ = ctl.mapKeys(armPostureGoal_);

  if(!config_.has("jointNearTargetDelta")){
    mc_rtc::log::warning("PreContactHand start | jointNearTargetDelta config entry missing. Using default value: {}", delta_);
  }
  config_("jointNearTargetDelta", delta_);

  // Text to say
  if(!config_.has("textToSay")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | textToSay config entry missing");
  }
  config_("textToSay", textToSay_);
  // Image URL to display on the tablet
  if(!config_.has("imageToDisplay")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | imageToDisplay config entry missing");
  }
  config_("imageToDisplay", imageToDisplay_);

  mc_rtc::log::info("PreContactHand start done");
}

bool PreContactHand::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  if(ctl.jointsNearTarget("pepper", postureGoalJoints_, delta_)){
    output("OK");
    return true;
  }

  return false;
}

void PreContactHand::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);
  // Remove added tasks
  ctl_.solver().removeTask(lookAtTarget_);

  // Command text to play from speakers
  if(ctl.pepperHasSpeakers()){
    auto & speakers = ctl_.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSay_);
  }else{
    mc_rtc::log::warning("PreContactHand teardown | Cannot say '{}'. Robot has no speakers", textToSay_);
  }
  // Set robot tablet screen picture
  if(ctl.pepperHasTablet()){
    auto & tablet = ctl_.robot().device<mc_pepper::VisualDisplay>("Tablet");
    tablet.display("http://198.18.0.1/apps/media/"+imageToDisplay_);
  }else{
    mc_rtc::log::warning("Cannot display '{}'. Robot has no tablet", imageToDisplay_);
  }

  mc_rtc::log::info("PreContactHand teardown done");
}

EXPORT_SINGLE_STATE("PreContactHand", PreContactHand)
