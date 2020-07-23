#include "RemoveContacts.h"
#include "../PepperFSMController.h"

#include <mc_pepper/devices/Speaker.h>
#include <mc_tasks/MetaTaskLoader.h>

void RemoveContacts::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void RemoveContacts::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Set target gripper opening
  if(config_.has("grippersStart")){
    ctl.processGrippers(config_("grippersStart"));
  }

  // Posture goals
  if(!config_.has("sequentialPostureGoals")){
    mc_rtc::log::error_and_throw<std::runtime_error>("RemoveContacts start | sequentialPostureGoals config entry missing");
  }
  config_("sequentialPostureGoals", sequentialPostureGoals_);
  if(!config_.has("jointNearTargetDelta")){
    mc_rtc::log::warning("RemoveContacts start | jointNearTargetDelta config entry missing. Using default value: {}", delta_);
  }
  config_("jointNearTargetDelta", delta_);

  // Set first goal as target
  ctl_.getPostureTask("pepper")->target(sequentialPostureGoals_[currentPostureGoal_]);
  ctl_.getPostureTask("pepper")->stiffness(3.0);
  currentPostureGoalJoints_ = ctl.mapKeys(sequentialPostureGoals_[currentPostureGoal_]);

  // Text to say
  if(!config_.has("textToSayStart")){
    mc_rtc::log::error_and_throw<std::runtime_error>("RemoveContacts start | textToSayStart config entry missing");
  }
  config_("textToSayStart", textToSayStart_);
  if(!config_.has("textToSayEnd")){
    mc_rtc::log::error_and_throw<std::runtime_error>("RemoveContacts start | textToSayEnd config entry missing");
  }
  config_("textToSayEnd", textToSayEnd_);

  // Command text to play from speakers
  if(ctl.pepperHasSpeakers()){
    auto & speakers = ctl_.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSayStart_);
  }else{
    mc_rtc::log::warning("RemoveContacts start | Cannot say '{}'. Robot has no speakers", textToSayStart_);
  }

  mc_rtc::log::info("RemoveContacts start done");
}

bool RemoveContacts::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Check if current posture goal is reached
  if(ctl.jointsNearTarget("pepper", currentPostureGoalJoints_, delta_)){
    // When reached set next goal if exists
    currentPostureGoal_++;
    mc_rtc::log::info("RemoveContacts run | Posture goal {} reached", currentPostureGoal_);
    if(currentPostureGoal_ < sequentialPostureGoals_.size()){
      ctl_.getPostureTask("pepper")->target(sequentialPostureGoals_[currentPostureGoal_]);
      currentPostureGoalJoints_ = ctl.mapKeys(sequentialPostureGoals_[currentPostureGoal_]);
    }else{
      // Exit state when all goals are reached
      mc_rtc::log::info("RemoveContacts run | All posture goals reached");
      output("OK");
      return true;
    }
  }
  return false;
}

void RemoveContacts::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);
  // Set target gripper opening
  if(config_.has("grippersEnd")){
    ctl.processGrippers(config_("grippersEnd"));
  }
  ctl_.getPostureTask("pepper")->stiffness(1.0);
  // Command text to play from speakers
  if(ctl.pepperHasSpeakers()){
    auto & speakers = ctl_.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSayEnd_);
  }else{
    mc_rtc::log::warning("RemoveContacts start | Cannot say '{}'. Robot has no speakers", textToSayEnd_);
  }
  mc_rtc::log::info("RemoveContacts teardown done");
}

EXPORT_SINGLE_STATE("RemoveContacts", RemoveContacts)
