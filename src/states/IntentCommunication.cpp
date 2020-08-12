#include "IntentCommunication.h"
#include "../PepperFSMController.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/plot.h>
#include <mc_pepper/devices/VisualDisplay.h>
#include <mc_pepper/devices/Speaker.h>

using Color = mc_rtc::gui::Color;
using Style = mc_rtc::gui::plot::Style;

void IntentCommunication::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void IntentCommunication::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_); // is it needed?

  // Set targets of right arm joints in posture task
  ctl.getPostureTask("pepper")->reset();
  if(!config_.has("rightHandPointing")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | rightHandPointing config entry missing");
  }
  rightHandPointing_ = config_("rightHandPointing");
  ctl.getPostureTask("pepper")->target(rightHandPointing_);
  ctl.getPostureTask("pepper")->stiffness(5.0);

  if(!config_.has("timeOut")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | timeOut config entry missing");
  }
  timeOut = config_("timeOut");

  if(!config_.has("imageToDisplay")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | imageToDisplay config entry missing");
  }
  config_("imageToDisplay", imageToDisplay_);

  if(!config_.has("textToSayStart")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | textToSayStart config entry missing");
  }
  config_("textToSayStart", textToSayStart_);

  if(!config_.has("textToSayEnd")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | textToSayEnd config entry missing");
  }
  config_("textToSayEnd", textToSayEnd_);

  // Unselect neck joints for the posture task
  ctl.getPostureTask("pepper")->selectUnactiveJoints(ctl.solver(), {"HeadYaw", "HeadPitch"});
  // Add camera orientation task
  if(!config_.has("lookAtTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | lookAtTask config entry missing");
  }
  lookAtTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtTask>(ctl.solver(), config_("lookAtTask"));
  ctl.solver().addTask(lookAtTask_);

  // Set robot tablet screen picture
  if(ctl.pepperHasTablet()){
    auto & tablet = ctl.robot().device<mc_pepper::VisualDisplay>("Tablet");
    tablet.display("http://198.18.0.1/apps/media/"+imageToDisplay_);
  }

  // Command text to play from speakers
  if(ctl.pepperHasSpeakers()){
    auto & speakers = ctl.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSayStart_);
  }

  // Set right gripper opening percentage
  ctl.robot().gripper("r_gripper").setTargetOpening(0.8);

  // Set human head orientation
  if(config_.has("lookAtTaskHuman")){
    lookAtTaskHuman_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtTask>(ctl.solver(), config_("lookAtTaskHuman"));
    lookAtTaskHuman_->target(ctl.robots().robot("pepper").bodyPosW("tablet").translation());
    ctl.solver().addTask(lookAtTaskHuman_);
  }else{
    mc_rtc::log::warning("IntentCommunication | lookAtTaskHuman config entry missing");
  }

  // Head to tablet initial angle
  humanHead_X_world = ctl.robots().robot("human").bodyPosW("HeadLink");
  head_to_tablet = ctl.robot().bodyPosW("tablet").translation() - humanHead_X_world.translation();
  head_to_tablet.normalize();
  // Angle between head_to_tablet and orientation of X axis of human head frame
  head_to_tablet_angle = acos(head_to_tablet.dot(humanHead_X_world.rotation().row(0)));

  if(!config_.has("head_to_tablet_angle_threshold")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | head_to_tablet_angle_threshold_ config entry missing");
  }
  config_("head_to_tablet_angle_threshold", head_to_tablet_angle_threshold_);

  // Add GUI elements
  ctl.gui()->addElement({"IntentCommunication", "Frames"},
      mc_rtc::gui::Transform("human head", [this]() { return humanHead_X_world; }),
      mc_rtc::gui::Transform("camera", [this]() { return camera_X_world; }),
      mc_rtc::gui::Transform("tablet", [this]() { return tablet_X_world; }),
      mc_rtc::gui::Arrow("head_to_tablet", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}),
                                    [this]() -> const Eigen::Vector3d & { return humanHead_X_world.translation(); },
                                    [this]() -> Eigen::Vector3d {
                                      Eigen::Vector3d end = humanHead_X_world.translation() + head_to_tablet/4;
                                      return end;
                                    }),
      mc_rtc::gui::Arrow("head_Y", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{0., 0., 1.}),
                                    [this]() -> const Eigen::Vector3d & { return humanHead_X_world.translation(); },
                                    [this]() -> Eigen::Vector3d {
                                      Eigen::Vector3d end = humanHead_X_world.translation() + humanHead_X_world.rotation().transpose().col(1)/4;
                                      return end;
                                    })
  );

  // Plot human to tablet orientation angle
  ctl.gui()->addPlot("Target face orientation",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime; }),
    mc_rtc::gui::plot::Y("Angle", [this]() { return head_to_tablet_angle * 180.0/M_PI; }, Color::Blue),
    mc_rtc::gui::plot::Y("Threshold", [this]() { return head_to_tablet_angle_threshold_; }, Color::Red, Style::Dotted)
  );

  // Add head to tablet angle to log
  ctl.logger().addLogEntry("head_to_tablet_angle", [this]() -> const double { return head_to_tablet_angle; });

  mc_rtc::log::success("IntentCommunication state start done");
}

bool IntentCommunication::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Update camera position
  camera_X_world = ctl.robot().bodyPosW(ctl.camOpticalFrame());
  humanHead_X_world = ctl.robots().robot("human").bodyPosW("HeadLink");

  lookAtTask_->target(humanHead_X_world.translation());

  // Check that human looked at the tablet at least once
  // Normalized vector from human head frame origin to robot tablet frame origin
  head_to_tablet = ctl.robot().bodyPosW("tablet").translation() - humanHead_X_world.translation();
  head_to_tablet.normalize();
  // Angle between head_to_tablet and orientation of X axis of human head frame
  head_to_tablet_angle = acos(head_to_tablet.dot(humanHead_X_world.rotation().row(0)));
  if(head_to_tablet_angle < head_to_tablet_angle_threshold_ * M_PI/180){
    humanLookedAtTablet =  true;
    auto & speakers = ctl.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSayEnd_);
  }

  // Exit the state based on the timeout
  if(stateTime >= timeOut){
    communacationDone_ = true;
  }

  if(communacationDone_ && humanLookedAtTablet){
    // Reset tablet screen
    auto & tablet = ctl.robot().device<mc_pepper::VisualDisplay>("Tablet");
    tablet.reset(true);
    // Take on final posture for this state if defined
    if(config_.has("finalHandPosition")){
      ctl.getPostureTask("pepper")->target(config_("finalHandPosition"));
    }
    // Set posture task stiffness back to 1.0
    ctl.getPostureTask("pepper")->stiffness(1.0);
    output("OK");
    return true;
  }

  stateTime = stateTime + ctl.solver().dt();
  return false;
}

void IntentCommunication::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Remove camera orientation task
  ctl.getPostureTask("pepper")->resetJointsSelector(ctl.solver());
  ctl.solver().removeTask(lookAtTask_);
  ctl.solver().removeTask(lookAtTaskHuman_);

  // Reset posture task
  ctl.getPostureTask("pepper")->reset();
  ctl.gui()->removeCategory({"IntentCommunication", "Frames"});
}

EXPORT_SINGLE_STATE("IntentCommunication", IntentCommunication)
