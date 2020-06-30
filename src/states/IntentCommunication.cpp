#include "IntentCommunication.h"
#include "../PepperFSMController.h"

#include <mc_pepper/devices/VisualDisplay.h>
#include <mc_pepper/devices/Speaker.h>
#include <tf/transform_broadcaster.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/plot.h>

using Color = mc_rtc::gui::Color;
using Style = mc_rtc::gui::plot::Style;

void IntentCommunication::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void IntentCommunication::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Set targets of right arm joints in posture task
  if(!config_.has("rightHandPointing")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | rightHandPointing config entry missing");
  }
  config_("rightHandPointing", rightHandPointing_);
  ctl_.getPostureTask("pepper")->target(rightHandPointing_);
  // Higher posture task stiffness to move faster
  ctl_.getPostureTask("pepper")->stiffness(5.0);

  // State time out
  if(!config_.has("timeOut")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | timeOut config entry missing");
  }
  config_("timeOut", timeOut_);

  // Image URL to display on the tablet
  if(!config_.has("imageToDisplay")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | imageToDisplay config entry missing");
  }
  config_("imageToDisplay", imageToDisplay_);
  // Set robot tablet screen picture
  if(ctl.pepperHasTablet()){
    auto & tablet = ctl_.robot().device<mc_pepper::VisualDisplay>("Tablet");
    tablet.display("http://198.18.0.1/apps/media/"+imageToDisplay_);
  }else{
    mc_rtc::log::warning("Cannot display '{}'. Robot has no tablet", imageToDisplay_);
  }

  // Text to say
  if(!config_.has("textToSayStart")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | textToSayStart config entry missing");
  }
  config_("textToSayStart", textToSayStart_);
  if(!config_.has("textToSayEnd")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | textToSayEnd config entry missing");
  }
  config_("textToSayEnd", textToSayEnd_);
  // Command text to play from speakers
  if(ctl.pepperHasSpeakers()){
    auto & speakers = ctl_.robot().device<mc_pepper::Speaker>("Speakers");
    speakers.say(textToSayStart_);
  }else{
    mc_rtc::log::warning("Cannot say '{}'. Robot has no speakers", textToSayStart_);
  }

  // Load IBVS task
  if(!config_.has("ibvsTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | ibvsTask config entry missing");
  }
  ibvsTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::GazeTask>(ctl_.solver(), config_("ibvsTask"));
  // Unselect neck joints from the posture task
  ctl_.getPostureTask("pepper")->selectUnactiveJoints(ctl_.solver(), {"HeadYaw", "HeadPitch"});
  ctl_.solver().addTask(ibvsTask_);

  // Right gripper oppening
  if(!config_.has("rightGripperOpening")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication start | rightGripperOpening config entry missing");
  }
  config_("rightGripperOpening", rightGripperOpening_);
  ctl_.robot().gripper("r_gripper").setTargetOpening(rightGripperOpening_);

  // Check if required to monitor human head orientation
  if(!config_.has("monitorHeadOrientation")){
    mc_rtc::log::warning("IntentCommunication start | monitorHeadOrientation config entry missing. Will use default: {}", monitorHeadOrientation_);
  }
  config_("monitorHeadOrientation", monitorHeadOrientation_);

  // Head to tablet angle threshold
  if(monitorHeadOrientation_){
    if(!config_.has("headToTabletAngleThreshold")){
      mc_rtc::log::warning("IntentCommunication start | headToTabletAngleThreshold config entry missing. Will use default: {}rad", headToTabletAngleThreshold_);
    }
    config_("headToTabletAngleThreshold", headToTabletAngleThreshold_);
  }

  // Start ROS topic monitoring in a separate thread
  rosThread_ = std::thread(std::bind(&IntentCommunication::monitorROSTopic, this));
  mc_rtc::log::info("ROS thread started"); 

  // Add GUI elements
  ctl_.gui()->addElement({"IntentCommunication", "Frames"},
      mc_rtc::gui::Transform("human head", [this]() { return humanHeadXWorld_; }),
      mc_rtc::gui::Transform("camera", [this]() { return cameraXWorld_; }),
      mc_rtc::gui::Transform("tablet", [this]() { return tabletXWorld_; }),
      mc_rtc::gui::Arrow("headTotablet", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{1., 0., 0.}),
                                    [this]() -> const Eigen::Vector3d & { return humanHeadXWorld_.translation(); },
                                    [this]() -> Eigen::Vector3d {
                                      Eigen::Vector3d end = humanHeadXWorld_.translation() + headToTablet_/4;
                                      return end;
                                    }),
      mc_rtc::gui::Arrow("headY", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color{0., 0., 1.}),
                                    [this]() -> const Eigen::Vector3d & { return humanHeadXWorld_.translation(); },
                                    [this]() -> Eigen::Vector3d {
                                      Eigen::Vector3d end = humanHeadXWorld_.translation() + humanHeadXWorld_.rotation().transpose().col(1)/4;
                                      return end;
                                    })

  );

  if(monitorHeadOrientation_){
    // Plot human to tablet angle
    ctl_.gui()->addPlot("Target face orientation",
      mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
      mc_rtc::gui::plot::Y("Angle", [this]() { return headToTabletAngle_; }, Color::Blue),
      mc_rtc::gui::plot::Y("Target", [this]() { return headToTabletAngleThreshold_; }, Color::Green, Style::Dotted)
    );

    // Add head to tablet angle to log
    ctl_.logger().addLogEntry("headToTabletAngle", [this]() -> const double { return headToTabletAngle_; });
  }

  mc_rtc::log::success("IntentCommunication state start done");
}

bool IntentCommunication::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Update camera transform
  cameraXWorld_ = ctl_.realRobot().bodyPosW(ctl.camOpticalFrame());

  // Update human head transform
  if(humanBodyMarkers_.find(ibvsFrameID_) == humanBodyMarkers_.end()){
    mc_rtc::log::warning("Body frame {} not found", ibvsFrameID_);
  }
  humanHeadXCamera_ = humanBodyMarkers_[ibvsFrameID_];

  // Update IBVS task error
  if(firstROSUpdateDone_){
    // Kepp human head in the FoV center
    ibvsTask_->error(humanHeadXCamera_.translation());
  }

  // Monitor human head, check if it was oriented to face the tablet at least once
  if(monitorHeadOrientation_ && firstROSUpdateDone_ && !humanLookedAtTablet_){
    // Human head frame in world frame
    humanHeadXWorld_ = humanHeadXCamera_ * cameraXWorld_;
    // Normalized vector from human head frame origin to robot tablet frame origin
    tabletXWorld_ = ctl_.realRobot().bodyPosW("tablet");
    headToTablet_ = tabletXWorld_.translation() - humanHeadXWorld_.translation();
    headToTablet_.normalize();
    // Compute angle between headToTablet and orientation of Y axis of head frame
    headToTabletAngle_ = acos(headToTablet_.dot(humanHeadXWorld_.rotation().transpose().col(1)));
    if(headToTabletAngle_ < headToTabletAngleThreshold_){
      humanLookedAtTablet_ =  true;
      if(ctl.pepperHasSpeakers()){
        auto & speakers = ctl_.robot().device<mc_pepper::Speaker>("Speakers");
        speakers.say(textToSayEnd_);
      }else{
        mc_rtc::log::warning("Cannot say '{}'. Robot has no speakers", textToSayEnd_);
      }
    }
  }

  // Consider communication as done after a timeout
  if(stateTime_ >= timeOut_){
    communacationDone_ = true;
  }

  // Exit the state if comunication is done and human has looked at the tablet
  if(communacationDone_ && humanLookedAtTablet_){
    // Reset tablet screen
    if(ctl.pepperHasTablet()){
      auto & tablet = ctl_.robot().device<mc_pepper::VisualDisplay>("Tablet");
      tablet.reset(true);
    }
    // Take on final  posture if defined
    if(config_.has("finalrightArmPosition")){
      ctl_.getPostureTask("pepper")->target(config_("finalrightArmPosition"));
    }
    // Set posture task stiffness back to 1.0
    ctl_.getPostureTask("pepper")->stiffness(1.0);
    output("OK");
    return true;
  }

  stateTime_ = stateTime_ + ctl_.solver().dt();
  return false;
}

void IntentCommunication::teardown(mc_control::fsm::Controller & ctl_)
{
  // Stop ROS thread
  stateNeedsROS_ = false;
  rosThread_.join();
  mc_rtc::log::info("ROS spinning thread finished, exiting the state");
  // Remove IBVS task
  ctl_.getPostureTask("pepper")->resetJointsSelector(ctl_.solver());
  ctl_.solver().removeTask(ibvsTask_);
  // Reset posture task
  ctl_.getPostureTask("pepper")->reset();
  // Remove added log entry
  ctl_.logger().removeLogEntry("headToTabletAngle");
  // Remove GUI elements
  ctl_.gui()->removeCategory({"IntentCommunication", "Frames"});
  if(monitorHeadOrientation_){
    ctl_.gui()->removePlot("Target face orientation");
  }
  mc_rtc::log::info("IntentCommunication teardown done");
}

void IntentCommunication::monitorROSTopic(){
  // Get ROS topic name
  if(!config_.has("rosTopic")){
    mc_rtc::log::error_and_throw<std::runtime_error>("IntentCommunication monitorROSTopic | rosTopic config entry missing");
  }
  // Subscribe to visual marker ROS topic
  std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
  ros::Subscriber sub = nh->subscribe(config_("rosTopic"), 100, &IntentCommunication::updateVisualMarkerPose, this);
  // Get ROS topic monitoring rate
  double rosRate = 30.0;
  if(!config_.has("rosRate")){
    mc_rtc::log::warning("NavigateToHuman monitorROSTopic | rosRate config entry missing. Will use default: {}fps", rosRate);
  }
  config_("rosRate", rosRate);
  // Monitor ROS topic messages
  ros::Rate r(rosRate);
  stateNeedsROS_ = true;
  while(stateNeedsROS_ && ros::ok()){
    ros::spinOnce();
    r.sleep();
  }
}

void IntentCommunication::updateVisualMarkerPose(const visualization_msgs::MarkerArray::ConstPtr& msg){

  for(const auto& m: msg->markers){
    humanBodyMarkers_[m.id] = sva::PTransformd(Eigen::Quaterniond(m.pose.orientation.w,
                                                                 m.pose.orientation.x,
                                                                 m.pose.orientation.y,
                                                                 m.pose.orientation.z).inverse(),
                                          Eigen::Vector3d(m.pose.position.x,
                                                          m.pose.position.y,
                                                          m.pose.position.z));
  }

  // Indicate that data was received at least once
  if(!firstROSUpdateDone_){
    firstROSUpdateDone_ = true;
  }
}

EXPORT_SINGLE_STATE("IntentCommunication", IntentCommunication)
