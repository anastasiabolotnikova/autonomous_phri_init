#include "NavigateToHuman.h"
#include "../PepperFSMController.h"

#include <mc_pepper/devices/TouchSensor.h>
#include <tf/transform_broadcaster.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/plot.h>

using Color = mc_rtc::gui::Color;
using Style = mc_rtc::gui::plot::Style;

void NavigateToHuman::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void NavigateToHuman::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Add mobile base PBVS task to solver
  if(!config_.has("mobileBasePBVSTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | mobileBasePBVSTask config entry missing");
  }
  mobileBasePBVSTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::PositionBasedVisServoTask>(ctl_.solver(), config_("mobileBasePBVSTask"));
  if(config_("addVSTasksToSolver", false)){
    ctl_.solver().addTask(mobileBasePBVSTask_);
    // Remove default mobile base position task
    ctl_.solver().removeTask(ctl.mobileBaseTask());
  }
  // PBVS task completion criteria
  if(!config_.has("pbvsTaskCompletion")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | pbvsTaskCompletion config entry missing");
  }
  config_("pbvsTaskCompletion", pbvsTaskCompletion_);

  // Translation of the mobile base target wrt visual marker frame
  if(!config_.has("targetXMarker")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | targetXMarker config entry missing");
  }
  targetXMarker_ = config_("targetXMarker");

  // After how many control iterations consider vision as lost
  if(!config_.has("visionLost")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | visionLost config entry missing");
  }
  config_("visionLost", visionLost_);

  // Load IBVS task
  if(!config_.has("ibvsTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | ibvsTask config entry missing");
  }
  ibvsTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::GazeTask>(ctl_.solver(), config_("ibvsTask"));
  if(config_("addVSTasksToSolver", false)){
    // Unselect neck joints from the posture task
    ctl_.getPostureTask("pepper")->selectUnactiveJoints(ctl_.solver(), {"HeadYaw", "HeadPitch"});
    ctl_.solver().addTask(ibvsTask_);
  }

  // Start ROS topic monitoring thread
  rosThread_ = std::thread(std::bind(&NavigateToHuman::monitorROSTopic, this));
  mc_rtc::log::info("ROS thread started");

  // Plot PBVS task error components
  ctl_.gui()->addPlot("Navigation to target",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("X error", [this]() { return mobileBasePBVSTask_->eval()[3]; }, Color::Red),
    mc_rtc::gui::plot::Y("Y error ", [this]() { return mobileBasePBVSTask_->eval()[4]; }, Color::Green),
    mc_rtc::gui::plot::Y("W_z error", [this]() { return mobileBasePBVSTask_->eval()[2]; }, Color::Blue)
  );

  // Update camera frame
  cameraXWorld_ = ctl_.robot().bodyPosW(ctl.camOpticalFrame());
  // Add mc_rtc::gui::Transform elements
  ctl_.gui()->addElement({"NavigateToHuman", "Frames"},
      mc_rtc::gui::Transform("marker", [this]() { return markerXCamera_ * cameraXWorld_; }),
      mc_rtc::gui::Transform("target", [this]() { return targetXCamera_ * cameraXWorld_ * sva::PTransformd(Eigen::Vector3d(0, 0, -((targetXCamera_ * cameraXWorld_).translation()[2]))); }),
      mc_rtc::gui::Transform("mBase", [this]() { return mobilebaseXCamera_ * cameraXWorld_; }),
      mc_rtc::gui::Transform("camera", [this]() { return cameraXWorld_; })
  );

  // Add PBVS log entries
  ctl_.logger().addLogEntry("PBVS_error", [this]() -> const Eigen::Vector6d { return mobileBasePBVSTask_->eval(); });
  ctl_.logger().addLogEntry("marker_pos", [this]() -> const Eigen::Vector3d { return markerXCamera_.translation(); });

  mc_rtc::log::success("NavigateToHuman state start done");
}

bool NavigateToHuman::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // State termination criteria
  if(mobileBasePBVSTask_->eval().norm() <= pbvsTaskCompletion_ && !firstStateRun_ && firstROSUpdateDone_){
    mc_rtc::log::info("mobileBasePBVSTask error: {}", mobileBasePBVSTask_->eval().norm());
    output("OK");
    return true;
  }

  // Updtae time for plot
  stateTime_ = stateTime_ + ctl_.solver().dt();

  // Update camera frame wrt world for visualization
  cameraXWorld_ = ctl_.robot().bodyPosW(ctl.camOpticalFrame());

  // Check if mobile base is stuck
  if(ctl.pepperHasBumpers()){
    for(const auto bn : ctl.bumperSensorNames()){
      auto & bumper = ctl_.robot().device<mc_pepper::TouchSensor>(bn);
      if(bumper.touch()){
        mobileBaseStuck_ = true;
      }
    }
  }

  // Keep count of control iterations without ROS update
  if(newROSData_){
    loopsWithoutROSUpdate_ = 0;
  }else{
    loopsWithoutROSUpdate_++;
  }

  if(mobileBaseStuck_){
    mc_rtc::log::warning("Mobile base stuck");
  }

  if(loopsWithoutROSUpdate_ >= visionLost_){
    mc_rtc::log::warning("Vision is lost");
  }

  // Mobile base not stuck, vision not lost. Proceed normally
  if(!mobileBaseStuck_ && loopsWithoutROSUpdate_ < visionLost_){
    if(!firstROSUpdateDone_){
      mc_rtc::log::warning("Waiting for first ROS update");
    }else{
      // IBVS task error update
      if(humanBodyMarkers_.find(ibvsRefFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::warning("Body frame for IBVS {} not found", ibvsRefFrame_);
      }else{
        ibvsTask_->error(humanBodyMarkers_[ibvsRefFrame_].translation());
      }
      // PBVS task error update
      if(humanBodyMarkers_.find(pbvsRefFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::warning("Body frame for PBVS {} not found", pbvsRefFrame_);
      }else{
        markerXCamera_ = humanBodyMarkers_[pbvsRefFrame_];
        mobilebaseXCamera_ = ctl_.robot().X_b1_b2(ctl.camOpticalFrame(), "base_link");
        targetXCamera_ = targetXMarker_ * markerXCamera_;
        targetXCamera_.rotation() = mobilebaseXCamera_.rotation();
        mobileBasePBVSTask_->error(mobilebaseXCamera_ * targetXCamera_.inv());
      }
    }
  }
  // Expect new data from ROS
  newROSData_ = false;

  // Prevent terminating the state before task eval is updated after first task error update
  if(firstStateRun_){
    firstStateRun_ = false;
  }

  return false;
}

void NavigateToHuman::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);
  // Stop ROS thread
  stateNeedsROS_ = false;
  rosThread_.join();
  // Remove GUI elements
  ctl_.gui()->removePlot("Navigation to target");
  ctl_.gui()->removeCategory({"NavigateToHuman", "Frames"});
  // Remove added log entries
  ctl_.logger().removeLogEntry("PBVS_error");
  ctl_.logger().removeLogEntry("marker_pos");
  if(config_("addVSTasksToSolver", false)){
    // Remove PBVS task
    ctl_.solver().removeTask(mobileBasePBVSTask_);
    // Add default mobile base task back to the solver
    ctl.mobileBaseTask()->reset();
    ctl_.solver().addTask(ctl.mobileBaseTask());
    // Romeve IBVS task
    ctl_.getPostureTask("pepper")->resetJointsSelector(ctl_.solver());
    ctl_.solver().removeTask(ibvsTask_);
  }
  mc_rtc::log::info("NavigateToHuman teardown done");
}

void NavigateToHuman::monitorROSTopic(){
  // Get ROS topic name
  if(!config_.has("rosTopic")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman monitorROSTopic | rosTopic config entry missing");
  }
  // Subscribe to visual marker ROS topic
  std::shared_ptr<ros::NodeHandle> nh = mc_rtc::ROSBridge::get_node_handle();
  ros::Subscriber sub = nh->subscribe(config_("rosTopic"), 100, &NavigateToHuman::updateVisualMarkerPose, this);
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

void NavigateToHuman::updateVisualMarkerPose(const visualization_msgs::MarkerArray::ConstPtr& msg){

  for(const auto& m: msg->markers){
    // Convert marker pose in camera frame into Plucker transform
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
  // Indicate that new data was received
  newROSData_ = true;
}

EXPORT_SINGLE_STATE("NavigateToHuman", NavigateToHuman)
