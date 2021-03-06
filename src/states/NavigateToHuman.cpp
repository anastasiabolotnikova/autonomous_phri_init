#include "NavigateToHuman.h"
#include "../PepperFSMController.h"

#include <mc_pepper/devices/TouchSensor.h>
#include <tf/transform_broadcaster.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rbdyn/rpy_utils.h>
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

  // Add VS tasks to solver or not (used for debugging the state)
  if(config_.has("addVSTasksToSolver")){
    config_("addVSTasksToSolver", addVSTasksToSolver_);
  }

  // Add mobile base PBVS task to solver
  if(!config_.has("mobileBasePBVSTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | mobileBasePBVSTask config entry missing");
  }
  mobileBasePBVSTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::PositionBasedVisServoTask>(ctl_.solver(), config_("mobileBasePBVSTask"));
  if(addVSTasksToSolver_){
    ctl_.solver().addTask(mobileBasePBVSTask_);
    // Remove default mobile base position task
    ctl_.solver().removeTask(ctl.mobileBaseTask());
  }
  // PBVS task completion criteria
  if(!config_("mobileBasePBVSTask").has("completion")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | completion config entry missing for mobileBasePBVSTask");
  }
  pbvsTaskCriteria_.configure(*mobileBasePBVSTask_, ctl_.solver().dt(), config_("mobileBasePBVSTask")("completion"));
  if(!config_("mobileBasePBVSTask")("completion").has("eval")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | eval config entry missing from PBVS task completion criteria");
  }
  pbvsTaskErrorThreshold_ = config_("mobileBasePBVSTask")("completion")("eval");

  // Translation of the mobile base target wrt visual marker frame
  if(!config_.has("targetXMarker")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | targetXMarker config entry missing");
  }
  targetXMarker_ = config_("targetXMarker");

  // After how many seconds consider vision as lost
  if(!config_.has("visionLost")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | visionLost config entry missing");
  }
  config_("visionLost", visionLost_);

  // Load IBVS task
  if(!config_.has("ibvsTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | ibvsTask config entry missing");
  }
  ibvsTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::GazeTask>(ctl_.solver(), config_("ibvsTask"));
  if(addVSTasksToSolver_){
    ctl_.solver().addTask(ibvsTask_);
  }

  // Upper back level acceptable interval
  if(!config_.has("minUpperBackLvl")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | minUpperBackLvl config entry missing");
  }
  if(!config_.has("maxUpperBackLvl")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman start | maxUpperBackLvl config entry missing");
  }
  config_("minUpperBackLvl", minUpperBackLvl_);
  config_("minUpperBackLvl", maxUpperBackLvl_);

  // Start ROS topic monitoring thread
  rosThread_ = std::thread(std::bind(&NavigateToHuman::monitorROSTopic, this));
  mc_rtc::log::info("NavigateToHuman start | ROS thread started");

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
  ctl_.logger().addLogEntry("PBVS_norm", [this]() -> const double { return mobileBasePBVSTask_->eval().norm(); });
  ctl_.logger().addLogEntry("PBVS_threshold", [this]() -> const double { return pbvsTaskErrorThreshold_; });
  ctl_.logger().addLogEntry("marker_pos", [this]() -> const Eigen::Vector3d { return markerXCamera_.translation(); });

  ctl_.logger().addLogEntry("TF_pelvis", [this]() -> const sva::PTransformd { return markerXCamera_; });
  ctl_.logger().addLogEntry("TF_head", [this]() -> const sva::PTransformd { return headXCamera_; });

  mc_rtc::log::success("NavigateToHuman state start done");
}

bool NavigateToHuman::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // State termination criteria
  if(addVSTasksToSolver_ && pbvsTaskCriteria_.completed(*mobileBasePBVSTask_)
      && !firstStateRun_ && firstROSUpdateDone_ && taskErrorUpdated_){
    mc_rtc::log::info("NavigateToHuman run | mobileBasePBVSTask eval: {}", mobileBasePBVSTask_->eval().norm());
    // Extra protection to prevent robot from moving mobile base
    if(addVSTasksToSolver_){
      // Remove PBVS task
      ctl_.solver().removeTask(mobileBasePBVSTask_);
      // Add default mobile base task back to the solver
      ctl.mobileBaseTask()->reset();
      ctl_.solver().addTask(ctl.mobileBaseTask());
      // Romeve IBVS task
      ctl_.solver().removeTask(ibvsTask_);
    }
    output("OK");
    return true;
  }

  // Updtae time for plot
  stateTime_ = stateTime_ + ctl_.solver().dt();

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
    timeWithoutROSUpdate_ = 0.0;
  }else{
    timeWithoutROSUpdate_ += ctl_.solver().dt();
  }

  if(mobileBaseStuck_){
    mc_rtc::log::warning("NavigateToHuman run | Mobile base stuck");
  }

  if(timeWithoutROSUpdate_ >= visionLost_){
    mc_rtc::log::warning("NavigateToHuman run | Vision is lost");
  }

  // Mobile base not stuck, vision not lost. Proceed normally
  if(!mobileBaseStuck_ && timeWithoutROSUpdate_ < visionLost_){
    if(!firstROSUpdateDone_){
      mc_rtc::log::warning("NavigateToHuman run | Waiting for the first ROS update");
    }else{
      // Update camera frame wrt world for visualization
      cameraXWorld_ = ctl_.robot().bodyPosW(ctl.camOpticalFrame());

      // IBVS task error update
      if(humanBodyMarkers_.find(ibvsRefFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::warning("NavigateToHuman run | Body frame for IBVS {} not found", ibvsRefFrame_);
      }else{
        headXCamera_ = humanBodyMarkers_[ibvsRefFrame_]; // for log
        ibvsTask_->error(humanBodyMarkers_[ibvsRefFrame_].translation());
      }

      // PBVS task error update
      if(humanBodyMarkers_.find(pbvsRefFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::warning("NavigateToHuman run | Body frame for PBVS {} not found", pbvsRefFrame_);
      }else{
        // Data update
        markerXCamera_ = humanBodyMarkers_[pbvsRefFrame_];
        mobilebaseXCamera_ = ctl_.robot().X_b1_b2(ctl.camOpticalFrame(), "base_link");

        // Correct markerXCamera_ rotation frame before computing targetXCamera_
        sva::PTransformd markerXWorld = markerXCamera_ * cameraXWorld_;
        Eigen::Vector3d markerX = markerXWorld.rotation().transpose().col(0);
        Eigen::Vector3d worldZ = Eigen::Vector3d(0, 0, 1);
        double humIncAng = v1v2Ang(markerX, worldZ); // TODO can still monitor if angle agrees with sitting straight assumption
        if(humIncAng != 0.0){
          // Correct marker frame inclination
          Eigen::Quaterniond quat = Eigen::Quaterniond().setFromTwoVectors(markerX, worldZ);
          markerXWorld = markerXWorld * sva::PTransformd(quat.inverse());
          markerXCamera_.rotation() = (markerXWorld * cameraXWorld_.inv()).rotation();
        }

        targetXCamera_ = targetXMarker_ * markerXCamera_;
        // Set orientation target to face the human
        targetXCamera_.rotation() = (mBaseRotTargetXWorld_ * cameraXWorld_.inv()).rotation();
        // Task error update
        mobileBasePBVSTask_->error(mobilebaseXCamera_ * targetXCamera_.inv());
        if(!taskErrorUpdated_){
          taskErrorUpdated_ = true;
          mc_rtc::log::info("NavigateToHuman run | PBVS task updated with error {}",
                              (mobilebaseXCamera_ * targetXCamera_.inv()).translation().transpose());
        }
      }

      // Record detected upper back level for other FSM states
      if(!std::isnan(ctl.humanUpperBackLevel())){
        if(humanUpperBackLevel_ > minUpperBackLvl_ && humanUpperBackLevel_ < maxUpperBackLvl_){
          ctl.humanUpperBackLevel(humanUpperBackLevel_);
        }else{
          mc_rtc::log::error("NavigateToHuman run | detected humanUpperBackLevel outside acceptable range {} < {} < {}. Will use model based value {}",
                                  minUpperBackLvl_, humanUpperBackLevel_, maxUpperBackLvl_, ctl.chairSeatHeight() + 0.3*ctl.humanHeight());
        }
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
  // Remove GUI elements
  ctl_.gui()->removePlot("Navigation to target");
  ctl_.gui()->removeCategory({"NavigateToHuman", "Frames"});
  // Remove added log entries
  ctl_.logger().removeLogEntry("PBVS_error");
  ctl_.logger().removeLogEntry("PBVS_norm");
  ctl_.logger().removeLogEntry("PBVS_threshold");
  ctl_.logger().removeLogEntry("marker_pos");
  ctl_.logger().removeLogEntry("TF_pelvis");
  ctl_.logger().removeLogEntry("TF_head");
  if(addVSTasksToSolver_){
    // Remove PBVS task
    ctl_.solver().removeTask(mobileBasePBVSTask_);
    // Add default mobile base task back to the solver
    ctl.mobileBaseTask()->reset();
    ctl_.solver().addTask(ctl.mobileBaseTask());
    // Romeve IBVS task
    ctl_.solver().removeTask(ibvsTask_);
  }
  // Stop ROS thread
  stateNeedsROS_ = false;
  rosThread_.join();
  mc_rtc::log::info("NavigateToHuman teardown done");
}

void NavigateToHuman::monitorROSTopic(){
  // Get ROS topic name
  if(!config_.has("rosTopic")){
    mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman monitorROSTopic | rosTopic config entry missing");
  }
  // Subscribe to visual marker ROS topic
  nh_ = mc_rtc::ROSBridge::get_node_handle();
  ros::Subscriber sub = nh_->subscribe(config_("rosTopic"), 100, &NavigateToHuman::updateVisualMarkerPose, this);
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
  mc_rtc::log::info("NavigateToHuman monitorROSTopic | ROS thread terminated");
}

void NavigateToHuman::updateVisualMarkerPose(const visualization_msgs::MarkerArray::ConstPtr& msg){
  // Reset marker map
  humanBodyMarkers_.clear();

  // Fill in map with new data
  for(const auto& m: msg->markers){
    // Convert marker pose in camera frame into Plucker transform
    // Quaternion is inversed as it must be expressed in successor frame (sva PTransform.h)
    humanBodyMarkers_[m.id] = sva::PTransformd(Eigen::Quaterniond(m.pose.orientation.w,
                                                                 m.pose.orientation.x,
                                                                 m.pose.orientation.y,
                                                                 m.pose.orientation.z).inverse(),
                                          Eigen::Vector3d(m.pose.position.x,
                                                          m.pose.position.y,
                                                          m.pose.position.z));
  }
  if(humanBodyMarkers_.size() != 0){
    if(!firstROSUpdateDone_){
      // Indicate that data was received at least once
      firstROSUpdateDone_ = true;
      // Check if PBVS frame exists
      if(humanBodyMarkers_.find(pbvsRefFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | Body frame for PBVS {} not found", pbvsRefFrame_);
      }
      // Human pelvis frame wrt world frame
      sva::PTransformd pelvisXCamera = humanBodyMarkers_[pbvsRefFrame_];
      sva::PTransformd pelvisXWorld = pelvisXCamera * cameraXWorld_;

      // Check if human pelvis frame inclination angle agrees with sitting straight assumption
      double maxHumanIncAng = 35.0; // deg
      if(!config_.has("maxHumanIncAng")){
        mc_rtc::log::warning("NavigateToHuman updateVisualMarkerPose | maxHumanIncAng config entry missing. Using default value: {}", maxHumanIncAng);
      }
      config_("maxHumanIncAng", maxHumanIncAng);
      Eigen::Vector3d pelvisX = pelvisXWorld.rotation().transpose().col(0);
      Eigen::Vector3d worldZ = Eigen::Vector3d(0, 0, 1);
      double humIncAng = v1v2Ang(pelvisX, worldZ);
      if(humIncAng > maxHumanIncAng){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | invalid initial detected human inclination");
      }else{
        mc_rtc::log::success("NavigateToHuman updateVisualMarkerPose | human inclination OK: {}deg", humIncAng);
      }
      // Inclination check passed
      if(humIncAng != 0.0){
        // Correct pelvis frame inclination
        Eigen::Quaterniond quat = Eigen::Quaterniond().setFromTwoVectors(pelvisX, worldZ);
        pelvisXWorld = pelvisXWorld * sva::PTransformd(quat.inverse());
      }
      // Compute mobile base orientation target in world frame
      mBaseRotTargetXWorld_ = sva::PTransformd((targetXMarker_ * pelvisXWorld).rotation());
      Eigen::Vector3d rpyToCheck = mc_rbdyn::rpyFromMat(mBaseRotTargetXWorld_.rotation());
      if(rpyToCheck(0) > 1e-3 || rpyToCheck(1) > 1e-3){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | mBaseRotTargetXWorld RPY check failed: {}",
                                                                                  rpyToCheck.transpose());
      }

      // Calculate human upper back distance from the ground
      if(humanBodyMarkers_.find(chestFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | chestFrame {} not found", chestFrame_);
      }
      if(humanBodyMarkers_.find(neckFrame_) == humanBodyMarkers_.end()){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | neckFrame {} not found", neckFrame_);
      }
      sva::PTransformd chestXWorld = humanBodyMarkers_[chestFrame_] * cameraXWorld_;
      sva::PTransformd neckXWorld = humanBodyMarkers_[neckFrame_] * cameraXWorld_;
      if(chestXWorld.translation()(2) >= neckXWorld.translation()(2)){
        mc_rtc::log::error_and_throw<std::runtime_error>("NavigateToHuman updateVisualMarkerPose | human detection failure: chest height ({}m) above or equal to neck ({}m)",
                                                                                                   chestXWorld.translation()(2), neckXWorld.translation()(2));
      }
      humanUpperBackLevel_ = (chestXWorld.translation()(2) + neckXWorld.translation()(2)) / 2;
      mc_rtc::log::info("NavigateToHuman updateVisualMarkerPose | detectedhuman upper back level from the ground: {}m",
                                                                                        humanUpperBackLevel_);
    }
    // Indicate that new data was received
    newROSData_ = true;
  }
}

double NavigateToHuman::v1v2Ang(Eigen::Vector3d v1, Eigen::Vector3d v2){
  return mc_rtc::constants::toDeg(std::acos(v1.dot(v2)/(v1.norm() * v2.norm())));
}

EXPORT_SINGLE_STATE("NavigateToHuman", NavigateToHuman)
