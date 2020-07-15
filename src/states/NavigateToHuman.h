#pragma once

#include <mc_tasks/PositionBasedVisServoTask.h>
#include "visualization_msgs/MarkerArray.h"
#include <mc_control/fsm/State.h>
#include <mc_tasks/GazeTask.h>
#include <mc_rtc/ros.h>
#include <thread>


struct NavigateToHuman : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    // ROS topic monitor function
    void monitorROSTopic();

    // ROS topic subscriber callback
    void updateVisualMarkerPose(const visualization_msgs::MarkerArray::ConstPtr& msg);

    // Compute angle between two 3D vectors
    double v1v2Ang(Eigen::Vector3d v1, Eigen::Vector3d v2);
    // Mobile base rotation target in world frame
    sva::PTransformd mBaseRotTargetXWorld_;

    // Full state configuration
    mc_rtc::Configuration config_;

    // Time for plot
    double stateTime_ = 0.0;

    // Indicate if state run function is called for the first time
    bool firstStateRun_ = true;

    // PBVS task for mobile base navigation
    std::shared_ptr<mc_tasks::PositionBasedVisServoTask> mobileBasePBVSTask_;
    // Task completion threshold
    double pbvsTaskCompletion_;

    // Desired mobilebase taget position w.r.t to the visual marker
    sva::PTransformd targetXMarker_ = sva::PTransformd::Identity();

    // Camera orientation task
    std::shared_ptr<mc_tasks::GazeTask> ibvsTask_;

    // ROS spinning thread
    std::thread rosThread_;
    // ROS thread is kept alive while stateNeedsROS_ is true
    bool stateNeedsROS_;
    // Start the state after first ROS update
    bool firstROSUpdateDone_ = false;
    // Track if data from ROS topic has been updated
    bool newROSData_ = false;

    // Consider vision lost if no new marker data recieved in last visionLost_ iterations
    unsigned int visionLost_;

    // Map marker id to frame transform
    std::map<unsigned int, sva::PTransformd> humanBodyMarkers_;

    // Monitor if contact with mobile base is detected through bumpers
    bool mobileBaseStuck_ = false;

    // Number of control iterations without new ROS data
    unsigned int loopsWithoutROSUpdate_ = 0;

    // Human body parts
    typedef enum
    {
      PELVIS,
      NAVEL,
      SPINE_CHEST,
      NECK,
      CLAVICLE_LEFT,
      SHOULDER_LEFT,
      ELBOW_LEFT,
      WRIST_LEFT,
      HAND_LEFT,
      HANDTIP_LEFT,
      THUMB_LEFT,
      CLAVICLE_RIGHT,
      SHOULDER_RIGHT,
      ELBOW_RIGHT,
      WRIST_RIGHT,
      HAND_RIGHT,
      HANDTIP_RIGHT,
      THUMB_RIGHT,
      HIP_LEFT,
      KNEE_LEFT,
      ANKLE_LEFT,
      FOOT_LEFT,
      HIP_RIGHT,
      KNEE_RIGHT,
      ANKLE_RIGHT,
      FOOT_RIGHT,
      HEAD,
      NOSE,
      EYE_LEFT,
      EAR_LEFT,
      EYE_RIGHT,
      EAR_RIGHT,
      TOTAL_HUMAN_FRAMES
    } HumanBodyFrame;

    // Consider first human detection result
    int firstHumanID_ = 100;

    // Human body frame to use for PBVS
    int pbvsRefFrame_ = firstHumanID_ + PELVIS;

    // Human body frame to use for IBVS
    int ibvsRefFrame_ = firstHumanID_ + HEAD;

    // Camera w.r.t world frame
    sva::PTransformd cameraXWorld_ = sva::PTransformd::Identity();

    // Detected transformation between visual marker and camera
    sva::PTransformd markerXCamera_ = sva::PTransformd::Identity();

    // Target experessed in the camera frame as target_X_marker * marker_X_camera
    sva::PTransformd targetXCamera_ = sva::PTransformd::Identity();

    // Mobile base position w.r.t camera as computed from kinematic chain
    sva::PTransformd mobilebaseXCamera_ = sva::PTransformd::Identity();

    // Detected human upper back (point between chest and neck) distance from the floor
    int chestFrame_ = firstHumanID_ + SPINE_CHEST;
    int neckFrame_ = firstHumanID_ + SPINE_CHEST;
    double humanUpperBackLevel_;
};
