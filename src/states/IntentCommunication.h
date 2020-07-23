#pragma once

#include "visualization_msgs/MarkerArray.h"
#include <mc_control/fsm/State.h>
#include <mc_tasks/GazeTask.h>
#include <mc_rtc/ros.h>
#include <thread>


struct IntentCommunication : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    // ROS topic monitoring thread function
    void monitorROSTopic();

    // ROS topic subscriber callback
    void updateVisualMarkerPose(const visualization_msgs::MarkerArray::ConstPtr& msg);

    // Full state configuration
    mc_rtc::Configuration config_;

    // Right arm to tablet pointing posture
    std::map<std::string, std::vector<double>> rightHandPointing_;

    // Timer
    double stateTime_ = 0.0;
    double timeOut_ = 0.0;

    // Name of image file to display on the tablet
    std::string imageToDisplay_ = "";

    // Text to play from speakers
    std::string textToSayStart_ = "";
    std::string textToSayEnd_ = "";
    bool textToSayEndSaid_ = false;

    // Camera orientation task
    std::shared_ptr<mc_tasks::GazeTask> ibvsTask_;

    // Gripper opening
    double rightGripperOpening_;

    // Monitor human head orientation
    bool monitorHeadOrientation_ = true;
    bool humanLookedAtTablet_ = false;
    Eigen::Vector3d headToTablet_;
    double headToTabletAngle_ = 1.5; //rad
    double headToTabletAngleThreshold_ = 1.0; //rad
    // Useful transforms
    sva::PTransformd cameraXWorld_ = sva::PTransformd::Identity();
    sva::PTransformd humanHeadXWorld_ = sva::PTransformd::Identity();
    sva::PTransformd humanHeadXCamera_ = sva::PTransformd::Identity();
    sva::PTransformd tabletXWorld_ = sva::PTransformd::Identity();

  // Monitor when communication is done
    bool communacationDone_ = false;

    // ROS spinning thread
    std::thread rosThread_;
    // ROS thread is kept alive while stateNeedsROS_ is true
    bool stateNeedsROS_;
    // Wait for at least one message from ROS topic
    bool firstROSUpdateDone_ = false;

    // Human body parts in camera frame
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

    // Map marker id to frame transform
    std::map<unsigned int, sva::PTransformd> humanBodyMarkers_;

    // For now we are interested in first (and only) human detection result
    int firstHumanID_ = 100;

    // Which frame to look at
    int ibvsFrameID_ = firstHumanID_ + HEAD;
};
