#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/LookAtTask.h>

struct IntentCommunication : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

  private:
    // Full state configuration
    mc_rtc::Configuration config_;

    // Right arm to tablet pointing posture
    std::map<std::string, std::vector<double>> rightHandPointing_;

    // Camera orientation task
    std::shared_ptr<mc_tasks::LookAtTask> lookAtTask_;

    // Name of image file to display on the tablet
    std::string imageToDisplay_ = "";

    // Text to play from speakers
    std::string textToSayStart_ = "";
    std::string textToSayEnd_ = "";

    // Monitor when communication is done
    bool communacationDone_ = false;

    // Monitor human head orientation
    std::shared_ptr<mc_tasks::LookAtTask> lookAtTaskHuman_;
    bool humanLookedAtTablet = false;

    // Timer
    double stateTime = 0.0;
    double timeOut = 0.0;

    sva::PTransformd camera_X_world = sva::PTransformd::Identity();
    sva::PTransformd humanHead_X_world = sva::PTransformd::Identity();
    sva::PTransformd humanHead_X_camera = sva::PTransformd::Identity();
    sva::PTransformd tablet_X_world = sva::PTransformd::Identity();
    Eigen::Vector3d head_to_tablet;
    double head_to_tablet_angle;
    double head_to_tablet_angle_threshold_;
};
