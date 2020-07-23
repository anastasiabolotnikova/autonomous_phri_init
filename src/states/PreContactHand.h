#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/LookAtSurfaceTask.h>

struct PreContactHand : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  std::shared_ptr<mc_tasks::LookAtSurfaceTask> lookAtTarget_;

  // Arm posture goals
  std::map<std::string, std::vector<double>> armPostureGoal_;
  std::vector<std::string> postureGoalJoints_;
  double delta_ = 0.05;

  // Multi-modal communication elements
  std::string textToSay_;
  std::string imageToDisplay_;

};
