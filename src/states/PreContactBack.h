#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/LookAtSurfaceTask.h>

struct PreContactBack : mc_control::fsm::State
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
  std::map<std::string, std::vector<double>> armPostureGoal1_;
  bool goal1Reached_ = false;
  std::map<std::string, std::vector<double>> armPostureGoal2_;
  bool goal2Reached_ = false;
  double delta_ = 0.05;
};
