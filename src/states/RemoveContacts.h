#pragma once

#include <mc_control/fsm/State.h>

struct RemoveContacts : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  // Sequential posture goals
  std::vector<std::map<std::string, std::vector<double>>> sequentialPostureGoals_;
  std::vector<std::string> currentPostureGoalJoints_;
  unsigned int currentPostureGoal_ = 0;
  double delta_ = 0.05;

  // Communication elements
  std::string textToSayStart_;
  std::string textToSayEnd_;
};
