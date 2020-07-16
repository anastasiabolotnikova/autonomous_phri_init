#pragma once

#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_control/fsm/State.h>

struct MakeContactBack : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  // Hand position task
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> handTask_;
  // By how much to move hand inward (Y axis)
  double moveInward_;
  // How long to stay in contact after it has been detected
  double inContactDuration_;
};
