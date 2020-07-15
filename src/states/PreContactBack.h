#pragma once

#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_control/CompletionCriteria.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/LookAtTask.h>

struct PreContactBack : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  std::shared_ptr<mc_tasks::LookAtTask> lookAtHandTarget_;
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> handTrajectoryTask_;
  mc_control::CompletionCriteria trajTaskCriteria_;

};
