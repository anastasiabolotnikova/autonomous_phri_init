#pragma once

#include <mc_control/fsm/State.h>

struct MoveMobileBase : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  // Relative position target increment
  std::vector<sva::PTransformd> sequentialTargets_;
  unsigned int targetCnt_ = 0;
  sva::PTransformd absTarget_;

  // State completion criteria
  double taskCompletion_;
};
