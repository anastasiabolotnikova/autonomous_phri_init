#pragma once

#include <mc_control/fsm/State.h>

struct FrameTest : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // State configuration
  mc_rtc::Configuration config_;

  sva::PTransformd pelvisXWorld_;
  sva::PTransformd pelvisXWorldCorrected_;
  sva::PTransformd mobileBaseXPelvis_;
  sva::PTransformd mobileBaseXWorldCorrected_;
};
