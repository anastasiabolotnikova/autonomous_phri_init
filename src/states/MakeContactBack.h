#pragma once

#include <mc_tasks/RelativeEndEffectorTask.h>
#include "../filters/MedianFilter.h"
#include <mc_control/fsm/State.h>
#include <xgboost/c_api.h>

struct MakeContactBack : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
  // Update input vector elements in inputVec_
  void updateInputVector(mc_control::fsm::Controller & ctl, std::vector<std::pair<std::string, std::string>> &features);
  // State plots
  void addPlot(mc_control::fsm::Controller & ctl);
  // State log
  void addLog(mc_control::fsm::Controller & ctl);

  // State configuration
  mc_rtc::Configuration config_;
  // State time
  double stateTime_ = 0.0;

  // Hand position task
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> handTask_;
  // By how much to move hand inward (Y axis)
  double moveInward_;
  // Monitor if contact is detected
  bool contactDetected_ = false;
  // How long to stay in contact after it has been detected
  double inContactDuration_;

  // XGBoost predictor
  BoosterHandle boosterHandle_;
  // Monitored joint info
  std::string monitoredJointName_;
  unsigned int monitoredJointIndex_;
  unsigned int monitoredJointRefOrder_;
  // Names of the model input features
  std::vector<std::pair<std::string, std::string>> features_;
  // Number of features
  bst_ulong numF_;
  // Input feature vector handle
  DMatrixHandle inputVec_;
  // Prediction vector length
  bst_ulong outLen_;
  // Prediction output vector
  const float *outVec_;
  // Expected position tracking error
  double errExp_ = 0.0;

  // Measured position tracking error
  double err_ = 0.0;
  // Residual value
  double jointResidual_ = 0.0;
  // Median filter
  const static int filterWindowSize_ = 50;
  MedianFilter<double, filterWindowSize_> medianFilter_;
  bool filterWindowFilled_ = false;
  // Filtered contact residual
  double jointResidualFiltered_ = 0.0;
  // Residual threshold
  double residualThreshold_;
};
