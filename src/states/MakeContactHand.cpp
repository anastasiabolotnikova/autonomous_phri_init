#include "MakeContactHand.h"
#include "../PepperFSMController.h"

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/plot.h>

using Color = mc_rtc::gui::Color;

void MakeContactHand::configure(const mc_rtc::Configuration & config)
{
  // Read state configuration
  if(!config.has("inContactDuration")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | inContactDuration config entry missing");
  }
  config("inContactDuration", inContactDuration_);
  if(!config.has("features")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | features config entry missing");
  }
  config("features", features_);
  if(!config.has("residualThreshold")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | residualThreshold config entry missing");
  }
  config("residualThreshold", residualThreshold_);

  // Load state config
  config_.load(config);
}

void MakeContactHand::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Load camera orientation task
  if(!config_.has("lookAtTarget")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | lookAtTarget config entry missing");
  }
  lookAtTarget_ = mc_tasks::MetaTaskLoader::load<mc_tasks::LookAtSurfaceTask>(ctl.solver(), config_("lookAtTarget"));
  ctl.solver().addTask(lookAtTarget_);

  // Get index of the joint for monitoring contact residual
  if(!config_.has("monitoredJoint")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | monitoredJoint config entry missing");
  }
  config_("monitoredJoint", monitoredJointName_);
  monitoredJointIndex_ = ctl_.robot().jointIndexByName(monitoredJointName_);
  auto it = std::find(ctl_.robot().refJointOrder().begin(), ctl_.robot().refJointOrder().end(), monitoredJointName_);
  monitoredJointRefOrder_ = std::distance(ctl_.robot().refJointOrder().begin(), it);

  // Path to model file
  if(!config_.has("pathToModel")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | pathToModel config entry missing");
  }
  std::string pathToModel;
  config_("pathToModel", pathToModel);
  // Load trained predictor model
  XGBoosterCreate(0, 0, &boosterHandle_);
  if(XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()) == -1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | XGBoost model cannot be loaded from {}, {}", pathToModel,
                                                                                XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()));
  }
  // Validate model feature vector size
  XGBoosterGetNumFeature(boosterHandle_, &numF_);
  if(numF_ != features_.size()){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | Feature vector size mismatch. Expect {}, got {}", features_.size(), numF_);
  } // FIX removing this check causes prediction failure

  // Add state plot and log
  addPlot(ctl_);
  addLog(ctl_);

  // Load posture goal
  if(!config_.has("armPostureGoal")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand start | armPostureGoal config entry missing");
  }
  config_("armPostureGoal", armPostureGoal_);
  ctl_.getPostureTask("pepper")->target(armPostureGoal_);

  mc_rtc::log::success("MakeContactHand state start done");
}

bool MakeContactHand::run(mc_control::fsm::Controller & ctl_)
{
  // Update feature vector
  updateInputVector(ctl_, features_);

  // Predict expected position tracking error
  if(XGBoosterPredict(boosterHandle_, inputVec_, 0, 0, 0, &outLen_, &outVec_) == -1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand run | XGBoosterPredict failure");
  }
  if(outLen_!=1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand run | Wrong prediction result size {}", outLen_);
  }

  // Compute discrepancy
  err_ = ctl_.robot().q()[monitoredJointIndex_][0] - ctl_.robot().encoderValues()[monitoredJointRefOrder_];
  errExp_ = outVec_[0];
  jointResidual_ = err_ - errExp_;

  // Add residual signal to the filter window
  medianFilter_.addSample(jointResidual_);
  // Wait for the filter window to be filled
  if(stateTime_/ctl_.solver().dt() > filterWindowSize_){
    filterWindowFilled_ = true;
  }
  if(filterWindowFilled_){
    // Compute residual signal filter window median
    jointResidualFiltered_ = medianFilter_.getMedian();
    // Check if contact is detected
    if(std::abs(jointResidualFiltered_) > residualThreshold_){
      contactDetected_ = true;
    }
  }

  if(contactDetected_){
    if(!postureTaskReset_){
      ctl_.getPostureTask("pepper")->reset();
      postureTaskReset_ = true;
    }
    // In contact period countdown
    inContactDuration_ -= ctl_.solver().dt();
    // State termination criteria
    if(inContactDuration_ <= 0){
      output("OK");
      return true;
    }
  }
  // Updtae time for plot
  stateTime_ = stateTime_ + ctl_.solver().dt();
  return false;
}

void MakeContactHand::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.solver().removeTask(lookAtTarget_);
  removePlot(ctl_);
  removeLog(ctl_);
  mc_rtc::log::info("MakeContactHand teardown done");
}

void MakeContactHand::updateInputVector(mc_control::fsm::Controller & ctl_, std::vector<std::pair<std::string, std::string>> &features){
  float featureArray[1][numF_];
  for (size_t i = 0; i < features.size(); i++) {
    unsigned int jointIndex = ctl_.robot().jointIndexByName(features[i].first);
    if(features[i].second == "alpha"){
      featureArray[0][i] = float(ctl_.robot().mbc().alpha[jointIndex][0]);
    }else if(features[i].second == "alphaD"){
      featureArray[0][i] = float(ctl_.robot().mbc().alphaD[jointIndex][0]);
    }else if(features[i].second == "torque"){
      featureArray[0][i] = float(ctl_.robot().mbc().jointTorque[jointIndex][0]);
    }else{
      mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand updateInputVector | cannot add feature {} of joint {}",
                                                                                            features[i].second, features[i].first);
    }
  }
  // Update input feature matrix handle
  if(XGDMatrixCreateFromMat((float *)featureArray, 1, numF_, -1, &inputVec_) == -1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactHand updateInputVector | XGBoost cannot update input feature matrix handle");
  }
}

void MakeContactHand::addPlot(mc_control::fsm::Controller & ctl_){
  // Plot filtered residual
  ctl_.gui()->addPlot("Contact detection",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("Residual", [this]() { return jointResidualFiltered_; }, Color::Blue),
    mc_rtc::gui::plot::Y("+Threshold", [this]() { return residualThreshold_; }, Color::Red),
    mc_rtc::gui::plot::Y("-Threshold", [this]() { return -residualThreshold_; }, Color::Red)
  );

  // Plot measured and predicted errors
  ctl_.gui()->addPlot("Joint position tracking error",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("err", [this]() { return err_; }, Color::Black),
    mc_rtc::gui::plot::Y("expErr", [this]() { return errExp_; }, Color::Green)
  );
}

void MakeContactHand::removePlot(mc_control::fsm::Controller & ctl_){
  ctl_.gui()->removePlot("Contact detection");
  ctl_.gui()->removePlot("Joint position tracking error");
}

void MakeContactHand::addLog(mc_control::fsm::Controller & ctl_){
  ctl_.logger().addLogEntry(monitoredJointName_ + "_residual_raw", [this]() { return jointResidual_; });
  ctl_.logger().addLogEntry(monitoredJointName_ + "_residual_filtered", [this]() { return jointResidualFiltered_; });
  ctl_.logger().addLogEntry(monitoredJointName_ + "_residual_threshold+", [this]() { return residualThreshold_; });
  ctl_.logger().addLogEntry(monitoredJointName_ + "_residual_threshold-", [this]() { return -residualThreshold_; });
  ctl_.logger().addLogEntry(monitoredJointName_ + "_err_measured", [this]() { return err_; });
  ctl_.logger().addLogEntry(monitoredJointName_ + "_err_predicted", [this]() { return errExp_; });
}

void MakeContactHand::removeLog(mc_control::fsm::Controller & ctl_){
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_residual_raw");
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_residual_filtered");
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_residual_threshold+");
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_residual_threshold-");
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_err_measured");
  ctl_.logger().removeLogEntry(monitoredJointName_ + "_err_predicted");
}

EXPORT_SINGLE_STATE("MakeContactHand", MakeContactHand)
