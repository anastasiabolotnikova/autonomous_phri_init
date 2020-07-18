#include "MakeContactBack.h"
#include "../PepperFSMController.h"

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/gui/plot.h>

using Color = mc_rtc::gui::Color;

void MakeContactBack::configure(const mc_rtc::Configuration & config)
{
  // Read state configuration
  if(!config.has("moveInward")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | moveInward config entry missing");
  }
  config("moveInward", moveInward_);
  if(!config.has("inContactDuration")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | inContactDuration config entry missing");
  }
  config("inContactDuration", inContactDuration_);
  if(!config.has("features")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | features config entry missing");
  }
  config("features", features_);
  if(!config.has("residualThreshold")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | residualThreshold config entry missing");
  }
  config("residualThreshold", residualThreshold_);

  // Load state config
  config_.load(config);
}

void MakeContactBack::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  // Load hand task
  if(!config_.has("handTask")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | handTask config entry missing");
  }
  handTask_ = mc_tasks::MetaTaskLoader::load<mc_tasks::RelativeEndEffectorTask>(ctl.solver(), config_("handTask"));
  // Set hand task target to move inward from the current position
  handTask_->add_ef_pose(sva::PTransformd(Eigen::Vector3d(0, moveInward_, 0)));
  ctl.solver().addTask(handTask_);

  // Get index of the joint for monitoring contact residual
  if(!config_.has("monitoredJoint")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | monitoredJoint config entry missing");
  }
  config_("monitoredJoint", monitoredJointName_);
  monitoredJointIndex_ = ctl_.robot().jointIndexByName(monitoredJointName_);
  auto it = std::find(ctl_.robot().refJointOrder().begin(), ctl_.robot().refJointOrder().end(), monitoredJointName_);
  monitoredJointRefOrder_ = std::distance(ctl_.robot().refJointOrder().begin(), it);

  // Path to model file
  if(!config_.has("pathToModel")){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | pathToModel config entry missing");
  }
  std::string pathToModel;
  config_("pathToModel", pathToModel);
  // Load trained predictor model
  XGBoosterCreate(0, 0, &boosterHandle_);
  if(XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()) == -1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | XGBoost model cannot be loaded from {}, {}", pathToModel,
                                                                                XGBoosterLoadModel(boosterHandle_, pathToModel.c_str()));
  }
  // Validate model feature vector size
  XGBoosterGetNumFeature(boosterHandle_, &numF_);
  if(numF_ != features_.size()){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack start | Feature vector size mismatch. Expect {}, got {}", features_.size(), numF_);
  } // FIX removing this check causes prediction failure

  // Add state plot and log
  addPlot(ctl_);
  addLog(ctl_);

  mc_rtc::log::success("MakeContactBack state start done");
}

bool MakeContactBack::run(mc_control::fsm::Controller & ctl_)
{
  if(!contactDetected_){
    // Update feature vector
    updateInputVector(ctl_, features_);

    // Predict expected position tracking error
    if(XGBoosterPredict(boosterHandle_, inputVec_, 0, 0, 0, &outLen_, &outVec_) == -1){
      mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack run | XGBoosterPredict failure");
    }
    if(outLen_!=1){
      mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack run | Wrong prediction result size {}", outLen_);
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
      if(std::abs(jointResidualFiltered_) > jointResidualFiltered_){
        contactDetected_ = true;
      }
    }
  }else{
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

void MakeContactBack::teardown(mc_control::fsm::Controller &)
{
}

void MakeContactBack::updateInputVector(mc_control::fsm::Controller & ctl_, std::vector<std::pair<std::string, std::string>> &features){
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
      mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack updateInputVector | cannot add feature {} of joint {}",
                                                                                            features[i].second, features[i].first);
    }
  }
  // Update input feature matrix handle
  if(XGDMatrixCreateFromMat((float *)featureArray, 1, numF_, -1, &inputVec_) == -1){
    mc_rtc::log::error_and_throw<std::runtime_error>("MakeContactBack updateInputVector | XGBoost cannot update input feature matrix handle");
  }
}

void MakeContactBack::addPlot(mc_control::fsm::Controller & ctl_){
  // Plot filtered residual
  ctl_.gui()->addPlot("Contact detection",
    mc_rtc::gui::plot::X("Time (s)", [this]() { return stateTime_; }),
    mc_rtc::gui::plot::Y("Residual", [this]() { return jointResidualFiltered_; }, Color::Blue),
    mc_rtc::gui::plot::Y("+Threshold", [this]() { return residualThreshold_; }, Color::Red),
    mc_rtc::gui::plot::Y("-Threshold", [this]() { return -residualThreshold_; }, Color::Red)
  );
}

void MakeContactBack::addLog(mc_control::fsm::Controller & ctl_){
  ctl_.logger().addLogEntry("Residual_raw", [this]() -> const double & { return jointResidual_; });
  ctl_.logger().addLogEntry("Residual_filtered", [this]() -> const double & { return jointResidualFiltered_; });
  ctl_.logger().addLogEntry("Residual_threshold+", [this]() -> const double & { return residualThreshold_; });
  ctl_.logger().addLogEntry("Residual_threshold-", [this]() -> const double & { return -residualThreshold_; });
  ctl_.logger().addLogEntry("Err_measured", [this]() -> const double & { return err_; });
  ctl_.logger().addLogEntry("Err_predicted", [this]() -> const double & { return errExp_; });
}

EXPORT_SINGLE_STATE("MakeContactBack", MakeContactBack)
