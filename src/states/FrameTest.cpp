#include "FrameTest.h"

#include "../PepperFSMController.h"
#include <mc_rbdyn/rpy_utils.h>

void FrameTest::configure(const mc_rtc::Configuration & config)
{
  // Load state config
  config_.load(config);
}

void FrameTest::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PepperFSMController &>(ctl_);

  pelvisXWorld_ = config_("pelvisXWorld");
  mobileBaseXPelvis_ = config_("mobileBaseXPelvis");
  ctl_.gui()->addElement({"FrameTest", "Frames"},
      mc_rtc::gui::Transform("World", [this]() { return sva::PTransformd::Identity(); }),
      mc_rtc::gui::Transform("pelvisXWorld", [this]() { return pelvisXWorld_; }),
      mc_rtc::gui::Transform("mobilebaseXPelvis", [this]() { return mobileBaseXPelvis_ * pelvisXWorld_; }),
      mc_rtc::gui::Transform("mobilebaseXWorldCorrected", [this]() { return mobileBaseXWorldCorrected_; }),
      mc_rtc::gui::Transform("pelvisXWorldCorrected", [this]() { return pelvisXWorldCorrected_; })
  );


  /* Check if human pelvis frame inclination angle agrees with sitting straingh assumption
     Rotation matrix based human pelvis frame inclination check */

  // Coordinates of the pelvis rotation frame X axis unit vector tip in the world frame
  Eigen::Vector3d pelvisX = pelvisXWorld_.rotation().transpose().col(0);
  // Angle between pelvisX and Z axis unit vector of the world frame
  Eigen::Vector3d worldZ = Eigen::Vector3d(0, 0, 1);
  // Angle between two vectors
  double theta = std::acos(pelvisX.dot(worldZ)/(pelvisX.norm() * worldZ.norm()));
  mc_rtc::log::info("FrameTest | Angle between pelvis X axis {} and world Z axis {} : {}deg", pelvisX.transpose(), 
                                                                                              worldZ.transpose(), 
                                                                                              mc_rtc::constants::toDeg(theta));

  // Compute quaternion rotation which brings pelvisX to the worldZ
  Eigen::Quaterniond q = Eigen::Quaterniond().setFromTwoVectors(pelvisX, worldZ);
  mc_rtc::log::info("FrameTest | Angle axis {}, {}", mc_rtc::constants::toDeg(Eigen::AngleAxisd(q).angle()), Eigen::AngleAxisd(q).axis().transpose());

  // Rotate pelvisX to be at worldZ
  Eigen::Vector3d correctedPelvisX = q * pelvisX;
  mc_rtc::log::info("FrameTest | corrected/rotated pelvisX vector (expect 0, 0, 1): {}", correctedPelvisX.transpose());

  // Apply shortest arc pelvisX->worldZ quaternion rotation to correct the entire frame
  pelvisXWorldCorrected_ = pelvisXWorld_ * sva::PTransformd(q.inverse());
  mc_rtc::log::info("FrameTest | Corrected pelvis frame X axis coordinates (expect 0 0 1): {}", pelvisXWorldCorrected_.rotation().transpose().col(0).transpose());
}

bool FrameTest::run(mc_control::fsm::Controller & ctl_)
{
  return false;
}

void FrameTest::teardown(mc_control::fsm::Controller &)
{
}

EXPORT_SINGLE_STATE("FrameTest", FrameTest)
