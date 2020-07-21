#pragma once

#include <mc_pepper/tasks/CoMRelativeBodyTask.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include "api.h"

struct PepperFSMController_DLLAPI PepperFSMController : public mc_control::fsm::Controller
{
    PepperFSMController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    std::map<std::string, std::vector<double>> uprightStanding() { return uprightStanding_; }

    std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask() {return mobileBaseTask_; }

    std::shared_ptr<mc_pepper::CoMRelativeBodyTask> comTask() {return comTask_; }

    bool pepperHasSpeakers() {return speakerDeviceName_ != ""; }
    bool pepperHasTablet() {return tabletDeviceName_ != ""; }
    bool pepperHasBumpers() {return bumperSensorNames_.size() != 0; }

    std::string speakerDeviceName() { return speakerDeviceName_; }
    std::string tabletDeviceName() { return tabletDeviceName_; }
    std::vector<std::string> bumperSensorNames() { return bumperSensorNames_; }

    std::string camOpticalFrame() { return camOpticalFrame_; }

    double humanHeight() { return humanHeight_; }
    double chairSeatHeight() { return chairSeatHeight_; }
    double humanUpperBackLevel() { return humanUpperBackLevel_; }
    void humanUpperBackLevel(double dist) { humanUpperBackLevel_ = dist; }

    void processGrippers(const mc_rtc::Configuration &gripper_config);

private:
    // Controller configuration
    mc_rtc::Configuration config_;

    // Time for plot
    double t_ = 0.0;

    // Default Pepper straight posture
    std::map<std::string, std::vector<double>> uprightStanding_;

    // MobileBase position task
    std::shared_ptr<mc_tasks::EndEffectorTask> mobileBaseTask_;

    // Relative CoM task
    std::shared_ptr<mc_pepper::CoMRelativeBodyTask> comTask_;

    // Robot device names
    std::string speakerDeviceName_ = "";
    std::string tabletDeviceName_ = "";
    std::vector<std::string> bumperSensorNames_ = {};

    // Camera optical frame name
    std::string camOpticalFrame_ = "";

    // Setup dimensions, default values are average (m)
    double humanHeight_ = 1.65;
    double chairSeatHeight_ = 0.45;

    // Human upper back distance from the ground
    double humanUpperBackLevel_ = std::numeric_limits<double>::quiet_NaN();
};
