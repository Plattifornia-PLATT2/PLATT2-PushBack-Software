#ifndef HOLONOMICCONTROL_HPP
#define HOLONIMICCONTROL_HPP

#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/helperFunctions.h"
#include <memory>


namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{


class HolonomicControl{
public:
    HolonomicControl(XDrive& drive, odometry::Odometry& odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID);

    void moveToPoint(double x_target, double y_target, double heading_target);
    bool atTarget(double x_target, double y_target, double heading_target);

private:
    XDrive& drivetrain;
    odometry::Odometry& odometry;
    std::unique_ptr<pid::PID> positionPID;
    std::unique_ptr<pid::PID> headingPID;
};
}}}}

#endif