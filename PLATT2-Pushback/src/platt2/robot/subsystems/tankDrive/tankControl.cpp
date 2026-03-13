#include "platt2/robot/subsystems/tankDrive/tankControl.hpp"
#include "platt2/helperFunctions.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <math.h>
#include <memory>
#include <utility>
#include <vector>
namespace platt2{
namespace robot{
namespace subsystems{
namespace tankDrive{

void TankControl::moveToPoint(double x_target, double y_target, double target_heading, double rSpeed, double wSpeed, double timeout) {



}

double TankControl::velocityProfile(double totalDistance, double remainingDistance, double currentVel, double maxVel, double maxAccel, bool& usePID) {

    double distanceTravelled = totalDistance - remainingDistance;
    bool reachedMaxVel       = currentVel >= maxVel;
    bool pastHalfway         = distanceTravelled >= totalDistance * 0.5;

    if ((!reachedMaxVel && !pastHalfway) && !usePID ) {
    //if (false) {
        // --- Acceleration phase ---
        double max_accel_delta = maxAccel * 0.01;
        double output = std::clamp(currentVel + max_accel_delta, 0.0, maxVel);
        return output;

    } else {
        // --- Handoff to decel/approach controller ---
        usePID = true;
        return std::clamp((positionPID->calculate(remainingDistance, 0)), 0.0, maxVel);
    }
}




TankControl::TankControl(std::shared_ptr<TankDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    this->positionPID = std::move(posPID);

}

}
}
}}
