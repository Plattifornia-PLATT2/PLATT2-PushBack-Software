#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
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
namespace holonomicDrive{

void HolonomicControl::moveToPoint(double x_target, double y_target, double target_heading, double rSpeed, double wSpeed, double timeout) {

    target_heading = target_heading*M_PI/180;

    MovementVector motionVector;
    polar p;

    double x_error;
    double y_error;
    double angle_error;

    std::vector<double> rArray(40,1);
    std::vector<double> wArray(40,1);

    avg rAve;
    avg wAve;

    double startTime = pros::millis();

    double startPosX = odometry->getX();
    double startPosY = odometry->getY();

    double moveDistance = CtoP(x_target-startPosX, y_target-startPosY).r;

    double currentVel = 0;

    double actualVel = 0;
    double distTravelled = 0;
    double lastDistTravelled = 0;
 
    while (true){
        
        x_error = odometry->getX() - x_target;
        y_error = odometry->getY() - y_target;  

        p = CtoP(x_error, y_error);

        angle_error = target_heading - (odometry->getHeading());

        if(angle_error > M_PI || angle_error < -M_PI){
            angle_error = -1 * sgn(angle_error) * (2*M_PI - std::abs(angle_error));
        }

        distTravelled = CtoP(odometry->getX() - startPosX,odometry->getY() - startPosY).r;
        actualVel = (distTravelled - lastDistTravelled) / 0.01;
        lastDistTravelled = distTravelled;
        
        currentVel = velocityProfile(moveDistance, p.r, currentVel, actualVel, 0.8, 1);
        motionVector.r = currentVel;
        motionVector.theta = p.theta - (odometry->getHeading());

        motionVector.w = std::clamp(headingPID->calculate(0, angle_error), -wSpeed, wSpeed);

        rAve = rollAverage(std::abs(motionVector.r), rArray);
        wAve = rollAverage(std::abs(motionVector.w), wArray);
        rArray = rAve.data;
        wArray = wAve.data;

        if (rAve.average < 0.025 && wAve.average < 00.01){break;}
        if (pros::millis()-startTime>timeout*1000){break;}
        
        drivetrain->moveVector(motionVector);
        pros::delay(10);

    } 

    motionVector.r = 0;
    motionVector.w = 0;
    motionVector.theta = 0;
    drivetrain->moveVector(motionVector);

}

double HolonomicControl::velocityProfile(double totalDistance, double remainingDistance,
                                          double currentVel,   double actualVel,
                                          double maxVel,       double maxAccel) {

    double distanceTravelled = totalDistance - remainingDistance;
    bool reachedMaxVel       = currentVel >= maxVel;
    bool pastHalfway         = distanceTravelled >= totalDistance * 0.5;

    if (!reachedMaxVel && !pastHalfway) {
        // --- Acceleration phase ---
        double max_accel_delta = maxAccel * 0.01;
        double output = std::clamp(currentVel + max_accel_delta, 0.0, maxVel);
        return output;

    } else {
        // --- Handoff to decel/approach controller ---
        return std::clamp(-1*(positionPID->calculate(0, remainingDistance)), -maxVel, maxVel);
    }
}





HolonomicControl::HolonomicControl(std::shared_ptr<XDrive> drive, std::shared_ptr<odometry::Odometry> odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID)
{
    drivetrain = drive;
    odometry = odom;
    positionPID = std::move(posPID);
    this->headingPID = std::move(headingPID);
}

}
}
}
}