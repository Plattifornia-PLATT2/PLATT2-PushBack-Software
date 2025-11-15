#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/helperFunctions.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{

void HolonomicControl::moveToPoint(double x_target, double y_target, double target_heading) {

    target_heading = target_heading*M_PI/180;

    MovementVector motionVector;
    polar p;

    double x_error;
    double y_error;
    double angle_error;

    std::vector<double> rArray(100,1);
    std::vector<double> wArray(100,1);

    avg rAve;
    avg wAve;

    while (true){
        //std::cout<<motionVector.w<<std::endl;

        x_error = x_target - odometry->getX();
        y_error = y_target - odometry->getY();

        p = CtoP(x_error, y_error);

        angle_error = target_heading - odometry->getHeading();

        if(angle_error > std::numbers::pi || angle_error < -std::numbers::pi){
            angle_error = -1 * sgn(angle_error) * (2*std::numbers::pi - std::abs(angle_error));
        }
        
        motionVector.r = positionPID->calculate(0, p.r)*0;
        motionVector.theta = p.theta;
        motionVector.w = headingPID->calculate(0, angle_error);

        rAve = rollAverage(std::abs(motionVector.r), rArray);
        wAve = rollAverage(std::abs(motionVector.w), wArray);
        rArray = rAve.data;
        wArray = wAve.data;

        std::cout<<"rAve: "<<rAve.average<<" wAve: "<<wAve.average<<std::endl;
        if (rAve.average < 0.05 && wAve.average < 0.20){break;}
        
        drivetrain->moveVector(motionVector);
        std::cout<<"Moving vector with w: "<<motionVector.w<<std::endl; 
        pros::delay(10);
    }
    std::cout<<"I got to loop end"<<std::endl;
    motionVector.r = 0;
    motionVector.w = 0;
    motionVector.theta = 0;
    drivetrain->moveVector(motionVector);

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