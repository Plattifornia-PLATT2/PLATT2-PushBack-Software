#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include <memory>
#include <numbers>
#include <utility>

namespace platt2{
namespace robot{
namespace subsystems{
namespace holonomicDrive{

void HolonomicControl::moveToPoint(double x_target, double y_target, double target_heading) {
    //double x_error = x_target - odometry.getX();
    //double y_error = y_target - odometry.getY();
    //double distance_error = hypot(x_error, y_error);
    //double travel_angle = atan2(y_error, x_error);
//
    //double linear_speed = positionPID->calculate(0.0, distance_error);
    //linear_speed = std::clamp(linear_speed, -1.0, 1.0);
//
    //double heading_error = target_heading - odometry.getHeading();
    //while (heading_error > M_PI) heading_error -= 2 * M_PI;
    //while (heading_error < -M_PI) heading_error += 2 * M_PI;
    //double rotation_output = headingPID->calculate(heading_error, odometry.getHeading());
    //rotation_output = std::clamp(rotation_output, -1.0, 1.0);
//
    //drivetrain.moveVector({travel_angle, linear_speed, target_heading, odometry.getHeading(), rotation_output});
    MovementVector motionVector;
    polor p;
    bool arrived = false;
    double x_error;
    double y_error;
    double angle_error;

    while (arrived == false){

        x_error = x_target - odometry.getX();
        y_error = y_target - odometry.getY();

        p = CtoP(x_error, y_error);

        angle_error = target_heading - odometry.getHeading();

        //
        if(angle_error > std::numbers::pi or angle_error < -std::numbers::pi){
            angle_error = -1 * sgn(angle_error) * (2*std::numbers::pi - std::abs(angle_error));
        }

        motionVector.r = positionPID->calculate(0, p.r);
        motionVector.theta = p.theta;
        motionVector.w = headingPID->calculate(0, angle_error);
        
        drivetrain.moveVector(motionVector);
    }
}

HolonomicControl::HolonomicControl(XDrive& drive, odometry::Odometry& odom, std::unique_ptr<robot::pid::PID> posPID, std::unique_ptr<robot::pid::PID> headingPID):
    drivetrain(drive),
    odometry(odom),
    positionPID(std::move(posPID)),
    headingPID(std::move(headingPID))
{
    
}

}
}
}
}