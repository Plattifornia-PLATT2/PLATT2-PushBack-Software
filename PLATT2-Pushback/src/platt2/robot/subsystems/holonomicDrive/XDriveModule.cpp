#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "MovementVector.hpp"
#include "platt2/helperFunctions.h"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "pros/screen.hpp"
#include <cmath>

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{
 
void XDriveModule::move_vector(MovementVector v){
    
   
    double p = -(cos(v.theta+getPhi())/cos(getTheta()));

    double m = ((p/v.normalization_scalar)*(1-fabs(v.w))) + (sgn(cos(v.theta)*sin(v.theta))*v.theta);
    
    
    //// Convert joystick vector to X/Y components
    //double v_x = v.linear_speed * std::cos(v.travel_angle);
    //double v_y = v.linear_speed * std::sin(v.travel_angle);
//
    //// Project onto this module’s drive axis
    //double translation = (v_x * std::cos(angleFromZero) +
    //                      v_y * std::sin(angleFromZero)) * v.normalization_scalar;
//
    //// Apply rotation
    //double rotation = v.target_heading / 127.0;
    //double combined = (translation + rotation) * 12000.0;

    // Drive the motor(s)
    module_motors.setVoltage(m);
}

XDriveModule::XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double theta, double phi ,double wheel_radius){
    module_motors.addMotor(motor_l);
    module_motors.addMotor(motor_r);
    this->theta = theta;
    this->phi = phi;
    this->wheel_radius = wheel_radius;
}

double XDriveModule::getTheta(){

    return this->theta;

}
double XDriveModule::getPhi(){

    return this->phi;

}


}}}} // namespaces