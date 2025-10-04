#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "MovementVector.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "pros/screen.hpp"
#include <cmath>

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{
 
void XDriveModule::move_vector(MovementVector v){
    double v_x = v.linear_speed * std::cos(v.travel_angle);
    double v_y = v.linear_speed * std::sin(v.travel_angle);

    double translation = v_x * std::cos(angleFromZero) + v_y * std::sin(angleFromZero);
    double rotation = v.target_heading/127;

    double combined = (translation + rotation) * 12000.0;
    module_motors.setVoltage(combined);
}

XDriveModule::XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double angleFromZero, double wheel_radius){
    module_motors.addMotor(motor_l);
    module_motors.addMotor(motor_r);
    this->angleFromZero = angleFromZero;
    this->wheel_radius = wheel_radius;
}

}}}} // namespaces