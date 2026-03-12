#include "platt2/robot/subsystems/tankDrive/tankDrive.hpp"
#include <vector>
#include "math.h"


namespace platt2{

namespace robot{

namespace subsystems{

namespace TankDrive{

    tankDrive::tankDrive(std::vector<std::unique_ptr<tankModule>> drive_modules){
        this->drive_modules = (std::move(drive_modules));
    }

    void tankDrive::moveVector(MovementVector v){ 
        
        drive_modules[0]->tankDrive::tankModule::module_motors.setVoltage(v.r+v.w);
        drive_modules[1]->tankDrive::tankModule::module_motors.setVoltage(v.r-v.w);
       
    }

    tankDrive::tankModule::tankModule(std::unique_ptr<pros::Motor>& motor_1, std::unique_ptr<pros::Motor>& motor_2, std::unique_ptr<pros::Motor>& motor_3, std::unique_ptr<pros::Motor>& motor_4){
    module_motors.addMotor(motor_1);
    module_motors.addMotor(motor_2);
    module_motors.addMotor(motor_3);
    module_motors.addMotor(motor_4);
    
}}}}}

