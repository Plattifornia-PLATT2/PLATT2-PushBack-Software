#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
//#include "pros/screen.h"
//#include "pros/screen.hpp"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/intake/IntakeSubsystem.hpp"
#include "subsystems/odometry/Odometry.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace platt2{

namespace robot{

    Robot::Robot(
        std::shared_ptr<subsystems::holonomicDrive::XDrive>& xdrive_subsystem,
        std::shared_ptr<subsystems::odometry::Odometry>& odometry_subsystem,
        std::shared_ptr<subsystems::holonomicDrive::HolonomicControl>& holonomic_controller,
        std::unique_ptr<subsystems::intake::IntakeSubsystem>& intake_subsystem
    )
    {
        current_alliance = NO_ALLIANCE;
        current_auton_route = NO_AUTON;
        current_config = NO_ROBOT;

        holonomicDrive_subsystem = xdrive_subsystem;
        odom_subsystem = odometry_subsystem;
        this->holonomic_controller = holonomic_controller;
        this->intake_subsystem = std::move(intake_subsystem);
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};

        while(true){
            
            double leftX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            double leftY = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
            double rightX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
            
            // Create movement vector
            subsystems::holonomicDrive::MovementVector movement;   

            movement.theta = atan2(leftY, leftX)-((odom_subsystem->getHeading())-(M_PI/2));
            movement.r = std::clamp(sqrt(leftX*leftX + leftY*leftY)/127, -1.0,1.0);
            movement.w = 0.1;
            //movement.w = rightX/127;

            // Send to subsystem
            holonomicDrive_subsystem->moveVector(movement);
            pros::delay(10);
        }
    }

    void Robot::autonControl(){
    std::cout<<"auton starting"<<std::endl;
    holonomic_controller->moveToPoint(012, 24, 0);
    //holonomic_controller->staticTest();
    }

}
}