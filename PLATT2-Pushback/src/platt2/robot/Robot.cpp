#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "subsystems/holonomicDrive/XDriveModule.hpp"
#include "subsystems/odometry/Odometry.hpp"
#include <algorithm>
#include <memory>

namespace platt2{

namespace robot{

    Robot::Robot(
        std::unique_ptr<subsystems::holonomicDrive::XDrive>& xdrive_subsystem,
        std::unique_ptr<subsystems::odometry::Odometry>& odometry_subsystem
    )
    {
        current_alliance = NO_ALLIANCE;
        current_auton_route = NO_AUTON;
        current_config = NO_ROBOT;

        holonomicDrive_subsystem = std::move(xdrive_subsystem);
        odom_subsystem = std::move(odometry_subsystem);
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};

        while(true){
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Create movement vector
        subsystems::holonomicDrive::MovementVector movement;
        movement.travel_angle = atan2(leftY, leftX);
        movement.linear_speed = sqrt(leftX*leftX + leftY*leftY) / 127.0;
        movement.target_heading = rightX;
        movement.current_heading = 0;
        
        // Send to subsystem
        holonomicDrive_subsystem->moveVector(movement);
        pros::delay(20);
        }
    }

}
}