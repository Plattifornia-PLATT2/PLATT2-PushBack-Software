#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
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
        std::shared_ptr<subsystems::intake::IntakeSubsystem>& intake_subsystem,
        platt2::robot::AllianceConfig alliance_config,
        platt2::robot::RobotConfig robot_config,
        platt2::robot::AutonConfig auton_config,
        std::unique_ptr<profiles::DriverProfile>& driver_profile
    ) : holonomicDrive_subsystem{xdrive_subsystem},
        odom_subsystem{odometry_subsystem},
        holonomic_controller{holonomic_controller},
        intake_subsystem{intake_subsystem},
        driver_profile{std::move(driver_profile)}
    {
        current_alliance = alliance_config;
        current_auton_route = auton_config;
        current_config = robot_config;
    }

    void Robot::driverControl(){

        pros::Controller controller{pros::Controller(pros::E_CONTROLLER_MASTER)};

        while(true){
            double leftX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
            double leftY = double(-controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
            double rightX = double(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
            // Create movement vector
            subsystems::holonomicDrive::MovementVector movement;   

            movement.theta = atan2(leftY, leftX)-((odom_subsystem->getHeading())-(M_PI/2));
            movement.r = std::clamp(sqrt(leftX*leftX + leftY*leftY)/127, -1.0,1.0);
            //movement.w = 0.1;
            movement.w = rightX/127;

            // Send to subsystem
            holonomicDrive_subsystem->moveVector(movement);

            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::IN);
            }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::OUT);
            }
            else{
                intake_subsystem->move_intake(subsystems::intake::IntakeDirection::STOP);
            }

            pros::delay(10);
        }
    }

    void Robot::autonControl(){
    holonomic_controller->moveToPoint(0, 24, 90);
    holonomic_controller->moveToPoint(-24, 24,0);
    }

}
}