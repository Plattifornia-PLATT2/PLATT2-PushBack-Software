#include "platt2/config/PinkConfig.hpp"
#include "platt2/robot/Robot.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "pros/motors.hpp"
#include <memory>
#include <vector>

namespace platt2{
namespace config{

std::shared_ptr<robot::Robot> PinkConfig::buildRobot(){

    //motors

    // LF Module
    std::unique_ptr<pros::v5::Motor> left_front_top{std::make_unique<pros::v5::Motor>(LEFT_FRONT_MODULE_TOP_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> left_front_bottom{std::make_unique<pros::v5::Motor>(LEFT_FRONT_MODULE_BOTTOM_PORT, DRIVE_GEARSET)};

    //RF Module
    std::unique_ptr<pros::v5::Motor> right_front_top{std::make_unique<pros::v5::Motor>(RIGHT_FRONT_MODULE_TOP_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> right_front_bottom{std::make_unique<pros::v5::Motor>(RIGHT_FRONT_MODULE_BOTTOM_PORT, DRIVE_GEARSET)};

    //LR Module
    std::unique_ptr<pros::v5::Motor> left_rear_top{std::make_unique<pros::v5::Motor>(LEFT_REAR_MODULE_TOP_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> left_rear_bottom{std::make_unique<pros::v5::Motor>(LEFT_REAR_MODULE_BOTTOM_PORT, DRIVE_GEARSET)};

    //RR Module
    std::unique_ptr<pros::v5::Motor> right_rear_top{std::make_unique<pros::v5::Motor>(RIGHT_REAR_MODULE_TOP_PORT, DRIVE_GEARSET)};
    std::unique_ptr<pros::v5::Motor> right_rear_bottom{std::make_unique<pros::v5::Motor>(RIGHT_REAR_MODULE_BOTTOM_PORT, DRIVE_GEARSET)};

    //X drive modules
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> left_front_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(left_front_top, left_front_bottom, 45, 2.75)};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> right_front_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(right_front_top, right_front_bottom, -45, 2.75)};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> left_rear_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(left_rear_top, left_rear_bottom, 235, 2.75)};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> right_rear_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(right_rear_top, right_rear_bottom, 180, 2.75)};

    // x drive system
    std::vector<std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule>> modules;
    modules.push_back(std::move(left_front_module));
    modules.push_back(std::move(right_front_module));
    modules.push_back(std::move(left_rear_module));
    modules.push_back(std::move(right_rear_module));

    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> XDrive_subsystem = std::make_unique<platt2::robot::subsystems::holonomicDrive::XDrive>(std::move(modules));

    // odom subsystem
    std::unique_ptr<robot::subsystems::odometry::Odometry> odom_subsystem = std::make_unique<robot::subsystems::odometry::Odometry>();

    std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>(XDrive_subsystem, odom_subsystem)

    };

    return robot;

}

}}