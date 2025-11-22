#include "platt2/config/PurpleConfig.hpp"

namespace platt2{
namespace config{
constexpr double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

std::shared_ptr<robot::Robot> PurpleConfig::buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance){

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

    //Intake motors
    std::unique_ptr<pros::Motor> front_intake_motor{std::make_unique<pros::Motor>(FRONT_INTAKE_MOTOR_PORT, INTAKE_GEARSET)};
    std::unique_ptr<pros::Motor> middle_intake_motor{std::make_unique<pros::Motor>(MIDDLE_INTAKE_MOTOR_PORT, INTAKE_GEARSET)};
    std::unique_ptr<pros::Motor> rear_intake_motor{std::make_unique<pros::Motor>(REAR_INTAKE_MOTOR_PORT, INTAKE_GEARSET)};
    std::unique_ptr<pros::Motor> upper_conveyor_motor{std::make_unique<pros::Motor>(UPPER_CONVEYOR_MOTOR_PORT, INTAKE_GEARSET)};

    //X drive modules
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> left_front_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(left_front_top, left_front_bottom, deg_to_rad(45),deg_to_rad(135))};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> left_rear_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(left_rear_top, left_rear_bottom, deg_to_rad(135),deg_to_rad(225))};    
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> right_rear_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(right_rear_top, right_rear_bottom, deg_to_rad(225),deg_to_rad(315))};
    std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule> right_front_module{std::make_unique<platt2::robot::subsystems::holonomicDrive::XDriveModule>(right_front_top, right_front_bottom, deg_to_rad(315),deg_to_rad(45))};


    // x drive system
    std::vector<std::unique_ptr<platt2::robot::subsystems::holonomicDrive::XDriveModule>> modules;
    modules.push_back(std::move(left_front_module));
    modules.push_back(std::move(right_front_module));
    modules.push_back(std::move(left_rear_module));
    modules.push_back(std::move(right_rear_module));

    std::shared_ptr<platt2::robot::subsystems::holonomicDrive::XDrive> XDrive_subsystem = std::make_shared<platt2::robot::subsystems::holonomicDrive::XDrive>(std::move(modules));

    // odom subsystem
    std::shared_ptr<robot::subsystems::odometry::Odometry> odom_subsystem = std::make_shared<robot::subsystems::odometry::Odometry>();

    // intake subsystem
    std::unique_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem = std::make_unique<robot::subsystems::intake::IntakeSubsystem>(std::move(front_intake_motor), std::move(middle_intake_motor), std::move(rear_intake_motor), std::move(upper_conveyor_motor));

    //holonomic control system
    std::unique_ptr<robot::pid::PID>position_pid = std::make_unique<robot::pid::PID>(position_dt, position_max, position_min, position_Kp, position_Kd, position_Ki);
    std::unique_ptr<robot::pid::PID>heading_pid = std::make_unique<robot::pid::PID>(heading_dt, heading_max, heading_min, heading_Kp, heading_Kd, heading_Ki);
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_contol_subsystem = std::make_shared<robot::subsystems::holonomicDrive::HolonomicControl>(XDrive_subsystem, odom_subsystem, std::move(position_pid), std::move(heading_pid));

    //build driver profile
    std::unique_ptr<profiles::DriverProfile> driver_profile;

    if(profile == robot::JON){
        std::unique_ptr<profiles::JonProfile> jon_profile = std::make_unique<profiles::JonProfile>();
        driver_profile = std::move(jon_profile);
    }
    else{
        std::unique_ptr<profiles::QuinnProfile> quinn_profile = std::make_unique<profiles::QuinnProfile>();
        driver_profile = std::move(quinn_profile);
    }

    // build robot object
    std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>(XDrive_subsystem, odom_subsystem, holonomic_contol_subsystem, intake_subsystem, alliance, platt2::robot::RobotConfig::PURPLE, auton, driver_profile)};

    return robot;

}

}}