#include "platt2/config/PurpleConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/auton/PurpleSkillsAuton.hpp"
#include "platt2/hal/TrackingWheel.hpp"
#include "platt2/robot/subsystems/odometry/TrackingWheelPositionTracker.hpp"
#include "pros/rotation.hpp"
#include <memory>

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
    std::unique_ptr<pros::Motor> lower_roller_motor{std::make_unique<pros::Motor>(LOWER_ROLLER_MOTOR_PORT, pros::MotorGears::green)};

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
    std::unique_ptr<pros::IMU> vex_imu = std::make_unique<pros::IMU>(VEX_IMU_PORT);
    std::unique_ptr<pros::Rotation> horiontal_encoder = std::make_unique<pros::Rotation>(HORIZONTAL_ENCODER_PORT);
    std::unique_ptr<pros::Rotation> vertical_encoder = std::make_unique<pros::Rotation>(VERTICAL_ENCODER_PORT);

    std::unique_ptr<hal::TrackingWheel> horizontal_tracking_wheel = std::make_unique<hal::TrackingWheel>(std::move(horiontal_encoder), TRACKING_WHEEL_DIAMETER);
    std::unique_ptr<hal::TrackingWheel> vertical_tracking_wheel = std::make_unique<hal::TrackingWheel>(std::move(vertical_encoder), TRACKING_WHEEL_DIAMETER);

    std::unique_ptr<robot::subsystems::odometry::TrackingWheelPositionTracker> position_tracker = std::make_unique<robot::subsystems::odometry::TrackingWheelPositionTracker>(std::move(horizontal_tracking_wheel), std::move(vertical_tracking_wheel), std::move(vex_imu));
    position_tracker->setOffsets(HORIZONTAL_TRACKING_WHEEL_OFFSET, VERTICAL_TRACKING_WHEEL_OFFSET);
    std::shared_ptr<robot::subsystems::odometry::Odometry> odom_subsystem;
    odom_subsystem = std::make_shared<robot::subsystems::odometry::Odometry>(std::move(position_tracker));
   
    robot::subsystems::odometry::OdometryPosition startingPos = {0,0,90};
    odom_subsystem->setPos(startingPos);
    
    // intake subsystem
    std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston{std::make_unique<pros::adi::DigitalOut>(ED_MECH_PISTON_PORT)};
    std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston{std::make_unique<pros::adi::DigitalOut>(UPPER_CONVEYOR_HEIGHT_PISTON_PORT)};
    std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston{std::make_unique<pros::adi::DigitalOut>(CONVEYOR_STOPPER_PISTON_PORT)};
    std::unique_ptr<pros::adi::DigitalOut> rake_mech_piston{std::make_unique<pros::adi::DigitalOut>(RAKE_MECH_PISTON_PORT)};
    std::unique_ptr<pros::adi::DigitalOut> descore_piston{std::make_unique<pros::adi::DigitalOut>(DESCORE_PISTON_PORT)};
    std::unique_ptr<pros::Distance> distance_sensor{std::make_unique<pros::Distance>(DISTANCE_SENSOR_PORT)};

    std::unique_ptr<pros::Motor> matchload_left_motor{std::make_unique<pros::Motor>(REAR_INTAKE_LEFT_MOTOR_PORT, pros::MotorGears::blue)};
    std::unique_ptr<pros::Motor> matchload_right_motor{std::make_unique<pros::Motor>(REAR_INTAKE_RIGHT_MOTOR_PORT, pros::MotorGears::blue)};


    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem = std::make_shared<robot::subsystems::intake::IntakeSubsystem>(
        std::move(front_intake_motor), 
        std::move(rear_intake_motor), 
        std::move(middle_intake_motor), 
        std::move(upper_conveyor_motor),
        std::move(lower_roller_motor),
        std::move(matchload_left_motor),
        std::move(matchload_right_motor),
        std::move(ed_mech_piston),
        std::move(upper_conveyor_height_piston),
        std::move(conveyor_stopper_piston),
        std::move(rake_mech_piston),
        std::move(descore_piston),
        std::move(distance_sensor)
    );

    // Color sort subsystem
    std::unique_ptr<pros::Optical> optical_sensor{std::make_unique<pros::Optical>(OPTICAL_SENSOR_PORT)};
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem = std::make_shared<robot::subsystems::colorsort::ColorSortSubsystem>(intake_subsystem,std::move(optical_sensor));

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

        // Build auton routine
    std::unique_ptr<auton::IAuton> auton_routine;

    if(auton == robot::SKILLS_1 ){
        std::unique_ptr<auton::PurpleSkillsAuton> purple_skills_auton = std::make_unique<auton::PurpleSkillsAuton>();
        auton_routine = std::move(purple_skills_auton);
        auton_routine->init(holonomic_contol_subsystem, odom_subsystem, intake_subsystem, color_sort_subsystem, alliance);
    }
    else {
        std::unique_ptr<auton::PurpleCompAuton> purple_comp_auton = std::make_unique<auton::PurpleCompAuton>();
        auton_routine = std::move(purple_comp_auton);
        auton_routine->init(holonomic_contol_subsystem, odom_subsystem, intake_subsystem, color_sort_subsystem, alliance);
    }

    // build robot object
    std::shared_ptr<robot::Robot> robot{std::make_shared<robot::Robot>(
        XDrive_subsystem, 
        odom_subsystem, 
        holonomic_contol_subsystem, 
        intake_subsystem, 
        alliance, 
        platt2::robot::RobotConfig::PURPLE, 
        auton, 
        driver_profile,
        auton_routine,
        color_sort_subsystem
    )};

    return robot;

}

}}