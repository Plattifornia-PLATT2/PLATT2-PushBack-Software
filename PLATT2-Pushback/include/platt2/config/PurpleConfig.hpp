#ifndef PURPLECONFIG_HPP
#define PURPLECONFIG_HPP

#include "platt2/config/IConfig.hpp"
#include "platt2/robot/Robot.hpp"
#include "platt2/robot/pid/pid.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/robot/subsystems/holonomicDrive/XDriveModule.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/profiles/JonProfile.hpp"
#include "platt2/profiles/DriverProfile.hpp"
#include "platt2/profiles/QuinnProfile.hpp"

#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/auton/PurpleSkillsAuton.hpp"

#include "pros/motors.hpp"


#include <memory>
#include <vector>
#include <cmath>

namespace platt2{

namespace config{

class PurpleConfig : public IConfig {
    private:
    static constexpr pros::MotorGears DRIVE_GEARSET{pros::MotorGears::blue};

    // ** Drive Ports **
    static constexpr int LEFT_FRONT_MODULE_TOP_PORT{1};
    static constexpr int LEFT_FRONT_MODULE_BOTTOM_PORT{-2};

    static constexpr int RIGHT_FRONT_MODULE_TOP_PORT{-3};
    static constexpr int RIGHT_FRONT_MODULE_BOTTOM_PORT{4};

    static constexpr int LEFT_REAR_MODULE_TOP_PORT{5};
    static constexpr int LEFT_REAR_MODULE_BOTTOM_PORT{-6};

    static constexpr int RIGHT_REAR_MODULE_TOP_PORT{-9};
    static constexpr int RIGHT_REAR_MODULE_BOTTOM_PORT{8};

    // ** Intake Ports **
    static constexpr int FRONT_INTAKE_MOTOR_PORT{10};
    static constexpr int MIDDLE_INTAKE_MOTOR_PORT{15};
    static constexpr int REAR_INTAKE_MOTOR_PORT{13};
    static constexpr int UPPER_CONVEYOR_MOTOR_PORT{14};
    static constexpr pros::MotorGears INTAKE_GEARSET{pros::MotorGears::blue};

    // ** Color Sort Ports **
    static constexpr int OPTICAL_SENSOR_PORT{18};

    // ** Pneumatics **
    static constexpr int ED_MECH_PISTON_PORT{1};
    static constexpr int UPPER_CONVEYOR_HEIGHT_PISTON_PORT{2};
    static constexpr int CONVEYOR_STOPPER_PISTON_PORT{3};

    // ** Linear PID Tuning Values **
    static constexpr double position_dt{0.01};
    static constexpr double position_max{0.4};
    static constexpr double position_min{-0.4};
    static constexpr double position_Kp{0.09};
    static constexpr double position_Kd{0.006};
    static constexpr double position_Ki{0};

    // ** Angular PID Tuning Values **
    static constexpr double heading_dt{0.01};
    static constexpr double heading_max{0.3};
    static constexpr double heading_min{-0.3};
    static constexpr double heading_Kp{0.3};
    static constexpr double heading_Kd{0.0};
    static constexpr double heading_Ki{0};

    public:

    std::shared_ptr<robot::Robot> buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance) override;
};
}
}

#endif