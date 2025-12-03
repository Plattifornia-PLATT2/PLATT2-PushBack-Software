#ifndef ROBOT_HPP
#define ROBOT_HPP

// ** PLATT2 Library Includes **
#include "platt2/profiles/DriverProfile.hpp"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorsort/ColorSort.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include "platt2/auton/IAuton.hpp"

// ** PROS API Includes **
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

// ** Standard Library Includes **
#include <memory>
#include <algorithm>
#include <cmath>
#include <utility>


namespace platt2
{
    namespace robot
    {
        class Robot
        {
            private:

            // Subsystems
            std::shared_ptr<subsystems::odometry::Odometry> odom_subsystem;
            std::shared_ptr<subsystems::holonomicDrive::IHolonomic> holonomicDrive_subsystem;
            std::shared_ptr<subsystems::holonomicDrive::HolonomicControl> holonomic_controller;
            std::shared_ptr<subsystems::intake::IntakeSubsystem> intake_subsystem;
            std::shared_ptr<subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem;

            // Auton Class
            std::unique_ptr<auton::IAuton> auton_routine;

            // Config Enums
            AllianceConfig current_alliance;
            RobotConfig current_config;
            AutonConfig current_auton_route;

            // Driver Profile
            std::unique_ptr<profiles::DriverProfile> driver_profile;

            public:
            
            void autonControl();

            void driverControl();

            Robot(std::shared_ptr<subsystems::holonomicDrive::XDrive>& xdrive_subsystem, 
                std::shared_ptr<subsystems::odometry::Odometry>& odometry_subsystem, 
                std::shared_ptr<subsystems::holonomicDrive::HolonomicControl>& holonomic_controller,
                std::shared_ptr<subsystems::intake::IntakeSubsystem>& intake_subsystem,
                platt2::robot::AllianceConfig alliance_config,
                platt2::robot::RobotConfig robot_config,
                platt2::robot::AutonConfig auton_config,
                std::unique_ptr<profiles::DriverProfile>& driver_profile,
                std::unique_ptr<auton::IAuton>& auton_routine,
                std::shared_ptr<subsystems::colorsort::ColorSortSubsystem>& color_sort_subsystem
            );

        };
    };
};

#endif