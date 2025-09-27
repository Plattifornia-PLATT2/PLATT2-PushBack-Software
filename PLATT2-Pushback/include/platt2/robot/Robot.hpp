#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "api.h"
#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "subsystems/holonomicDrive/IHolonomicDrive.hpp"
#include "subsystems/holonomicDrive/XDrive.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include <memory>

namespace platt2
{
    namespace robot
    {
        class Robot
        {
            private:

            std::unique_ptr<subsystems::odometry::Odometry> odom_subsystem;
            std::unique_ptr<subsystems::holonomicDrive::IHolonomic>  holonomicDrive_subsystem;
            AllianceConfig current_alliance;
            RobotConfig current_config;
            AutonConfig current_auton_route;

            public:
            
            void autonControl();

            void driverControl();

            Robot(std::unique_ptr<subsystems::holonomicDrive::XDrive> xdrive_subsystem, std::unique_ptr<subsystems::odometry::Odometry> odometry_subsystem);

        };
    };
};

#endif