#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
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

        this->odom_subsystem = std::move(odometry_subsystem);
        this->holonomicDrive_subsystem = std::move(xdrive_subsystem);
    }

}
}