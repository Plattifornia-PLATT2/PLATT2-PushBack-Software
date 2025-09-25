#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/ERobotConfig.hpp"
#include <algorithm>
#include <memory>

namespace platt2{

namespace robot{

    Robot::Robot()
    {
        current_alliance = NO_ALLIANCE;
        current_auton_route = NO_AUTON;
        current_config = NO_ROBOT;
    }
}
}