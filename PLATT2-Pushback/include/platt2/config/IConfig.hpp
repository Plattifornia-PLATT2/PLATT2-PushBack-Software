#ifndef ICONFIG_HPP
#define ICONFIG_HPP

#include <memory>

#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/EDriverConfig.hpp"
#include "platt2/robot/Robot.hpp"

namespace platt2{

namespace config{

    class IConfig{
        public:

        virtual ~IConfig() = default;

        virtual std::shared_ptr<robot::Robot> buildRobot(robot::AutonConfig auton, robot::DriverProfile profile, robot::AllianceConfig alliance) = 0;
    };
}
}

#endif