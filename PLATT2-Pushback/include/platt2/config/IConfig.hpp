#ifndef ICONFIG_HPP
#define ICONFIG_HPP

#include <memory>

#include "platt2/robot/Robot.hpp"

namespace platt2{

namespace config{

    class IConfig{
        public:

        virtual ~IConfig() = default;

        virtual std::shared_ptr<robot::Robot> buildRobot() = 0;
    };
}
}

#endif