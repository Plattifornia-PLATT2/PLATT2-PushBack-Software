#ifndef MATCHCONTROLLER_HPP
#define MATCHCONTROLLER_HPP

#include "config/IConfig.hpp"
//#include "config/PinkConfig.hpp"
#include "menu/menu.hpp"
#include "platt2/robot/Robot.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/EAutonConfig.hpp"
#include "platt2/menu/menu.hpp"

#include <memory>
namespace platt2{

class MatchController{
    private:
    robot::AutonConfig auton_config;
    std::unique_ptr<config::IConfig> config{};
    robot::AllianceConfig alliance_config; 
    menu::RobotConfigMenu menu{};
    std::shared_ptr<robot::Robot> robot{};

    public:
    void init();

    void driveControl();

    void autonControl();

};
}

#endif