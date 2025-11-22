#include "platt2/MatchController.hpp"
#include "EAllianceConfig.hpp"
#include "ERobotConfig.hpp"
#include "platt2/config/PinkConfig.hpp"
#include "platt2/config/PurpleConfig.hpp"
#include "platt2/menu/menu.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace platt2{

void MatchController::init(){
    menu.build();

    while(menu.menuInUse()){
        pros::delay(10);
    }

    menu::FullConfig roboConfig =  menu.getFullConfig();

    if(roboConfig.robotConfig == robot::PINK) {
        std::unique_ptr<config::PinkConfig> pink{std::make_unique<config::PinkConfig>()};
        config = std::move(pink);
    }
    else if (roboConfig.robotConfig == robot::PURPLE) {
        std::unique_ptr<config::PurpleConfig> purple{std::make_unique<config::PurpleConfig>()};
        config = std::move(purple);
    }

    robot = config->buildRobot(roboConfig.autonConfig, roboConfig.driverProfile, robot::NO_ALLIANCE);
}

void MatchController::driveControl(){
   robot->driverControl();
}

void MatchController::autonControl(){
   robot->autonControl();
}
}