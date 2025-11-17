#include "platt2/MatchController.hpp"
#include "platt2/config/PinkConfig.hpp"
#include <memory>

namespace platt2{

void MatchController::init(){
    std::unique_ptr<config::PinkConfig> pink{std::make_unique<config::PinkConfig>()};
    config = std::move(pink);
    robot = config->buildRobot();
}

void MatchController::driveControl(){
    robot->driverControl();
}

void MatchController::autonControl(){
    std::cout<<"i want to die"<<std::endl;
   robot->autonControl();
}
}