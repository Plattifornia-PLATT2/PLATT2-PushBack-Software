#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"

namespace platt2{
namespace auton{

void PinkCompAuton::init(
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem
) {
    this->holonomic_subsytem = holonomic_subsytem;
    this->odometry_subsystem = odometry_subsystem;
    this->intake_subsystem = intake_subsystem;
    this->color_sort_subsystem = color_sort_subsystem;
}

void PinkCompAuton::run() {
    // TODO: Implementation goes here
    holonomic_subsytem->moveToPoint(0, 24, 90);
    holonomic_subsytem->moveToPoint(-24, 24,0);

}

}
}