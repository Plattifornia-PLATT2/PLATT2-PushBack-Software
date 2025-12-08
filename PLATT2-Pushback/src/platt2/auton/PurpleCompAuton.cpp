#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::init(
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

void PurpleCompAuton::run() {
    holonomic_subsytem->moveToPoint(88, 6, 270, 0.2, 0.2);
    holonomic_subsytem->moveToPoint(88, 6, 0, 0.2, 0.2);

    //holonomic_subsytem->moveToPoint(118, 12, 270, 0.3);
    
    // TODO: Implementation goes here
}

}
}