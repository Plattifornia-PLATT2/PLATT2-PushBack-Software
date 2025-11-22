#include "platt2/auton/PurpleCompAuton.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::Init(
    std::shared_ptr<robot::subsystems::holonomicDrive::IHolonomic> holonomic_subsytem, 
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem
) {
    this->holonomic_subsytem = holonomic_subsytem;
    this->odometry_subsystem = odometry_subsystem;
    this->intake_subsystem = intake_subsystem;
}

void PurpleCompAuton::run() {
    // TODO: Implementation goes here
}

}
}