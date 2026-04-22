#include "platt2/auton/PurplePumpnDumpAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

namespace platt2{
namespace auton{

void PurplePumpnDumpAuton::init(
    std::shared_ptr<robot::subsystems::tankDrive::TankControl> tank_subsytem, 
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem,
    robot::AllianceConfig alliance_color
) {
    this->tank_subsytem = tank_subsytem;
    this->odometry_subsystem = odometry_subsystem;
    this->intake_subsystem = intake_subsystem;
    this->color_sort_subsystem = color_sort_subsystem;
    this->alliance_color= alliance_color;

    if(odometry_subsystem){
        robot::subsystems::odometry::Position startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}


std::string PurplePumpnDumpAuton::getName() const {
    return AUTON_NAME;
}

void PurplePumpnDumpAuton::run() {


    tank_subsytem->moveToPoint({117,40,90},false, 0.3, 8);

    tank_subsytem->moveToPoint({117,9.5,90},true, 0.2, 5);

    tank_subsytem->moveToPoint({118,42,90},false, 0.2, 5);
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    tank_subsytem->moveToPoint({116.5,9.5,90},true, 0.2, 5);
    tank_subsytem->moveToPoint({118,42,90},false, 0.2, 5);

    //Descore
    tank_subsytem->moveToPoint({106.5,24,60},true, 0.2, 8);
    tank_subsytem->moveToPoint({106.5,44,90},false, 0.2, 5);
    tank_subsytem->moveToPoint({106.5,62,90},false, 0.2, 5);

}

}}