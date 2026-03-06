#include "platt2/auton/PurplePumpnDumpAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

namespace platt2{
namespace auton{

void PurplePumpnDumpAuton::init(
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem,
    robot::AllianceConfig alliance_color
) {
    this->holonomic_subsytem = holonomic_subsytem;
    this->odometry_subsystem = odometry_subsystem;
    this->intake_subsystem = intake_subsystem;
    this->color_sort_subsystem = color_sort_subsystem;
    this->alliance_color= alliance_color;

    if(odometry_subsystem){
        robot::subsystems::odometry::OdometryPosition startingPos;
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
    //Auto Run 2: Pump and Dump all Blocks into long goal
    holonomic_subsytem->moveToPoint(117.75, 24, 90, 0.7);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(117.75, 13, 90, 0.3, 0.4, 3);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(117, 42, 90, 0.5, 0.4, 3);
    intake_subsystem->toggle_conveyor_stopper_piston(),
    pros::delay(1500);
    holonomic_subsytem->moveToPoint(117.75, 12.5, 90, 0.3, 0.4, 3);
    pros::delay(2750);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(3000);
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(117.5, 42, 90, 0.5, 0.4, 3);
    intake_subsystem->toggle_conveyor_stopper_piston(),
    pros::delay(4000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(108.75, 38, 90, 0.5, 0.2, 2);
    holonomic_subsytem->moveToPoint(108.75, 48, 90, 0.5, 0.2, 2);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(109, 62, 90, 0.8, 0.2, 2);
}

}}