#include "platt2/auton/PinkPumpnDumpAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"


namespace platt2{
namespace auton{

void PinkPumpnDumpAuton::init(
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


std::string PinkPumpnDumpAuton::getName() const {
    return AUTON_NAME;
}

void PinkPumpnDumpAuton::run() {
    tank_subsytem->moveToPoint({42,40,90},false, 0.3, 8);
 tank_subsytem->moveToPoint({116.5,9.25,90},true, 0.5, 35);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1250);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117,43,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({116.5,9,90},true, 0.2, 5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(2000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2000);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117,43,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Descore
    tank_subsytem->moveToPoint({106,24,60},true, 0.2, 8);
    intake_subsystem->toggle_descore_piston();
    tank_subsytem->moveToPoint({106,47,90},false, 0.5, 5);
    intake_subsystem->toggle_descore_piston();
}
}
}