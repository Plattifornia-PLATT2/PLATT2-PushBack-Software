#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::init(
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
    this->alliance_color = alliance_color;
    if(alliance_color == robot::RED){
        rejected_color = robot::subsystems::colorsort::BLUE;
    }
    else if (alliance_color == robot::BLUE){
        rejected_color = robot::subsystems::colorsort::RED;
    }

    if(odometry_subsystem){
        robot::subsystems::odometry::Position startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}


std::string PurpleCompAuton::getName() const {
    return AUTON_NAME;
}

void PurpleCompAuton::run() {

    tank_subsytem->moveToPoint({79.5,61.5,137},false, 0.5, 15);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(750);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    tank_subsytem->moveToPoint({115.5,10,90},true, 0.4, 15);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1100);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117,44,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();   
     intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(100);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117, 10,90},true, 0.2, 5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117.5,44,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Descore
    tank_subsytem->moveToPoint({106.85,24,85},true, 0.2, 5);
    intake_subsystem->toggle_descore_piston();
    tank_subsytem->moveToPoint({106.85,47,90},false, 0.4, 5);
    intake_subsystem->toggle_descore_piston();
    tank_subsytem->moveToPoint({106.85,62,90},false, 0.25, 5);


}

}
}