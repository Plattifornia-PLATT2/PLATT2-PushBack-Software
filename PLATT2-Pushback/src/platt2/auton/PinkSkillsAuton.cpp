#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorSort/colorSort.hpp"
#include "pros/rtos.hpp"

namespace platt2{
namespace auton{

void PinkSkillsAuton::init(
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

    if(odometry_subsystem){
        robot::subsystems::odometry::Position startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}


std::string PinkSkillsAuton::getName() const {
    return AUTON_NAME;
}

void PinkSkillsAuton::run() {
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    tank_subsytem->moveToPoint({56, 40, 90}, false, 0.3, 5);
    tank_subsytem->moveToPoint({35, 60.5, 125}, false, 0.3, 10);
    tank_subsytem->moveToPoint({28, 68.5, 120}, false, 0.1, 5);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    tank_subsytem->moveToPoint({37, 45, 90}, true, 0.3, 5);
    intake_subsystem->toggle_ed_mech_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    tank_subsytem->moveToPoint({61, 61, 45}, false, 0.3, 10);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_ed_mech_piston();
    tank_subsytem->moveToPoint({26,9,90},true, 0.5, 25);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(900);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({27,43,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({27,9,90},true, 0.2, 5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2000);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({27,43,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
}

}
}