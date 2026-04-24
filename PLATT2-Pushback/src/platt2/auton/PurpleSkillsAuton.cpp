#include "platt2/auton/PurpleSkillsAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/rtos.hpp"
namespace platt2{
namespace auton{

void PurpleSkillsAuton::init(
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


std::string PurpleSkillsAuton::getName() const {
    return AUTON_NAME;
}

void PurpleSkillsAuton::run() {
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    tank_subsytem->moveToPoint({86, 40, 70}, false , .3, 5);
    tank_subsytem->moveToPoint({105, 55, 60}, false , .3, 5);
    tank_subsytem->moveToPoint({114, 67.5, 60}, false , .1, 8);
    tank_subsytem->moveToPoint({96, 45,90}, true, .5, 15);
    tank_subsytem->moveToPoint({80,62,137},false, 0.5, 10);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    tank_subsytem->moveToPoint({115.5,10,90},true, 0.4,15);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1250);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117,43.5,90},false, 0.2, 5);
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
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({117.5,43.5,90},false, 0.2, 5);
    intake_subsystem->toggle_conveyor_stopper_piston();
}

}
}