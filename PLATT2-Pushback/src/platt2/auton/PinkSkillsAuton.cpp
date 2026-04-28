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
        
    uint32_t now = pros::millis();

    tank_subsytem->moveToPoint({42,40,90},false, 0.4, 10);
    tank_subsytem->moveToPoint({28,9,90},true, 0.4, 10);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(900);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({26.5,42,90},false, 0.4, 4);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    tank_subsytem->moveToPoint({27,9,90},true, 0.4, 5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2700);
    intake_subsystem->move_intake(robot::subsystems::intake::TOP_REV);
    pros::delay(250);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    intake_subsystem->toggle_rear_intake_piston();

    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_ed_mech_piston();
    tank_subsytem->moveToPoint({62, 58.5, 45}, false, 0.5, 5);

    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2200);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    intake_subsystem->toggle_upper_conveyor_height_piston();

    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_ed_mech_piston();
    pros::delay(250);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    

    std::cout << "Time taken: " << (pros::millis() - now) / 1000.0 << " seconds" << std::endl;  
}

}
}