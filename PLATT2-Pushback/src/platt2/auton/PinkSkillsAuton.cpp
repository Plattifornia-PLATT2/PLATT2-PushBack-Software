#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2\robot\subsystems\colorSort\colorSort.hpp"
#include "pros/rtos.hpp"

namespace platt2{
namespace auton{

void PinkSkillsAuton::init(
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
    this->alliance_color = alliance_color;

    if(odometry_subsystem){
        robot::subsystems::odometry::OdometryPosition startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}

void PinkSkillsAuton::run() {
    //intake_subsystem->toggle_rake_mech_piston();
    //pros::delay(750);
    holonomic_subsytem->moveToPoint(70, 37, 90, 0.3);
    //holonomic_subsytem->moveToPoint(70, 28, 90,0.1);
    //intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(25, 28, 90, 0.5);

    //Near Matchloader
    holonomic_subsytem->moveToPoint(25, 9, 90,0.3, 0.2, 1.5);
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    //intake_subsystem->toggle_rear_intake_piston();
    //pros::delay(1500);
    //intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    //intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(14, 38, 90, 0.3);
    holonomic_subsytem->moveToPoint(14, 110, 90, 0.5);

    //Far Side Long Goal
    //intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(23.5, 110, 270, 0.3);
    holonomic_subsytem->moveToPoint(23.5, 106, 270, 0.3, 0.2, 1.5);
    pros::delay(500);
    //intake_subsystem->toggle_conveyor_stopper_piston();
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    //pros::delay(1500);
    //intake_subsystem->toggle_conveyor_stopper_piston();
    //intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    //Far Matchloader
    holonomic_subsytem->moveToPoint(26, 138, 270, 0.3,0.2, 2);
    pros::delay(500);
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    //intake_subsystem->toggle_rear_intake_piston();
    //pros::delay(1500);
    //intake_subsystem->toggle_rear_intake_piston();
    //intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    //Far Long Goal Side
    holonomic_subsytem->moveToPoint(23.5, 106, 270, 0.3, 0.2, 2);
    pros::delay(500);
    //intake_subsystem->toggle_conveyor_stopper_piston();
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    //pros::delay(1500);
    //intake_subsystem->toggle_conveyor_stopper_piston();
    //intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    //Under Goal Blue Blocks
    holonomic_subsytem->moveToPoint(50, 110, 215, 0.3);
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(12, 72, 215, 0.5);

    //Blue Blocks on Wall    
    holonomic_subsytem->moveToPoint(12, 20, 215, 0.5);

    //Blue Blocks Near
    holonomic_subsytem->moveToPoint(90, 35, 0, 0.5);

    //Line Up for Low Middle Goal
    holonomic_subsytem->moveToPoint(80, 65, 140, 0.3);
    //intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    //pros::delay(1500);

    //Line Up for Park
    holonomic_subsytem->moveToPoint(90, 35, 0, 0.5);
    holonomic_subsytem->moveToPoint(72, 35, 0, 0.5);
    holonomic_subsytem->moveToPoint(72, 24, 0, 0.5);



    




















}

}
}