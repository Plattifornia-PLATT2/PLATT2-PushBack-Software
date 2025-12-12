#include "platt2/auton/PurpleSkillsAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/rtos.hpp"
namespace platt2{
namespace auton{

void PurpleSkillsAuton::init(
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
}

void PurpleSkillsAuton::run() {
    // TODO: Implementation goes here
    //intake_subsystem->toggle_rake_mech_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(88, 19, 270, 0.2, 0.2);
    //intake_subsystem->toggle_rake_mech_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(118, 20, 275);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(118, 7.5, 275, 0.2, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(119, 12, 270, 0.15, 0.15, 3);
    holonomic_subsytem->moveToPoint(119,39.5, 90, 0.3, 0.15, 3);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    //intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    //pros::delay(300);
    //intake_subsystem->move_intake(robot::subsystems::intake::IN);
    //pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();

    holonomic_subsytem->moveToPoint(119,34, 90,0.3,0.15);
    pros::delay(200);
    /*holonomic_subsytem->moveToPoint(109,34, 90);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(109,46, 90);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(109,50, 90);
   intake_subsystem->toggle_descore_piston(); */
    holonomic_subsytem->moveToPoint(110,22, 90);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(93,57.5, 45,0.3,0.15);
    pros::delay(200);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::RED);
    holonomic_subsytem->moveToPoint(93,67, 45, 0.15);
    pros::delay(200);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::OFF);
    holonomic_subsytem->moveToPoint(130,67, 25, 0.15);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(130,110, 0, 0.2, 0.15);
    pros::delay(200);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(118.5,108, 270,0.3,.15);
    holonomic_subsytem->moveToPoint(118.5,101.5, 270,0.2, 0.15, 3);//score 2
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(119,130.5, 90, .2,.2); // reload
    pros::delay(100);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(119.5,120, 90);
    holonomic_subsytem->moveToPoint(132,127.5, 325); // get 2 side balls 
    holonomic_subsytem->moveToPoint(132,105, 325);
    holonomic_subsytem->moveToPoint(88,128.5, 270, 0.2,0.2);
    pros::delay(200);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(87.25,115, 270, 0.1);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(95,115, 180, 0.1);
    holonomic_subsytem->moveToPoint(70,115, 180, 0.4, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_ed_mech_piston();
    holonomic_subsytem->moveToPoint(88 ,87, 225);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(84 ,83.5, 225);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    //holonomic_subsytem->moveToPoint(93,82, 135);
    //intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    //holonomic_subsytem->moveToPoint(93,60, 315);
    //holonomic_subsytem->moveToPoint(83, 58, 225);















}

}
}