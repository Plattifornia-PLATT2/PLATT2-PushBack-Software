#include "platt2/auton/PurpleSkillsAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/rtos.hpp"
namespace platt2{
namespace auton{

void PurpleSkillsAuton::init(
    std::shared_ptr<robot::subsystems::holonomicDrive::HolonomicControl> holonomic_subsytem, 
    std::shared_ptr<robot::subsystems::odometry::Odometry> odometry_subsystem, 
    std::shared_ptr<robot::subsystems::intake::IntakeSubsystem> intake_subsystem,
    std::shared_ptr<robot::subsystems::colorsort::ColorSortSubsystem> color_sort_subsystem  
) {
    this->holonomic_subsytem = holonomic_subsytem;
    this->odometry_subsystem = odometry_subsystem;
    this->intake_subsystem = intake_subsystem;
    this->color_sort_subsystem = color_sort_subsystem;
}

void PurpleSkillsAuton::run() {
    // TODO: Implementation goes here
    //intake_subsystem->toggle_rake_mech_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(88, 19, 270, 0.2, 0.2);
    //intake_subsystem->toggle_rake_mech_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(119, 20, 270);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(119, 7.5, 270, 0.2, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(750);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(750);
    holonomic_subsytem->moveToPoint(119,40, 90, 0.3, 0.3, 3);
    pros::delay(200);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(300);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(119,34, 90);
    pros::delay(200);
    /*holonomic_subsytem->moveToPoint(109,34, 90);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(109,46, 90);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(109,50, 90);
   intake_subsystem->toggle_descore_piston(); */
    holonomic_subsytem->moveToPoint(110,22, 90);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(93,57.5, 45);
    pros::delay(200);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);
    holonomic_subsytem->moveToPoint(93,69, 45);
    pros::delay(200);
    holonomic_subsytem->moveToPoint(93,69, 0);
    pros::delay(1000);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::OFF);
    holonomic_subsytem->moveToPoint(132,69, 0, 0.15);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(132,105, 180, 0.3, 0.15);
    pros::delay(200);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(119,105, 270);
    pros::delay(200);
    holonomic_subsytem->moveToPoint(119,101, 270,0.3, 0.15);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(119,130, 90);
    pros::delay(100);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(133,127.5, 325);
    holonomic_subsytem->moveToPoint(133,105, 325);
    holonomic_subsytem->moveToPoint(88,134, 270);
    intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(88,115, 270, 0.1);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(60,115, 180);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_ed_mech_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(85.5 ,82, 225);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(93,69, 135);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    holonomic_subsytem->moveToPoint(85.5, 46, 315
    );















}

}
}