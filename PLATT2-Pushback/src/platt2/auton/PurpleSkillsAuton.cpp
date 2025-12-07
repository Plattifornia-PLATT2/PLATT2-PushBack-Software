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
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(88, 19, 270, 0.2, 0.2);
    intake_subsystem->toggle_rake_mech_piston();
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
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(750);
    holonomic_subsytem->moveToPoint(119,40, 90, 0.3, 0.3, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1250);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(300);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1250);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(110,22, 90);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(98,57.5, 45);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);
    holonomic_subsytem->moveToPoint(98,69, 45);
    holonomic_subsytem->moveToPoint(98,69, 0);
    pros::delay(1000);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::OFF);
    holonomic_subsytem->moveToPoint(132,69, 0, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(132,105, 180);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(119,105, 270);
    holonomic_subsytem->moveToPoint(119,100, 270);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_conveyor_stopper_piston();





}

}
}