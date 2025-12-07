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
    holonomic_subsytem->moveToPoint(119, 7, 270, 0.2, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1200);
    holonomic_subsytem->moveToPoint(120,39, 90, 0.3, 0.3, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(110,22, 90);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(92,57.5, 45);
    holonomic_subsytem->moveToPoint(92,69, 45);
    holonomic_subsytem->moveToPoint(92,69, 0);
    holonomic_subsytem->moveToPoint(132,69, 0, 0.15);
    holonomic_subsytem->moveToPoint(132,110, 180);


}

}
}