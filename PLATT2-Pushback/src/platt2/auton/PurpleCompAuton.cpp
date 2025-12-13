#include "platt2/auton/PurpleCompAuton.hpp"
#include "platt2/EAllianceConfig.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace auton{

void PurpleCompAuton::init(
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

void PurpleCompAuton::run() {
    
        // TODO: Implementation goes here
    //intake_subsystem->toggle_rake_mech_piston();
    if (alliance_color == robot::BLUE){
        color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::RED);
    }else{
         color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);
    }
    
    pros::delay(200);
    //intake_subsystem->toggle_rake_mech_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(119.5, 20, 275, .4, .15, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(119.25, 8.25, 275, 0.2, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1000);
    //intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(119.5, 12, 270, 0.15, 0.15, 2);
    holonomic_subsytem->moveToPoint(119,39.5, 90, 0.3, 0.15, 2.5);
    pros::delay(200);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(119.25, 8.25, 275, 0.3, 0.3, 3);
    pros::delay(1500);
    holonomic_subsytem->moveToPoint(119.25, 12, 270, 0.15, 0.15, 2.5);
    holonomic_subsytem->moveToPoint(120,39.5, 90, 0.3, 0.15, 2.5);
    pros::delay(200);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(119.25, 8, 275, 0.3, 0.3, 3);
    pros::delay(1000);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    /*
    pros::delay(1000);    
    color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::OFF);
    holonomic_subsytem->moveToPoint(86, 57, 130, 0.3, 0.3, 5);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
     pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    */
    //holonomic_subsytem->moveToPoint(118, 12, 270, 0.3);
    
    // TODO: Implementation goes here
}

}
}