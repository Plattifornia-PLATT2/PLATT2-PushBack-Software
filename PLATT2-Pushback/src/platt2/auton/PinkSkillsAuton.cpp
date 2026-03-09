#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2/robot/subsystems/colorSort/colorSort.hpp"
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


std::string PinkSkillsAuton::getName() const {
    return AUTON_NAME;
}

void PinkSkillsAuton::run() {
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(70, 36.5, 90, 0.5);
    intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(70, 39, 90,0.5);
    holonomic_subsytem->moveToPoint(24, 24, 90, 0.5, 0.2);

    //Near Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(24, 12, 90,0.2, 0.2, 3);
    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(12.25, 38, 95, 0.5);
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(13, 38, 270, 0.5, 0.2);
    holonomic_subsytem->moveToPoint(13, 120, 270);
    
    //Far Side Long Goal
    //holonomic_subsytem->moveToPoint(23, 144, 270, 0.5);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(21.5, 120, 270, 0.4, 0.5);
    holonomic_subsytem->moveToPoint(21.5, 99, 270, 0.4, 0.5, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(24.5, 138, 270, 0.3,0.4, 3);
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP); 

    //Far Long Goal Side
    holonomic_subsytem->moveToPoint(21.25, 99, 270, 0.5, 0.4, 2.5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    intake_subsystem->toggle_rear_intake_piston();

    //Under Goal Blue Blocks
    holonomic_subsytem->moveToPoint(50, 110, 215, 0.5);
    color_sort_subsystem->setSortOption(robot::subsystems::colorsort::RED);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(11.75, 68, 210, 0.5);

    //Blue Blocks on Wall    
    holonomic_subsytem->moveToPoint(11.75, 17.5, 210, 0.5, 0.2, 3.5);

    //Blue Blocks Near Goal
    holonomic_subsytem->moveToPoint(24, 30, 0, 0.5);
    holonomic_subsytem->moveToPoint(90, 30, 0, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);


    //Line Up for Low Middle Goal
    holonomic_subsytem->moveToPoint(81.5, 64, 135, 0.5, 0.4, 4.5);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(3000);
    holonomic_subsytem->moveToPoint(85.5, 6.5, 140, 0.3, 0.2, 4.5);


}

}
}