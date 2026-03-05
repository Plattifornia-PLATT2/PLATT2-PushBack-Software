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
    holonomic_subsytem->moveToPoint(24, 28, 90, 0.5, 0.2, 2.5);

    //Near Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(23, 16, 90,0.2, 0.2, 3);
    pros::delay(2500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(10.75, 38, 95, 0.5);
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(10.75, 38, 270, 0.5, 0.2);
    holonomic_subsytem->moveToPoint(10.75, 120, 270);

    //Far Side Long Goal
    //holonomic_subsytem->moveToPoint(23, 144, 270, 0.5);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(23, 120, 270, 0.4, 0.5);
    holonomic_subsytem->moveToPoint(22.75, 104, 270, 0.4, 0.5, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(24.5, 144, 270, 0.5,0.4, 3);
    pros::delay(3000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP); 

    //Far Long Goal Side
    holonomic_subsytem->moveToPoint(22.75, 105 , 270, 0.5, 0.4, 2.5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    intake_subsystem->toggle_rear_intake_piston();

    //Under Goal Blue Blocks
    holonomic_subsytem->moveToPoint(50, 110, 215, 0.5);
    //color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::RED);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(10, 68, 215, 0.5);

    //Blue Blocks on Wall    
    holonomic_subsytem->moveToPoint(10, 29, 215, 0.5, 0.2, 3.5);

    //Blue Blocks Near Goal
    holonomic_subsytem->moveToPoint(24, 35.5, 0, 0.5);
    holonomic_subsytem->moveToPoint(90, 37, 0, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);


    //Line Up for Low Middle Goal
    holonomic_subsytem->moveToPoint(82.25, 68.25, 140, 0.5, 0.4, 4.5);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(3000);
    holonomic_subsytem->moveToPoint(85.5, 6.5, 140, 0.3, 0.2, 4.5);


}

}
}