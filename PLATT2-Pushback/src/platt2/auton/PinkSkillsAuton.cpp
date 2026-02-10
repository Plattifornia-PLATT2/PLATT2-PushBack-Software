#include "platt2/auton/PinkSkillsAuton.hpp"
#include "platt2/robot/subsystems/colorSort/ColorSort.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "platt2\robot\subsystems\colorSort\colorSort.hpp"

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
    /*// TODO: Implementation goes here
    //holonomic_subsytem->moveToPoint(0, 24, 90);
    holonomic_subsytem->moveToPoint(54, 19, 90, 0.2, 0.2, 2);
    //intake_subsystem->toggle_rake_mech_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(24.5, 20, 265, 0.3, 0.3, 2.5);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(23.5, 9, 265, 0.2, 0.1, 2);
    //pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1700);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(23.5, 40, 90, 0.4, 0.3, 4);//Score
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_conveyor_stopper_piston();
    color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);

    holonomic_subsytem->moveToPoint(23,21,90, 0.2);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(53.5,69,110);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::OFF);

    holonomic_subsytem->moveToPoint(12,69, 155, 0.2, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(12.5,110, 270, 0.4, 0.6);
    
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(24,101.5, 270, 0.2);//Score 2
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2000);
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(26,115, 95, 0.3,0.3,3);
    color_sort_subsystem->setSortedColor(robot::subsystems::colorsort::BLUE);

    holonomic_subsytem->moveToPoint(24,131, 95, 0.3, 0.3, 3); //reload
    intake_subsystem->move_intake(robot::subsystems::intake::OUT);
    pros::delay(200);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(1700);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);

    holonomic_subsytem->moveToPoint(23,110, 90);

    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(8,129, 225, 0.3, 0.3, 2);
    holonomic_subsytem->moveToPoint(8,112, 225, 0.2, 0.3, 3);
    holonomic_subsytem->moveToPoint(51,127.5, 90);
    pros::delay(300);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(300);
    holonomic_subsytem->moveToPoint(51,115, 90, 0.2);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(300);
    holonomic_subsytem->moveToPoint(48,115, 0);
    holonomic_subsytem->moveToPoint(54,115, 0);

    holonomic_subsytem->moveToPoint(58,85, 315);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(2000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    */
    //Meets at middle goal at 18 and 24 seconds.
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(70, 31, 90);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(70, 28, 90,0.1);
    pros::delay(2000);
    intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(10.75, 30, 190,0.5, 0.3, 4);
    holonomic_subsytem->moveToPoint(10.75, 17, 190,0.3,0.3, 2);
    holonomic_subsytem->moveToPoint(46, 48, 90);
    holonomic_subsytem->moveToPoint(46.5, 73, 85,0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(48, 48, 45,0.3);
    //intake_subsystem->toggle_ed_mech();
    holonomic_subsytem->moveToPoint(59, 61, 45,0.3, 0.3, 2);
    holonomic_subsytem->moveToPoint(38, 84, 45,0.3);
    holonomic_subsytem->moveToPoint(58.5, 86.5, 45,0.3, 0.3, 4);
    holonomic_subsytem->moveToPoint(27, 110, 270,0.3);
    //Tube Point
    holonomic_subsytem->moveToPoint(27, 140, 270, 0.2, 0.3, 2);
    holonomic_subsytem->moveToPoint(12, 110, 90, 0.1, 0.2);
    pros::delay(250);
    holonomic_subsytem->moveToPoint(12, 40, 90);
    holonomic_subsytem->moveToPoint(24, 24, 90, 0.1, 0.2);
    holonomic_subsytem->moveToPoint(24, 24, 90, 0.1, 0.2);
    holonomic_subsytem->moveToPoint(24, 44, 90, 0.2, 0.2, 2);
    holonomic_subsytem->moveToPoint(24, 4, 90, 0.2, 0.2, 2);
    holonomic_subsytem->moveToPoint(24, 44, 90, 0.2, 0.2, 2);












}

}
}