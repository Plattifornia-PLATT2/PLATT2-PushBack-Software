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

    if(odometry_subsystem){
        robot::subsystems::odometry::OdometryPosition startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}

void PurpleSkillsAuton::run() {
    //Near Matchloader
    holonomic_subsytem->moveToPoint(116, 24, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(116, 13.75, 90, 0.5, 0.4, 3);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(128.5, 36, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(128.5, 36, 270, 0.5, 0.2);
    holonomic_subsytem->moveToPoint(128.5, 109, 270, 0.5);

    //Far Long Goal First Score
    holonomic_subsytem->moveToPoint(115.75, 109, 270, 0.35);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(115.75, 99, 270, 0.5, 1, 2);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(114, 128.5, 270, 0.5, 0.4, 2);
    pros::delay(2500);

    //Far Long Goal Second Score
    holonomic_subsytem->moveToPoint(115.75, 99, 270, .5, 1, 3);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Blue Park Zone Blocks
    holonomic_subsytem->moveToPoint(86, 115.5, 270);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(67, 115.5, 270, 0.5);
    holonomic_subsytem->moveToPoint(67,117, 270, 0.5, 0.4, 1.5);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(67,109, 270, 0.5, 0.4, 2);
    holonomic_subsytem->moveToPoint(67,113, 270, 0.5, 0.4, 2);
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_rake_mech_piston();

    //Red Blocks on Wall
    holonomic_subsytem->moveToPoint(128.5, 115, 0);
    pros::delay(750);

    //Red Block in Middle by Long Goal
    holonomic_subsytem->moveToPoint(98, 113, 180);
    holonomic_subsytem->moveToPoint(89.5, 70, 270);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    
    //Upper Middle Goal
    
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(81, 80.5, 215, 0.5);
    intake_subsystem->toggle_ed_mech_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);

    //Block Low Middle Goal
    holonomic_subsytem->moveToPoint(82.25, 110, 215);
    holonomic_subsytem->moveToPoint(52.5, 85.5, 215, 0.8, 0.4, 3.5);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(44, 102, 215, 0.8);





}

}
}