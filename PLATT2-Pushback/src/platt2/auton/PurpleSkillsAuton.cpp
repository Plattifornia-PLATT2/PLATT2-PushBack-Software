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
    holonomic_subsytem->moveToPoint(116, 24, 90,0.5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(116, 12, 90, 0.5, 0.4, 3);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(128, 36, 90, 0.5);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(128, 36, 270, 0.5, 0.2);
    holonomic_subsytem->moveToPoint(130, 105, 270, 0.5);

    //Far Long Goal First Score
    holonomic_subsytem->moveToPoint(115.5, 105, 270, 0.5);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(115.5, 98, 270, 0.5, 0.4, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(114.5, 127, 270, 0.3, 0.4, 2);
    pros::delay(2500);

    //Far Long Goal Second Score
    holonomic_subsytem->moveToPoint(115.5, 98, 270, 0.5, 0.4, 3);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Blue Park Zone Blocks
    holonomic_subsytem->moveToPoint(86, 113.5, 270,0.5);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(68, 113.5, 270, 0.5);
    holonomic_subsytem->moveToPoint(68, 113, 270, 0.5, 0.4, 1.5);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(68,108, 270, 0.5);
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_rake_mech_piston();

    //Red Blocks on Wall
    holonomic_subsytem->moveToPoint(128, 113, 0, 0.5);

    //Red Block in Middle by Long Goal
    holonomic_subsytem->moveToPoint(96, 113, 180, 0.5);
    holonomic_subsytem->moveToPoint(86, 70, 270, 0.5);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    
    //Upper Middle Goal
    intake_subsystem->toggle_ed_mech_piston();
    holonomic_subsytem->moveToPoint(80.75, 79.5, 215, 0.5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);

    //Block Low Middle Goal
    holonomic_subsytem->moveToPoint(82.25, 102, 215, 0.5);
    holonomic_subsytem->moveToPoint(57, 84, 210, 0.5, 0.4, 3.5);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(44, 102, 210, 0.5);





}

}
}