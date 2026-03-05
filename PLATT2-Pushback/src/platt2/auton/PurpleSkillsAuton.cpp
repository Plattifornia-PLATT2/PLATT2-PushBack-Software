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


std::string PurpleSkillsAuton::getName() const {
    return AUTON_NAME;
}

void PurpleSkillsAuton::run() {
    //Near Matchloader
    holonomic_subsytem->moveToPoint(116, 24, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(116, 13.5, 90, 0.5, 0.4, 3);
    pros::delay(1500);
    holonomic_subsytem->moveToPoint(128, 36, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(128, 36, 270, 0.5, 0.2);
    holonomic_subsytem->moveToPoint(126, 105, 270, 0.5, 0.8l);

    //Far Long Goal First Score
    holonomic_subsytem->moveToPoint(117, 105, 270, 0.15, 1);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    pros::delay(200);
    holonomic_subsytem->moveToPoint(117.5, 99, 270, 0.3, 0.2, 2);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(114.5, 128.5, 270, 0.5, 0.4, 2);
    pros::delay(2500);

    //Far Long Goal Second Score
    holonomic_subsytem->moveToPoint(117.5, 99, 270, 0.6, 0.2, 3);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(250);


    //Blue Park Zone Blocks
    //color_sort_subsystem->setSortOption(robot::subsystems::colorsort::BLUE);
    holonomic_subsytem->moveToPoint(89, 118, 270);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(71.25, 117, 270, 0.5, 1);
    holonomic_subsytem->moveToPoint(71.25,119, 270, 0.5, 0.4, 1);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    holonomic_subsytem->moveToPoint(69.75,109, 270, 0.5, 0.4, 2);
    holonomic_subsytem->moveToPoint(69.75,113, 270, 0.5, 0.4, 2);
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(1000);
    intake_subsystem->toggle_rear_intake_piston();

    //Red Blocks on Wall
    holonomic_subsytem->moveToPoint(132, 115, 0, 0.8, 0.4, 3);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(750);

    //Red Block in Middle by Long Goal
    holonomic_subsytem->moveToPoint(101, 113, 180);
    holonomic_subsytem->moveToPoint(96, 70, 270);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    
    //Upper Middle Goal
    holonomic_subsytem->moveToPoint(90, 82.25, 215);
    holonomic_subsytem->moveToPoint(81, 82.5, 215, 0.5);
    intake_subsystem->toggle_ed_mech_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);

    //Block Low Middle Goal
    holonomic_subsytem->moveToPoint(84.25, 110, 215);
    holonomic_subsytem->moveToPoint(60, 85.5, 215, 0.8, 0.4, 2);
    pros::delay(1500);
    holonomic_subsytem->moveToPoint(46, 102, 215, 0.8);





}

}
}