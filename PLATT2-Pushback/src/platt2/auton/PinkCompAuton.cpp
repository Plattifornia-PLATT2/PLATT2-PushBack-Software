#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"

namespace platt2{
namespace auton{

void PinkCompAuton::init(
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
    this->alliance_color= alliance_color;

    if(odometry_subsystem){
        robot::subsystems::odometry::OdometryPosition startingPos;
        startingPos.x = STARTING_X_POSITION;
        startingPos.y = STARTING_Y_POSITION;
        startingPos.heading = STARTING_HEADING;
        odometry_subsystem->setPos(startingPos);
    }
}

void PinkCompAuton::run() {
    // TODO: Implementation goes here
    //Near Matchloader
    holonomic_subsytem->moveToPoint(120, 24, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(120, 16, 90, 0.7, 0.4, 3);
    holonomic_subsytem->moveToPoint(132, 24, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(132, 24, 270);
    holonomic_subsytem->moveToPoint(132, 120, 270);

    //Far Long Goal First Score
    holonomic_subsytem->moveToPoint(120, 120, 270);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(121, 105, 270, 0.7, 0.4, 3);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Far Matchloader
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(120, 144, 270);
    pros::delay(2500);

    //Far Long Goal Second Score
    holonomic_subsytem->moveToPoint(121, 105, 270, 0.7, 0.4, 3);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(3000);
    intake_subsystem->toggle_conveyor_stopper_piston();

    //Blue Park Zone Blocks
    holonomic_subsytem->moveToPoint(110, 120, 270);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(72, 120, 270);
    intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(72,115, 270);
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_rake_mech_piston();

    //Red Blocks on Wall
    holonomic_subsytem->moveToPoint(144, 120, 180);

    //Red Block in Middle by Long Goal
    holonomic_subsytem->moveToPoint(96, 120, 180);
    holonomic_subsytem->moveToPoint(96, 70, 270);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    
    //Upper Middle Goal
    intake_subsystem->toggle_ed_mech_piston();
    holonomic_subsytem->moveToPoint(82.25, 86.25, 210);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(2500);

    //Block Low Middle Goal
    holonomic_subsytem->moveToPoint(82.25, 102, 215);
    holonomic_subsytem->moveToPoint(68.25, 86.25, 210);
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(66, 102, 210);
}

}
}