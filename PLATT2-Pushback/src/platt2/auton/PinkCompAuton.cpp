#include "platt2/auton/PinkCompAuton.hpp"
#include "platt2/robot/subsystems/holonomicDrive/HolonomicControl.hpp"
#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
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
    /*//Auto Run 1: Scores one ball in upper Middle goal. Pumps a few blocks from matchloaded into control zone
    holonomic_subsytem->moveToPoint(60, 40, 90, 0.8, 0.4, 2);
    intake_subsystem->toggle_conveyor_stopper_piston();
    intake_subsystem->toggle_ed_mech_piston();
    holonomic_subsytem->moveToPoint(57.25, 58, 42, 0.5, 0.4, 2);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    pros::delay(400);
    intake_subsystem->toggle_ed_mech_piston();
    intake_subsystem->toggle_conveyor_stopper_piston();
    holonomic_subsytem->moveToPoint(24, 28, 90);
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(23, 14.5, 90,0.5, 0.4, 3);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(22.5, 42.5, 90, 0.5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2000);
    holonomic_subsytem->moveToPoint(23, 14.5, 90,0.5, 0.4,2);
    pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(22.5, 42.5, 90, 0.5, 0.4, 3.5);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(12, 36, 90, 0.5, 0.2, 1.5);
    holonomic_subsytem->moveToPoint(13.25, 48, 90, 0.5, 0.2, 1.5);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(13.75, 62, 90, 0.5, 0.2);*/

    //Auto Run 2: Pump and Dump all Blocks into long goal
    holonomic_subsytem->moveToPoint(24, 22, 90);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(23, 12, 90,0.5, 0.4, 2);
    intake_subsystem->toggle_upper_conveyor_height_piston();
    holonomic_subsytem->moveToPoint(22.5, 41, 90, 0.5, 0.4, 2);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2000);
    holonomic_subsytem->moveToPoint(23, 12, 90,0.5, 0.4, 2);
    pros::delay(1500);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(2500);
    holonomic_subsytem->moveToPoint(22.5, 41, 90, 0.5, 0.4, 2);
    intake_subsystem->toggle_conveyor_stopper_piston();
    pros::delay(1500);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(12, 36, 90, 0.5, 0.2, 1.5);
    holonomic_subsytem->moveToPoint(13.25, 48, 90, 0.5, 0.2, 1.5);
    intake_subsystem->toggle_descore_piston();
    holonomic_subsytem->moveToPoint(13.75, 62, 90, 0.5, 0.2);
}

}
}