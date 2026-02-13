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
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(500);
    intake_subsystem->toggle_rear_intake_piston();
    holonomic_subsytem->moveToPoint(96, 48, 270, 0.3);
    holonomic_subsytem->moveToPoint(96, 75, 270, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(96, 55, 220, 0.3, 0.3, 3);
    holonomic_subsytem->moveToPoint(76, 57, 130, 0.3, 0.3, 3);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(3000);
    holonomic_subsytem->moveToPoint(96, 55, 270, 0.3);
    holonomic_subsytem->moveToPoint(96, 85, 270, 0.3);
    holonomic_subsytem->moveToPoint(64, 100, 270, 0.3, 0.3, 4);
    holonomic_subsytem->moveToPoint(64, 117, 270, 0.3, 0.3);
    pros::delay(250);
    intake_subsystem->toggle_rake_mech_piston();
    pros::delay(500);
    intake_subsystem->move_intake(robot::subsystems::intake::IN);
    holonomic_subsytem->moveToPoint(64, 107, 270, 0.2, 0.9);
    holonomic_subsytem->moveToPoint(64, 112, 270, 0.2, 0.9);
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    pros::delay(1500);
    intake_subsystem->toggle_rear_intake_piston();
    intake_subsystem->toggle_rake_mech_piston();
    holonomic_subsytem->moveToPoint(79, 80, 220, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(96, 55, 220, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(76, 57, 135, 0.3, 0.3);
    intake_subsystem->move_intake(robot::subsystems::intake::OUT_LOW_GOAL);
    pros::delay(3000);
    intake_subsystem->move_intake(robot::subsystems::intake::STOP);
    holonomic_subsytem->moveToPoint(94, 65, 305, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(94, 112, 270, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(112, 112, 270, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(112, 126, 270, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(122, 110, 270, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(122, 48, 270, 0.3, 0.3);
    holonomic_subsytem->moveToPoint(112, 24, 270, 0.3, 0.3);
    







}

}
}