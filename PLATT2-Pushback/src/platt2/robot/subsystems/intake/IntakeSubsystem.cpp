#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    IntakeSubsystem::IntakeSubsystem(
        std::unique_ptr<pros::Motor>front_intake, 
        std::unique_ptr<pros::Motor>rear_intake, 
        std::unique_ptr<pros::Motor>mid_intake, 
        std::unique_ptr<pros::Motor>upper_conveyor,
        std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston,
        std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston,
        std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston
    )
        {
        front_intake_motor = std::move(front_intake);
        rear_intake_motor = std::move(rear_intake);
        middle_intake_motor = std::move(mid_intake);
        upper_conveyor_motor = std::move(upper_conveyor);
        this->ed_mech_piston = std::move(ed_mech_piston);
        this->upper_conveyor_height_piston = std::move(upper_conveyor_height_piston);
        this->conveyor_stopper_piston = std::move(conveyor_stopper_piston);
    }

    void IntakeSubsystem::move_intake(IntakeDirection direction){
        switch(direction){
            case IN:{
                front_intake_motor->move_velocity(intake_speed);
                middle_intake_motor->move_velocity(intake_speed);
                rear_intake_motor->move_velocity(intake_speed);
                upper_conveyor_motor->move_velocity(intake_speed);
                break;
            }
            case OUT:{
                front_intake_motor->move_velocity(-intake_speed);
                middle_intake_motor->move_velocity(-intake_speed);
                rear_intake_motor->move_velocity(-intake_speed);
                upper_conveyor_motor->move_velocity(-intake_speed);
                break;   
            }
            case OUT_LOW_GOAL:{
                front_intake_motor->move_velocity(-intake_speed);
                middle_intake_motor->move_velocity(-intake_speed);
                rear_intake_motor->move_velocity(intake_speed);
                upper_conveyor_motor->move_velocity(-intake_speed);   
                break;
            }
            case STOP:{
                front_intake_motor->move_velocity(0);
                middle_intake_motor->move_velocity(0);
                rear_intake_motor->move_velocity(0);
                upper_conveyor_motor->move_velocity(0);
                break;
            }
        }
    }

    void IntakeSubsystem::toggle_ed_mech_piston(){
        ed_mech_piston_state = !ed_mech_piston_state;
        ed_mech_piston->set_value(ed_mech_piston_state);
    }

    void IntakeSubsystem::toggle_upper_conveyor_height_piston(){
        upper_conveyor_height_piston_state = !upper_conveyor_height_piston_state;
        upper_conveyor_height_piston->set_value(upper_conveyor_height_piston_state);
    }

    void IntakeSubsystem::toggle_conveyor_stopper_piston(){
        conveyor_stopper_piston_state = !conveyor_stopper_piston_state;
        conveyor_stopper_piston->set_value(conveyor_stopper_piston_state);
    }

}
}
}
}