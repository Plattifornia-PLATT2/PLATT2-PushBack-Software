#include "platt2/robot/subsystems/intake/IntakeSubsystem.hpp"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <memory>

namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    IntakeSubsystem::IntakeSubsystem(
        std::unique_ptr<pros::Motor>front_intake, 
        std::unique_ptr<pros::Motor>rear_intake, 
        std::unique_ptr<pros::Motor>mid_intake, 
        std::unique_ptr<pros::Motor>upper_conveyor,
        std::unique_ptr<pros::Motor>lower_roller,
        std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston,
        std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston,
        std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston,
        std::unique_ptr<pros::adi::DigitalOut> rake_mech_piston,
        std::unique_ptr<pros::adi::DigitalOut> descore_piston,
        std::unique_ptr<pros::Distance> distance_sensor
    )
        {
        front_intake_motor = std::move(front_intake);
        rear_intake_motor = std::move(rear_intake);
        middle_intake_motor = std::move(mid_intake);
        upper_conveyor_motor = std::move(upper_conveyor);
        lower_roller_motor = std::move(lower_roller);
        this->ed_mech_piston = std::move(ed_mech_piston);
        this->upper_conveyor_height_piston = std::move(upper_conveyor_height_piston);
        this->conveyor_stopper_piston = std::move(conveyor_stopper_piston);
        this->rake_mech_piston = std::move(rake_mech_piston);
        this->descore_piston = std::move(descore_piston);
        this->distance_sensor = std::move(distance_sensor);

        if(rear_intake_motor){
            rear_intake_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        }

    }

    void IntakeSubsystem::move_intake(IntakeDirection direction){
        switch(direction){
            case IN:{
                if(colorSort){
                    front_intake_motor->move_velocity(intake_speed);
                    middle_intake_motor->move_velocity(intake_speed);
                    rear_intake_motor->move_velocity(-intake_speed);
                    upper_conveyor_motor->move_velocity(intake_speed);
                    lower_roller_motor->move_velocity(intake_speed);
                }
                else{
                    front_intake_motor->move_velocity(intake_speed);
                    middle_intake_motor->move_velocity(intake_speed);
                    rear_intake_motor->move_velocity(intake_speed);
                    upper_conveyor_motor->move_velocity(intake_speed);
                    lower_roller_motor->move_velocity(intake_speed);
                }

                break;
            }
            case OUT:{
                front_intake_motor->move_velocity(-intake_speed);
                middle_intake_motor->move_velocity(-intake_speed);
                rear_intake_motor->move_velocity(-intake_speed);
                upper_conveyor_motor->move_velocity(-intake_speed);
                lower_roller_motor->move_velocity(-intake_speed);
                break;   
            }
            case OUT_LOW_GOAL:{
                front_intake_motor->move_velocity(-intake_speed/2);
                middle_intake_motor->move_velocity(-intake_speed/2);
                rear_intake_motor->move_velocity(-intake_speed/2);
                upper_conveyor_motor->move_velocity(-intake_speed/2);   
                lower_roller_motor->move_velocity(-intake_speed/2);
                break;
            }
            case STOP:{
                front_intake_motor->move_velocity(0);
                middle_intake_motor->move_velocity(0);
                rear_intake_motor->move_velocity(0);
                upper_conveyor_motor->move_velocity(0);
                lower_roller_motor->move_velocity(0);   
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

    void IntakeSubsystem::toggle_descore_piston(){
        descore_piston_state = !descore_piston_state;
        descore_piston->set_value(descore_piston_state);
    }

    void IntakeSubsystem::toggle_rake_mech_piston(){
        rake_mech_piston_state = !rake_mech_piston_state;
        rake_mech_piston->set_value(rake_mech_piston_state);
    }

    void IntakeSubsystem::move_rear_motor(IntakeDirection direction){
        switch(direction){
            case IN:{
                rear_intake_motor->move_velocity(intake_speed);
                lower_roller_motor->move_velocity(intake_speed);
                break;
            }
            case OUT:{
                rear_intake_motor->move_velocity(-intake_speed);
                break;   
            }
            case STOP:{
                rear_intake_motor->move_velocity(0);
                lower_roller_motor->move_velocity(0);
                break;
            }
        }
    }

    void IntakeSubsystem::tare_rear_motor_position(){
        rear_intake_motor->tare_position();
    }

    double IntakeSubsystem::get_rear_motor_position(){
        return rear_intake_motor->get_position();
    }

    void IntakeSubsystem::auto_unload(){
        auto_unload_active = true;
        double start_time = pros::millis();
        while(distance_sensor->get() < MAX_DISTANCE || pros::millis() < start_time + 7000){
            move_intake(OUT);
        }
        move_intake(STOP);
        auto_unload_active = false;
    }

    bool IntakeSubsystem::is_auto_unload_active(){
        return auto_unload_active;
    }

    void IntakeSubsystem::colorSortMode(bool state){
        colorSort = state;
    }
    
}
}
}
}