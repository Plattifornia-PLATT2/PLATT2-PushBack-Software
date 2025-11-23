#ifndef INTAKESUBSYSTEM_HPP
#define INTAKESUBSYSTEM_HPP

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include <memory>
namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    enum IntakeDirection{
        IN = 1,
        OUT = 2,
        STOP = 3,
        OUT_LOW_GOAL=4
    };

    /**
    @brief A class to control the Intake subsytem
    @author Dominic Young
    */
    class IntakeSubsystem{

        private:
        double intake_speed{600};
        
        // ** Motors **
        std::unique_ptr<pros::Motor> front_intake_motor;
        std::unique_ptr<pros::Motor> rear_intake_motor;
        std::unique_ptr<pros::Motor> middle_intake_motor;
        std::unique_ptr<pros::Motor> upper_conveyor_motor;

        // ** Solenoids **
        std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston;
        std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston;
        std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston;

        bool conveyor_stopper_piston_state = false;
        bool conveyor_stopper_piston_lastState = false;
        bool ed_mech_piston_state = false;
        bool ed_mech_piston_lastState = false;
        bool upper_conveyor_height_piston_state = false;
        bool upper_conveyor_height_piston_lastState = false;

        public:
        void move_intake(IntakeDirection direction);

        void toggle_ed_mech_piston();
        void toggle_upper_conveyor_height_piston();
        void toggle_conveyor_stopper_piston();

        IntakeSubsystem(
            std::unique_ptr<pros::Motor>front_intake, 
            std::unique_ptr<pros::Motor>rear_intake, 
            std::unique_ptr<pros::Motor>mid_intake, 
            std::unique_ptr<pros::Motor>upper_conveyor,
            std::unique_ptr<pros::adi::DigitalOut> ed_mech_piston,
            std::unique_ptr<pros::adi::DigitalOut> upper_conveyor_height_piston,
            std::unique_ptr<pros::adi::DigitalOut> conveyor_stopper_piston
        );


    };
}
}
}
}

#endif