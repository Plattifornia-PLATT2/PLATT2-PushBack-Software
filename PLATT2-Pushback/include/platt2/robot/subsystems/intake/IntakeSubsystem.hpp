#ifndef INTAKESUBSYSTEM_HPP
#define INTAKESUBSYSTEM_HPP

#include "pros/motors.hpp"
#include <memory>
namespace platt2{
namespace robot{
namespace subsystems{
namespace intake{

    enum IntakeDirection{
        IN = 1,
        OUT = 2,
        STOP = 3
    };

    /**
    @brief A class to control the Intake subsytem
    @author Dominic Young
    */
    class IntakeSubsystem{

        private:
        double intake_speed{400};
    
        std::unique_ptr<pros::Motor> front_intake_motor;
        std::unique_ptr<pros::Motor> rear_intake_motor;
        std::unique_ptr<pros::Motor> middle_intake_motor;
        std::unique_ptr<pros::Motor> upper_conveyor_motor;
        public:
        void move_intake(IntakeDirection direction);

        IntakeSubsystem(std::unique_ptr<pros::Motor>front_intake, std::unique_ptr<pros::Motor>rear_intake, std::unique_ptr<pros::Motor>mid_intake, std::unique_ptr<pros::Motor>upper_conveyor);


    };
}
}
}
}

#endif