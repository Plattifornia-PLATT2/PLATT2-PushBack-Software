#ifndef INTAKESUBSYSTEM_HPP
#define INTAKESUBSYSTEM_HPP

#include "platt2/hal/Solenoid.hpp"
#include "pros/motors.hpp"

#include <memory>
#include <type_traits>

/**
 * @brief Namespace for PLATT2 library code.
 * @authors PLATT2 Development team.
 */
namespace platt2{

/**
 * @brief Namespace for robot-related code.
 * @authors PLATT2 Development team.
 */
namespace robot{

/**
 * @brief Namespace for subsystem-related code.
 * @authors PLATT2 Development team.
 */
namespace subsystems{

/**
 * @brief Namespace for intake subsystem related code.
 * @authors PLATT2 Development team.
 */
namespace intake{

    /**
     * @brief An enumeration to represent the direction of the intake subsystem.
     * @author Dominic Young
     */
    enum IntakeDirection{
        IN = 1,
        OUT = 2,
        STOP = 3,
        OUT_LOW_GOAL = 4
    };

    /**
    @brief A class to control the Intake subsytem
    @author Dominic Young
    */
    class IntakeSubsystem{

        private:
        const double NORMAL_INTAKE_VOLTAGE {127};
        const double LOW_GOAL_INTAKE_VOLTAGE {-127};

        // ** Motors **
        std::unique_ptr<pros::Motor> front_intake_motor;
        std::unique_ptr<pros::Motor> left_matchload_motor;
        std::unique_ptr<pros::Motor> right_matchload_motor;

        // ** Solenoids **
        std::unique_ptr<hal::Solenoid> matchload_piston;

        public:
        void setFrontIntakeMotor(std::unique_ptr<pros::Motor> motor);
        void setLeftMatchloadMotor(std::unique_ptr<pros::Motor> motor);
        void setRightMatchloadMotor(std::unique_ptr<pros::Motor> motor);
        void setMatchloadPiston(std::unique_ptr<hal::Solenoid> piston);


        /**
         * @brief Moves the intake motors in the specified direction.
         * 
         * @param direction The direction to move the intake motors.
         */
        void moveIntake(IntakeDirection direction);

        /**
         * @brief Toggles the position of the matchload pistons.
         * 
         */
        void toggleMatchloadPistons();

        /**
         * @brief Gets the current position of the matchload piston
         * 
         * @return true if extended
         * @return false if retracted
         */
        bool getMatchloadPistonState();

    };
}
}
}
}

#endif