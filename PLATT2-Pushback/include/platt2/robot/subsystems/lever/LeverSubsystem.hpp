#ifndef LEVER_SUBSYSTEM_HPP
#define LEVER_SUBSYSTEM_HPP

/**
 * @brief Namespace for all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
#include "platt2/hal/MotorGroup.hpp"
#include "pros/motors.hpp"
#include <memory>
#include "platt2/hal/Solenoid.hpp"

namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code.
 * @author Dominic Young
 */
namespace robot{

/**
 * @brief Namespace for all subsystem code
 * @author Dominic Young
 */
namespace subsystems{

/**
 * @brief Namespace for the lever subsystem code
 * @author Dominic Young
 */
namespace lever{

    /**
     * @brief Enum to represent the different setpoints the lever needs to move to
     * @author Dominic Young
     */
    enum LeverPositions{
        MIDDLE,
        LONG,
        HOME
    };

    class LeverSubsystem{

        private:
            // Position Constants
            const double LONG_LEVER_TRAVEL_SETPOINT = 0.0;
            const double MIDDLE_LEVER_TRAVEL_SETPOINT = 0.0;
            const double HOME_LEVER_TRAVEL_SETPOINT = 0.0;

            // Power Constants
            const double LONG_LEVER_VOLTAGE = 127;
            const double MIDDLE_LEVER_VOLTAGE = 127;

            // Lever Motors
            std::unique_ptr<pros::Motor> left_motor;
            std::unique_ptr<pros::Motor> right_motor;

        public:
            /**
             * @brief Set the Left Motor object
             * 
             * @param motor motor to set
             */
            void setLeftMotor(std::unique_ptr<pros::Motor> motor);

            /**
             * @brief Set the Right Motor object
             * 
             * @param motor motor to set
             */
            void setRightMotor(std::unique_ptr<pros::Motor> motor);

            /**
             * @brief Moves the lever to the defined position
             * @param position the position enum to use
             */
            void moveToPosition(LeverPositions position);
    };

}
}
}
}

#endif