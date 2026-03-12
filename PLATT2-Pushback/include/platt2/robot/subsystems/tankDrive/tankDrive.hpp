#ifndef XDRIVE_HPP
#define XDRIVE_HPP


/**
 * @brief Namespace containing all PLATT2 library code.
 * @authors PLATT2 Development team.
 */
#include "platt2/hal/MotorGroup.hpp"
#include "pros/motors.hpp"
#include <memory>
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
 * @brief Namespace for holonomic drive related code.
 * @authors PLATT2 Development team.
 */


class tankModule{
    public:
        hal::MotorGroup module_motors;
        tankModule(std::unique_ptr<pros::Motor>& motor_1, std::unique_ptr<pros::Motor>& motor_2, std::unique_ptr<pros::Motor>& motor_3, std::unique_ptr<pros::Motor>& motor_4);

};
class tankDrive{

    public:

        struct MovementVector {
            double r;
            double w;
        };

        void moveVector(MovementVector v);

        
        std::vector<std::unique_ptr<tankModule>> drive_modules;

 
        tankDrive(std::vector<std::unique_ptr<tankModule>> drive_modules);
};

}
}
}

#endif