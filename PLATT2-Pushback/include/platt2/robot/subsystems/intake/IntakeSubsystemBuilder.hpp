#ifndef INTAKE_SUBSYSTEM_BUILDER_HPP
#define INTAKE_SUBSYSTEM_BUILDER_HPP

#include "IntakeSubsystem.hpp"
#include "platt2/hal/Solenoid.hpp"
#include "pros/motors.hpp"

#include <memory>

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

    class IntakeSubsystemBuilder{
        private:
        // ** Motors **
        std::unique_ptr<pros::Motor> front_intake_motor;
        std::unique_ptr<pros::Motor> left_matchload_motor;
        std::unique_ptr<pros::Motor> right_matchload_motor;

        // ** Solenoids **
        std::unique_ptr<hal::Solenoid> matchload_piston;

        public:
        IntakeSubsystemBuilder* withFrontIntakeMotor(std::unique_ptr<pros::Motor>& motor);
        IntakeSubsystemBuilder* withLeftMatchloadMotor(std::unique_ptr<pros::Motor>& motor);
        IntakeSubsystemBuilder* withRightMatchloadMotor(std::unique_ptr<pros::Motor>& motor);
        IntakeSubsystemBuilder* withMatchloadPiston(std::unique_ptr<hal::Solenoid>& piston);

        std::unique_ptr<IntakeSubsystem> build();
    };

}}}}

#endif