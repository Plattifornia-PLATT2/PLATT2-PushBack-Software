#ifndef LEVER_SUBSYSTEMM_BUILDER_HPP
#define LEVER_SUBSYSTEMM_BUILDER_HPP

#include <memory>
#include <utility>
#include "platt2/hal/MotorGroup.hpp"
#include "pros/motors.hpp"
#include "platt2/robot/subsystems/lever/LeverSubsystem.hpp"

namespace platt2{

/**
 * @brief Namespace for all hardware abstraction layer code.
 * @author Dominic Young
 */
namespace robot{

/**
 * @brief Namespace for all subsystem code
 * 
 */
namespace subsystems{

/**
 * @brief Namespace for the lever subsystem code
 * @author Dominic Young
 */
namespace lever{

    class LeverSubsystemBuilder{
        private:
            std::unique_ptr<pros::Motor> left_motor;
            std::unique_ptr<pros::Motor> right_motor;

        public:
            /**
             * @brief Adds a left motor to the builder
             * 
             * @param motor The motor to add
             * @return LeverSubsystemBuilder* Pointer to this builder 
             */
            LeverSubsystemBuilder* withLeftMotor(std::unique_ptr<pros::Motor>& motor);

            /**
             * @brief Adds a right motor to the builder
             * 
             * @param motor The motor to add
             * @return LeverSubsystemBuilder* Pointer to this builder 
             */
            LeverSubsystemBuilder* withRightMotor(std::unique_ptr<pros::Motor>& motor);

            /**
             * @brief Builds a LeverSubsystem
             * 
             * @return std::unique_ptr<LeverSubsystem> to the built subsystem
             */
            std::unique_ptr<LeverSubsystem> build();
    };

}}}}

#endif