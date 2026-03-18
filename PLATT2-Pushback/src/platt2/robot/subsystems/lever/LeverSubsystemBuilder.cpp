#include "platt2/robot/subsystems/lever/LeverSubsystemBuilder.hpp"

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

    LeverSubsystemBuilder* LeverSubsystemBuilder::withLeftMotor(std::unique_ptr<pros::Motor>& motor){
        left_motor = std::move(motor);
        return this;
    }

    LeverSubsystemBuilder* LeverSubsystemBuilder::withRightMotor(std::unique_ptr<pros::Motor>& motor){
        right_motor = std::move(motor);
        return this;
    }

    std::unique_ptr<LeverSubsystem> LeverSubsystemBuilder::build(){
        std::unique_ptr<LeverSubsystem> subsystem = std::make_unique<LeverSubsystem>();

        subsystem->setLeftMotor(std::move(left_motor));
        subsystem->setRightMotor(std::move(right_motor));

        return subsystem;
    }


}}}}