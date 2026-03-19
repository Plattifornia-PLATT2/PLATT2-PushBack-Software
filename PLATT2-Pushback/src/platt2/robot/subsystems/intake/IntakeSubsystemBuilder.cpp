#include "platt2/robot/subsystems/intake/IntakeSubsystemBuilder.hpp"
#include "IntakeSubsystem.hpp"
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

    IntakeSubsystemBuilder* IntakeSubsystemBuilder::withFrontIntakeMotor(std::unique_ptr<pros::Motor>& motor){
        front_intake_motor = std::move(motor);
        return this;
    }

    IntakeSubsystemBuilder* IntakeSubsystemBuilder::withLeftMatchloadMotor(std::unique_ptr<pros::Motor>& motor){
        left_matchload_motor = std::move(motor);
        return this;
    }    

    IntakeSubsystemBuilder* IntakeSubsystemBuilder::withRightMatchloadMotor(std::unique_ptr<pros::Motor>& motor){
        right_matchload_motor = std::move(motor);
        return this;
    }

    IntakeSubsystemBuilder* IntakeSubsystemBuilder::withMatchloadPiston(std::unique_ptr<hal::Solenoid>& piston){
        matchload_piston = std::move(piston);
        return this;
    }

    std::unique_ptr<IntakeSubsystem> IntakeSubsystemBuilder::build(){
        std::unique_ptr<IntakeSubsystem> subsystem = std::make_unique<IntakeSubsystem>();

        subsystem->setFrontIntakeMotor(std::move(front_intake_motor));
        subsystem->setLeftMatchloadMotor(std::move(left_matchload_motor));
        subsystem->setRightMatchloadMotor(std::move(right_matchload_motor));
        subsystem->setMatchloadPiston(std::move(matchload_piston));

        return subsystem;
    }

}}}}