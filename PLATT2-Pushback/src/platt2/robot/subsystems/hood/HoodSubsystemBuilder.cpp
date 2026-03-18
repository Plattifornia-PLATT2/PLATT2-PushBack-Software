#include "platt2/robot/subsystems/hood/HoodSubsystemBuilder.hpp"

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
 * @brief Namespace for the hood subsystem code
 * @author Dominic Young
 */
namespace hood{

    HoodSubsystemBuilder* HoodSubsystemBuilder::withLongPiston(std::unique_ptr<hal::Solenoid>& piston){
        long_height_piston = std::move(piston);
        return this;
    }

    HoodSubsystemBuilder* HoodSubsystemBuilder::withMiddlePiston(std::unique_ptr<hal::Solenoid>& piston){
        middle_height_piston = std::move(piston);
        return this;
    }

    HoodSubsystemBuilder* HoodSubsystemBuilder::withStopperPiston(std::unique_ptr<hal::Solenoid>& piston){
        stopper_piston = std::move(piston);
        return this;
    }

    std::unique_ptr<HoodSubsystem> HoodSubsystemBuilder::build(){
        std::unique_ptr<HoodSubsystem> subsystem = std::make_unique<HoodSubsystem>();

        subsystem->setLongPiston(std::move(long_height_piston));
        subsystem->setMiddlePiston(std::move(middle_height_piston));
        subsystem->setStopperPiston(std::move(stopper_piston));

        return subsystem;
    }


}}}}