#ifndef HOOD_SUBSYSTEM_BUILDER_HPP
#define HOOOD_SUBSYSTEM_BUILDER_HPP

#include "platt2/hal/Solenoid.hpp"
#include "platt2/robot/subsystems/hood/HoodSubsystem.hpp"
#include <memory>
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

    class HoodSubsystemBuilder{
        private:
            std::unique_ptr<hal::Solenoid> long_height_piston;
            std::unique_ptr<hal::Solenoid> middle_height_piston;
            std::unique_ptr<hal::Solenoid> stopper_piston;
        
        public:
            HoodSubsystemBuilder* withLongPiston(std::unique_ptr<hal::Solenoid>& piston);
            HoodSubsystemBuilder* withMiddlePiston(std::unique_ptr<hal::Solenoid>& piston);
            HoodSubsystemBuilder* withStopperPiston(std::unique_ptr<hal::Solenoid>& piston);

            std::unique_ptr<HoodSubsystem> build();
    }; 

}}}}

#endif