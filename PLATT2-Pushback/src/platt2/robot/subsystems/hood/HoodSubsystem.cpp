#include "platt2/robot/subsystems/hood/HoodSubsystem.hpp"

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

    void HoodSubsystem::setLongPiston(std::unique_ptr<hal::Solenoid> piston){
        long_height_piston = std::move(piston);
    }

    void HoodSubsystem::setMiddlePiston(std::unique_ptr<hal::Solenoid> piston){
        middle_height_piston = std::move(piston);
    }

    void HoodSubsystem::setStopperPiston(std::unique_ptr<hal::Solenoid> piston){
        stopper_piston = std::move(piston);
    }

    void HoodSubsystem::toggleLongHeightPiston(){
        long_height_piston->toggleState();
    }

    void HoodSubsystem::toggleMiddleHeightPiston(){
        middle_height_piston->toggleState();
    }

    void HoodSubsystem::toggleStopperPiston(){
        stopper_piston->toggleState();
    }

    
}}}}