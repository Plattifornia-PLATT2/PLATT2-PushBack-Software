#ifndef HOOD_SUBSYSTEM_HPP
#define HOOOD_SUBSYSTEM_HPP

#include "platt2/hal/Solenoid.hpp"
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

    class HoodSubsystem{
        private:
        std::unique_ptr<hal::Solenoid> long_height_piston;
        std::unique_ptr<hal::Solenoid> middle_height_piston;
        std::unique_ptr<hal::Solenoid> stopper_piston;

        public:
        /**
         * @brief Sets the Long Piston pointer
         * 
         * @param piston piston to set
         */
        void setLongPiston(std::unique_ptr<hal::Solenoid> piston);

        /**
         * @brief Sets the Middle Piston pointer
         * 
         * @param piston piston to set
         */
        void setMiddlePiston(std::unique_ptr<hal::Solenoid> piston);

        /**
         * @brief Sets the Stopper Piston pointer
         * 
         * @param piston piston to set
         */
        void setStopperPiston(std::unique_ptr<hal::Solenoid> piston);
    };

}}}}

#endif