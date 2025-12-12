#ifndef JONPROFILE_HPP
#define JONPROFILE_HPP

#include "platt2/profiles/DriverProfile.hpp"
#include "pros/misc.h"

/**
 * @brief Namespace for all PLATT2 library code.
 * @author PLATT2 Development Team
 */
namespace platt2{

/**
 * @brief Namespace for controller profile code.
 * @author Dominic Young
 */
namespace profiles{

    /**
     * @brief A struct for Jon's controller mappings
     * 
     */
    struct JonProfile : public DriverProfile{
    JonProfile() {
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;
        auto_unload_button = pros::E_CONTROLLER_DIGITAL_Y;
        edMech_toggle = pros::E_CONTROLLER_DIGITAL_R1;
        highScore_toggle = pros::E_CONTROLLER_DIGITAL_R2;
        stopper_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        colorSort_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
        rakeMech_toggle = pros::E_CONTROLLER_DIGITAL_LEFT;
        fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        heading_reset = pros::E_CONTROLLER_DIGITAL_B;
        descore_toggle = pros::E_CONTROLLER_DIGITAL_A;
        scoreHighMacro = pros::E_CONTROLLER_DIGITAL_X;
        scoreMiddleHighMacro = pros::E_CONTROLLER_DIGITAL_LEFT;
    }
    };
}
}

#endif