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
        auto_unload_button = pros::E_CONTROLLER_DIGITAL_X;
        edMech_toggle = pros::E_CONTROLLER_DIGITAL_Y;
        highScore_toggle = pros::E_CONTROLLER_DIGITAL_A;
        stopper_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
        colorSort_toggle = pros::E_CONTROLLER_DIGITAL_RIGHT;
        rakeMech_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        fieldOriented_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        heading_reset = pros::E_CONTROLLER_DIGITAL_LEFT;
        descore_toggle = pros::E_CONTROLLER_DIGITAL_B;
        scoreHighMacro = pros::E_CONTROLLER_DIGITAL_R1;
        scoreMiddleHighMacro = pros::E_CONTROLLER_DIGITAL_R2;
    }
    };
}
}

#endif