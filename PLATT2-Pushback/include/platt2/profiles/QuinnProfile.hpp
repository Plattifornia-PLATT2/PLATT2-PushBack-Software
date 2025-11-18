#ifndef QUINNPROFILE_HPP
#define QUINNPROFILE_HPP

#include "platt2/profiles/DriverProfile.hpp"
#include "pros/misc.h"

namespace platt2{
namespace profiles{

    struct QuinnProfile : public DriverProfile{

        // ---Digital Mappings---

        // Intake Mappings
        pros::controller_digital_e_t frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        pros::controller_digital_e_t frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;

        // Pnuematic Actuations
        pros::controller_digital_e_t edMech_toggle = pros::E_CONTROLLER_DIGITAL_R1;
        pros::controller_digital_e_t highScore_toggle = pros::E_CONTROLLER_DIGITAL_R2;
        pros::controller_digital_e_t stopper_toggle = pros::E_CONTROLLER_DIGITAL_UP;

        // Color Sort
        pros::controller_digital_e_t colorSort_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
    };

}
}

#endif