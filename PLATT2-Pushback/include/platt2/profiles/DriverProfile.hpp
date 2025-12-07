#ifndef EDRIVERPROFILE_HPP
#define EDRIVERPROFILE_HPP

#include "pros/misc.h"

namespace platt2{
namespace profiles{

    struct DriverProfile{

        // ---Digital Mappings---

        // Intake Mappings
        pros::controller_digital_e_t frontIntake_IN;
        pros::controller_digital_e_t frontIntake_OUT;
        pros::controller_digital_e_t auto_unload_button;

        // Pnuematic Actuations
        pros::controller_digital_e_t edMech_toggle;
        pros::controller_digital_e_t highScore_toggle;
        pros::controller_digital_e_t stopper_toggle;
        pros::controller_digital_e_t rakeMech_toggle;
        pros::controller_digital_e_t descore_toggle;

        // Color Sort
        pros::controller_digital_e_t colorSort_toggle;

        // Field Oriented Drive
        pros::controller_digital_e_t fieldOriented_toggle;
        pros::controller_digital_e_t heading_reset;
    };
}
}
#endif