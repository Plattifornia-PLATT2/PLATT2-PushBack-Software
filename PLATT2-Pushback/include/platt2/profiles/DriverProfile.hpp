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

        // Pnuematic Actuations
        pros::controller_digital_e_t edMech_toggle;
        pros::controller_digital_e_t highScore_toggle;
        pros::controller_digital_e_t stopper_toggle;
        pros::controller_digital_e_t rakeMech_toggle;

        // Color Sort
        pros::controller_digital_e_t colorSort_toggle;
    };
}
}
#endif