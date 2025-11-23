#ifndef JONPROFILE_HPP
#define JONPROFILE_HPP

#include "platt2/profiles/DriverProfile.hpp"
#include "pros/misc.h"

namespace platt2{
namespace profiles{

    struct JonProfile : public DriverProfile{
    JonProfile() {
        frontIntake_IN = pros::E_CONTROLLER_DIGITAL_L1;
        frontIntake_OUT = pros::E_CONTROLLER_DIGITAL_L2;
        edMech_toggle = pros::E_CONTROLLER_DIGITAL_R1;
        highScore_toggle = pros::E_CONTROLLER_DIGITAL_R2;
        stopper_toggle = pros::E_CONTROLLER_DIGITAL_UP;
        colorSort_toggle = pros::E_CONTROLLER_DIGITAL_DOWN;
    }
    };

}
}

#endif