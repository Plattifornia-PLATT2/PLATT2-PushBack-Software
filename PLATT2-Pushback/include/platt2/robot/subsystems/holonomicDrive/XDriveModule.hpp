#ifndef XDRIVEMODULE_HPP
#define XDRIVEMODULE_HPP

#include "platt2/hal/MotorGroup.hpp"
#include "platt2/robot/subsystems/holonomicDrive/MovementVector.hpp"

namespace platt2{

namespace robot{

namespace subsystems{

namespace holonomicDrive{

    class XDriveModule {
        private:

        hal::MotorGroup module_motors;
        
        double k_rot;
        double angleFromZero;
        double wheel_radius;

        public:
        void move_vector(MovementVector v);

        double get_rpm();

        XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double angleFromZero, double wheel_radius);
    };
}
}
}
}

#endif