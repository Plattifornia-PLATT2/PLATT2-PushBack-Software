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
        
        double theta;
        double phi;

        public:
        void move_vector(MovementVector v);
        double getTheta();
        double getPhi();

        //double get_rpm();
        //double getAngleFromZero();

        XDriveModule(std::unique_ptr<pros::Motor>& motor_l, std::unique_ptr<pros::Motor>& motor_r, double theta, double phi);
        // theta refers to the angle between N the turning center and the center of the wheel
        // phi refers to the angle between N the turning center and the direction the wheel is faceing 
    };
}
}
}
}

#endif