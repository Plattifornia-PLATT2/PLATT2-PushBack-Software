#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

//#include "api.h"
#include "platt2/hal/OpticalTrackingSensor.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
//#include "pros/rtos.hpp"


namespace platt2{

namespace robot{

namespace subsystems{

namespace odometry{
        
    class Odometry{
        private:
        hal::OpticalTrackingSensor otos;

        public:
        OdometryPosition getPos();
        double getX();
        double getY();
        double getHeading();
        Odometry();
    };

};
};
};
};
#endif