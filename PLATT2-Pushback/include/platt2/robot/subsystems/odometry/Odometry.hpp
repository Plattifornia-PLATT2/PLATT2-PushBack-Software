#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

//#include "api.h"
#include "platt2/hal/OpticalTrackingSensor.hpp"
#include "platt2/robot/subsystems/odometry/OdometryPosition.hpp"
#include "pros/imu.hpp"
#include <memory>
//#include "pros/rtos.hpp"


namespace platt2{

namespace robot{

namespace subsystems{

namespace odometry{
        
    class Odometry{
        private:
        hal::OpticalTrackingSensor otos;
        std::unique_ptr<pros::IMU> vex_imu;

        public:
        OdometryPosition getPos();
        double getX();
        double getY();
        double getHeading();
        void resetHeading();
        Odometry(std::unique_ptr<pros::IMU> vex_imu);
    };

};
};
};
};
#endif