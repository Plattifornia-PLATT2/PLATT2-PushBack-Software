#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "OdometryPosition.hpp"
#include "pros/rtos.hpp"
#include <memory>

namespace platt2{

namespace robot{

namespace subsystems{

namespace odometry{

    OdometryPosition Odometry::getPos(){
        OdometryPosition curPos;
        curPos.x = otos.getXPosition();
        curPos.y = otos.getYPosition();
        curPos.heading = otos.getHeading();

        return curPos;
    }

    Odometry::Odometry():
    otos(0,0)
    {
         
    }
}
}
}
}