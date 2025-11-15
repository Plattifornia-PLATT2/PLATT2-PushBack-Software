#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "OdometryPosition.hpp"
//#include "pros/rtos.hpp"
//#include <memory>

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

    double Odometry::getX(){
        return otos.getXPosition();
    }
    double Odometry::getY(){
        return otos.getYPosition();
    }
    double Odometry::getHeading(){
        //return otos.getHeading();
        return 0.0;
    }

    Odometry::Odometry():
    otos(0,0)
    {
         
    }
}
}
}
}