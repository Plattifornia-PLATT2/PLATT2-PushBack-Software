#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "OdometryPosition.hpp"
#include <memory>
#include "pros/rtos.hpp"
#include <math.h>

namespace platt2 {

namespace robot {

namespace subsystems {

namespace odometry {

OdometryPosition Odometry::getPos() {
  OdometryPosition curPos;
  curPos.x = otos.getXPosition();
  curPos.y = otos.getYPosition();
  curPos.heading = otos.getHeading();

  return curPos;
}

double Odometry::getX() { return otos.getXPosition(); }
double Odometry::getY() { return otos.getYPosition(); }
double Odometry::getHeading() { return otos.getHeading(); }
double Odometry::getVexHeading() {return otos.getVexHeading();}

void Odometry::resetHeading() {
  otos.resetHeading();
}

Odometry::Odometry(std::unique_ptr<pros::IMU> vex_imu, double x, double y, double h)
: otos(x, y, h,  std::move(vex_imu)) 
{
   
}
} // namespace odometry
} // namespace subsystems
} // namespace robot
} // namespace platt2