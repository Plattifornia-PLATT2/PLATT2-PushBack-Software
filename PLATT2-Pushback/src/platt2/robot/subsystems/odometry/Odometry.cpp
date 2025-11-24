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
double Odometry::getHeading() {
  // return otos.getHeading();
    double heading = vex_imu->get_heading();
    /*// Normalize in case heading is outside 0–360
    while (heading < 0)   heading += 360;
    while (heading >= 360) heading -= 360;

    // Convert to -180 to 180
    if (heading > 180)
        heading -= 360;*/
    heading = heading * M_PI / 180.0;        // convert to radians

    // wrap to [-pi, pi]
    heading = fmod(heading + M_PI, 2.0*M_PI);
    if (heading < 0) heading += 2.0*M_PI;
    heading -= M_PI;

    return -heading; // radians in [-pi, pi]
}

Odometry::Odometry(std::unique_ptr<pros::IMU> vex_imu)
    : otos(0, 0), vex_imu(std::move(vex_imu)) {
  if (this->vex_imu) {
    this->vex_imu->reset();
    while (this->vex_imu->is_calibrating()) {
      pros::delay(10);
    }
  }
}
} // namespace odometry
} // namespace subsystems
} // namespace robot
} // namespace platt2