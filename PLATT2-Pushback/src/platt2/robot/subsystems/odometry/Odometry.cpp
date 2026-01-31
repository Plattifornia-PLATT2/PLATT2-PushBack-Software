#include "platt2/robot/subsystems/odometry/Odometry.hpp"
#include "IPositionTracker.hpp"
#include "OdometryPosition.hpp"
#include <memory>
#include "TrackingWheelPositionTracker.hpp"
#include "pros/rtos.hpp"
#include <math.h>

namespace platt2 {

namespace robot {

namespace subsystems {

namespace odometry {

OdometryPosition Odometry::getPos() {
  return position_tracker->getPos();
}

double Odometry::getX() { return position_tracker->getX(); }
double Odometry::getY() { return position_tracker->getY(); }
double Odometry::getHeading() { return position_tracker->getHeading(); }
double Odometry::getVexHeading() {return 0;}
void Odometry::initVexImu() {}

void Odometry::setPos(OdometryPosition pos){
  position_tracker->setPos(pos);
}

void Odometry::resetHeading() {
  // TODO
}

Odometry::Odometry(std::unique_ptr<IPositionTracker> position_tracker, double x, double y, double h)
{
   this->position_tracker = std::move(position_tracker);
}

} // namespace odometry
} // namespace subsystems
} // namespace robot
} // namespace platt2