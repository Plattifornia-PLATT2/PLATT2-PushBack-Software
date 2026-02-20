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

static void taskThunk(void *p) {
    reinterpret_cast<Odometry*>(p)->startTracking();
}

OdometryPosition Odometry::getPos() {
  return position_tracker->getPos();
}

double Odometry::getX() { return position_tracker->getX(); }
double Odometry::getY() { return position_tracker->getY(); }
double Odometry::getHeading() { return position_tracker->getIMUHeading(); }
double Odometry::getVexHeading() {return 0;}
void Odometry::initVexImu() {position_tracker->init();}

void Odometry::setPos(OdometryPosition pos){
  position_tracker->setPos(pos);
}

void Odometry::resetHeading() {
  // TODO
}

void Odometry::startTracking(){
  position_tracker->updatePosition();
}

void Odometry::setOffsets(double x, double y){
  position_tracker->setOffsets(x, y);
}

Odometry::Odometry(std::unique_ptr<IPositionTracker> position_tracker):
m_trackingTask(taskThunk, this)
{
   this->position_tracker = std::move(position_tracker);
}

} // namespace odometry
} // namespace subsystems
} // namespace robot
} // namespace platt2