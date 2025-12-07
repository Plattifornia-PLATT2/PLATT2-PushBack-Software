#ifndef OPTICAL_TRACKING_SENSOR_HPP
#define OPTICAL_TRACKING_SENSOR_HPP

#include "pros/serial.hpp"
#include "pros/screen.hpp"
#include "pros/imu.hpp"
#include <memory>
#include <string>
#include <stdint.h>
#include <iostream>

namespace platt2{

namespace hal{

class OpticalTrackingSensor{

private:
    const int BAUD_RATE = 74880;

    pros::Serial m_serialInterface = pros::Serial(4, BAUD_RATE);

    std::unique_ptr<pros::IMU> vex_imu;

    pros::Task m_otosTask;

    double xOffset;

    double yOffset;

    double hOffset;

    double xPos;

    double yPos;

    double heading;

    double localHeading;

    double getBoundedHeading();

public:
    OpticalTrackingSensor(double xOffset, double yOffset, double hOffset, std::unique_ptr<pros::IMU> vex_imu);

    void readData();

    double getXPosition();

    double getYPosition();

    double getHeading();

    void resetHeading();

    double getVexHeading();
};

} // namespace hal
} // namespace platt2
#endif