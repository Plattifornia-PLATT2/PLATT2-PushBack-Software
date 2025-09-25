#ifndef OPTICAL_TRACKING_SENSOR_HPP
#define OPTICAL_TRACKING_SENSOR_HPP

#include "pros/serial.hpp"
#include "pros/screen.hpp"
#include <string>
#include <stdint.h>
#include <iostream>

namespace platt2{

namespace hal{

class OpticalTrackingSensor{

private:
    const int BAUD_RATE = 74880;

    pros::Serial m_serialInterface = pros::Serial(19, BAUD_RATE);

    double xOffset;

    double yOffset;

    double xPos;

    double yPos;

    double heading;

    std::unique_ptr<pros::Task> readTask;

    static void taskTrampoline(void* param);

public:
    OpticalTrackingSensor(double xOffset, double yOffset);

    void readData() const;

    double getXPosition();

    double getYPosition();

    double getHeading();
};

} // namespace hal
} // namespace platt2
#endif