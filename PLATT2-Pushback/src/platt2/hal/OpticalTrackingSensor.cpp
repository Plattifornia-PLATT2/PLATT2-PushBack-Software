#include "PLATT2/hal/OpticalTrackingSensor.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
//#include <cstdint>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <math.h>
#include <memory>
#include <string>
#include <sys/types.h>
#include <vector>

namespace platt2{

namespace hal{

static void taskThunk(void *p) {
    reinterpret_cast<OpticalTrackingSensor*>(p)->readData();
}

OpticalTrackingSensor::OpticalTrackingSensor(double xOffset, double yOffset, std::unique_ptr<pros::IMU> vex_imu):
 m_otosTask(taskThunk, this),
 vex_imu(std::move(vex_imu))
{
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    xPos = 0;
    yPos = 0;
    heading = 0;

    if(vex_imu){
        this->vex_imu->reset();
        while(vex_imu->is_calibrating()){
            pros::delay(10);
        }
        vex_imu->set_heading(90.0);
    }
}

double OpticalTrackingSensor::getXPosition(){
    return xPos;
}

double OpticalTrackingSensor::getYPosition(){
    return yPos;
}

double OpticalTrackingSensor::getHeading(){
    return heading;
}

void OpticalTrackingSensor::resetHeading(){
    if(vex_imu){
        vex_imu->set_heading(0);
        localHeading = 0;
    }
}

double OpticalTrackingSensor::getBoundedHeading(){
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

void OpticalTrackingSensor::readData(){

    double tempHeading;
    double imuHeading;
    std::string fullRead = "";
    std::string writeString = "";
    while(true){
        pros::screen::erase();
        writeString = "";
        imuHeading = vex_imu->get_heading();
        imuHeading = std::round(imuHeading * 100.0) / 100.0;
        std::cout<<"IMU Heading"<<imuHeading<<std::endl;
        writeString = "/H:" + std::to_string(imuHeading) + ";";
        std::vector<uint8_t> writeData (writeString.begin(), writeString.end());
        std::cout<<writeData.data()<<std::endl;
        if(m_serialInterface.get_write_free() > 0){
            m_serialInterface.write(reinterpret_cast<uint8_t*>(writeData.data()), writeData.size());
            pros::delay(5);
        }

        fullRead = "";
        std::string xPosStr = "";
        std::string yPosStr = "";
        std::string hPosStr = "";
        while(!m_serialInterface.get_read_avail()){
            pros::delay(10);
        }

        while(m_serialInterface.get_read_avail()){
            char byteRead = static_cast<char>(m_serialInterface.read_byte());
            fullRead.push_back(byteRead);
        }
        std::cout<<"Full Read"<<fullRead<<std::endl; 
        if(fullRead.find('/') != std::string::npos){
                fullRead = fullRead.substr(fullRead.find('/'));
        if(fullRead.size() > 0){
        int xIndexFront = fullRead.find("X");
        int xIndexBack = fullRead.find(";", xIndexFront+1);
        xPosStr = fullRead.substr(xIndexFront+2, xIndexBack-(xIndexFront+2));

        int yIndexFront = fullRead.find("Y");
        int yIndexBack = fullRead.find(";", yIndexFront+1);
        yPosStr = fullRead.substr(yIndexFront+2, yIndexBack-(yIndexFront+2));

        int hIndexFront = fullRead.find("H");
        int hIndexBack = fullRead.find(";", hIndexFront+1);
        hPosStr = fullRead.substr(hIndexFront+2, hIndexBack-(hIndexFront+2));
        std::cout<<hPosStr<<std::endl; 
        }

        try{
        xPos = std::stod(xPosStr);
        }
        catch( std::exception e){

        }
        try{
        yPos = std::stod(yPosStr);
        } catch (std::exception e){

        }
        try{

        tempHeading = (std::stod(hPosStr)*M_PI/180)+(M_PI/2);
        if (tempHeading<0){
            tempHeading = tempHeading+(2*M_PI);
        }
        heading = tempHeading;

        } catch (std::exception e) {

        }

    } else {
        m_serialInterface.flush();
        std::cout<< "Flushed serial interface due to missing data." << std::endl;
    }

        //std::cout << xPosStr << yPosStr << hPosStr << std::endl;
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "X Pos: %s", xPosStr);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Y Pos: %s", yPosStr);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Heading: %f", heading);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "Write String: %f", imuHeading);

        pros::delay(10);
        
    
}
}
} // namespace hal
} // namespace platt2