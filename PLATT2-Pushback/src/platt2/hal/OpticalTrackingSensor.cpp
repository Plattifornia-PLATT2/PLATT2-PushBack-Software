#include "PLATT2/hal/OpticalTrackingSensor.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
//#include <cstdint>
#include <sys/types.h>

namespace platt2{

namespace hal{

static void taskThunk(void *p) {
    reinterpret_cast<OpticalTrackingSensor*>(p)->readData();
}

OpticalTrackingSensor::OpticalTrackingSensor(double xOffset, double yOffset):
 m_otosTask(taskThunk, this)
{
    this->xOffset = xOffset;
    this->yOffset = yOffset;
    xPos = 0;
    yPos = 0;
    heading = 0;
    std::cout<<"Lol";
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

void OpticalTrackingSensor::readData(){

    while(true){
        std::string fullRead {};
        
        while(m_serialInterface.get_read_avail()){
            char byteRead = static_cast<char>(m_serialInterface.read_byte());
            fullRead.push_back(byteRead);
        }
        
        int xIndexFront = fullRead.find("X");
        int xIndexBack = fullRead.find(";", xIndexFront+1);
        std::string xPosStr = fullRead.substr(xIndexFront+2, xIndexBack-(xIndexFront+2));

        int yIndexFront = fullRead.find("Y");
        int yIndexBack = fullRead.find(";", yIndexFront+1);
        std::string yPosStr = fullRead.substr(yIndexFront+2, yIndexBack-(yIndexFront+2));

        int hIndexFront = fullRead.find("H");
        int hIndexBack = fullRead.find(";", hIndexFront+1);
        std::string hPosStr = fullRead.substr(hIndexFront+2, hIndexBack-(hIndexFront+2));


        //std::cout << xPosStr << yPosStr << hPosStr << std::endl;
        pros::screen::erase();
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "X Pos: %s", xPosStr);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Y Pos: %s", yPosStr);
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "Heading: %s", hPosStr);
        

        pros::delay(10);
    }
}
} // namespace hal
} // namespace platt2