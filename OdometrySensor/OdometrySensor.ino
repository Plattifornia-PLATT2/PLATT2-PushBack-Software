#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include <SoftwareSerial.h>

//tx and rx pins
const int txPin = 3;
const int rxPin = 2;
const long baud_rate = 74880;

QwiicOTOS odomSensor;
SoftwareSerial rs485Serial(rxPin, txPin);

void setup() {
  Wire.begin();
  rs485Serial.begin(baud_rate);
  Serial.begin(baud_rate);
  odomSensor.begin();

  delay(1000); // delay to prevent heading being screwed up due to touch
  odomSensor.calibrateImu(1000, true);

  sfe_otos_pose2d_t offset = {0, 0, 0};
  odomSensor.setOffset(offset);


  odomSensor.resetTracking();


}

void loop() {
  // put your main code here, to run repeatedly

  if (rs485Serial.available()) {
    String line = rs485Serial.readStringUntil(';');
    if (line.startsWith("/")) {
      float heading = line.substring(line.indexOf(':') + 1).toFloat();
      sfe_otos_pose2d_t newPos;
      odomSensor.getPosition(newPos);
      newPos.h = heading;
      odomSensor.setPosition(newPos);
      Serial.println(heading);
    }
    else{
      rs485Serial.flush();
    }

    sfe_otos_pose2d_t currentPos;
  odomSensor.getPosition(currentPos);

  float currentX = currentPos.x;
  float currentY = currentPos.y;
  float currentHeading = currentPos.h;

  String outputString{"/"};
  outputString += "X:" + String(currentX) + ";";
  outputString += "Y:" + String(currentY) + ";";
  outputString += "H:" + String(currentHeading) + ";";

  Serial.println(outputString);

  rs485Serial.print(outputString);

  delay(10);


   
  }

  

}

