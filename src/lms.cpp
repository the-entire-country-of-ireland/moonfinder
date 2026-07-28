// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigen.h>

#include "lms.h"
#include "display.h"
#include "online_calibration.h"

#define SDA 22
#define SCL 27

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);

Eigen::Vector3d magPoint;
Eigen::Vector3d accPoint;
Eigen::Vector3d magPointTrans;
Eigen::Vector3d accPointTrans;

double target = -0.5;
AffineFinder calibration(target);

void initSensors() {
    Wire.begin(SDA, SCL);
    Serial.begin(115200);

    if (accel.begin()) {
        accel.setRange(LSM303_RANGE_4G);
        accel.setMode(LSM303_MODE_NORMAL);
        Serial.println("Accel ready");
    } else {
        Serial.println("Accel sensor not found");
    while(1);
    }

    if (lis2mdl.begin()) {
        lis2mdl.enableAutoRange(true);
        Serial.println("Mag ready");
    } else {
        Serial.println("Mag sensor not found");
        while(1);
    }
}



void updateSensors() {
  sensors_event_t accelEvent;
  sensors_event_t magEvent;

  accel.getEvent(&accelEvent);
  accPoint = Eigen::Vector3d(accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z);
  accPointTrans = accPoint.normalized();

  lis2mdl.getEvent(&magEvent);
  magPoint = Eigen::Vector3d(magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z);
  magPointTrans = calibration.transform(magPoint);

  
}

void printSensorsToSerial(bool transformed) {
    Serial.print("Accel: ");
    if(transformed)
        printEigen(accPointTrans);
    else
        printEigen(accPoint);
    Serial.print("Mag: ");
    
    if(transformed)
        printEigen(magPointTrans);
    else
        printEigen(magPoint);
}


void printSensorsToDisplay(bool transformed) {
    tft.setTextWrap(false);

    Eigen::Vector3d accTmp = accPoint;
    Eigen::Vector3d magTmp = magPoint;
    if(transformed) {
        accTmp = accPointTrans;
        magTmp = magPointTrans;
    }

    tft.setFont(&fonts::FreeSans9pt7b); tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print("Acc: ");
    tft.setFont(&fonts::Font7); tft.setTextSize(0.35);
    tft.printf("% 2.2f , % 2.2f , % 2.2f        ", accTmp[0], accTmp[1], accTmp[2]);

    tft.setCursor(5, 25);
    tft.setFont(&fonts::FreeSans9pt7b); tft.setTextSize(1);
    tft.print("Mag: ");
    tft.setFont(&fonts::Font7); tft.setTextSize(0.35);
    tft.printf("% 2.2f , % 2.2f , % 2.2f        ", magTmp[0], magTmp[1], magTmp[2]);

}

Eigen::Vector3d getAccelReading() {
    return accPointTrans;
}
Eigen::Vector3d getMagReading() {
    return magPointTrans;
}