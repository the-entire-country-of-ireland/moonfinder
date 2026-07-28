// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#ifndef LMS_H
#define LMS_H

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include "online_calibration.h"

#include "display.h"

#define SDA 22
#define SCL 27

extern Adafruit_LSM303_Accel_Unified accel;
extern Adafruit_LIS2MDL lis2mdl;

extern Eigen::Vector3d magPoint;
extern Eigen::Vector3d accPoint;
extern Eigen::Vector3d magPointTrans;
extern Eigen::Vector3d accPointTrans;
extern AffineFinder calibration;

void initSensors();
void updateSensors();
void printSensorsToSerial(bool transformed=true);
void printSensorsToDisplay(bool transformed=true);
Eigen::Vector3d getAccelReading();
Eigen::Vector3d getMagReading();

#endif // LMS_H