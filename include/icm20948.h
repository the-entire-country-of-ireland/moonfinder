// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#ifndef ICM_H
#define ICM_H

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_Sensor.h>
#include "online_calibration.h"

#include "display.h"

#define SDA 22
#define SCL 27

extern Adafruit_ICM20948 icm20948;

extern Eigen::Vector3d magPoint;
extern Eigen::Vector3d accPoint;
extern Eigen::Vector3d gyroPoint;
extern Eigen::Vector3d magPointTrans;
extern Eigen::Vector3d accPointTrans;
extern Eigen::Vector3d gyroPointTrans;

extern AffineFinder calibration;

void scanI2C();
void initSensors();
void updateSensors();
void printSensorsToSerial(bool transformed=true);
void printSensorsToDisplay(bool transformed=true);
Eigen::Vector3d getAccelReading();
Eigen::Vector3d getMagReading();

#endif // ICM_H