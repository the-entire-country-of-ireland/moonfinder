// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>
#include "AHRS.h"
#include "display.h"

#define SDA 22
#define SCL 27

Vector3f accelReading;
Vector3f magReading;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);

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
  accelReading = Vector3f(accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z);

  lis2mdl.getEvent(&magEvent);
  magReading = Vector3f(magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z);

}

void printSensorsToSerial() {
  Serial.print("Accel: ");
  accelReading.println(2);
  Serial.print("Mag: ");
  magReading.println(2);

}

void printSensorsToDisplay() {
  String strAccel = "Accel: " + accelReading.toString(2);
  String strMag = "Mag: " + magReading.toString(2);
  String strSpaces = "      \n"; // Add spaces to clear previous text
  String strCombined = strAccel + strSpaces + strMag + strSpaces;
  printStringToDisplay(strCombined);
}

Vector3f getAccelReading() {
    return accelReading;
}
Vector3f getMagReading() {
    return magReading;
}