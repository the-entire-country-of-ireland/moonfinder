#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>

#include "lms_backend.h"

namespace {
Adafruit_LSM303_Accel_Unified accelerometer(54321);
Adafruit_LIS2MDL magnetometerSensor(12345);
}

bool Lsm303Lis2mdlBackend::begin() {
    if (!accelerometer.begin()) {
        Serial.println("LSM303 accelerometer not found");
        return false;
    }
    accelerometer.setRange(LSM303_RANGE_4G);
    accelerometer.setMode(LSM303_MODE_NORMAL);

    if (!magnetometerSensor.begin()) {
        Serial.println("LIS2MDL magnetometer not found");
        return false;
    }
    magnetometerSensor.enableAutoRange(true);
    Serial.println("LSM303 accelerometer and LIS2MDL magnetometer ready");
    return true;
}

bool Lsm303Lis2mdlBackend::update() {
    sensors_event_t acceleration;
    sensors_event_t magneticField;
    accelerometer.getEvent(&acceleration);
    magnetometerSensor.getEvent(&magneticField);
    acceleration_ << acceleration.acceleration.x, acceleration.acceleration.y,
                     acceleration.acceleration.z;
    magnetometer_ << magneticField.magnetic.x, magneticField.magnetic.y,
                     magneticField.magnetic.z;
    gyroscope_.setZero();
    return true;
}