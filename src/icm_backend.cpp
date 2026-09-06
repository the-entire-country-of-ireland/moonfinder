#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#include "icm_backend.h"

namespace {
constexpr uint8_t IcmAddress = 0x69;
Adafruit_ICM20948 icm;
}

bool Icm20948Backend::begin() {
    if (!icm.begin_I2C(IcmAddress)) {
        Serial.println("Failed to find ICM-20948!");
        return false;
    }
    icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
    icm.setMagDataRate(AK09916_MAG_DATARATE_20_HZ);
    Serial.println("ICM-20948 ready");
    return true;
}

bool Icm20948Backend::update() {
    sensors_event_t acceleration;
    sensors_event_t gyroscope;
    sensors_event_t magnetometer;
    sensors_event_t temperature;
    icm.getEvent(&acceleration, &gyroscope, &temperature, &magnetometer);
    acceleration_ << acceleration.acceleration.x, acceleration.acceleration.y,
                     acceleration.acceleration.z;
    gyroscope_ << gyroscope.gyro.x, gyroscope.gyro.y, gyroscope.gyro.z;
    magnetometer_ << magnetometer.magnetic.x, magnetometer.magnetic.y,
                     magnetometer.magnetic.z;
    return true;
}