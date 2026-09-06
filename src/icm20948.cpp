// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigen.h>

#include "icm20948.h"
#include "display.h"
#include "online_calibration.h"

#define SDA 22
#define SCL 27

// I2C Addresses
#define ICM20948_ADDR 0x69

// Create sensor objects
Adafruit_ICM20948 icm20948;

Eigen::Vector3d magPoint;
Eigen::Vector3d accPoint;
Eigen::Vector3d gyroPoint;
Eigen::Vector3d magPointTrans;
Eigen::Vector3d accPointTrans;
Eigen::Vector3d gyroPointTrans;

double target = -0.907;
AffineFinder calibration(target);

void scanI2C() {
  Serial.println("\nScanning I2C bus...");
  byte count = 0;
  
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s)");
}


void initSensors() {
    Wire.begin(SDA, SCL);
    Serial.begin(115200);
    scanI2C();

    // Initialize ICM-20948
    Serial.println("\n--- Initializing ICM-20948 ---");
    if (!icm20948.begin_I2C(ICM20948_ADDR)) {
        Serial.println("Failed to find ICM-20948!");
    } else {
        Serial.println("ICM-20948 Found!");
        icm20948.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
        icm20948.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
        icm20948.setMagDataRate(AK09916_MAG_DATARATE_20_HZ);
    }

}

unsigned long lastMagReadUs = 0;
bool newMag = false;
bool magReady(ak09916_data_rate_t rate) {
  unsigned long interval;
  switch(rate) {
    case AK09916_MAG_DATARATE_10_HZ:  interval = 100000; break;
    case AK09916_MAG_DATARATE_20_HZ:  interval = 50000;  break;
    case AK09916_MAG_DATARATE_50_HZ:  interval = 20000;  break;
    case AK09916_MAG_DATARATE_100_HZ: interval = 10000;  break;
    case AK09916_MAG_DATARATE_SHUTDOWN: return false;
    default: interval = 10000;
  }
  
  unsigned long now = micros();
  if (now - lastMagReadUs >= interval) {
    lastMagReadUs = now;
    return true;
  }
  return false;
}


void updateSensors() {
    sensors_event_t accel, gyro, mag, temp;
    newMag = magReady(icm20948.getMagDataRate());
    icm20948.getEvent(&accel, &gyro, &temp, &mag);

    // Check if enough time passed for new mag reading
    // if (newMag) {
    //     Serial.println("NEW Mag -------------------------------------------");
    // } else {
    //     Serial.println("Cached mag data");
    // }

    accPoint = Eigen::Vector3d(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    gyroPoint = Eigen::Vector3d(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    magPoint = Eigen::Vector3d(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    accPointTrans = accPoint.normalized();
    magPointTrans = calibration.transform(magPoint);
    gyroPointTrans = gyroPoint;

}

void printSensorsToSerial(bool transformed) {
    Serial.print("Accel: ");
    if(transformed)
        printEigen(accPointTrans);
    else
        printEigen(accPoint);

    Serial.print("Gyro: ");
    if(transformed)
        printEigen(gyroPointTrans);
    else
        printEigen(gyroPoint);

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

    printVectorToDisplay("Acc: ", accTmp, 5);
    printVectorToDisplay("Mag: ", magTmp, 25);
    printVectorToDisplay("Gyro: ", gyroPoint, 45);

}

Eigen::Vector3d getAccelReading() {
    return accPointTrans;
}
Eigen::Vector3d getMagReading() {
    return magPointTrans;
}