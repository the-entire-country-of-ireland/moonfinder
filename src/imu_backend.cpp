#include <Wire.h>
#include <ArduinoEigen.h>

#include "imu_backend.h"
#include "icm_backend.h"
#include "lms_backend.h"

Eigen::Vector3d magPoint = Eigen::Vector3d::Zero();
Eigen::Vector3d accPoint = Eigen::Vector3d::Zero();
Eigen::Vector3d gyroPoint = Eigen::Vector3d::Zero();
Eigen::Vector3d magPointTrans = Eigen::Vector3d::Zero();
Eigen::Vector3d accPointTrans = Eigen::Vector3d::Zero();
Eigen::Vector3d gyroPointTrans = Eigen::Vector3d::Zero();
bool sensorReady = false;

double target = -0.907;
AffineFinder calibration(target);

#if defined(MOONFINDER_SENSOR_BACKEND_LMS)
Lsm303Lis2mdlBackend sensorBackend;
#else
Icm20948Backend sensorBackend;
#endif

void scanI2C() {
    Serial.println("\nScanning I2C bus...");
    byte count = 0;
    for (byte address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found device at 0x%02X\n", address);
            ++count;
        }
    }
    Serial.printf("Found %u device(s)\n", count);
}

void initSensors() {
    Wire.begin(SDA, SCL);
    Serial.begin(115200);
    scanI2C();
    sensorReady = sensorBackend.begin();
#if defined(MOONFINDER_SENSOR_BACKEND_LMS)
    Serial.println("Sensor backend: LSM303 accelerometer + LIS2MDL magnetometer");
#else
    Serial.println("Sensor backend: ICM-20948");
#endif
}

void updateSensors() {
    if (!sensorReady || !sensorBackend.update()) return;
    accPoint = sensorBackend.acceleration();
    magPoint = sensorBackend.magnetometer();
    gyroPoint = sensorBackend.gyroscope();
    accPointTrans = accPoint.norm() > 0.0 ? accPoint.normalized() : accPoint;
    magPointTrans = calibration.transform(magPoint);
    gyroPointTrans = gyroPoint;
}

void printSensorsToSerial(bool transformed) {
    Serial.print("Accel: ");
    printEigen(transformed ? accPointTrans : accPoint);
    Serial.print("Gyro: ");
    printEigen(transformed ? gyroPointTrans : gyroPoint);
    Serial.print("Mag: ");
    printEigen(transformed ? magPointTrans : magPoint);
}

void printSensorsToDisplay(bool transformed) {
    tft.setTextWrap(false);
    Eigen::Vector3d acc = transformed ? accPointTrans : accPoint;
    Eigen::Vector3d mag = transformed ? magPointTrans : magPoint;
    printVectorToDisplay("Acc: ", acc, 5);
    printVectorToDisplay("Mag: ", mag, 25);
    printVectorToDisplay("Gyro: ", transformed ? gyroPointTrans : gyroPoint, 45);
}

Eigen::Vector3d getAccelReading() { return accPointTrans; }
Eigen::Vector3d getMagReading() { return magPointTrans; }