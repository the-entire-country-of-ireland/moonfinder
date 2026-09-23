#pragma once

#include <ArduinoEigen.h>
#include "online_calibration.h"
#include "display.h"

class ImuBackend {
public:
    virtual ~ImuBackend() = default;

    virtual bool begin() = 0;
    virtual bool update() = 0;
    virtual bool hasGyroscope() const = 0;

    const Eigen::Vector3d& acceleration() const { return acceleration_; }
    const Eigen::Vector3d& magnetometer() const { return magnetometer_; }
    const Eigen::Vector3d& gyroscope() const { return gyroscope_; }

protected:
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d magnetometer_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroscope_ = Eigen::Vector3d::Zero();
};

extern Eigen::Vector3d magPoint;
extern Eigen::Vector3d accPoint;
extern Eigen::Vector3d gyroPoint;
extern Eigen::Vector3d magPointTrans;
extern Eigen::Vector3d accPointTrans;
extern Eigen::Vector3d gyroPointTrans;
extern bool sensorReady;
extern AffineFinder calibration;

void scanI2C();
void initSensors();
bool updateSensors();
void printSensorsToSerial(bool transformed = true);
void printSensorsToDisplay(bool transformed = true, int heightOffset = 0);
Eigen::Vector3d getAccelReading();
Eigen::Vector3d getMagReading();