#pragma once

#include <ArduinoEigen.h>
#include <esp_system.h>

void printMatrix3d(const Eigen::Matrix3d& mat);
void printVector3d(const Eigen::Vector3d& vec);
const char* resetReasonName(esp_reset_reason_t reason);
void dumpRuntime(const char* where, int sectorSampleCount, int collectedSectors);