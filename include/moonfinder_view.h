#pragma once

#include <ArduinoEigen.h>
#include "AHRS.h"
#include "rtc.h"

struct MoonfinderViewState {
    double azimuthOffsetDeg = 0.0;
    double elevationOffsetDeg = 0.0;
};

void drawMoonfinderView(const NeuOrientation& orientation,
                        const MoonPosition& moon,
                        RTC& clock,
                        MoonfinderViewState& state,
                        bool clearScreen = false);
void adjustMoonfinderOffsets(MoonfinderViewState& state, int deltaX, int deltaY);
Eigen::Vector3d computeTelescopeNeuVector(const NeuOrientation& orientation,
                                          const MoonfinderViewState& state);
