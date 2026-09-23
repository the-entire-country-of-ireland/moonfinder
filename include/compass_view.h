#pragma once

#include "AHRS.h"
#include "rtc.h"

void drawCompassView(const NeuOrientation& orientation, RTC& clock, bool clearScreen = false);
double compassHeadingDegrees(const NeuOrientation& orientation);
