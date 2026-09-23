#pragma once

#include "rtc.h"

enum class CalibrationUiAction {
    None,
    ConfirmYes,
    ConfirmNo,
    SectorToggle,
    OptimizeSave,
    SwitchView
};

void drawCalibrationConfirmView(RTC& clock);
CalibrationUiAction calibrationConfirmTouch(int16_t x, int16_t y);

void drawCalibrationView(RTC& clock, bool waitingForSector,
                         int collectedSectors, int currentSamples,
                         bool clearScreen = false);
void drawCalibrationWorkingView(RTC& clock, const char* message);
CalibrationUiAction calibrationViewTouch(int16_t x, int16_t y);
