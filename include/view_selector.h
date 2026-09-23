#pragma once

#include "rtc.h"

enum class ViewSelectorChoice {
    None,
    Compass,
    Moonfinder,
    Renderer3d,
    Calibration,
    Back
};

void drawViewSelector(RTC& clock);
ViewSelectorChoice viewSelectorTouch(int16_t x, int16_t y);
