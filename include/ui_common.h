#pragma once

#include <Arduino.h>
#include "display.h"
#include "rtc.h"

enum class ScreenMode {
    Renderer3d,
    Moonfinder,
    Compass2d,
    ViewSelector,
    ConfirmCalibration,
    Calibration
};

struct UiRect {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;

    bool contains(int16_t px, int16_t py) const {
        return px >= x && px < x + w && py >= y && py < y + h;
    }
};

constexpr int16_t UiHeaderHeight = 50;
constexpr int16_t UiFooterTop = 280;

UiRect uiSwitchViewRect();
void uiDrawHeader(const char* title, RTC& clock, uint16_t background = TFT_BLACK);
void uiDrawButton(const UiRect& rect, const char* label,
                  uint16_t fillColor, uint16_t textColor = TFT_WHITE,
                  uint16_t borderColor = TFT_LIGHTGREY);
void uiDrawSwitchViewButton();
bool uiSwitchViewHit(int16_t x, int16_t y);
void uiDrawCenteredText(const String& text, int16_t y,
                        uint16_t color = TFT_WHITE,
                        uint16_t background = TFT_BLACK,
                        bool large = false);
double uiWrap360(double degrees);
double uiWrap180(double degrees);
const char* uiCardinal16(double headingDegrees);
