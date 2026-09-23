#include "ui_common.h"

#include <cmath>

UiRect uiSwitchViewRect() {
    return UiRect{12, UiFooterTop + 3, static_cast<int16_t>(tft.width() - 24), 33};
}

void uiDrawCenteredText(const String& text, int16_t y,
                        uint16_t color, uint16_t background, bool large) {
    tft.setTextWrap(false);
    tft.setTextSize(1);
    tft.setFont(large ? &fonts::FreeSans12pt7b : &fonts::FreeSans9pt7b);
    tft.setTextColor(color, background);
    tft.setTextDatum(textdatum_t::top_center);
    tft.drawString(text, tft.width() / 2, y);
    tft.setTextDatum(textdatum_t::top_left);
}

void uiDrawHeader(const char* title, RTC& clock, uint16_t background) {
    tft.fillRect(0, 0, tft.width(), UiHeaderHeight, background);
    uiDrawCenteredText(String(title), 2, TFT_WHITE, background, true);

    String stamp = clock.getEasternDateTimeString();
    stamp += " ET";
    tft.setTextWrap(false);
    tft.setTextSize(1);
    tft.setFont(&fonts::Font0);
    tft.setTextColor(TFT_LIGHTGREY, background);
    tft.setTextDatum(textdatum_t::top_center);
    tft.drawString(stamp, tft.width() / 2, 32);
    tft.setTextDatum(textdatum_t::top_left);
}

void uiDrawButton(const UiRect& rect, const char* label,
                  uint16_t fillColor, uint16_t textColor,
                  uint16_t borderColor) {
    tft.fillRoundRect(rect.x, rect.y, rect.w, rect.h, 7, fillColor);
    tft.drawRoundRect(rect.x, rect.y, rect.w, rect.h, 7, borderColor);
    tft.setTextWrap(false);
    tft.setTextSize(1);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setTextColor(textColor, fillColor);
    tft.setTextDatum(textdatum_t::middle_center);
    tft.drawString(label, rect.x + rect.w / 2, rect.y + rect.h / 2);
    tft.setTextDatum(textdatum_t::top_left);
}

void uiDrawSwitchViewButton() {
    uiDrawButton(uiSwitchViewRect(), "SWITCH VIEW", TFT_DARKCYAN);
}

bool uiSwitchViewHit(int16_t x, int16_t y) {
    return uiSwitchViewRect().contains(x, y);
}

double uiWrap360(double degrees) {
    while (degrees < 0.0) degrees += 360.0;
    while (degrees >= 360.0) degrees -= 360.0;
    return degrees;
}

double uiWrap180(double degrees) {
    degrees = uiWrap360(degrees);
    if (degrees > 180.0) degrees -= 360.0;
    return degrees;
}

const char* uiCardinal16(double headingDegrees) {
    static const char* labels[] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    const int index = static_cast<int>(std::floor((uiWrap360(headingDegrees) + 11.25) / 22.5)) & 15;
    return labels[index];
}
