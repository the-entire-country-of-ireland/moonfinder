#include "compass_view.h"

#include <cmath>
#include <cstdio>
#include "display.h"
#include "ui_common.h"

namespace {
constexpr double Pi = 3.14159265358979323846;

void pointOnCompass(int16_t cx, int16_t cy, double radius, double angleDeg,
                    int16_t& x, int16_t& y) {
    const double angle = angleDeg * Pi / 180.0;
    x = static_cast<int16_t>(std::lround(cx + radius * std::sin(angle)));
    y = static_cast<int16_t>(std::lround(cy - radius * std::cos(angle)));
}

void drawCompassRose(double headingDeg) {
    const int16_t cx = tft.width() / 2;
    const int16_t cy = 174;
    const int16_t radius = 91;

    tft.fillRect(0, UiHeaderHeight, tft.width(), UiFooterTop - UiHeaderHeight, TFT_BLACK);
    tft.drawCircle(cx, cy, radius, TFT_LIGHTGREY);
    tft.drawCircle(cx, cy, radius - 2, TFT_DARKGREY);

    for (int bearing = 0; bearing < 360; bearing += 5) {
        const double relative = uiWrap180(static_cast<double>(bearing) - headingDeg);
        const bool major30 = (bearing % 30) == 0;
        const bool major10 = (bearing % 10) == 0;
        const int16_t tick = major30 ? 11 : (major10 ? 7 : 4);
        int16_t x1, y1, x2, y2;
        pointOnCompass(cx, cy, radius - 4, relative, x1, y1);
        pointOnCompass(cx, cy, radius - 4 - tick, relative, x2, y2);
        tft.drawLine(x1, y1, x2, y2, major30 ? TFT_WHITE : TFT_DARKGREY);
    }

    struct Cardinal { int bearing; const char* label; uint16_t color; };
    const Cardinal cardinals[] = {
        {0, "N", TFT_RED}, {90, "E", TFT_WHITE},
        {180, "S", TFT_WHITE}, {270, "W", TFT_WHITE}
    };
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setTextSize(1);
    tft.setTextDatum(textdatum_t::middle_center);
    for (const auto& cardinal : cardinals) {
        int16_t x, y;
        pointOnCompass(cx, cy, radius - 25,
                       uiWrap180(cardinal.bearing - headingDeg), x, y);
        tft.setTextColor(cardinal.color, TFT_BLACK);
        tft.drawString(cardinal.label, x, y);
    }
    tft.setTextDatum(textdatum_t::top_left);

    // Magnetic north needle in device-relative screen coordinates.
    int16_t northX, northY, southX, southY;
    pointOnCompass(cx, cy, radius - 34, -headingDeg, northX, northY);
    pointOnCompass(cx, cy, radius - 34, 180.0 - headingDeg, southX, southY);
    tft.drawLine(cx, cy, southX, southY, TFT_LIGHTGREY);
    tft.drawLine(cx + 1, cy, southX + 1, southY, TFT_LIGHTGREY);
    tft.drawLine(cx, cy, northX, northY, TFT_RED);
    tft.drawLine(cx + 1, cy, northX + 1, northY, TFT_RED);

    const double theta = (-headingDeg) * Pi / 180.0;
    const double leftTheta = theta - 0.16;
    const double rightTheta = theta + 0.16;
    const int16_t baseRadius = radius - 48;
    const int16_t leftX = static_cast<int16_t>(std::lround(cx + baseRadius * std::sin(leftTheta)));
    const int16_t leftY = static_cast<int16_t>(std::lround(cy - baseRadius * std::cos(leftTheta)));
    const int16_t rightX = static_cast<int16_t>(std::lround(cx + baseRadius * std::sin(rightTheta)));
    const int16_t rightY = static_cast<int16_t>(std::lround(cy - baseRadius * std::cos(rightTheta)));
    tft.fillTriangle(northX, northY, leftX, leftY, rightX, rightY, TFT_RED);
    tft.fillCircle(cx, cy, 6, TFT_WHITE);
    tft.fillCircle(cx, cy, 3, TFT_BLACK);

    // Fixed lubber line: the top of the display is the device forward direction.
    tft.fillTriangle(cx, cy - radius - 1, cx - 6, cy - radius + 11,
                     cx + 6, cy - radius + 11, TFT_CYAN);
}
}  // namespace

double compassHeadingDegrees(const NeuOrientation& orientation) {
    if (!orientation.valid) return NAN;

    // NEU convention: x=N, y=E, z=U.  The device +X axis is treated as
    // the compass forward direction.  atan2(E, N) gives 0=N, 90=E.
    Eigen::Vector3d forward = orientation.x_neu;
    forward.z() = 0.0;
    if (!forward.allFinite() || std::hypot(forward.x(), forward.y()) < 1e-9) return NAN;
    return uiWrap360(std::atan2(forward.y(), forward.x()) * 180.0 / Pi);
}

void drawCompassView(const NeuOrientation& orientation, RTC& clock, bool clearScreen) {
    if (clearScreen) tft.fillScreen(TFT_BLACK);
    uiDrawHeader("COMPASS", clock);
    uiDrawSwitchViewButton();

    const double heading = compassHeadingDegrees(orientation);
    if (!std::isfinite(heading)) {
        tft.fillRect(0, UiHeaderHeight, tft.width(), UiFooterTop - UiHeaderHeight, TFT_BLACK);
        uiDrawCenteredText("Waiting for orientation", 145, TFT_YELLOW, TFT_BLACK, false);
        return;
    }

    drawCompassRose(heading);

    char headingText[32];
    std::snprintf(headingText, sizeof(headingText), "%03.0f deg  %s",
                  heading, uiCardinal16(heading));
    uiDrawCenteredText(String(headingText), 53, TFT_WHITE, TFT_BLACK, true);
}
