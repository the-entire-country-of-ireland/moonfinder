#include "moonfinder_view.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include "display.h"
#include "ui_common.h"

namespace {
constexpr double Pi = 3.14159265358979323846;
constexpr double FovWidthDeg = 60.0;
constexpr double FovHeightDeg = 45.0;
constexpr double MoonAngularDiameterDeg = 0.52;
constexpr double OffsetDragDegreesPerPixel = 0.025;

void drawArrowHead(int16_t tipX, int16_t tipY, double angle, uint16_t color) {
    const int16_t leftX = tipX - static_cast<int16_t>(10.0 * std::cos(angle - Pi / 6.0));
    const int16_t leftY = tipY - static_cast<int16_t>(10.0 * std::sin(angle - Pi / 6.0));
    const int16_t rightX = tipX - static_cast<int16_t>(10.0 * std::cos(angle + Pi / 6.0));
    const int16_t rightY = tipY - static_cast<int16_t>(10.0 * std::sin(angle + Pi / 6.0));
    tft.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
}

double bearingFromNeu(const Eigen::Vector3d& v) {
    return uiWrap360(std::atan2(v.y(), v.x()) * 180.0 / Pi);
}

double elevationFromNeu(const Eigen::Vector3d& v) {
    return std::asin(std::max(-1.0, std::min(1.0, v.z()))) * 180.0 / Pi;
}
}  // namespace

Eigen::Vector3d computeTelescopeNeuVector(const NeuOrientation& orientation,
                                          const MoonfinderViewState& state) {
    if (!orientation.valid) return Eigen::Vector3d::Zero();

    const Eigen::Vector3d boresight = orientation.z_neu.normalized();
    const double azimuth = std::atan2(boresight.y(), boresight.x()) +
                           state.azimuthOffsetDeg * Pi / 180.0;
    const double elevation = std::asin(std::max(-1.0, std::min(1.0, boresight.z()))) +
                             state.elevationOffsetDeg * Pi / 180.0;
    return Eigen::Vector3d(std::cos(elevation) * std::cos(azimuth),
                           std::cos(elevation) * std::sin(azimuth),
                           std::sin(elevation)).normalized();
}

void adjustMoonfinderOffsets(MoonfinderViewState& state, int deltaX, int deltaY) {
    state.azimuthOffsetDeg = uiWrap180(state.azimuthOffsetDeg +
                                      deltaX * OffsetDragDegreesPerPixel);
    state.elevationOffsetDeg = std::max(-45.0, std::min(45.0,
        state.elevationOffsetDeg - deltaY * OffsetDragDegreesPerPixel));
}

void drawMoonfinderView(const NeuOrientation& orientation,
                        const MoonPosition& moon,
                        RTC& clock,
                        MoonfinderViewState& state,
                        bool clearScreen) {
    if (clearScreen) tft.fillScreen(TFT_BLACK);
    uiDrawHeader("MOONFINDER", clock);
    uiDrawSwitchViewButton();

    const int16_t plotLeft = 8;
    const int16_t plotTop = 108;
    const int16_t plotRight = tft.width() - 9;
    const int16_t plotBottom = 270;
    const int16_t centerX = (plotLeft + plotRight) / 2;
    const int16_t centerY = (plotTop + plotBottom) / 2;
    const double halfWidth = (plotRight - plotLeft) / 2.0;
    const double halfHeight = (plotBottom - plotTop) / 2.0;

    tft.fillRect(0, UiHeaderHeight, tft.width(), UiFooterTop - UiHeaderHeight, TFT_BLACK);

    char line[48];
    if (moon.valid) {
        std::snprintf(line, sizeof(line), "Moon Az %5.1f  El %+5.1f",
                      moon.azimuth_deg, moon.elevation_deg);
        uiDrawCenteredText(String(line), 51, TFT_YELLOW, TFT_BLACK, false);
    } else {
        uiDrawCenteredText("Moon position unavailable", 51, TFT_YELLOW, TFT_BLACK, false);
    }

    std::snprintf(line, sizeof(line), "Offset Az %+4.1f  El %+4.1f",
                  state.azimuthOffsetDeg, state.elevationOffsetDeg);
    uiDrawCenteredText(String(line), 87, TFT_LIGHTGREY, TFT_BLACK, false);

    tft.fillRect(plotLeft, plotTop, plotRight - plotLeft + 1,
                 plotBottom - plotTop + 1, TFT_BLACK);
    tft.drawRoundRect(plotLeft, plotTop, plotRight - plotLeft + 1,
                      plotBottom - plotTop + 1, 6, TFT_DARKGREY);
    tft.drawLine(centerX, plotTop + 2, centerX, plotBottom - 2, TFT_DARKGREY);
    tft.drawLine(plotLeft + 2, centerY, plotRight - 2, centerY, TFT_DARKGREY);
    tft.fillCircle(centerX, centerY, 3, TFT_WHITE);

    if (!orientation.valid || !moon.valid) {
        uiDrawCenteredText("Waiting for attitude", centerY - 8, TFT_YELLOW, TFT_BLACK, false);
        return;
    }

    const Eigen::Vector3d telescopeNeu = computeTelescopeNeuVector(orientation, state);
    const Eigen::Vector3d moonNeu(moon.enu.y(), moon.enu.x(), moon.enu.z());
    const double telescopeAz = bearingFromNeu(telescopeNeu);
    const double telescopeEl = elevationFromNeu(telescopeNeu);
    const double azDelta = uiWrap180(moon.azimuth_deg - telescopeAz);
    const double elDelta = moon.elevation_deg - telescopeEl;

    std::snprintf(line, sizeof(line), "Delta Az %+5.1f  El %+5.1f",
                  azDelta, elDelta);
    uiDrawCenteredText(String(line), 69, TFT_WHITE, TFT_BLACK, false);

    const bool inFov = std::abs(azDelta) <= FovWidthDeg / 2.0 &&
                       std::abs(elDelta) <= FovHeightDeg / 2.0;
    const double unclampedX = centerX + azDelta / (FovWidthDeg / 2.0) * halfWidth;
    const double unclampedY = centerY - elDelta / (FovHeightDeg / 2.0) * halfHeight;
    const int16_t tipX = static_cast<int16_t>(std::max<double>(plotLeft + 9,
        std::min<double>(plotRight - 9, unclampedX)));
    const int16_t tipY = static_cast<int16_t>(std::max<double>(plotTop + 9,
        std::min<double>(plotBottom - 9, unclampedY)));

    tft.drawLine(centerX, centerY, tipX, tipY, TFT_YELLOW);
    drawArrowHead(tipX, tipY, std::atan2(tipY - centerY, tipX - centerX), TFT_YELLOW);

    if (inFov) {
        const int16_t radius = std::max<int16_t>(2, static_cast<int16_t>(
            MoonAngularDiameterDeg / FovWidthDeg * (plotRight - plotLeft)));
        tft.drawCircle(tipX, tipY, radius, TFT_YELLOW);
        tft.drawCircle(tipX, tipY, radius + 1, TFT_YELLOW);
    }
}
