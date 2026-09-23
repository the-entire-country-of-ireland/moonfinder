#include "renderer_view.h"

#include <ArduinoEigen.h>
#include "display.h"
#include "imu_backend.h"
#include "isometric.h"
#include "ui_common.h"

namespace {
constexpr uint16_t HistoryCapacity = 256;
constexpr double HistoryMinChange = 0.005;
Eigen::Vector3d accHistory[HistoryCapacity];
Eigen::Vector3d magHistory[HistoryCapacity];
uint16_t historyCount = 0;
uint16_t historyNext = 0;

void resetHistory() {
    historyCount = 0;
    historyNext = 0;
}

bool recordHistory() {
    if (!sensorReady || accPointTrans.norm() < 1e-9 || magPointTrans.norm() < 1e-9) return false;
    if (historyCount > 0) {
        const uint16_t last = (historyNext + HistoryCapacity - 1) % HistoryCapacity;
        if ((accPointTrans - accHistory[last]).norm() < HistoryMinChange &&
            (magPointTrans - magHistory[last]).norm() < HistoryMinChange) return false;
    }
    accHistory[historyNext] = accPointTrans;
    magHistory[historyNext] = magPointTrans;
    historyNext = (historyNext + 1) % HistoryCapacity;
    if (historyCount < HistoryCapacity) ++historyCount;
    return true;
}

void drawHistoryPoint(const Eigen::Vector3d& value, uint16_t color) {
    const int cx = tft.width() / 2;
    const int cy = 168;
    const float scale = 58.0f;
    drawPoint(projectPoint(value, currentRotation, scale, cx, cy), color);
}

void drawHistory() {
    const uint16_t first = historyCount == HistoryCapacity ? historyNext : 0;
    for (uint16_t i = 0; i < historyCount; ++i) {
        const uint16_t index = (first + i) % HistoryCapacity;
        drawHistoryPoint(accHistory[index], TFT_LIGHTBLUE);
        drawHistoryPoint(magHistory[index], TFT_PINK);
    }
}

void drawSceneArea() {
    tft.fillRect(0, UiHeaderHeight, tft.width(), UiFooterTop - UiHeaderHeight, TFT_BLACK);
    const int cx = tft.width() / 2;
    const int cy = 168;
    const float scale = 58.0f;
    drawAxes(currentRotation, scale, cx, cy);
    drawWireSphere(currentRotation, scale, cx, cy, TFT_DARKGREY);
    drawHistory();

    tft.setFont(&fonts::Font0);
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTBLUE, TFT_BLACK);
    tft.setCursor(7, 55);
    tft.print("ACC");
    tft.setTextColor(TFT_PINK, TFT_BLACK);
    tft.setCursor(42, 55);
    tft.print("MAG");
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(78, 55);
    tft.print("drag to rotate");
}
}  // namespace

void rendererDrag(int deltaX, int deltaY) {
    updateRotationFromTouch(deltaX, deltaY);
}

void drawRendererView(RTC& clock, bool redrawScene, bool resetSensorHistory) {
    if (resetSensorHistory) resetHistory();
    const bool newPoint = recordHistory();
    uiDrawHeader("3D SENSOR", clock);
    uiDrawSwitchViewButton();

    if (redrawScene) {
        drawSceneArea();
    } else if (newPoint) {
        const uint16_t index = (historyNext + HistoryCapacity - 1) % HistoryCapacity;
        drawHistoryPoint(accHistory[index], TFT_LIGHTBLUE);
        drawHistoryPoint(magHistory[index], TFT_PINK);
    }
}
