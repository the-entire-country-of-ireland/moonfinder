#include "renderer_view.h"

#include <ArduinoEigen.h>
#include <cmath>
#include "display.h"
#include "imu_backend.h"
#include "isometric.h"
#include "ui_common.h"

namespace {
constexpr uint16_t HistoryCapacity = 256;
constexpr double HistoryMinChange = 0.005;
constexpr int SceneCenterX = SCREEN_WIDTH / 2;
constexpr int SceneCenterY = 168 - UiHeaderHeight;
constexpr float SceneScale = 58.0f;

Eigen::Vector3d accHistory[HistoryCapacity];
Eigen::Vector3d magHistory[HistoryCapacity];
uint16_t historyCount = 0;
uint16_t historyNext = 0;

void resetHistoryData() {
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

void drawProjectedLine(LGFX_Sprite& canvas,
                       const Eigen::Vector3d& a,
                       const Eigen::Vector3d& b,
                       uint16_t color) {
    const Eigen::Vector2d pa = projectPoint(a, currentRotation, SceneScale,
                                            SceneCenterX, SceneCenterY);
    const Eigen::Vector2d pb = projectPoint(b, currentRotation, SceneScale,
                                            SceneCenterX, SceneCenterY);
    canvas.drawLine(static_cast<int16_t>(pa.x()), static_cast<int16_t>(pa.y()),
                    static_cast<int16_t>(pb.x()), static_cast<int16_t>(pb.y()), color);
}

Eigen::Vector2d projectedPoint(const Eigen::Vector3d& value) {
    return projectPoint(value, currentRotation, SceneScale, SceneCenterX, SceneCenterY);
}

bool pointFitsContent(const Eigen::Vector2d& p) {
    constexpr int radius = 2;
    return p.x() >= radius && p.x() < SCREEN_WIDTH - radius &&
           p.y() >= radius && p.y() < UiContentHeight - radius;
}

void drawHistoryPointOnSprite(const Eigen::Vector3d& value, uint16_t color) {
    const Eigen::Vector2d p = projectedPoint(value);
    if (!pointFitsContent(p)) return;
    sprite.fillCircle(static_cast<int16_t>(p.x()), static_cast<int16_t>(p.y()), 2, color);
}

void drawHistoryPointOnScreen(const Eigen::Vector3d& value, uint16_t color) {
    const Eigen::Vector2d p = projectedPoint(value);
    if (!pointFitsContent(p)) return;
    tft.fillCircle(static_cast<int16_t>(p.x()),
                   static_cast<int16_t>(p.y()) + UiHeaderHeight,
                   2, color);
}

void drawAxesOnSprite() {
    const Eigen::Vector3d origin(0.0, 0.0, 0.0);
    const Eigen::Vector3d xAxis(1.4, 0.0, 0.0);
    const Eigen::Vector3d yAxis(0.0, 1.4, 0.0);
    const Eigen::Vector3d zAxis(0.0, 0.0, 1.4);

    drawProjectedLine(sprite, origin, xAxis, TFT_RED);
    drawProjectedLine(sprite, origin, yAxis, TFT_GREEN);
    drawProjectedLine(sprite, origin, zAxis, TFT_BLUE);

    const Eigen::Vector2d x = projectedPoint(xAxis);
    const Eigen::Vector2d y = projectedPoint(yAxis);
    const Eigen::Vector2d z = projectedPoint(zAxis);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setFont(&fonts::FreeSans9pt7b);
    sprite.setTextSize(1);
    sprite.setTextDatum(textdatum_t::middle_center);
    sprite.drawString("X", static_cast<int16_t>(x.x()), static_cast<int16_t>(x.y()));
    sprite.drawString("Y", static_cast<int16_t>(y.x()), static_cast<int16_t>(y.y()));
    sprite.drawString("Z", static_cast<int16_t>(z.x()), static_cast<int16_t>(z.y()));
    sprite.setTextDatum(textdatum_t::top_left);
}

void drawWireSphereOnSprite(uint16_t color) {
    constexpr int lonSteps = 16;
    constexpr int latSteps = 8;
    constexpr int ringSteps = 32;

    for (int i = 1; i < latSteps; ++i) {
        const float v = -HALF_PI + i * (PI / latSteps);
        const float z = std::sin(v);
        const float r = std::cos(v);
        Eigen::Vector2d prev;
        for (int j = 0; j <= ringSteps; ++j) {
            const float u = j * (2.0f * PI / ringSteps);
            const Eigen::Vector3d p(r * std::cos(u), r * std::sin(u), z);
            const Eigen::Vector2d q = projectedPoint(p);
            if (j > 0) {
                sprite.drawLine(static_cast<int16_t>(prev.x()), static_cast<int16_t>(prev.y()),
                                static_cast<int16_t>(q.x()), static_cast<int16_t>(q.y()), color);
            }
            prev = q;
        }
    }

    for (int i = 0; i < lonSteps; ++i) {
        const float theta = i * (2.0f * PI / lonSteps);
        for (int j = 0; j < ringSteps; ++j) {
            const float v1 = -HALF_PI + j * (PI / ringSteps);
            const float v2 = -HALF_PI + (j + 1) * (PI / ringSteps);
            const Eigen::Vector3d p1(std::cos(v1) * std::cos(theta),
                                     std::cos(v1) * std::sin(theta), std::sin(v1));
            const Eigen::Vector3d p2(std::cos(v2) * std::cos(theta),
                                     std::cos(v2) * std::sin(theta), std::sin(v2));
            drawProjectedLine(sprite, p1, p2, color);
        }
    }
}

void drawHistoryOnSprite() {
    const uint16_t first = historyCount == HistoryCapacity ? historyNext : 0;
    for (uint16_t i = 0; i < historyCount; ++i) {
        const uint16_t index = (first + i) % HistoryCapacity;
        drawHistoryPointOnSprite(accHistory[index], TFT_LIGHTBLUE);
        drawHistoryPointOnSprite(magHistory[index], TFT_PINK);
    }
}

void drawLegendOnSprite() {
    sprite.setFont(&fonts::Font0);
    sprite.setTextSize(1);
    sprite.setTextColor(TFT_LIGHTBLUE, TFT_BLACK);
    sprite.setCursor(7, 5);
    sprite.print("ACC");
    sprite.setTextColor(TFT_PINK, TFT_BLACK);
    sprite.setCursor(42, 5);
    sprite.print("MAG");
    sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    sprite.setCursor(78, 5);
    sprite.print("drag to rotate");
}

void renderWholeScene() {
    if (!uiEnsureContentSprite()) return;

    sprite.fillSprite(TFT_BLACK);
    drawAxesOnSprite();
    drawWireSphereOnSprite(TFT_DARKGREY);
    drawHistoryOnSprite();
    drawLegendOnSprite();
    uiPushContentSprite();
}

void drawNewestHistoryPairIncrementally() {
    if (!uiEnsureContentSprite() || historyCount == 0) return;
    const uint16_t index = (historyNext + HistoryCapacity - 1) % HistoryCapacity;

    // Keep the backing sprite synchronized with the LCD, but update only the
    // two tiny dirty regions on the physical display.
    drawHistoryPointOnSprite(accHistory[index], TFT_LIGHTBLUE);
    drawHistoryPointOnSprite(magHistory[index], TFT_PINK);
    drawHistoryPointOnScreen(accHistory[index], TFT_LIGHTBLUE);
    drawHistoryPointOnScreen(magHistory[index], TFT_PINK);
}
}  // namespace

void rendererDrag(int deltaX, int deltaY) {
    updateRotationFromTouch(deltaX, deltaY);
}

void drawRendererView(RTC& clock, bool redrawScene,
                      bool resetSensorHistory, bool clearScreen) {
    if (clearScreen) uiDrawPageChrome("3D SENSOR", clock, TFT_BLACK, true);
    if (resetSensorHistory) resetHistoryData();

    const bool newPoint = recordHistory();
    if (redrawScene || clearScreen) {
        renderWholeScene();
    } else if (newPoint) {
        drawNewestHistoryPairIncrementally();
    }
}
