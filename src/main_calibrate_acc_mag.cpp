#include <ArduinoEigen.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cmath>

#include "display.h"
#include "isometric.h"
#include "imu_backend.h"
#include "sd_card.h"
#include "rtc.h"
#include "online_calibration.h"
#include "AHRS.h"
#include "sector_calibrator.h"
#include "utils.h"
#include "i2c_config.h"

using Eigen::Vector3f;

enum class ScreenMode { Renderer3d, Moonfinder, ConfirmCalibration, Calibration };

// Screen field of view. Change these values when the optical/display setup changes.
constexpr double MoonfinderFovWidthDeg = 60.0;
constexpr double MoonfinderFovHeightDeg = 45.0;
constexpr double MoonAngularDiameterDeg = 0.52;
// Positive offsets move the telescope boresight relative to the IMU +Z axis.
double telescopePitchOffsetDeg = 0.0;
double telescopeYawOffsetDeg = 0.0;
constexpr double OffsetDragDegreesPerPixel = 0.025;

SDCard sdCard;
String foldername;
sector_calib::SectorCalibrator<float> calib;
ScreenMode screenMode = ScreenMode::Renderer3d;
int collectedSectors = 0;
int lastTouchX = -1;
int lastTouchY = -1;
bool lastTouchActive = false;
bool calibrationWaitingForSector = false;
unsigned long lastSampleUs = 0;
unsigned long lastRendererRefreshMs = 0;
unsigned long lastMoonPlotRefreshMs = 0;
RTC rtcClock;
NeuOrientation currentNeuOrientation;
MoonPosition currentMoonPosition;
constexpr uint16_t SensorHistoryCapacity = 256;
constexpr double SensorHistoryMinChange = 0.005;
Eigen::Vector3d accHistory[SensorHistoryCapacity];
Eigen::Vector3d magHistory[SensorHistoryCapacity];
uint16_t sensorHistoryCount = 0;
uint16_t sensorHistoryNext = 0;

void setupSDCard() {
    if (!sdCard.init()) {
        Serial.println("SD card unavailable; calibration will not be persisted.");
        return;
    }
    foldername = sdCard.createFolder("readings", "/data/magacc_pitch");
    sdCard.printDirectory("/data/magacc_pitch", 2);
        Eigen::Matrix<float, 3, 4> transform;
        float target = 0.0f;
        if (sdCard.loadCalibrationTransform(transform, target)) {
                calibration.load(transform.block<3, 3>(0, 0).cast<double>(),
                                                 transform.col(3).cast<double>());
                Serial.println("Loaded optimized calibration from SD card.");
        }
}

void startNewSector() {
    if (!sdCard.isInitialized()) return;
    String filename = sdCard.createFile("sector", foldername);
    if (filename.isEmpty()) {
        Serial.println("Failed to create sector file!");
    }
}

void resetSensorHistory() {
    sensorHistoryCount = 0;
    sensorHistoryNext = 0;
}

bool recordSensorHistory() {
    if (!sensorReady || accPointTrans.norm() == 0.0 || magPointTrans.norm() == 0.0) {
        return false;
    }
    if (sensorHistoryCount > 0) {
        const uint16_t last = (sensorHistoryNext + SensorHistoryCapacity - 1) % SensorHistoryCapacity;
        if ((accPointTrans - accHistory[last]).norm() < SensorHistoryMinChange &&
            (magPointTrans - magHistory[last]).norm() < SensorHistoryMinChange) {
            return false;
        }
    }
    accHistory[sensorHistoryNext] = accPointTrans;
    magHistory[sensorHistoryNext] = magPointTrans;
    sensorHistoryNext = (sensorHistoryNext + 1) % SensorHistoryCapacity;
    if (sensorHistoryCount < SensorHistoryCapacity) ++sensorHistoryCount;
    return true;
}

void drawSensorPoint(const Eigen::Vector3d& value, uint16_t color) {
    const int cx = tft.width() / 2;
    const int cy = tft.height() / 2 - 10;
    const float scale = 50.0f;
    drawPoint(projectPoint(value, currentRotation, scale, cx, cy), color);
}

void drawSensorHistory() {
    const uint16_t first = sensorHistoryCount == SensorHistoryCapacity
        ? sensorHistoryNext : 0;
    for (uint16_t i = 0; i < sensorHistoryCount; ++i) {
        const uint16_t index = (first + i) % SensorHistoryCapacity;
        drawSensorPoint(accHistory[index], TFT_LIGHTBLUE);
        drawSensorPoint(magHistory[index], TFT_PINK);
    }
}

void drawRendererScreen(bool redrawScene = false, bool resetHistory = false) {
    tft.setTextWrap(false);
    if (resetHistory) resetSensorHistory();
    const bool newSensorHistory = recordSensorHistory();
    if (redrawScene) {
        tft.fillScreen(TFT_BLACK);
        drawScene();
        drawSensorHistory();
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.fillRoundRect(0, 235, 115, 30, 5, TFT_DARKCYAN);
        tft.setTextColor(TFT_WHITE, TFT_DARKCYAN);
        tft.setFont(&fonts::FreeSans9pt7b);
        tft.drawString("MOON", 30, 242);
        tft.fillRoundRect(125, 235, 115, 30, 5, TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.setFont(&fonts::FreeSans9pt7b);
        tft.drawString("CALIBRATE", 132, 242);
    } else if (newSensorHistory) {
        const uint16_t index = (sensorHistoryNext + SensorHistoryCapacity - 1) % SensorHistoryCapacity;
        drawSensorPoint(accHistory[index], TFT_BLUE);
        drawSensorPoint(magHistory[index], TFT_RED);
    }
    printSensorsToDisplay(true);
}

double wrapAngleDegrees(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

Eigen::Vector3d computeTelescopeNeuVector() {
    constexpr double pi = 3.14159265358979323846;
    if (!currentNeuOrientation.valid) return Eigen::Vector3d::Zero();

    const Eigen::Vector3d imuBoresight = currentNeuOrientation.z_neu.normalized();
    const double imuYaw = std::atan2(imuBoresight.y(), imuBoresight.x());
    const double imuPitch = std::asin(std::max(-1.0, std::min(1.0, imuBoresight.z())));
    const double telescopeYaw = imuYaw + telescopeYawOffsetDeg * pi / 180.0;
    const double telescopePitch = imuPitch + telescopePitchOffsetDeg * pi / 180.0;
    return Eigen::Vector3d(std::cos(telescopePitch) * std::cos(telescopeYaw),
                           std::cos(telescopePitch) * std::sin(telescopeYaw),
                           std::sin(telescopePitch)).normalized();
}

void drawArrowHead(int16_t tipX, int16_t tipY, double angle, uint16_t color) {
    constexpr double pi = 3.14159265358979323846;
    const int16_t leftX = tipX - static_cast<int16_t>(10.0 * std::cos(angle - pi / 6.0));
    const int16_t leftY = tipY - static_cast<int16_t>(10.0 * std::sin(angle - pi / 6.0));
    const int16_t rightX = tipX - static_cast<int16_t>(10.0 * std::cos(angle + pi / 6.0));
    const int16_t rightY = tipY - static_cast<int16_t>(10.0 * std::sin(angle + pi / 6.0));
    tft.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
}

void drawMoonfinderScreen(bool redrawPlot = false, bool clearScreen = false) {
    constexpr double pi = 3.14159265358979323846;
    const int16_t plotLeft = 12;
    const int16_t plotTop = 100;
    const int16_t plotRight = tft.width() - 12;
    const int16_t plotBottom = 215;
    const int16_t centerX = (plotLeft + plotRight) / 2;
    const int16_t centerY = (plotTop + plotBottom) / 2;
    const double halfWidth = (plotRight - plotLeft) / 2.0;
    const double halfHeight = (plotBottom - plotTop) / 2.0;

    tft.setTextWrap(false);
    if (redrawPlot) {
        if (clearScreen) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setFont(&fonts::FreeSans12pt7b);
            tft.setCursor(3, 8);
            tft.print("MOONFINDER");
            tft.fillRoundRect(195, 5, 40, 26, 4, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
            tft.setFont(&fonts::FreeSans9pt7b);
            tft.drawString("3D", 204, 12);
        }

        tft.fillRect(plotLeft, plotTop, plotRight - plotLeft + 1,
                     plotBottom - plotTop + 1, TFT_BLACK);
        tft.drawRect(plotLeft, plotTop, plotRight - plotLeft, plotBottom - plotTop, TFT_DARKGREY);
        tft.drawLine(centerX, plotTop, centerX, plotBottom, TFT_DARKGREY);
        tft.drawLine(plotLeft, centerY, plotRight, centerY, TFT_DARKGREY);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.setCursor(plotLeft + 3, plotTop + 3);
        tft.printf("pitch +%.0f", MoonfinderFovHeightDeg / 2.0);
        tft.setCursor(plotLeft + 3, plotBottom - 20);
        tft.printf("pitch -%.0f", MoonfinderFovHeightDeg / 2.0);
        tft.setCursor(plotLeft + 4, plotBottom + 3);
        tft.printf("yaw -%.0f", MoonfinderFovWidthDeg / 2.0);
        tft.setCursor(plotRight - 60, plotBottom + 3);
        tft.printf("yaw +%.0f", MoonfinderFovWidthDeg / 2.0);
        lastMoonPlotRefreshMs = millis();
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(8, 62);
    tft.printf("Yaw offset %+6.1f deg       ", telescopeYawOffsetDeg);
    tft.setCursor(8, 80);
    tft.printf("Pitch offset %+5.1f deg      ", telescopePitchOffsetDeg);
    tft.setCursor(8, 298);
    tft.printf("%s        ", rtcClock.getDateTimeString().c_str());

    if (!currentNeuOrientation.valid || !currentMoonPosition.valid) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(22, centerY);
        tft.print("Waiting for orientation...                 ");
        return;
    }

    const Eigen::Vector3d moonNeu(currentMoonPosition.enu.y(),
                                  currentMoonPosition.enu.x(),
                                  currentMoonPosition.enu.z());
    const Eigen::Vector3d imuNeu = currentNeuOrientation.z_neu.normalized();
    const Eigen::Vector3d telescopeNeu = computeTelescopeNeuVector();
    const double currentYaw = std::atan2(telescopeNeu.y(), telescopeNeu.x()) * 180.0 / pi;
    const double currentPitch = std::asin(std::max(-1.0, std::min(1.0, telescopeNeu.z()))) * 180.0 / pi;
    const double moonYaw = std::atan2(moonNeu.y(), moonNeu.x()) * 180.0 / pi;
    const double moonPitch = std::asin(std::max(-1.0, std::min(1.0, moonNeu.z()))) * 180.0 / pi;
    const double yawDelta = wrapAngleDegrees(moonYaw - currentYaw);
    const double pitchDelta = moonPitch - currentPitch;
    const bool inFov = std::abs(yawDelta) <= MoonfinderFovWidthDeg / 2.0 &&
                       std::abs(pitchDelta) <= MoonfinderFovHeightDeg / 2.0;

    const double unclampedX = centerX + yawDelta / (MoonfinderFovWidthDeg / 2.0) * halfWidth;
    const double unclampedY = centerY - pitchDelta / (MoonfinderFovHeightDeg / 2.0) * halfHeight;
    const int16_t tipX = static_cast<int16_t>(std::max<double>(plotLeft + 8, std::min<double>(plotRight - 8, unclampedX)));
    const int16_t tipY = static_cast<int16_t>(std::max<double>(plotTop + 8, std::min<double>(plotBottom - 8, unclampedY)));
    if (redrawPlot) {
        tft.drawLine(centerX, centerY, tipX, tipY, TFT_YELLOW);
        drawArrowHead(tipX, tipY, std::atan2(tipY - centerY, tipX - centerX), TFT_YELLOW);
        tft.fillCircle(centerX, centerY, 3, TFT_WHITE);

        if (inFov) {
            const int16_t radius = std::max<int16_t>(2, static_cast<int16_t>(
                MoonAngularDiameterDeg / MoonfinderFovWidthDeg * (plotRight - plotLeft) / 2.0));
            tft.drawCircle(tipX, tipY, radius, TFT_YELLOW);
        }
    }

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(8, 45);
    tft.printf("Yaw %+6.1f  Pitch %+6.1f deg    ", yawDelta, pitchDelta);
    tft.setCursor(8, 238);
    tft.printf("IMU %+6.2f %+6.2f %+6.2f    ", imuNeu.x(), imuNeu.y(), imuNeu.z());
    tft.setCursor(8, 258);
    tft.printf("TEL %+6.2f %+6.2f %+6.2f    ", telescopeNeu.x(), telescopeNeu.y(), telescopeNeu.z());
    tft.setCursor(8, 278);
    tft.printf("MON %+6.2f %+6.2f %+6.2f    ", moonNeu.x(), moonNeu.y(), moonNeu.z());
}

void printMoonVectorsToSerial() {
    if (!currentNeuOrientation.valid || !currentMoonPosition.valid) return;
    const Eigen::Vector3d imuNeu = currentNeuOrientation.z_neu.normalized();
    const Eigen::Vector3d telescopeNeu = computeTelescopeNeuVector();
    const Eigen::Vector3d moonNeu(currentMoonPosition.enu.y(),
                                  currentMoonPosition.enu.x(),
                                  currentMoonPosition.enu.z());
    Serial.printf("Moonfinder IMU NEU=(%.4f, %.4f, %.4f) telescope NEU=(%.4f, %.4f, %.4f) moon NEU=(%.4f, %.4f, %.4f) offsets pitch=%.2f yaw=%.2f\n",
                  imuNeu.x(), imuNeu.y(), imuNeu.z(),
                  telescopeNeu.x(), telescopeNeu.y(), telescopeNeu.z(),
                  moonNeu.x(), moonNeu.y(), moonNeu.z(),
                  telescopePitchOffsetDeg, telescopeYawOffsetDeg);
}

void drawConfirmScreen() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(12, 40);
    tft.print("START CALIBRATION?");
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(12, 62);
    tft.print("  Move the sensor\n    through each sector.");
    tft.fillRoundRect(10, 120, 100, 45, 6, TFT_GREEN);
    tft.fillRoundRect(130, 120, 100, 45, 6, TFT_RED);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString("YES", 42, 138);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawString("NO", 168, 138);
}

void drawCalibrationScreen(bool redrawScreen = false, uint16_t topColor = TFT_GREEN) {
    const int mid = tft.height() / 2;
    if (redrawScreen) {
        tft.fillScreen(TFT_MAROON);
        tft.fillRect(0, 0, tft.width(), mid, topColor);
        tft.setTextColor(TFT_WHITE, TFT_MAROON);
        tft.setFont(&fonts::FreeSans12pt7b);
        tft.setCursor(5, mid + 40);
        tft.print("STOP CALIBRATION");
        tft.setFont(&fonts::FreeSans9pt7b);
        tft.setCursor(8, mid + 68);
        tft.print("BOTTOM:\n    OPTIMIZE + SAVE");
    }
    tft.setTextWrap(false);
    tft.setTextColor(TFT_BLACK, topColor);
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setCursor(8, 30);
    tft.print("CALIBRATING");
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(8, 56);
    if (calibrationWaitingForSector) {
        tft.print("Ready for next sector                         ");
    } else {
        tft.printf("Sector %d\n   %d / 500 samples    ", collectedSectors + 1,
                   calib.currentSectorSampleCount());
    }
    tft.setCursor(8, 102);
    tft.printf("TOP: %s                                      ",
               calibrationWaitingForSector ? "START SECTOR" : "FINISH SECTOR");
    if (redrawScreen) {
        tft.setTextColor(TFT_WHITE, TFT_MAROON);
        tft.setCursor(8, mid + 68);
        tft.printf("BOTTOM:\n    OPTIMIZE + SAVE\n    (%d done)       ", collectedSectors);
    }
}

bool finishCurrentSector() {
    if (!calib.isCollectingSectorSamples()) return false;
    const int sampleCount = calib.currentSectorSampleCount();
    if (sampleCount < 50) {
        calib.finishSector();
        dumpRuntime("short sector discarded", sampleCount, collectedSectors);
        return false;
    }
    dumpRuntime("sector finalize begin", sampleCount, collectedSectors);
    tft.fillScreen(TFT_ORANGE);
    bool wrote = calib.writeSectorSamplesToFile(sdCard);
    bool ended = calib.finishSector();
    if (ended) ++collectedSectors;
    dumpRuntime(wrote && ended ? "sector finalize ok" : "sector finalize failed",
                calib.currentSectorSampleCount(), collectedSectors);
    return wrote && ended;
}

void optimizeAndSaveCalibration() {
    finishCurrentSector();
    if (collectedSectors == 0) {
        screenMode = ScreenMode::Renderer3d;
        drawRendererScreen(true, true);
        return;
    }

    tft.fillScreen(TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setCursor(8, 45);
    tft.print("OPTIMIZING...");
    sector_calib::SectorCalibrator<float>::SolveOptions options;
    options.pitch_weight = 1.0f;
    options.norm_weight = 1.0f;
    options.dot_weight = 1.0f;
    options.max_iters = 100;
    auto result = calib.solve(options, true);

    calibration.load(result.W.block<3, 3>(0, 0).cast<double>(),
                     result.W.col(3).cast<double>());
    bool saved = sdCard.saveCalibrationTransform(result.W, result.c);
    Serial.printf("Calibration sectors=%d samples=%d cost=%.6f saved=%s\n",
                  collectedSectors, calib.totalAcceptedSampleCount(), result.cost,
                  saved ? "yes" : "no");
    screenMode = ScreenMode::Renderer3d;
    drawRendererScreen(true, true);
}

void handleTouch() {
    if (!touchActive || lastTouchActive) return;

    if (screenMode == ScreenMode::Renderer3d) {
        if (touchX >= 0 && touchX < 115 && touchY >= 230 && touchY < 270) {
            screenMode = ScreenMode::Moonfinder;
            drawMoonfinderScreen(true, true);
        } else if (touchX >= 125 && touchX < 240 && touchY >= 230 && touchY < 270) {
            screenMode = ScreenMode::ConfirmCalibration;
            drawConfirmScreen();
        }
    } else if (screenMode == ScreenMode::Moonfinder) {
        if (touchX >= 195 && touchX < 235 && touchY >= 5 && touchY < 31) {
            screenMode = ScreenMode::Renderer3d;
            drawRendererScreen(true, true);
        }
    } else if (screenMode == ScreenMode::ConfirmCalibration) {
        if (touchY >= 115 && touchY < 175 && touchX < 120) {
            collectedSectors = 0;
            calib.reset();
            screenMode = ScreenMode::Calibration;
            calibrationWaitingForSector = true;
            drawCalibrationScreen(true, TFT_YELLOW);
        } else if (touchY >= 115 && touchY < 175 && touchX >= 120) {
            screenMode = ScreenMode::Renderer3d;
            drawRendererScreen(true, true);
        }
    } else if (touchY < tft.height() / 2) {
        if (calibrationWaitingForSector) {
            calibrationWaitingForSector = false;
            startNewSector();
            calib.beginSector();
            drawCalibrationScreen(true, TFT_GREEN);
        } else {
            finishCurrentSector();
            calibrationWaitingForSector = true;
            drawCalibrationScreen(true, TFT_YELLOW);
        }
    } else {
        optimizeAndSaveCalibration();
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    esp_reset_reason_t reset_reason = esp_reset_reason();
    Serial.printf("\nBOOT reset_reason=%d %s\n", (int)reset_reason,
                  resetReasonName(reset_reason));
    initDisplay(0);
    rtcClock.init(MoonlightI2cSda, MoonlightI2cScl);
    rtcClock.syncSystemClock();
    Serial.printf("Current time: %s\n", rtcClock.getISO8601().c_str());
    initSensors();
    setupSDCard();
    initRotation(0.0f, -0.6f, 0.3f);
    drawRendererScreen(true, true);
}

void loop() {
    updateSensors();
    currentNeuOrientation = computeNeuOrientation(accPoint, magPointTrans);
    currentMoonPosition = computeMoonEnu(rtcClock.currentTime());
    // if (currentNeuOrientation.valid && currentMoonPosition.valid) {
    //     Serial.printf("NEU x=(%.3f, %.3f, %.3f) Moon ENU=(%.3f, %.3f, %.3f) az=%.1f el=%.1f\n",
    //                   currentNeuOrientation.x_neu.x(), currentNeuOrientation.x_neu.y(),
    //                   currentNeuOrientation.x_neu.z(), currentMoonPosition.enu.x(),
    //                   currentMoonPosition.enu.y(), currentMoonPosition.enu.z(),
    //                   currentMoonPosition.azimuth_deg, currentMoonPosition.elevation_deg);
    // }
    updateTouch();
    if (screenMode == ScreenMode::Moonfinder && touchActive && lastTouchActive &&
        lastTouchX >= 0 && lastTouchY >= 0) {
        const int deltaX = touchX - lastTouchX;
        const int deltaY = touchY - lastTouchY;
        if (deltaX != 0 || deltaY != 0) {
            telescopeYawOffsetDeg += deltaX * OffsetDragDegreesPerPixel;
            telescopePitchOffsetDeg -= deltaY * OffsetDragDegreesPerPixel;
            drawMoonfinderScreen(true, false);
        }
    }
    const unsigned long nowUs = micros();
    const float dt = lastSampleUs == 0 ? 0.0f : (nowUs - lastSampleUs) * 1.0e-6f;
    lastSampleUs = nowUs;

    if (screenMode == ScreenMode::Calibration &&
        calib.isCollectingSectorSamples() &&
        !calib.isSectorComplete()) {
        calib.recordSample(magPoint.cast<float>(), accPoint.cast<float>(),
                   gyroPoint.cast<float>(), dt);
        if (calib.isSectorComplete()) {
            if (finishCurrentSector()) {
                calibrationWaitingForSector = true;
                drawCalibrationScreen(true, TFT_YELLOW);
            }
        } else {
            drawCalibrationScreen();
        }
    } else if (screenMode == ScreenMode::Renderer3d && touchActive &&
               lastTouchActive && lastTouchX >= 0 && lastTouchY >= 0) {
        updateRotationFromTouch(touchX - lastTouchX, touchY - lastTouchY);
        drawRendererScreen(true);
    }

    if ((screenMode == ScreenMode::Renderer3d || screenMode == ScreenMode::Moonfinder) &&
        millis() - lastRendererRefreshMs >= 50) {
        lastRendererRefreshMs = millis();
        if (screenMode == ScreenMode::Renderer3d) drawRendererScreen();
        else {
            const bool refreshPlot = millis() - lastMoonPlotRefreshMs >= 50;
            drawMoonfinderScreen(refreshPlot, false);
            // printMoonVectorsToSerial();
        }
    }

    handleTouch();
    lastTouchActive = touchActive;
    lastTouchX = touchActive ? touchX : -1;
    lastTouchY = touchActive ? touchY : -1;
    delay(20);
}