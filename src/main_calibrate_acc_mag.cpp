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
#include "online_calibration.h"
#include "AHRS.h"
#include "sector_calibrator.h"
#include "utils.h"

using Eigen::Vector3f;

enum class ScreenMode { Renderer, ConfirmCalibration, Calibration };

SDCard sdCard;
String foldername;
sector_calib::SectorCalibrator<float> calib;
ScreenMode screenMode = ScreenMode::Renderer;
int collectedSectors = 0;
int lastTouchX = -1;
int lastTouchY = -1;
bool lastTouchActive = false;
unsigned long lastSampleUs = 0;

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

void drawRendererScreen() {
    tft.fillScreen(TFT_BLACK);
    drawScene();
    printSensorsToDisplay(true);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillRoundRect(148, 8, 84, 30, 5, TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.drawString("CALIBRATE", 155, 28);
}

void drawConfirmScreen() {
    tft.fillScreen(TFT_NAVY);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setCursor(12, 55);
    tft.print("START CALIBRATION?");
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(12, 82);
    tft.print("Move the sensor through each sector.");
    tft.fillRoundRect(10, 120, 100, 45, 6, TFT_GREEN);
    tft.fillRoundRect(130, 120, 100, 45, 6, TFT_RED);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString("YES", 42, 148);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawString("NO", 168, 148);
}

void drawCalibrationScreen() {
    const int mid = tft.height() / 2;
    tft.fillScreen(TFT_MAROON);
    tft.fillRect(0, 0, tft.width(), mid, TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setCursor(8, 30);
    tft.print("CALIBRATING");
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(8, 56);
    tft.printf("Sector %d   %d / 500 samples", collectedSectors + 1,
               calib.currentSectorSampleCount());
    tft.setCursor(8, 82);
    tft.print("TOP: SAVE + NEXT SECTOR");
    tft.setTextColor(TFT_WHITE, TFT_MAROON);
    tft.setFont(&fonts::FreeSans12pt7b);
    tft.setCursor(8, mid + 40);
    tft.print("STOP CALIBRATION");
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setCursor(8, mid + 68);
    tft.printf("BOTTOM: OPTIMIZE + SAVE  (%d done)", collectedSectors);
}

bool finishCurrentSector() {
    if (!calib.isCollectingSectorSamples()) return false;
    dumpRuntime("sector finalize begin", calib.currentSectorSampleCount(), collectedSectors);
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
        screenMode = ScreenMode::Renderer;
        drawRendererScreen();
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
    screenMode = ScreenMode::Renderer;
    drawRendererScreen();
}

void handleTouch() {
    if (!touchActive || lastTouchActive) return;

    if (screenMode == ScreenMode::Renderer) {
        if (touchX < 135 && touchY >= 34 && touchY < 70) {
            screenMode = ScreenMode::ConfirmCalibration;
            drawConfirmScreen();
        }
    } else if (screenMode == ScreenMode::ConfirmCalibration) {
        if (touchY >= 115 && touchY < 175 && touchX < 120) {
            collectedSectors = 0;
            calib.reset();
            screenMode = ScreenMode::Calibration;
            startNewSector();
            calib.beginSector();
            drawCalibrationScreen();
        } else if (touchY >= 115 && touchY < 175 && touchX >= 120) {
            screenMode = ScreenMode::Renderer;
            drawRendererScreen();
        }
    } else if (touchY < tft.height() / 2) {
        finishCurrentSector();
        startNewSector();
        calib.beginSector();
        drawCalibrationScreen();
    } else {
        optimizeAndSaveCalibration();
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    esp_reset_reason_t reset_reason = esp_reset_reason();
    Serial.printf("\nBOOT reset_reason=%d %s\n", (int)reset_reason,
                  resetReasonName(reset_reason));
    initDisplay(0);
    initSensors();
    setupSDCard();
    initRotation(0.0f, -0.6f, 0.3f);
    drawRendererScreen();
}

void loop() {
    updateSensors();
    updateTouch();
    const unsigned long nowUs = micros();
    const float dt = lastSampleUs == 0 ? 0.0f : (nowUs - lastSampleUs) * 1.0e-6f;
    lastSampleUs = nowUs;

    if (screenMode == ScreenMode::Calibration &&
        calib.isCollectingSectorSamples() &&
        !calib.isSectorComplete()) {
        calib.recordSample(magPoint.cast<float>(), accPoint.cast<float>(),
                   gyroPoint.cast<float>(), dt);
        drawCalibrationScreen();
    } else if (screenMode == ScreenMode::Renderer && touchActive &&
               lastTouchActive && lastTouchX >= 0 && lastTouchY >= 0) {
        updateRotationFromTouch(touchX - lastTouchX, touchY - lastTouchY);
        drawRendererScreen();
    }

    handleTouch();
    lastTouchActive = touchActive;
    lastTouchX = touchActive ? touchX : -1;
    lastTouchY = touchActive ? touchY : -1;
    delay(40);
}