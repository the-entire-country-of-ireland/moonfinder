#include <Arduino.h>
#include <ArduinoEigen.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_sntp.h>
#include <time.h>

#include "AHRS.h"
#include "calibration_view.h"
#include "compass_view.h"
#include "display.h"
#include "i2c_config.h"
#include "imu_backend.h"
#include "isometric.h"
#include "moonfinder_view.h"
#include "renderer_view.h"
#include "rtc.h"
#include "sd_card.h"
#include "sector_calibrator.h"
#include "ui_common.h"
#include "utils.h"
#include "view_selector.h"

namespace {
constexpr char WifiSsid[] = "Verizon_4VHJ97";
constexpr char WifiPassword[] = "rid-spare3-wiry";
constexpr unsigned long WifiConnectTimeoutMs = 15000;
constexpr unsigned long NtpSyncTimeoutMs = 10000;
constexpr unsigned long MoonUpdatePeriodMs = 1000;
constexpr unsigned long UiRefreshPeriodMs = 80;
constexpr double AttitudeEmaAlpha = 0.12;

SDCard sdCard;
String foldername;
bool sdCardReady = false;
sector_calib::SectorCalibrator<float> calib;

ScreenMode screenMode = ScreenMode::Compass2d;
ScreenMode previousScreenMode = ScreenMode::Compass2d;
MoonfinderViewState moonfinderState;

int collectedSectors = 0;
bool calibrationWaitingForSector = false;
bool calibrationSessionActive = false;

int lastTouchX = -1;
int lastTouchY = -1;
bool lastTouchActive = false;
unsigned long lastSampleUs = 0;
unsigned long lastUiRefreshMs = 0;
unsigned long lastMoonUpdateMs = 0;

RTC rtcClock;
NeuOrientation currentNeuOrientation;
MoonPosition currentMoonPosition;

Eigen::Vector3d accPointEma = Eigen::Vector3d::Zero();
Eigen::Vector3d magPointTransEma = Eigen::Vector3d::Zero();
bool attitudeEmaInitialized = false;
volatile bool ntpTimeAdjusted = false;

void onNtpTimeAdjusted(struct timeval*) {
    ntpTimeAdjusted = true;
}

void resetAttitudeEma() {
    accPointEma.setZero();
    magPointTransEma.setZero();
    attitudeEmaInitialized = false;
}

void updateAttitudeEma() {
    // Smooth the quantities used by attitude estimation only.  The
    // accelerometer is filtered before normalization; the magnetometer is
    // filtered after affine calibration.  Raw/calibration and 3-D data remain
    // completely untouched.
    if (!accPoint.allFinite() || !magPointTrans.allFinite() ||
        accPoint.norm() < 1e-9 || magPointTrans.norm() < 1e-9) {
        return;
    }

    if (!attitudeEmaInitialized) {
        accPointEma = accPoint;
        magPointTransEma = magPointTrans;
        attitudeEmaInitialized = true;
        return;
    }

    accPointEma += AttitudeEmaAlpha * (accPoint - accPointEma);
    magPointTransEma += AttitudeEmaAlpha * (magPointTrans - magPointTransEma);
}

bool syncTimeFromWiFi() {
    if (WifiSsid[0] == '\0') {
        Serial.println("WiFi time sync skipped; configure WifiSsid and WifiPassword.");
        return false;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(WifiSsid, WifiPassword);
    Serial.print("Connecting to WiFi for time sync");
    const unsigned long connectionStart = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - connectionStart < WifiConnectTimeoutMs) {
        delay(50);
        Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(" failed; using RTC or compile-time fallback.");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return false;
    }

    Serial.println(" connected.");

    // Do not use getLocalTime() as the synchronization test here.  It only
    // checks whether the system clock looks plausible, so a stale RTC-seeded
    // system clock can make it return immediately before an SNTP packet has
    // arrived.  Wait for the ESP32 SNTP adjustment callback instead.
    ntpTimeAdjusted = false;
    sntp_set_time_sync_notification_cb(onNtpTimeAdjusted);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    const unsigned long syncStart = millis();
    while (!ntpTimeAdjusted && millis() - syncStart < NtpSyncTimeoutMs) {
        delay(25);
    }

    if (!ntpTimeAdjusted) {
        Serial.println("NTP time sync failed; using RTC or compile-time fallback.");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        return false;
    }

    const time_t systemTime = time(nullptr);
    const bool rtcUpdated = systemTime > 0 &&
                            rtcClock.setUtcUnixTime(static_cast<uint32_t>(systemTime));
    const bool systemClockReady = rtcClock.useSystemClock();
    Serial.printf("NTP synchronized UTC: %s  RTC writeback: %s\n",
                  systemClockReady ? rtcClock.getISO8601().c_str() : "no",
                  rtcUpdated ? "yes" : "no");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return systemClockReady;
}

void setupSDCard() {
    if (!sdCard.init()) {
        Serial.println("SD card unavailable; calibration will not be persisted.");
        return;
    }

    sdCardReady = true;
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

void startNewSectorFile() {
    if (!sdCardReady) {
        Serial.println("SD card unavailable; sector will be held in RAM only.");
        return;
    }
    const String filename = sdCard.createFile("sector", foldername);
    if (filename.isEmpty()) Serial.println("Failed to create sector file!");
}

void drawCurrentView(bool clearScreen = false, bool resetRendererHistory = false) {
    switch (screenMode) {
        case ScreenMode::Compass2d:
            drawCompassView(currentNeuOrientation, rtcClock, clearScreen);
            break;
        case ScreenMode::Moonfinder:
            drawMoonfinderView(currentNeuOrientation, currentMoonPosition,
                               rtcClock, moonfinderState, clearScreen);
            break;
        case ScreenMode::Renderer3d:
            drawRendererView(rtcClock, true, resetRendererHistory, clearScreen);
            break;
        case ScreenMode::ViewSelector:
            drawViewSelector(rtcClock);
            break;
        case ScreenMode::ConfirmCalibration:
            drawCalibrationConfirmView(rtcClock);
            break;
        case ScreenMode::Calibration:
            drawCalibrationView(rtcClock, calibrationWaitingForSector,
                                collectedSectors,
                                calib.currentSectorSampleCount(), clearScreen);
            break;
    }
}

void setScreen(ScreenMode mode, bool resetRendererHistory = false) {
    screenMode = mode;
    drawCurrentView(true, resetRendererHistory);
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
    drawCalibrationWorkingView(rtcClock, "SAVING SECTOR...");
    const bool wrote = sdCardReady && calib.writeSectorSamplesToFile(sdCard);
    const bool ended = calib.finishSector();
    if (ended) ++collectedSectors;
    dumpRuntime(wrote && ended ? "sector finalize ok" : "sector finalize failed",
                calib.currentSectorSampleCount(), collectedSectors);
    return ended;
}

void enterViewSelector() {
    // A half-recorded sector is not allowed to continue invisibly while another
    // page is displayed.  Finish it (or discard if too short) before leaving.
    if (screenMode == ScreenMode::Calibration && calib.isCollectingSectorSamples()) {
        finishCurrentSector();
        calibrationWaitingForSector = true;
    }
    previousScreenMode = screenMode;
    setScreen(ScreenMode::ViewSelector);
}

void optimizeAndSaveCalibration() {
    finishCurrentSector();
    calibrationWaitingForSector = true;

    if (collectedSectors == 0) {
        calibrationSessionActive = false;
        setScreen(ScreenMode::Renderer3d, true);
        return;
    }

    drawCalibrationWorkingView(rtcClock, "OPTIMIZING...");
    sector_calib::SectorCalibrator<float>::SolveOptions options;
    options.pitch_weight = 1.0f;
    options.norm_weight = 1.0f;
    options.dot_weight = 1.0f;
    options.max_iters = 100;
    const auto result = calib.solve(options, true);

    calibration.load(result.W.block<3, 3>(0, 0).cast<double>(),
                     result.W.col(3).cast<double>());
    resetAttitudeEma();
    const bool saved = sdCardReady && sdCard.saveCalibrationTransform(result.W, result.c);
    Serial.printf("Calibration sectors=%d samples=%d cost=%.6f saved=%s\n",
                  collectedSectors, calib.totalAcceptedSampleCount(), result.cost,
                  saved ? "yes" : "no");

    calibrationSessionActive = false;
    setScreen(ScreenMode::Renderer3d, true);
}

void handleViewSelectorChoice(ViewSelectorChoice choice) {
    switch (choice) {
        case ViewSelectorChoice::Compass:
            setScreen(ScreenMode::Compass2d);
            break;
        case ViewSelectorChoice::Moonfinder:
            setScreen(ScreenMode::Moonfinder);
            break;
        case ViewSelectorChoice::Renderer3d:
            setScreen(ScreenMode::Renderer3d, true);
            break;
        case ViewSelectorChoice::Calibration:
            if (calibrationSessionActive) setScreen(ScreenMode::Calibration);
            else setScreen(ScreenMode::ConfirmCalibration);
            break;
        case ViewSelectorChoice::Back:
            setScreen(previousScreenMode,
                      previousScreenMode == ScreenMode::Renderer3d);
            break;
        case ViewSelectorChoice::None:
            break;
    }
}

void handleTouchStart() {
    if (!touchActive || lastTouchActive) return;

    if (screenMode == ScreenMode::ViewSelector) {
        handleViewSelectorChoice(viewSelectorTouch(touchX, touchY));
        return;
    }

    if (screenMode == ScreenMode::ConfirmCalibration) {
        const CalibrationUiAction action = calibrationConfirmTouch(touchX, touchY);
        if (action == CalibrationUiAction::ConfirmYes) {
            collectedSectors = 0;
            calib.reset();
            calibrationSessionActive = true;
            calibrationWaitingForSector = true;
            setScreen(ScreenMode::Calibration);
        } else if (action == CalibrationUiAction::ConfirmNo) {
            setScreen(ScreenMode::ViewSelector);
        }
        return;
    }

    if (screenMode == ScreenMode::Calibration) {
        const CalibrationUiAction action = calibrationViewTouch(touchX, touchY);
        if (action == CalibrationUiAction::SwitchView) {
            enterViewSelector();
        } else if (action == CalibrationUiAction::SectorToggle) {
            if (calibrationWaitingForSector) {
                startNewSectorFile();
                calib.beginSector();
                calibrationWaitingForSector = false;
                // Only status/buttons changed; keep the rest of the page intact.
                drawCurrentView(false);
            } else {
                // finishCurrentSector() intentionally shows a transient working
                // page while writing, so restore the calibration page afterward.
                finishCurrentSector();
                calibrationWaitingForSector = true;
                drawCurrentView(true);
            }
        } else if (action == CalibrationUiAction::OptimizeSave) {
            optimizeAndSaveCalibration();
        }
        return;
    }

    if (uiSwitchViewHit(touchX, touchY)) enterViewSelector();
}

void handleTouchDrag() {
    if (!touchActive || !lastTouchActive || lastTouchX < 0 || lastTouchY < 0) return;
    if (touchY >= UiFooterTop || lastTouchY >= UiFooterTop ||
        touchY < UiHeaderHeight || lastTouchY < UiHeaderHeight) return;

    const int deltaX = touchX - lastTouchX;
    const int deltaY = touchY - lastTouchY;
    if (deltaX == 0 && deltaY == 0) return;

    if (screenMode == ScreenMode::Moonfinder) {
        adjustMoonfinderOffsets(moonfinderState, deltaX, deltaY);
        drawMoonfinderView(currentNeuOrientation, currentMoonPosition,
                           rtcClock, moonfinderState, false);
    } else if (screenMode == ScreenMode::Renderer3d) {
        rendererDrag(deltaX, deltaY);
        drawRendererView(rtcClock, true, false, false);
    }
}

void recordCalibrationSample(float dt) {
    if (screenMode != ScreenMode::Calibration ||
        !calib.isCollectingSectorSamples() || calib.isSectorComplete()) {
        return;
    }

    calib.recordSample(magPoint.cast<float>(), accPoint.cast<float>(),
                       gyroPoint.cast<float>(), dt);
    if (calib.isSectorComplete()) {
        finishCurrentSector();
        calibrationWaitingForSector = true;
        drawCurrentView(true);
    }
}

uint16_t activePageBackground() {
    switch (screenMode) {
        case ScreenMode::ViewSelector:
        case ScreenMode::ConfirmCalibration:
            return TFT_NAVY;
        default:
            return TFT_BLACK;
    }
}

void refreshActiveView() {
    // The title/footer are static.  Updating only the timestamp band prevents
    // the header from flashing at the 80 ms UI cadence.
    uiUpdateHeaderClock(rtcClock, activePageBackground(), false);

    switch (screenMode) {
        case ScreenMode::Compass2d:
            drawCompassView(currentNeuOrientation, rtcClock, false);
            break;
        case ScreenMode::Moonfinder:
            drawMoonfinderView(currentNeuOrientation, currentMoonPosition,
                               rtcClock, moonfinderState, false);
            break;
        case ScreenMode::Renderer3d:
            drawRendererView(rtcClock, false, false, false);
            break;
        case ScreenMode::Calibration:
            drawCalibrationView(rtcClock, calibrationWaitingForSector,
                                collectedSectors, calib.currentSectorSampleCount(), false);
            break;
        case ScreenMode::ViewSelector:
        case ScreenMode::ConfirmCalibration:
            // Static pages: only the clock band above needs periodic service.
            break;
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(100);
    const esp_reset_reason_t resetReason = esp_reset_reason();
    Serial.printf("\nBOOT reset_reason=%d %s\n", static_cast<int>(resetReason),
                  resetReasonName(resetReason));

    initDisplay(2);  // Portrait: 240 x 320 on the CYD.
    rtcClock.init(MoonlightI2cSda, MoonlightI2cScl);
    // Try real SNTP first.  Only seed the ESP32 system clock from the hardware
    // RTC when network synchronization actually fails.
    if (!syncTimeFromWiFi()) {
        rtcClock.syncSystemClock();
    }
    Serial.printf("Current UTC: %s\n", rtcClock.getISO8601().c_str());
    Serial.printf("Current Eastern: %s ET\n",
                  rtcClock.getEasternDateTimeString().c_str());

    initSensors();
    setupSDCard();
    initRotation(0.0f, -0.6f, 0.3f);
    currentNeuOrientation = computeNeuOrientation(Eigen::Vector3d::Zero(),
                                                  Eigen::Vector3d::Zero());

    currentMoonPosition = computeMoonEnu(rtcClock.currentTime());
    lastMoonUpdateMs = millis();
    setScreen(ScreenMode::Compass2d);
}

void loop() {
    const bool freshSensorSample = updateSensors();
    if (freshSensorSample) updateAttitudeEma();
    if (attitudeEmaInitialized) {
        currentNeuOrientation = computeNeuOrientation(accPointEma, magPointTransEma);
    }

    const unsigned long nowMs = millis();
    if (lastMoonUpdateMs == 0 || nowMs - lastMoonUpdateMs >= MoonUpdatePeriodMs) {
        currentMoonPosition = computeMoonEnu(rtcClock.currentTime());
        lastMoonUpdateMs = nowMs;
    }

    updateTouch();
    handleTouchDrag();

    const unsigned long nowUs = micros();
    const float dt = lastSampleUs == 0 ? 0.0f : (nowUs - lastSampleUs) * 1.0e-6f;
    lastSampleUs = nowUs;
    recordCalibrationSample(dt);

    handleTouchStart();

    if (nowMs - lastUiRefreshMs >= UiRefreshPeriodMs) {
        lastUiRefreshMs = nowMs;
        refreshActiveView();
    }

    lastTouchActive = touchActive;
    lastTouchX = touchActive ? touchX : -1;
    lastTouchY = touchActive ? touchY : -1;
    delay(20);
}
