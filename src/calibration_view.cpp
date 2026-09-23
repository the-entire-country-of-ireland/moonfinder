#include "calibration_view.h"

#include <cstdio>

#include "display.h"
#include "ui_common.h"

namespace {
UiRect confirmYesRect() { return UiRect{12, 150, 102, 52}; }
UiRect confirmNoRect() { return UiRect{126, 150, 102, 52}; }
UiRect sectorRect() { return UiRect{12, 146, 216, 50}; }
UiRect optimizeRect() { return UiRect{12, 207, 216, 50}; }
}

void drawCalibrationConfirmView(RTC& clock) {
    tft.fillScreen(TFT_NAVY);
    uiDrawHeader("CALIBRATION", clock, TFT_NAVY);
    uiDrawCenteredText("Start a new calibration?", 72, TFT_WHITE, TFT_NAVY, false);
    uiDrawCenteredText("Move through several poses.", 98, TFT_LIGHTGREY, TFT_NAVY, false);
    uiDrawCenteredText("Record one sector at a time.", 118, TFT_LIGHTGREY, TFT_NAVY, false);
    uiDrawButton(confirmYesRect(), "START", TFT_GREEN);
    uiDrawButton(confirmNoRect(), "CANCEL", TFT_MAROON);
}

CalibrationUiAction calibrationConfirmTouch(int16_t x, int16_t y) {
    if (confirmYesRect().contains(x, y)) return CalibrationUiAction::ConfirmYes;
    if (confirmNoRect().contains(x, y)) return CalibrationUiAction::ConfirmNo;
    return CalibrationUiAction::None;
}

void drawCalibrationView(RTC& clock, bool waitingForSector,
                         int collectedSectors, int currentSamples,
                         bool clearScreen) {
    if (clearScreen) tft.fillScreen(TFT_BLACK);
    uiDrawHeader("CALIBRATION", clock);
    uiDrawSwitchViewButton();

    tft.fillRect(0, UiHeaderHeight, tft.width(), UiFooterTop - UiHeaderHeight, TFT_BLACK);
    uiDrawCenteredText(waitingForSector ? "READY" : "RECORDING", 58,
                       waitingForSector ? TFT_YELLOW : TFT_GREEN,
                       TFT_BLACK, true);

    char line[40];
    std::snprintf(line, sizeof(line), "Completed sectors: %d", collectedSectors);
    uiDrawCenteredText(String(line), 91, TFT_WHITE, TFT_BLACK, false);
    std::snprintf(line, sizeof(line), "Current samples: %d / 500", currentSamples);
    uiDrawCenteredText(String(line), 113, TFT_LIGHTGREY, TFT_BLACK, false);

    uiDrawButton(sectorRect(), waitingForSector ? "START SECTOR" : "FINISH SECTOR",
                 waitingForSector ? TFT_GREEN : TFT_ORANGE,
                 waitingForSector ? TFT_WHITE : TFT_BLACK);
    uiDrawButton(optimizeRect(), "OPTIMIZE + SAVE", TFT_BLUE);
}

CalibrationUiAction calibrationViewTouch(int16_t x, int16_t y) {
    if (uiSwitchViewHit(x, y)) return CalibrationUiAction::SwitchView;
    if (sectorRect().contains(x, y)) return CalibrationUiAction::SectorToggle;
    if (optimizeRect().contains(x, y)) return CalibrationUiAction::OptimizeSave;
    return CalibrationUiAction::None;
}


void drawCalibrationWorkingView(RTC& clock, const char* message) {
    tft.fillScreen(TFT_ORANGE);
    uiDrawHeader("CALIBRATION", clock, TFT_ORANGE);
    uiDrawCenteredText(String(message), 142, TFT_BLACK, TFT_ORANGE, true);
}
