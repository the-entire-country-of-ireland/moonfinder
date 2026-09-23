#include "view_selector.h"

#include "display.h"
#include "ui_common.h"

namespace {
UiRect compassRect() { return UiRect{12, 82, 102, 62}; }
UiRect moonRect() { return UiRect{126, 82, 102, 62}; }
UiRect renderRect() { return UiRect{12, 157, 102, 62}; }
UiRect calibrateRect() { return UiRect{126, 157, 102, 62}; }
UiRect backRect() { return UiRect{12, 239, 216, 40}; }
}

void drawViewSelector(RTC& clock) {
    tft.fillScreen(TFT_NAVY);
    uiDrawHeader("SWITCH VIEW", clock, TFT_NAVY);
    uiDrawCenteredText("Choose a view", 57, TFT_LIGHTGREY, TFT_NAVY, false);
    uiDrawButton(compassRect(), "COMPASS", TFT_DARKCYAN);
    uiDrawButton(moonRect(), "MOON", TFT_DARKCYAN);
    uiDrawButton(renderRect(), "3D SENSOR", TFT_DARKCYAN);
    uiDrawButton(calibrateRect(), "CALIBRATE", TFT_DARKCYAN);
    uiDrawButton(backRect(), "BACK", TFT_DARKGREY);
}

ViewSelectorChoice viewSelectorTouch(int16_t x, int16_t y) {
    if (compassRect().contains(x, y)) return ViewSelectorChoice::Compass;
    if (moonRect().contains(x, y)) return ViewSelectorChoice::Moonfinder;
    if (renderRect().contains(x, y)) return ViewSelectorChoice::Renderer3d;
    if (calibrateRect().contains(x, y)) return ViewSelectorChoice::Calibration;
    if (backRect().contains(x, y)) return ViewSelectorChoice::Back;
    return ViewSelectorChoice::None;
}
