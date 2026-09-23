#pragma once

#include "rtc.h"

void drawRendererView(RTC& clock,
                      bool redrawScene = false,
                      bool resetHistory = false,
                      bool clearScreen = false);
void rendererDrag(int deltaX, int deltaY);
