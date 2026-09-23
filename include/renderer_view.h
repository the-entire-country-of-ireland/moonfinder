#pragma once

#include "rtc.h"

void drawRendererView(RTC& clock, bool redrawScene = false, bool resetHistory = false);
void rendererDrag(int deltaX, int deltaY);
