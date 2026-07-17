// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include "LGFX_ESP32_2432S028R_CYD.hpp"


#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

extern LGFX tft;
extern LGFX_Sprite sprite;


// Touchscreen coordinates: (x, y) and pressure (z)
extern int16_t touchX, touchY;
extern bool touchActive;


void initDisplay(int rotation=0);
void updateTouch();
void printTouchToSerial();
void printTouchToDisplay();
void printStringToDisplay(String str);

#endif // DISPLAY_H