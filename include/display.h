// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

extern TFT_eSPI tft;

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

extern SPIClass touchscreenSPI;
extern XPT2046_Touchscreen touchscreen;


// Touchscreen coordinates: (x, y) and pressure (z)
extern int touchX, touchY, touchZ;
extern bool touchActive;

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

void initDisplay(int rotation=0);
void updateTouch();
void printTouchToSerial();
void printTouchToDisplay();
void printStringToDisplay(String str);

#endif // DISPLAY_H