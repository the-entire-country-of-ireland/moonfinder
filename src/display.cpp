// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#include "display.h"
#include <ArduinoEigen.h>

LGFX tft;
LGFX_Sprite sprite(&tft);

bool touchActive = false;
int16_t touchX = 0;
int16_t touchY = 0;

#define FONT_SIZE 4


void initDisplay(int rotation) {
  Serial.begin(115200);

  tft.init();
  tft.initDMA();
  tft.setRotation(rotation);
  tft.setBrightness(128);  // 0 ~ 255

  // Set the color mode as needed (default is 16).
  // 16 requires less SPI communication and operates faster, but the red and blue gradations are 5 bits.
  // 24 requires more SPI communication, but produces clearer tonal expression.
  tft.setColorDepth(16);  // Set to 16-bit RGB565
//tft.setColorDepth(24);  // Set to 24-bit RGB888 (the number of colors displayed will be 18-bit RGB666 depending on the panel)

  if (!tft.touch()) {
    Serial.println("Touch device not found.");
  }

  tft.clear(0xFFFFFFu);         // Fill the background with white
  tft.setBaseColor(0x000000u);  // Specify black as the background color
  tft.clear();                  // Fill with background color

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // tft.setTextDatum(textdatum_t::baseline_center);
  // tft.setFont(&fonts::FreeSans12pt7b);
  // tft.drawString("Touch screen to test", tft.width() / 2, tft.height() / 2);
}

void updateTouch() {
  uint16_t x, y;
  if (tft.getTouch(&x, &y)) {
    touchX = x;
    touchY = y;
    touchActive = true;
    // tft.fillScreen(TFT_BLACK);
  }
  else {
    touchActive = false;
  }
}

// Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
void printTouchToSerial() {
  Serial.print("Touch: ");
  if(touchActive) {
    Serial.print("(");
    Serial.print(touchX);
    Serial.print(", ");
    Serial.print(touchY);
    Serial.println(")");
  }
  else {
    Serial.println("idle");
  }
}

// Print touchscreen coordinates without exceeding the portrait CYD width.
void printTouchToDisplay() {
  tft.setTextWrap(false);
  tft.setTextDatum(textdatum_t::top_center);
  tft.setTextSize(1);
  tft.setFont(&fonts::FreeSans12pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  char line[32];
  snprintf(line, sizeof(line), "Touch X: %d", touchX);
  tft.drawString(line, tft.width() / 2, 120);
  snprintf(line, sizeof(line), "Touch Y: %d", touchY);
  tft.drawString(line, tft.width() / 2, 155);
  tft.setTextDatum(textdatum_t::top_left);
}

void printStringToDisplay(String str) {
  tft.setTextWrap(false);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(4, 5);
  tft.print(str);
}

void printVectorToDisplay(const char * str, const Eigen::Vector3d vec, int height) {
  tft.setTextWrap(false);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setFont(&fonts::Font0);
  tft.setTextSize(1);
  tft.setCursor(4, height);
  tft.printf("%s %.2f, %.2f, %.2f", str, vec[0], vec[1], vec[2]);
}