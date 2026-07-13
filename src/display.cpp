// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#include "display.h"

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);


bool touchActive = false;
int touchX = 0;
int touchY = 0;
int touchZ = 0;

#define FONT_SIZE 4


void initDisplay(int rotation) {
  Serial.begin(115200);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  touchscreen.setRotation(rotation);

  // Start the tft display
  tft.init();
  // Set the TFT display rotation in landscape mode
  tft.setRotation(rotation);

  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  // Set X and Y coordinates for center of display
  int centerY = tft.height() / 2;
  int centerX = tft.width() / 2;

  tft.drawCentreString("Hello, world!", centerX, 30, FONT_SIZE);
  tft.drawCentreString("Touch screen to test", centerX, centerY, FONT_SIZE);

}

void updateTouch() {
  // Checks if Touchscreen was touched, and saves X, Y and Pressure (Z) info 
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();
    // Calibrate Touchscreen points with map function to the correct width and height
    // // TODO what the magic number?
    // y = map(p.y, 0, 4095, 1, tft.width());
    // x = map(p.x, 0, 4095, 1, tft.height());
    touchX = p.x;
    touchY = p.y;
    touchZ = p.z;
    touchActive = true;
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
    Serial.print(") | Pressure = ");
    Serial.print(touchZ);
    Serial.println();
  }
  else {
    Serial.println("idle");
  }
}

// Print Touchscreen info about X, Y and Pressure (Z) on the TFT Display
void printTouchToDisplay() {
  // Clear TFT screen
  // tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  int centerX = SCREEN_HEIGHT / 2;
  int textY = 80;
 
  String tempText = "   X = " + String(touchX) + "   ";
  tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);

  textY += 20;
  tempText = "   Y = " + String(touchY) + "   ";
  tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);

  textY += 20;
  tempText = "   Pressure = " + String(touchZ) + "   ";
  tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);
}

void printStringToDisplay(String str) {
  tft.setCursor(0, 5);
  tft.println(str);
}