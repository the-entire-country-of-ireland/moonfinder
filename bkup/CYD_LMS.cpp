// // ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

// #include <Wire.h>
// #include <SPI.h>

// #include <Adafruit_LSM303_Accel.h>
// #include <Adafruit_LIS2MDL.h>
// #include <Adafruit_Sensor.h>
// #include "AHRS.h"
// #define SDA 22
// #define SCL 27

// Vector3f accelReading;
// Vector3f magReading;

// Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
// Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);

// #include <Adafruit_GFX.h>
// #include <Adafruit_ILI9341.h>
// #include <TFT_eSPI.h>
// #include <XPT2046_Touchscreen.h>

// TFT_eSPI tft = TFT_eSPI();

// // Touchscreen pins
// #define XPT2046_IRQ 36   // T_IRQ
// #define XPT2046_MOSI 32  // T_DIN
// #define XPT2046_MISO 39  // T_OUT
// #define XPT2046_CLK 25   // T_CLK
// #define XPT2046_CS 33    // T_CS

// SPIClass touchscreenSPI = SPIClass(VSPI);
// XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// #define SCREEN_WIDTH 320
// #define SCREEN_HEIGHT 240
// #define FONT_SIZE 4


// // Touchscreen coordinates: (x, y) and pressure (z)
// int x, y, z;
// bool touchActive;

// // Print Touchscreen info about X, Y and Pressure (Z) on the Serial Monitor
// void printTouchToSerial() {
//   Serial.print("Touch: ");
//   if(touchActive) {
//     Serial.print("(");
//     Serial.print(x);
//     Serial.print(", ");
//     Serial.print(y);
//     Serial.print(") | Pressure = ");
//     Serial.print(z);
//     Serial.println();
//   }
//   else {
//     Serial.println("idle");
//   }
// }

// // Print Touchscreen info about X, Y and Pressure (Z) on the TFT Display
// void printTouchToDisplay() {
//   // Clear TFT screen
//   // tft.fillScreen(TFT_WHITE);
//   tft.setTextColor(TFT_BLACK, TFT_WHITE);

//   int centerX = SCREEN_HEIGHT / 2;
//   int textY = 80;
 
//   String tempText = "   X = " + String(x) + "   ";
//   tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);

//   textY += 20;
//   tempText = "   Y = " + String(y) + "   ";
//   tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);

//   textY += 20;
//   tempText = "   Pressure = " + String(z) + "   ";
//   tft.drawCentreString(tempText, centerX, textY, FONT_SIZE);
// }

// void initSensors() {

//   if (accel.begin()) {
//     accel.setRange(LSM303_RANGE_4G);
//     accel.setMode(LSM303_MODE_NORMAL);
//     Serial.println("Accel ready");
//   } else {
//     Serial.println("Accel sensor not found");
//     while(1);
//   }

//   if (lis2mdl.begin()) {
//     lis2mdl.enableAutoRange(true);
//     Serial.println("Mag ready");
//   } else {
//     Serial.println("Mag sensor not found");
//     while(1);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   Wire.begin(SDA, SCL);

//   // Start the SPI for the touchscreen and init the touchscreen
//   touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
//   touchscreen.begin(touchscreenSPI);
//   // Set the Touchscreen rotation in landscape mode
//   // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
//   touchscreen.setRotation(1);

//   // Start the tft display
//   tft.init();
//   // Set the TFT display rotation in landscape mode
//   tft.setRotation(1);

//   // Clear the screen before writing to it
//   tft.fillScreen(TFT_WHITE);
//   tft.setTextColor(TFT_BLACK, TFT_WHITE);
  
//   // Set X and Y coordinates for center of display
//   int centerY = tft.width() / 2;
//   int centerX = tft.height() / 2;

//   tft.drawCentreString("Hello, world!", centerX, 30, FONT_SIZE);
//   tft.drawCentreString("Touch screen to test", centerX, centerY, FONT_SIZE);

//   initSensors();
// }

// void updateSensors() {
//   sensors_event_t accelEvent;
//   sensors_event_t magEvent;

//   accel.getEvent(&accelEvent);
//   accelReading = Vector3f(accelEvent.acceleration.x, accelEvent.acceleration.y, accelEvent.acceleration.z);

//   lis2mdl.getEvent(&magEvent);
//   magReading = Vector3f(magEvent.magnetic.x, magEvent.magnetic.y, magEvent.magnetic.z);

// }

// void printSensorsToSerial() {
//   Serial.print("Accel: ");
//   accelReading.println(2);
//   Serial.print("Mag: ");
//   magReading.println(2);

// }

// void printSensorsToDisplay() {
//   tft.setCursor(0, 0);

//   tft.print("Acc: ");
//   tft.println(accelReading.toString(2) + "      ");

//   tft.println();
//   tft.print("Mag: ");
//   tft.println(magReading.toString(2) + "      ");

// }


// void updateTouch() {
//   // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z) info on the TFT display and Serial Monitor
//   if (touchscreen.tirqTouched() && touchscreen.touched()) {
//     // Get Touchscreen points
//     TS_Point p = touchscreen.getPoint();
//     // Calibrate Touchscreen points with map function to the correct width and height
//     y = map(p.y, 200, 3700, 1, SCREEN_HEIGHT);
//     x = map(p.x, 240, 3800, 1, SCREEN_WIDTH);
//     z = p.z;
//     touchActive = true;
//   }
//   else {
//     touchActive = false;
//   }
// }

// void loop() {
//   updateTouch();
//   updateSensors();
  
  
//   printSensorsToSerial();
//   printSensorsToDisplay();
//   printTouchToSerial();
//   printTouchToDisplay();
  
//   delay(50);
//   // tft.fillScreen(TFT_WHITE);
// }