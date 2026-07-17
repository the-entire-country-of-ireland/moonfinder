// #include "LGFX_ESP32_2432S028R_CYD.hpp"

// static LGFX tft;

// #define TFT_ROTATION  2   // 0, 2: Portrait / 1, 3: Landscape


// static void tft_init(void) {
//   tft.init();
//   tft.initDMA();
//   tft.setColorDepth(16);                  // Set to 16-bit (RGB565)
//   tft.setRotation(TFT_ROTATION);          // Set panel rotation

//   if (!tft.touch()) {
//     Serial.println("Touch device not found.");
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   while (millis() < 1000);

//   tft_init();
// }

// void loop() {
//   uint16_t x, y;
//   if (tft.getTouch(&x, &y)) {
//     Serial.println("x: " + String(x) + ", y: " + String(y));

//     // Draw a white spot at the detected coordinates
//     tft.fillCircle(x, y, 2, TFT_WHITE);
//   }
  
// }