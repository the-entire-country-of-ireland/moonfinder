// #include <Wire.h>
// #include <Adafruit_ICM20948.h>
// #include <Adafruit_Sensor.h>

// #include "icm20948.h"
// #include "sd_card.h"

// SDCard sdCard;

// using Eigen::Vector3d;
// using Eigen::Vector2d;

// #define maxRecords 250
// int currentRecord = 0;
// bool recordsFilled = false;
// Eigen::Matrix<double, maxRecords, 3> magHistory;
// Eigen::Matrix<double, maxRecords, 3> accHistory;
// Eigen::Matrix<double, maxRecords, 3> gyroHistory;
// int timeHistory[maxRecords];
// long int lasttime, currenttime, deltatime;
// bool historySaved = false;
// bool saveAttempted = false;
// unsigned long lastSaveAttemptTime = 0;
// bool lastTouchActive = false;
// bool tapDetected = false;

// bool saveHistory() {
//     if (!sdCard.isAvailable()) {
//         Serial.println("ERROR: SD card unavailable, cannot save history");
//         return false;
//     }

//     String filename = sdCard.getCurrentFilename();
//     if (filename.isEmpty()) {
//         Serial.println("ERROR: No current SD filename available");
//         return false;
//     }

//     fs::File file = SD.open(filename, FILE_APPEND);
//     if (!file) {
//         Serial.println("ERROR: Failed to open file for bulk history save: " + filename);
//         return false;
//     }

//     file.println("NEW_MEASUREMENTS");
//     char lineBuffer[256];
//     for (int j = 0; j < currentRecord; j++) {
//         int len = snprintf(
//             lineBuffer, sizeof(lineBuffer),
//             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
//             magHistory(j, 0), magHistory(j, 1), magHistory(j, 2),
//             accHistory(j, 0), accHistory(j, 1), accHistory(j, 2),
//             gyroHistory(j, 0), gyroHistory(j, 1), gyroHistory(j, 2),
//             timeHistory[j]);

//         if (len > 0) {
//             file.println(lineBuffer);
//         }

//         if ((j & 0x1F) == 0) {
//             delay(0);
//         }
//     }

//     file.close();
//     return true;
// }

// void resetHistory() {
//     currentRecord = 0;
//     recordsFilled = false;
//     historySaved = false;
//     tft.fillScreen(TFT_GREEN);
// }

// void appendHistory() {
//     if (recordsFilled) return;

//     if (currentRecord >= maxRecords) {
//         recordsFilled = true;
//         historySaved = false;
//         tft.fillScreen(TFT_RED);
//         return;
//     }

//     for (int i = 0; i < 3; i++) {
//         magHistory(currentRecord, i) = magPoint(i);
//         accHistory(currentRecord, i) = accPoint(i);
//         gyroHistory(currentRecord, i) = gyroPoint(i);
//     }
//     timeHistory[currentRecord] = deltatime;

//     currentRecord++;

//     if (currentRecord >= maxRecords) {
//         recordsFilled = true;
//         historySaved = false;
//         tft.fillScreen(TFT_RED);
//     }
// }


// void setupSDCard(SDCard& sdCard) {
//     // Initialize SD card
//     if (!sdCard.init()) {
//         Serial.println("Failed to initialize SD card!");
//         return;
//     }
    
//     String filename = sdCard.createFile("readings", "/data/magaccgyro");
    
//     if (filename.isEmpty()) {
//         Serial.println("Failed to create file!");
//         return;
//     }
    
//     Serial.println("Writing data to: " + filename);
//     sdCard.printDirectory("/data", 3);

// }



// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10);

//   Serial.println("ICM20948 Test");
    
//   // Scan I2C bus
//   scanI2C();
//   initSensors();
//   setupSDCard(sdCard);

//   initDisplay(0);
//   tft.fillScreen(TFT_GREEN);
  
//   delay(100);
//   lasttime = millis();
// }

// bool serialPrintf(const char* format, ...) {
//   char buffer[256];
//   va_list args;
//   va_start(args, format);
//   vsnprintf(buffer, sizeof(buffer), format, args);
//   va_end(args);
  
//   Serial.println(buffer);
  
//   return true;
// }

// void updateTime() {
//     currenttime = millis();
//     deltatime = currenttime - lasttime;
//     lasttime = currenttime;
// }

// void loop() {
//     delay(15);

//     updateSensors(); // Read ICM-20948
//     updateTime();

//     updateTouch();
//     tapDetected = !lastTouchActive && touchActive;
//     lastTouchActive = touchActive;

//     if (recordsFilled && tapDetected) {
//         resetHistory();
//         delay(0);
//         return;
//     }

//     if (!recordsFilled) {
//         appendHistory();

//         serialPrintf(
//             "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
//             magPoint[0], magPoint[1], magPoint[2],
//             accPoint[0], accPoint[1], accPoint[2],
//             gyroPoint[0], gyroPoint[1], gyroPoint[2],
//             deltatime);
//     } else {
//         unsigned long now = millis();
//         if (!historySaved && !saveAttempted) {
//             historySaved = saveHistory();
//             saveAttempted = true;
//             lastSaveAttemptTime = now;
//         }
//         if (saveAttempted && !historySaved && now - lastSaveAttemptTime > 1000) {
//             saveAttempted = false;
//         }
//     }

//     delay(0);
// }


