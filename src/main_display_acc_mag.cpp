// #include <ArduinoEigen.h>
// #include <fstream>
// #include <iostream>

// #include "display.h"
// #include "isometric.h"
// #include "imu_backend.h"
// #include "sd_card.h"
// #include "online_calibration.h"
// #include "AHRS.h"
// #include "sector_calibrator.h"

// using Eigen::Vector3d;
// using Eigen::Vector2d;

// int lastTouchX = -1;
// int lastTouchY = -1;

// #define maxRecords 1000
// int currentRecord = 0;
// bool recordsFilled = false;
// Eigen::Matrix<double, maxRecords, 3> magHistory;
// Eigen::Matrix<double, maxRecords, 3> accHistory;

// SDCard sdCard;  // Uses default VSPI pins

// void setupSDCard() {
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

// void printMatrix3d(const Eigen::Matrix3d& mat) {
//   for (int i = 0; i < 3; i++) {
//     for (int j = 0; j < 3; j++) {
//       Serial.print(mat(i, j), 6);   // 6 decimal places
//       Serial.print("\t");
//     }
//     Serial.println();
//   }
// }

// void printVector3d(const Eigen::Vector3d& vec) {
//   for (int i = 0; i < 3; i++) {
//     Serial.print(vec(i), 6);
//     Serial.println();
//   }
// }

// void loadCalibration() {
//     Eigen::Matrix3d M;
//     Eigen::Vector3d c;

//     if(sdCard.isAvailable()) {
//         sdCard.printDirectory("/data/", 1);
//         sdCard.printFileLines(     "/data/calibration.txt");
//         sdCard.loadCalibrationData("/data/calibration.txt", M, c);
//     }
//     else{
//         // default values, if no sd card is inserted
//         M << 2.217, 0.017, 0.103, 0.132, 2.07, 0.09, -0.161, -0.037, 1.862;
//         M = M / 100.0;
//         c << -0.31, -0.06, -0.499;
//     }
//     calibration.load(M, c);
//     Serial.println("loaded calibration data!");
//     Serial.print("Matrix: ");
//     printMatrix3d(M);
//     Serial.print("constant: ");
//     printVector3d(c);
    
// }

// void setup() {
//     Serial.begin(115200);

//     initDisplay(0);
//     initSensors();
//     delay(50);

//     tft.fillScreen(TFT_BLACK);
//     // Initialize rotation with default angles
//     initRotation(0.0f, -0.6f, 0.3f);
//     drawScene();

//     setupSDCard();
//     // loadCalibration();

//     magHistory.setZero();
//     accHistory.setZero();
//     magPoint.setZero();
//     accPoint.setZero();

// }

// void appendHistory() {
//     // add the raw values
//     // calibration.addSample(magPoint, accPoint);
//     for(int i = 0; i < 3; i++) {
//         magHistory(currentRecord, i) = magPoint(i);
//         accHistory(currentRecord, i) = accPoint(i);
//     }

//     // calibration.addSample(magPoint, accPoint);

//     if(sdCard.isAvailable()) {
//         sdCard.appendLinef("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", 
//             magPoint[0], magPoint[1], magPoint[2],
//             accPoint[0], accPoint[1], accPoint[2],
//             gyroPoint[0], gyroPoint[1], gyroPoint[2]);
//     }

//     // cyclic buffer
//     currentRecord++;
//     if (currentRecord >= maxRecords) {
//         currentRecord = 0; 
//         recordsFilled = true;
//     }
// }

// void renderPoint(const Vector3d& point, uint16_t color=TFT_WHITE, float scale=1.0f) {
//     const Vector2d projectedPoint = projectPoint(point, 
//         currentRotation,
//         scale, tft.width() / 2, tft.height() / 2);

//     drawPoint(projectedPoint, color);
// }

// void renderAllPoints() {
//     int recordsToRender = recordsFilled ? maxRecords : currentRecord;

//     for (int i = 0; i < recordsToRender; ++i) {
//         Vector3d magPointTmp = magHistory.row(i);
//         Vector3d accPointTmp = accHistory.row(i);
        
//         magPointTmp = calibration.transform(magPointTmp);
//         // magPointTmp = magPointTmp.normalized();
//         accPointTmp = accPointTmp.normalized();
        
//         renderPoint(magPointTmp, TFT_PINK, 50.0f);
//         renderPoint(accPointTmp, TFT_LIGHTBLUE, 50.0f);
//     }
// }

// void loop() {
//     updateTouch();

//     updateSensors();
//     appendHistory();

//     if (touchActive) {
//         if((lastTouchX != -1) && (lastTouchY != -1)) {
//             float deltaX = touchX - lastTouchX;
//             float deltaY = touchY - lastTouchY;
            
//             updateRotationFromTouch(deltaX, deltaY);
//         }
//         lastTouchX = touchX;
//         lastTouchY = touchY;
        
//         // calibration.solve();

//         // only redraw the entire scene when the touch is active to reduce flickering
//         tft.fillScreen(TFT_BLACK);
//         drawScene();
//         renderAllPoints();
//     } else {
//         lastTouchX = -1;
//         lastTouchY = -1;
//     }

//     // printSensorsToSerial();
//     printSensorsToDisplay(true);
        
//     renderPoint(magPointTrans, TFT_RED, 50.0f);
//     renderPoint(accPointTrans, TFT_BLUE, 50.0f);

//     tft.setFont(&fonts::FreeSans9pt7b); tft.setTextSize(1);
//     tft.setCursor(5, 250);
//     tft.printf("Norm: %2.2f     %2.2f         ", magPointTrans.dot(magPointTrans), accPoint.dot(accPoint) / (9.8 * 9.8));
//     tft.setCursor(5, 275);
//     tft.printf("Overlap: %2.2f      %2.2f        ", magPointTrans.dot(accPointTrans), magPointTrans.normalized().dot(accPointTrans));

    
//     delay(30);
// }