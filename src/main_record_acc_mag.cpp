// #include "display.h"
// #include "isometric.h"
// #include "lms.h"
// #include "online_calibration.h"
// #include "sd_card.h"
// #include <ArduinoEigen.h>

// using Eigen::Vector3d;
// using Eigen::Vector2d;



// int lastTouchX = -1;
// int lastTouchY = -1;

// #define maxRecords 300
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
    
//     String filename = sdCard.createFile("magacc_readings", "/data");
    
//     if (filename.isEmpty()) {
//         Serial.println("Failed to create file!");
//         return;
//     }
    
//     Serial.println("Writing data to: " + filename);
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

//     magHistory.setZero();
//     accHistory.setZero();
//     magPoint.setZero();
//     accPoint.setZero();

//     Eigen::Matrix3d M;
//     Eigen::Vector3d c;
//     M << 0.017271, -0.0003308557, -0.00058555,
//          0.000158786, 0.018554, 0.0007986444,
//         -0.0001457, -0.0018629, 0.01879666;
//     c << -0.055873, -0.08966, -0.797;
//     calibration.load(M, c);
// }

// void appendHistory() {
//     // add the raw values
//     calibration.addSample(magPoint, accPoint);
//     for(int i = 0; i < 3; i++) {
//         magHistory(currentRecord, i) = magPoint(i);
//         accHistory(currentRecord, i) = accPoint(i);
//     }

//     if(sdCard.isAvailable()) {
//         sdCard.appendLinef("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", 
//             magPoint[0], magPoint[1], magPoint[2],
//             accPoint[0], accPoint[1], accPoint[2]);
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
        
//         calibration.solve();

//         // only redraw the entire scene when the touch is active to reduce flickering
//         tft.fillScreen(TFT_BLACK);
//         drawScene();
//         renderAllPoints();
//     } else {
//         lastTouchX = -1;
//         lastTouchY = -1;
//     }

    

//     printSensorsToSerial();
//     printSensorsToDisplay(true);

//     renderPoint(magPointTrans, TFT_RED, 50.0f);
//     renderPoint(accPointTrans, TFT_BLUE, 50.0f);
    
//     delay(30);
// }