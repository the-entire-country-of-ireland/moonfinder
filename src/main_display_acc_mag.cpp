#include <ArduinoEigen.h>
#include <fstream>
#include <iostream>

#include "display.h"
#include "isometric.h"
#include "lms.h"
#include "sd_card.h"
#include "online_calibration.h"
#include "AHRS.h"

using Eigen::Vector3d;
using Eigen::Vector2d;

int lastTouchX = -1;
int lastTouchY = -1;

#define maxRecords 500
int currentRecord = 0;
bool recordsFilled = false;
Eigen::Matrix<double, maxRecords, 3> magHistory;
Eigen::Matrix<double, maxRecords, 3> accHistory;

SDCard sdCard;  // Uses default VSPI pins

void setupSDCard() {
    // Initialize SD card
    if (!sdCard.init()) {
        Serial.println("Failed to initialize SD card!");
        return;
    }
    
    String filename = sdCard.createFile("magacc_readings", "/data");
    
    if (filename.isEmpty()) {
        Serial.println("Failed to create file!");
        return;
    }
    
    Serial.println("Writing data to: " + filename);
    sdCard.printDirectory("/data", 3);

}

void printMatrix3d(const Eigen::Matrix3d& mat) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(mat(i, j), 6);   // 6 decimal places
      Serial.print("\t");
    }
    Serial.println();
  }
}

void printVector3d(const Eigen::Vector3d& vec) {
  for (int i = 0; i < 3; i++) {
    Serial.print(vec(i), 6);
    Serial.println();
  }
}

void loadCalibration() {
    Eigen::Matrix3d M;
    Eigen::Vector3d c;

    if(sdCard.isAvailable()) {
        sdCard.printDirectory("/data/", 1);
        sdCard.printFileLines(     "/data/calibration.txt");
        sdCard.loadCalibrationData("/data/calibration.txt", M, c);
    }
    else{
        // default values, if no sd card is inserted
        M << 0.02191344348162641, -0.000573975309675504, -0.0008785351217481529,
0.0007582154619115889, 0.020103931916349484, -0.0013875491058414167,
0.0003737297742823399, -0.0015972984399375007, 0.0165140583871701;
        c << -0.24466064581052546, 0.2237080518646763, -0.5992611215218402;
    }
    calibration.load(M, c);
    Serial.println("loaded calibration data!");
    Serial.print("Matrix: ");
    printMatrix3d(M);
    Serial.print("constant: ");
    printVector3d(c);
    
}

void setup() {
    Serial.begin(115200);

    initDisplay(0);
    initSensors();
    delay(50);

    tft.fillScreen(TFT_BLACK);
    // Initialize rotation with default angles
    initRotation(0.0f, -0.6f, 0.3f);
    drawScene();

    setupSDCard();
    loadCalibration();

    magHistory.setZero();
    accHistory.setZero();
    magPoint.setZero();
    accPoint.setZero();

}

void appendHistory() {
    // add the raw values
    // calibration.addSample(magPoint, accPoint);
    for(int i = 0; i < 3; i++) {
        magHistory(currentRecord, i) = magPoint(i);
        accHistory(currentRecord, i) = accPoint(i);
    }

    // calibration.addSample(magPoint, accPoint);

    if(sdCard.isAvailable()) {
        sdCard.appendLinef("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", 
            magPoint[0], magPoint[1], magPoint[2],
            accPoint[0], accPoint[1], accPoint[2]);
    }

    // cyclic buffer
    currentRecord++;
    if (currentRecord >= maxRecords) {
        currentRecord = 0; 
        recordsFilled = true;
    }
}

void renderPoint(const Vector3d& point, uint16_t color=TFT_WHITE, float scale=1.0f) {
    const Vector2d projectedPoint = projectPoint(point, 
        currentRotation,
        scale, tft.width() / 2, tft.height() / 2);

    drawPoint(projectedPoint, color);
}

void renderAllPoints() {
    int recordsToRender = recordsFilled ? maxRecords : currentRecord;

    for (int i = 0; i < recordsToRender; ++i) {
        Vector3d magPointTmp = magHistory.row(i);
        Vector3d accPointTmp = accHistory.row(i);
        
        magPointTmp = calibration.transform(magPointTmp);
        // magPointTmp = magPointTmp.normalized();
        accPointTmp = accPointTmp.normalized();
        
        renderPoint(magPointTmp, TFT_PINK, 50.0f);
        renderPoint(accPointTmp, TFT_LIGHTBLUE, 50.0f);
    }
}

void loop() {
    updateTouch();

    updateSensors();
    appendHistory();

    if (touchActive) {
        if((lastTouchX != -1) && (lastTouchY != -1)) {
            float deltaX = touchX - lastTouchX;
            float deltaY = touchY - lastTouchY;
            
            updateRotationFromTouch(deltaX, deltaY);
        }
        lastTouchX = touchX;
        lastTouchY = touchY;
        
        // calibration.solve();

        // only redraw the entire scene when the touch is active to reduce flickering
        tft.fillScreen(TFT_BLACK);
        drawScene();
        renderAllPoints();
    } else {
        lastTouchX = -1;
        lastTouchY = -1;
    }

    // printSensorsToSerial();
    printSensorsToDisplay(true);
        
    renderPoint(magPointTrans, TFT_RED, 50.0f);
    renderPoint(accPointTrans, TFT_BLUE, 50.0f);

    tft.setFont(&fonts::FreeSans9pt7b); tft.setTextSize(1);
    tft.setCursor(5, 250);
    tft.printf("Norm: %2.2f     %2.2f         ", magPointTrans.dot(magPointTrans), accPoint.dot(accPoint) / (9.8 * 9.8));
    tft.setCursor(5, 275);
    tft.printf("Overlap: %2.2f      %2.2f        ", magPointTrans.dot(accPointTrans), magPointTrans.normalized().dot(accPointTrans));

    
    // Example sensor readings (device frame)
    Vector3d gravity_enu = Vector3d(0.0, 0.0, -1.0);
    Vector3d magnetic_north_enu = Vector3d(-0.08036317018458757, 0.4135656507192516, 0.9069207316094637);
    Orientation orient = computeOrientation(
        accPointTrans, magPointTrans, gravity_enu, magnetic_north_enu);

    // Serial.println("Device x-axis in ENU:");
    // printVector3d(orient.x_enu);
    // Serial.println("Device y-axis in ENU:");
    // printVector3d(orient.y_enu);
    // Serial.println("Device z-axis in ENU:");
    // printVector3d(orient.z_enu);

    Serial.println("Device axis in ENU:");
    printVector3d(orient.y_enu);
    printVectorToDisplay("ENU: ", orient.y_enu, 45);
    
    delay(30);
}