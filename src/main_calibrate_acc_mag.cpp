#include <ArduinoEigen.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cmath>

#include "display.h"
#include "isometric.h"
#include "icm20948.h"
#include "sd_card.h"
#include "online_calibration.h"
#include "AHRS.h"
#include "sector_calibrator.h"

using Eigen::Vector3d;
using Eigen::Vector2d;

int lastTouchX = -1;
int lastTouchY = -1;
bool lastTouchActive = false;

SDCard sdCard;  // Uses default VSPI pins
String foldername;

sector_calib::SectorCalibrator<double> calib;

void startNewSector();
void setupSDCard();

void setupSDCard() {
    // Initialize SD card
    if (!sdCard.init()) {
        Serial.println("Failed to initialize SD card!");
        return;
    }
    
    foldername = sdCard.createFolder("readings", "/data/magacc_pitch");
    
    sdCard.printDirectory("/data/magacc_pitch", 2);

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
        M << 2.217, 0.017, 0.103, 0.132, 2.07, 0.09, -0.161, -0.037, 1.862;
        M = M / 100.0;
        c << -0.31, -0.06, -0.499;
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

    setupSDCard();

    magPoint.setZero();
    accPoint.setZero();

    tft.fillScreen(TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
}

void startNewSector() {
    String filename = sdCard.createFile("sector", foldername);
    
    if (filename.isEmpty()) {
        Serial.println("Failed to create file!");
        return;
    }
    
    Serial.println("Writing data to: " + filename);

}

void loop() {
    updateSensors();
    // printSensorsToSerial(false);
    updateTouch();

    if(calib.am_in_sector() && !calib.is_sector_full()) {
        calib.add_sample(magPoint, accPoint);
    }

    if ((touchActive && !lastTouchActive && !calib.am_in_sector())) {
        tft.fillScreen(TFT_GREEN);
        tft.setTextColor(TFT_BLACK, TFT_GREEN);
        startNewSector();
        calib.start_new_sector();
    }

    else if(calib.is_sector_full() || (touchActive && !lastTouchActive && calib.am_in_sector())) {
        tft.fillScreen(TFT_ORANGE);
        calib.writeSectorToFile(sdCard);
        calib.end_sector();
        tft.fillScreen(TFT_YELLOW);
        tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    }

    int sectorSize = calib.sector_current_size();
    tft.setFont(&fonts::FreeSans18pt7b); tft.setTextSize(1);
    tft.setCursor(20, 70);
    tft.printf("Samples: %d      ", sectorSize);

    lastTouchActive = touchActive;
    delay(40);
}