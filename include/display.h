// ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD)*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include "LGFX_ESP32_2432S028R_CYD.hpp"
#include <ArduinoEigen.h>

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

extern LGFX tft;
extern LGFX_Sprite sprite;


// Touchscreen coordinates: (x, y) and pressure (z)
extern int16_t touchX, touchY;
extern bool touchActive;


void initDisplay(int rotation=0);
void updateTouch();
void printTouchToSerial();
void printTouchToDisplay();
void printStringToDisplay(String str);

template <typename Derived>
void printEigen(const Eigen::MatrixBase<Derived>& m, uint8_t digits = 2)
{
    for (int i = 0; i < m.rows(); ++i) {
        for (int j = 0; j < m.cols(); ++j) {
            Serial.print(m(i, j), digits);
            if (j < m.cols() - 1) {
                Serial.print('\t');
            }
        }
        Serial.println();
    }
}

template <typename Derived>
String eigenToString(const Eigen::MatrixBase<Derived>& m, uint8_t digits = 2)
{
    String s;

    const int rows = m.rows();
    const int cols = m.cols();

    // First pass: determine max string length for each column
    int* colWidths = new int[cols];
    for (int j = 0; j < cols; ++j) {
        colWidths[j] = 0;
        for (int i = 0; i < rows; ++i) {
            String val = String(m(i, j), digits);
            if (val.length() > colWidths[j]) {
                colWidths[j] = val.length();
            }
        }
    }

    // Second pass: build the aligned string
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            String val = String(m(i, j), digits);

            // Left-pad with spaces to match column width
            for (int k = 0; k < colWidths[j] - val.length(); ++k) {
                s += ' ';
            }

            s += val;

            if (j < cols - 1) {
                s += '\t';
            }
        }

        if (i < rows - 1) {
            s += '\n';
        }
    }

    delete[] colWidths;
    return s;
}

#endif // DISPLAY_H