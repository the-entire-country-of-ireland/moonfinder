#include "display.h"
#include "isometric.h"

float rotationX = 0.55f;
float rotationY = -0.65f;
float rotationZ = -1.0f;

int lastTouchX = -1;
int lastTouchY = -1;

void setup() {
    Serial.begin(115200);

    initDisplay(2);
    delay(50);
    tft.fillScreen(TFT_WHITE);

    tft.drawCentreString("Drag to rotate", tft.width() / 2, 20, 4);
}

void loop() {
    updateTouch();
    printTouchToSerial();
    
    if (touchActive) {
        if((lastTouchX != -1) && (lastTouchY != -1)) {
            rotationX += (touchX - lastTouchX) * 0.001f;
            rotationY += (touchY - lastTouchY) * 0.001f;
        }
        lastTouchX = touchX;
        lastTouchY = touchY;
    } else{
        lastTouchX = -1;
        lastTouchY = -1;
    }

    drawScene(rotationX, rotationY, rotationZ);
    delay(30);
}