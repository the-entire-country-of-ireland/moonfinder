#include "display.h"
#include "isometric.h"
#include "lms.h"
#include "online_calibration.h"

float rotationX = 0.55f;
float rotationY = -0.65f;
float rotationZ = -1.0f;

int lastTouchX = -1;
int lastTouchY = -1;

Eigen::Matrix<double,10,10> S;

void setup() {
    Serial.begin(115200);

    initDisplay(2);
    initSensors();
    delay(50);

    tft.fillScreen(TFT_BLACK);
    tft.invertDisplay(1);
    tft.drawCentreString("Drag to rotate", tft.width() / 2, 25, 4);
    drawScene(rotationX, rotationY, rotationZ);

    S = createBlankScatterMatrix();
}

#define maxRecords 2000
int currentRecord = 0;
bool recordsFilled = false;
Vec3 magHistory[maxRecords];

void appendMagHistory() {
    auto magReading = getMagReading();
    const Vec3 point{magReading.x, magReading.y, magReading.z};
    magHistory[currentRecord] = point;

    currentRecord++;
    if (currentRecord >= maxRecords) {
        currentRecord = 0;
        recordsFilled = true;
    }
}

void renderPoint(Vec3 point, uint16_t color=TFT_WHITE) {
    const Vec2 projectedPoint = projectPoint(point, rotationX, rotationY, rotationZ, 
        1.0f, tft.width() / 2, tft.height() / 2);

    drawPoint(projectedPoint, color);
}
void renderAllPoints(uint16_t color=TFT_WHITE) {
    int recordsToRender = recordsFilled ? maxRecords : currentRecord;

    for (int i = 0; i < recordsToRender; ++i) {
        renderPoint(magHistory[i], color);

    }
}

void onlineUpdateEllipsoid() {
    auto v = getMagReading();
    update_scatter_matrix(S, v.x, v.y, v.z);

    int recordsToRender = recordsFilled ? maxRecords : currentRecord;
    if(recordsToRender < 50) return;

    // Fit once
    Eigen::Vector3d center, radii;
    Eigen::Matrix3d rotation;
    fit_ellipsoid_from_scatter(S, center, radii, rotation);

    // Print results
    printVector(center, "Estimated center");
    printVector(radii, "Estimated radii");
    printMatrix(rotation, "Estimated rotation");
}

void loop() {
    updateSensors();
    updateTouch();
    printTouchToSerial();
    printSensorsToSerial();
    printSensorsToDisplay();

    appendMagHistory();

    if (touchActive) {
        if((lastTouchX != -1) && (lastTouchY != -1)) {
            rotationX += (touchX - lastTouchX) * 0.001f;
            rotationY += (touchY - lastTouchY) * 0.001f;
        }
        lastTouchX = touchX;
        lastTouchY = touchY;
        
        // only redraw the entire scene when the touch is active to reduce flickering
        tft.fillScreen(TFT_BLACK);
        drawScene(rotationX, rotationY, rotationZ);
        renderAllPoints();
    } else{
        lastTouchX = -1;
        lastTouchY = -1;
    }

    auto magReading = getMagReading();
    const Vec3 point{magReading.x, magReading.y, magReading.z};
    renderPoint(point, TFT_PURPLE);
    tft.invertDisplay(1);

    onlineUpdateEllipsoid();

    delay(30);
}