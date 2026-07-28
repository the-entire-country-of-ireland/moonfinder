#include <ArduinoEigen.h>
#include "display.h"
#include "isometric.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

Matrix3d currentRotation = Matrix3d::Identity();

// Initialize rotation from Euler angles
void initRotation(float rx, float ry, float rz) {
    currentRotation = eulerToMatrix(rx, ry, rz);
}

// Convert Euler angles to rotation matrix
Matrix3d eulerToMatrix(float rx, float ry, float rz) {
    const float cx = cos(rx), sx = sin(rx);
    const float cy = cos(ry), sy = sin(ry);
    const float cz = cos(rz), sz = sin(rz);
    
    Matrix3d Rx, Ry, Rz;
    
    Rx << 1,   0,    0,
          0,  cx, -sx,
          0,  sx,  cx;
    
    Ry << cy,  0,  sy,
          0,   1,   0,
         -sy,  0,  cy;
    
    Rz << cz, -sz,  0,
          sz,  cz,  0,
          0,   0,   1;
    
    return Rz * Ry * Rx;
}

// Convert rotation matrix back to Euler angles
void matrixToEuler(const Matrix3d& R, float& rx, float& ry, float& rz) {
    ry = asin(-R(2, 0));
    
    if (cos(ry) > 1e-6) {
        rx = atan2(R(2, 1), R(2, 2));
        rz = atan2(R(1, 0), R(0, 0));
    } else {
        rx = atan2(-R(1, 2), R(1, 1));
        rz = 0;
    }
}

void updateRotationFromTouch(float deltaX, float deltaY) {
    const float sensitivity = 0.005f;
    
    // Get screen-space axes (camera right and up)
    Vector3d screenRight(1.0f, 0.0f, 0.0f);
    Vector3d screenUp(0.0f, 1.0f, 0.0f);
    
    // Transform by current rotation
    screenRight = currentRotation * screenRight;
    screenUp = currentRotation * screenUp;
    
    // Build rotation axis in world space
    Vector3d axis = -deltaX * sensitivity * screenUp + 
                     deltaY * sensitivity * screenRight;
    
    float angle = axis.norm();
    
    if (angle > 1e-6f) {
        axis /= angle;  // Normalize
        
        // Create incremental rotation matrix using Rodrigues' formula
        Matrix3d K;
        K <<      0, -axis.z(),  axis.y(),
             axis.z(),       0, -axis.x(),
            -axis.y(), axis.x(),       0;
        
        Matrix3d deltaR = Matrix3d::Identity() + 
                         sin(angle) * K + 
                         (1 - cos(angle)) * K * K;
        
        // Apply incremental rotation
        currentRotation = deltaR * currentRotation;
    }
}

Vector3d rotate3d(const Vector3d& p, const Matrix3d& R) {
    return R * p;
}

Vector2d projectPoint(const Vector3d& p, const Matrix3d& R, float scale, int cx, int cy) {
    Vector3d r = rotate3d(p, R);
    const float tx = (r.x() - r.y()) * scale;
    const float ty = (r.x() + r.y()) * 0.0f * scale - r.z() * 1.4f * scale;
    return Vector2d(cx + tx, cy + ty);
}

void drawLine(const Vector2d& a, const Vector2d& b, uint16_t color) {
    tft.drawLine(static_cast<int16_t>(a.x()), static_cast<int16_t>(a.y()),
                static_cast<int16_t>(b.x()), static_cast<int16_t>(b.y()), color);
}

void drawPoint(const Vector2d& p, uint16_t color) {
    tft.fillCircle(static_cast<int16_t>(p.x()), static_cast<int16_t>(p.y()), 2, color);
}

void drawShape(const PointSet& shape, const Matrix3d& R, float scale, int cx, int cy) {
    for (uint8_t i = 0; i < shape.edgeCount; ++i) {
        const uint8_t a = shape.edges[i * 2];
        const uint8_t b = shape.edges[i * 2 + 1];
        const Vector2d pa = projectPoint(shape.points[a], R, scale, cx, cy);
        const Vector2d pb = projectPoint(shape.points[b], R, scale, cx, cy);
        drawLine(pa, pb, shape.color);
    }

    for (uint8_t i = 0; i < shape.pointCount; ++i) {
        const Vector2d p = projectPoint(shape.points[i], R, scale, cx, cy);
        drawPoint(p, shape.color);
    }
}

void drawWireSphere(const Matrix3d& R, float scale, int cx, int cy, uint16_t color) {
    const int lonSteps = 16;   // meridians
    const int latSteps = 8;    // parallels
    const int ringSteps = 32;  // points per ring

    // Latitude rings (parallels)
    for (int i = 1; i < latSteps; ++i) {
        float v = -HALF_PI + i * (PI / latSteps);
        float z = sin(v);
        float r = cos(v);

        Vector2d first, prev;
        for (int j = 0; j <= ringSteps; ++j) {
            float u = j * (2.0f * PI / ringSteps);
            Vector3d p(r * cos(u), r * sin(u), z);
            Vector2d q = projectPoint(p, R, scale, cx, cy);

            if (j == 0) {
                first = q;
            } else {
                drawLine(prev, q, color);
            }
            prev = q;
        }
    }

    // Longitude rings (meridians)
    for (int i = 0; i < lonSteps; ++i) {
        float theta = i * (2.0f * PI / lonSteps);

        for (int j = 0; j < ringSteps; ++j) {
            float v1 = -HALF_PI + j * (PI / ringSteps);
            float v2 = -HALF_PI + (j + 1) * (PI / ringSteps);

            Vector3d p1(cos(v1) * cos(theta), cos(v1) * sin(theta), sin(v1));
            Vector3d p2(cos(v2) * cos(theta), cos(v2) * sin(theta), sin(v2));

            Vector2d q1 = projectPoint(p1, R, scale, cx, cy);
            Vector2d q2 = projectPoint(p2, R, scale, cx, cy);

            drawLine(q1, q2, color);
        }
    }
}

void drawAxes(const Matrix3d& R, float scale, int cx, int cy) {
    const Vector3d origin(0.0f, 0.0f, 0.0f);
    const Vector3d xAxis(1.4f, 0.0f, 0.0f);
    const Vector3d yAxis(0.0f, 1.4f, 0.0f);
    const Vector3d zAxis(0.0f, 0.0f, 1.4f);

    const Vector2d o = projectPoint(origin, R, scale, cx, cy);
    const Vector2d x = projectPoint(xAxis, R, scale, cx, cy);
    const Vector2d y = projectPoint(yAxis, R, scale, cx, cy);
    const Vector2d z = projectPoint(zAxis, R, scale, cx, cy);

    drawLine(o, x, TFT_RED);
    drawLine(o, y, TFT_GREEN);
    drawLine(o, z, TFT_BLUE);

    // Draw labels at the end of each axis
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFont(&fonts::FreeSans9pt7b);
    tft.setTextSize(1.5);
    tft.drawString("X", static_cast<int16_t>(x.x()), static_cast<int16_t>(x.y()));
    tft.drawString("Y", static_cast<int16_t>(y.x()), static_cast<int16_t>(y.y()));
    tft.drawString("Z", static_cast<int16_t>(z.x()), static_cast<int16_t>(z.y()));
}

void drawRotation() {
    float rx, ry, rz;
    matrixToEuler(currentRotation, rx, ry, rz);
    Eigen::Vector3d vec(rx, ry, rz);

    printVectorToDisplay("Rot: ", vec, 300);
}

void drawScene() {
    tft.fillRect(0, 50, tft.width(), tft.height(), TFT_BLACK);

    const int cx = tft.width() / 2;
    const int cy = tft.height() / 2 - 10;
    const float scale = 50.0f;

    drawAxes(currentRotation, scale, cx, cy);
    drawWireSphere(currentRotation, scale, cx, cy, TFT_GREY);
    drawRotation();
}

void drawSceneCube() {
    tft.fillRect(0, 50, tft.width(), tft.height(), TFT_BLACK);

    const int cx = tft.width() / 2;
    const int cy = tft.height() / 2 - 10;
    const float scale = min(tft.width(), tft.height()) / 6.0f;

    drawAxes(currentRotation, scale * 1.3, cx, cy);

    static const Vector3d cubePoints[] = {
        Vector3d(-1.0f, -1.0f, -1.0f),
        Vector3d(1.0f, -1.0f, -1.0f),
        Vector3d(1.0f, 1.0f, -1.0f),
        Vector3d(-1.0f, 1.0f, -1.0f),
        Vector3d(-1.0f, -1.0f, 1.0f),
        Vector3d(1.0f, -1.0f, 1.0f),
        Vector3d(1.0f, 1.0f, 1.0f),
        Vector3d(-1.0f, 1.0f, 1.0f)
    };

    static const uint8_t cubeEdges[] = {
        0, 1, 1, 2, 2, 3, 3, 0,
        4, 5, 5, 6, 6, 7, 7, 4,
        0, 4, 1, 5, 2, 6, 3, 7
    };

    static const PointSet cube = {cubePoints, cubeEdges, 8, 12, TFT_CYAN};

    drawShape(cube, currentRotation, scale, cx, cy);
}