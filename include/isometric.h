#ifndef ISOMETRIC_H
#define ISOMETRIC_H

#include "display.h"
#include <ArduinoEigen.h>

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;

struct PointSet {
  const Vector3d* points;
  const uint8_t* edges;
  uint8_t pointCount;
  uint8_t edgeCount;
  uint16_t color;
};

// Rotation matrix management
extern Matrix3d currentRotation;

void initRotation(float rx, float ry, float rz);
Matrix3d eulerToMatrix(float rx, float ry, float rz);
void matrixToEuler(const Matrix3d& R, float& rx, float& ry, float& rz);
void updateRotationFromTouch(float deltaX, float deltaY);

// Core rendering functions
Vector3d rotate3d(const Vector3d& p, const Matrix3d& R);
Vector2d projectPoint(const Vector3d& p, const Matrix3d& R, float scale, int cx, int cy);

void drawLine(const Vector2d& a, const Vector2d& b, uint16_t color);
void drawPoint(const Vector2d& p, uint16_t color);
void drawShape(const PointSet& shape, const Matrix3d& R, float scale, int cx, int cy);
void drawAxes(const Matrix3d& R, float scale, int cx, int cy);
void drawWireSphere(const Matrix3d& R, float scale, int cx, int cy, uint16_t color);
void drawScene();
void drawSceneCube();

#endif