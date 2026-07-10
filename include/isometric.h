#include "display.h"

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Vec2 {
  float x;
  float y;
};

struct PointSet {
  const Vec3* points;
  const uint8_t* edges;
  uint8_t pointCount;
  uint8_t edgeCount;
  uint16_t color;
};


Vec3 rotate3d(Vec3 p, float rx, float ry, float rz);
Vec2 projectPoint(Vec3 p, float rx, float ry, float rz, float scale, int cx, int cy);


void drawLine(const Vec2& a, const Vec2& b, uint16_t color);
void drawPoint(const Vec2& p, uint16_t color);
void drawShape(const PointSet& shape, float rx, float ry, float rz, float scale, int cx, int cy);
void drawAxes(float rx, float ry, float rz, float scale, int cx, int cy);
void drawScene(float rx, float ry, float rz);