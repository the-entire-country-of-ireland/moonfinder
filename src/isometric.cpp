#include "display.h"
#include "isometric.h"


Vec3 rotate3d(Vec3 p, float rx, float ry, float rz) {
  const float cx = cos(rx);
  const float sx = sin(rx);
  const float cy = cos(ry);
  const float sy = sin(ry);
  const float cz = cos(rz);
  const float sz = sin(rz);

  const float y1 = p.y * cx - p.z * sx;
  const float z1 = p.y * sx + p.z * cx;

  const float x2 = p.x * cy + z1 * sy;
  const float z2 = -p.x * sy + z1 * cy;

  const float x3 = x2 * cz - y1 * sz;
  const float y3 = x2 * sz + y1 * cz;

  return Vec3{x3, y3, z2};
}

Vec2 projectPoint(Vec3 p, float rx, float ry, float rz, float scale, int cx, int cy) {
  Vec3 r = rotate3d(p, rx, ry, rz);
  const float tx = (r.x - r.y) * scale;
  const float ty = (r.x + r.y) * 0.0f * scale - r.z * 1.4f * scale;
  return Vec2{cx + tx, cy + ty};
}

void drawLine(const Vec2& a, const Vec2& b, uint16_t color) {
  tft.drawLine(static_cast<int16_t>(a.x), static_cast<int16_t>(a.y),
              static_cast<int16_t>(b.x), static_cast<int16_t>(b.y), color);
}

void drawPoint(const Vec2& p, uint16_t color) {
  tft.fillCircle(static_cast<int16_t>(p.x), static_cast<int16_t>(p.y), 2, color);
}

void drawShape(const PointSet& shape, float rx, float ry, float rz, float scale, int cx, int cy) {
  for (uint8_t i = 0; i < shape.edgeCount; ++i) {
    const uint8_t a = shape.edges[i * 2];
    const uint8_t b = shape.edges[i * 2 + 1];
    const Vec2 pa = projectPoint(shape.points[a], rx, ry, rz, scale, cx, cy);
    const Vec2 pb = projectPoint(shape.points[b], rx, ry, rz, scale, cx, cy);
    drawLine(pa, pb, shape.color);
  }

  for (uint8_t i = 0; i < shape.pointCount; ++i) {
    const Vec2 p = projectPoint(shape.points[i], rx, ry, rz, scale, cx, cy);
    drawPoint(p, shape.color);
  }
}

void drawAxes(float rx, float ry, float rz, float scale, int cx, int cy) {
  const Vec3 origin{0.0f, 0.0f, 0.0f};
  const Vec3 xAxis{1.4f, 0.0f, 0.0f};
  const Vec3 yAxis{0.0f, 1.4f, 0.0f};
  const Vec3 zAxis{0.0f, 0.0f, 1.4f};

  const Vec2 o = projectPoint(origin, rx, ry, rz, scale, cx, cy);
  const Vec2 x = projectPoint(xAxis, rx, ry, rz, scale, cx, cy);
  const Vec2 y = projectPoint(yAxis, rx, ry, rz, scale, cx, cy);
  const Vec2 z = projectPoint(zAxis, rx, ry, rz, scale, cx, cy);

  drawLine(o, x, TFT_RED);
  drawLine(o, y, TFT_GREEN);
  drawLine(o, z, TFT_BLUE);
}

void drawScene(float rx, float ry, float rz) {

  tft.fillRect(0, 50, tft.width(), tft.height(), TFT_BLACK);

  const int cx = tft.width() / 2;
  const int cy = tft.height() / 2 - 10;
  const float scale = min(tft.width(), tft.height()) / 6.0f;

  drawAxes(rx, ry, rz, scale, cx, cy);

  static const Vec3 cubePoints[] = {
    {-1.0f, -1.0f, -1.0f},
    {1.0f, -1.0f, -1.0f},
    {1.0f, 1.0f, -1.0f},
    {-1.0f, 1.0f, -1.0f},
    {-1.0f, -1.0f, 1.0f},
    {1.0f, -1.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    {-1.0f, 1.0f, 1.0f}
  };

  static const uint8_t cubeEdges[] = {
    0, 1, 1, 2, 2, 3, 3, 0,
    4, 5, 5, 6, 6, 7, 7, 4,
    0, 4, 1, 5, 2, 6, 3, 7
  };

  static const PointSet cube = {cubePoints, cubeEdges, 8, 12, TFT_CYAN};

  drawShape(cube, rx, ry, rz, scale, cx, cy);

}
