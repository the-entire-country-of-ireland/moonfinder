#pragma once
#include <Arduino.h>  // Needed for String
#include <ArduinoEigen.h>

// Location: Baltimore, MD (WGS84)
static const float LAT      = 39.2904f;
static const float LON      = -76.6122f;
static const float ALT_M    = 0.0f;        // meters above ellipsoid
static const float DECYEAR  = 2026.323f;   // 2025 + (dayOfYear/365)

// SLERP smoothing factor (0<β<1)
static const float BETA     = 0.1f;


struct Vec3 {
    float x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3(const float arr[3]) : x(arr[0]), y(arr[1]), z(arr[2]) {}

    float operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        if (i == 2) return z;
        return NAN;
    }

    // Implicit conversion to Eigen types
    operator Eigen::Vector3f() const {
        return Eigen::Vector3f(x, y, z);
    }

    operator Eigen::Vector3d() const {
        return Eigen::Vector3d(static_cast<double>(x),
                                static_cast<double>(y),
                                static_cast<double>(z));
    }

    Vec3 operator+(const Vec3& other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 operator-(const Vec3& other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    Vec3 operator*(float scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(float scalar) const {
        if (scalar == 0) return Vec3(0, 0, 0); // Handle division by zero
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3& operator+=(const Vec3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3& operator-=(const Vec3& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vec3& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vec3& operator/=(float scalar) {
        if (scalar != 0) { // Avoid division by zero
            x /= scalar;
            y /= scalar;
            z /= scalar;
        }
        return *this;
    }
    

    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 cross(const Vec3& other) const {
        return Vec3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    float norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    Vec3 normalized() const {
        float n = norm();
        if (n == 0) return Vec3(0, 0, 0);
        return Vec3(x / n, y / n, z / n);
    }

    String toString(int decimalPoints = 2) const {
        return String("(") + String(x, decimalPoints) + ", " + String(y, decimalPoints) + ", " + String(z, decimalPoints) + ")";
    }

    void print(int decimalPoints = 2) const {
        Serial.print(toString(decimalPoints));
    }

    void println(int decimalPoints = 2) const {
        Serial.println(toString(decimalPoints));
    }
};

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    float norm() const {
        return sqrt(w * w + x * x + y * y + z * z);
    }

    Quaternion normalized() const {
        float n = norm();
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    Quaternion operator-() const {
        return Quaternion(-w, -x, -y, -z);
    }

    Quaternion operator*(float scalar) const {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    Quaternion operator+(const Quaternion& other) const {
        return Quaternion(w + other.w, x + other.x, y + other.y, z + other.z);
    }

    float dot(const Quaternion& other) const {
        return w * other.w + x * other.x + y * other.y + z * other.z;
    }

    String toString() const {
        return String("(") + w + ", " + x + ", " + y + ", " + z + ")";
    }

    void print() const {
        Serial.print("[");
        Serial.print(w, 4); Serial.print(", ");
        Serial.print(x, 4); Serial.print(", ");
        Serial.print(y, 4); Serial.print(", ");
        Serial.print(z, 4); Serial.println("]");
    } 
};


void initReferenceVectors();

// Function to compute the 3x3 rotation matrix from a quaternion
void QuaternionToRotationMatrix(const Quaternion& q, float rotationMatrix[3][3]);
Quaternion RotationMatrixToQuaternion(const float rotationMatrix[3][3]);

Quaternion DavenportQMethod(const Vec3* body,
                             const Vec3* refv,
                             const float* wght,
                             int n);


// Function to apply affine transformation
Vec3 applyAffineTransformation(const Vec3& raw_vector, const float AFFINE_TRANSFORMATION[4][4]);
// Function to apply linear transformation
Vec3 applyLinearTransformation(const Vec3& raw_vector, const float LINEAR_TRANSFORMATION[3][3]);

// Function to transpose a 3x3 matrix in-place
void transposeMatrixInPlace(float matrix[3][3]);
Vec3 computeDeviceOrientationENU(Vec3& accel_v, Vec3& mag_v);


extern Quaternion q_smooth;
extern Vec3   body[3];
extern Vec3   refv[3];
extern float      wght[3];
extern float decl, cos_decl, sin_decl;