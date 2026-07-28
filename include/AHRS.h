#pragma once
#include <ArduinoEigen.h>

using namespace Eigen;

// Location: Baltimore, MD (WGS84)
static const float LAT      = 39.2904f;
static const float LON      = -76.6122f;
static const float ALT_M    = 0.0f;        // meters above ellipsoid
static const float DECYEAR  = 2026.323f;   // 2025 + (dayOfYear/365)

extern float decl, cos_decl, sin_decl;

// SLERP smoothing factor (0<β<1)
static const float BETA     = 0.1f;


/**
 * @brief Struct to hold the computed orientation results
 */
struct Orientation {
    Quaterniond q;    // Rotation quaternion (device frame -> ENU frame)
    Matrix3d R;       // Rotation matrix (device frame -> ENU frame)
    Vector3d x_enu;   // Device x-axis expressed in ENU
    Vector3d y_enu;   // Device y-axis expressed in ENU
    Vector3d z_enu;   // Device z-axis expressed in ENU
};

/**
 * @brief Computes device orientation in ENU frame using Davenport's Q-Method
 *
 * @param acc_d Accelerometer reading in device frame (any units, will be normalized)
 * @param mag_d Magnetometer reading in device frame (any units, will be normalized)
 * @param gravity_enu Reference gravity direction in ENU (default: (0,0,1), "up" convention)
 * @param magnetic_north_enu Reference magnetic field direction in ENU
 * @return Orientation struct containing quaternion, rotation matrix, and device axes in ENU
 */
Orientation computeOrientation(
    const Vector3d& acc_d,
    const Vector3d& mag_d,
    const Vector3d& gravity_enu = Vector3d(0.0, 0.0, 1.0),
    const Vector3d& magnetic_north_enu = Vector3d(
        0.4135656507192516,
        -0.08036317018458757,
        0.9069207316094637
    )
);
