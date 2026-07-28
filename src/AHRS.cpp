#include "AHRS.h"
#include <wmm.h>                        // World Magnetic Model (Bolder Flight)
using namespace bfs;    // for wrldmagm()

// globals shared with Attitude.ino
Vec3   body[3];
Vec3   refv[3];
float      wght[3];
Quaternion q_smooth;
float decl, cos_decl, sin_decl;


void initReferenceVectors() {
  // 1) Compute local declination from WMM2020
  WmmData mm = wrldmagm(ALT_M, LAT, LON, DECYEAR, WMM2020);
  decl = mm.declination_deg * (PI / 180.0f);
  cos_decl = cos(decl);
  sin_decl = sin(decl);
  

  // 2) Earth‐frame reference vectors in East-North-Up
  refv[0] = Vec3{0, 0, -1};                    // gravity down
  refv[1] = Vec3{-0.08036317018458757, 0.4135656507192516, 0.9069207316094637};  // magnetic north

  // 3) Weights
  wght[0] = 3.0f;
  wght[1] = 1.0f;

  // 4) Initialize SLERP state
  q_smooth = Quaternion{1, 0, 0, 0};
}

Quaternion DavenportQMethod(const Vec3* body,
                             const Vec3* refv,
                             const float* wght,
                             int n) {
    // Build 3x3 B matrix
    float B[3][3] = {{0}};
    for (int i = 0; i < n; ++i) {
        float w = wght[i];
        const Vec3& b = body[i];
        const Vec3& r = refv[i];
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                B[j][k] += w * b[j] * r[k];
    }

    // Build symmetric 4x4 K matrix (invariant form)
    float S[3][3], sigma = 0;
    for (int i = 0; i < 3; ++i) {
        sigma += B[i][i];
        for (int j = 0; j < 3; ++j)
            S[i][j] = B[i][j] + B[j][i];
    }

    Vec3 Z = {
        B[1][2] - B[2][1],
        B[2][0] - B[0][2],
        B[0][1] - B[1][0]
    };

    // Create the 4x4 K matrix manually
    float K[4][4] = {
        { sigma,       Z.x,       Z.y,       Z.z },
        { Z.x, S[0][0] - sigma, S[0][1],     S[0][2] },
        { Z.y, S[1][0],     S[1][1] - sigma, S[1][2] },
        { Z.z, S[2][0],     S[2][1],     S[2][2] - sigma }
    };

    // Power iteration to find dominant eigenvector (quaternion)
    Quaternion q{1, 0, 0, 0};
    for (int iter = 0; iter < 30; ++iter) {
        Quaternion q_next = {
            K[0][0]*q.w + K[0][1]*q.x + K[0][2]*q.y + K[0][3]*q.z,
            K[1][0]*q.w + K[1][1]*q.x + K[1][2]*q.y + K[1][3]*q.z,
            K[2][0]*q.w + K[2][1]*q.x + K[2][2]*q.y + K[2][3]*q.z,
            K[3][0]*q.w + K[3][1]*q.x + K[3][2]*q.y + K[3][3]*q.z
        };
        q = q_next.normalized();
    }

    return q;
}

Vec3 applyAffineTransformation(const Vec3& raw_vector, const float AFFINE_TRANSFORMATION[4][4]){
    return Vec3{
        AFFINE_TRANSFORMATION[0][0] * raw_vector.x +
        AFFINE_TRANSFORMATION[0][1] * raw_vector.y +
        AFFINE_TRANSFORMATION[0][2] * raw_vector.z +
        AFFINE_TRANSFORMATION[0][3],

        AFFINE_TRANSFORMATION[1][0] * raw_vector.x +
        AFFINE_TRANSFORMATION[1][1] * raw_vector.y +
        AFFINE_TRANSFORMATION[1][2] * raw_vector.z +
        AFFINE_TRANSFORMATION[1][3],

        AFFINE_TRANSFORMATION[2][0] * raw_vector.x +
        AFFINE_TRANSFORMATION[2][1] * raw_vector.y +
        AFFINE_TRANSFORMATION[2][2] * raw_vector.z +
        AFFINE_TRANSFORMATION[2][3]
    };
}


Vec3 applyLinearTransformation(const Vec3& raw_vector, const float LINEAR_TRANSFORMATION[3][3]){
    return Vec3{
        LINEAR_TRANSFORMATION[0][0] * raw_vector.x +
        LINEAR_TRANSFORMATION[0][1] * raw_vector.y +
        LINEAR_TRANSFORMATION[0][2] * raw_vector.z,

        LINEAR_TRANSFORMATION[1][0] * raw_vector.x +
        LINEAR_TRANSFORMATION[1][1] * raw_vector.y +
        LINEAR_TRANSFORMATION[1][2] * raw_vector.z,

        LINEAR_TRANSFORMATION[2][0] * raw_vector.x +
        LINEAR_TRANSFORMATION[2][1] * raw_vector.y +
        LINEAR_TRANSFORMATION[2][2] * raw_vector.z
    };
}

Quaternion RotationMatrixToQuaternion(const float rotationMatrix[3][3]) {
    float trace = rotationMatrix[0][0] + rotationMatrix[1][1] + rotationMatrix[2][2];
    Quaternion q;

    if (trace > 0) {
        float s = sqrt(trace + 1.0f) * 2.0f; // s = 4 * qw
        q.w = 0.25f * s;
        q.x = (rotationMatrix[2][1] - rotationMatrix[1][2]) / s;
        q.y = (rotationMatrix[0][2] - rotationMatrix[2][0]) / s;
        q.z = (rotationMatrix[1][0] - rotationMatrix[0][1]) / s;
    } else if ((rotationMatrix[0][0] > rotationMatrix[1][1]) && (rotationMatrix[0][0] > rotationMatrix[2][2])) {
        float s = sqrt(1.0f + rotationMatrix[0][0] - rotationMatrix[1][1] - rotationMatrix[2][2]) * 2.0f; // s = 4 * qx
        q.w = (rotationMatrix[2][1] - rotationMatrix[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (rotationMatrix[0][1] + rotationMatrix[1][0]) / s;
        q.z = (rotationMatrix[0][2] + rotationMatrix[2][0]) / s;
    } else if (rotationMatrix[1][1] > rotationMatrix[2][2]) {
        float s = sqrt(1.0f + rotationMatrix[1][1] - rotationMatrix[0][0] - rotationMatrix[2][2]) * 2.0f; // s = 4 * qy
        q.w = (rotationMatrix[0][2] - rotationMatrix[2][0]) / s;
        q.x = (rotationMatrix[0][1] + rotationMatrix[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (rotationMatrix[1][2] + rotationMatrix[2][1]) / s;
    } else {
        float s = sqrt(1.0f + rotationMatrix[2][2] - rotationMatrix[0][0] - rotationMatrix[1][1]) * 2.0f; // s = 4 * qz
        q.w = (rotationMatrix[1][0] - rotationMatrix[0][1]) / s;
        q.x = (rotationMatrix[0][2] + rotationMatrix[2][0]) / s;
        q.y = (rotationMatrix[1][2] + rotationMatrix[2][1]) / s;
        q.z = 0.25f * s;
    }

    return q.normalized();
}


// Function to compute the 3x3 rotation matrix from a quaternion
void QuaternionToRotationMatrix(const Quaternion& q, float rotationMatrix[3][3]) {
    Quaternion normalizedQ = q.normalized(); // Ensure the quaternion is normalized
    float w = normalizedQ.w;
    float x = normalizedQ.x;
    float y = normalizedQ.y;
    float z = normalizedQ.z;

    // Compute the rotation matrix elements
    rotationMatrix[0][0] = 1 - 2 * (y * y + z * z);
    rotationMatrix[0][1] = 2 * (x * y - z * w);
    rotationMatrix[0][2] = 2 * (x * z + y * w);

    rotationMatrix[1][0] = 2 * (x * y + z * w);
    rotationMatrix[1][1] = 1 - 2 * (x * x + z * z);
    rotationMatrix[1][2] = 2 * (y * z - x * w);

    rotationMatrix[2][0] = 2 * (x * z - y * w);
    rotationMatrix[2][1] = 2 * (y * z + x * w);
    rotationMatrix[2][2] = 1 - 2 * (x * x + y * y);
}

void transposeMatrixInPlace(float matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            // Swap elements at (i, j) and (j, i)
            float temp = matrix[i][j];
            matrix[i][j] = matrix[j][i];
            matrix[j][i] = temp;
        }
    }
}



Vec3 computeDeviceOrientationENU(Vec3& accel_v, Vec3& mag_v) {
    // Normalize accelerometer readings to get the "Up" vector
    Vec3 up = accel_v.normalized(); // "Up" is aligned with gravity (accelerometer)

    // Magnetic north vector in the ENU frame (provided)
    Vec3 magnetic_north_enu = Vec3(0.4135656507192516, -0.08036317018458757, 0.9069207316094637);

    // Remove the "Up" component from the magnetic north vector to get the horizontal component
    Vec3 N_mag = (magnetic_north_enu - up * (magnetic_north_enu.dot(up))).normalized();

    // Define the true north vector in the horizontal NE plane
    Vec3 N = Vec3(0, 1, 0); // True north in the ENU frame lies along the Y-axis

    // Compute the angle between N_mag and N in the horizontal plane
    float cos_theta = N_mag.dot(N); // Cosine of the angle
    float sin_theta = N_mag.cross(N).dot(up); // Sine of the angle (using the "Up" axis for direction)

    // Perform a rotation around the "Up" axis by the angle to align N_mag with N
    Vec3 north = N_mag * cos_theta + up.cross(N_mag) * sin_theta + up * (up.dot(N_mag) * (1 - cos_theta));

    // Compute the "East" vector as the cross product of "North" and "Up"
    Vec3 east = north.cross(up).normalized();

    // The device's X-axis in the body frame is {1, 0, 0}
    Vec3 x_axis_body = Vec3(1, 0, 0);

    // Transform the X-axis from the body frame to the ENU frame
    Vec3 x_axis_enu = (east * x_axis_body.x +
                  north * x_axis_body.y +
                  up * x_axis_body.z).normalized();

    return x_axis_enu;
}