#include "AHRS.h"
#include <wmm.h>    // World Magnetic Model (Bolder Flight)

using namespace Eigen;

float decl, cos_decl, sin_decl;

void initReferenceVectors() {
  // 1) Compute local declination from WMM2020
  bfs::WmmData mm = bfs::wrldmagm(ALT_M, LAT, LON, DECYEAR, bfs::WMM2020);
  decl = mm.declination_deg * (PI / 180.0f);
  cos_decl = cos(decl);
  sin_decl = sin(decl);

  Eigen::Vector3d(-0.080363, 0.4135656, 0.90692);  // magnetic north

}


Orientation computeOrientation(
    const Vector3d& acc_d,
    const Vector3d& mag_d,
    const Vector3d& gravity_enu,
    const Vector3d& magnetic_north_enu
) {
    Orientation result;

    // Normalize device-frame measurements
    Vector3d b1 = acc_d.normalized();
    Vector3d b2 = mag_d.normalized();

    // Normalize reference vectors
    Vector3d r1 = gravity_enu.normalized();
    Vector3d r2 = magnetic_north_enu.normalized();

    // Step 1: Attitude profile matrix B
    Matrix3d B = b1 * r1.transpose() + b2 * r2.transpose();

    // Step 2: Extract S, sigma, Z
    Matrix3d S = B + B.transpose();
    double sigma = B.trace();

    Vector3d Z;
    Z(0) = B(1, 2) - B(2, 1);
    Z(1) = B(2, 0) - B(0, 2);
    Z(2) = B(0, 1) - B(1, 0);

    // Step 3: Form Davenport K matrix (4x4)
    Matrix4d K;
    K.block<3, 3>(0, 0) = S - sigma * Matrix3d::Identity();
    K.block<3, 1>(0, 3) = Z;
    K.block<1, 3>(3, 0) = Z.transpose();
    K(3, 3) = sigma;

    // Step 4: Eigen-decomposition, find max eigenvalue's eigenvector
    SelfAdjointEigenSolver<Matrix4d> eigensolver(K);

    if (eigensolver.info() != Success) {
        // Fallback: identity orientation
        result.q = Quaterniond::Identity();
        result.R = Matrix3d::Identity();
        result.x_enu = Vector3d(1, 0, 0);
        result.y_enu = Vector3d(0, 1, 0);
        result.z_enu = Vector3d(0, 0, 1);
        return result;
    }

    int maxIndex;
    eigensolver.eigenvalues().maxCoeff(&maxIndex);
    Vector4d q_vec = eigensolver.eigenvectors().col(maxIndex);

    // Step 5: Build quaternion (Eigen order: w, x, y, z)
    Quaterniond q(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    q.normalize();

    // Step 6: Rotation matrix
    Matrix3d R = q.toRotationMatrix();

    // Step 7: Extract device axes in ENU
    result.q = q;
    result.R = R;
    result.x_enu = R.col(0);
    result.y_enu = R.col(1);
    result.z_enu = R.col(2);

    return result;
}
