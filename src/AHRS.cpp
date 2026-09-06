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

NeuOrientation computeNeuOrientation(const Vector3d& acceleration,
                                     const Vector3d& calibrated_magnetometer) {
    NeuOrientation result;
    result.device_to_neu.setIdentity();
    result.x_neu = result.device_to_neu.col(0);
    result.y_neu = result.device_to_neu.col(1);
    result.z_neu = result.device_to_neu.col(2);
    result.valid = false;

    if (acceleration.norm() < 1e-9 || calibrated_magnetometer.norm() < 1e-9) {
        return result;
    }

    const Vector3d up = acceleration.normalized();
    Vector3d north = calibrated_magnetometer - up * up.dot(calibrated_magnetometer);
    if (north.norm() < 1e-9) return result;
    north.normalize();
    const Vector3d east = up.cross(north).normalized();

    result.device_to_neu.row(0) = north.transpose();
    result.device_to_neu.row(1) = east.transpose();
    result.device_to_neu.row(2) = up.transpose();
    result.x_neu = result.device_to_neu.col(0);
    result.y_neu = result.device_to_neu.col(1);
    result.z_neu = result.device_to_neu.col(2);
    result.valid = true;
    return result;
}

MoonPosition computeMoonEnu(const DateTime& utc) {
    constexpr double pi = 3.14159265358979323846;
    constexpr double deg = pi / 180.0;
    MoonPosition result{};
    result.enu.setZero();
    result.azimuth_deg = 0.0;
    result.elevation_deg = 0.0;
    result.distance_km = 0.0;
    result.valid = false;
    if (utc.unixtime() == 0) return result;

    const double jd = 2440587.5 + static_cast<double>(utc.unixtime()) / 86400.0;
    const double days = jd - 2451543.5;
    const double obliquity = (23.4393 - 3.563e-7 * days) * deg;

    const double node = (125.1228 - 0.0529538083 * days) * deg;
    const double inclination = 5.1454 * deg;
    const double argument = (318.0634 + 0.1643573223 * days) * deg;
    const double meanAnomaly = (115.3654 + 13.0649929509 * days) * deg;
    const double eccentricAnomaly = meanAnomaly + 0.0549 * std::sin(meanAnomaly) *
        (1.0 + 0.0549 * std::cos(meanAnomaly));
    const double xOrbital = 60.2666 * (std::cos(eccentricAnomaly) - 0.0549);
    const double yOrbital = 60.2666 * std::sqrt(1.0 - 0.0549 * 0.0549) *
        std::sin(eccentricAnomaly);
    const double trueAnomaly = std::atan2(yOrbital, xOrbital);
    const double radius = std::sqrt(xOrbital * xOrbital + yOrbital * yOrbital);
    const double longitude = trueAnomaly + argument;
    const double xEcliptic = radius * (std::cos(node) * std::cos(longitude) -
        std::sin(node) * std::sin(longitude) * std::cos(inclination));
    const double yEcliptic = radius * (std::sin(node) * std::cos(longitude) +
        std::cos(node) * std::sin(longitude) * std::cos(inclination));
    const double zEcliptic = radius * std::sin(longitude) * std::sin(inclination);

    const double rightAscension = std::atan2(
        yEcliptic * std::cos(obliquity) - zEcliptic * std::sin(obliquity), xEcliptic);
    const double declination = std::atan2(
        zEcliptic * std::cos(obliquity) + yEcliptic * std::sin(obliquity),
        std::sqrt(xEcliptic * xEcliptic +
                  std::pow(yEcliptic * std::cos(obliquity) - zEcliptic * std::sin(obliquity), 2)));

    const double jd0 = std::floor(jd - 0.5) + 0.5;
    const double sidereal = (280.46061837 + 360.98564736629 * (jd0 - 2451545.0) +
        360.98564736629 * (jd - jd0) + LON) * deg;
    const double hourAngle = sidereal - rightAscension;
    const double latitude = LAT * deg;
    const double east = std::cos(declination) * std::sin(hourAngle);
    const double north = std::cos(latitude) * std::sin(declination) -
        std::sin(latitude) * std::cos(declination) * std::cos(hourAngle);
    const double up = std::sin(latitude) * std::sin(declination) +
        std::cos(latitude) * std::cos(declination) * std::cos(hourAngle);

    result.enu << east, north, up;
    result.enu.normalize();
    result.azimuth_deg = std::atan2(east, north) / deg;
    if (result.azimuth_deg < 0.0) result.azimuth_deg += 360.0;
    result.elevation_deg = std::asin(up) / deg;
    result.distance_km = radius * 6378.14 / 60.2666;
    result.valid = true;
    return result;
}
