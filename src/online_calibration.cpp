#include <Arduino.h>
#include <ArduinoEigen.h>
#include <cfloat>
#include <cmath>
#include "online_calibration.h"

using namespace Eigen;

// ================================================================
// Constructor
// ================================================================
MagnetometerCalibration::MagnetometerCalibration() {
    reset();
}

// ================================================================
// Reset calibration to initial state
// ================================================================
void MagnetometerCalibration::reset() {
    S_ = Matrix<double,10,10>::Zero();
    center_ = Vector3d::Zero();
    radii_ = Vector3d::Ones();
    rotation_ = Matrix3d::Identity();
}

// ================================================================
// Update the scatter matrix S (10x10) incrementally
// ================================================================
void MagnetometerCalibration::updateScatterMatrix(double x, double y, double z) {
    double v[10] = {
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
        1.0
    };

    // S += v * v^T
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j <= i; ++j) {
            S_(i, j) += v[i] * v[j];
        }
    }
}

// ================================================================
// Final constrained ellipsoid fit using accumulated scatter matrix
// ================================================================
void MagnetometerCalibration::fitEllipsoid() {
    // Ensure symmetric
    Matrix<double,10,10> Sym = S_;
    for (int i = 0; i < 10; ++i)
        for (int j = i+1; j < 10; ++j)
            Sym(i, j) = Sym(j, i);

    // --- Step 1: Eigen decomposition ---
    SelfAdjointEigenSolver<Matrix<double,10,10>> es(Sym);
    VectorXd evals = es.eigenvalues();
    MatrixXd evecs = es.eigenvectors();

    // --- Step 2: Select smallest positive eigenvalue ---
    int minIndex = -1;
    double minVal = DBL_MAX;
    for (int i = 0; i < evals.size(); ++i) {
        if (evals(i) > 0 && evals(i) < minVal) {
            minVal = evals(i);
            minIndex = i;
        }
    }

    if (minIndex < 0) {
        Serial.println("No valid positive eigenvalue found!");
        return;
    }

    VectorXd q = evecs.col(minIndex);

    // --- Step 3: Extract parameters ---
    double A = q(0), B = q(1), C = q(2);
    double Dxy = q(3), Dxz = q(4), Dyz = q(5);
    double Dx = q(6), Dy = q(7), Dz = q(8);
    double D0 = q(9);

    Matrix3d Amat;
    Amat << A, Dxy, Dxz,
            Dxy, B, Dyz,
            Dxz, Dyz, C;

    Vector3d bvec(Dx, Dy, Dz);

    // --- Step 4: Compute center ---
    center_ = -Amat.inverse() * bvec;

    // --- Step 5: Compute constant term ---
    double val = center_.transpose() * Amat * center_ - D0;

    // --- Step 6: Normalize and compute axes ---
    Amat /= val;

    SelfAdjointEigenSolver<Matrix3d> es2(Amat);
    rotation_ = es2.eigenvectors();
    Vector3d eigvals = es2.eigenvalues();

    for (int i = 0; i < 3; ++i) {
        eigvals(i) = fabs(eigvals(i));
        radii_(i) = 1.0 / sqrt(eigvals(i));
    }
}

// ================================================================
// Transform magnetometer reading using current calibration
// ================================================================
Vector3d MagnetometerCalibration::transformMagnetometer(const Vector3d& data) const {
    // 1. Translate to center
    Vector3d result = data - center_;
    
    // 2. Rotate to align with coordinate axes
    result = rotation_.transpose() * result;
    
    // 3. Scale to unit sphere
    result = result.cwiseQuotient(radii_);
    
    // 4. Rotate back to original orientation
    result = rotation_ * result;
    
    return result;
}

// ================================================================
// Utility functions for debugging
// ================================================================
void MagnetometerCalibration::printVector(const Vector3d& v, const char* label) const {
    Serial.printf("%s: [%.6f, %.6f, %.6f]\n", label, v(0), v(1), v(2));
}

void MagnetometerCalibration::printMatrix(const Matrix3d& M, const char* label) const {
    Serial.printf("%s:\n", label);
    for (int i = 0; i < 3; ++i)
        Serial.printf("  [%.6f, %.6f, %.6f]\n", M(i,0), M(i,1), M(i,2));
}

void MagnetometerCalibration::printCenter(const char* label) const {
    printVector(center_, label);
}

void MagnetometerCalibration::printRadii(const char* label) const {
    printVector(radii_, label);
}

void MagnetometerCalibration::printRotation(const char* label) const {
    printMatrix(rotation_, label);
}

RotationFinder::RotationFinder() {
    G.setZero();
    h.setZero();
    R.setIdentity();
}

void RotationFinder::addSample(const Eigen::Vector3d& mag, const Eigen::Vector3d& acc) {
    Eigen::Vector3d m = mag;
    Eigen::Vector3d a = acc.normalized();
    
    Eigen::Matrix<double, 9, 1> k;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            k(i * 3 + j) = m(i) * a(j);
    
    G += k * k.transpose();
    h += k;
}

void RotationFinder::solve() {
    Eigen::Matrix<double, 9, 1> r = G.ldlt().solve(h);
    
    Eigen::Matrix3d R_raw;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_raw(j, i) = r(i * 3 + j);
    
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
}



AffineFinder::AffineFinder(double target_) {
    target = target_;
    reset();
}

void AffineFinder::reset() {
    G.setZero();
    h.setZero();

    M.setIdentity();
    c.setZero();
    solved = false;
}


void AffineFinder::addSample(const Eigen::Vector3d& mag, const Eigen::Vector3d& acc) {
    Eigen::Vector3d m = mag;
    Eigen::Vector3d a = acc.normalized();

    /*
        We use homogeneous magnetometer vector:
            m_aug = [m_x, m_y, m_z, 1]^T
        and affine transform:
            L = [ M c ]
        where L is 3x4.
        Constraint:
            a^T L m_aug = target
        Vectorized:
            a^T L m_aug = (m_aug ⊗ a)^T vec(L)
        k is 12x1:
            k = m_aug ⊗ a
        using column-major vec(L), which matches Eigen's storage order.
    */

    Eigen::Matrix<double, 4, 1> m_aug;
    m_aug << m(0), m(1), m(2), 1.0;

    Eigen::Matrix<double, 12, 1> k;

    int idx = 0;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 3; ++row) {
            k(idx++) = m_aug(col) * a(row);
        }
    }

    G += k * k.transpose();
    h += k;
}

bool AffineFinder::solve() {
    Eigen::LDLT<Eigen::Matrix<double, 12, 12> > ldlt(G);
    Eigen::Matrix<double, 12, 1> p = ldlt.solve(h);

    if (ldlt.info() != Eigen::Success) {
        return false;
    }

    p = p * target;

    Eigen::Matrix<double, 3, 4> T;
    int idx = 0;
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 3; ++row) {
            T(row, col) = p(idx++);
        }
    }

    M = T.block<3, 3>(0, 0);
    c = T.col(3);

    solved = true;
    return true;
}

void AffineFinder::load(const Eigen::Matrix3d &_M, const Eigen::Vector3d &_c)
{
    solved = true;
    M = _M;
    c = _c;
}