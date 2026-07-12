#include <Arduino.h>
#include <ArduinoEigen.h>
#include <cfloat>
#include <cmath>
#include "online_calibration.h"

using namespace Eigen;

Matrix<double,10,10> createBlankScatterMatrix() {
  return Matrix<double,10,10>::Zero();
}

// ================================================================
// Utility functions for pretty-printing matrices and vectors
// ================================================================
void printVector(const Vector3d& v, const char* label) {
  Serial.printf("%s: [%.6f, %.6f, %.6f]\n", label, v(0), v(1), v(2));
}

void printMatrix(const Matrix3d& M, const char* label) {
  Serial.printf("%s:\n", label);
  for (int i = 0; i < 3; ++i)
    Serial.printf("  [%.6f, %.6f, %.6f]\n", M(i,0), M(i,1), M(i,2));
}

// ================================================================
// Update the scatter matrix S (10x10) incrementally
// ================================================================
void update_scatter_matrix(Matrix<double,10,10>& S, double x, double y, double z) {
  double v[10] = {
    x*x, y*y, z*z,
    2*x*y, 2*x*z, 2*y*z,
    2*x, 2*y, 2*z,
    1.0
  };

  // S += v * v^T
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j <= i; ++j) {
      S(i, j) += v[i] * v[j];
    }
  }
}

// ================================================================
// Final constrained ellipsoid fit using accumulated scatter matrix
// ================================================================
void fit_ellipsoid_from_scatter(const Matrix<double,10,10>& S,
                                Vector3d& center,
                                Vector3d& radii,
                                Matrix3d& rotation)
{
  // Ensure symmetric
  Matrix<double,10,10> Sym = S;
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
  center = -Amat.inverse() * bvec;

  // --- Step 5: Compute constant term ---
  double val = center.transpose() * Amat * center - D0;

  // --- Step 6: Normalize and compute axes ---
  Amat /= val;

  SelfAdjointEigenSolver<Matrix3d> es2(Amat);
  rotation = es2.eigenvectors();
  Vector3d eigvals = es2.eigenvalues();

  for (int i = 0; i < 3; ++i) {
    eigvals(i) = fabs(eigvals(i));
    radii(i) = 1.0 / sqrt(eigvals(i));
  }
}




// example usage
// stupid ordering of axes from magnetometer
// update_scatter_matrix(S, y, x, -z);

// // Fit once
// Vector3d center, radii;
// Matrix3d rotation;
// fit_ellipsoid_from_scatter(S, center, radii, rotation);

// // Print results
// printVector(center, "Estimated center");
// printVector(radii, "Estimated radii");
// printMatrix(rotation, "Estimated rotation");  

