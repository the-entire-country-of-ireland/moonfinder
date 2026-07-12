#include <ArduinoEigen.h>



Eigen::Matrix<double,10,10> createBlankScatterMatrix();

// ================================================================
// Utility functions for pretty-printing matrices and vectors
// ================================================================
void printVector(const Eigen::Vector3d& v, const char* label);
void printMatrix(const Eigen::Matrix3d& M, const char* label);

// ================================================================
// Update the scatter matrix S (10x10) incrementally
// ================================================================
void update_scatter_matrix(Eigen::Matrix<double,10,10>& S, double x, double y, double z);

// ================================================================
// Final constrained ellipsoid fit using accumulated scatter matrix
// ================================================================
void fit_ellipsoid_from_scatter(const Eigen::Matrix<double,10,10>& S,
                                Eigen::Vector3d& center,
                                Eigen::Vector3d& radii,
                                Eigen::Matrix3d& rotation);


