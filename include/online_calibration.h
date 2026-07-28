#ifndef ONLINE_CALIBRATION_H
#define ONLINE_CALIBRATION_H

#include <ArduinoEigen.h>

class MagnetometerCalibration {
public:
    // Constructor
    MagnetometerCalibration();
    
    // Reset calibration to initial state
    void reset();
    
    // Update scatter matrix with new measurement
    void updateScatterMatrix(double x, double y, double z);
    
    // Perform ellipsoid fit using accumulated data
    void fitEllipsoid();
    
    // Transform magnetometer reading using current calibration
    Eigen::Vector3d transformMagnetometer(const Eigen::Vector3d& data) const;
    
    // Getters
    const Eigen::Matrix<double,10,10>& getScatterMatrix() const { return S_; }
    const Eigen::Vector3d& getCenter() const { return center_; }
    const Eigen::Vector3d& getRadii() const { return radii_; }
    const Eigen::Matrix3d& getRotation() const { return rotation_; }
    
    // Utility functions for debugging
    void printCenter(const char* label = "Center") const;
    void printRadii(const char* label = "Radii") const;
    void printRotation(const char* label = "Rotation") const;
    
private:
    Eigen::Matrix<double,10,10> S_;
    Eigen::Vector3d center_;
    Eigen::Vector3d radii_;
    Eigen::Matrix3d rotation_;
    
    // Helper functions
    void printVector(const Eigen::Vector3d& v, const char* label) const;
    void printMatrix(const Eigen::Matrix3d& M, const char* label) const;
};


class RotationFinder {
    
public:
    Eigen::Matrix<double, 9, 9> G;
    Eigen::Matrix<double, 9, 1> h;
    Eigen::Matrix3d R;

    RotationFinder();
    
    // Add a single mag/acc pair to the accumulator
    void addSample(const Eigen::Vector3d& mag, const Eigen::Vector3d& acc);
    
    // Solve for rotation matrix from accumulated data
    void solve();
    
};

class AffineFinder {

public:
    // Accumulator matrix and vector
    Eigen::Matrix<double, 12, 12> G;
    Eigen::Matrix<double, 12, 1> h;
    double target;

    // Affine transform x = M * m + c
    Eigen::Matrix3d M;
    Eigen::Vector3d c;
    bool solved;

    AffineFinder(double target_);

    // Reset accumulators and affine transform
    void reset();


    // load previous solution
    void load(const Eigen::Matrix3d &_M, const Eigen::Vector3d &_c);

    // Add a single mag/acc pair to the accumulator
    void addSample(const Eigen::Vector3d& mag, const Eigen::Vector3d& acc);

    // Solve for affine transform from accumulated data
    bool solve();

    // Apply affine transform to a magnetometer vector
    Eigen::Vector3d transform(const Eigen::Vector3d& mag) const
    {
        return M * mag + c;
    }
    

};

#endif // ONLINE_CALIBRATION_H