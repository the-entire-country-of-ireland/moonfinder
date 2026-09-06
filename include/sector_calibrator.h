#pragma once

// Sectorized limited-memory magnetometer/accelerometer calibration.
// Designed for ESP32 + ArduinoEigen, but also builds on a host with Eigen.
//
// Model:
//   y = W [m; 1] = A*m + t = A*(m - b), with b = -A^{-1}t.
//
// Stored objectives, all fixed-size after sector finalization:
//   1) sector finite-rotation/pitch objective:       0.5 w^T G_pitch w
//   2) unit-norm objective via Q = W^T W:            0.5 sum_k (mbar_k^T Q mbar_k - 1)^2
//   3) constant accel-mag dot objective:             0.5 sum_k (ahat_k^T W mbar_k - c)^2
//   4) optional prior/regularization on W.
//
// No samples are retained across sectors. The active sector is temporarily staged so that
// n_s and theta_s,k can be estimated from accelerometer data before forming G_pitch.
// If n_s/theta_s,k are supplied externally, the same B accumulators can be updated online.

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <ArduinoEigenDense.h>
#else
  #include <Eigen/Dense>
#endif

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "sd_card.h"

namespace sector_calib {

template <typename Scalar>
class SectorCalibrator {
public:
  using Vec3    = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4    = Eigen::Matrix<Scalar, 4, 1>;
  using Vec10   = Eigen::Matrix<Scalar, 10, 1>;
  using Vec12   = Eigen::Matrix<Scalar, 12, 1>;
  using Vec13   = Eigen::Matrix<Scalar, 13, 1>;
  using Mat3    = Eigen::Matrix<Scalar, 3, 3>;
  using Mat34   = Eigen::Matrix<Scalar, 3, 4>;
  using Mat3x12 = Eigen::Matrix<Scalar, 3, 12>;
  using Mat10   = Eigen::Matrix<Scalar, 10, 10>;
  using Mat10x12= Eigen::Matrix<Scalar, 10, 12>;
  using Mat12   = Eigen::Matrix<Scalar, 12, 12>;
  using Mat13   = Eigen::Matrix<Scalar, 13, 13>;

  struct Sample {
    Vec3 mag;
    Vec3 acc;
    Vec3 gyro;
    Scalar dt;
    Vec3 acc_unit;
  };

  struct Options {
    int max_sector_samples = 500;       // bounded active-sector scratch only
    Scalar min_acc_norm = Scalar(9.5);
    Scalar max_acc_norm = Scalar(10.1);
  };

  struct SolveOptions {
    Scalar pitch_weight = Scalar(1.0);
    Scalar norm_weight  = Scalar(1.0);
    Scalar dot_weight   = Scalar(1.0);
    Scalar prior_weight = Scalar(1e-8);  // weak anchor to initializer; set 0 to disable
    Scalar lm_lambda0   = Scalar(1e-3);
    Scalar lm_up        = Scalar(10.0);
    Scalar lm_down      = Scalar(0.35);
    int max_iters       = 60;
    Scalar step_tol     = Scalar(1e-7);
    Scalar cost_tol     = Scalar(1e-10);
  };

  struct SectorInfo {
    int n = 0;
    Vec3 axis = Vec3::Zero();
    Scalar acc_rms = 0;
    Scalar span_rad = 0;
  };

  struct Result {
    Mat34 W = Mat34::Zero();
    Mat3 A = Mat3::Zero();
    Vec3 bias = Vec3::Zero();
    Vec3 t = Vec3::Zero();
    Scalar c = 0;
    Scalar cost = 0;
    int iterations = 0;
    bool converged = false;
  };

  explicit SectorCalibrator(const Options& opt = Options()) : opt_(opt) { reset(); }

  void reset() {
    G_pitch_.setZero();
    G_norm_.setZero();
    h_norm_.setZero();
    G_dot_.setZero();
    ellipsoid_N_.setZero();
    current_.clear();
    current_.reserve(opt_.max_sector_samples);
    sectors_.clear();
    sectors_.reserve(32);
    sample_count_ = 0;
    norm_count_ = 0;
    in_sector_ = false;
    have_minmax_ = false;
    initialized_ = false;
    W_prior_.setZero();
    W_current_.setZero();
    c_current_ = 0;
  }

  void beginSector() {
    current_.clear();
    current_.reserve(opt_.max_sector_samples);
    in_sector_ = true;
  }

  // acc is normalized internally.
  bool recordSample(const Vec3& mag, const Vec3& acc) {
    return recordSample(mag, acc, Vec3::Zero(), Scalar(0));
  }

  bool recordSample(const Vec3& mag, const Vec3& acc,
                    const Vec3& gyro, Scalar dt) {
    Scalar an = acc.norm();
    if ((an < opt_.min_acc_norm) || (an > opt_.max_acc_norm) || !isFinite(mag) || !isFinite(acc)) return false;

    Vec3 ahat = acc / an;

    // Direct streaming accumulators.
    Vec10 aq = normFeature(mag);
    G_norm_ += aq * aq.transpose();
    h_norm_ += aq;
    ++norm_count_;

    Vec13 dd = Vec13::Zero();
    dd.template head<12>() = dotFeature(mag, ahat);
    dd(12) = Scalar(-1);
    G_dot_ += dd * dd.transpose();

    Vec10 e = ellipsoidFeature(mag);
    ellipsoid_N_ += e * e.transpose();

    if (!have_minmax_) {
      mag_min_ = mag;
      mag_max_ = mag;
      have_minmax_ = true;
    } else {
      mag_min_ = mag_min_.cwiseMin(mag);
      mag_max_ = mag_max_.cwiseMax(mag);
    }

    if (!in_sector_) return true;

    // Active-sector scratch. This is discarded when the sector is finished.
    if ((int)current_.size() < opt_.max_sector_samples) {
      current_.push_back(Sample{mag, acc, gyro, dt, ahat});
    }
    ++sample_count_;
    return true;
  }

  bool isSectorComplete() const {
    if (!in_sector_) return false;
    return (int)current_.size() == opt_.max_sector_samples;
  }

  bool isCollectingSectorSamples() const {
    return in_sector_;
  }

  int currentSectorSampleCount() const {
    if (!in_sector_) return 0;
    return (int) current_.size();

  }

  // Fits sector axis/theta from active-sector accel data, then updates the global pitch quadratic.
  bool finishSector() {
    in_sector_ = false;
    const int K = (int)current_.size();
    if (K < 50) {
      current_.clear();
      return false;
    }

    Mat3 S = Mat3::Zero();
    for (const auto& s : current_) S += s.acc_unit * s.acc_unit.transpose();

    Eigen::SelfAdjointEigenSolver<Mat3> es(S);
    if (es.info() != Eigen::Success) {
      current_.clear();
      return false;
    }

    // Smallest eigenvector is the normal to the accelerometer great-circle plane.
    Vec3 n = es.eigenvectors().col(0).normalized();

    // Deterministic sign convention.
    int imax = 0;
    if (std::abs(n(1)) > std::abs(n(imax))) imax = 1;
    if (std::abs(n(2)) > std::abs(n(imax))) imax = 2;
    if (n(imax) < Scalar(0)) n = -n;

    // Phase reference from first projected accelerometer sample. The zero is arbitrary.
    Vec3 e1 = current_[0].acc_unit - n * (n.dot(current_[0].acc_unit));
    if (e1.norm() < Scalar(1e-6)) {
      e1 = es.eigenvectors().col(2);
      e1 -= n * n.dot(e1);
    }
    e1.normalize();
    Vec3 e2 = n.cross(e1).normalized();

    Mat3x12 S_B = Mat3x12::Zero();
    Mat12 S_BB = Mat12::Zero();
    Scalar last_theta = 0;
    bool have_last = false;
    Scalar theta_min = std::numeric_limits<Scalar>::infinity();
    Scalar theta_max = -std::numeric_limits<Scalar>::infinity();
    Scalar acc_rss = 0;
    int used = 0;

    for (const auto& s : current_) {
      Scalar res = n.dot(s.acc_unit);
      acc_rss += res * res;

      Vec3 p = s.acc_unit - n * res;
      Scalar pn = p.norm();
      if (pn <= Scalar(1e-9)) continue;
      p /= pn;

      Scalar theta = std::atan2(p.dot(e2), p.dot(e1));
      if (have_last) theta = unwrapNear(theta, last_theta);
      have_last = true;
      last_theta = theta;
      theta_min = std::min(theta_min, theta);
      theta_max = std::max(theta_max, theta);

      Mat3 U = rodrigues(n, -theta);
      Mat3x12 B = U * magnetometerDesignMatrix(s.mag);
      S_B += B;
      S_BB += B.transpose() * B;
      ++used;
    }

    if (used < 50) {
      current_.clear();
      return false;
    }

    Mat12 Gs = S_BB - (S_B.transpose() * S_B) / Scalar(used);
    G_pitch_ += Scalar(0.5) * (Gs + Gs.transpose());

    SectorInfo info;
    info.n = used;
    info.axis = n;
    info.acc_rms = std::sqrt(acc_rss / Scalar(K));
    info.span_rad = theta_max - theta_min;
    sectors_.push_back(info);
    current_.clear();
    return true;
  }

  const Mat12& pitch_quadratic() const { return G_pitch_; }
  const Mat10& norm_quadratic() const { return G_norm_; }
  const Vec10& norm_linear() const { return h_norm_; }
  const Mat13& dot_quadratic() const { return G_dot_; }
  const std::vector<SectorInfo>& sectors() const { return sectors_; }
  int totalAcceptedSampleCount() const { return sample_count_; }

  Mat34 initialTransformFromBounds() const {
    Mat34 W = Mat34::Zero();
    if (!have_minmax_) {
      W.template block<3,3>(0,0).setIdentity();
      return W;
    }
    Vec3 b = Scalar(0.5) * (mag_min_ + mag_max_);
    Vec3 scale;
    for (int i = 0; i < 3; ++i) {
      scale(i) = Scalar(2.0) / std::max(mag_max_(i) - mag_min_(i), Scalar(1e-9));
    }
    Mat3 A = scale.asDiagonal();
    W.template block<3,3>(0,0) = A;
    W.col(3) = -A * b;
    return W;
  }

  Mat34 initialTransformFromEllipsoid() const {
    if (norm_count_ < 20) return initialTransformFromBounds();
    Eigen::SelfAdjointEigenSolver<Mat10> es(ellipsoid_N_);
    if (es.info() != Eigen::Success) return initialTransformFromBounds();
    Vec10 beta = es.eigenvectors().col(0);

    for (int sgn_i = 0; sgn_i < 2; ++sgn_i) {
      Scalar sgn = (sgn_i == 0) ? Scalar(1) : Scalar(-1);
      Vec10 bta = sgn * beta;
      Mat3 Q;
      Q << bta(0), bta(3), bta(4),
           bta(3), bta(1), bta(5),
           bta(4), bta(5), bta(2);
      Vec3 u = bta.template segment<3>(6);
      Scalar k = bta(9);
      Eigen::FullPivLU<Mat3> lu(Q);
      if (!lu.isInvertible()) continue;
      Vec3 center = -Scalar(0.5) * lu.solve(u);
      Scalar r = (center.transpose() * Q * center)(0) - k;
      if (!(r > Scalar(1e-12))) continue;
      Mat3 C = Q / r;
      C = Scalar(0.5) * (C + C.transpose());
      Eigen::SelfAdjointEigenSolver<Mat3> ce(C);
      if (ce.info() != Eigen::Success) continue;
      if (ce.eigenvalues().minCoeff() <= Scalar(1e-9)) continue;
      Mat3 A = ce.eigenvectors() * ce.eigenvalues().cwiseSqrt().asDiagonal() * ce.eigenvectors().transpose();
      Mat34 W = Mat34::Zero();
      W.template block<3,3>(0,0) = A;
      W.col(3) = -A * center;
      return W;
    }
    return initialTransformFromBounds();
  }

  Result solve(const SolveOptions& so = SolveOptions(), bool use_ellipsoid_init = true) {
    Result out;
    if (!initialized_) {
      W_current_ = use_ellipsoid_init ? initialTransformFromEllipsoid()
                  : initialTransformFromBounds();
      W_prior_ = W_current_;
      c_current_ = initialDot(W_current_);
      initialized_ = true;
    }

    Vec13 x = packTransform(W_current_, c_current_);
    Scalar lambda = so.lm_lambda0;
    Scalar prev_cost = cost(x, so);

    for (int iter = 0; iter < so.max_iters; ++iter) {
      Mat13 H = Mat13::Zero();
      Vec13 g = Vec13::Zero();
      accumulateGaussNewton(x, so, H, g);

      Mat13 H_lm = H;
      for (int i = 0; i < 13; ++i) H_lm(i,i) += lambda * (std::abs(H(i,i)) + Scalar(1e-6));

      Eigen::ColPivHouseholderQR<Mat13> qr(H_lm);
      Vec13 dx = qr.solve(-g);
      if (!isFinite(dx)) break;

      Vec13 trial = x + dx;
      trial(12) = clamp(trial(12), Scalar(-2.0), Scalar(2.0));
      Scalar trial_cost = cost(trial, so);

      if (std::isfinite((double)trial_cost) && trial_cost < prev_cost) {
        Scalar rel_dec = (prev_cost - trial_cost) / std::max(prev_cost, Scalar(1));
        x = trial;
        prev_cost = trial_cost;
        lambda = std::max(lambda * so.lm_down, Scalar(1e-12));
        out.iterations = iter + 1;
        if (dx.norm() < so.step_tol || rel_dec < so.cost_tol) {
          out.converged = true;
          break;
        }
      } else {
        lambda *= so.lm_up;
      }
    }

    unpackTransform(x, W_current_, c_current_);
    out.W = W_current_;
    out.A = W_current_.template block<3,3>(0,0);
    out.t = W_current_.col(3);
    out.bias = -out.A.fullPivLu().solve(out.t);
    out.c = c_current_;
    out.cost = prev_cost;
    return out;
  }

  Scalar diagnosticNormSquaredMean(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    Vec10 q = qFromTransform(W);
    return (h_norm_.dot(q)) / Scalar(norm_count_);
  }

  Scalar diagnosticNormSquaredStddev(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    Vec10 q = qFromTransform(W);
    Scalar sum = h_norm_.dot(q);
    Scalar sum2 = (q.transpose() * G_norm_ * q)(0);
    Scalar mean = sum / Scalar(norm_count_);
    return std::sqrt(std::max(sum2 / Scalar(norm_count_) - mean*mean, Scalar(0)));
  }

  Scalar diagnosticNormSquaredRmsResidual(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    Vec10 q = qFromTransform(W);
    Scalar ss = (q.transpose() * G_norm_ * q)(0) - Scalar(2) * h_norm_.dot(q) + Scalar(norm_count_);
    return std::sqrt(std::max(ss / Scalar(norm_count_), Scalar(0)));
  }

  Scalar diagnosticDotMean(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    Vec12 w = vectorizeTransform(W);
    Vec12 sd = -G_dot_.template block<12,1>(0,12);
    return sd.dot(w) / Scalar(norm_count_);
  }

  Scalar diagnosticDotStddev(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    Vec12 w = vectorizeTransform(W);
    Vec12 sd = -G_dot_.template block<12,1>(0,12);
    Scalar sum = sd.dot(w);
    Scalar sum2 = (w.transpose() * G_dot_.template block<12,12>(0,0) * w)(0);
    Scalar mean = sum / Scalar(norm_count_);
    return std::sqrt(std::max(sum2 / Scalar(norm_count_) - mean*mean, Scalar(0)));
  }

  Scalar diagnosticDotRmsResidual(const Mat34& W, Scalar c) const {
    if (norm_count_ <= 0) return 0;
    Vec13 x = packTransform(W, c);
    Scalar ss = (x.transpose() * G_dot_ * x)(0);
    return std::sqrt(std::max(ss / Scalar(norm_count_), Scalar(0)));
  }

  bool writeSectorSamplesToFile(SDCard& sdCard) const {
    if (!sdCard.isAvailable()) return false;

#if defined(ARDUINO)
    File file = SD.open(sdCard.getCurrentFilename(), FILE_APPEND);
    if (!file) {
      Serial.println("ERROR: Failed to open sector file for append");
      return false;
    }

    char buffer[128];
    size_t written_samples = 0;
    for (const auto& s : current_) {
      int length = snprintf(
        buffer,
        sizeof(buffer),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f\n",
        s.mag[0], s.mag[1], s.mag[2],
        s.acc[0], s.acc[1], s.acc[2],
        s.gyro[0], s.gyro[1], s.gyro[2], s.dt
      );

      if (length <= 0 || length >= (int)sizeof(buffer) ||
          file.write((const uint8_t*)buffer, length) != (size_t)length) {
        file.close();
        Serial.println("ERROR: Failed to write sector sample");
        return false;
      }

      ++written_samples;
      if ((written_samples & 31) == 0) yield();
    }

    file.flush();
    file.close();
    return true;
#else
    return false;
#endif
  }

private:
  Options opt_;
  Mat12 G_pitch_;
  Mat10 G_norm_;
  Vec10 h_norm_;
  Mat13 G_dot_;
  Mat10 ellipsoid_N_;
  std::vector<Sample> current_;
  std::vector<SectorInfo> sectors_;
  int sample_count_ = 0;
  int norm_count_ = 0;
  bool in_sector_ = false;
  bool have_minmax_ = false;
  Vec3 mag_min_ = Vec3::Zero();
  Vec3 mag_max_ = Vec3::Zero();
  bool initialized_ = false;
  Mat34 W_prior_;
  Mat34 W_current_;
  Scalar c_current_ = 0;

  static bool isFinite(const Vec3& v) {
    return std::isfinite((double)v(0)) && std::isfinite((double)v(1)) && std::isfinite((double)v(2));
  }
  static bool isFinite(const Vec13& v) {
    for (int i = 0; i < 13; ++i) if (!std::isfinite((double)v(i))) return false;
    return true;
  }
  static Scalar clamp(Scalar x, Scalar lo, Scalar hi) {
    return std::max(lo, std::min(hi, x));
  }

  static Scalar unwrapNear(Scalar theta, Scalar ref) {
    const Scalar two_pi = Scalar(6.2831853071795864769);
    while (theta - ref >  Scalar(3.14159265358979323846)) theta -= two_pi;
    while (theta - ref < -Scalar(3.14159265358979323846)) theta += two_pi;
    return theta;
  }

  static Mat3 rodrigues(const Vec3& axis, Scalar angle) {
    Vec3 n = axis.normalized();
    Mat3 K;
    K << Scalar(0), -n(2), n(1),
         n(2), Scalar(0), -n(0),
        -n(1), n(0), Scalar(0);
    Mat3 I = Mat3::Identity();
    return I * std::cos(angle) + K * std::sin(angle) + (n * n.transpose()) * (Scalar(1) - std::cos(angle));
  }

  static Mat3x12 magnetometerDesignMatrix(const Vec3& m) {
    Mat3x12 M = Mat3x12::Zero();
    M.template block<3,3>(0,0) = m(0) * Mat3::Identity();
    M.template block<3,3>(0,3) = m(1) * Mat3::Identity();
    M.template block<3,3>(0,6) = m(2) * Mat3::Identity();
    M.template block<3,3>(0,9) = Mat3::Identity();
    return M;
  }

  static Vec10 normFeature(const Vec3& m) {
    Scalar x = m(0), y = m(1), z = m(2), one = Scalar(1);
    Vec10 a;
    a << x*x, y*y, z*z, one,
         Scalar(2)*x*y, Scalar(2)*x*z, Scalar(2)*x,
         Scalar(2)*y*z, Scalar(2)*y,
         Scalar(2)*z;
    return a;
  }

  static Vec10 ellipsoidFeature(const Vec3& m) {
    Scalar x = m(0), y = m(1), z = m(2);
    Vec10 a;
    a << x*x, y*y, z*z, Scalar(2)*x*y, Scalar(2)*x*z, Scalar(2)*y*z, x, y, z, Scalar(1);
    return a;
  }

  static Vec12 dotFeature(const Vec3& m, const Vec3& ahat) {
    Vec12 d;
    d.template segment<3>(0) = m(0) * ahat;
    d.template segment<3>(3) = m(1) * ahat;
    d.template segment<3>(6) = m(2) * ahat;
    d.template segment<3>(9) = ahat;
    return d;
  }

  static Vec12 vectorizeTransform(const Mat34& W) {
    Vec12 w;
    int k = 0;
    for (int c = 0; c < 4; ++c)
      for (int r = 0; r < 3; ++r)
        w(k++) = W(r,c);
    return w;
  }

  static Mat34 transformFromVector(const Vec12& w) {
    Mat34 W;
    int k = 0;
    for (int c = 0; c < 4; ++c)
      for (int r = 0; r < 3; ++r)
        W(r,c) = w(k++);
    return W;
  }

  static Vec13 packTransform(const Mat34& W, Scalar c) {
    Vec13 x;
    x.template head<12>() = vectorizeTransform(W);
    x(12) = c;
    return x;
  }

  static void unpackTransform(const Vec13& x, Mat34& W, Scalar& c) {
    W = transformFromVector(x.template head<12>());
    c = x(12);
  }

  static Vec10 qFromTransform(const Mat34& W) {
    Eigen::Matrix<Scalar,4,4> Q = W.transpose() * W;
    Vec10 q;
    q << Q(0,0), Q(1,1), Q(2,2), Q(3,3),
         Q(0,1), Q(0,2), Q(0,3), Q(1,2), Q(1,3), Q(2,3);
    return q;
  }

  static Mat10x12 jacobianQFromTransform(const Mat34& W) {
    Mat10x12 J = Mat10x12::Zero();
    Vec3 c0 = W.col(0), c1 = W.col(1), c2 = W.col(2), c3 = W.col(3);
    J.template block<1,3>(0,0) = (Scalar(2)*c0).transpose();
    J.template block<1,3>(1,3) = (Scalar(2)*c1).transpose();
    J.template block<1,3>(2,6) = (Scalar(2)*c2).transpose();
    J.template block<1,3>(3,9) = (Scalar(2)*c3).transpose();

    J.template block<1,3>(4,0) = c1.transpose();
    J.template block<1,3>(4,3) = c0.transpose();
    J.template block<1,3>(5,0) = c2.transpose();
    J.template block<1,3>(5,6) = c0.transpose();
    J.template block<1,3>(6,0) = c3.transpose();
    J.template block<1,3>(6,9) = c0.transpose();
    J.template block<1,3>(7,3) = c2.transpose();
    J.template block<1,3>(7,6) = c1.transpose();
    J.template block<1,3>(8,3) = c3.transpose();
    J.template block<1,3>(8,9) = c1.transpose();
    J.template block<1,3>(9,6) = c3.transpose();
    J.template block<1,3>(9,9) = c2.transpose();
    return J;
  }

  Scalar initialDot(const Mat34& W) const {
    if (norm_count_ <= 0) return 0;
    return clamp(diagnosticDotMean(W), Scalar(-1.5), Scalar(1.5));
  }

  Scalar cost(const Vec13& x, const SolveOptions& so) const {
    Vec12 w = x.template head<12>();
    Scalar C = Scalar(0.5) * so.pitch_weight * (w.transpose() * G_pitch_ * w)(0);

    Mat34 W = transformFromVector(w);
    Vec10 q = qFromTransform(W);
    Scalar norm_ss = (q.transpose() * G_norm_ * q)(0) - Scalar(2) * h_norm_.dot(q) + Scalar(norm_count_);
    C += Scalar(0.5) * so.norm_weight * norm_ss;

    C += Scalar(0.5) * so.dot_weight * (x.transpose() * G_dot_ * x)(0);

    if (so.prior_weight > Scalar(0)) {
      Vec12 wp = vectorizeTransform(W_prior_);
      Vec12 d = w - wp;
      C += Scalar(0.5) * so.prior_weight * d.squaredNorm();
    }
    return C;
  }

  void accumulateGaussNewton(const Vec13& x, const SolveOptions& so, Mat13& H, Vec13& g) const {
    Vec12 w = x.template head<12>();
    Mat34 W = transformFromVector(w);

    // Exact pitch quadratic: 0.5*w^T G*w.
    H.template block<12,12>(0,0) += so.pitch_weight * G_pitch_;
    g.template head<12>() += so.pitch_weight * (G_pitch_ * w);

    // Exact dot quadratic: 0.5*x^T G_dot*x.
    H += so.dot_weight * G_dot_;
    g += so.dot_weight * (G_dot_ * x);

    // Norm residual is linear in q=vech(W^T W), then q is quadratic in W.
    // GN Hessian uses J_q^T G_q J_q; gradient is exact first derivative J_q^T(G_q q - h_q).
    Vec10 q = qFromTransform(W);
    Mat10x12 Jq = jacobianQFromTransform(W);
    Vec10 sq = G_norm_ * q - h_norm_;
    H.template block<12,12>(0,0) += so.norm_weight * (Jq.transpose() * G_norm_ * Jq);
    g.template head<12>() += so.norm_weight * (Jq.transpose() * sq);

    if (so.prior_weight > Scalar(0)) {
      Vec12 wp = vectorizeTransform(W_prior_);
      H.template block<12,12>(0,0) += so.prior_weight * Mat12::Identity();
      g.template head<12>() += so.prior_weight * (w - wp);
    }
  }
};

} // namespace sector_calib
