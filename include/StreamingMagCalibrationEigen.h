#pragma once

#ifdef ARDUINO
#include <ArduinoEigen.h>
#else
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#endif

#include <math.h>

template <typename Scalar = double>
class StreamingMagCalibration {
 public:
  using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vec4 = Eigen::Matrix<Scalar, 4, 1>;
  using Vec10 = Eigen::Matrix<Scalar, 10, 1>;
  using Vec12 = Eigen::Matrix<Scalar, 12, 1>;
  using Mat10 = Eigen::Matrix<Scalar, 10, 10>;
  using Mat12 = Eigen::Matrix<Scalar, 12, 12>;
  using Mat10x12 = Eigen::Matrix<Scalar, 10, 12>;
  using Mat34 = Eigen::Matrix<Scalar, 3, 4, Eigen::RowMajor>;

  enum class Presolve { LDLT, Spectral };
  enum class Status {
    Success,
    NoData,
    RankDeficient,
    NoDescent,
    MaxIterations
  };

  struct Options {
    Scalar alpha = Scalar(1);
    Scalar beta = Scalar(1);
    Scalar damping = Scalar(1e-6);
    Scalar gradient_tol = Scalar(1e-6);
    Scalar step_tol = Scalar(1e-7);
    Scalar rank_tol = Scalar(1e-7);
    int max_iterations = 30;
    int max_damping_trials = 15;
    Presolve presolve = Presolve::LDLT;
  };

  struct Result {
    Status status = Status::NoData;
    Mat34 W = Mat34::Zero();
    Scalar objective = Scalar(0);
    Scalar gradient_inf = Scalar(0);
    int iterations = 0;
    bool ok() const { return status == Status::Success; }
  };

  explicit StreamingMagCalibration(Scalar target_dot) : T_(target_dot) {
    reset();
  }

  void reset() {
    Ha_.setZero();
    ga_.setZero();
    Hn_.setZero();
    gn_.setZero();
    ca_ = cn_ = weight_sum_ = Scalar(0);
  }

  bool update(const Vec3& mag_raw, const Vec3& acc_raw,
              Scalar weight = Scalar(1)) {
    const Scalar an = acc_raw.norm();
    if (!(weight > Scalar(0)) || !(an > Scalar(0)) || !finite(an)) return false;

    const Vec3 a = acc_raw / an;
    Vec4 x;
    x.template head<3>() = mag_raw;
    x(3) = Scalar(1);

    Vec12 z;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c) z(4 * r + c) = a(r) * x(c);

    const Vec10 phi = quadraticFeatures(x);
    Ha_.noalias() += weight * (z * z.transpose());
    ga_.noalias() += weight * T_ * z;
    ca_ += weight * T_ * T_;
    Hn_.noalias() += weight * (phi * phi.transpose());
    gn_.noalias() += weight * phi;
    cn_ += weight;
    weight_sum_ += weight;
    return true;
  }

  Result solve() const { return solve(Options()); }

  Result solve(const Options& opt) const {
    Result out;
    if (!(weight_sum_ > Scalar(0))) return out;

    Vec12 w;
    if (!initialize(w, opt)) {
      out.status = Status::RankDeficient;
      return out;
    }

    Scalar f;
    Vec12 g;
    Mat12 H;
    model(w, opt, f, g, H);
    Scalar lambda = opt.damping;

    for (int iter = 0; iter < opt.max_iterations; ++iter) {
      out.iterations = iter;
      const Scalar ginf = infNorm(g);
      if (ginf <= opt.gradient_tol) {
        finish(out, Status::Success, w, f, ginf, iter);
        return out;
      }

      Vec12 d = H.diagonal().cwiseAbs();
      for (int i = 0; i < 12; ++i)
        if (d(i) < Scalar(1e-8)) d(i) = Scalar(1e-8);

      bool accepted = false;
      Vec12 step = Vec12::Zero();
      for (int trial = 0; trial < opt.max_damping_trials; ++trial) {
        Mat12 Hd = H;
        for (int i = 0; i < 12; ++i) Hd(i, i) += lambda * d(i);

        Eigen::LDLT<Mat12> ldlt(Hd);
        if (ldlt.info() != Eigen::Success) {
          lambda *= Scalar(10);
          continue;
        }
        step = ldlt.solve(-g);
        if (ldlt.info() != Eigen::Success || !finiteVector(step)) {
          lambda *= Scalar(10);
          continue;
        }

        Scalar f_new;
        Vec12 g_new;
        Mat12 H_new;
        model(w + step, opt, f_new, g_new, H_new);
        if (finite(f_new) && f_new < f) {
          w += step;
          f = f_new;
          g = g_new;
          H = H_new;
          lambda /= Scalar(3);
          if (lambda < Scalar(1e-12)) lambda = Scalar(1e-12);
          accepted = true;
          break;
        }
        lambda *= Scalar(10);
      }

      if (!accepted) {
        finish(out, Status::NoDescent, w, f, infNorm(g), iter);
        return out;
      }

      if (step.norm() <= opt.step_tol * (Scalar(1) + w.norm())) {
        finish(out, Status::Success, w, f, infNorm(g), iter + 1);
        return out;
      }
    }

    finish(out, Status::MaxIterations, w, f, infNorm(g), opt.max_iterations);
    return out;
  }

  static Vec3 transform(const Mat34& W, const Vec3& mag_raw) {
    Vec4 x;
    x.template head<3>() = mag_raw;
    x(3) = Scalar(1);
    return W * x;
  }

 private:
  Mat12 Ha_;
  Vec12 ga_;
  Mat10 Hn_;
  Vec10 gn_;
  Scalar ca_, cn_, weight_sum_, T_;

  static bool finite(Scalar x) { return ::isfinite(static_cast<double>(x)); }

  static bool finiteVector(const Vec12& x) {
    for (int i = 0; i < 12; ++i)
      if (!finite(x(i))) return false;
    return true;
  }

  static Scalar infNorm(const Vec12& x) {
    Scalar v = Scalar(0);
    for (int i = 0; i < 12; ++i) {
      const Scalar a = x(i) < Scalar(0) ? -x(i) : x(i);
      if (a > v) v = a;
    }
    return v;
  }

  static void pairAt(int k, int& i, int& j) {
    switch (k) {
      case 0: i = 0; j = 0; break;
      case 1: i = 1; j = 1; break;
      case 2: i = 2; j = 2; break;
      case 3: i = 3; j = 3; break;
      case 4: i = 0; j = 1; break;
      case 5: i = 0; j = 2; break;
      case 6: i = 0; j = 3; break;
      case 7: i = 1; j = 2; break;
      case 8: i = 1; j = 3; break;
      default: i = 2; j = 3; break;
    }
  }

  static Vec10 quadraticFeatures(const Vec4& x) {
    Vec10 phi;
    for (int k = 0; k < 10; ++k) {
      int i, j;
      pairAt(k, i, j);
      phi(k) = (i == j) ? x(i) * x(j) : Scalar(2) * x(i) * x(j);
    }
    return phi;
  }

  static void unpackW(const Vec12& w, Mat34& W) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c) W(r, c) = w(4 * r + c);
  }

  static void packGram(const Vec12& w, Vec10& q, Mat10x12& J) {
    Mat34 W;
    unpackW(w, W);
    const Eigen::Matrix<Scalar, 4, 4> Q = W.transpose() * W;
    J.setZero();

    for (int k = 0; k < 10; ++k) {
      int i, j;
      pairAt(k, i, j);
      q(k) = Q(i, j);
      for (int r = 0; r < 3; ++r) {
        const int wi = 4 * r + i;
        const int wj = 4 * r + j;
        if (i == j) {
          J(k, wi) = Scalar(2) * W(r, i);
        } else {
          J(k, wi) = W(r, j);
          J(k, wj) = W(r, i);
        }
      }
    }
  }

  bool initialize(Vec12& w, const Options& opt) const {
    if (opt.presolve == Presolve::Spectral) {
      Eigen::SelfAdjointEigenSolver<Mat12> es(Ha_);
      if (es.info() != Eigen::Success) return false;
      const Vec12 eval = es.eigenvalues();
      const Scalar largest = eval(11);
      if (!(largest > Scalar(0)) || eval(0) <= opt.rank_tol * largest) return false;
      Vec12 y = es.eigenvectors().transpose() * ga_;
      for (int i = 0; i < 12; ++i) y(i) /= eval(i);
      w = es.eigenvectors() * y;
      return finiteVector(w);
    }

    Eigen::LDLT<Mat12> ldlt(Ha_);
    if (ldlt.info() != Eigen::Success) return false;
    const Vec12 d = ldlt.vectorD().cwiseAbs();
    const Scalar largest = d.maxCoeff();
    if (!(largest > Scalar(0)) || d.minCoeff() <= opt.rank_tol * largest) return false;
    w = ldlt.solve(ga_);
    return ldlt.info() == Eigen::Success && finiteVector(w);
  }

  void model(const Vec12& w, const Options& opt, Scalar& f, Vec12& g,
             Mat12& H) const {
    Vec10 q;
    Mat10x12 J;
    packGram(w, q, J);

    const Scalar inv = Scalar(1) / weight_sum_;
    const Vec12 ra = Ha_ * w - ga_;
    const Vec10 rn = Hn_ * q - gn_;
    const Scalar fa = w.dot(Ha_ * w) - Scalar(2) * ga_.dot(w) + ca_;
    const Scalar fn = q.dot(Hn_ * q) - Scalar(2) * gn_.dot(q) + cn_;

    f = Scalar(0.5) * inv * (opt.alpha * fa + opt.beta * fn);
    g = inv * (opt.alpha * ra + opt.beta * J.transpose() * rn);
    H = inv * (opt.alpha * Ha_ + opt.beta * J.transpose() * Hn_ * J);
    H = (Scalar(0.5) * (H + H.transpose())).eval();
  }

  static void finish(Result& out, Status status, const Vec12& w, Scalar f,
                     Scalar ginf, int iterations) {
    out.status = status;
    unpackW(w, out.W);
    out.objective = f;
    out.gradient_inf = ginf;
    out.iterations = iterations;
  }
};
