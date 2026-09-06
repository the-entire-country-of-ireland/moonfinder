#include "sector_calibrator.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <cmath>

using Cal = sector_calib::SectorCalibrator<double>;

static bool parse_csv10(const std::string& line, double v[10]) {
  std::stringstream ss(line);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, ',') && i < 10) {
    try { v[i++] = std::stod(item); }
    catch (...) { return false; }
  }
  return i >= 10;
}

int main(int argc, char** argv) {
  const char* path = argc > 1 ? argv[1] : "/mnt/data/pitch_readings.txt";

  Cal::Options opt;
  opt.max_sector_samples = 1500;
  Cal calib(opt);

  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << "\n";
    return 2;
  }

  std::string line;
  bool have_sector = false;
  int sectors_seen = 0;
  while (std::getline(f, line)) {
    if (line.find("NEW_MEASUREMENTS") != std::string::npos) {
      if (have_sector) {
        calib.finishSector();
        ++sectors_seen;
      }
      calib.beginSector();
      have_sector = true;
      continue;
    }
    double v[10];
    if (!parse_csv10(line, v)) continue;
    Cal::Vec3 mag(v[0], v[1], v[2]);
    Cal::Vec3 acc(v[3], v[4], v[5]);
    calib.recordSample(mag, acc);
  }
  if (have_sector) {
    calib.finishSector();
    ++sectors_seen;
  }

  Cal::SolveOptions so;
  so.pitch_weight = 3.0;
  so.norm_weight = 3.0;
  so.dot_weight = 3.0;
  so.prior_weight = 1e-8;
  so.max_iters = 100;
  so.lm_lambda0 = 1e-3;

  auto result = calib.solve(so, true);

  std::cout << std::setprecision(10);
  std::cout << "sectors_seen " << sectors_seen
            << " accepted " << calib.sectors().size()
            << " total_samples " << calib.totalAcceptedSampleCount() << "\n";
  std::cout << "iterations " << result.iterations
            << " converged " << result.converged
            << " cost " << result.cost << "\n";
  std::cout << "A\n" << result.A << "\n";
  std::cout << "bias\n" << result.bias.transpose() << "\n";
  std::cout << "t\n" << result.t.transpose() << "\n";
  std::cout << "c " << result.c << "\n";
  std::cout << "norm2_mean " << calib.diagnosticNormSquaredMean(result.W)
            << " norm2_std " << calib.diagnosticNormSquaredStddev(result.W)
            << " norm2_rms_resid " << calib.diagnosticNormSquaredRmsResidual(result.W) << "\n";
  std::cout << "dot_mean " << calib.diagnosticDotMean(result.W)
            << " dot_std " << calib.diagnosticDotStddev(result.W)
            << " dot_rms_resid " << calib.diagnosticDotRmsResidual(result.W, result.c) << "\n";
  if (!calib.sectors().empty()) {
    std::cout << "first_axis " << calib.sectors()[0].axis.transpose()
              << " span_deg " << calib.sectors()[0].span_rad * 180.0 / M_PI
              << " acc_rms " << calib.sectors()[0].acc_rms << "\n";
  }
  return 0;
}
