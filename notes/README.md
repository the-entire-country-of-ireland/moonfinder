# ESP32 Sectorized Magnetometer Calibration

This package contains a compact C++/Eigen implementation of a sectorized, limited-memory magnetometer/accelerometer calibration routine.

The model is

```math
y = W \begin{bmatrix}m\\1\end{bmatrix} = A m + t = A(m-b), \qquad b=-A^{-1}t.
```

The implementation is written to build with ArduinoEigen on ESP32 and with Eigen on a host machine.

## Main idea

During each sector, the user slowly pitches the sensor along one approximately fixed mechanical axis. The accelerometer determines the sector axis `n_s` and scalar pitch coordinate `theta_s,k`. For a calibrated magnetometer, the finite-rotation condition is

```math
R(n_s,-\theta_{s,k}) W \bar m_{s,k} \approx q_s.
```

Eliminating the unknown sector vector `q_s` gives a quadratic form

```math
L_pitch = w^T G_pitch w,
```

where `w = vec(W)`.

The unit-norm objective is also stored without samples. Define

```math
Q = W^T W,
```

then

```math
||W\bar m_k||^2 - 1 = \bar m_k^T Q \bar m_k - 1.
```

This is linear in `q = vech(Q)`, so the stream stores

```math
G_norm = sum a_k a_k^T,  h_norm = sum a_k,
```

and evaluates

```math
L_norm = q^T G_norm q - 2 h_norm^T q + N.
```

The constant accel/mag dot objective is exactly quadratic in `[w;c]`:

```math
L_dot = sum_k (\hat a_k^T W\bar m_k - c)^2.
```

## Memory behavior

No samples are retained across sectors. The class stores only:

- `G_pitch`: 12 x 12
- `G_norm`: 10 x 10
- `h_norm`: 10
- `G_dot`: 13 x 13
- an ellipsoid normal matrix for initialization: 10 x 10
- min/max magnetometer values for fallback initialization
- a bounded active-sector scratch buffer, discarded at `end_sector()`

The active-sector scratch buffer is required when `n_s` and `theta_s,k` must be estimated from the accelerometer data after the sector. If an external system provides `n_s` and `theta_s,k` online, the same `B` accumulators can be updated without staging the sector.

## API

```cpp
using Cal = sector_calib::SectorCalibrator<float>;
Cal calib;

calib.start_new_sector();
calib.add_sample(mag, acc);
calib.end_sector();

Cal::SolveOptions opts;
auto result = calib.solve(opts);
```

`mag` and `acc` are `Eigen::Matrix<float,3,1>` or `double` equivalents, depending on template type.

## Host test

Build with a local Eigen installation. In this container the Eigen headers came from CasADi:

```bash
clang++ -std=c++17 -O0 \
  -I/opt/pyvenv/lib/python3.13/site-packages/casadi/include/eigen3 \
  test_sector_calibrator.cpp -o test_sector_calibrator

./test_sector_calibrator /mnt/data/pitch_readings.txt
```

The test parses `NEW_MEASUREMENTS` blocks from `pitch_readings.txt`, uses only mag/acc columns, finalizes each sector, and solves the streamed objective.

## Files

- `sector_calibrator.h` — header-only C++ implementation
- `test_sector_calibrator.cpp` — host test using `pitch_readings.txt`
- `python_reference_streaming_quadratic.py` — Python reference for the exact same streamed quadratic/GN objective
- `cpp_test_output.txt` — C++ output from the sample data
- `python_reference_streaming_quadratic_output.txt` — Python reference output
- `esp32_example.ino` — minimal Arduino sketch skeleton
