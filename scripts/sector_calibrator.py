"""Python reference implementation of the streamed sector calibrator.

The implementation mirrors include/sector_calibrator.h. It supports:
- whole-sector batch ingestion with add_sector();
- streaming ingestion with begin_sector(), add_sample(), end_sector();
- one-call streaming boundaries with ingest(..., start_sector/end_sector);
- the same quadratic accumulators and damped Gauss-Newton solver.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np


Q_PAIRS = (
    (0, 0), (1, 1), (2, 2), (3, 3),
    (0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3),
)


@dataclass
class Options:
    max_sector_samples: int = 500
    min_acc_norm: float = 9.5
    max_acc_norm: float = 10.1


@dataclass
class SolveOptions:
    pitch_weight: float = 1.0
    perp_weight: float = 1.0
    norm_weight: float = 1.0
    dot_weight: float = 1.0
    prior_weight: float = 1e-8
    lm_lambda0: float = 1e-3
    lm_up: float = 10.0
    lm_down: float = 0.35
    max_iters: int = 60
    step_tol: float = 1e-7
    cost_tol: float = 1e-10


@dataclass
class SectorInfo:
    n: int
    axis: np.ndarray
    acc_rms: float
    span_rad: float


@dataclass
class Result:
    W: np.ndarray
    A: np.ndarray
    bias: np.ndarray
    t: np.ndarray
    c: float
    cost: float
    iterations: int
    converged: bool


def affine_transform(W, mag):
    """Apply a 3x4 affine calibration matrix to an (N, 3) array."""
    mag = np.asarray(mag, dtype=float)
    W = np.asarray(W, dtype=float)
    return mag @ W[:, :3].T + W[:, 3]


def save_calibration_transform(W, target, path=r"D:\data\calibration.txt"):
    """Save a calibration transform using the C++ SD-card file format."""
    W = np.asarray(W, dtype=float)
    if W.shape != (3, 4):
        raise ValueError("W must have shape (3, 4)")
    output_path = Path(path)
    lines = ["MOONFINDER_CALIBRATION_V1", f"target {float(target):.9g}"]
    lines.extend(
        " ".join(f"{float(value):.9g}" for value in row)
        for row in W
    )
    output_path.write_text("\n".join(lines) + "\n", encoding="ascii")
    return output_path


def sector_rotation_basis(acc):
    """Estimate the C++ sector axis, inverse rotations, and unit acceleration."""
    acc = np.asarray(acc, dtype=float)
    acc_unit = acc / np.linalg.norm(acc, axis=1, keepdims=True)
    scatter = acc_unit.T @ acc_unit
    _, eigenvectors = np.linalg.eigh(scatter)
    axis = eigenvectors[:, 0]
    axis /= np.linalg.norm(axis)
    axis_index = np.argmax(np.abs(axis))
    if axis[axis_index] < 0:
        axis = -axis
    e1 = acc_unit[0] - axis * np.dot(axis, acc_unit[0])
    if np.linalg.norm(e1) < 1e-6:
        e1 = eigenvectors[:, 2] - axis * np.dot(axis, eigenvectors[:, 2])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    theta = np.unwrap(np.arctan2(acc_unit @ e2, acc_unit @ e1))
    rotations = np.stack([_rodrigues(axis, -angle) for angle in theta])
    return acc_unit, axis, theta, rotations


def calibrated_sector_objectives(W, mag, acc, c=0.0):
    """Return per-sample norm/dot residuals and perpendicular vectors."""
    y = affine_transform(W, mag)
    acc_unit = np.asarray(acc, dtype=float)
    acc_unit /= np.linalg.norm(acc_unit, axis=1, keepdims=True)
    return {
        "y": y,
        "norm_residual": np.sum(y * y, axis=1) - 1.0,
        "dot_residual": np.sum(y * acc_unit, axis=1) - c,
        "perpendicular": y - np.sum(y * acc_unit, axis=1, keepdims=True) * acc_unit,
        "acc_unit": acc_unit,
    }


def sector_variances(W, mag, acc):
    """Return pitch and perpendicular centered variances for one sector."""
    values = calibrated_sector_objectives(W, mag, acc)
    _, _, _, rotations = sector_rotation_basis(acc)
    aligned = np.einsum("nij,nj->ni", rotations, values["y"])
    aligned_perpendicular = np.einsum(
        "nij,nj->ni", rotations, values["perpendicular"]
    )
    pitch_centered = aligned - np.mean(aligned, axis=0, keepdims=True)
    perp_centered = aligned_perpendicular - np.mean(
        aligned_perpendicular, axis=0, keepdims=True
    )
    return {
        "pitch": float(np.mean(np.sum(pitch_centered**2, axis=1))),
        "perpendicular": float(np.mean(np.sum(perp_centered**2, axis=1))),
    }


def tilt_compensated_horizontal_components(y, acc):
    """Return compass-plane coordinates (-east, north) for calibrated vectors."""
    acc_unit = np.asarray(acc, dtype=float)
    acc_unit /= np.linalg.norm(acc_unit, axis=1, keepdims=True)
    y = np.asarray(y, dtype=float)
    horizontal = y - np.sum(y * acc_unit, axis=1, keepdims=True) * acc_unit
    forward = np.array([1.0, 0.0, 0.0])
    forward_level = forward - np.sum(
        acc_unit * forward, axis=1, keepdims=True
    ) * acc_unit
    forward_level /= np.linalg.norm(forward_level, axis=1, keepdims=True)
    right_level = np.cross(acc_unit, forward_level)
    right_level /= np.linalg.norm(right_level, axis=1, keepdims=True)
    north = np.sum(horizontal * forward_level, axis=1)
    east = np.sum(horizontal * right_level, axis=1)
    return -east, north


def plot_diagnostics(calibrator, W, sectors, c=0.0):
    """Plot norm/dot residual histograms and per-sector pitch/perp variances."""
    import matplotlib.pyplot as plt

    norm_residuals = []
    dot_residuals = []
    pitch_variances = []
    perp_variances = []
    for mag, acc, _, _ in sectors:
        values = calibrated_sector_objectives(W, mag, acc, c)
        norm_residuals.extend(values["norm_residual"])
        dot_residuals.extend(values["dot_residual"])
        variance = sector_variances(W, mag, acc)
        pitch_variances.append(variance["pitch"])
        perp_variances.append(variance["perpendicular"])

    fig, axes = plt.subplots(2, 2, figsize=(8, 6))
    axes[0, 0].hist(norm_residuals, bins=40, color="tab:blue", alpha=0.8)
    axes[0, 0].set_title(r"Norm residual: $\|y\|^2 - 1$")
    axes[0, 1].hist(dot_residuals, bins=40, color="tab:orange", alpha=0.8)
    axes[0, 1].set_title(r"Dot residual: $\hat{a}^T y - c$")
    axes[1, 0].plot(pitch_variances, "o-", color="tab:green")
    axes[1, 0].set_title("Pitch variance per sector")
    axes[1, 1].plot(perp_variances, "o-", color="tab:red")
    axes[1, 1].set_title("Perpendicular variance per sector")
    for axis in axes.flat:
        axis.grid(alpha=0.25)
    axes[0, 0].set_ylabel("count")
    axes[0, 1].set_ylabel("count")
    axes[1, 0].set_xlabel("sector")
    axes[1, 1].set_xlabel("sector")
    plt.tight_layout()
    return fig, axes


def plot_unit_sphere(W, sectors):
    """Plot calibrated magnetometer and unit accelerometer data on a unit sphere."""
    import plotly.graph_objects as go

    figure = go.Figure()
    u = np.linspace(0.0, 2.0 * np.pi, 40)
    v = np.linspace(0.0, np.pi, 20)
    figure.add_trace(go.Surface(
        x=np.outer(np.cos(u), np.sin(v)),
        y=np.outer(np.sin(u), np.sin(v)),
        z=np.outer(np.ones_like(u), np.cos(v)),
        surfacecolor=np.zeros((len(u), len(v))),
        colorscale=[[0.0, "#9ca3af"], [1.0, "#9ca3af"]],
        showscale=False,
        opacity=0.16,
        name="unit sphere",
        hoverinfo="skip",
    ))
    colors = [
        f"hsl({int(360 * index / max(len(sectors), 1))}, 75%, 48%)"
        for index in range(len(sectors))
    ]
    for index, (mag, acc, _, _) in enumerate(sectors):
        y = affine_transform(W, mag)
        acc_unit = acc / np.linalg.norm(acc, axis=1, keepdims=True)
        figure.add_trace(go.Scatter3d(
            x=y[:, 0], y=y[:, 1], z=y[:, 2],
            mode="markers",
            name=f"sector {index} mag",
            marker={"size": 3, "color": colors[index], "opacity": 0.55},
        ))
        figure.add_trace(go.Scatter3d(
            x=-acc_unit[:, 0], y=-acc_unit[:, 1], z=-acc_unit[:, 2],
            mode="markers",
            name=f"sector {index} acc",
            marker={"size": 4, "symbol": "diamond", "color": colors[index],
                    "opacity": 0.8},
        ))
    figure.update_layout(
        title="Calibrated magnetometer and unit accelerometer on the unit sphere",
        width=850,
        height=700,
        scene={
            "xaxis_title": "x",
            "yaxis_title": "y",
            "zaxis_title": "z",
            "aspectmode": "cube",
        },
        margin={"l": 0, "r": 0, "b": 0, "t": 45},
        legend={"font": {"size": 9}},
    )
    return figure


def _cross_matrix(vector):
    """Return the matrix such that vector cross omega equals matrix @ omega."""
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def estimate_angular_rate(vectors, times):
    """Estimate body angular rate from one or more fixed-vector tracks.

    For a fixed world vector represented in body coordinates, dv/dt = v x omega.
    ``vectors`` has shape (N, 3) or (N, M, 3), and times has shape (N,).
    """
    vectors = np.asarray(vectors, dtype=float)
    if vectors.ndim == 2:
        vectors = vectors[:, None, :]
    times = np.asarray(times, dtype=float)
    derivatives = np.gradient(vectors, times, axis=0, edge_order=1)
    rates = np.zeros((len(times), 3))
    for sample_index in range(len(times)):
        design = np.vstack([_cross_matrix(vector)
                            for vector in vectors[sample_index]])
        target = derivatives[sample_index].reshape(-1)
        rates[sample_index] = np.linalg.lstsq(design, target, rcond=None)[0]
    return rates


def gyro_motion_dataset(sectors, W, use_acc=True, use_mag=True, trim=1):
    """Build raw gyro and kinematic angular-rate pairs from recorded sectors."""
    gyro_rows = []
    reference_rows = []
    sector_rows = []
    valid_rows = []
    for sector_index, (mag, acc, gyro, dt) in enumerate(sectors):
        mag = np.asarray(mag, dtype=float)
        acc = np.asarray(acc, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        dt = np.asarray(dt, dtype=float)
        times = np.concatenate(([0.0], np.cumsum(dt[1:])))
        acc_unit = acc / np.linalg.norm(acc, axis=1, keepdims=True)
        vectors = []
        if use_acc:
            vectors.append(acc_unit)
        if use_mag:
            vectors.append(affine_transform(W, mag))
        if not vectors:
            raise ValueError("at least one motion vector track is required")
        reference = estimate_angular_rate(np.stack(vectors, axis=1), times)
        gyro_rows.append(gyro)
        reference_rows.append(reference)
        sector_rows.append(np.full(len(gyro), sector_index, dtype=int))
        valid = np.ones(len(gyro), dtype=bool)
        valid[:trim] = False
        valid[-trim:] = False
        valid_rows.append(valid)
    return (np.vstack(gyro_rows), np.vstack(reference_rows),
            np.concatenate(sector_rows), np.concatenate(valid_rows))


def fit_diagonal_bias_gyro(sectors, W, use_acc=True, use_mag=True, trim=1):
    """Fit omega_ref = diag(scale) @ (gyro - bias) post-hoc."""
    gyro, reference, sector_index, valid = gyro_motion_dataset(
        sectors, W, use_acc=use_acc, use_mag=use_mag, trim=trim
    )
    coefficients = np.array([
        np.linalg.lstsq(
            np.column_stack((gyro[valid, axis], np.ones(valid.sum()))),
            reference[valid, axis],
            rcond=None,
        )[0]
        for axis in range(3)
    ])
    scale = coefficients[:, 0]
    intercept = coefficients[:, 1]
    bias = np.divide(-intercept, scale, out=np.zeros(3), where=np.abs(scale) > 1e-12)
    predicted = gyro * scale + intercept
    residual = predicted - reference
    correlations = np.array([
        np.corrcoef(predicted[valid, axis], reference[valid, axis])[0, 1]
        if np.std(predicted[valid, axis]) > 0 and np.std(reference[valid, axis]) > 0 else 0.0
        for axis in range(3)
    ])
    per_sector = []
    for index in range(len(sectors)):
        mask = (sector_index == index) & valid
        axis_correlation = np.array([
            np.corrcoef(predicted[mask, axis], reference[mask, axis])[0, 1]
            if np.std(predicted[mask, axis]) > 0 and np.std(reference[mask, axis]) > 0 else 0.0
            for axis in range(3)
        ])
        per_sector.append({
            "sector": index,
            "samples": int(mask.sum()),
            "rmse": float(np.sqrt(np.mean(residual[mask]**2))),
            "correlation": axis_correlation,
        })
    return {
        "scale": scale,
        "bias": bias,
        "intercept": intercept,
        "gyro": gyro,
        "reference": reference,
        "predicted": predicted,
        "residual": residual,
        "correlation": correlations,
        "sector_index": sector_index,
        "valid": valid,
        "per_sector": per_sector,
    }


def plot_gyro_motion_fit(report):
    """Plot fitted gyro versus kinematic angular-rate references."""
    import matplotlib.pyplot as plt

    valid = report["valid"]
    gyro = report["predicted"][valid]
    reference = report["reference"][valid]
    fig, axes = plt.subplots(2, 2, figsize=(8, 6))
    labels = ("x", "y", "z")
    for axis_index, label in enumerate(labels):
        axes.flat[axis_index].scatter(
            reference[:, axis_index], gyro[:, axis_index], s=4, alpha=0.25
        )
        limits = np.array([
            reference[:, axis_index].min(), reference[:, axis_index].max(),
            gyro[:, axis_index].min(), gyro[:, axis_index].max(),
        ])
        axes.flat[axis_index].plot(limits[[0, 1]], limits[[0, 1]], "k--", linewidth=0.8)
        axes.flat[axis_index].set_title(
            f"gyro {label}: correlation {report['correlation'][axis_index]:.2f}"
        )
        axes.flat[axis_index].set_xlabel("kinematic rate")
        axes.flat[axis_index].set_ylabel("calibrated gyro rate")
        axes.flat[axis_index].grid(alpha=0.25)
    rmse = [row["rmse"] for row in report["per_sector"]]
    axes[1, 1].clear()
    axes[1, 1].plot(rmse, "o-", color="tab:red")
    axes[1, 1].set_title("gyro motion RMSE per sector")
    axes[1, 1].set_xlabel("sector")
    axes[1, 1].set_ylabel("RMSE")
    axes[1, 1].grid(alpha=0.25)
    plt.tight_layout()
    return fig, axes


def plot_gyro_x_comparison(reports):
    """Plot gyro-X fits from multiple motion-reference reports together."""
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(8, 6))
    colors = ("tab:blue", "tab:orange", "tab:green")
    for (name, report), color in zip(reports.items(), colors):
        valid = report["valid"]
        reference = report["reference"][valid, 0]
        predicted = report["predicted"][valid, 0]
        order = np.argsort(reference)
        axis.plot(reference[order], predicted[order], ".", ms=2,
                  alpha=0.25, color=color, label=name)
    limits = axis.get_xlim()
    axis.plot(limits, limits, "k--", linewidth=0.8, label="ideal")
    axis.set_xlabel("kinematic gyro X reference")
    axis.set_ylabel("fitted calibrated gyro X")
    axis.set_title("Gyro X agreement across motion references")
    axis.grid(alpha=0.25)
    axis.legend()
    figure.tight_layout()
    return figure, axis


def integrate_calibrated_gyro(sector, report, axis=0, N=None, phase_sign=-1.0):
    """Integrate one calibrated gyro axis into the sector phase convention."""
    _, _, gyro, dt = sector
    gyro = np.asarray(gyro, dtype=float)
    dt = np.asarray(dt, dtype=float)
    if N is not None:
        gyro = gyro[:N]
        dt = dt[:N]
    calibrated = phase_sign * report["scale"][axis] * (
        gyro[:, axis] - report["bias"][axis]
    )
    phase = np.zeros(len(calibrated))
    if len(phase) > 1:
        phase[1:] = np.cumsum(
            0.5 * (calibrated[:-1] + calibrated[1:]) * dt[1:]
        )
    time = np.concatenate(([0.0], np.cumsum(dt[1:])))
    return time, phase


def sector_phase_tracks(W, sector, gyro_report, axis=0, N=None):
    """Return integrated gyro, accel, and calibrated-mag phase tracks."""
    mag, acc, _, _ = sector
    if N is not None:
        mag = mag[:N]
        acc = acc[:N]
        sector = (mag, acc, sector[2][:N], sector[3][:N])
    acc_unit, rotation_axis, _, _ = sector_rotation_basis(acc)
    _, gyro_phase = integrate_calibrated_gyro(sector, gyro_report, axis)
    e1 = acc_unit[0] - rotation_axis * np.dot(rotation_axis, acc_unit[0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(rotation_axis, e1)
    accel_phase = np.unwrap(np.arctan2(
        acc_unit @ e2, acc_unit @ e1
    ))
    y = affine_transform(W, mag)
    mag_phase = np.unwrap(np.arctan2(y @ e2, y @ e1))
    accel_phase -= accel_phase[0]
    mag_phase -= mag_phase[0]
    return gyro_phase, accel_phase, mag_phase


def plot_integrated_gyro_motion(W, sectors, gyro_report, N=None, max_sectors=None):
    """Plot integrated calibrated gyro phase against accel/mag phase tracks."""
    import matplotlib.pyplot as plt

    count = len(sectors) if max_sectors is None else min(max_sectors, len(sectors))
    figure, axes = plt.subplots(count, 1, figsize=(8, max(3, 2.2 * count)),
                                squeeze=False, sharex=False)
    for index in range(count):
        time, _ = integrate_calibrated_gyro(sectors[index], gyro_report, N=N)
        gyro_phase, accel_phase, mag_phase = sector_phase_tracks(
            W, sectors[index], gyro_report, N=N
        )
        axis = axes[index, 0]
        axis.plot(time, gyro_phase, label="integrated calibrated gyro X")
        axis.plot(time, accel_phase, label="accelerometer phase")
        axis.plot(time, mag_phase, label="calibrated magnetometer phase", alpha=0.7)
        axis.set_title(f"sector {index}")
        axis.set_ylabel("angle (rad)")
        axis.grid(alpha=0.25)
        axis.legend(fontsize="small")
    axes[-1, 0].set_xlabel("time (s)")
    figure.suptitle("Integrated gyro motion versus vector-derived phase")
    figure.tight_layout()
    return figure, axes


def _norm_feature(m):
    x, y, z = np.asarray(m, dtype=float)
    return np.array([x*x, y*y, z*z, 1.0, 2*x*y, 2*x*z, 2*x,
                     2*y*z, 2*y, 2*z])


def _ellipsoid_feature(m):
    x, y, z = np.asarray(m, dtype=float)
    return np.array([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z,
                     x, y, z, 1.0])


def _dot_feature(m, acc_unit):
    m = np.asarray(m, dtype=float)
    a = np.asarray(acc_unit, dtype=float)
    return np.r_[m[0] * a, m[1] * a, m[2] * a, a]


def weighted_dot_matrix(W, sectors, eps=1e-8):
    """Build G_dot with frozen inverse squared calibrated-magnitude weights."""
    G_dot = np.zeros((13, 13))
    for mag, acc, _, _ in sectors:
        mag = np.asarray(mag, dtype=float)
        acc = np.asarray(acc, dtype=float)
        acc_unit = acc / np.linalg.norm(acc, axis=1, keepdims=True)
        y = affine_transform(W, mag)
        weights = 1.0 / np.maximum(np.sum(y * y, axis=1), eps)
        for sample_mag, sample_acc, weight in zip(mag, acc_unit, weights):
            b = np.r_[_dot_feature(sample_mag, sample_acc), -1.0]
            G_dot += weight * np.outer(b, b)
    return G_dot


def _magnetometer_design(m):
    m = np.asarray(m, dtype=float)
    design = np.zeros((3, 12))
    design[:, 0:3] = m[0] * np.eye(3)
    design[:, 3:6] = m[1] * np.eye(3)
    design[:, 6:9] = m[2] * np.eye(3)
    design[:, 9:12] = np.eye(3)
    return design


def _vectorize_transform(W):
    return np.asarray(W, dtype=float).reshape(3, 4, order="F").ravel(order="F")


def _transform_from_vector(w):
    return np.asarray(w, dtype=float).reshape(3, 4, order="F")


def diagonal_bias_transform(parameters):
    """Build W from [scale_x, scale_y, scale_z, bias_x, bias_y, bias_z]."""
    parameters = np.asarray(parameters, dtype=float)
    W = np.zeros((3, 4))
    W[:, :3] = np.diag(parameters[:3])
    W[:, 3] = parameters[3:6]
    return W


def _diagonal_embedding():
    embedding = np.zeros((12, 6))
    embedding[0, 0] = 1.0
    embedding[4, 1] = 1.0
    embedding[8, 2] = 1.0
    embedding[9, 3] = 1.0
    embedding[10, 4] = 1.0
    embedding[11, 5] = 1.0
    return embedding


def _q_from_transform(W):
    Q = np.asarray(W).T @ np.asarray(W)
    return np.array([Q[i, j] for i, j in Q_PAIRS])


def _jacobian_q(W):
    J = np.zeros((10, 12))
    columns = [np.asarray(W)[:, i] for i in range(4)]
    for diagonal, column_index in enumerate((0, 1, 2, 3)):
        start = 3 * column_index
        J[diagonal, start:start + 3] = 2.0 * columns[column_index]
    off_diagonals = ((4, 0, 1), (5, 0, 2), (6, 0, 3),
                     (7, 1, 2), (8, 1, 3), (9, 2, 3))
    for row, i, j in off_diagonals:
        J[row, 3*i:3*i+3] = columns[j]
        J[row, 3*j:3*j+3] = columns[i]
    return J


def _rodrigues(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    I = np.eye(3)
    return I * np.cos(angle) + K * np.sin(angle) + np.outer(axis, axis) * (1.0 - np.cos(angle))


def _unwrap_near(theta, reference):
    while theta - reference > np.pi:
        theta -= 2.0 * np.pi
    while theta - reference < -np.pi:
        theta += 2.0 * np.pi
    return theta


class SectorCalibrator:
    """Reference implementation of the C++ streamed quadratic calibrator."""

    def __init__(self, options: Optional[Options] = None):
        self.options = options or Options()
        self.reset()

    def reset(self):
        self.G_pitch = np.zeros((12, 12))
        self.G_perp = np.zeros((12, 12))
        self.G_norm = np.zeros((10, 10))
        self.h_norm = np.zeros(10)
        self.G_dot = np.zeros((13, 13))
        self.ellipsoid_N = np.zeros((10, 10))
        self.mag_min = None
        self.mag_max = None
        self.norm_count = 0
        self.sample_count = 0
        self.sectors = []
        self.current = []
        self.in_sector = False
        self.initialized = False
        self.W_prior = np.zeros((3, 4))
        self.W_current = np.zeros((3, 4))
        self.c_current = 0.0

    def begin_sector(self):
        self.current = []
        self.in_sector = True

    def add_sample(self, mag, acc, gyro=None, dt=0.0):
        mag = np.asarray(mag, dtype=float)
        acc = np.asarray(acc, dtype=float)
        acc_norm = np.linalg.norm(acc)
        if (acc_norm < self.options.min_acc_norm or
                acc_norm > self.options.max_acc_norm or
                not np.all(np.isfinite(mag)) or not np.all(np.isfinite(acc))):
            return False
        acc_unit = acc / acc_norm
        aq = _norm_feature(mag)
        self.G_norm += np.outer(aq, aq)
        self.h_norm += aq
        self.norm_count += 1
        dd = np.r_[_dot_feature(mag, acc_unit), -1.0]
        self.G_dot += np.outer(dd, dd)
        e = _ellipsoid_feature(mag)
        self.ellipsoid_N += np.outer(e, e)
        self.mag_min = mag.copy() if self.mag_min is None else np.minimum(self.mag_min, mag)
        self.mag_max = mag.copy() if self.mag_max is None else np.maximum(self.mag_max, mag)
        if self.in_sector:
            if len(self.current) < self.options.max_sector_samples:
                self.current.append((mag.copy(), acc.copy(), acc_unit.copy(),
                                     np.zeros(3) if gyro is None else np.asarray(gyro, dtype=float).copy(),
                                     float(dt)))
            self.sample_count += 1
        return True

    def end_sector(self):
        self.in_sector = False
        samples = self.current
        self.current = []
        if len(samples) < 50:
            return False
        acc_units = np.array([sample[2] for sample in samples])
        S = acc_units.T @ acc_units
        eigenvalues, eigenvectors = np.linalg.eigh(S)
        if not np.all(np.isfinite(eigenvalues)):
            return False
        axis = eigenvectors[:, 0]
        axis /= np.linalg.norm(axis)
        axis_index = np.argmax(np.abs(axis))
        if axis[axis_index] < 0:
            axis = -axis
        e1 = acc_units[0] - axis * np.dot(axis, acc_units[0])
        if np.linalg.norm(e1) < 1e-6:
            e1 = eigenvectors[:, 2] - axis * np.dot(axis, eigenvectors[:, 2])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(axis, e1)
        S_B = np.zeros((3, 12))
        S_BB = np.zeros((12, 12))
        S_C = np.zeros((3, 12))
        S_CC = np.zeros((12, 12))
        last_theta = 0.0
        have_last = False
        theta_min = np.inf
        theta_max = -np.inf
        acc_rss = 0.0
        used = 0
        for mag, _, acc_unit, _, _ in samples:
            residual = np.dot(axis, acc_unit)
            acc_rss += residual * residual
            p = acc_unit - axis * residual
            pn = np.linalg.norm(p)
            if pn <= 1e-9:
                continue
            p /= pn
            theta = np.arctan2(np.dot(p, e2), np.dot(p, e1))
            if have_last:
                theta = _unwrap_near(theta, last_theta)
            have_last = True
            last_theta = theta
            theta_min = min(theta_min, theta)
            theta_max = max(theta_max, theta)
            B = _rodrigues(axis, -theta) @ _magnetometer_design(mag)
            S_B += B
            S_BB += B.T @ B
            P = np.eye(3) - np.outer(acc_unit, acc_unit)
            C = _rodrigues(axis, -theta) @ P @ _magnetometer_design(mag)
            S_C += C
            S_CC += C.T @ C
            used += 1
        if used < 50:
            return False
        Gs = S_BB - S_B.T @ S_B / used
        self.G_pitch += 0.5 * (Gs + Gs.T)
        Gperp = S_CC - S_C.T @ S_C / used
        self.G_perp += 0.5 * (Gperp + Gperp.T)
        self.sectors.append(SectorInfo(used, axis, np.sqrt(acc_rss / len(samples)),
                                        theta_max - theta_min))
        return True

    def add_sector(self, mag, acc, gyro=None, dt=None):
        mag = np.asarray(mag, dtype=float)
        acc = np.asarray(acc, dtype=float)
        if mag.ndim != 2 or mag.shape[1] != 3 or acc.shape != mag.shape:
            raise ValueError("mag and acc must both have shape (N, 3)")
        gyro = np.zeros_like(mag) if gyro is None else np.asarray(gyro, dtype=float)
        dt = np.zeros(len(mag)) if dt is None else np.asarray(dt, dtype=float)
        self.begin_sector()
        for index in range(len(mag)):
            self.add_sample(mag[index], acc[index], gyro[index], dt[index])
        return self.end_sector()

    def ingest(self, mag, acc, *, start_sector=False, end_sector=False, gyro=None, dt=0.0):
        """Add one point; optionally begin before it and finalize after it."""
        if start_sector:
            self.begin_sector()
        accepted = self.add_sample(mag, acc, gyro, dt)
        finished = self.end_sector() if end_sector else False
        return accepted, finished

    def initial_transform_from_bounds(self):
        W = np.zeros((3, 4))
        if self.mag_min is None:
            W[:, :3] = np.eye(3)
            return W
        center = 0.5 * (self.mag_min + self.mag_max)
        scale = 2.0 / np.maximum(self.mag_max - self.mag_min, 1e-9)
        W[:, :3] = np.diag(scale)
        W[:, 3] = -scale * center
        return W

    def initial_transform_from_ellipsoid(self):
        if self.norm_count < 20:
            return self.initial_transform_from_bounds()
        eigenvalues, eigenvectors = np.linalg.eigh(self.ellipsoid_N)
        beta = eigenvectors[:, 0]
        for sign in (1.0, -1.0):
            bta = sign * beta
            Q = np.array([[bta[0], bta[3], bta[4]],
                          [bta[3], bta[1], bta[5]],
                          [bta[4], bta[5], bta[2]]])
            u = bta[6:9]
            k = bta[9]
            if abs(np.linalg.det(Q)) < 1e-12:
                continue
            center = -0.5 * np.linalg.solve(Q, u)
            radius = center @ Q @ center - k
            if radius <= 1e-12:
                continue
            C = 0.5 * (Q / radius + (Q / radius).T)
            ce_values, ce_vectors = np.linalg.eigh(C)
            if ce_values.min() <= 1e-9:
                continue
            A = ce_vectors @ np.diag(np.sqrt(ce_values)) @ ce_vectors.T
            W = np.zeros((3, 4))
            W[:, :3] = A
            W[:, 3] = -A @ center
            return W
        return self.initial_transform_from_bounds()

    def _initial_dot(self, W):
        if self.norm_count <= 0:
            return 0.0
        w = _vectorize_transform(W)
        sd = -self.G_dot[:12, 12]
        return np.clip(sd @ w / self.norm_count, -1.5, 1.5)

    def _cost(self, x, options):
        w = x[:12]
        W = _transform_from_vector(w)
        q = _q_from_transform(W)
        cost = 0.5 * options.pitch_weight * (w @ self.G_pitch @ w)
        cost += 0.5 * options.perp_weight * (w @ self.G_perp @ w)
        norm_ss = q @ self.G_norm @ q - 2.0 * self.h_norm @ q + self.norm_count
        cost += 0.5 * options.norm_weight * norm_ss
        cost += 0.5 * options.dot_weight * (x @ self.G_dot @ x)
        if options.prior_weight > 0:
            d = w - _vectorize_transform(self.W_prior)
            cost += 0.5 * options.prior_weight * (d @ d)
        return float(cost)

    def solve(self, options=None, use_ellipsoid_init=True):
        options = options or SolveOptions()
        if not self.initialized:
            self.W_current = (self.initial_transform_from_ellipsoid() if use_ellipsoid_init
                              else self.initial_transform_from_bounds())
            self.W_prior = self.W_current.copy()
            self.c_current = self._initial_dot(self.W_current)
            self.initialized = True
        x = np.r_[_vectorize_transform(self.W_current), self.c_current]
        damping = options.lm_lambda0
        previous_cost = self._cost(x, options)
        iterations = 0
        converged = False
        for iteration in range(options.max_iters):
            w = x[:12]
            W = _transform_from_vector(w)
            H = np.zeros((13, 13))
            g = np.zeros(13)
            H[:12, :12] += options.pitch_weight * self.G_pitch
            g[:12] += options.pitch_weight * self.G_pitch @ w
            H[:12, :12] += options.perp_weight * self.G_perp
            g[:12] += options.perp_weight * self.G_perp @ w
            H += options.dot_weight * self.G_dot
            g += options.dot_weight * self.G_dot @ x
            q = _q_from_transform(W)
            Jq = _jacobian_q(W)
            sq = self.G_norm @ q - self.h_norm
            H[:12, :12] += options.norm_weight * (Jq.T @ self.G_norm @ Jq)
            g[:12] += options.norm_weight * (Jq.T @ sq)
            if options.prior_weight > 0:
                d = w - _vectorize_transform(self.W_prior)
                H[:12, :12] += options.prior_weight * np.eye(12)
                g[:12] += options.prior_weight * d
            H_lm = H + np.diag(damping * (np.abs(np.diag(H)) + 1e-6))
            dx = np.linalg.lstsq(H_lm, -g, rcond=None)[0]
            if not np.all(np.isfinite(dx)):
                break
            trial = x + dx
            trial[12] = np.clip(trial[12], -2.0, 2.0)
            trial_cost = self._cost(trial, options)
            if np.isfinite(trial_cost) and trial_cost < previous_cost:
                relative_decrease = (previous_cost - trial_cost) / max(previous_cost, 1.0)
                x = trial
                previous_cost = trial_cost
                damping = max(damping * options.lm_down, 1e-12)
                iterations = iteration + 1
                if np.linalg.norm(dx) < options.step_tol or relative_decrease < options.cost_tol:
                    converged = True
                    break
            else:
                damping *= options.lm_up
        self.W_current = _transform_from_vector(x[:12])
        self.c_current = float(x[12])
        A = self.W_current[:, :3]
        t = self.W_current[:, 3]
        bias = -np.linalg.solve(A, t)
        return Result(self.W_current.copy(), A.copy(), bias, t.copy(), self.c_current,
                      previous_cost, iterations, converged)

    def solve_diagonal(self, options=None, use_ellipsoid_init=True):
        """Solve the same objectives with diagonal scale plus additive bias W."""
        options = options or SolveOptions()
        embedding = _diagonal_embedding()
        initial = (self.initial_transform_from_ellipsoid() if use_ellipsoid_init
                   else self.initial_transform_from_bounds())
        parameters = np.r_[np.diag(initial[:, :3]), initial[:, 3]]
        prior = parameters.copy()
        c = self._initial_dot(diagonal_bias_transform(parameters))
        previous_cost = np.inf
        damping = options.lm_lambda0
        iterations = 0
        converged = False

        def objective(values, target):
            W = diagonal_bias_transform(values)
            w = _vectorize_transform(W)
            q = _q_from_transform(W)
            cost = 0.5 * options.pitch_weight * (w @ self.G_pitch @ w)
            cost += 0.5 * options.perp_weight * (w @ self.G_perp @ w)
            norm_ss = q @ self.G_norm @ q - 2.0 * self.h_norm @ q + self.norm_count
            cost += 0.5 * options.norm_weight * norm_ss
            x = np.r_[w, target]
            cost += 0.5 * options.dot_weight * (x @ self.G_dot @ x)
            if options.prior_weight > 0:
                difference = values - prior
                cost += 0.5 * options.prior_weight * (difference @ difference)
            return float(cost)

        previous_cost = objective(parameters, c)
        for iteration in range(options.max_iters):
            W = diagonal_bias_transform(parameters)
            w = _vectorize_transform(W)
            x = np.r_[w, c]
            H_full = options.pitch_weight * self.G_pitch
            H_full = H_full + options.perp_weight * self.G_perp
            g_full = H_full @ w
            H_dot = options.dot_weight * self.G_dot
            H_reduced = embedding.T @ H_full @ embedding
            gradient = embedding.T @ g_full
            H_reduced = H_reduced + embedding.T @ H_dot[:12, :12] @ embedding
            gradient = gradient + embedding.T @ (H_dot[:12, :] @ x)
            q = _q_from_transform(W)
            Jq = _jacobian_q(W) @ embedding
            square_residual = self.G_norm @ q - self.h_norm
            H_reduced += options.norm_weight * (Jq.T @ self.G_norm @ Jq)
            gradient += options.norm_weight * (Jq.T @ square_residual)
            dot_gradient_c = H_dot[12, :12] @ w + H_dot[12, 12] * c
            reduced_H = np.zeros((7, 7))
            reduced_H[:6, :6] = H_reduced
            reduced_H[:6, 6] = embedding.T @ H_dot[:12, 12]
            reduced_H[6, :6] = reduced_H[:6, 6]
            reduced_H[6, 6] = H_dot[12, 12]
            reduced_gradient = np.r_[gradient, dot_gradient_c]
            if options.prior_weight > 0:
                reduced_H[:6, :6] += options.prior_weight * np.eye(6)
                reduced_gradient[:6] += options.prior_weight * (parameters - prior)
            damped = reduced_H + np.diag(
                damping * (np.abs(np.diag(reduced_H)) + 1e-6)
            )
            step = np.linalg.lstsq(damped, -reduced_gradient, rcond=None)[0]
            if not np.all(np.isfinite(step)):
                break
            trial_parameters = parameters + step[:6]
            trial_c = np.clip(c + step[6], -2.0, 2.0)
            trial_cost = objective(trial_parameters, trial_c)
            if np.isfinite(trial_cost) and trial_cost < previous_cost:
                relative_decrease = (previous_cost - trial_cost) / max(previous_cost, 1.0)
                parameters = trial_parameters
                c = trial_c
                previous_cost = trial_cost
                damping = max(damping * options.lm_down, 1e-12)
                iterations = iteration + 1
                if np.linalg.norm(step) < options.step_tol or relative_decrease < options.cost_tol:
                    converged = True
                    break
            else:
                damping *= options.lm_up
        W = diagonal_bias_transform(parameters)
        A = W[:, :3]
        t = W[:, 3]
        bias = -np.linalg.solve(A, t)
        return Result(W, A, bias, t, float(c), previous_cost, iterations, converged)

    def solve_irls(self, sectors, options=None, outer_iters=4,
                   use_ellipsoid_init=True, eps=1e-8):
        """Batch IRLS for relative dot residuals using replayable sector samples.

        Each outer pass freezes 1 / ||W mbar||^2, rebuilds G_dot in one batch
        pass, and runs the existing LM solver from the previous solution.
        """
        options = options or SolveOptions()
        if not sectors:
            raise ValueError("sectors must contain at least one sector")
        if outer_iters < 1:
            raise ValueError("outer_iters must be positive")

        if not self.initialized:
            self.W_current = (self.initial_transform_from_ellipsoid() if use_ellipsoid_init
                              else self.initial_transform_from_bounds())
            self.W_prior = self.W_current.copy()
            self.c_current = self._initial_dot(self.W_current)
            self.initialized = True

        unweighted_G_dot = self.G_dot.copy()
        result = None
        for _ in range(outer_iters):
            self.G_dot = weighted_dot_matrix(self.W_current, sectors, eps)
            result = self.solve(options, use_ellipsoid_init=False)

        # Leave the final frozen weighted matrix available for diagnostics.
        if result is None:
            self.G_dot = unweighted_G_dot
            raise RuntimeError("IRLS did not execute")
        return result

    def diagnostics(self, W=None):
        W = self.W_current if W is None else np.asarray(W, dtype=float)
        q = _q_from_transform(W)
        norm_mean = self.h_norm @ q / max(self.norm_count, 1)
        norm_ss = q @ self.G_norm @ q - 2.0 * self.h_norm @ q + self.norm_count
        w = _vectorize_transform(W)
        dot_mean = (-self.G_dot[:12, 12] @ w) / max(self.norm_count, 1)
        dot_ss = np.r_[w, self.c_current] @ self.G_dot @ np.r_[w, self.c_current]
        return {
            "norm_squared_mean": float(norm_mean),
            "norm_squared_rms_residual": float(np.sqrt(max(norm_ss / max(self.norm_count, 1), 0.0))),
            "dot_mean": float(dot_mean),
            "dot_rms_residual": float(np.sqrt(max(dot_ss / max(self.norm_count, 1), 0.0))),
        }


def load_sector_file(path: Path):
    data = np.loadtxt(path, delimiter=",")
    return data[:, 0:3] / 1.0, data[:, 3:6], data[:, 6:9], data[:, 9]


def load_sector_directory(directory: Path):
    return [load_sector_file(path) for path in sorted(Path(directory).glob("*"))]
