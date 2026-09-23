import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from sector_calibrator import (  # noqa: E402
    SectorCalibrator,
    SolveOptions,
    _jacobian_q,
    _q_from_transform,
    _vectorize_transform,
    load_sector_directory,
    weighted_dot_matrix,
)


ROOT = Path(__file__).parents[1]
DATA_DIR = ROOT / "scripts" / "data" / "magacc_pitch" / "readings_32"


def make_sector(seed, count=60):
    rng = np.random.default_rng(seed)
    angles = np.linspace(-0.8, 0.8, count)
    axis = np.array([0.2, -0.3, 0.93])
    axis /= np.linalg.norm(axis)
    reference = np.array([1.0, 0.0, 0.0])
    reference -= axis * (axis @ reference)
    reference /= np.linalg.norm(reference)
    second = np.cross(axis, reference)
    acc_unit = np.cos(angles)[:, None] * reference + np.sin(angles)[:, None] * second
    acc = 9.8 * acc_unit
    mag = rng.normal(size=(count, 3))
    return mag, acc


def test_q_jacobian_matches_finite_difference():
    rng = np.random.default_rng(3)
    W = rng.normal(size=(3, 4))
    J = _jacobian_q(W)
    w = _vectorize_transform(W)
    numeric = np.zeros_like(J)
    epsilon = 1e-6
    for index in range(12):
        plus = w.copy()
        minus = w.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        numeric[:, index] = (
            _q_from_transform(plus.reshape(3, 4, order="F"))
            - _q_from_transform(minus.reshape(3, 4, order="F"))
        ) / (2.0 * epsilon)
    np.testing.assert_allclose(J, numeric, rtol=1e-5, atol=1e-7)


def test_batch_and_streaming_accumulators_match():
    sectors = [make_sector(1), make_sector(2)]
    batch = SectorCalibrator()
    for mag, acc in sectors:
        assert batch.add_sector(mag, acc)

    streaming = SectorCalibrator()
    for mag, acc in sectors:
        streaming.ingest(mag[0], acc[0], start_sector=True)
        for index in range(1, len(mag) - 1):
            streaming.ingest(mag[index], acc[index])
        accepted, finished = streaming.ingest(mag[-1], acc[-1], end_sector=True)
        assert accepted and finished

    np.testing.assert_allclose(batch.G_norm, streaming.G_norm)
    np.testing.assert_allclose(batch.h_norm, streaming.h_norm)
    np.testing.assert_allclose(batch.G_dot, streaming.G_dot)
    np.testing.assert_allclose(batch.G_pitch, streaming.G_pitch)
    np.testing.assert_allclose(batch.G_perp, streaming.G_perp)
    assert len(batch.sectors) == len(streaming.sectors) == 2


def test_solver_returns_finite_result_on_experimental_data():
    if not DATA_DIR.exists():
        return
    files = sorted(DATA_DIR.glob("*"))[:4]
    sectors = load_sector_directory(DATA_DIR)[:4]
    assert len(files) == len(sectors) > 0
    calibrator = SectorCalibrator()
    for mag, acc, _, _ in sectors:
        assert calibrator.add_sector(mag, acc)
    result = calibrator.solve(SolveOptions(max_iters=8), use_ellipsoid_init=False)
    assert result.W.shape == (3, 4)
    assert np.all(np.isfinite(result.W))
    assert np.isfinite(result.cost)
    assert np.all(np.isfinite(result.bias))


def test_batch_irls_returns_finite_weighted_solution():
    if not DATA_DIR.exists():
        return
    sectors = load_sector_directory(DATA_DIR)[:4]
    calibrator = SectorCalibrator()
    for mag, acc, _, _ in sectors:
        assert calibrator.add_sector(mag, acc)
    result = calibrator.solve_irls(
        sectors,
        SolveOptions(max_iters=8),
        outer_iters=2,
        use_ellipsoid_init=True,
    )
    weighted = weighted_dot_matrix(result.W, sectors)
    assert result.W.shape == (3, 4)
    assert weighted.shape == (13, 13)
    assert np.all(np.isfinite(result.W))
    assert np.all(np.isfinite(weighted))
