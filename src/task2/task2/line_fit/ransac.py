"""Pure-numpy 2D RANSAC line fitter.

No rclpy / ROS imports → unit-testable from a plain Python REPL. The service
node in `node.py` is the only consumer; everything here is geometry.

Line representation everywhere in this module:

    normal . p = offset

where `normal` is a unit 2-vector and `offset` is a signed scalar (distance
from the origin along `normal`). For a point p on the line: normal · p == offset.
For any other point p, |normal · p - offset| is the signed perpendicular
distance to the line.
"""

from __future__ import annotations

import numpy as np

from .exceptions import LineFitError, LineFitInternalError


# Smallest separation (m) between the two sampled points for a candidate to be
# considered. Two near-coincident samples produce a numerically unstable normal.
_MIN_SAMPLE_SEPARATION = 1e-3


def _line_from_two_points(p1: np.ndarray, p2: np.ndarray) -> tuple[np.ndarray, float]:
    """Construct (unit_normal, signed_offset) for the line through p1 and p2.

    Returns ((nx, ny), offset). The normal is the unit vector perpendicular to
    (p2 - p1); its sign is arbitrary here and will be re-oriented by the caller.
    """
    direction = p2 - p1
    length = float(np.linalg.norm(direction))
    if length < _MIN_SAMPLE_SEPARATION:
        raise LineFitInternalError(
            "two-point line: samples are too close to define a direction"
        )
    # 90-degree rotation: (dx, dy) -> (-dy, dx).
    normal = np.array([-direction[1], direction[0]], dtype=float) / length
    offset = float(normal @ p1)
    return normal, offset


def _refit_lsq(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Total-least-squares refit on a set of inlier points.

    Uses PCA: the eigenvector of the smallest eigenvalue of the centred
    covariance matrix is the normal to the best-fit line.
    """
    if points.shape[0] < 2:
        raise LineFitInternalError(f"refit needs >=2 points, got {points.shape[0]}")
    centroid = points.mean(axis=0)
    centred = points - centroid
    cov = centred.T @ centred
    _eigvals, eigvecs = np.linalg.eigh(cov)
    # eigh returns eigenvalues in ascending order → smallest first.
    normal = eigvecs[:, 0]
    length = float(np.linalg.norm(normal))
    if length < 1e-9:
        raise LineFitInternalError("refit produced a degenerate normal")
    normal = normal / length
    offset = float(normal @ centroid)
    return normal, offset


def fit_line_2d(
    points: np.ndarray,
    iterations: int = 100,
    inlier_threshold: float = 0.03,
    rng: np.random.Generator | None = None,
) -> tuple[np.ndarray, float, np.ndarray]:
    """RANSAC + least-squares refit for a 2D line.

    Parameters
    ----------
    points
        Shape (N, 2) array of (x, y) points in any frame.
    iterations
        Number of RANSAC trials. 100 is comfortable for the ~hundreds of
        points produced by a single lidar cone.
    inlier_threshold
        Maximum |signed distance to candidate line| (m) for a point to count
        as an inlier.
    rng
        Optional np.random.Generator. Pass one for deterministic behaviour
        in tests; otherwise a fresh default_rng() is used.

    Returns
    -------
    (unit_normal, signed_offset, inlier_mask)
        Where `unit_normal` is shape (2,), `signed_offset` is float, and
        `inlier_mask` is a boolean array of shape (N,).

    Raises
    ------
    LineFitError
        If fewer than 2 points are given, or no iteration produces any inliers
        (degenerate / empty point cloud).
    """
    if points.ndim != 2 or points.shape[1] != 2:
        raise LineFitError(f"points must be (N, 2), got shape {points.shape}")
    n = points.shape[0]
    if n < 2:
        raise LineFitError(f"need at least 2 points, got {n}")

    rng = rng if rng is not None else np.random.default_rng()

    best_inlier_count = 0
    best_inlier_mask: np.ndarray | None = None

    for _ in range(iterations):
        idx_a, idx_b = rng.choice(n, size=2, replace=False)
        p_a, p_b = points[idx_a], points[idx_b]
        try:
            normal, offset = _line_from_two_points(p_a, p_b)
        except LineFitInternalError:
            # Sampled points coincided; skip this iteration.
            continue

        distances = np.abs(points @ normal - offset)
        inlier_mask = distances <= inlier_threshold
        count = int(inlier_mask.sum())

        if count > best_inlier_count:
            best_inlier_count = count
            best_inlier_mask = inlier_mask

    if best_inlier_mask is None or best_inlier_count < 2:
        raise LineFitError(
            f"RANSAC failed: no iteration produced >=2 inliers (best={best_inlier_count})"
        )

    # Total-least-squares refit on the best inlier set for a precise final answer.
    final_normal, final_offset = _refit_lsq(points[best_inlier_mask])
    return final_normal, final_offset, best_inlier_mask
