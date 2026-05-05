#!/usr/bin/env python3
"""Render 3D visuals for the 7 reference cylinder fits.

Each visual shows the input PCD points, the fitted cylinder axis,
and a translucent cylinder surface at the fitted radius. The fit
parameters are pulled out of the fit log captured from
`cylinder_fitter_node`.
"""
from __future__ import annotations

import math
import os
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SAMPLES = REPO / "samples"
LOG = SAMPLES / "cylinder_fitter" / "fit_log.txt"
PCD_DIR = Path.home() / (
    "Downloads/TreeMapping-master/tree_mapping/pointclouds/"
    "pcd_real_for_cylinders_manual/paloverde"
)

CASE_LABELS = {
    0: ("_base", 0.12634136960388964),
    1: ("_base_wTrans", 0.14512224962743242),
    2: ("_left", 0.083676066395241541),
    4: ("_left_wOutlierRingOnly", 0.087997618743898318),
    5: ("_right", 0.1256250843095913),
    6: ("_rightleft", 0.090725929243689774),
    7: ("_rightright", 0.052456760430584705),
}

FIT_LINE_RE = re.compile(
    r"FIT DONE: rho=(?P<rho>[-\d.]+) r=(?P<r>[-\d.]+) "
    r"theta=(?P<theta>[-\d.]+) deg phi=(?P<phi>[-\d.]+) deg "
    r"alpha=(?P<alpha>[-\d.]+) deg"
)


def parse_fit_log(path: Path) -> dict[int, dict[str, float]]:
    out: dict[int, dict[str, float]] = {}
    current: int | None = None
    for line in path.read_text().splitlines():
        m = re.match(r"CASE file_num=(\d+)", line.strip())
        if m:
            current = int(m.group(1))
            continue
        m = FIT_LINE_RE.search(line)
        if m and current is not None:
            out[current] = {k: float(v) for k, v in m.groupdict().items()}
    return out


def load_pcd_xyz(path: Path) -> np.ndarray:
    pts: list[list[float]] = []
    in_data = False
    for line in path.read_text().splitlines():
        if not in_data:
            if line.startswith("DATA"):
                in_data = True
            continue
        toks = line.split()
        if len(toks) < 3:
            continue
        try:
            pts.append([float(toks[0]), float(toks[1]), float(toks[2])])
        except ValueError:
            pass
    return np.asarray(pts)


def axis_from_pca(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    centroid = points.mean(axis=0)
    centred = points - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    direction = vh[0]
    return centroid, direction / np.linalg.norm(direction)


def draw_cylinder_surface(
    ax,
    centroid: np.ndarray,
    direction: np.ndarray,
    points: np.ndarray,
    radius: float,
    color: str,
    alpha: float = 0.18,
    n_theta: int = 36,
    n_z: int = 20,
) -> None:
    # Build an orthonormal frame whose third axis is `direction`.
    n = direction
    helper = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(n, helper)
    u = u / np.linalg.norm(u)
    v = np.cross(n, u)

    # Project all points onto the axis to find the z extent of the cluster.
    centred = points - centroid
    z_proj = centred @ n
    z_min, z_max = z_proj.min(), z_proj.max()
    pad = 0.1 * (z_max - z_min + 1e-6)
    z_min -= pad
    z_max += pad

    theta = np.linspace(0.0, 2.0 * math.pi, n_theta)
    z = np.linspace(z_min, z_max, n_z)
    Theta, Z = np.meshgrid(theta, z)

    X = (
        centroid[0]
        + radius * (np.cos(Theta) * u[0] + np.sin(Theta) * v[0])
        + Z * n[0]
    )
    Y = (
        centroid[1]
        + radius * (np.cos(Theta) * u[1] + np.sin(Theta) * v[1])
        + Z * n[1]
    )
    Zw = (
        centroid[2]
        + radius * (np.cos(Theta) * u[2] + np.sin(Theta) * v[2])
        + Z * n[2]
    )
    ax.plot_surface(
        X,
        Y,
        Zw,
        color=color,
        alpha=alpha,
        linewidth=0,
        antialiased=True,
        shade=True,
    )

    # Axis line
    axis_pts = np.array(
        [centroid + z_min * n, centroid + z_max * n]
    )
    ax.plot(
        axis_pts[:, 0], axis_pts[:, 1], axis_pts[:, 2],
        color=color, linewidth=2.0, alpha=0.9,
    )


def render_case(file_num: int, fit: dict[str, float], out_dir: Path) -> Path:
    suf, expected_r = CASE_LABELS[file_num]
    pcd = PCD_DIR / f"cloud_00000000{suf}.pcd"
    pts = load_pcd_xyz(pcd)
    centroid, direction = axis_from_pca(pts)

    fig = plt.figure(figsize=(7.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        pts[:, 0], pts[:, 1], pts[:, 2],
        s=12, c="#1f77b4", alpha=0.85,
        edgecolor="k", linewidth=0.25,
    )
    draw_cylinder_surface(
        ax, centroid, direction, pts,
        radius=fit["r"], color="#d62728", alpha=0.18,
    )

    delta = fit["r"] - expected_r
    title = (
        f"file_num={file_num}  branch={suf}  n={len(pts)} pts\n"
        f"fitted r = {fit['r']:.6f} m   "
        f"upstream r = {expected_r:.6f} m   "
        f"|Δ| = {abs(delta):.2e} m\n"
        f"ρ = {fit['rho']:.3f}   "
        f"θ = {fit['theta']:.2f}°   "
        f"φ = {fit['phi']:.2f}°   "
        f"α = {fit['alpha']:.2f}°"
    )
    ax.set_title(title, fontsize=9.5)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")

    # equal-aspect tight bounds
    rng = pts.ptp(axis=0)
    centre = pts.mean(axis=0)
    half = max(rng) * 0.6
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)
    ax.view_init(elev=15.0, azim=-65.0)

    fig.tight_layout()
    out = out_dir / f"case_{file_num}{suf}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def render_grid(out_dir: Path, fits: dict[int, dict[str, float]]) -> Path:
    file_nums = [0, 1, 2, 4, 5, 6, 7]
    fig = plt.figure(figsize=(15.5, 8.5))
    for idx, fnum in enumerate(file_nums):
        suf, expected_r = CASE_LABELS[fnum]
        pcd = PCD_DIR / f"cloud_00000000{suf}.pcd"
        pts = load_pcd_xyz(pcd)
        centroid, direction = axis_from_pca(pts)

        ax = fig.add_subplot(2, 4, idx + 1, projection="3d")
        ax.scatter(
            pts[:, 0], pts[:, 1], pts[:, 2],
            s=6, c="#1f77b4", alpha=0.85, edgecolor="k", linewidth=0.15,
        )
        draw_cylinder_surface(
            ax, centroid, direction, pts,
            radius=fits[fnum]["r"], color="#d62728", alpha=0.20,
        )
        ax.set_title(
            f"#{fnum} {suf}\nr={fits[fnum]['r']:.4f} m (Δ={fits[fnum]['r']-expected_r:+.1e})",
            fontsize=9,
        )
        ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        rng = pts.ptp(axis=0)
        centre = pts.mean(axis=0)
        half = max(rng) * 0.6
        ax.set_xlim(centre[0] - half, centre[0] + half)
        ax.set_ylim(centre[1] - half, centre[1] + half)
        ax.set_zlim(centre[2] - half, centre[2] + half)
        ax.view_init(elev=15.0, azim=-65.0)

    fig.suptitle(
        "Cylinder fitter — 7 reference cases\n"
        "blue = input PCD points    red surface = fitted cylinder at radius r",
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = out_dir / "grid_all_cases.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def main() -> None:
    out_dir = SAMPLES / "cylinder_fitter"
    out_dir.mkdir(parents=True, exist_ok=True)
    fits = parse_fit_log(LOG)
    if not fits:
        raise SystemExit(f"no FIT DONE lines parsed from {LOG}")

    print(f"parsed {len(fits)} fits")
    for fnum, fit in sorted(fits.items()):
        out = render_case(fnum, fit, out_dir)
        print(f"  case {fnum} -> {out.relative_to(REPO)}")
    grid = render_grid(out_dir, fits)
    print(f"  grid    -> {grid.relative_to(REPO)}")


if __name__ == "__main__":
    main()
