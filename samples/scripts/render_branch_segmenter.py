#!/usr/bin/env python3
"""Colorize the per-pixel instance_mask CSV emitted by branch_segmenter_node.

The CSV is one row per range-image scanline; values are zero-padded
2-digit instance ids (00, 01, ...) or `-1` for unassigned pixels.
We render two PNGs per cloud:
  * `instance_mask_colored.png` -- 2D label map with one tab20 colour per id
  * `instance_histogram.png`    -- bar chart of pixel counts per id

The original `range_image_*.png` and `range_image_modified_*.png`
files (already saved by the node) are left in place.
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SAMPLES = REPO / "samples"


def load_mask(csv_path: Path) -> np.ndarray:
    rows: list[list[int]] = []
    for line in csv_path.read_text().splitlines():
        toks = [t.strip() for t in line.split(",") if t.strip()]
        if not toks:
            continue
        rows.append([int(t) for t in toks])
    if not rows:
        raise SystemExit(f"empty mask in {csv_path}")
    width = max(len(r) for r in rows)
    arr = np.full((len(rows), width), fill_value=-1, dtype=np.int16)
    for i, r in enumerate(rows):
        arr[i, : len(r)] = r
    return arr


def render_label_map(mask: np.ndarray, out: Path, title: str) -> None:
    cmap = plt.get_cmap("tab20")
    n_ids = int(mask.max()) + 1 if mask.max() >= 0 else 1

    rgb = np.zeros((*mask.shape, 4), dtype=np.float32)
    rgb[..., 3] = 1.0
    bg = mask < 0
    rgb[bg] = (0.08, 0.08, 0.08, 1.0)
    fg = ~bg
    if fg.any():
        norm_ids = mask[fg].astype(np.float32) / max(1, n_ids - 1)
        rgb[fg] = cmap(norm_ids)

    fig, ax = plt.subplots(
        figsize=(min(18.0, max(7.0, mask.shape[1] / 200.0)),
                 max(2.5, mask.shape[0] / 60.0)),
    )
    ax.imshow(rgb, interpolation="nearest", aspect="auto")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("range-image column (azimuth)")
    ax.set_ylabel("range-image row (elevation)")
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)


def render_histogram(mask: np.ndarray, out: Path, title: str) -> None:
    counts = Counter(int(v) for v in mask.ravel())
    counts.pop(-1, None)
    if not counts:
        return
    ids = sorted(counts)
    vals = [counts[i] for i in ids]
    cmap = plt.get_cmap("tab20")
    n_ids = ids[-1] + 1
    bar_colors = [cmap(i / max(1, n_ids - 1)) for i in ids]

    fig, ax = plt.subplots(figsize=(8.0, 4.5))
    ax.bar(ids, vals, color=bar_colors, edgecolor="k", linewidth=0.4)
    ax.set_xlabel("instance id")
    ax.set_ylabel("pixels")
    ax.set_title(title, fontsize=10)
    ax.set_xticks(ids)
    ax.grid(axis="y", linestyle=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)


def render_for_cloud(cloud_dir: Path) -> None:
    csv = next(cloud_dir.glob("branch_instances_*.txt"), None)
    if csv is None:
        print(f"no instance mask in {cloud_dir}")
        return
    mask = load_mask(csv)
    name = cloud_dir.name
    n_inst = int(mask.max()) + 1 if mask.max() >= 0 else 0
    n_pixels = int((mask >= 0).sum())
    print(f"{name}: mask {mask.shape}, {n_inst} instances, "
          f"{n_pixels} assigned pixels")

    label_out = cloud_dir / "instance_mask_colored.png"
    render_label_map(
        mask, label_out,
        f"{name} -- branch_segmenter instance_mask  "
        f"({n_inst} ids, {n_pixels} pixels)",
    )
    print(f"  -> {label_out.relative_to(REPO)}")

    hist_out = cloud_dir / "instance_histogram.png"
    render_histogram(
        mask, hist_out,
        f"{name} -- pixels per instance id",
    )
    print(f"  -> {hist_out.relative_to(REPO)}")


def main() -> None:
    base = SAMPLES / "branch_segmenter"
    if not base.exists():
        raise SystemExit(f"missing {base}")
    for cloud_dir in sorted(p for p in base.iterdir() if p.is_dir()):
        render_for_cloud(cloud_dir)


if __name__ == "__main__":
    main()
