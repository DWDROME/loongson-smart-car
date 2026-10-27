#!/usr/bin/env python3
"""Compare raw board exposure with the epoch171 training-domain white track."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit(
        "This tool needs OpenCV and NumPy. Run it with the vision-training venv."
    ) from exc


SOURCE_WIDTH = 320
SOURCE_HEIGHT = 240
REGION_X0 = 70
REGION_Y0 = 12
REGION_X1 = 250
REGION_Y1 = 220
MAX_SATURATION = 55
MIN_LUMA = 35.0
MIN_SELECTED_PIXELS = 800
DEFAULT_SPLITS = ("train", "val")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure the low-saturation white-track luma of 320x240 raw frames. "
            "This tool is read-only: it never changes camera, model, or vehicle parameters."
        )
    )
    parser.add_argument("--manifest", type=Path, required=True,
                        help="Clean-v3 split manifest used by the deployed model.")
    parser.add_argument("--raw-root", type=Path, required=True,
                        help="Root joined with manifest source_relpath.")
    parser.add_argument(
        "--board-dir",
        action="append",
        default=[],
        metavar="NAME=DIR",
        help="Raw 320x240 board-frame directory for one exposure; repeat per exposure.",
    )
    parser.add_argument(
        "--samples-per-sequence",
        type=int,
        default=5,
        help="Evenly spaced training frames sampled from each sequence (default: 5).",
    )
    parser.add_argument(
        "--discard-board-first",
        type=int,
        default=1,
        help="Discard this many chronologically first board frames per exposure (default: 1).",
    )
    parser.add_argument("--output", type=Path,
                        help="Optional JSON report path; stdout always receives the report.")
    return parser.parse_args()


def read_manifest(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"manifest does not exist: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"split", "source_relpath", "sequence_id", "frame_index"}
        fields = set(reader.fieldnames or ())
        missing = sorted(required - fields)
        if missing:
            raise ValueError(f"manifest is missing columns: {', '.join(missing)}")
        return list(reader)


def evenly_spaced_rows(rows: list[dict[str, str]], count: int) -> list[dict[str, str]]:
    if count <= 0:
        raise ValueError("--samples-per-sequence must be > 0")
    by_sequence: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_sequence[row["sequence_id"]].append(row)

    selected: list[dict[str, str]] = []
    for sequence_id in sorted(by_sequence):
        sequence = sorted(
            by_sequence[sequence_id], key=lambda item: int(item["frame_index"])
        )
        last_index = len(sequence) - 1
        indices = {round(index * last_index / (count - 1)) for index in range(count)} if count > 1 else {0}
        selected.extend(sequence[index] for index in sorted(indices))
    return selected


def white_track_measurement(image_path: Path) -> dict[str, float | int]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode image: {image_path}")
    if image.shape != (SOURCE_HEIGHT, SOURCE_WIDTH, 3):
        raise ValueError(
            f"raw-frame contract is 320x240 BGR, got {image.shape[1]}x{image.shape[0]} at {image_path}"
        )

    region = image[REGION_Y0:REGION_Y1, REGION_X0:REGION_X1]
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    blue, green, red = cv2.split(region)
    luma = (
        0.114 * blue.astype(np.float32)
        + 0.587 * green.astype(np.float32)
        + 0.299 * red.astype(np.float32)
    )
    mask = (hsv[:, :, 1] <= MAX_SATURATION) & (luma >= MIN_LUMA)
    values = luma[mask]
    if values.size < MIN_SELECTED_PIXELS:
        raise ValueError(
            f"white-track mask kept only {values.size} pixels at {image_path}; expected >= {MIN_SELECTED_PIXELS}"
        )

    return {
        "p10": float(np.percentile(values, 10)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
        "mean": float(values.mean()),
        "mask_pixels": int(values.size),
        "mask_fraction": float(mask.mean()),
    }


def summarize(measurements: Iterable[dict[str, float | int]]) -> dict[str, Any]:
    items = list(measurements)
    if not items:
        raise ValueError("no valid frames were measured")
    fields = ("p10", "p50", "p90", "mean", "mask_fraction")
    summary: dict[str, Any] = {"frame_count": len(items)}
    for field in fields:
        values = np.asarray([float(item[field]) for item in items], dtype=np.float64)
        summary[field] = {
            "median": float(np.median(values)),
            "p10": float(np.percentile(values, 10)),
            "p25": float(np.percentile(values, 25)),
            "p75": float(np.percentile(values, 75)),
            "p90": float(np.percentile(values, 90)),
            "mean": float(values.mean()),
        }
    return summary


def parse_board_dir(value: str) -> tuple[str, Path]:
    name, separator, path_text = value.partition("=")
    if not separator or not name or not path_text:
        raise ValueError(f"--board-dir must be NAME=DIR, got: {value}")
    path = Path(path_text)
    if not path.is_dir():
        raise ValueError(f"board directory does not exist: {path}")
    return name, path


def image_paths(directory: Path) -> list[Path]:
    suffixes = {".png", ".jpg", ".jpeg", ".bmp"}
    paths = sorted(path for path in directory.rglob("*") if path.suffix.lower() in suffixes)
    if not paths:
        raise ValueError(f"no image files found in: {directory}")
    return paths


def main() -> int:
    args = parse_args()
    if args.samples_per_sequence <= 0 or args.discard_board_first < 0:
        raise ValueError("sampling counts must be non-negative, and samples-per-sequence must be > 0")

    rows = read_manifest(args.manifest)
    training_rows = [row for row in rows if row["split"] in DEFAULT_SPLITS]
    if not training_rows:
        raise ValueError("manifest has no train/val rows; test is intentionally excluded")

    selected_rows = evenly_spaced_rows(training_rows, args.samples_per_sequence)
    seen_paths: set[str] = set()
    training_measurements: list[dict[str, float | int]] = []
    for row in selected_rows:
        relpath = row["source_relpath"]
        if relpath in seen_paths:
            continue
        seen_paths.add(relpath)
        training_measurements.append(white_track_measurement(args.raw_root / relpath))
    training_summary = summarize(training_measurements)

    board_reports: list[dict[str, Any]] = []
    for board_arg in args.board_dir:
        name, directory = parse_board_dir(board_arg)
        paths = image_paths(directory)[args.discard_board_first:]
        if not paths:
            raise ValueError(f"all board frames were discarded for {name}")
        measurements = [white_track_measurement(path) for path in paths]
        board_reports.append({
            "name": name,
            "directory": str(directory),
            "discarded_first_frames": args.discard_board_first,
            "summary": summarize(measurements),
        })

    target = training_summary["p50"]
    target_min = float(target["p10"])
    target_max = float(target["p90"])
    for board in board_reports:
        p50 = float(board["summary"]["p50"]["median"])
        board["p50_distance_to_training_median"] = abs(p50 - float(target["median"]))
        board["p50_within_training_p10_p90"] = target_min <= p50 <= target_max

    closest = min(board_reports, key=lambda item: item["p50_distance_to_training_median"], default=None)
    report: dict[str, Any] = {
        "tool": "white_track_exposure_calibration",
        "read_only": True,
        "contract": {
            "source_shape_bgr": [SOURCE_WIDTH, SOURCE_HEIGHT, 3],
            "region_xyxy": [REGION_X0, REGION_Y0, REGION_X1, REGION_Y1],
            "selection": f"HSV saturation <= {MAX_SATURATION}; luma >= {MIN_LUMA}",
            "luma": "0.114*B + 0.587*G + 0.299*R",
            "test_rows_used": 0,
        },
        "training": {
            "manifest": str(args.manifest),
            "splits": list(DEFAULT_SPLITS),
            "selected_unique_raw_frames": len(training_measurements),
            "summary": training_summary,
        },
        "board_exposures": board_reports,
        "recommendation": None if closest is None else {
            "closest_capture": closest["name"],
            "target_training_p50_median": target["median"],
            "target_training_p50_p10_p90": [target_min, target_max],
            "closest_p50_median": closest["summary"]["p50"]["median"],
            "within_training_band": closest["p50_within_training_p10_p90"],
            "action": (
                "choose this static exposure candidate, then run labeled current-camera validation"
                if closest["p50_within_training_p10_p90"]
                else "no sampled exposure aligns with the training p50 band; do not compensate with runtime gain or confidence threshold"
            ),
        },
    }
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
