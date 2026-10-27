#!/usr/bin/env python3
"""Prepare the official six-class stele dataset for the fixed 64x64 NCNN input."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

try:
    import cv2
except ImportError as exc:
    raise SystemExit(
        "This tool needs OpenCV. Run it with the vision-training venv."
    ) from exc


OUTPUT_WIDTH = 64
OUTPUT_HEIGHT = 64
IMAGE_SUFFIXES = {".jpg", ".jpeg"}


@dataclass(frozen=True)
class ClassSpec:
    parent_dir: str
    leaf_dir: str
    fine_label: str
    business_label: str
    expected_count: int


OFFICIAL_CLASS_SPECS = (
    ClassSpec("武器", "A-枪支", "firearms", "weapon", 54),
    ClassSpec("武器", "B-爆炸物", "explosive", "weapon", 50),
    ClassSpec("物资", "C-急救包", "first_aid_kit", "supplies", 51),
    ClassSpec("物资", "D-望远镜", "telescope", "supplies", 53),
    ClassSpec("载具", "E-装甲车", "armoured_car", "vehicle", 41),
    ClassSpec("载具", "F-救护车", "ambulance", "vehicle", 55),
)


@dataclass(frozen=True)
class SourceItem:
    path: Path
    relative_path: Path
    output_relative_path: Path
    fine_label: str
    business_label: str
    width: int
    height: int
    source_sha256: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Strictly validate the official six-class dataset and resize every full image "
            "to a lossless 64x64 BGR PNG evaluation input."
        )
    )
    parser.add_argument("--source", type=Path, required=True, help="Read-only source root.")
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="New output directory. Existing paths are never overwritten.",
    )
    parser.add_argument(
        "--archive",
        type=Path,
        help="Optional source ZIP recorded in summary.json.",
    )
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def tree_digest(items: list[SourceItem]) -> str:
    digest = hashlib.sha256()
    for item in sorted(items, key=lambda value: value.relative_path.as_posix()):
        line = f"{item.source_sha256}  {item.relative_path.as_posix()}\n"
        digest.update(line.encode("utf-8"))
    return digest.hexdigest()


def validate_directory_names(source: Path, specs: tuple[ClassSpec, ...]) -> None:
    expected_parents = {spec.parent_dir for spec in specs}
    unexpected_root_entries = sorted(
        path.name for path in source.iterdir() if not path.is_dir()
    )
    if unexpected_root_entries:
        raise ValueError(f"unsupported entries in source root: {unexpected_root_entries}")
    actual_parents = {path.name for path in source.iterdir() if path.is_dir()}
    if actual_parents != expected_parents:
        raise ValueError(
            f"source top-level directories mismatch: expected={sorted(expected_parents)} "
            f"actual={sorted(actual_parents)}"
        )

    for parent_name in sorted(expected_parents):
        expected_leaves = {
            spec.leaf_dir for spec in specs if spec.parent_dir == parent_name
        }
        parent = source / parent_name
        unexpected_parent_entries = sorted(
            path.name for path in parent.iterdir() if not path.is_dir()
        )
        if unexpected_parent_entries:
            raise ValueError(
                f"unsupported entries under {parent_name}: {unexpected_parent_entries}"
            )
        actual_leaves = {path.name for path in parent.iterdir() if path.is_dir()}
        if actual_leaves != expected_leaves:
            raise ValueError(
                f"leaf directories mismatch under {parent_name}: "
                f"expected={sorted(expected_leaves)} actual={sorted(actual_leaves)}"
            )


def collect_source_items(
    source: Path, specs: tuple[ClassSpec, ...] = OFFICIAL_CLASS_SPECS
) -> list[SourceItem]:
    if not source.is_dir():
        raise ValueError(f"source directory does not exist: {source}")
    validate_directory_names(source, specs)

    items: list[SourceItem] = []
    output_paths: set[str] = set()
    for spec in specs:
        leaf = source / spec.parent_dir / spec.leaf_dir
        unsupported = sorted(
            path.name
            for path in leaf.iterdir()
            if not path.is_file() or path.suffix.lower() not in IMAGE_SUFFIXES
        )
        if unsupported:
            raise ValueError(f"unsupported entries in {leaf}: {unsupported}")

        paths = sorted(path for path in leaf.iterdir() if path.is_file())
        if len(paths) != spec.expected_count:
            raise ValueError(
                f"image count mismatch for {spec.fine_label}: "
                f"expected={spec.expected_count} actual={len(paths)}"
            )

        for path in paths:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None or image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"cannot decode as BGR image: {path}")
            output_relative = Path(spec.business_label) / (
                f"{spec.fine_label}__{path.stem}.png"
            )
            output_key = output_relative.as_posix()
            if output_key in output_paths:
                raise ValueError(f"duplicate derived output path: {output_key}")
            output_paths.add(output_key)
            items.append(
                SourceItem(
                    path=path,
                    relative_path=path.relative_to(source),
                    output_relative_path=output_relative,
                    fine_label=spec.fine_label,
                    business_label=spec.business_label,
                    width=int(image.shape[1]),
                    height=int(image.shape[0]),
                    source_sha256=sha256_file(path),
                )
            )
    return items


def write_manifest(path: Path, rows: list[dict[str, str | int]]) -> None:
    fields = (
        "source_relative_path",
        "source_sha256",
        "source_width",
        "source_height",
        "fine_label",
        "expected_label",
        "output_relative_path",
        "output_sha256",
    )
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def prepare_dataset(
    source: Path,
    output: Path,
    archive: Path | None = None,
    specs: tuple[ClassSpec, ...] = OFFICIAL_CLASS_SPECS,
) -> dict[str, object]:
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    if output == source or source in output.parents:
        raise ValueError(f"output must be outside the read-only source tree: {output}")
    if output.exists():
        raise ValueError(f"output path already exists; refusing to overwrite: {output}")
    if archive is not None:
        archive = archive.expanduser().resolve()
        if not archive.is_file():
            raise ValueError(f"archive does not exist: {archive}")

    items = collect_source_items(source, specs)
    staging = output.with_name(f"{output.name}.tmp-{os.getpid()}")
    if staging.exists():
        raise ValueError(f"staging path already exists: {staging}")

    rows: list[dict[str, str | int]] = []
    try:
        for label in sorted({spec.business_label for spec in specs}):
            (staging / label).mkdir(parents=True, exist_ok=False)

        for item in items:
            if sha256_file(item.path) != item.source_sha256:
                raise ValueError(f"source changed during preparation: {item.path}")
            image = cv2.imread(str(item.path), cv2.IMREAD_COLOR)
            if image is None or image.shape != (item.height, item.width, 3):
                raise ValueError(f"source decode changed during preparation: {item.path}")
            resized = cv2.resize(
                image,
                (OUTPUT_WIDTH, OUTPUT_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )
            if resized.shape != (OUTPUT_HEIGHT, OUTPUT_WIDTH, 3):
                raise ValueError(f"unexpected resize result for: {item.path}")

            output_path = staging / item.output_relative_path
            if not cv2.imwrite(
                str(output_path), resized, [cv2.IMWRITE_PNG_COMPRESSION, 3]
            ):
                raise OSError(f"failed to write derived image: {output_path}")
            verified = cv2.imread(str(output_path), cv2.IMREAD_COLOR)
            if verified is None or verified.shape != resized.shape or not (verified == resized).all():
                raise ValueError(f"derived PNG verification failed: {output_path}")

            rows.append(
                {
                    "source_relative_path": item.relative_path.as_posix(),
                    "source_sha256": item.source_sha256,
                    "source_width": item.width,
                    "source_height": item.height,
                    "fine_label": item.fine_label,
                    "expected_label": item.business_label,
                    "output_relative_path": item.output_relative_path.as_posix(),
                    "output_sha256": sha256_file(output_path),
                }
            )

        manifest_path = staging / "manifest.csv"
        write_manifest(manifest_path, rows)
        dimension_counts = Counter(f"{item.width}x{item.height}" for item in items)
        class_counts = Counter(item.business_label for item in items)
        fine_counts = Counter(item.fine_label for item in items)
        output_tree = hashlib.sha256()
        for row in sorted(rows, key=lambda value: str(value["output_relative_path"])):
            line = f"{row['output_sha256']}  {row['output_relative_path']}\n"
            output_tree.update(line.encode("utf-8"))
        for item in items:
            if sha256_file(item.path) != item.source_sha256:
                raise ValueError(f"source changed before output finalization: {item.path}")
        summary: dict[str, object] = {
            "tool": "prepare_stele_dataset",
            "source_read_only": True,
            "source_root": str(source),
            "source_image_count": len(items),
            "source_tree_sha256": tree_digest(items),
            "archive": None if archive is None else {
                "path": str(archive),
                "sha256": sha256_file(archive),
            },
            "input_dimensions": dict(sorted(dimension_counts.items())),
            "business_counts": dict(sorted(class_counts.items())),
            "fine_counts": dict(sorted(fine_counts.items())),
            "resize": {
                "mode": "full_image",
                "width": OUTPUT_WIDTH,
                "height": OUTPUT_HEIGHT,
                "interpolation": "cv2.INTER_LINEAR",
                "crop": False,
                "padding": False,
                "augmentation": False,
            },
            "manifest_sha256": sha256_file(manifest_path),
            "output_tree_sha256": output_tree.hexdigest(),
        }
        (staging / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        staging.rename(output)
        return summary
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    args = parse_args()
    summary = prepare_dataset(args.source, args.output, args.archive)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2)
