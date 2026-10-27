from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


TOOL_PATH = Path(__file__).parents[1] / "prepare_stele_dataset.py"
SPEC = importlib.util.spec_from_file_location("prepare_stele_dataset", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_specs(expected_count: int = 1):
    return tuple(
        MODULE.ClassSpec(parent, leaf, fine, business, expected_count)
        for parent, leaf, fine, business in (
            ("武器", "A-枪支", "firearms", "weapon"),
            ("武器", "B-爆炸物", "explosive", "weapon"),
            ("物资", "C-急救包", "first_aid_kit", "supplies"),
            ("物资", "D-望远镜", "telescope", "supplies"),
            ("载具", "E-装甲车", "armoured_car", "vehicle"),
            ("载具", "F-救护车", "ambulance", "vehicle"),
        )
    )


def write_image(path: Path, width: int = 19, height: int = 13) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    x = np.arange(width, dtype=np.uint8)[None, :]
    y = np.arange(height, dtype=np.uint8)[:, None]
    image = np.stack(
        (
            np.broadcast_to(x, (height, width)),
            np.broadcast_to(y, (height, width)),
            (np.broadcast_to(x, (height, width)) + np.broadcast_to(y, (height, width))) % 255,
        ),
        axis=2,
    )
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"cannot create fixture: {path}")


def build_fixture(root: Path, specs, nonstandard: bool = False) -> None:
    for index, spec in enumerate(specs):
        width = 23 if nonstandard and index == 0 else 19
        height = 11 if nonstandard and index == 0 else 13
        write_image(root / spec.parent_dir / spec.leaf_dir / "sample.jpg", width, height)


class PrepareSteleDatasetTest(unittest.TestCase):
    def test_prepares_all_mappings_and_exact_inter_linear_pixels(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = test_specs()
            build_fixture(source, specs)

            summary = MODULE.prepare_dataset(source, output, specs=specs)

            self.assertEqual(summary["source_image_count"], 6)
            self.assertEqual(
                summary["business_counts"],
                {"supplies": 2, "vehicle": 2, "weapon": 2},
            )
            source_image = cv2.imread(
                str(source / "物资" / "C-急救包" / "sample.jpg"), cv2.IMREAD_COLOR
            )
            expected = cv2.resize(source_image, (64, 64), interpolation=cv2.INTER_LINEAR)
            actual = cv2.imread(
                str(output / "supplies" / "first_aid_kit__sample.png"),
                cv2.IMREAD_COLOR,
            )
            self.assertTrue(np.array_equal(actual, expected))
            self.assertTrue((output / "manifest.csv").is_file())
            self.assertTrue((output / "summary.json").is_file())

    def test_rejects_unknown_top_level_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = test_specs()
            build_fixture(source, specs)
            (source / "未知").mkdir()
            with self.assertRaisesRegex(ValueError, "top-level directories mismatch"):
                MODULE.prepare_dataset(source, output, specs=specs)
            self.assertFalse(output.exists())

    def test_rejects_output_inside_source_tree(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / "source"
            specs = test_specs()
            build_fixture(source, specs)
            with self.assertRaisesRegex(ValueError, "outside the read-only source tree"):
                MODULE.prepare_dataset(source, source / "derived", specs=specs)

    def test_rejects_unexpected_source_root_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            specs = test_specs()
            build_fixture(source, specs)
            (source / "notes.txt").write_text("unexpected", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unsupported entries in source root"):
                MODULE.prepare_dataset(source, root / "output", specs=specs)

    def test_rejects_count_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = test_specs(expected_count=2)
            build_fixture(source, specs)
            with self.assertRaisesRegex(ValueError, "image count mismatch"):
                MODULE.prepare_dataset(source, output, specs=specs)

    def test_rejects_undecodable_image(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = test_specs()
            build_fixture(source, specs)
            bad = source / "武器" / "A-枪支" / "sample.jpg"
            bad.write_bytes(b"not an image")
            with self.assertRaisesRegex(ValueError, "cannot decode as BGR image"):
                MODULE.prepare_dataset(source, output, specs=specs)

    def test_rejects_duplicate_derived_name(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = (
                MODULE.ClassSpec("武器", "A", "same", "weapon", 1),
                MODULE.ClassSpec("武器", "B", "same", "weapon", 1),
            )
            build_fixture(source, specs)
            with self.assertRaisesRegex(ValueError, "duplicate derived output path"):
                MODULE.prepare_dataset(source, output, specs=specs)

    def test_records_nonstandard_source_dimensions(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source"
            output = root / "output"
            specs = test_specs()
            build_fixture(source, specs, nonstandard=True)
            summary = MODULE.prepare_dataset(source, output, specs=specs)
            self.assertEqual(summary["input_dimensions"], {"19x13": 5, "23x11": 1})


if __name__ == "__main__":
    unittest.main()
