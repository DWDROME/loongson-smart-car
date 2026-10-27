#!/usr/bin/env python3

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
REMOTE_TOOL = TOOLS_DIR / "car_remote_tool.sh"
PARAM_FILE = TOOLS_DIR / "params_config.local.txt"


class RaceResultRemoteTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.bin_dir = self.root / "bin"
        self.output_dir = self.root / "results"
        self.bin_dir.mkdir()
        self._write_executable(
            self.bin_dir / "ssh",
            "#!/bin/sh\n"
            "if [ \"${FAKE_SSH_FILE_EXISTS:-1}\" = 1 ]; then exit 0; fi\n"
            "exit 1\n",
        )
        self._write_executable(
            self.bin_dir / "scp",
            "#!/bin/sh\n"
            "for last do :; done\n"
            "cp \"$FAKE_RACE_RESULT_FILE\" \"$last\"\n",
        )

    def tearDown(self):
        self.temp_dir.cleanup()

    @staticmethod
    def _write_executable(path: Path, content: str) -> None:
        path.write_text(content, encoding="utf-8")
        path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def _run(self, content: str | None, file_exists: bool = True) -> subprocess.CompletedProcess[str]:
        fixture = self.root / "board_result.log"
        if content is not None:
            fixture.write_text(content, encoding="utf-8")

        env = os.environ.copy()
        env.update(
            {
                "PATH": f"{self.bin_dir}:{env['PATH']}",
                "FAKE_SSH_FILE_EXISTS": "1" if file_exists else "0",
                "FAKE_RACE_RESULT_FILE": str(fixture),
                "LOCAL_RACE_RESULT_DIR": str(self.output_dir),
                "LOCAL_PARAM_FILE": str(PARAM_FILE),
                "SSH_TARGET": "root@192.0.2.1",
            }
        )
        return subprocess.run(
            ["bash", str(REMOTE_TOOL), "race-result"],
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )

    def test_finished_result_is_displayed_and_archived(self):
        content = (
            "version=1\n"
            "status=finished\n"
            "model_id=test_model\n"
            "count=3\n"
            "overflow=0\n"
            "sequence=supplies,vehicle,weapon\n"
        )
        result = self._run(content)
        self.assertIn("本场识别结果：物资 -> 车辆 -> 武器", result.stdout)
        archives = list(self.output_dir.glob("race_result_*.log"))
        self.assertEqual(len(archives), 1)

        self._run(content)
        archives = list(self.output_dir.glob("race_result_*.log"))
        self.assertEqual(len(archives), 2)

    def test_running_result_is_marked_incomplete(self):
        result = self._run(
            "version=1\n"
            "status=running\n"
            "model_id=test_model\n"
            "count=1\n"
            "overflow=0\n"
            "sequence=weapon\n"
        )
        self.assertIn("本场识别结果（未到第二个斑马线）：武器", result.stdout)

    def test_empty_result_is_explicit(self):
        result = self._run(
            "version=1\n"
            "status=finished\n"
            "model_id=test_model\n"
            "count=0\n"
            "overflow=0\n"
            "sequence=\n"
        )
        self.assertIn("本场识别结果：无图片识别记录", result.stdout)

    def test_unknown_class_keeps_raw_file(self):
        result = self._run(
            "version=1\n"
            "status=finished\n"
            "model_id=test_model\n"
            "count=1\n"
            "overflow=0\n"
            "sequence=unknown\n"
        )
        self.assertIn("未知类别", result.stdout)
        self.assertEqual(len(list(self.output_dir.glob("race_result_*.log"))), 1)

    def test_count_over_capacity_is_rejected(self):
        result = self._run(
            "version=1\n"
            "status=finished\n"
            "model_id=test_model\n"
            "count=9\n"
            "overflow=1\n"
            "sequence=weapon,weapon,weapon,weapon,weapon,weapon,weapon,weapon,weapon\n"
        )
        self.assertIn("超过固定容量 8", result.stdout)

    def test_missing_board_file_is_explicit(self):
        result = self._run(None, file_exists=False)
        self.assertIn("本场识别结果不存在", result.stdout)
        self.assertFalse(self.output_dir.exists())


if __name__ == "__main__":
    unittest.main()
