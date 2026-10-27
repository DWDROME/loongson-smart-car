#!/usr/bin/env python3

import os
import stat
import subprocess
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
BISAI_TOOL = TOOLS_DIR / "bisai.sh"


class BisaiToolTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.call_log = self.root / "calls.log"
        self.remote_tool = self.root / "fake_remote.sh"
        self.remote_tool.write_text(
            "#!/usr/bin/env bash\n"
            "printf '%s\\n' \"$*\" >> \"$BISAI_CALL_LOG\"\n",
            encoding="utf-8",
        )
        self.remote_tool.chmod(self.remote_tool.stat().st_mode | stat.S_IXUSR)

    def tearDown(self):
        self.temp_dir.cleanup()

    def run_bisai(self, *args: str, input_text: str | None = None) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env.update(
            {
                "BISAI_REMOTE_TOOL": str(self.remote_tool),
                "BISAI_CALL_LOG": str(self.call_log),
            }
        )
        return subprocess.run(
            ["bash", str(BISAI_TOOL), *args],
            input=input_text,
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )

    def calls(self) -> list[str]:
        if not self.call_log.exists():
            return []
        return self.call_log.read_text(encoding="utf-8").splitlines()

    def test_cli_commands_delegate_to_remote_tool(self):
        expected = {
            "start": "start",
            "stop": "stop",
            "apply": "apply",
            "pull": "pull",
            "result": "race-result",
            "log": "log",
        }
        for command, delegated in expected.items():
            with self.subTest(command=command):
                self.run_bisai(command)
                self.assertEqual(self.calls()[-1], delegated)

    def test_menu_only_exposes_competition_actions(self):
        result = self.run_bisai(input_text="1\n3\n4\n5\nq\n")
        self.assertEqual(self.calls(), ["start", "apply", "pull", "race-result"])
        self.assertIn("开始比赛", result.stdout)
        self.assertIn("写入本地参数到板端", result.stdout)
        self.assertIn("从板端读取参数到本地", result.stdout)
        self.assertIn("查看本场识别结果", result.stdout)
        self.assertNotIn("调速度环", result.stdout)
        self.assertNotIn("设置相机曝光", result.stdout)


if __name__ == "__main__":
    unittest.main()
