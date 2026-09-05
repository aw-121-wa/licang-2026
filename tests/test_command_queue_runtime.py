import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CommandQueueTest(unittest.TestCase):
    def test_accepts_fifo_despite_diagnostic_states(self):
        with tempfile.TemporaryDirectory(prefix="command-tests-") as temp:
            binary = str(Path(temp) / ("queue.exe" if os.name == "nt" else "queue"))
            args = [shutil.which("gcc"), "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                    "-flto", "-fwhole-program"]
            for directory in ("User/Algorithm", "User/Config", "User/Task", "User/Robot",
                              "User/BSP", "User/Device/imu", "User/Device/servo",
                              "User/Device/turntable", "User/Device/rfid", "tests/host"):
                args += ["-I", directory]
            args += ["tests/host/command_queue_runtime.c", "-o", binary]
            result = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([binary], capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
