"""Execute production C; stubs replace only HAL, timing and physical devices."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RfidRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if not compiler:
            raise RuntimeError("Host gcc is required for RFID behavioral tests")
        cls.temp = tempfile.TemporaryDirectory(prefix="rfid-tests-")
        cls.binary = Path(cls.temp.name) / ("rfid.exe" if os.name == "nt" else "rfid")
        args = [compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror"]
        for directory in ("tests/host", "User/Config", "User/BSP", "User/Device/rfid",
                          "User/Robot", "User/Device/servo", "User/Device/camera",
                          "User/Device/turntable", "User/Algorithm"):
            args += ["-I", directory]
        args += [p for p in ("tests/host/rfid_runtime.c",
                 "User/Device/rfid/rfid.c", "User/Robot/ball_sequence.c")]
        result = subprocess.run(args + ["-o", str(cls.binary)], cwd=ROOT, capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_runtime_scenarios(self):
        for scenario in ("raw", "frames", "variants", "rearm", "first", "corrupt", "clamp", "duplicate",
                         "retry", "turn_fail", "stop", "timeout", "capacity"):
            with self.subTest(scenario=scenario):
                result = subprocess.run([str(self.binary), scenario], capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
