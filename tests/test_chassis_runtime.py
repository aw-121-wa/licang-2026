"""Run real chassis algorithms and motor protocol against a timed UART plant."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ChassisRuntimeTest(unittest.TestCase):
    def test_distance_and_failures(self):
        compiler = shutil.which("gcc")
        self.assertIsNotNone(compiler, "Host gcc is required")
        with tempfile.TemporaryDirectory(prefix="chassis-tests-") as temp:
            binary = str(Path(temp) / ("chassis.exe" if os.name == "nt" else "chassis"))
            args = [compiler, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror"]
            for directory in ("User/Algorithm", "User/BSP", "User/Device/imu",
                              "User/Config", "tests/host"):
                args += ["-I", directory]
            args += ["tests/host/chassis_runtime.c", "User/Algorithm/motion_control.c",
                     "User/Algorithm/mecanum_kinematics.c", "User/BSP/motor_control.c",
                     "-lm", "-o", binary]
            compiled = subprocess.run(args, cwd=ROOT, capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
            for scenario in ("forward", "lateral", "short", "slow", "diagonal", "overrun",
                             "jitter", "wrap", "stop", "offline", "uart", "early",
                             "sync_fail", "heading", "invalid", "terminal", "brake", "settle",
                             "settle_stop", "settle_offline", "settle_uart"):
                with self.subTest(scenario=scenario):
                    run = subprocess.run([binary, scenario], capture_output=True, text=True, timeout=10)
                    self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
                    if scenario in ("brake", "settle"):
                        print(run.stdout.strip())
            maximum_error = 0.0
            for distance in (1, 18, 90, 117, 997, 1800):
                for angle in (0, 90, 180, -90, 30, -135):
                    for speed in (25, 85, 130, 220):
                        with self.subTest(distance=distance, angle=angle, speed=speed):
                            run = subprocess.run([binary, "jitter", str(distance), str(angle), str(speed)],
                                                 capture_output=True, text=True, timeout=10)
                            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
                            actual = float(run.stdout.split("distance=")[1].split()[0])
                            maximum_error = max(maximum_error, abs(actual - distance))
            print(f"144 distance/direction/speed cases: maximum ideal-model error {maximum_error:.3f} mm")
