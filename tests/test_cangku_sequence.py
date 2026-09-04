import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CangkuSequenceContractTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8-sig")

    def test_command_is_queued_and_executed_by_chassis_task(self):
        uart_h = self.read("App/uart_command.h")
        uart_c = self.read("App/uart_command.c")
        freertos_c = self.read("Core/Src/freertos.c")

        self.assertIn("CHASSIS_CMD_CANGKU", uart_h)
        self.assertRegex(uart_c, r'if \(strcmp\(command, "CANGKU"\) == 0\)')
        self.assertIn('"OK CANGKU\\r\\n"', uart_c)
        self.assertIn("command.type == CHASSIS_CMD_CANGKU", freertos_c)
        self.assertIn("CangkuSequence_Run()", freertos_c)

    def test_cangku_uses_180_degree_rotation_and_exact_gray_alignment(self):
        cangku_h = self.read("App/cangku_task.h")
        cangku_c = self.read("App/cangku_task.c")
        gray_c = self.read("App/gray_align.c")

        self.assertIn("CANGKU_STATE_ROTATE", cangku_h)
        self.assertIn("MotionControl_RotateDeg(180.0f)", cangku_c)
        self.assertIn("GrayAlign_Run()", cangku_c)
        self.assertIn("sample.mid2 == 0U", gray_c)
        self.assertIn("sample.in2 != 0U", gray_c)
        self.assertIn("sample.in1 != 0U", gray_c)
        self.assertIn("sample.mid1 == 0U", gray_c)

    def test_cangku_moves_left_50mm_after_gray_alignment(self):
        cangku_h = self.read("App/cangku_task.h")
        cangku_c = self.read("App/cangku_task.c")

        self.assertIn("CANGKU_AFTER_LINE_LEFT_DISTANCE_MM", cangku_c)
        self.assertIn("CANGKU_STATE_AFTER_LINE_LEFT", cangku_h)
        gray_position = cangku_c.index("gray_status = GrayAlign_Run();")
        left_position = cangku_c.index(
            "CANGKU_AFTER_LINE_LEFT_DISTANCE_MM", gray_position
        )
        first_backward_position = cangku_c.index(
            "CangkuSequence_RunMove(180.0f)", left_position
        )
        self.assertLess(gray_position, left_position)
        self.assertLess(left_position, first_backward_position)
        self.assertIn("90.0f", cangku_c[left_position:left_position + 1800])

    def test_cangku_preserves_imu_failure_as_a_distinct_status(self):
        cangku_h = self.read("App/cangku_task.h")
        cangku_c = self.read("App/cangku_task.c")
        freertos_c = self.read("Core/Src/freertos.c")

        self.assertIn("CANGKU_STATUS_ERROR_IMU", cangku_h)
        self.assertIn("MOTION_ERROR_IMU_LOST", cangku_c)
        self.assertIn("CANGKU_STATUS_ERROR_IMU", freertos_c)

    def test_cangku_action_order_and_six_reverse_turns(self):
        cangku_c = self.read("App/cangku_task.c")
        run_body = cangku_c.split("CangkuSequenceStatus CangkuSequence_Run(void)", 1)[1]
        self.assertEqual(len(re.findall(
            r"CangkuSequence_RunReverseTurntable\(\)", run_body)), 6)

        expected_order = (
            "Cangku_Action13",
            "Cangku_Action14",
            "Cangku_Action14",
            "Cangku_Action13",
            "Cangku_Action14",
            "Cangku_Action13",
            "Cangku_Action15",
            "Cangku_Action13",
            "Cangku_Action15",
            "Cangku_Action13",
            "Cangku_Action15",
            "Cangku_Action13",
        )
        positions = []
        cursor = 0
        for action in expected_order:
            position = run_body.find(action, cursor)
            self.assertNotEqual(position, -1, action)
            positions.append(position)
            cursor = position + len(action)

        self.assertEqual(len(re.findall(
            r"Turntable_MoveOneSlotReverseAndWait", cangku_c)), 1)
        self.assertEqual(positions, sorted(positions))

    def test_reverse_turntable_keeps_slot_parameters_and_opposes_configured_direction(self):
        turntable_h = self.read("App/turntable_control.h")
        turntable_c = self.read("App/turntable_control.c")

        self.assertIn("Turntable_MoveOneSlotReverseAndWait", turntable_h)
        self.assertIn("TURNTABLE_ONE_SLOT_PULSES", turntable_c)
        self.assertIn("TURNTABLE_MOVE_SPEED_RPM", turntable_c)
        self.assertIn("TURNTABLE_MOVE_ACCELERATION", turntable_c)
        self.assertRegex(
            turntable_c,
            r"TURNTABLE_SLOT_DIRECTION == ZDT_DIR_CW.*ZDT_DIR_CCW",
        )

    def test_stop_cleanup_releases_servo_and_turntable_for_later_commands(self):
        cangku_c = self.read("App/cangku_task.c")
        canceled_body = cangku_c.split("canceled:", 1)[1].split(
            "failed:", 1
        )[0]

        self.assertIn("ServoAction_SequenceState = SERVO_SEQUENCE_DONE", canceled_body)
        self.assertIn("Turntable_Stop()", canceled_body)

    def test_cangku_is_a_path_step_after_existing_path(self):
        path_h = self.read("App/path_sequence.h")
        path_c = self.read("App/path_sequence.c")
        freertos_c = self.read("Core/Src/freertos.c")

        self.assertIn("#include \"cangku_task.h\"", path_h)
        self.assertIn("PATH_STEP_CANGKU", path_h)
        self.assertIn("PATH_SEQUENCE_CANGKU", path_h)
        self.assertIn("PATH_SEQUENCE_ERROR_CANGKU", path_h)
        self.assertIn("PATH_STEP_CANGKU", path_c)
        self.assertIn("PathSequence_LastCangkuStatus = CangkuSequence_Run();", path_c)
        self.assertIn("PATH_SEQUENCE_ERROR_CANGKU", path_c)
        self.assertIn("CangkuSequence_Run()", freertos_c)
        self.assertIn("PATH_SEQUENCE_ERROR_CANGKU", freertos_c)
        self.assertIn("App/cangku_task.c", self.read("CMakeLists_armcc.txt"))
        self.assertIn("cangku_task.c", self.read("MDK-ARM/chassis_motor.uvprojx"))

        table = path_c.split(
            "static const PathStep PathSequence_CommandQueue[]", 1
        )[1].split("#define PATH_SEQUENCE_STEP_COUNT", 1)[0]
        self.assertLess(table.index("/* STEP 11"), table.index("/* STEP 12"))


if __name__ == "__main__":
    unittest.main()
