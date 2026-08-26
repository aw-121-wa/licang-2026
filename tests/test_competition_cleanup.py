import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CompetitionCleanupContractTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8-sig")

    def test_path_and_legacy_motion_framework_are_removed(self):
        self.assertFalse((ROOT / "App/competition_path.c").exists())
        self.assertFalse((ROOT / "App/competition_path.h").exists())
        motion_h = self.read("Motor/motion_control.h")
        motion_c = self.read("Motor/motion_control.c")
        uart_h = self.read("App/uart_command.h")
        self.assertNotIn("MotionControl_RunDefaultSequence", motion_h + motion_c)
        self.assertNotIn("DIAGONAL_TEST_", motion_h + motion_c)
        self.assertNotIn("CHASSIS_CMD_RUN_PATH", uart_h)
        self.assertNotIn("CHASSIS_MODE_PATH", uart_h)
        for symbol in (
            "MotionControl_MoveMm",
            "MotionControl_MovePolarMm",
            "MotionControl_MoveLeftFrontMm",
            "MotionControl_MoveRightFrontMm",
            "MotionControl_MoveLeftRearMm",
            "MotionControl_MoveRightRearMm",
            "MotionControl_MovePolarBlendSegmentMm",
        ):
            self.assertNotIn(symbol, motion_h + motion_c)

    def test_motion_control_owns_body_speed_heading_and_lateral_compensation(self):
        motion_h = self.read("Motor/motion_control.h")
        motion_c = self.read("Motor/motion_control.c")
        gray_c = self.read("App/gray_align.c")
        rz_c = self.read("App/round_pillar.c")
        self.assertIn("MotionControl_SetBodySpeed", motion_h)
        self.assertIn("MotionControl_ResetHeadingReference", motion_h)
        self.assertIn("MotionControl_GetHeadingCorrection", motion_h)
        self.assertIn("LATERAL_FORWARD_COMPENSATION", motion_c)
        self.assertIn("MotionControl_SetBodySpeed", gray_c)
        self.assertIn("MotionControl_GetHeadingCorrection", gray_c)
        self.assertIn("MotionControl_SetBodySpeed", rz_c)
        self.assertIn("MotionControl_GetHeadingCorrection", rz_c)
        self.assertNotIn("MecanumKinematics_Solve", gray_c + rz_c)

    def test_rz_integrates_vision_into_orbit(self):
        rz_h = self.read("App/round_pillar.h")
        rz_c = self.read("App/round_pillar.c")
        servo_h = self.read("App/servo_action.h")

        self.assertIn("SERVO_ACTION_PILLAR_CAMERA_GROUP", servo_h)
        self.assertIn("SERVO_ACTION_PILLAR_GRAB_GROUP", servo_h)
        self.assertIn("SERVO_ACTION_PILLAR_CAMERA_TIMEOUT_MS", servo_h)
        self.assertIn("SERVO_ACTION_PILLAR_GRAB_TIMEOUT_MS", servo_h)
        self.assertRegex(rz_h, r"#define\s+RZ_CAMERA_RAISE_WAIT_MS\s+1000U")
        self.assertRegex(rz_h, r"#define\s+RZ_ORBIT_FORWARD_RPM\s+63\.0f")
        self.assertRegex(rz_h, r"#define\s+RZ_ORBIT_OMEGA_RPM\s+49\.0f")
        self.assertRegex(rz_h, r"#define\s+RZ_CW_TARGET_DEG\s+\(-720\.0f\)")
        self.assertRegex(rz_h, r"#define\s+RZ_CCW_REVERSE_DEG\s+90\.0f")
        self.assertRegex(rz_h, r"#define\s+RZ_ORBIT_TIMEOUT_MS\s+30000U")
        self.assertRegex(rz_h, r"#define\s+RZ_GRAB_COUNT\s+4U")
        self.assertIn("RoundPillar_OrbitAndGrab", rz_c)
        self.assertIn("RoundPillar_HandleDetectedBall", rz_c)
        self.assertNotIn("RoundPillar_WaitForMaixCam", rz_c)
        for token in (
            "WarehouseControl_",
            "SERVO_ACTION_RETURN_GROUP",
            "SERVO_ACTION_GRAB_GROUP",
        ):
            self.assertNotIn(token, rz_h + rz_c)
        for token in (
            "RZ_ORBIT_FORWARD_RPM",
            "RZ_ORBIT_OMEGA_RPM",
            "RZ_CW_TARGET_DEG",
            "RZ_CCW_REVERSE_DEG",
            "RZ_ORBIT_TIMEOUT_MS",
            "ROUND_PILLAR_ERROR_ORBIT_TIMEOUT",
        ):
            self.assertIn(token, rz_h + rz_c)
        self.assertIn("#include \"maixcam_link.h\"", rz_c)
        self.assertIn("MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)", rz_c)
        self.assertIn("MaixCamLink_TakeReply()", rz_c)
        self.assertIn("while (current_yaw > RZ_CW_TARGET_DEG)", rz_c)
        self.assertIn("while (current_yaw < reverse_target_yaw)", rz_c)

        run_start = rz_c.index("RoundPillarStatus RoundPillar_Run")
        run_body = rz_c[run_start:]
        self.assertLess(
            run_body.index("SERVO_ACTION_PILLAR_CAMERA_GROUP"),
            run_body.index("RoundPillar_OrbitAndGrab()"),
        )
        self.assertIn("ServoAction_StartGroupNoWait", run_body)
        self.assertNotIn("ServoAction_RunGroup", run_body)
        orbit_start = rz_c.index("RoundPillar_OrbitAndGrab")
        orbit_body = rz_c[orbit_start:]
        self.assertIn("MaixCamLink_TakeReply()", orbit_body)
        self.assertIn("RoundPillar_HandleDetectedBall(&grab_count)", orbit_body)
        self.assertNotIn("MAIXCAM_REQUEST_TIMEOUT_MS", orbit_body)
        handler_start = rz_c.index("RoundPillar_HandleDetectedBall")
        handler_body = rz_c[handler_start:orbit_start]
        self.assertIn("osDelay(", handler_body)
        self.assertLess(
            handler_body.index("osDelay("),
            handler_body.index("RoundPillar_Stop()"),
        )

        self.assertEqual(
            rz_c.count("SERVO_ACTION_PILLAR_CAMERA_GROUP"), 1
        )
        self.assertEqual(
            rz_c.count("SERVO_ACTION_PILLAR_GRAB_GROUP"), 1
        )

    def test_uart5_keeps_only_competition_commands_in_help_and_status(self):
        uart_c = self.read("App/uart_command.c")
        self.assertIn('"F <mm>', uart_c)
        self.assertIn('"ROT CCW <deg>', uart_c)
        self.assertIn('"BALL', uart_c)
        self.assertIn('"RZ', uart_c)
        self.assertIn('"GRAB', uart_c)
        self.assertIn('"STOP', uart_c)
        self.assertIn('"STATUS', uart_c)
        self.assertIn('"HELP', uart_c)
        for token in ("PATH ", "MAIX_TX", "ARM_MOTION_COUNT", "WAREHOUSE_G2_DONE"):
            self.assertNotIn(token, uart_c)
        for token in ("HEAD_ERR", "HEAD_CORR", "DIST=", "TARGET=", "BALL_STATE", "WAREHOUSE_BALL"):
            self.assertIn(token, uart_c)

    def test_turntable_uses_usart6_and_runs_after_ball_actions(self):
        usart_h = self.read("Core/Inc/usart.h")
        usart_c = self.read("Core/Src/usart.c")
        main_c = self.read("Core/Src/main.c")
        freertos_c = self.read("Core/Src/freertos.c")
        turntable_h = self.read("App/turntable_control.h")
        turntable_c = self.read("App/turntable_control.c")
        warehouse_c = self.read("App/warehouse_control.c")
        round_c = self.read("App/round_pillar.c")
        cangku_h = self.read("Motor/cangku_motor.h")

        self.assertIn("extern UART_HandleTypeDef huart6;", usart_h)
        self.assertIn("void MX_USART6_UART_Init(void);", usart_h)
        self.assertIn("void MX_USART6_UART_Init(void)", usart_c)
        self.assertIn("huart6.Instance = USART6", usart_c)
        self.assertIn("GPIO_PIN_6|GPIO_PIN_7", usart_c)
        self.assertIn("GPIO_AF8_USART6", usart_c)
        self.assertIn("MX_USART6_UART_Init();", main_c)
        self.assertIn("WarehouseControl_Init(&huart6)", freertos_c)
        self.assertNotIn("WarehouseControl_Init(&huart1)", freertos_c)

        self.assertIn("ZDT_MOTOR_ADDR                 0x05U", cangku_h)
        self.assertIn("TURNTABLE_ONE_SLOT_PULSES           1280U", turntable_h)
        self.assertIn("Turntable_MoveOneSlotAndWait(", turntable_h)
        self.assertIn("Turntable_MoveOneSlotAndWait(", turntable_c)
        self.assertIn("Turntable_MoveOneSlotAndWait(", warehouse_c)
        self.assertIn("Turntable_MoveOneSlotAndWait(", round_c)

        group4 = round_c.index("ServoAction_RunGroup(")
        turntable = round_c.index("Turntable_MoveOneSlotAndWait(", group4)
        grab_count = round_c.index("(*grab_count)++", group4)
        self.assertLess(group4, turntable)
        self.assertLess(turntable, grab_count)


if __name__ == "__main__":
    unittest.main()
