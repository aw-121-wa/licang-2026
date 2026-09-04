import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class CompetitionCleanupContractTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8-sig")

    def test_fixed_path_and_legacy_motion_framework_contract(self):
        self.assertFalse((ROOT / "App/competition_path.c").exists())
        self.assertFalse((ROOT / "App/competition_path.h").exists())
        self.assertTrue((ROOT / "User/Robot/path_sequence.c").exists())
        self.assertTrue((ROOT / "User/Robot/path_sequence.h").exists())
        motion_h = self.read("User/Algorithm/motion_control.h")
        motion_c = self.read("User/Algorithm/motion_control.c")
        uart_h = self.read("User/Task/uart_command.h")
        self.assertNotIn("MotionControl_RunDefaultSequence", motion_h + motion_c)
        self.assertNotIn("DIAGONAL_TEST_", motion_h + motion_c)
        self.assertIn("CHASSIS_CMD_PATH", uart_h)
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
        motion_h = self.read("User/Algorithm/motion_control.h")
        motion_c = self.read("User/Algorithm/motion_control.c")
        gray_c = self.read("User/Algorithm/gray_align.c")
        rz_c = self.read("User/Robot/round_pillar.c")
        self.assertIn("MotionControl_SetBodySpeed", motion_h)
        self.assertIn("MotionControl_ResetHeadingReference", motion_h)
        self.assertIn("MotionControl_GetHeadingCorrection", motion_h)
        self.assertIn("LATERAL_FORWARD_COMPENSATION", motion_c)
        self.assertIn("MotionControl_SetBodySpeed", gray_c)
        self.assertIn("MotionControl_GetHeadingCorrection", gray_c)
        self.assertIn("MotionControl_SetBodySpeed", rz_c)
        self.assertIn("MotionControl_GetHeadingCorrection", rz_c)
        self.assertNotIn("MecanumKinematics_Solve", gray_c + rz_c)

    def test_jy60_uses_usart2_9600_and_legacy_parser_contract(self):
        usart_c = self.read("Core/Src/usart.c")
        ioc = self.read("chassis_motor.ioc")
        imu_h = self.read("User/Device/imu/jy61p.h")
        imu_c = self.read("User/Device/imu/jy61p.c")
        project = self.read("PROJECT.md")
        requirements = self.read("REQUIREMENTS.md")

        self.assertIn("huart2.Init.BaudRate = 9600;", usart_c)
        self.assertNotIn("huart2.Init.BaudRate = 115200;", usart_c)
        self.assertIn("huart2.Init.WordLength = UART_WORDLENGTH_8B;", usart_c)
        self.assertIn("huart2.Init.StopBits = UART_STOPBITS_1;", usart_c)
        self.assertIn("huart2.Init.Parity = UART_PARITY_NONE;", usart_c)
        self.assertIn("huart2.Init.Mode = UART_MODE_TX_RX;", usart_c)
        self.assertIn("huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;", usart_c)
        self.assertIn("PD5     ------> USART2_TX", usart_c)
        self.assertIn("PD6     ------> USART2_RX", usart_c)
        self.assertIn("USART2.BaudRate=9600", ioc)
        self.assertNotIn("USART2.BaudRate=115200", ioc)
        self.assertIn("USART2.IPParameters=VirtualMode-Asynchronous,BaudRate", ioc)
        self.assertIn("PD5.Signal=USART2_TX", ioc)
        self.assertIn("PD6.Signal=USART2_RX", ioc)
        self.assertIn("UART7.BaudRate=9600", ioc)

        self.assertIn("Legacy Jy61P_* API", imu_h)
        for token in (
            "0x53U",
            "rx_buffer[10]",
            "Jy61P_UpdateContinuousYaw",
            "Jy61P_LastUpdateTick",
            "HAL_UART_Receive_IT",
            "delta > 180.0f",
            "delta < -180.0f",
        ):
            self.assertIn(token, imu_c)
        self.assertIn("JY60", project + requirements)
        self.assertNotIn("JY901S", project + requirements + imu_h)
        self.assertIn("Jy61P_*", project + requirements)

    def test_rz_integrates_vision_into_orbit(self):
        rz_h = self.read("User/Robot/round_pillar.h")
        rz_c = self.read("User/Robot/round_pillar.c")
        servo_h = self.read("User/Device/servo/servo_action.h")
        config_h = self.read("User/Config/robot_config.h")

        self.assertIn("SERVO_ACTION_PILLAR_CAMERA_GROUP", config_h)
        self.assertIn("SERVO_ACTION_PILLAR_GRAB_GROUP", config_h)
        self.assertIn("SERVO_ACTION_PILLAR_CAMERA_TIMEOUT_MS", config_h)
        self.assertIn("SERVO_ACTION_PILLAR_GRAB_TIMEOUT_MS", config_h)
        self.assertRegex(config_h, r"#define\s+RZ_CAMERA_RAISE_WAIT_MS\s+1000U")
        self.assertRegex(config_h, r"#define\s+RZ_ORBIT_FORWARD_RPM\s+62\.0f")
        self.assertRegex(config_h, r"#define\s+RZ_ORBIT_OMEGA_RPM\s+49\.0f")
        self.assertRegex(config_h, r"#define\s+RZ_ORBIT_TARGET_DEG\s+\(352\.0f\)")
        self.assertNotIn("RZ_CW_TARGET_DEG", rz_h)
        self.assertNotIn("RZ_CCW_REVERSE_DEG", rz_h)
        self.assertRegex(config_h, r"#define\s+RZ_ORBIT_TIMEOUT_MS\s+15000U")
        self.assertRegex(config_h, r"#define\s+RZ_GRAB_COUNT\s+4U")
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
            "RZ_ORBIT_TARGET_DEG",
            "RZ_ORBIT_TIMEOUT_MS",
            "ROUND_PILLAR_ERROR_ORBIT_TIMEOUT",
        ):
            self.assertIn(token, rz_h + rz_c)
        self.assertIn("#include \"maixcam_link.h\"", rz_c)
        self.assertIn("MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)", rz_c)
        self.assertIn("MaixCamLink_TakeReply()", rz_c)
        self.assertIn("while (current_yaw < RZ_ORBIT_TARGET_DEG)", rz_c)
        self.assertIn(
            "MotionControl_SetBodySpeed(-RZ_ORBIT_FORWARD_RPM,\n"
            "                                       0.0f,\n"
            "                                       RZ_ORBIT_OMEGA_RPM)",
            rz_c,
        )
        self.assertNotIn(
            "MotionControl_SetBodySpeed(-RZ_ORBIT_FORWARD_RPM,\n"
            "                                       0.0f,\n"
            "                                       -RZ_ORBIT_OMEGA_RPM)",
            rz_c,
        )
        self.assertNotIn("reverse_start_yaw", rz_c)
        self.assertNotIn("reverse_target_yaw", rz_c)
        self.assertNotIn("RZ_CCW_REVERSE_DEG", rz_c)
        self.assertNotIn("while (current_yaw < reverse_target_yaw)", rz_c)

        run_start = rz_c.index("RoundPillarStatus RoundPillar_Run")
        run_body = rz_c[run_start:]
        self.assertLess(
            run_body.index("SERVO_ACTION_PILLAR_CAMERA_GROUP"),
            run_body.index("RoundPillar_OrbitAndGrab()"),
        )
        self.assertIn("ServoAction_StartGroupNoWait", run_body)
        self.assertNotIn("ServoAction_RunGroup", run_body)
        orbit_start = rz_c.index("RoundPillar_OrbitAndGrab")
        orbit_end = rz_c.index("RoundPillarStatus RoundPillar_Run", orbit_start)
        orbit_body = rz_c[orbit_start:orbit_end]
        self.assertIn("MaixCamLink_TakeReply()", orbit_body)
        self.assertIn("RoundPillar_HandleDetectedBall(&grab_count)", orbit_body)
        self.assertIn("grab_count < RZ_GRAB_COUNT", orbit_body)
        self.assertNotIn("grab_count != RZ_GRAB_COUNT", orbit_body)
        self.assertIn("MotionControl_ResetHeadingReference();", orbit_body)
        self.assertIn("MotionControl_State = MOTION_STATUS_FINISHED;", orbit_body)
        self.assertIn("return ROUND_PILLAR_OK;", orbit_body)
        self.assertNotIn("MAIXCAM_REQUEST_TIMEOUT_MS", orbit_body)
        self.assertEqual(
            orbit_body.count("RoundPillar_WaitSettled(RZ_STOP_SETTLE_MS)"), 1
        )
        handler_start = rz_c.index("RoundPillar_HandleDetectedBall")
        handler_body = rz_c[handler_start:orbit_start]
        self.assertIn("RoundPillar_WaitSettled", handler_body)
        self.assertLess(
            handler_body.index("RoundPillar_Stop()"),
            handler_body.index("RoundPillar_WaitSettled"),
        )

        self.assertEqual(
            rz_c.count("SERVO_ACTION_PILLAR_CAMERA_GROUP"), 1
        )
        self.assertEqual(
            rz_c.count("SERVO_ACTION_PILLAR_GRAB_GROUP"), 1
        )

    def test_uart5_keeps_only_competition_commands_in_help_and_status(self):
        uart_c = self.read("User/Task/uart_command.c")
        self.assertIn('"F <mm>', uart_c)
        self.assertIn('"ROT CCW <deg>', uart_c)
        self.assertIn('"BALL', uart_c)
        self.assertIn('"RZ', uart_c)
        self.assertIn('"GRAB', uart_c)
        self.assertIn('"STOP', uart_c)
        self.assertIn('"STATUS', uart_c)
        self.assertIn('"HELP', uart_c)
        for token in ("MAIX_TX", "ARM_MOTION_COUNT", "WAREHOUSE_G2_DONE"):
            self.assertNotIn(token, uart_c)
        self.assertIn("PATH\\r\\n", uart_c)
        for token in ("HEAD_ERR", "HEAD_CORR", "DIST=", "TARGET=", "BALL_STATE", "WAREHOUSE_BALL"):
            self.assertIn(token, uart_c)

    def test_fixed_path_sequence_is_static_and_wired_to_chassis_task(self):
        path_h = self.read("User/Robot/path_sequence.h")
        path_c = self.read("User/Robot/path_sequence.c")
        uart_h = self.read("User/Task/uart_command.h")
        uart_c = self.read("User/Task/uart_command.c")
        freertos_c = self.read("User/Task/task_control.c")
        cmake_c = self.read("CMakeLists.txt")
        cmake_armcc = self.read("CMakeLists_armcc.txt")
        uvprojx = self.read("MDK-ARM/chassis_motor.uvprojx")

        for token in (
            "PATH_STEP_MOVE", "PATH_STEP_ROTATE", "PATH_STEP_BALL",
            "PATH_STEP_RZ", "PATH_STEP_SERVO_GROUP", "PATH_STEP_STAIR",
            "PATH_STEP_CANGKU",
            "PATH_SEQUENCE_LF20_1800",
            "PATH_SEQUENCE_F2300", "PATH_SEQUENCE_ROTATE1_178",
            "PATH_SEQUENCE_BALL", "PATH_SEQUENCE_ROTATE2_178",
            "PATH_SEQUENCE_BACK1820", "PATH_SEQUENCE_RZ",
            "PATH_SEQUENCE_BACK310", "PATH_SEQUENCE_GROUP0",
            "PATH_SEQUENCE_F330",
            "PATH_SEQUENCE_STAIR",
            "PATH_SEQUENCE_LEFT_2000",
            "PATH_SEQUENCE_CANGKU",
            "PATH_SEQUENCE_DONE", "PATH_SEQUENCE_CANCELED",
            "PATH_SEQUENCE_ERROR",
            "PATH_SEQUENCE_ERROR_CANGKU",
        ):
            self.assertIn(token, path_h)

        table_start = path_c.index("static const PathStep PathSequence_CommandQueue[]")
        table_end = path_c.index("#define PATH_SEQUENCE_STEP_COUNT", table_start)
        table = path_c[table_start:table_end]
        self.assertEqual(table.count("PATH_STEP_MOVE"), 5)
        self.assertEqual(table.count("PATH_STEP_ROTATE"), 2)
        self.assertEqual(table.count("178.0f"), 2)
        self.assertNotIn("PATH_SEQUENCE_ROTATE1_180", path_h + path_c)
        self.assertNotIn("PATH_SEQUENCE_ROTATE2_180", path_h + path_c)
        self.assertEqual(table.count("PATH_STEP_BALL"), 1)
        self.assertEqual(table.count("PATH_STEP_RZ"), 1)
        self.assertEqual(table.count("PATH_STEP_SERVO_GROUP"), 2)
        self.assertEqual(table.count("PATH_STEP_STAIR"), 1)
        self.assertEqual(table.count("PATH_STEP_CANGKU"), 1)
        for token in (
            "1800U", "20.0f", "MOTION_DIAGONAL_CRUISE_RPM",
            "2300U", "0.0f", "MOTION_CRUISE_RPM",
            "1810U", "180.0f",
            "330U", "90.0f",
            "1650U",
            "SERVO_ACTION_START_GROUP",
        ):
            self.assertIn(token, table)
        self.assertEqual(table.count("SERVO_ACTION_START_GROUP"), 2)
        self.assertEqual(table.count("MOTION_DIAGONAL_CRUISE_RPM"), 1)
        self.assertIn("/* STEP 11", table)
        self.assertEqual(table.count("/* STEP"), 13)
        self.assertLess(table.index("/* STEP 7"), table.index("/* STEP 8"))
        self.assertLess(table.index("/* STEP 8"), table.index("/* STEP 9"))
        self.assertLess(table.index("/* STEP 9"), table.index("/* STEP 10"))
        self.assertLess(table.index("/* STEP 10"), table.index("/* STEP 11"))
        self.assertIn(
            "330U,\n        0.0f,\n        MOTION_CRUISE_RPM",
            table,
        )
        self.assertIn(
            "1650U,\n        90.0f,\n        MOTION_CRUISE_RPM",
            table,
        )

        self.assertIn("PATH_SEQUENCE_STEP_COUNT", path_c)
        self.assertNotIn("xQueueSend", path_c)
        self.assertIn("PathSequence_Run()", freertos_c)
        self.assertIn("command.type == CHASSIS_CMD_PATH", freertos_c)
        self.assertIn("OK PATH", uart_c)
        self.assertIn("ERR FORMAT", uart_c)
        self.assertIn("PATH_STATE", uart_c)
        self.assertIn("PATH_STEP", uart_c)
        self.assertIn("PATH_LAST", uart_c)
        self.assertIn("PATH_BALL_LAST", uart_c)
        self.assertIn("WAREHOUSE_STATE", uart_c)
        self.assertIn("WAREHOUSE_BALL", uart_c)
        self.assertEqual(path_c.count("WarehouseControl_IsReadyForAction()"), 1)
        ball_step = path_c.index("case PATH_STEP_BALL:")
        self.assertLess(
            path_c.index("WarehouseControl_IsReadyForAction()"), ball_step
        )
        self.assertIn("PathSequence_LastBallStatus = BallSequence_Run();", path_c)
        self.assertIn("PathSequence_LastStatus = status;", path_c)
        for build_file in (cmake_c, cmake_armcc, uvprojx):
            self.assertIn("path_sequence.c", build_file)

        run_start = path_c.index("PathSequenceStatus PathSequence_Run")
        run_body = path_c[run_start:]
        order = (
            "MotionControl_MovePolarSegmentMm(",
            "MotionControl_RotateDeg(",
            "BallSequence_Run()",
            "RoundPillar_Run()",
            "StairSequence_Run()",
            "ServoAction_RunGroup(",
            "PathSequence_StopChassis();",
        )
        positions = [run_body.index(token) for token in order]
        self.assertEqual(positions, sorted(positions))
        self.assertEqual(run_body.count("MotionControl_RotateDeg("), 1)
        self.assertEqual(table.count("PATH_STEP_ROTATE"), 2)
        self.assertIn("PathSequence_Cancel()", run_body)
        self.assertIn("PathSequence_Fail(PATH_SEQUENCE_ERROR_BALL)", run_body)
        self.assertIn("PathSequence_Fail(PATH_SEQUENCE_ERROR_SERVO)", run_body)
        self.assertIn("PathSequence_Fail(PATH_SEQUENCE_ERROR_STAIR)", run_body)
        self.assertIn("MotionControl_WasStopped()", run_body)
        self.assertNotIn("ServoAction_StartGroupNoWait", run_body)
        stair_case = run_body.index("case PATH_STEP_STAIR:")
        self.assertLess(
            table.index("PATH_STEP_STAIR"),
            table.index("1650U"),
        )
        self.assertLess(
            run_body.index("StairSequence_Run()", stair_case),
            run_body.index("PathSequence_Fail(PATH_SEQUENCE_ERROR_STAIR)", stair_case),
        )
        self.assertIn('return "LEFT_2000";', path_c)
        self.assertNotIn("PATH_SEQUENCE_LR20_", path_h + path_c)
        self.assertIn('return "GROUP0";', path_c)
        self.assertIn("PATH_SEQUENCE_ERROR_STAIR", freertos_c)
        self.assertIn("PATH_SEQUENCE_ERROR_CANGKU", freertos_c)
        self.assertIn("PATH_SEQUENCE_ERROR_SERVO", freertos_c)
        self.assertNotIn("ROUND_PILLAR_ERROR_MAIX_TIMEOUT", path_c)

    def test_turntable_uses_usart6_and_runs_after_ball_actions(self):
        usart_h = self.read("Core/Inc/usart.h")
        usart_c = self.read("Core/Src/usart.c")
        main_c = self.read("Core/Src/main.c")
        freertos_c = self.read("User/Task/task_control.c")
        turntable_h = self.read("User/Device/turntable/turntable_control.h")
        turntable_c = self.read("User/Device/turntable/turntable_control.c")
        warehouse_c = self.read("User/Robot/warehouse_control.c")
        round_c = self.read("User/Robot/round_pillar.c")
        cangku_h = self.read("User/BSP/cangku_motor.h")
        config_h = self.read("User/Config/robot_config.h")

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
        self.assertRegex(
            config_h, r"#define\s+TURNTABLE_ONE_SLOT_PULSES\s+1280U"
        )
        self.assertIn("Turntable_MoveOneSlotAndWait(", turntable_h)
        self.assertIn("Turntable_MoveOneSlotAndWait(", turntable_c)
        self.assertIn("Turntable_MoveOneSlotAndWait(", warehouse_c)
        self.assertIn("Turntable_MoveOneSlotAndWait(", round_c)

        group4 = round_c.index("ServoAction_RunGroup(")
        turntable = round_c.index("Turntable_MoveOneSlotAndWait(", group4)
        grab_count = round_c.index("(*grab_count)++", group4)
        self.assertLess(group4, turntable)
        self.assertLess(turntable, grab_count)

    def test_stair_sequence_isolated_and_wired(self):
        stair_h = self.read("User/Robot/stair_sequence.h")
        stair_c = self.read("User/Robot/stair_sequence.c")
        uart_h = self.read("User/Task/uart_command.h")
        uart_c = self.read("User/Task/uart_command.c")
        freertos_c = self.read("User/Task/task_control.c")
        cmake_c = self.read("CMakeLists.txt")
        cmake_armcc = self.read("CMakeLists_armcc.txt")
        motion_h = self.read("User/Algorithm/motion_control.h")
        motion_c = self.read("User/Algorithm/motion_control.c")
        config_h = self.read("User/Config/robot_config.h")

        for token in (
            "STAIR_GROUP_0", "STAIR_GROUP_5", "STAIR_GROUP_6", "STAIR_GROUP_7",
            "STAIR_GROUP_8", "STAIR_GROUP_9", "STAIR_GROUP_10",
            "STAIR_GROUP_11", "STAIR_GROUP_12",
            "STAIR_CAMERA_POSE_WAIT_MS", "STAIR_SERVO_TIMEOUT_MS",
            "STAIR_VISION_TIMEOUT_MS", "STAIR_FORWARD_RPM",
            "STAIR_INITIAL_BACKWARD_MM",
        ):
            self.assertIn(token, stair_h + config_h)
        self.assertRegex(config_h, r"#define\s+STAIR_VISION_TIMEOUT_MS\s+1000U")
        gray_h = self.read("User/Algorithm/gray_align.h")
        gray_c = self.read("User/Algorithm/gray_align.c")
        ball_c = self.read("User/Robot/ball_sequence.c")
        self.assertIn("GrayAlign_RunUnlimited(void)", gray_h)
        self.assertIn("GrayAlign_RunInternal", gray_c)
        self.assertIn("GrayAlign_RunUnlimited();", stair_c)
        self.assertNotIn("gray_status = GrayAlign_Run();", stair_c)
        self.assertIn("gray_status = GrayAlign_Run();", ball_c)
        self.assertIn("GRAY_ALIGN_TIMEOUT_MS", gray_h + gray_c)
        self.assertIn("timeout_enabled", gray_c)
        self.assertIn("MotionControl_StopRequested", gray_c)
        self.assertIn("MaixCamLink_SendRequest(MAIXCAM_COLOR_RED)", stair_c)
        self.assertIn("MotionControl_MovePolarSegmentMmUntil", stair_c)
        self.assertIn("Turntable_MoveOneSlotAndWait", stair_c)
        self.assertNotIn("WarehouseControl_", stair_c)
        self.assertNotIn("Warehouse_BallCount", stair_c)
        self.assertNotIn("MecanumKinematics_Solve", stair_c)
        self.assertIn("CHASSIS_CMD_STAIR", uart_h + uart_c + freertos_c)
        self.assertIn("STAIR_STATE", uart_c)
        self.assertIn("STAIR_LAST", uart_c)
        self.assertIn("User/Robot/stair_sequence.c", cmake_c)
        self.assertIn("User/Robot/stair_sequence.c", cmake_armcc)
        self.assertIn("MotionControlEarlyStopCheck", motion_h)
        self.assertIn("early_stop_check", motion_c)

        self.assertRegex(
            config_h, r"#define\s+STAIR_INITIAL_BACKWARD_MM\s+18U"
        )
        self.assertIn("STAIR_STATE_MOVE20_AFTER_ALIGN", stair_h)
        self.assertIn("STAIR_STATE_MOVE20_AFTER_ALIGN", stair_c)
        run_start = stair_c.index("StairSequenceStatus StairSequence_Run(void)")
        run_body = stair_c[run_start:]
        self.assertLess(
            run_body.index("gray_status = GrayAlign_RunUnlimited();"),
            run_body.index("status = StairSequence_Move20AfterAlign();"),
        )
        self.assertLess(
            run_body.index("status = StairSequence_Move20AfterAlign();"),
            run_body.index("status = StairSequence_RunPart3();"),
        )

    def test_stair_part2_has_four_points_and_single_g8_setup(self):
        stair_h = self.read("User/Robot/stair_sequence.h")
        stair_c = self.read("User/Robot/stair_sequence.c")

        for token in (
            "STAIR_STATE_PART2_G8",
            "STAIR_STATE_PART2_P1",
            "STAIR_STATE_PART2_MOVE_TO_P2",
            "STAIR_STATE_PART2_P2",
            "STAIR_STATE_PART2_MOVE_TO_P3",
            "STAIR_STATE_PART2_P3",
            "STAIR_STATE_PART2_MOVE_TO_P4",
            "STAIR_STATE_PART2_P4",
        ):
            self.assertIn(token, stair_h)
            self.assertIn(token, stair_c)

        part2_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart2"
        )
        part3_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart3",
            part2_start,
        )
        part2 = stair_c[part2_start:part3_start]

        self.assertIn("for (point = 0U; point < 4U; point++)", part2)
        self.assertIn("if (point > 0U)", part2)
        self.assertIn("P1 is already reached", part2)
        self.assertEqual(
            part2.count("StairSequence_Part2MoveToNextPoint()"), 1
        )
        self.assertEqual(
            part2.count("StairSequence_RunCameraPoseGroup(STAIR_GROUP_8)"),
            1,
        )
        self.assertEqual(part2.count("StairSequence_CheckRedBallStopped()"), 1)
        self.assertLess(
            part2.index("StairSequence_RunCameraPoseGroup(STAIR_GROUP_8)"),
            part2.index("for (point = 0U; point < 4U; point++)"),
        )
        self.assertLess(
            part2.index("StairSequence_RunCameraPoseGroup(STAIR_GROUP_8)"),
            part2.index("StairSequence_CheckRedBallStopped()"),
        )
        self.assertLess(
            part2.index("StairSequence_Part2MoveToNextPoint()"),
            part2.index("StairSequence_CheckRedBallStopped()"),
        )
        self.assertNotIn("break;", part2)
        self.assertLess(part2.index("StairSequence_RunTransitionGroup("),
                        part2.index("StairSequence_Move117("))

        move_start = stair_c.index(
            "static StairPart2MoveResult "
            "StairSequence_Part2MoveToNextPoint"
        )
        move_end = stair_c.index(
            "static StairSequenceState StairSequence_Part2PointState",
            move_start,
        )
        move_body = stair_c[move_start:move_end]
        self.assertIn("MotionControl_MovePolarSegmentMmUntil", move_body)
        self.assertIn("StairSequence_VisionEarlyStopCheck", move_body)
        self.assertNotIn("ServoAction_", move_body)
        self.assertNotIn("StairSequence_RunCameraPoseGroup", move_body)
        self.assertNotIn("StairSequence_CheckRedBallStopped", move_body)
        self.assertNotIn("Turntable_MoveOneSlotAndWait", move_body)
        self.assertNotIn("STAIR_BALL_FOUND", move_body)
        self.assertIn("STAIR_PART2_MOVE_VISION_STOP", move_body)

        part1_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart1"
        )
        part1 = stair_c[part1_start:part2_start]
        self.assertIn(
            "StairSequence_Move90ThenCheck(\n"
            "        STAIR_STATE_PART1_MOVE90,\n"
            "        STAIR_STATE_PART1_CHECK2);",
            part1,
        )
        part3 = stair_c[part3_start:]
        self.assertIn(
            "StairSequence_Move90ThenCheck(\n"
            "        STAIR_STATE_PART3_MOVE90,\n"
            "        STAIR_STATE_PART3_CHECK2);",
            part3,
        )

        run_start = stair_c.index(
            "StairSequenceStatus StairSequence_Run(void)"
        )
        run_body = stair_c[run_start:]
        self.assertLess(
            run_body.index("status = StairSequence_RunPart3();"),
            run_body.index("status = StairSequence_RunPart2();"),
        )
        self.assertLess(
            run_body.index("status = StairSequence_RunPart2();"),
            run_body.index("status = StairSequence_RunPart1();"),
        )

    def test_stair_action_flow_matches_requested_group_order(self):
        stair_h = self.read("User/Robot/stair_sequence.h")
        stair_c = self.read("User/Robot/stair_sequence.c")

        for token in (
            "STAIR_STATE_PART1_G0",
            "STAIR_STATE_PART2_G7",
            "STAIR_STATE_PART3_G10",
            "STAIR_STATE_PART3_MOVE117",
            "StairSequence_Move90ThenCheck",
        ):
            self.assertIn(token, stair_h + stair_c)

        part1_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart1"
        )
        part2_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart2",
            part1_start,
        )
        part3_start = stair_c.index(
            "static StairSequenceStatus StairSequence_RunPart3",
            part2_start,
        )
        run_start = stair_c.index(
            "StairSequenceStatus StairSequence_Run(void)"
        )
        third_part = stair_c[part1_start:part2_start]
        second_part = stair_c[part2_start:part3_start]
        first_part = stair_c[part3_start:run_start]

        self.assertIn("STAIR_GROUP_5", third_part)
        self.assertIn("STAIR_GROUP_6", third_part)
        self.assertIn(
            "StairSequence_Move90ThenCheck(",
            third_part,
        )
        self.assertIn(
            "STAIR_GROUP_0, STAIR_STATE_PART1_G0",
            third_part.replace("\n", " "),
        )
        self.assertNotIn("STAIR_GROUP_7, STAIR_STATE_PART1_G7", third_part)
        self.assertNotIn("StairSequence_Move117(", third_part)

        self.assertIn("for (point = 0U; point < 4U; point++)", second_part)
        self.assertIn(
            "STAIR_GROUP_7, STAIR_STATE_PART2_G7",
            second_part.replace("\n", " "),
        )
        self.assertNotIn(
            "STAIR_GROUP_10, STAIR_STATE_PART2_G10",
            second_part.replace("\n", " "),
        )
        self.assertIn("StairSequence_Move117(STAIR_STATE_PART2_MOVE117)", second_part)
        self.assertIn("117U, 180.0f", stair_c)

        self.assertIn("STAIR_GROUP_11", first_part)
        self.assertEqual(first_part.count("STAIR_GROUP_12"), 2)
        self.assertIn(
            "STAIR_GROUP_10, STAIR_STATE_PART3_G10",
            first_part.replace("\n", " "),
        )
        self.assertIn(
            "StairSequence_Move117(STAIR_STATE_PART3_MOVE117)",
            first_part,
        )

        helper_start = stair_c.index(
            "static StairBallResult StairSequence_Move90ThenCheck"
        )
        helper_end = stair_c.index(
            "/*\n * Part 2 uses", helper_start
        )
        helper = stair_c[helper_start:helper_end]
        self.assertIn("90U,\n        180.0f", helper)
        self.assertIn("117U, 180.0f", stair_c)


if __name__ == "__main__":
    unittest.main()
