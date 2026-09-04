from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class UserArchitectureContractTest(unittest.TestCase):
    def test_user_directory_layout_and_moved_sources_exist(self):
        expected = (
            "User/BSP/motor_control.c",
            "User/BSP/cangku_motor.c",
            "User/Algorithm/mecanum_kinematics.c",
            "User/Algorithm/motion_control.c",
            "User/Algorithm/gray_align.c",
            "User/Device/servo/servo_action.c",
            "User/Device/rfid/rfid.c",
            "User/Device/imu/jy61p.c",
            "User/Device/camera/maixcam_link.c",
            "User/Device/turntable/turntable_control.c",
            "User/Robot/ball_sequence.c",
            "User/Robot/round_pillar.c",
            "User/Robot/stair_sequence.c",
            "User/Robot/path_sequence.c",
            "User/Robot/cangku_task.c",
            "User/Robot/warehouse_control.c",
            "User/Task/uart_command.c",
            "User/Task/task_control.c",
            "User/Config/robot_config.h",
            "User/Config/pin_config.h",
        )
        for relative in expected:
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_cube_mx_directories_and_build_tools_are_retained(self):
        for relative in (
            "Core/Src/main.c",
            "Core/Src/freertos.c",
            "Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal.h",
            "Middlewares/Third_Party/FreeRTOS/Source/tasks.c",
            ".vscode/build.ps1",
            ".vscode/tasks.json",
            ".vscode/settings.json",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)

    def test_build_manifests_use_user_paths_for_bsp_and_devices(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        armcc = (ROOT / "CMakeLists_armcc.txt").read_text(encoding="utf-8")
        for source in (
            "User/BSP/motor_control.c",
            "User/BSP/cangku_motor.c",
            "User/Device/servo/servo_action.c",
            "User/Device/rfid/rfid.c",
            "User/Device/imu/jy61p.c",
            "User/Device/camera/maixcam_link.c",
            "User/Device/turntable/turntable_control.c",
        ):
            self.assertIn(source, cmake)
            self.assertIn(source, armcc)
        self.assertNotIn("App/competition_path.c", armcc)

    def test_algorithm_robot_and_config_contract(self):
        for relative in (
            "User/Algorithm/mecanum_kinematics.h",
            "User/Algorithm/motion_control.h",
            "User/Algorithm/gray_align.h",
            "User/Robot/ball_sequence.h",
            "User/Robot/round_pillar.h",
            "User/Robot/stair_sequence.h",
            "User/Robot/path_sequence.h",
            "User/Robot/cangku_task.h",
            "User/Robot/warehouse_control.h",
        ):
            self.assertTrue((ROOT / relative).is_file(), relative)
        config = (ROOT / "User/Config/robot_config.h").read_text(encoding="utf-8")
        self.assertIn("#define MOTOR_WHEEL_DIAMETER_MM          75U", config)
        self.assertIn("#define MOTION_CRUISE_RPM          130.0f", config)
        self.assertIn("#define MOTION_DIAGONAL_CRUISE_RPM  85.0f", config)
        pins = (ROOT / "User/Config/pin_config.h").read_text(encoding="utf-8")
        self.assertIn("PD8", pins)
        self.assertIn("PD0", pins)
        self.assertIn("PD1", pins)
        self.assertIn("PD3", pins)

    def test_main_and_freertos_keep_only_system_shells(self):
        main = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
        freertos = (ROOT / "Core/Src/freertos.c").read_text(encoding="utf-8")
        task = (ROOT / "User/Task/task_control.c").read_text(encoding="utf-8")
        state = (ROOT / "User/Robot/state_machine.c").read_text(encoding="utf-8")
        self.assertNotIn("ServoAction_Init", main)
        self.assertNotIn("MaixCamLink_Init", main)
        self.assertNotIn("RFID_Init", main)
        self.assertNotIn("BallSequence_Init", main)
        self.assertNotIn("void StartChassisTask(void *argument)", freertos)
        self.assertIn("void StartChassisTask(void *argument)", task)
        self.assertIn("void RobotUser_Init(void)", state)
        self.assertIn("ServoAction_Init", state)
        self.assertIn("MaixCamLink_Init", state)
        self.assertIn("RFID_Init", state)
        self.assertIn("BallSequence_Init", state)

    def test_old_user_source_paths_are_not_in_build_manifests(self):
        for relative in ("CMakeLists.txt", "CMakeLists_armcc.txt"):
            text = (ROOT / relative).read_text(encoding="utf-8")
            for old in (
                "App/ball_sequence.c",
                "App/path_sequence.c",
                "Motor/motion_control.c",
                "IMU/jy61p.c",
            ):
                self.assertNotIn(old, text)

    def test_public_header_names_and_tooling_are_preserved(self):
        self.assertIn(
            '#include "motion_control.h"',
            (ROOT / "User/Robot/path_sequence.c").read_text(encoding="utf-8"),
        )
        self.assertIn(
            '"${workspaceFolder}/User/Robot"',
            (ROOT / ".vscode/c_cpp_properties.json").read_text(encoding="utf-8"),
        )


if __name__ == "__main__":
    unittest.main()
