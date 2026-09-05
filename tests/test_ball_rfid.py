import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class BallRfidContractTest(unittest.TestCase):
    def read(self, relative):
        return (ROOT / relative).read_text(encoding="utf-8-sig")

    def test_rfid_module_exposes_uid_api(self):
        rfid_h = self.read("User/Device/rfid/rfid.h")
        rfid_c = self.read("User/Device/rfid/rfid.c")

        self.assertIn("RFID_Init", rfid_h)
        self.assertIn("RFID_Read_ID", rfid_h)
        self.assertIn("RFID_Get_ID", rfid_h)
        self.assertIn("RFID_Clear", rfid_h)
        self.assertIn("HAL_UART_Receive_IT", rfid_c)
        self.assertIn("huart8", rfid_c)

    def test_ball_stores_full_uids_in_five_entry_batch(self):
        ball_h = self.read("User/Robot/ball_sequence.h")
        ball_c = self.read("User/Robot/ball_sequence.c")
        config_h = self.read("User/Config/robot_config.h")

        self.assertRegex(config_h, r"#define\s+BALL_GRAB_MAX\s+5U")
        self.assertIn("uint32_t grabbed_ball_id", ball_h)
        self.assertIn("grabbed_ball_id", ball_h)
        self.assertIn("grabbed_ball_count", ball_h)
        self.assertIn("BALL_SEQUENCE_WAITING_RFID", ball_h)
        self.assertIn("RFID_Read_ID", ball_c)
        self.assertIn("BALL_GRAB_MAX", ball_c)
        self.assertIn("BALL_Get_Grabbed_ID", ball_h)
        self.assertIn("BALL_Get_ID_List", ball_h)

    def test_uart8_is_configured_for_rfid_on_pe0_pe1(self):
        usart_h = self.read("Core/Inc/usart.h")
        usart_c = self.read("Core/Src/usart.c")
        it_h = self.read("Core/Inc/stm32f7xx_it.h")
        it_c = self.read("Core/Src/stm32f7xx_it.c")
        state_machine_c = self.read("User/Robot/state_machine.c")

        self.assertIn("huart8", usart_h)
        self.assertIn("MX_UART8_Init", usart_h)
        self.assertIn("115200", usart_c)
        self.assertIn("PE0", usart_c)
        self.assertIn("PE1", usart_c)
        self.assertIn("UART8_IRQHandler", it_h)
        self.assertIn("HAL_UART_IRQHandler(&huart8)", it_c)
        self.assertIn("RFID_Init", state_machine_c)
        self.assertIn("RFID_UartRxCpltCallback", state_machine_c)

    def test_build_projects_include_rfid_source(self):
        cmake_gcc = self.read("CMakeLists.txt")
        cmake_armcc = self.read("CMakeLists_armcc.txt")
        uvprojx = self.read("MDK-ARM/chassis_motor.uvprojx")

        self.assertIn("User/Device/rfid/rfid.c", cmake_gcc)
        self.assertIn("User/Device/rfid/rfid.c", cmake_armcc)
        self.assertIn("../User/Device/rfid/rfid.c", uvprojx)
        self.assertIn("../User/Device/rfid/rfid.h", uvprojx)

    def test_warehouse_mechanical_counter_remains_six(self):
        warehouse_h = self.read("User/Robot/warehouse_control.h")
        self.assertIn("#define WAREHOUSE_TOTAL_BALLS               6U", warehouse_h)


if __name__ == "__main__":
    unittest.main()
