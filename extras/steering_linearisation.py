from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import time

if __name__ == "__main__":
    try:
        gps = UBX(port="/dev/ttyACM0", baud=9600, origin=(0, 0))
        hw_controller = HardwareController(gps=gps)
        hw_controller.center()
        volt_left = []
        volt_right = []
        volt_center = []
        for i in range(100):
            volt_center.append(hw_controller.steer_adc.voltage)

        hw_controller.steer_dac.value = 10000

        while True:
            user_input = input("Continue:")
            if user_input == "":
                break

        hw_controller.left()
        while True:
            user_input = input("Continue:")
            if user_input == "":
                break
        for i in range(100):
            volt_left.append(hw_controller.steer_adc.voltage)

        hw_controller.right()

        while True:
            user_input = input("Continue:")
            if user_input == "":
                break

        for i in range(100):
            volt_right.append(hw_controller.steer_adc.voltage)

        while True:
            user_input = input("Continue:")
            if user_input == "":
                break

        left_avg = sum(volt_left)/len(volt_left)
        right_avg = sum(volt_right)/len(volt_right)
        center_avg = sum(volt_center)/len(volt_center)
        print(f"Left: {left_avg}")
        print(f"Right: {right_avg}")
        print(f"Center: {center_avg}")
    except KeyboardInterrupt:
        hw_controller.center()
