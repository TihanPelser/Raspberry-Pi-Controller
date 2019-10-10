from hardware.speed_controller import SpeedController
from hardware.steering_controller import SteeringController
from gps_interface.ublox_interface import UBX

from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import yaml

if __name__ == "__main__":
    current_speed = 0
    current_angle = 0
    max_speed = 10
    max_angle_left = 20
    max_angle_right = -20

    # Setup GPS
    gps = UBX(port="/dev/ttyACM0", baud=9600)
    gps.start_reading()

    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    # Setup Steering Controller
    steering_controller = SteeringController(i2c=i2c)

    # Setup Speed Controller
    speed_controller = SpeedController(i2c=i2c, gps=gps)

    try:
        while True:
            print("Press <ENTER> to start setup")
            user_input = input("Enter value:\n")
            try:
                assert user_input in [""], "Please enter a valid option!\r"
            except AssertionError:
                continue

            steering_controller.start_steering_control()
            speed_controller.start_speed_control()

            print("Press <ENTER> again when ready")
            user_input = input("Enter value:\n")
            try:
                assert user_input in [""], "\rPlease enter a valid option!"
            except AssertionError:
                continue

            print("Use A and D to steer and W and S to accelerate/decelerate. P to quit\n")
            while True:
                try:
                    user_input = input("\r")
                    try:
                        assert user_input in ["w", "W", "a", "A", "s", "S", "d", "D", "p", "P"]
                    except AssertionError:
                        continue

                    if user_input.upper() == "W":
                        current_speed = min(current_speed + 1, max_speed)
                        speed_controller.set_speed(current_speed)
                    elif user_input.upper() == "S":
                        current_speed = max(current_speed - 1, 0)
                        speed_controller.set_speed(current_speed)
                    elif user_input.upper() == "A":
                        current_angle = min(current_angle + 1, max_angle_left)
                        steering_controller.set_steering_angle(current_angle)
                    elif user_input.upper() == "D":
                        current_angle = max(current_angle - 1, max_angle_right)
                        steering_controller.set_steering_angle(current_angle)

                    elif user_input.upper() == "P":
                        break

                    print(f"\rSpeed: {speed_controller.get_speed()}:{current_speed}, Steering Angle: {steering_controller.get_steering_angle()}:{current_angle}")
                except KeyboardInterrupt:
                    continue
    except KeyboardInterrupt:
        print("\rExiting...")
        speed_controller.stop_speed_control()
        steering_controller.stop_steering_control()
        exit(0)
