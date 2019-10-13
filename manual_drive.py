from hardware.speed_controller import SpeedController
from hardware.steering_controller import SteeringController
from gps_interface.ublox_interface import UBX
from hardware.hardware_controller import HardwareController

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
    gps = UBX(port="/dev/ttyACM0", baud=9600, origin=(0,0))
    # gps.start_reading()

    # I2C Setup

    # Setup Steering Controller
    # steering_controller = SteeringController(i2c=i2c)
    #
    # # Setup Speed Controller
    # speed_controller = SpeedController(i2c=i2c, gps=gps)

    hw_controller = HardwareController(gps=gps)
    gps_data = []

    try:
        while True:
            print("Press <ENTER> to start setup")
            while True:
                user_input = input("Enter value:\n")
                if user_input == "":
                    break
                else:
                    continue
            hw_controller.start_control()

            print("Press <ENTER> again when ready")
            user_input = input("Enter value:\n")
            while True:
                user_input = input("Enter value:\n")
                if user_input == "":
                    break
                else:
                    continue
            print("Use A and D to steer and W and S to accelerate/decelerate. P to quit\n")
            while True:
                try:
                    user_input = input("\r")
                    # try:
                    #     assert user_input in ["w", "W", "a", "A", "s", "S", "d", "D", "p", "P"]
                    # except AssertionError:
                    #     continue

                    print("Starting GPS log:")
                    gps_data.append([gps.lat, gps.lon, gps.heading, gps.two_dim_speed])

                    if user_input.upper() == "W":
                        current_speed = min(current_speed + 1, max_speed)
                        hw_controller.set_speed(current_speed)
                    elif user_input.upper() == "S":
                        current_speed = max(current_speed - 1, 0)
                        hw_controller.set_speed(current_speed)
                    elif user_input.upper() == "A":
                        current_angle = min(current_angle + 1, max_angle_left)
                        hw_controller.set_steering_angle(current_angle)
                    elif user_input.upper() == "D":
                        current_angle = max(current_angle - 1, max_angle_right)
                        hw_controller.set_steering_angle(current_angle)

                    elif user_input.upper() == "P":
                        hw_controller.stop_control()
                        break
                    else:
                        try:
                            angle = float(user_input)
                            if angle > max_angle_left:
                                current_angle = max_angle_left
                            elif angle < max_angle_right:
                                current_angle = max_angle_right
                            else:
                                current_angle = angle
                            hw_controller.set_steering_angle(current_angle)
                        except ValueError:
                            pass

                    print(f"\rSpeed: {hw_controller.get_speed()}:{current_speed}, Steering Angle: {hw_controller.get_steering_angle()}:{current_angle}")
                except KeyboardInterrupt:
                    continue
    except KeyboardInterrupt:
        print("\rExiting...")
        hw_controller.stop_control()
        print("Saving gps data...")
        with open("gps_data.txt", "w+") as file:
            for data_point in gps_data:
                file.write(str(data_point) + "\n")
        exit(0)
