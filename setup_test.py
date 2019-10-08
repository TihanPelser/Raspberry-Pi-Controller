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

    # Setup GPS
    gps = UBX(port="/dev/ttyACM0", baud=9600)

    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    # Setup Steering Controller
    steering_controller = SteeringController(i2c=i2c)

    # Setup Speed Controller
    speed_controller = SpeedController(i2c=i2c, gps=gps)

    try:
        while True:
            print("<S> to test Steering Control")
            print("<D> to test Speed Control")
            user_input = input("Enter value:\n")
            try:
                assert user_input in ["S", "s", "D", "d"], "Please enter a valid option!"
            except AssertionError:
                continue

            if user_input.upper() == "S":
                print("Steering control selected...")
                steering_controller.start_steering_control()
                try:
                    while True:
                        angle = input("Steering Angle:\n")
                        try:
                            angle = float(angle)
                        except TypeError:
                            print("Please enter a number")
                            continue

                        steering_controller.set_steering_angle(angle=angle)
                except KeyboardInterrupt:
                    print("Stopping steering control")
                    steering_controller.stop_steering_control()
                    continue

            elif user_input.upper() == "D":
                print("Speed control selected...")
                speed_controller.start_speed_control()
                try:
                    while True:
                        speed = input("Speed:\n")
                        try:
                            speed = float(speed)
                        except TypeError:
                            print("Please enter a number")
                            continue

                        speed_controller.set_speed(speed=speed)
                except KeyboardInterrupt:
                    print("Stopping speed control")
                    speed_controller.stop_speed_control()
                    continue

    except KeyboardInterrupt:
        print("Exiting...")
        exit(0)
