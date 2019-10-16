from gps_interface.ublox_interface import UBX
from hardware.hardware_controller import HardwareController
import sys
import time
import threading


def gather_data

if __name__ == "__main__":
    TYPE = sys.argv[1]
    FILE_NAME = sys.argv[2]
    try:
        assert TYPE in ["steer", "speed"], "Enter either 'speed' or 'steer'"
    except AssertionError:
        exit()
    gps = UBX()
    gps.start_reading()
    hw = HardwareController(gps=gps)
    response_data = []
    try:
        while True:
            hw.start_control()
            user_input = input(f"Enter a value")
            start = time.time()
            measuring_time = 0.
            while measuring_time <= 10:
                if TYPE == "steer":
                    hw.set_steering_angle(float(user_input))
                    data = hw.get_current_data()
                    response_data.append([data.steering_angle_set_point, data.current_steering_angle])
                elif TYPE == "speed"
                    hw.set_speed(float(user_input))
                    data = hw.get_current_data()
                    response_data.append([data.speed_set_point, data.current_speed])
                measuring_time = time.time() - start
    except KeyboardInterrupt:
        print("Done sampling")
        hw.stop_control()

    with open(f"ResponseData/{FILE_NAME}.txt") as file:
        for data_point in response_data:

