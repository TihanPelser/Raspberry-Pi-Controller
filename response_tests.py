from gps_interface.ublox_interface import UBX
from hardware.hardware_controller import HardwareController
import sys
import time
import threading


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
    user_input = input(f"Enter a value")
    try:
        hw.start_control()
        time.sleep(5)
        # Compensate for heavy battery
        hw.set_steering_angle(-1)
        print("Started Sampling")
        start = time.time()
        measuring_time = 0.
        while measuring_time <= 2:
            if TYPE == "steer":
                hw.set_steering_angle(0)
                time.sleep(0.01)
                data = hw.get_current_data()
                response_data.append([data.steering_angle_set_point, data.current_steering_angle])
            elif TYPE == "speed":
                hw.set_speed(float(user_input))
                time.sleep(0.01)
                data = hw.get_current_data()
                response_data.append([data.speed_set_point, data.current_speed])
            measuring_time = time.time() - start
        while measuring_time <= 5:
            if TYPE == "steer":
                hw.set_steering_angle(float(user_input))
                time.sleep(0.01)
                data = hw.get_current_data()
                response_data.append([data.steering_angle_set_point, data.current_steering_angle])
            elif TYPE == "speed":
                hw.set_speed(float(user_input))
                time.sleep(0.01)
                data = hw.get_current_data()
                response_data.append([data.speed_set_point, data.current_speed])
            measuring_time = time.time() - start
    except KeyboardInterrupt:
        print("Manual stop")
        hw.stop_control()
        exit()
    print("Done sampling")
    with open(f"ResponseData/{FILE_NAME}.txt", "w+") as file:
        for data_point in response_data:
            file.write(str(data_point) + "\n")

    hw.stop_control()

    print("Finished writing")

