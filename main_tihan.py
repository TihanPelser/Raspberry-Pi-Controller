# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from coordinate_conversions import xy
import numpy as np
import time

# PATH CONTROLLER
from controller.DQNController import DQNController

INPUT_PATH_FILE = "sample-path"
SPEED_SET_POINT = 0
WAY_POINT_THRESHOLD = 1

def read_path(path_file: str):
    path_data = []
    with open(path_file, "r") as file:
        x, y = file.read().split(",")
        path.append([x, y])

    return np.array(path_data)


if __name__ == "__main__":

    RUN_NAME = sys.argv[1]

    path = read_path(INPUT_PATH_FILE)

    gps = UBX()
    # I2C Setup
    # i2c = busio.I2C(board.SCL, board.SDA)

    gps.start_reading()

    hardware_controller = HardwareController(gps=gps)

    path_controller = DQNController(model_file="controller/models/COMPLEX_ARCH_1_IN_5_OUT.h5")
    print("Waiting for GPS signal...")
    time.sleep(2)
    origin = np.array([gps.lat, gps.long])

    path = xy.convert_path(origin=origin, path=path)

    gps_results = []
    hardware_results = []

    end_reached = False
    current_way_point_index = 0
    current_way_point_coords = path[current_way_point_index]
    try:
        hardware_controller.start_control()

        # Speed Override:
        # speed_volt = 0.8
        # hardware_controller.speed_dac.value = int(round(speed_volt/5) * 65535)
        while not end_reached:
            # L
            hardware_results.append(hardware_controller.get_current_data())
            gps_results.append(gps.get_current_data())

            current_heading = xy.convert_heading(gps.heading)
            current_coords = np.array([gps.lat, gps.long])
            heading_to_next = xy.
            continue

    except KeyboardInterrupt:
        hardware_controller.shutdown()

    with open(f"TihanResults/{RUN_NAME}_gps.txt", "w+") as file:
        pass

    with open(f"TihanResults/{RUN_NAME}_hw.txt", "w+") as file:
        pass
