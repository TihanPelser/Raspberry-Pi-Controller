# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from coordinate_conversions import xy
import numpy as np

# PATH CONTROLLER
from controller.DQNController import DQNController

INPUT_PATH_FILE = "sample-path"


def read_path(path_file: str):
    path_data = []
    with open(path_file, "r") as file:
        x, y = file.read().split(",")
        path.append([x, y])

    return np.array(path_data)


if __name__ == "__main__":

    gps_results_file = sys.argv[1]
    hardware_results_file = sys.argv[2]

    path = read_path(INPUT_PATH_FILE)
    # parser = argparse.ArgumentParser()
    #
    # parser.add_argument('--gps-log', help='Optional GPS log file')
    # parser.add_argument('--path', help='The input path file')
    #
    # args = parser.parse_args()
    #
    # if len(sys.argv) > 1:
    #     gps_log = sys.argv[1]
    # else:
    #     gps_log = None


    gps = UBX(port="/dev/ttyACM0", baud=9600)
    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    gps.start_reading()

    hardware_controller = HardwareController(gps=gps)

    path_controller = DQNController(model_file="controller/models/COMPLEX_ARCH_1_IN_5_OUT.h5")

    gps_data = gps.get_current_data()
    origin = np.array([gps_data.lat, gps_data.long])

    path = xy.convert_path(origin=origin, path=path)

    gps_results = []
    hardware_results = []

    try:
        hardware_controller.startup()
        hardware_controller.start_control()

        # Speed Override:
        # speed_volt = 0.8
        # hardware_controller.speed_dac.value = int(round(speed_volt/5) * 65535)
        while True:
            hardware_results.append(hardware_controller.get_current_data())
            gps_results.append(gps.get_current_data())

            continue

    except KeyboardInterrupt:
        hardware_controller.shutdown()

    with open(f"TihanResults/{gps_results_file}.txt", "w+") as file:
        pass

    with open(f"TihanResults/{hardware_results_file}.txt", "w+") as file:
        pass
