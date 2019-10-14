# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse


# PATH CONTROLLER
from controller.DQNController import DQNController

if __name__ == "__main__":

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

    gps_log = None
    input_path = "sample-path"

    gps = UBX(port="/dev/ttyACM0", baud=9600, origin=(0, 0), gps_log=gps_log)
    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    hardware_controller = HardwareController(gps=gps)

    path_controller = DQNController(model_file="controller/models/COMPLEX_ARCH_1_IN_5_OUT.h5")

    while True:
        continue
