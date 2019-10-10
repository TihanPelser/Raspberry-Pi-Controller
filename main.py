# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX

# PATH CONTROLLER
from controller.DQNController import DQNController

if __name__ == "__main__":

    gps = UBX(port="/dev/ttyACM0", baud=9600)
    gps.start_reading()

    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    hardware_controller = HardwareController(i2c=i2c, gps=gps)

    path_controller = DQNController(model_file="controller/models/COMPLEX_ARCH_1_IN_5_OUT.h5")

    while True:
        continue
