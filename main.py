# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import yaml
from hardware.steering_controller import SteeringController
from hardware.speed_controller import DriveController


if __name__ == "__main__":

    # I2C Setup
    i2c = busio.I2C(board.SCL, board.SDA)

    # Setup Steering Controller
    steering_controller = SteeringController(i2c=i2c)
    
    # Setup drive controller
    drive_controller = DriveController(i2c=i2c)
