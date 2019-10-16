from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import time

if __name__ == "__main__":
    i2c_interface = busio.I2C(board.SCL, board.SDA)
    while not i2c_interface.try_lock():
        pass

    while True:
        addr = i2c_interface.scan()
        print(f"Address found : {addr}")
        time.sleep(2)