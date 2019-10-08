from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import time

if __name__ == "__main__":
    # I2C and ADC Setup
    i2c_interface = busio.I2C(board.SCL, board.SDA)
    # First I2C bus is normal RasPi I2C pins
    # i2c_adc = busio.I2C(board.SCL, board.SDA)

    # Second I2C bus is set to: SCL GPIO17, SDA GPIO27
    # i2c_dac_steering = busio.I2C(board.D17, board.D27)

    # Setup Steering Angle ADC
    # adc = ADC.ADS1115(i2c_interface, address=72)
    # adc_input = AnalogIn(adc, ADC.P0)

    # Setup Steering Angle DAC
    dac_output = DAC.MCP4725(i2c_interface, address=97)
    output_voltage = 0.
    direction = True
    add = 0.1
    try:
        while True:
            dac_output.raw_value = round(output_voltage / 5 * 4095)
            if output_voltage >= 4.9 and direction:
                direction = not direction
                print(f"Voltage going down")
                add *= -1
            if output_voltage <= 0.1 and not direction:
                direction = not direction
                print(f"Voltage going up")
                add *= -1
            output_voltage += add

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping...")
        exit(0)