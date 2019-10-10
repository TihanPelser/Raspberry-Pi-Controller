from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import threading
import numpy as np
from typing import Optional
from collections import deque


# Steering DAC Address = 97

def _convert_voltage_to_distance(volt: float) -> float:
    # Output distance in [mm]
    return 133.51*volt - 4.209


def _convert_distance_to_voltage(distance: float) -> float:
    # Output voltage [V]
    return (distance + 4.209) / 133.51


def _convert_distance_to_angle(y: float) -> float:

    a = 111.68
    b = 81.20
    c = 97.52
    l_total = 451.0601
    l0 = 111.214

    l = l_total - y

    gamma = np.arccos((a**2 + c**2 + l0**2 - b**2)/(2 * a * np.sqrt(c**2 + l0**2))) + np.arctan(c/l0)

    delta = np.arccos((a**2 + c**2 + l**2 - b**2)/(2 * a * np.sqrt(c**2 + l**2))) + np.arctan(c/l)

    # Left +, Right -
    return delta - gamma


def _convert_angle_to_distance(angle: float) -> float:
    # TODO: Trig calculations
    pass


class SteeringController:

    def __init__(self, i2c: busio.I2C):
        # Steering Digital Pins
        self.left_pin = DIO.DigitalInOut(board.D21)     # D1
        self.right_pin = DIO.DigitalInOut(board.D20)    # D2
        # Set as outputs
        self.left_pin.direction = DIO.Direction.OUTPUT
        self.right_pin.direction = DIO.Direction.OUTPUT

        # I2C and ADC Setup
        self._i2c_interface = i2c

        # Setup Steering Angle ADC
        adc = ADC.ADS1115(self._i2c_interface, address=72)
        self.adc_input = AnalogIn(adc, ADC.P0)

        # Setup Steering Angle DAC
        self.dac_output = DAC.MCP4725(self._i2c_interface, address=97)

        # Steering
        # 0 Degree Steer = 2.577V
        self._output_voltage = 1.3
        self._steering_angle_set_point = 0.
        self._current_steering_angle = 0.
        self._adc_measurements = deque(maxlen=10)

        # Steering PID Controller
        # Not implemented
        self.P = 0.1
        self.D = 1
        self._prev_error = 0.

        # Threading
        self._steering_thread = None
        self._stop_threads = False

        # Set DAC Output
        self.dac_output.value = 0

    def start_steering_control(self):
        self.dac_output.value = int(round((self._output_voltage / 5) * 65535))  # 16bits
        self._stop_threads = False
        self._steering_thread = threading.Thread(target=self._correct_steering_angle, name="steering", daemon=True)
        self._steering_thread.start()
        print("Steering started...")

    def stop_steering_control(self):
        self._stop_threads = True
        self._output_voltage = 0
        self._steering_angle_set_point = 0
        self.dac_output.value = 0
        print("Steering stopped")

    def _correct_steering_angle(self):
        # TODO: Implement PID Steering Control
        while not self._stop_threads:
            self._update_current_steering_angle()
            error = self._steering_angle_set_point - self._current_steering_angle
            if error > 1:
                self.left()
            elif error < -1:
                self.right()
            else:
                self.center()

    def _update_current_steering_angle(self) -> None:
        self._adc_measurements.append(self.adc_input.voltage)
        pot_voltage_avg = sum(self._adc_measurements)/len(self._adc_measurements)
        lat_distance = _convert_voltage_to_distance(pot_voltage_avg)
        self._current_steering_angle = np.rad2deg(_convert_distance_to_angle(lat_distance))

    def _set_output_voltage(self, voltage_setting: float):
        self._output_voltage = min(voltage_setting, 5.)

    def get_steering_angle(self) -> float:
        return self._current_steering_angle

    def set_steering_angle(self, angle: float) -> None:
        self._steering_angle_set_point = angle

    def left(self):
        self.right_pin.value = False
        self.left_pin.value = True

    def right(self):
        self.left_pin.value = False
        self.right_pin.value = True

    def center(self):
        self.left_pin.value = False
        self.right_pin.value = False


