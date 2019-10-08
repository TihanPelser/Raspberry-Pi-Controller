from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
import threading
import numpy as np
from typing import Optional


# Steering DAC Address = 97

def _convert_voltage_to_distance(volt: float) -> float:
    # Output distance in [mm]
    return 133.51*volt - 4.209

def _convert_distance_to_voltage(distance: float) -> float:
    # Output voltage [V]
    return (distance + 4.209) / 133.51

def _convert_distance_to_angle(y: float) -> float:

    a = 81.20
    b = 111.68
    c = 97.52
    l_total = 451.0601
    l0 = 124.1643

    l = l_total - y

    gamma = np.arccos((a**2 + c**2 + l0**2 - b**2)/(2 * a * np.sqrt(c**2 + l0**2))) + np.arctan(c/l0)

    delta = np.arccos((a**2 + c**2 + l**2 - b**2)/(2 * a * np.sqrt(c**2 + l**2))) + np.arctan(c/l)

    # Left +, Right -
    return delta - gamma


def _convert_angle_to_distance(angle: float) -> float:
    pass


class SteeringController:

    def __init__(self, i2c: busio.I2C):
        # Steering Digital Pins
        self.left = DIO.DigitalInOut(board.D21)     # D1
        self.right = DIO.DigitalInOut(board.D20)    # D2
        # Set as outputs
        self.left.direction = DIO.Direction.OUTPUT
        self.right.direction = DIO.Direction.OUTPUT

        # I2C and ADC Setup
        self._i2c_interface = i2c

        # Setup Steering Angle ADC
        adc = ADC.ADS1115(self._i2c_interface, address=72)
        self.adc_input = AnalogIn(adc, ADC.P0)

        # Setup Steering Angle DAC
        self.dac_output = DAC.MCP4725(self._i2c_interface, address=97)

        # Steering
        # 0 Degree Steer = 2.48V
        self._output_voltage = 0
        self._max_output_voltage = 1.
        self._steering_angle_set_point = 0.
        self._current_steering_angle = 0.

        # Steering PID Controller
        self.P = 0.1
        self.D = 1
        self._prev_error = 0.

        # Threading
        self._steering_thread = None
        self._stop_threads = False

        # Set DAC Output
        self.dac_output.value = 0

    def start_steering_control(self):
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
            p_term = error * self.P
            d_term = (error - self._prev_error) * self.D
            self._prev_error = error
            output = p_term + d_term
            self._set_output_voltage(abs(output))
            if output > 0:
                self.right.value = False
                self.left.value = True
            elif output < 0:
                self.left.value = False
                self.right.value = True
            self.dac_output.value = int(round((self._output_voltage / 5) * 4095))

    def _update_current_steering_angle(self) -> None:
        pot_voltage = self.adc_input.voltage
        lat_distance = _convert_voltage_to_distance(pot_voltage)
        self._current_steering_angle = _convert_distance_to_angle(lat_distance)

    def _set_output_voltage(self, voltage_setting: float, voltage_override: Optional[float] = None):
        if voltage_setting > self._max_output_voltage:
            if voltage_override is not None:
                if voltage_setting > voltage_override:
                    self._output_voltage = voltage_override
                else:
                    self._output_voltage = voltage_setting
            else:
                self._output_voltage = self._max_output_voltage
        else:
            self._output_voltage = voltage_setting

    def get_steering_angle(self) -> float:
        return self._current_steering_angle

    def set_steering_angle(self, angle: float) -> None:
        self._steering_angle_set_point = angle


