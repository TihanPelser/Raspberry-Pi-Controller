from adafruit_blinka.board import raspi_40pin as board
import busio
import adafruit_ads1x15.ads1115 as ADC
import adafruit_mcp4725 as DAC
from adafruit_ads1x15.analog_in import AnalogIn
import digitalio as DIO
from gps_interface.ublox_interface import UBX
import threading
import time
import numpy as np
from collections import deque, namedtuple

state = namedtuple("state", "speed_set_point, current_speed, steering_angle_set_point, current_steering_angle,"
                            "steer_adc_set_point, steer_adc_avg, steer_dac_value, speed_dac_value")


# BASED ON LINEARISATION
def _linear_angle_to_voltage(angle: float) -> float:
    return 0.0133 * angle + 2.58296


def _linear_voltage_to_angle(voltage: float) -> float:
    return (voltage - 2.58296) / 0.0133


class HardwareController:
    def __init__(self, gps: UBX):
        # self.steer = SteeringController(i2c=i2c)
        # self.speed = SpeedController(i2c=i2c, gps=gps)

        # Steering Digital Pins
        self.left_pin = DIO.DigitalInOut(board.D21)  # D1
        self.right_pin = DIO.DigitalInOut(board.D20)  # D2
        # Set as outputs
        self.left_pin.direction = DIO.Direction.OUTPUT
        self.right_pin.direction = DIO.Direction.OUTPUT

        # Drive Controller (Millipak) Digital Pins
        self.forward = DIO.DigitalInOut(board.D16)  # D3
        self.reverse = DIO.DigitalInOut(board.D26)  # D4
        self.fs1 = DIO.DigitalInOut(board.D19)  # D5
        self.seat = DIO.DigitalInOut(board.D13)  # D6
        self.power = DIO.DigitalInOut(board.D6)  # D7

        # Set as outputs
        self.forward.direction = DIO.Direction.OUTPUT
        self.reverse.direction = DIO.Direction.OUTPUT
        self.fs1.direction = DIO.Direction.OUTPUT
        self.seat.direction = DIO.Direction.OUTPUT
        self.power.direction = DIO.Direction.OUTPUT

        # GPS
        self._gps = gps
        # self._gps.start_reading()

        # I2C and ADC Setup
        self._i2c_interface = busio.I2C(board.SCL, board.SDA)

        # Setup Steering Angle ADC
        adc = ADC.ADS1115(self._i2c_interface, address=72)
        self.steer_adc = AnalogIn(adc, ADC.P0)

        # Setup Steering Angle DAC
        self.steer_dac = DAC.MCP4725(self._i2c_interface, address=97)

        # Setup Drive Speed DAC
        self.speed_dac = DAC.MCP4725(self._i2c_interface, address=96)

        # Speed
        self._measured_speed = 0.
        self._speed_set_point = 0.
        self._stop = False

        # Speed PID Controller
        self._prev_error_speed = 0.
        self._cumulative_error_speed = 0.
        self.p_gain_speed = 0.01
        self.d_gain_speed = 1.
        self.i_gain_speed = 0.
        self._speed_output_voltage = 0
        self._max_speed_voltage = 1.6
        self.driving_forward = True

        # Steering
        # 0 Degree Steer = 2.582V
        self._steer_output_voltage = 0.
        self._max_steer_voltage = 3.5
        self._min_steer_voltage = 0.05
        self._steering_angle_set_point = 0.
        self._current_steering_angle = 0.
        self._steer_adc_measurements = deque(maxlen=3)
        self._steering_changed = False
        self._steer_adc_average = 0.
        self._steer_adc_set_point = 0.
        self.left_max = 20
        self.right_max = -20

        # Steering PID Controller
        # Not implemented
        self.p_gain_steer = 15.
        self.d_gain_steer = 0.
        self.i_gain_steer = .2
        self._prev_error_steer = 0.
        self._cumulative_error_steer = 0.

        # Set DAC Output
        self.steer_dac.value = 0
        self.speed_dac.value = 0

        # Threading
        self._stop_threads = False
        self._control_thread = None

    def _startup_sequence(self):
        self.fs1.value = False
        self.forward.value = False
        self.reverse.value = False
        self.power.value = False
        self.seat.value = False

        time.sleep(0.2)

        self.seat.value = True

        time.sleep(0.2)

        self.power.value = True

        # self.fs1.value = True

    def _shutdown_sequence(self):
        self.forward.value = False
        self.reverse.value = False
        self.fs1.value = False
        self.seat.value = False
        self.power.value = False

    def startup(self):
        print("Starting up motor...")
        self._startup_sequence()
        time.sleep(1)
        self.forward.value = True
        time.sleep(0.2)
        self.fs1.value = True
        print("Resetting steering angle...")
        self.steer_dac.value = 0
        self._update_current_steering_angle()
        self._steer_adc_set_point = _linear_angle_to_voltage(self._steering_angle_set_point)

    def shutdown(self):
        print("Shutting down motor...")
        self._shutdown_sequence()
        print("Shutting down steering actuator...")
        self.speed_dac.value = 0
        self.steer_dac.value = 0
        self.center()
        print("Shutdown complete!")

    def start_control(self):
        if self._gps.is_reading is False:
            print("GPS not reading!")
            return
        print("Starting hardware control...")
        self.startup()
        self._stop_threads = False
        self._control_thread = threading.Thread(target=self._control, name="hw_control", daemon=True)
        self._control_thread.start()
        print("Control thread started!")

    def stop_control(self):
        print("Stopping hardware control...")
        self.stop()
        print("Straightening steering...")
        self.set_steering_angle(0)
        self._stop_threads = True

        self.shutdown()

    def _control(self):
        while not self._stop_threads:
            self.correct_steering_angle()
            self.correct_speed()

    # SPEED
    def correct_speed(self):
        self._update_speed()
        if self._stop:
            self.speed_dac.value = 0
            return
        error = self._speed_set_point - self._measured_speed
        self._cumulative_error_speed += error
        p_term = error * self.p_gain_speed
        d_term = (error - self._prev_error_speed) * self.d_gain_speed
        i_term = self._cumulative_error_speed * self.i_gain_speed
        self._prev_error_speed = error
        # TODO: Improve control algorithm
        output = p_term + d_term + i_term
        self._increment_speed_output_voltage(increment=output)

    def set_speed(self, speed: float):
        self._stop = False
        if speed > 0:
            self._speed_set_point = speed
            if not self.driving_forward:
                self.direction_forward()
        elif speed < 0:
            self._speed_set_point = abs(speed)
            if self.driving_forward:
                self.direction_reverse()
        elif speed == 0:
            self.stop()

    def stop(self):
        self._stop = True
        if self.driving_forward:
            self.direction_reverse()
        else:
            self.direction_forward()

    def get_speed(self) -> float:
        if self.driving_forward:
            direction = 1
        else:
            direction = -1
        return self._measured_speed * direction

    def _update_speed(self):
        self._measured_speed = self._gps.speed

    def _increment_speed_output_voltage(self, increment):
        new_voltage = max(self._speed_output_voltage + increment, 0)
        # self._speed_output_voltage = min(new_voltage, self._max_speed_voltage)
        self._speed_output_voltage = sorted([0, new_voltage, self._max_speed_voltage])[1]
        self.speed_dac.value = int(round(self._speed_output_voltage / 5 * 65535))

    def increase_max_speed_voltage(self, new_max: float):
        self._max_speed_voltage = min(new_max, 5.)

    def direction_forward(self):
        self.reverse.value = False
        self.fs1.value = False
        time.sleep(0.1)
        self.forward.value = True
        self.fs1.value = True
        self.driving_forward = True

    def direction_reverse(self):
        self.forward.value = False
        self.fs1.value = False
        time.sleep(0.1)
        self.reverse.value = True
        self.fs1.value = True
        self.driving_forward = False

    # STEERING
    def left(self):
        self.right_pin.value = False
        self.left_pin.value = True

    def right(self):
        self.left_pin.value = False
        self.right_pin.value = True

    def center(self):
        self.left_pin.value = False
        self.right_pin.value = False

    def correct_steering_angle(self):
        self._update_current_steering_angle()
        error = self._steer_adc_set_point - self._steer_adc_average
        # error = self._steer_adc_set_point - self._steer_adc_average
        self._cumulative_error_steer += error
        self._prev_error_steer = error
        p_term = self.p_gain_steer * error
        d_term = self.d_gain_steer * (error - self._prev_error_steer)
        i_term = self.i_gain_steer * self._cumulative_error_steer
        output = p_term + d_term + i_term
        # if not self._steering_changed:
        #     return
        self._increment_steering_output_voltage(new_voltage=abs(output))
        if error > 0.002:
            self.left()
        elif error < -0.002:
            self.right()
        else:
            self._cumulative_error_steer = 0.
            self._steering_changed = False
            self.center()

    def _update_current_steering_angle(self) -> None:
        self._steer_adc_measurements.append(self.steer_adc.voltage)
        self._steer_adc_average = sum(self._steer_adc_measurements) / len(self._steer_adc_measurements)
        self._current_steering_angle = _linear_voltage_to_angle(self._steer_adc_average)

    def _increment_steering_output_voltage(self, new_voltage: float):
        # v = max(new_voltage, self._min_steer_voltage)
        self._steer_output_voltage = sorted([self._min_steer_voltage, new_voltage, self._max_steer_voltage])[1]
        self.steer_dac.value = int(round(self._steer_output_voltage / 5 * 65535))

    def get_steering_angle(self) -> float:
        return self._current_steering_angle

    def set_steering_angle(self, angle: float) -> None:
        # if angle > self.left_max:
        #     self._steering_angle_set_point = self.left_max
        # elif angle < self.right_max:
        #     self._steering_angle_set_point = self.right_max
        # else:
        #     self._steering_angle_set_point = angle
        self._steering_changed = True
        # Returns middle value
        self._steering_angle_set_point = sorted([self.left_max, angle, self.right_max])[1]
        self._steer_adc_set_point = _linear_angle_to_voltage(self._steering_angle_set_point)

    # Data logging
    ############################################################

    def get_current_data(self):
        data = state(speed_set_point=self._speed_set_point, current_speed=self._measured_speed,
                     steering_angle_set_point=self._steering_angle_set_point,
                     current_steering_angle=self._current_steering_angle, steer_adc_set_point=self._steer_adc_set_point,
                     steer_adc_avg=self._steer_adc_average, steer_dac_value=self._steer_output_voltage,
                     speed_dac_value=self._speed_output_voltage)
        return data

