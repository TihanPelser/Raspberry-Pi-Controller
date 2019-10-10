from adafruit_blinka.board import raspi_40pin as board
import adafruit_mcp4725 as DAC
import digitalio as DIO
import threading
from gps_interface.ublox_interface import UBX
import time

# Driving Speed DAC Address = 96


class SpeedController:

    def __init__(self, i2c, gps: UBX):

        # Drive Controller (Millipak) Digital Pins
        self.forward = DIO.DigitalInOut(board.D16)      # D3
        self.reverse = DIO.DigitalInOut(board.D26)      # D4
        self.fs1 = DIO.DigitalInOut(board.D19)          # D5
        self.seat = DIO.DigitalInOut(board.D13)         # D6
        self.power = DIO.DigitalInOut(board.D6)         # D7

        # Set as outputs
        self.forward.direction = DIO.Direction.OUTPUT
        self.reverse.direction = DIO.Direction.OUTPUT
        self.fs1.direction = DIO.Direction.OUTPUT
        self.seat.direction = DIO.Direction.OUTPUT
        self.power.direction = DIO.Direction.OUTPUT

        # I2C and ADC Setup

        self._i2c_interface = i2c

        # Setup Drive Speed DAC
        self.dac_output = DAC.MCP4725(self._i2c_interface, address=96)

        # Speed
        self._measured_speed = 0.
        self._speed_set_point = 0.

        # Speed PID Controller
        self._prev_error = 0.
        self.p_gain = 0.5
        self.d_gain = 1
        self._output_voltage = 0.
        self._max_output_voltage = 1.

        # GPS
        self._gps = gps

        # Threading
        self._speed_thread = None
        self._stop_threads = False

        # Set DAC Output to 0V
        self.dac_output.value = 0

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

    def start_speed_control_thread(self):
        self._startup_sequence()
        time.sleep(1)
        self.forward.value = True
        time.sleep(0.2)
        self.fs1.value = True
        self._stop_threads = False
        self._speed_thread = threading.Thread(target=self._correct_speed_threaded, name="speed", daemon=True)
        self._speed_thread.start()
        print("Driving started")

    def startup(self):
        self._startup_sequence()
        time.sleep(1)
        self.forward.value = True
        time.sleep(0.2)
        self.fs1.value = True

    def shutdown(self):
        self._shutdown_sequence()

    def stop_speed_control_thread(self):
        self._shutdown_sequence()
        self._stop_threads = True
        self._output_voltage = 0
        self._speed_set_point = 0
        self.dac_output.value = 0
        print("Driving stopped")

    def _correct_speed_threaded(self):
        while not self._stop_threads:
            self._update_speed()
            error = self._speed_set_point - self._measured_speed
            p_term = error * self.p_gain
            d_term = error * self.d_gain
            self._prev_error = error
            # TODO: Improve control algorithm
            output = p_term + d_term
            self._increment_output_voltage(increment=output)

    def correct_speed(self):
        self._update_speed()
        error = self._speed_set_point - self._measured_speed
        p_term = error * self.p_gain
        d_term = error * self.d_gain
        self._prev_error = error
        # TODO: Improve control algorithm
        output = p_term + d_term
        self._increment_output_voltage(increment=output)

    def set_speed(self, speed: float):
        self._speed_set_point = speed

    def get_speed(self) -> float:
        return self._measured_speed

    def _update_speed(self):
        self._measured_speed = self._gps.two_dim_speed

    def _increment_output_voltage(self, increment):
        new_voltage = max(self._output_voltage + increment, 0)
        self._output_voltage = min(new_voltage, self._max_output_voltage)
        self.dac_output.value = int(round(self._output_voltage / 5 * 65535))

    def increase_max_output_voltage(self, new_max: float):
        self._max_output_voltage = min(new_max, 5.)

