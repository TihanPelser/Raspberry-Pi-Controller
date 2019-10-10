from hardware.speed_controller import SpeedController
from hardware.steering_controller import SteeringController
import busio
from adafruit_blinka.board import raspi_40pin as board
from gps_interface.ublox_interface import UBX
import threading
import time


class HardwareController:
    def __init__(self, i2c: busio.I2C, gps: UBX):
        self.steer = SteeringController(i2c=i2c)
        self.speed = SpeedController(i2c=i2c, gps=gps)

        # Threading
        self._stop_threads = False
        self._control_thread = None

    def start_control(self):
        print("Starting hardware control...")
        print("Setting up speed controller...")
        self.speed.startup()
        print("Setting up steering controller...")
        self.steer.startup()
        self._stop_threads = False
        self._control_thread = threading.Thread(target=self._control, name="hw_control", daemon=True)
        print("Control thread started!")

    def stop_control(self):
        print("Stopping hardware control...")
        self._stop_threads = True
        self.speed.shutdown()
        self.steer.shutdown()

    def _control(self):
        while not self._stop_threads:
            self.steer.correct_steering_angle()
            self.speed.correct_speed()

    def set_speed(self, speed):
        self.speed.set_speed(speed=speed)

    def set_steering_angle(self, angle):
        self.steer.set_steering_angle(angle=angle)

