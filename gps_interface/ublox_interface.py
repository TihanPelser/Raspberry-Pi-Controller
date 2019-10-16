import serial
import queue
import threading
from collections import deque
from typing import Optional
from collections import namedtuple

position_data = namedtuple("pos_data", "lat, long, heading, speed")


class UBX:
    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 9600):
        self.device = serial.Serial(port=port, baudrate=baud)
        self.last_message = None
        self.queue = queue.Queue()
        self.is_reading = False

        # Threading
        self._read_thread = None
        self._stop_threads = None

        # Position data
        self.speed = 0.
        self.heading = 0.
        self.lat = 0.
        self.long = 0

        # Moving average data
        self._speed_list = deque(maxlen=3)
        self._heading_list = deque(maxlen=3)
        self._lat_list = deque(maxlen=3)
        self._long_list = deque(maxlen=3)

    def start_reading(self):
        self._stop_threads = False
        self._read_thread = threading.Thread(target=self.read_messages, name="gps", daemon=True)
        self._read_thread.start()
        self.is_reading = True

    def stop_reading(self):
        self._stop_threads = True
        self.is_reading = False

    def _update_average_speed(self, speed):
        # Convert to m/s
        self._speed_list.append(speed/100)
        self.speed = sum(self._speed_list) / len(self._speed_list)

    def _update_heading(self, heading):
        self._heading_list.append(heading)
        self.heading = sum(self._heading_list) / len(self._heading_list)

    def _update_lat(self, lat):
        self._lat_list.append(lat)
        self.lat = sum(self._lat_list) / len(self._lat_list)

    def _update_lon(self, lon):
        self._long_list.append(lon)
        self.long = sum(self._long_list) / len(self._long_list)

    def _update_all(self):
        # Uses last received message to update required properties
        if self.last_message["type"] == "VELNED":
            self._update_average_speed(self.last_message["speed2D"])
            self._update_heading(self.last_message["heading"])
        if self.last_message["type"] == "POSLLH":
            self._update_lat(self.last_message["LAT"])
            self._update_lon(self.last_message["LON"])

    def read_messages(self):
        # Uncomment for file writing
        # f = open("sample_parsed_2.txt", "w+")

        self.device.reset_input_buffer()
        try:
            while not self._stop_threads:
                b = self.device.read(1)
                # print(b)
                if b == b'\xB5':
                    b = self.device.read(1)
                    if b == b'\x62':
                        msg_class = self.device.read(1)
                        msg_id = self.device.read(1)
                        length_byte = self.device.read(2)
                        length = int.from_bytes(length_byte, byteorder='little', signed=False)
                        payload = self.device.read(length)
                        check = self.device.read(2)
                        # if msg_id == b'\x01':
                        #     name = "POSECEF"
                        if msg_id == b'\x02':
                            name = "POSLLH"
                        # elif msg_id == b'\x03':
                        #     name = "STATUS"
                        elif msg_id == b'\x12':
                            name = "VELNED"
                        # elif msg_id == b'\x30':
                        #     name = "SVINFO"
                        # elif msg_id == b'\x35':
                        #     name = "SAT"
                        else:
                            # Skip message if not important
                            continue

                        msg = {
                            "name": name,
                            "payload": payload,
                            "check": check
                        }
                        converted_msg = self.parse_message(msg)
                        self.last_message = converted_msg
                        self._update_all()

        except KeyboardInterrupt:
            print("Done reading")
            # Uncomment if writing to a file
            # f.close()

    def parse_message(self, message: dict) -> dict:
        name = message["name"]
        payload = message["payload"]
        if name == "VELNED":
            converted = {
                "type": "VELNED",
                "iTOW": int.from_bytes(payload[0:4], signed=False, byteorder='little'),
                "velN": int.from_bytes(payload[4:8], signed=True, byteorder='little'),
                "velE": int.from_bytes(payload[8:12], signed=True, byteorder='little'),
                "velD": int.from_bytes(payload[12:16], signed=True, byteorder='little'),
                "speed3D": int.from_bytes(payload[16:20], signed=False, byteorder='little'),
                "speed2D": int.from_bytes(payload[20:24], signed=False, byteorder='little'),
                "heading": int.from_bytes(payload[24:28], signed=True, byteorder='little') * 10**-5,
                "speed_accuracy": int.from_bytes(payload[28:32], signed=False, byteorder='little'),
                "heading_accuracy": int.from_bytes(payload[32:36], signed=False, byteorder='little') * 10**-5,
            }

            return converted

        elif name == "POSECEF":

            converted = {
                "type": "POSECEF",
                "iTOW": int.from_bytes(payload[0:4], signed=False, byteorder='little'),
                "ECEFX": int.from_bytes(payload[4:8], signed=True, byteorder='little'),
                "ECEFY": int.from_bytes(payload[8:12], signed=True, byteorder='little'),
                "ECEFZ": int.from_bytes(payload[12:16], signed=True, byteorder='little'),
                "position_accuracy": int.from_bytes(payload[16:20], signed=False, byteorder='little'),
            }

            return converted

        elif name == "POSLLH":

            converted = {
                "type": "POSLLH",
                "iTOW": int.from_bytes(payload[0:4], signed=False, byteorder='little'),
                "LON": int.from_bytes(payload[4:8], signed=True, byteorder='little') * 10**-7,
                "LAT": int.from_bytes(payload[8:12], signed=True, byteorder='little') * 10**-7,
                "HEIGHT": int.from_bytes(payload[12:16], signed=True, byteorder='little'),
                "hMSL": int.from_bytes(payload[16:20], signed=True, byteorder='little'),
                "hACC": int.from_bytes(payload[20:24], signed=False, byteorder='little'),
                "vACC": int.from_bytes(payload[24:28], signed=False, byteorder='little'),
            }
            return converted
        else:
            converted = {
                "type": "NONE",
            }
            return converted

    def get_current_data(self):
        return position_data(lat=self.lat, long=self.long, heading=self.heading, speed=self.speed)
