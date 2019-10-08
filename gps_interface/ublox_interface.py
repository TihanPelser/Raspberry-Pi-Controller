import serial
import queue
import threading


class UBX:
    def __init__(self, port, baud):
        self.device = serial.Serial(port=port, baudrate=baud)
        self.last_message = []
        self.queue = queue.Queue()

        # Position data
        self.two_dim_speed = 0.
        self.x_pos = 0.
        self.y_pos = 0.
        self.heading = 0.

    def read_messages(self):
        # Uncomment for file writing
        # f = open("sample_parsed_2.txt", "w+")

        self.device.reset_input_buffer()
        try:
            while True:
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
                        if msg_id == b'\x01':
                            name = "POSECEF"
                        elif msg_id == b'\x02':
                            name = "POSLLH"
                        elif msg_id == b'\x03':
                            name = "STATUS"
                        elif msg_id == b'\x12':
                            name = "VELNED"
                        elif msg_id == b'\x30':
                            name = "SVINFO"
                        elif msg_id == b'\x35':
                            name = "SAT"
                        else:
                            name = "UNKNOWN"
                        # self.last_message = [name, payload, check]
                        msg = {
                            "name": name,
                            "payload": payload,
                            "check": check
                        }
                        converted_msg = self.parse_message(msg)
                        self.last_message = converted_msg
                        #print(self.last_message)
                        self.queue.put(converted_msg)
                        ##########################
                        # Writing to files
                        ##########################
                        # f.write(str(self.last_message) + '\n')
                        # f.write(msg_class)
                        # f.write(msg_id)
                        # f.write(length_byte)
                        # f.write(payload)
                        # f.write(check)
                        # f.write(b'\n')

        except KeyboardInterrupt:
            print("Done reading")
            # Uncomment if writing to a file
            # f.close()

    def parse_message(self, message: dict):
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
