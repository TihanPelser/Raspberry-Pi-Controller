import queue

class MockUBX:
    def __init__(self, data_file):
        self.device = 'device'
        self.last_message = []
        f = open(data_file, 'rb')
        self.raw_data = f.read()
        self.data = [self.raw_data[i:i+1] for i in range(0, len(self.raw_data), 2)]
        self.queue = queue.Queue()

    def read_messages(self):
        ind = 0
        try:

            while True:
                b = self.data[ind]
                print(type(b))
                if b == b'\xB5':
                    ind += 1
                    b = self.data[ind]
                    print("First match")
                    if b == b'\x62':
                        ind += 1
                        msg_class = self.data[ind]
                        ind += 1
                        msg_id = self.data[ind]
                        ind += 1
                        length_byte = self.data[ind:ind+2]
                        ind += 2
                        length = int.from_bytes(length_byte, byteorder='little', signed=False)
                        payload = self.data[ind:length+1]
                        ind += length + 1
                        check = self.data[ind:ind+2]
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
                        self.last_message = [name, payload, check]
                        print(self.last_message)
                        self.queue.put(self.last_message)

                        if ind == len(self.data) -1:
                            ind = 0

                    else:
                        ind += 1
                else:
                    ind += 1

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

        except IndexError:
            print("resetting index")
            ind = 0

    def test(self):
        print(self.data)