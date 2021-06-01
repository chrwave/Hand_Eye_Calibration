import socket
import struct
import threading
import sys
import time

from . import config_ur


class communication_thread():
    def __init__(self, ip, port):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        # Creating the socket
        self.socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        time_start = time.time()
        self.socket.connect((ip, port))

        # The thread keeps going as long as this variable is true
        self.running = True

        # The data which the thread optains from the ur
        self.data = 0

        # Read the message size
        self.message_size = float('inf')
        self.message_size = self.get_message_size()

        # The thread which keeps receiving data
        self.receive_thread = threading.Thread(target=self.receive)

        # Starting the Thread
        print('    Starting communication thread...')
        self.receive_thread.start()

    def receive(self):
        while self.running:
            data = (self.socket.recv(2048))
            data = self.transform_data(data)
            # If no error occurred then update data
            if not data == -4444:
                self.data = data

    def shutdown(self):
        self.running = False
        self.receive_thread.join()
        self.socket.close()

    def get_message_size(self):
        data = (self.socket.recv(2048))
        message_size = int(self.transform_data_point(data, 'message_size'))
        self.data = self.transform_data(data)
        
        return message_size

    def transform_data_point(self, data, data_name):
        # There is an exception for message size
        if data_name == 'message_size':
            byte_position = config_ur.DATA_MAP[data_name]
            data_type = '!i'
            # Message size is only 4 long
            data_size = 4 
        else:
            byte_position = (config_ur.DATA_MAP[data_name] - 1) * 8 + 4
            data_type = '!d'
            # Message size is an integer
            data_size = 8
        
        # Check that there is data to be read in the position
        if self.message_size < byte_position + data_size:
            return -4444


        data = data[byte_position:byte_position + data_size]

        # Convert the data from \x hex notation to plain hex
        if self.python_2:
            data = data.encode('hex')
        else:
            data = data.hex()

        if len(data) == data_size * 2:
            if self.python_2:
                data = struct.unpack(data_type, data.decode('hex'))[0]
            else:
                data = struct.unpack(data_type, bytes.fromhex(data))[0]
            return data
        else:
            return -4444

    def transform_data(self, data):
        data_string = ''
        for data_type in config_ur.DATA_MAP:
            data_point = self.transform_data_point(data, data_type)
            # Check if an error occurred
            if data_point == -4444:
                continue
            data_string += data_type + ':' + str(data_point) + ';'
        return data_string
