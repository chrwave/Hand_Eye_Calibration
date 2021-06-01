import time
import sys

from . import config_gripper
from .communication_gripper import GripperSerial


class Gripper:
    def __init__(self, usb_port=None):
        # Checking python version
        self.python_2 = (sys.version_info.major == 2)

        # Getting gripper connection class
        self.gripper_serial = GripperSerial(usb_port)

        # Activating gripper
        print('    Activating gripper...')
        success = self.activate()
        if success:
            print('        Gripper connection succesfully established.')
        else:
            print('        Activation timeout. No connection with gripper.')

    def activate(self):
        return self.gripper_serial.activate()

    def read(self):
        response = self.gripper_serial.read()
        split = response.split(';')
        register_07D0 = split[0]
        register_07D1 = split[1]
        register_07D2 = split[2]
        return register_07D0, register_07D1, register_07D2

    def get_position(self):
        _, _, register_07D2 = self.read()
        return int(register_07D2[0:2], 16)

    def get_status(self):
        register_07D0, _, _ = self.read()
        status = bin(int(register_07D0[0:2], 16))[2:].zfill(8)
        gOBJ = int(status[0:2], 2)
        gSTA = int(status[2:4], 2)
        gGTO = int(status[4], 2)
        gACT = int(status[7], 2)
        return gOBJ, gSTA, gGTO, gACT

    def set(self, pos, speed=255, force=255, wait=False):
        if (0 <= pos <= 255) & (0 <= speed <= 255) & (0 <= force <= 255):
            self.gripper_serial.set(('{:d};{:d};{:d}'.format(int(pos),
                                                             int(speed),
                                                             int(force))))
        else:
            if not 0 <= pos <= 255:
                print('Gripper position not allowed.')
            if not 0 <= speed <= 255:
                print('Gripper speed not allowed.')
            if not 0 <= force <= 255:
                print('Gripper force not allowed.')

        if wait:
            self.wait()

    def wait(self):
        while True:
            gOBJ, _, _, _ = self.get_status()
            if gOBJ != 0:
                break

    def close(self, speed=255, force=255, wait=False):
        self.set(pos=255, speed=speed, force=force)
        if wait:
            self.wait()

    def open(self, speed=255, force=255, wait=False):
        self.set(pos=0, speed=speed, force=force)
        if wait:
            self.wait()

    def shutdown(self):
        self.gripper_serial.shutdown()
        time.sleep(0.5)
