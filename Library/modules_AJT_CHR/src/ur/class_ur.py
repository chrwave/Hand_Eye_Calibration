import time
from math import pi
import numpy as np
import socket
import sys

from . import config_ur
from .communication_ur import communication_thread


class UR:
    def __init__(self, ip=None, port=None):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)

        #
        self.rotation_matrice = np.array([[0, 0, 0],
                                          [0, 0, 0],
                                          [0, 0, 0]])

        #
        self.origen_task = 0

        #
        self.transform_init(config_ur.TRANSFORM['p0i'],
                            config_ur.TRANSFORM['pxi'],
                            config_ur.TRANSFORM['pyi'])

        # The default position of the gripper
        self.home_position = config_ur.HOME['position']
        self.home_angle = config_ur.HOME['angle']

        # Dictionary containing all the ur data which have been reading
        self.ur_data = {}

        # Connecting socket directly to robot
        self.socket = socket.socket(socket.AF_INET,
                                    socket.SOCK_STREAM)
        
        # If no ip is provided, then use default
        if ip is None:
            self.ip = config_ur.IP
        else:
            self.ip = ip

        # If no port is provided, then use default
        if port is None:
            self.port = config_ur.PORT
        else:
            self.port = port

        # Connect to the UR arm
        self.socket.connect((self.ip, self.port))
        # Starting communication script
        self.communication_thread = communication_thread(self.ip, self.port)

    def transform_init(self, p0i, pxi, pyi):
        p0 = np.array(p0i)
        px = np.array(pxi)
        py = np.array(pyi)
        p0 = p0/1000.
        px = px/1000.
        py = py/1000.
        vx = px-p0
        vy = py-p0
        vx = vx/np.linalg.norm(vx)
        vy = vy/np.linalg.norm(vy)
        vz = np.cross(vx, vy)
        self.rotation_matrice = np.array([vx, vy, vz])
        self.rotation_matrice = np.transpose(self.rotation_matrice)
        self.origen_task = p0
        
        bTt = np.identity(4)
        bTt[0:3,0:3] = self.rotation_matrice
        bTt[0:3,3] = self.origen_task
        return bTt

    def transform(self, x, y, z):
        b = np.array([x, y, z])
        t = self.rotation_matrice.dot(np.transpose(b)) + self.origen_task
        return t

    def inverse_transform(self, x, y, z):
        b = np.array([x, y, z])
        it = np.transpose(self.rotation_matrice).dot(np.transpose((b - self.origen_task)))
        return it

    def set_tcp(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        self.socket.send(('set_tcp(p[' + str(x) + ',' + str(y) +
                                  ',' + str(z) + ',' + str(rx) + ',' +
                                  str(ry) + ',' + str(rz) + '])\n').encode())
        time.sleep(0.1)

    def get_position(self, world=True):
        self.read()
        # The older version have the position values in a different spot
        if (self.communication_thread.message_size >=
                config_ur.MESSAGE_SIZE_TO_VERSION['3.0']):
            x = self.ur_data['x_actual']
            y = self.ur_data['y_actual']
            z = self.ur_data['z_actual']
            #These have been inserted by Andreas & Christian, to get the rotation vector
            rx = self.ur_data['rx_actual'] 
            ry = self.ur_data['ry_actual']
            rz = self.ur_data['rz_actual']
        else:
            x = self.ur_data['x']
            y = self.ur_data['y']
            z = self.ur_data['z']
            #These have been inserted by Andreas & Christian, to get the rotation vector
            rx = self.ur_data['rx']
            ry = self.ur_data['ry']
            rz = self.ur_data['rz']
        if world:
            return self.inverse_transform(x, y, z, rx, ry, rz)
        else:
            return (x, y, z, rx, ry, rz)
            
    def get_joints_rad(self):
        #Get Joint space in radians, Andreas & Christian
        self.read()
        q_b = self.ur_data['q_b']
        q_s = self.ur_data['q_s']
        q_e = self.ur_data['q_e']
        q_w1 = self.ur_data['q_w1']
        q_w2 = self.ur_data['q_w2']
        q_w3 = self.ur_data['q_w2']
        
        qb = np.rad2deg(q_b)
        qs = np.rad2deg(q_s)
        qe = np.rad2deg(q_e)
        qw1 = np.rad2deg(q_w1)
        qw2 = np.rad2deg(q_w2)
        qw3 = np.rad2deg(q_w3)
        return q_b, q_s, q_e, q_w1, q_w2, q_w3, qb, qs, qe, qw1, qw2, qw3     
    
    def get_joints_radv2(self):
        #Get Joint space in radians, Andreas & Christian
        self.read()
        q_b = self.ur_data['b']
        q_s = self.ur_data['s']
        q_e = self.ur_data['e']
        q_w1 = self.ur_data['w1']
        q_w2 = self.ur_data['w2']
        q_w3 = self.ur_data['w3']
        
        qb = np.rad2deg(q_b)
        qs = np.rad2deg(q_s)
        qe = np.rad2deg(q_e)
        qw1 = np.rad2deg(q_w1)
        qw2 = np.rad2deg(q_w2)
        qw3 = np.rad2deg(q_w3)
        return q_b, q_s, q_e, q_w1, q_w2, q_w3, qb, qs, qe, qw1, qw2, qw3  

    def move(self, x, y, z, rx=pi, ry=0, rz=0, acc=1, speed=0.1,
             transform=True, wait=False):
        if transform:
            x, y, z = self.transform(x, y, z)
        self.socket.send(('movel(p[' + str(x) + ',' + str(y) +
                                  ',' + str(z) + ',' + str(rx) + ',' +
                                  str(ry) + ',' + str(rz) + '],' + str(acc) +
                                  ',' + str(speed) + ')\n').encode())
        if wait:
            self.wait()

    def move_relative(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1,
                      wait=False):
        x_current, y_current, z_current = self.get_position(world=False)
        # The older version have the position values in a different spot
        if (self.communication_thread.message_size >=
                config_ur.MESSAGE_SIZE_TO_VERSION['3.0']):
            rx_current = self.ur_data['rx_actual']
            ry_current = self.ur_data['ry_actual']
            rz_current = self.ur_data['rz_actual']
        else:
            rx_current = self.ur_data['rx']
            ry_current = self.ur_data['ry']
            rz_current = self.ur_data['rz']

        self.move(x_current + x, y_current + y, z_current + z,
                  rx_current + rx, ry_current + ry, rz_current + rz,
                  acc, speed, transform=False, wait=wait)

    def move_tool(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=1, speed=0.1,
                  wait=False):
        send_string = 'movel(pose_trans(get_forward_kin(),' +\
                      'p[' + str(x) + ',' + str(y) + ',' + str(z) +\
                      ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + ']' +\
                      '),' + str(acc) + ',' + str(speed) + ')\n'
        self.socket.send(send_string.encode())
        if wait:
            self.wait()

    def speed(self, x=0, y=0, z=0, rx=0, ry=0, rz=0, acc=0.5, time=1,
              wait=False):
        self.socket.send(('speedl([' + str(x) + ',' + str(y) + ',' +
                                  str(z) + ',' + str(rx) + ',' + str(ry) +
                                  ',' + str(rz) + '],' + str(acc) + ',' +
                                  str(time) + ')\n').encode())
        if wait:
            self.wait()

    def set_home(self, pos, angle):
        self.home_position = pos
        self.home_angle = angle

    def home(self):
        self.move(self.home_position[0], self.home_position[1], self.home_position[2],
                  self.home_angle[0], self.home_angle[1], self.home_angle[2])

    def read(self):
        data = self.communication_thread.data
        # Removing last entry: empty due to fenceposting in sending process
        data_split = data.split(';')[:-1]
        for item in data_split:
            data_point, data_value = item.split(':')
            self.ur_data[data_point] = float(data_value)

    def wait(self):
        # Hold-off to let the robot start movement before using data
        time.sleep(0.1)
        controller_time = 0

        if (self.communication_thread.message_size <
                config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
            velocity_series = [[1] * 20] * 6

        while True:
            self.read()

            # Test if new data have arrived
            if controller_time != self.ur_data['time']:
                controller_time = self.ur_data['time']
            else:
                # If not sleep the rate that is equal to
                # when the next new data should arive
                if int(str(self.port)[-1]) >= 3:
                    time.sleep(1/1000)
                else:
                    time.sleep(1/20)
                continue

            # If newer software then read the status directly
            if (self.communication_thread.message_size >=
                    config_ur.MESSAGE_SIZE_TO_VERSION['3.2']):
                if self.ur_data['status'] == 1:
                    break
            # Otherwise check if the arm is still moving
            else:
                current_velocities = [self.ur_data['v_b'],
                                      self.ur_data['v_s'],
                                      self.ur_data['v_e'],
                                      self.ur_data['v_w1'],
                                      self.ur_data['v_w2'],
                                      self.ur_data['v_w3']]
                total_mean_velocity = 0
                for i, velocity in enumerate(velocity_series):
                    velocity_series[i], velocity_mean = self.moving_average(velocity, current_velocities[i])
                    total_mean_velocity += abs(velocity_mean)
                
                if total_mean_velocity < config_ur.VELOCITY_MEAN_THRESHOLD * 6:
                    #time.sleep(0.05)  # Give time for velocities to reach 0
                    break

    def moving_average(self, signal, new_point):
        if new_point > 1e5:
            new_point = 0
        new_signal = signal[1:] + [new_point]
        average = sum(new_signal)/len(new_signal)

        return new_signal, average

    def send_line(self, _str):
        if type(_str) is str:
            self.socket.send(_str.encode())
        elif type(_str) is bytes:
            self.socket.send(_str)
        else:
            print("Input to send_line must be of type str or type bytes")

    def shutdown(self):
        self.communication_thread.shutdown()
