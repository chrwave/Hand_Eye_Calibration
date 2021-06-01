from math import pi

VELOCITY_MEAN_THRESHOLD = 0.001

VERSION_3_2_MESSAGE_SIZE = 1060

MESSAGE_SIZE_TO_VERSION = {'3.0': 1044, '3.2': 1060}


IP = '192.38.66.254'
PORT = 30003

DATA_MAP = {'message_size': 0, 'time': 1,
            'q_b': 2,'q_s': 3, 'q_e': 4, 'q_w1': 5, 'q_w2': 6, 'q_w3': 7,
            'b': 32, 's': 33, 'e': 34, 'w1': 35, 'w2': 36, 'w3': 37,
            'v_b': 38, 'v_s': 39, 'v_e': 40, 'v_w1': 41, 'v_w2': 42, 'v_w3': 43,
            'x_actual': 56, 'y_actual': 57, 'z_actual': 58, 'rx_actual': 59, 'ry_actual': 60, 'rz_actual': 61,
            'v_x': 62, 'v_y': 63, 'v_z': 64, 'v_rx': 65, 'v_ry': 66, 'v_rz': 67,
            'f_x': 68, 'f_y': 69, 'f_z': 70, 'f_rx': 71, 'f_ry': 72, 'f_rz': 73,
            'x': 74, 'y': 75, 'z': 76, 'rx': 77, 'ry': 78, 'rz': 79,
            'robot_mode': 95, 'status': 132}

# UR5 transform
TRANSFORM = {'p0i':[-397.30, 383.02, 66.19],
             'pxi':[-400.92, 157.97, 65.54], 
             'pyi':[-222.82, 380.36, 63.89]}

# UR5 transform
TRANSFORM0 = {'p0i':[-410.28, 369.50, 237.60],
             'pxi':[-413.24, 194.39, 237.63], 
             'pyi':[-286.15, 368.49, 237.62]}

'''
# UR3 transform
TRANSFORM = {'p0i':[-119.38, 449.1, 10.66],
             'pxi':[-124.4, 232.3, 8.09], 
             'pyi':[31.39, 446.77, 10.76]}
'''

HOME = {'position':[0.15, 0.12, 0.20],
        'angle':[pi, 0, 0]}
