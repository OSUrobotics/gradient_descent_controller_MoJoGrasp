#!/usr/bin/python3

import numpy as np
import pybullet as p
from numpy import pi
import logging, colorlog
from colorlog import ColoredFormatter
import pathlib

# path_name = sorted(pathlib.Path('.').glob('**/*.urdf'))

# print(str(path_name))

a = np.array([[ 0.7024227,   0.71176274,  0.,         -0.090539  ],
            [-0.71176274, 0.7024227,  -0.,          0.01301506],
            [ 0.,          0.,          1.,         -0.05      ],
            [ 0.,          0.,          0.,          1.        ]])

b = np.array([0.01939, 0.1025,  0.05,    1.])

print(np.matmul(a, b))
# handler = colorlog.StreamHandler()
# handler.setFormatter(colorlog.ColoredFormatter("%(log_color)s%(levelname)s - %(name)s - %(message)s%(reset)s"))
# logger = colorlog.getLogger("simulator")
# logger.addHandler(handler)
# a_list = []

# logging.basicConfig(format='%(levelname)s - %(message)s')

# a_list[0] = 6
# a_list[1] = 7
# a_list[2] = 8
# a_list[3] = 9

# print(a_list)
# goal_position = np.array([1,2,3])
# current_position = np.array([4,10,3])

# delta = goal_position - current_position

# print(delta * 0.01)

# print(np.linalg.norm(delta))


# test_array = np.identity(4)
# test_array[0:3,3] =  [1,2,3]

# a = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, pi/2, pi/4]))
# print(a)
# test_array[0:3,0:3] = np.reshape(a, (3,3))


# print(test_array)


# a = np.array([[1, 0, 0],
#               [0, 1, 0],
#               [0, 0, 1]])

# print(p.getQuaternionFromMatrix(a))



# logger.warning('test')