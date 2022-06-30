#!/usr/bin/python3

import numpy as np
import pybullet as p
from numpy import pi
a_list = []


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


test_array = np.identity(4)
test_array[0:3,3] =  [1,2,3]

a = p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0, pi/2, pi/4]))
print(a)
test_array[0:3,0:3] = np.reshape(a, (3,3))


print(test_array)