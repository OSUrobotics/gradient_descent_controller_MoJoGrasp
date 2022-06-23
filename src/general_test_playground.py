#!/usr/bin/python3

import numpy as np

goal_position = np.array([1,2,3])
current_position = np.array([4,10,3])

delta = goal_position - current_position

print(delta * 0.01)

print(np.linalg.norm(delta))