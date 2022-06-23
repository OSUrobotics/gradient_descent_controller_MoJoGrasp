from multiprocessing.dummy import current_process
import pybullet as p

import numpy as np


from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper


class HandController():

    MAX_MOVE = 0.01

    def __init__(self, gripper: UpdatedTwoFingerGripper, cube: UpdatedObjectBase, ) -> None:
        self.hand = gripper
        self.cube = cube

        self.end_effector_links = [self.hand.fingers[finger]["index_values"][-1] for finger in self.hand.fingers.keys()]

        self.goal_position = None
        self.current_cube_pose = None


    def get_current_cube_position(self):
        # this is the location of the origin defined by the urdf in the world frame
        self.current_cube_pose = self.cube.get_curr_pose() 

    def get_next_cube_position(self):
        
        current_position = np.array(self.current_cube_pose[0])

        goal_position = np.array(self.goal_position)

        
        distance_to_goal = goal_position - current_position
        euclidean_dis = np.linalg.norm(distance_to_goal)
        if euclidean_dis > self.MAX_MOVE:
            next_position = current_position + (distance_to_goal * (self.MAX_MOVE / euclidean_dis))
        else:
            next_position = current_position + distance_to_goal


        return next_position



